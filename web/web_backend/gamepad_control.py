"""Exclusive gamepad lease, validation, forwarding and timeout safety."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Literal, Optional

from fastapi import WebSocket
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)

from opi_console.config import AppConfig, coerce_config
from opi_console.stm32_proxy import Stm32Proxy
from protocol import SafetyState, SetBodyCommand
from web_backend.control_arbiter import (
    ControlMode,
    ControlModeConflict,
)
from web_backend.control_state import ControlState
from web_backend.gamepad_mapping import map_gamepad_state


logger = logging.getLogger("opi_console.gamepad")

_ZERO_COMMAND = SetBodyCommand()
_COMMAND_NAMES = ("surge", "sway", "heave", "roll", "pitch", "yaw")


def command_dict(command: SetBodyCommand) -> dict[str, float]:
    return dict(zip(_COMMAND_NAMES, command.values()))


def command_is_zero(command: SetBodyCommand, tolerance: float = 1e-7) -> bool:
    return all(abs(value) <= tolerance for value in command.values())


class GamepadControlError(RuntimeError):
    def __init__(self, status_code: int, detail: str):
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


class BodyCommandMessage(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    surge: float
    sway: float
    heave: float
    roll: float
    pitch: float
    yaw: float

    @model_validator(mode="after")
    def validate_axes(self):
        for name in _COMMAND_NAMES:
            value = getattr(self, name)
            if not math.isfinite(value) or value < -1.0 or value > 1.0:
                raise ValueError(
                    f"mapped_command.{name} must be finite and within -1..1")
        return self

    def to_command(self) -> SetBodyCommand:
        return SetBodyCommand(**self.model_dump())


class GamepadStateMessage(BaseModel):
    model_config = ConfigDict(extra="forbid")

    type: Literal["gamepad_state"]
    version: Literal[1]
    session_id: str = Field(
        min_length=8, max_length=64, pattern=r"^[A-Za-z0-9_-]+$")
    sequence: int = Field(strict=True, ge=0, le=(1 << 63) - 1)
    client_time_ns: int = Field(strict=True, ge=0)
    control_enabled: bool = Field(strict=True)
    gamepad_connected: bool = Field(strict=True)
    axes: list[float] = Field(min_length=6, max_length=6)
    buttons: list[int] = Field(min_length=4, max_length=32)
    mapped_command: BodyCommandMessage
    device: Optional[dict[str, Any]] = None
    hats: list[list[int]] = Field(default_factory=list, max_length=8)

    @field_validator("axes", mode="before")
    @classmethod
    def validate_raw_axes(cls, value):
        if not isinstance(value, list) or len(value) != 6:
            raise ValueError("axes must contain exactly six values")
        normalized = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError("axes must contain only numbers")
            converted = float(item)
            if not math.isfinite(converted):
                raise ValueError("axes must reject NaN and Infinity")
            if converted < -1.0 or converted > 1.0:
                raise ValueError("axes must be within -1..1")
            normalized.append(converted)
        return normalized

    @field_validator("buttons", mode="before")
    @classmethod
    def validate_buttons(cls, value):
        if not isinstance(value, list):
            raise ValueError("buttons must be an array")
        normalized = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, int):
                raise ValueError("buttons must contain integer 0 or 1")
            if item not in (0, 1):
                raise ValueError("buttons must contain integer 0 or 1")
            normalized.append(item)
        return normalized

    @field_validator("hats", mode="before")
    @classmethod
    def validate_hats(cls, value):
        if value is None:
            return []
        if not isinstance(value, list):
            raise ValueError("hats must be an array")
        for item in value:
            if (
                not isinstance(item, list)
                or len(item) != 2
                or any(
                    isinstance(axis, bool)
                    or not isinstance(axis, int)
                    or axis not in (-1, 0, 1)
                    for axis in item
                )
            ):
                raise ValueError("each hat must contain two values in -1, 0, 1")
        return value


@dataclass(frozen=True)
class LatestGamepadFrame:
    session_id: str
    sequence: int
    client_time_ns: int
    received_mono: float
    received_wall: float
    control_enabled: bool
    gamepad_connected: bool
    axes: tuple[float, ...]
    buttons: tuple[int, ...]
    device: Optional[dict[str, Any]]
    command: SetBodyCommand
    heave_conflict: bool


class GamepadControlService:
    """Owns one WebSocket lease and forwards only the newest validated frame."""

    def __init__(
        self,
        proxy: Stm32Proxy,
        control_state: ControlState,
        config: Optional[AppConfig] = None,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self._proxy = proxy
        self._control = control_state
        self._config = coerce_config(config or proxy.config)
        self._gamepad = self._config.gamepad
        self._clock = clock
        self._wall_clock = wall_clock
        self._socket_lock = asyncio.Lock()
        self._ws: Optional[WebSocket] = None
        self._session_id: Optional[str] = None
        self._last_sequence: Optional[int] = None
        self._latest: Optional[LatestGamepadFrame] = None
        self._pump_task: Optional[asyncio.Task] = None
        self._stopping = False
        self._zeroed_for_timeout = False
        self._resume_requires_neutral = False
        self._last_forwarded_sequence: Optional[int] = None
        self._last_forwarded_at = 0.0
        self._last_stm32_ack = False
        self._last_error: Optional[str] = None
        self._last_disconnect_reason: Optional[str] = None
        self._last_shutdown_result: Optional[dict[str, bool]] = None
        self.rejected_messages = 0
        self.duplicate_messages = 0

    @property
    def task_running(self) -> bool:
        return bool(self._pump_task and not self._pump_task.done())

    @property
    def client_connected(self) -> bool:
        return self._ws is not None

    async def start(self) -> None:
        self._stopping = False
        if self._pump_task is None or self._pump_task.done():
            self._pump_task = asyncio.create_task(
                self._pump_loop(), name="gamepad-command-pump")

    async def stop(self) -> None:
        self._stopping = True
        task = self._pump_task
        self._pump_task = None
        if task and not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await self.safe_shutdown("gamepad_service_stopped")
        async with self._socket_lock:
            ws = self._ws
            self._ws = None
            self._session_id = None
            self._last_sequence = None
            self._latest = None
        if ws is not None:
            try:
                await ws.close(code=1001)
            except Exception:
                pass

    async def connect(self, ws: WebSocket) -> bool:
        await ws.accept()
        async with self._socket_lock:
            if self._ws is not None:
                await ws.send_json({
                    "type": "gamepad_error",
                    "version": 1,
                    "reason": "lease_already_owned",
                })
                await ws.close(code=4409)
                return False
            self._ws = ws
            self._session_id = None
            self._last_sequence = None
            self._latest = None
            self._last_error = None
            self._last_disconnect_reason = None
            self._zeroed_for_timeout = False
            self._resume_requires_neutral = False
        await ws.send_json({
            "type": "gamepad_hello",
            "version": 1,
            "lease_granted": True,
            "server_time_ns": time.monotonic_ns(),
            "mapping_config": self._gamepad.model_dump(),
        })
        logger.info("Gamepad WebSocket lease granted")
        return True

    async def disconnect(self, ws: WebSocket, reason: str) -> None:
        async with self._socket_lock:
            if ws is not self._ws:
                return
            self._ws = None
            self._session_id = None
            self._last_sequence = None
            self._latest = None
            self._last_disconnect_reason = reason
        logger.warning("Gamepad WebSocket disconnected: %s", reason)
        if not self._stopping:
            await self.safe_shutdown(reason)

    async def handle_message(self, ws: WebSocket, raw: str) -> None:
        if ws is not self._ws:
            return
        if len(raw) > 65536:
            self.rejected_messages += 1
            self._last_error = "gamepad_message_too_large"
            await self._send_ack(
                ws, None, False, "message_exceeds_65536_bytes")
            return
        try:
            message = GamepadStateMessage.model_validate_json(raw)
        except ValidationError as exc:
            self.rejected_messages += 1
            self._last_error = "invalid_gamepad_message"
            await self._send_ack(
                ws, None, False, f"validation_error: {exc.errors()[0]['msg']}")
            return
        except (json.JSONDecodeError, ValueError) as exc:
            self.rejected_messages += 1
            self._last_error = "invalid_gamepad_json"
            await self._send_ack(ws, None, False, f"invalid_json: {exc}")
            return

        if self._session_id is None:
            self._session_id = message.session_id
        elif message.session_id != self._session_id:
            self.rejected_messages += 1
            await self._send_ack(
                ws, message.sequence, False, "session_id_changed")
            return

        if (
            self._last_sequence is not None
            and message.sequence <= self._last_sequence
        ):
            self.rejected_messages += 1
            self.duplicate_messages += 1
            await self._send_ack(
                ws, message.sequence, False, "sequence_not_increasing")
            return

        try:
            mapped = map_gamepad_state(
                message.axes, message.buttons, self._gamepad)
        except ValueError as exc:
            self.rejected_messages += 1
            await self._send_ack(
                ws, message.sequence, False, f"mapping_error: {exc}")
            return

        submitted = message.mapped_command.to_command()
        if any(
            not math.isclose(actual, expected, rel_tol=1e-5, abs_tol=1e-5)
            for actual, expected in zip(
                submitted.values(), mapped.command.values())
        ):
            self.rejected_messages += 1
            self._last_error = "mapped_command_mismatch"
            await self._send_ack(
                ws, message.sequence, False, "mapped_command_mismatch",
                mapped.command)
            return

        now_mono = self._clock()
        frame = LatestGamepadFrame(
            session_id=message.session_id,
            sequence=message.sequence,
            client_time_ns=message.client_time_ns,
            received_mono=now_mono,
            received_wall=self._wall_clock(),
            control_enabled=message.control_enabled,
            gamepad_connected=message.gamepad_connected,
            axes=tuple(message.axes),
            buttons=tuple(message.buttons),
            device=message.device,
            command=mapped.command,
            heave_conflict=mapped.heave_conflict,
        )
        self._last_sequence = message.sequence
        self._latest = frame
        if (
            self._resume_requires_neutral
            and command_is_zero(frame.command)
        ):
            self._resume_requires_neutral = False
            self._zeroed_for_timeout = False
        elif not self._resume_requires_neutral:
            self._zeroed_for_timeout = False
        self._last_error = None

        if (
            self._control.arbiter.mode == ControlMode.GAMEPAD
            and (not frame.control_enabled or not frame.gamepad_connected)
        ):
            reason = (
                "gamepad_control_disabled"
                if not frame.control_enabled else "gamepad_unplugged")
            await self.safe_shutdown(reason)

        accepted, reason = self._frame_eligibility(frame)
        await self._send_ack(
            ws, message.sequence, accepted, reason, mapped.command,
            mapped.heave_conflict)

    async def enable(self) -> None:
        if not self._config.features.gamepad_control:
            raise GamepadControlError(403, "Gamepad control is disabled")
        async with self._control.lock:
            if self._control.arbiter.mode != ControlMode.IDLE:
                raise GamepadControlError(
                    409,
                    f"{self._control.arbiter.mode.value} is already owned by "
                    f"{self._control.arbiter.owner}",
                )
            state = self._proxy.refresh_link_state()
            frame = self._latest
            if frame is None or self._session_id is None or self._ws is None:
                raise GamepadControlError(
                    409, "No gamepad client owns the control lease")
            age_ms = (self._clock() - frame.received_mono) * 1000.0
            if age_ms > self._gamepad.zero_timeout_ms:
                raise GamepadControlError(
                    409, "Latest gamepad frame is stale")
            if not frame.gamepad_connected:
                raise GamepadControlError(409, "USB gamepad is disconnected")
            if not frame.control_enabled:
                raise GamepadControlError(
                    409, "Forwarder control_enabled is false")
            if not command_is_zero(frame.command):
                raise GamepadControlError(
                    409, "Center all sticks and release A/Y before GAMEPAD")
            self._require_online_idle_neutral(state)

            activating_ws = self._ws
            ok = await self._proxy.enable_body_control()
            if not ok:
                self._control.arbiter.force_idle(
                    "gamepad_enable_failed")
                await self._proxy.disarm()
                raise GamepadControlError(
                    504, self._proxy.state.last_command_error
                    or "Failed to enable STM32 body control")

            latest = self._latest
            activation_error = None
            if (
                self._ws is not activating_ws
                or self._session_id != frame.session_id
                or latest is None
                or latest.session_id != frame.session_id
            ):
                activation_error = "Gamepad lease changed while enabling"
            elif (
                (self._clock() - latest.received_mono) * 1000.0
                > self._gamepad.zero_timeout_ms
            ):
                activation_error = "Latest gamepad frame became stale while enabling"
            elif not latest.gamepad_connected:
                activation_error = "USB gamepad disconnected while enabling"
            elif not latest.control_enabled:
                activation_error = "Forwarder disabled control while enabling"
            elif not command_is_zero(latest.command):
                activation_error = (
                    "Center all sticks and release A/Y before GAMEPAD")
            elif (
                self._control.motion_inhibited
                or self._control.safety_transition_active
            ):
                activation_error = "Safety state changed while enabling GAMEPAD"

            if activation_error is not None:
                self._control.arbiter.force_idle(
                    "gamepad_enable_aborted")
                rollback_ok = await self._proxy.disarm()
                if not rollback_ok:
                    self._control.inhibit_motion(
                        "gamepad_enable_rollback_unconfirmed")
                raise GamepadControlError(409, activation_error)

            try:
                self._control.arbiter.acquire(
                    ControlMode.GAMEPAD, latest.session_id)
            except ControlModeConflict as exc:
                self._control.arbiter.force_idle(
                    "gamepad_enable_conflict")
                rollback_ok = await self._proxy.disarm()
                if not rollback_ok:
                    self._control.inhibit_motion(
                        "gamepad_enable_rollback_unconfirmed")
                raise GamepadControlError(409, str(exc)) from exc
            self._resume_requires_neutral = False
            self._zeroed_for_timeout = False
        logger.warning("GAMEPAD mode enabled for session %s", latest.session_id)

    async def disable(self, reason: str = "gamepad_api_disable") -> dict:
        if self._control.arbiter.mode != ControlMode.GAMEPAD:
            raise GamepadControlError(409, "GAMEPAD mode is not active")
        return await self.safe_shutdown(reason)

    async def safe_shutdown(self, reason: str) -> dict:
        transition_started = self._control.begin_safety_transition(reason)
        try:
            async with self._control.lock:
                return await self._shutdown_locked(reason)
        finally:
            if transition_started:
                self._control.finish_safety_transition()

    async def _shutdown_locked(self, reason: str) -> dict:
        result = {"zero": True, "disable": True, "disarm": True}
        mode_was_gamepad = (
            self._control.arbiter.mode == ControlMode.GAMEPAD)
        if not mode_was_gamepad:
            self._last_disconnect_reason = reason
            self._last_shutdown_result = result
            return result
        state = self._proxy.refresh_link_state()

        if (
            state.safety_state == SafetyState.ARMED_ACTIVE
            and state.to_dict()["body_control_enabled"]
        ):
            result["zero"] = await self._bounded(
                self._proxy.stop_body_motion(), "zero_body_command")
            result["disable"] = await self._bounded(
                self._proxy.disable_body_control(), "disable_body_control")
        else:
            result["zero"] = await self._bounded(
                self._proxy.force_neutral(
                    reason, confirm=True, timeout=0.4),
                "force_neutral",
            )

        state = self._proxy.refresh_link_state()
        if state.safety_state not in (
            SafetyState.DISARMED,
            SafetyState.EMERGENCY_STOP,
        ):
            result["disarm"] = await self._bounded(
                self._proxy.disarm(), "disarm")

        self._control.arbiter.force_idle(reason)
        self._zeroed_for_timeout = True
        self._last_disconnect_reason = reason
        self._last_shutdown_result = result
        if not all(result.values()):
            self._control.inhibit_motion(
                f"{reason}_shutdown_unconfirmed")
        logger.warning(
            "GAMEPAD shutdown reason=%s zero=%s disable=%s disarm=%s",
            reason, result["zero"], result["disable"], result["disarm"])
        return result

    async def _bounded(self, operation, name: str) -> bool:
        try:
            return bool(await asyncio.wait_for(operation, timeout=0.5))
        except asyncio.TimeoutError:
            self._last_error = f"{name}_timeout"
            logger.error("GAMEPAD safety operation timed out: %s", name)
            return False
        except Exception as exc:
            self._last_error = f"{name}_failed: {exc}"
            logger.exception("GAMEPAD safety operation failed: %s", name)
            return False

    def _require_online_idle_neutral(self, state) -> None:
        if not state.serial_connected:
            raise GamepadControlError(503, "Serial transport is disconnected")
        if not state.stm32_online or state.status_stale:
            raise GamepadControlError(
                503, "STM32 is offline or status is stale")
        if state.safety_state == SafetyState.EMERGENCY_STOP:
            raise GamepadControlError(409, "Emergency stop is active")
        if state.safety_state == SafetyState.FAULT:
            raise GamepadControlError(409, "STM32 is in FAULT")
        if state.safety_state != SafetyState.ARMED_IDLE:
            raise GamepadControlError(
                409, "ARM the system before entering GAMEPAD")
        if any(value != 1500 for value in state.confirmed_pwm):
            raise GamepadControlError(
                409, "All eight confirmed PWM values must be 1500us")
        if self._control.motion_inhibited:
            raise GamepadControlError(
                409, f"Motion is inhibited: "
                f"{self._control.motion_inhibit_reason}")
        if self._control.safety_transition_active:
            raise GamepadControlError(
                409, "A safety transition is in progress")

    def _frame_eligibility(
        self, frame: LatestGamepadFrame
    ) -> tuple[bool, str]:
        if not frame.gamepad_connected:
            return False, "gamepad_disconnected"
        if not frame.control_enabled:
            return False, "control_disabled"
        if self._control.arbiter.mode != ControlMode.GAMEPAD:
            return False, "mode_not_gamepad"
        if self._control.arbiter.owner != frame.session_id:
            return False, "gamepad_lease_not_mode_owner"
        if self._control.motion_inhibited:
            return False, "motion_inhibited"
        if self._control.safety_transition_active:
            return False, "safety_transition_active"
        if (
            self._resume_requires_neutral
            and not command_is_zero(frame.command)
        ):
            return False, "center_controls_after_timeout"
        state = self._proxy.refresh_link_state()
        if not state.serial_connected or not state.stm32_online:
            return False, "stm32_offline"
        if state.status_stale:
            return False, "stm32_status_stale"
        if state.safety_state == SafetyState.EMERGENCY_STOP:
            return False, "estop_active"
        if state.safety_state == SafetyState.FAULT:
            return False, "fault_active"
        if (
            state.safety_state != SafetyState.ARMED_ACTIVE
            or not state.to_dict()["body_control_enabled"]
        ):
            return False, "stm32_body_control_inactive"
        return True, "accepted"

    async def _pump_loop(self) -> None:
        interval = 1.0 / self._gamepad.send_hz
        while True:
            started = self._clock()
            try:
                await self.process_once(started)
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("Gamepad command pump recovered from error")
            elapsed = self._clock() - started
            await asyncio.sleep(max(0.0, interval - elapsed))

    async def process_once(self, now: Optional[float] = None) -> None:
        if self._control.arbiter.mode != ControlMode.GAMEPAD:
            return
        frame = self._latest
        if frame is None:
            await self.safe_shutdown("gamepad_frame_missing")
            return
        current = self._clock() if now is None else now
        age_ms = (current - frame.received_mono) * 1000.0
        if age_ms >= self._gamepad.disconnect_timeout_ms:
            await self.safe_shutdown("gamepad_timeout_1000ms")
            return

        if (
            self._resume_requires_neutral
            and not command_is_zero(frame.command)
        ):
            return

        accepted, reason = self._frame_eligibility(frame)
        if not accepted:
            await self.safe_shutdown(f"gamepad_rejected_{reason}")
            return

        if age_ms >= self._gamepad.zero_timeout_ms:
            if not self._zeroed_for_timeout:
                async with self._control.lock:
                    ok = await self._bounded(
                        self._proxy.stop_body_motion(),
                        "gamepad_timeout_zero",
                    )
                self._last_stm32_ack = ok
                self._zeroed_for_timeout = True
                self._resume_requires_neutral = True
                if not ok:
                    await self.safe_shutdown(
                        "gamepad_timeout_zero_unconfirmed")
            return

        async with self._control.lock:
            if self._control.arbiter.mode != ControlMode.GAMEPAD:
                return
            ok = await self._bounded(
                self._proxy.send_body_command(frame.command),
                "set_body_command",
            )
        self._last_stm32_ack = ok
        if ok:
            self._last_forwarded_sequence = frame.sequence
            self._last_forwarded_at = self._wall_clock()
        else:
            await self.safe_shutdown("gamepad_set_body_command_failed")

    async def _send_ack(
        self,
        ws: WebSocket,
        sequence: Optional[int],
        accepted: bool,
        reason: str,
        command: Optional[SetBodyCommand] = None,
        heave_conflict: bool = False,
    ) -> None:
        try:
            await ws.send_json({
                "type": "gamepad_ack",
                "version": 1,
                "session_id": self._session_id,
                "sequence": sequence,
                "accepted": accepted,
                "reason": reason,
                "server_time_ns": time.monotonic_ns(),
                "control_mode": self._control.arbiter.mode.value,
                "mapped_command": command_dict(
                    command if command is not None else _ZERO_COMMAND),
                "heave_conflict": heave_conflict,
                "last_forwarded_sequence": self._last_forwarded_sequence,
                "last_stm32_ack": self._last_stm32_ack,
            })
        except Exception:
            logger.debug("Failed to send gamepad ACK", exc_info=True)

    def status_snapshot(self) -> dict:
        frame = self._latest
        age_ms = (
            max(0.0, (self._clock() - frame.received_mono) * 1000.0)
            if frame is not None else None
        )
        accepted, reason = (
            self._frame_eligibility(frame)
            if frame is not None else (False, "no_gamepad_frame")
        )
        return {
            "client_connected": self.client_connected,
            "lease_session_id": self._session_id,
            "lease_active": self.client_connected and self._session_id is not None,
            "gamepad_connected": (
                frame.gamepad_connected if frame is not None else False),
            "control_enabled": (
                frame.control_enabled if frame is not None else False),
            "eligible": accepted,
            "eligibility_reason": reason,
            "device": frame.device if frame is not None else None,
            "axes": list(frame.axes) if frame is not None else [],
            "buttons": list(frame.buttons) if frame is not None else [],
            "mapped_command": command_dict(
                frame.command if frame is not None else _ZERO_COMMAND),
            "heave_conflict": (
                frame.heave_conflict if frame is not None else False),
            "last_sequence": frame.sequence if frame is not None else None,
            "last_received_at": (
                frame.received_wall if frame is not None else None),
            "command_age_ms": int(age_ms) if age_ms is not None else None,
            "last_forwarded_sequence": self._last_forwarded_sequence,
            "last_forwarded_at": self._last_forwarded_at or None,
            "last_stm32_ack": self._last_stm32_ack,
            "last_error": self._last_error,
            "last_disconnect_reason": self._last_disconnect_reason,
            "last_shutdown_result": self._last_shutdown_result,
            "rejected_messages": self.rejected_messages,
            "duplicate_messages": self.duplicate_messages,
            "resume_requires_neutral": self._resume_requires_neutral,
            "zero_timeout_ms": self._gamepad.zero_timeout_ms,
            "disconnect_timeout_ms": self._gamepad.disconnect_timeout_ms,
            "send_hz": self._gamepad.send_hz,
        }
