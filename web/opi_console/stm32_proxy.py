"""
High-level STM32 proxy API.

Wraps SerialTransport with typed command/response methods.
Maintains a cached copy of the latest STM32 state for web API queries.
"""

import asyncio
import logging
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Callable

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol", "shared"))

from protocol import (
    MsgType, SafetyState, NeutralReason,
    SetPwm, SetBodyCommand, MotionTuning, StatusReport, SensorReport,
    HeartbeatAck, SafetyEvent,
)

from opi_console.serial_transport import SerialTransport
from opi_console.config import AppConfig, coerce_config
from opi_console.motion_tuning import (
    MotionTuningStore,
    clone_motion_tuning,
    motion_tuning_equal,
    validate_motion_tuning,
)

logger = logging.getLogger("opi_console.proxy")


def _json_number(value: float):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


@dataclass
class CachedState:
    """Cached STM32 state, updated from telemetry."""
    safety_state: int = SafetyState.DISARMED
    flags: int = 0
    pwm: List[int] = field(default_factory=lambda: [1500] * 8)
    requested_pwm: List[int] = field(default_factory=lambda: [1500] * 8)
    error_count: int = 0
    heartbeat_missed: int = 0
    neutral_reason: int = NeutralReason.NONE
    active_channel: int = 0xFF
    request_state: str = "idle"
    last_command_error: Optional[str] = None
    last_request_at: float = 0.0

    last_any_frame_at: float = 0.0
    last_heartbeat_ack_at: float = 0.0
    last_status_report_at: float = 0.0
    last_sensor_report_at: float = 0.0
    last_command_ack_at: float = 0.0
    _last_status_report_mono: float = field(default=0.0, repr=False)
    _last_sensor_report_mono: float = field(default=0.0, repr=False)

    # Sensors
    depth_m: float = 0.0
    pressure_mbar: float = 0.0
    water_temp_c: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    accel: List[float] = field(default_factory=lambda: [0.0] * 3)
    gyro: List[float] = field(default_factory=lambda: [0.0] * 3)
    mag: List[float] = field(default_factory=lambda: [0.0] * 3)
    # Serial stats
    serial_connected: bool = False
    stm32_online: bool = False
    status_stale: bool = True
    sensors_stale: bool = True
    motion_tuning_synced: bool = False
    motion_tuning_sync_state: str = "pending"
    motion_tuning_sync_error: Optional[str] = None

    @property
    def confirmed_pwm(self) -> List[int]:
        return self.pwm

    @property
    def desired_pwm(self) -> List[int]:
        """Backward-compatible alias for requested_pwm."""
        return self.requested_pwm

    @desired_pwm.setter
    def desired_pwm(self, value: List[int]):
        self.requested_pwm = value

    @property
    def last_update(self) -> float:
        """Backward-compatible alias for the status-report timestamp."""
        return self.last_status_report_at

    @last_update.setter
    def last_update(self, value: float):
        self.last_status_report_at = value

    @property
    def sensors_last_update(self) -> float:
        return self.last_sensor_report_at

    @sensors_last_update.setter
    def sensors_last_update(self, value: float):
        self.last_sensor_report_at = value

    @property
    def desired_last_update(self) -> float:
        return self.last_request_at

    @desired_last_update.setter
    def desired_last_update(self, value: float):
        self.last_request_at = value

    def to_dict(self) -> dict:
        now_mono = time.monotonic()
        status_age = (
            now_mono - self._last_status_report_mono
            if self._last_status_report_mono else None
        )
        sensors_age = (
            now_mono - self._last_sensor_report_mono
            if self._last_sensor_report_mono else None
        )
        try:
            safety_state_name = SafetyState(self.safety_state).name
        except ValueError:
            safety_state_name = f"UNKNOWN_{self.safety_state}"
        try:
            neutral_reason_name = NeutralReason(self.neutral_reason).name
        except ValueError:
            neutral_reason_name = f"UNKNOWN_{self.neutral_reason}"
        return {
            "safety_state": self.safety_state,
            "safety_state_name": safety_state_name,
            "armed": self.safety_state in (
                SafetyState.ARMED_IDLE,
                SafetyState.ARMED_ACTIVE,
                SafetyState.MANUAL_TEST,
            ),
            "control_enable": bool(self.flags & 0x01),
            "float_enabled": bool(self.flags & 0x02),
            "angle_enabled": bool(self.flags & 0x04),
            "manual_pwm_enabled": bool(self.flags & 0x08),
            "estop_locked": bool(self.flags & 0x10),
            "body_control_enabled": bool(self.flags & 0x20),
            "horizontal_saturated": bool(self.flags & 0x40),
            "vertical_saturated": bool(self.flags & 0x80),
            "motion_tuning_synced": self.motion_tuning_synced,
            "motion_tuning_sync_state": self.motion_tuning_sync_state,
            "motion_tuning_sync_error": self.motion_tuning_sync_error,
            "pwm": list(self.pwm),
            "confirmed_pwm": list(self.pwm),
            "requested_pwm": list(self.requested_pwm),
            "desired_pwm": list(self.requested_pwm),
            "request_state": self.request_state,
            "last_command_error": self.last_command_error,
            "neutral_reason": self.neutral_reason,
            "neutral_reason_name": neutral_reason_name,
            "active_channel": None if self.active_channel == 0xFF else self.active_channel,
            "error_count": self.error_count,
            "heartbeat_missed": self.heartbeat_missed,
            "last_any_frame_at": self.last_any_frame_at,
            "last_heartbeat_ack_at": self.last_heartbeat_ack_at,
            "last_status_report_at": self.last_status_report_at,
            "last_sensor_report_at": self.last_sensor_report_at,
            "last_command_ack_at": self.last_command_ack_at,
            "last_update": self.last_status_report_at,
            "desired_last_update": self.last_request_at,
            "status_stale": self.status_stale,
            "sensors_stale": self.sensors_stale,
            "status_age_ms": int(status_age * 1000) if status_age is not None else None,
            "sensors_age_ms": int(sensors_age * 1000) if sensors_age is not None else None,
            "stm32_online": self.stm32_online,
            "serial_connected": self.serial_connected,
        }

    def sensors_to_dict(self) -> dict:
        return {
            "depth_m": _json_number(self.depth_m),
            "pressure_mbar": _json_number(self.pressure_mbar),
            "water_temp_c": _json_number(self.water_temp_c),
            "yaw": _json_number(self.yaw),
            "pitch": _json_number(self.pitch),
            "roll": _json_number(self.roll),
            "accel": [_json_number(value) for value in self.accel],
            "gyro": [_json_number(value) for value in self.gyro],
            "mag": [_json_number(value) for value in self.mag],
            "last_sensor_report_at": self.last_sensor_report_at,
            "last_update": self.last_sensor_report_at,
            "sensors_stale": self.sensors_stale,
        }


class Stm32Proxy:
    """High-level async API for interacting with the STM32."""

    def __init__(self, transport: SerialTransport,
                 config: Optional[AppConfig] = None,
                 sensor_poll_hz: Optional[float] = None,
                 status_stale_timeout_s: Optional[float] = None,
                 sensors_stale_timeout_s: Optional[float] = None):
        self._transport = transport
        self.config = coerce_config(
            config if config is not None else transport._config)
        pwm_cfg = self.config.pwm
        telemetry_cfg = self.config.telemetry
        self._state = CachedState(
            pwm=[pwm_cfg.neutral_us] * pwm_cfg.channel_count,
            requested_pwm=[pwm_cfg.neutral_us] * pwm_cfg.channel_count,
        )
        self._lock = asyncio.Lock()
        self._neutral_us = pwm_cfg.neutral_us
        self._pwm_test_min = pwm_cfg.min_test_us
        self._pwm_test_max = pwm_cfg.max_test_us
        self._min_test_duration_ms = pwm_cfg.min_test_duration_ms
        self._max_test_duration_ms = pwm_cfg.max_test_duration_ms
        self._channel_count = pwm_cfg.channel_count
        self._online_timeout_s = telemetry_cfg.stm32_online_timeout_s
        self._status_stale_timeout_s = float(
            status_stale_timeout_s if status_stale_timeout_s is not None
            else telemetry_cfg.status_stale_timeout_s)
        self._sensors_stale_timeout_s = float(
            sensors_stale_timeout_s if sensors_stale_timeout_s is not None
            else telemetry_cfg.sensors_stale_timeout_s)
        self._sensor_poll_hz = float(
            sensor_poll_hz if sensor_poll_hz is not None
            else telemetry_cfg.sensors_hz)
        self._report_request_timeout_s = telemetry_cfg.request_timeout_s
        self._command_confirmation_timeout_s = (
            telemetry_cfg.command_confirmation_timeout_s)
        self._command_confirmation_poll_interval_s = (
            telemetry_cfg.command_confirmation_poll_interval_s)
        self._sensor_poll_task: Optional[asyncio.Task] = None
        self._motion_tuning_sync_task: Optional[asyncio.Task] = None
        self._status_request_lock = asyncio.Lock()
        self._sensor_request_lock = asyncio.Lock()
        self._motion_tuning_request_lock = asyncio.Lock()
        self._motion_tuning_apply_lock = asyncio.Lock()
        self._sensor_request_in_flight = False
        self.sensor_poll_requests = 0
        self.sensor_poll_failures = 0
        self._last_sensor_poll_warning_mono = 0.0
        self._logged_online = False
        self._last_stm32_uptime_s: Optional[int] = None
        self._connection_generation = transport.connection_generation
        self._neutral_reason_override: Optional[NeutralReason] = None
        self._motion_tuning_store = MotionTuningStore(
            self.config.resolve_path(self.config.motion_tuning.file))
        self._desired_motion_tuning = self._motion_tuning_store.load()
        self._confirmed_motion_tuning: Optional[MotionTuning] = None
        self._motion_tuning_sync_interval_s = (
            self.config.motion_tuning.sync_interval_s)
        self.last_command_sequence: Optional[int] = None
        self._event_callbacks: Dict[str, List[Callable]] = {
            "status": [],
            "sensors": [],
            "connection": [],
            "event": [],
        }

        # Register frame handler
        transport.on_frame(self._on_frame)

    # -------- Properties --------

    @property
    def state(self) -> CachedState:
        self.refresh_link_state()
        return self._state

    def refresh_link_state(self) -> CachedState:
        """Update online/stale flags from transport timestamps."""
        self._sync_connection_generation()
        now_mono = time.monotonic()
        self._state.serial_connected = self._transport.connected
        self._state.last_any_frame_at = self._transport.last_any_frame_at
        self._state.last_heartbeat_ack_at = self._transport.last_heartbeat_ack_at
        self._state.last_command_ack_at = self._transport.last_command_ack_at
        rx_age = (now_mono - self._transport._last_any_frame_mono
                  if self._transport._last_any_frame_mono else None)
        online = bool(
            self._transport.connected
            and self._transport._last_any_frame_mono
            and rx_age is not None
            and rx_age <= self._online_timeout_s
        )
        if online and not self._logged_online:
            logger.info("STM32 online")
            self._logged_online = True
        elif not online and self._logged_online:
            logger.warning("STM32 offline/stale (rx_age=%s)", rx_age)
            self._logged_online = False
        self._state.stm32_online = online
        self._state.status_stale = (not online) or (
            self._state._last_status_report_mono == 0.0
            or (now_mono - self._state._last_status_report_mono)
            > self._status_stale_timeout_s
        )
        self._state.sensors_stale = (not online) or (
            self._state._last_sensor_report_mono == 0.0
            or (now_mono - self._state._last_sensor_report_mono)
            > self._sensors_stale_timeout_s
        )
        return self._state

    def _sync_connection_generation(self):
        generation = self._transport.connection_generation
        if generation == self._connection_generation:
            return
        logger.info(
            "Serial connection generation changed %d -> %d; "
            "invalidating cached telemetry",
            self._connection_generation, generation)
        self._logged_online = False
        self._connection_generation = generation
        self._state.last_status_report_at = 0.0
        self._state.last_sensor_report_at = 0.0
        self._state._last_status_report_mono = 0.0
        self._state._last_sensor_report_mono = 0.0
        self._state.stm32_online = False
        self._state.status_stale = True
        self._state.sensors_stale = True
        self._state.requested_pwm = [self._neutral_us] * self._channel_count
        self._state.request_state = "idle"
        self._state.last_command_error = None
        self._state.active_channel = 0xFF
        self._state.motion_tuning_synced = False
        self._state.motion_tuning_sync_state = "pending"
        self._state.motion_tuning_sync_error = None
        self._confirmed_motion_tuning = None
        self._last_stm32_uptime_s = None
        self._neutral_reason_override = None

    def status_snapshot(self) -> dict:
        self.refresh_link_state()
        return self._state.to_dict()

    def sensors_snapshot(self) -> dict:
        self.refresh_link_state()
        return self._state.sensors_to_dict()

    def motion_tuning_snapshot(self) -> dict:
        self.refresh_link_state()
        return {
            "desired": self._desired_motion_tuning.to_dict(),
            "confirmed": (
                self._confirmed_motion_tuning.to_dict()
                if self._confirmed_motion_tuning is not None
                else None
            ),
            "synced": self._state.motion_tuning_synced,
            "sync_state": self._state.motion_tuning_sync_state,
            "sync_error": self._state.motion_tuning_sync_error,
            "persist_path": str(self._motion_tuning_store.path),
        }

    async def start_background_tasks(self):
        """Start process-wide telemetry and tuning synchronization loops."""
        if (
            self._sensor_poll_hz > 0
            and (
                self._sensor_poll_task is None
                or self._sensor_poll_task.done()
            )
        ):
            self._sensor_poll_task = asyncio.create_task(
                self._sensor_poll_loop(), name="stm32-sensor-poll")
        if (
            self.config.features.motion_tuning
            and (
                self._motion_tuning_sync_task is None
                or self._motion_tuning_sync_task.done()
            )
        ):
            self._motion_tuning_sync_task = asyncio.create_task(
                self._motion_tuning_sync_loop(),
                name="stm32-motion-tuning-sync",
            )

    @property
    def sensor_poll_task_running(self) -> bool:
        return bool(
            self._sensor_poll_task is not None
            and not self._sensor_poll_task.done()
        )

    @property
    def motion_tuning_sync_task_running(self) -> bool:
        return bool(
            self._motion_tuning_sync_task is not None
            and not self._motion_tuning_sync_task.done()
        )

    async def stop_background_tasks(self):
        tasks = [
            task for task in (
                self._sensor_poll_task,
                self._motion_tuning_sync_task,
            )
            if task is not None and not task.done()
        ]
        self._sensor_poll_task = None
        self._motion_tuning_sync_task = None
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _sensor_poll_loop(self):
        interval = 1.0 / self._sensor_poll_hz
        while True:
            try:
                state = self.refresh_link_state()
                if self._transport.connected and state.stm32_online:
                    self.sensor_poll_requests += 1
                    result = await self.request_sensors()
                    if result is None:
                        self.sensor_poll_failures += 1
                        now = time.monotonic()
                        if now - self._last_sensor_poll_warning_mono >= 10.0:
                            logger.warning(
                                "Sensor polling request timed out "
                                "(failures=%d)", self.sensor_poll_failures)
                            self._last_sensor_poll_warning_mono = now
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                raise
            except Exception:
                self.sensor_poll_failures += 1
                logger.exception(
                    "Sensor polling task recovered from an unexpected error")
                await asyncio.sleep(interval)

    async def _motion_tuning_sync_loop(self):
        while True:
            try:
                state = self.refresh_link_state()
                if (
                    self._transport.connected
                    and state.stm32_online
                    and not state.motion_tuning_synced
                    and state.safety_state in (
                        SafetyState.DISARMED,
                        SafetyState.ARMED_IDLE,
                    )
                ):
                    await self.synchronize_motion_tuning()
                await asyncio.sleep(self._motion_tuning_sync_interval_s)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._state.motion_tuning_synced = False
                self._state.motion_tuning_sync_state = "error"
                self._state.motion_tuning_sync_error = str(exc)
                logger.exception(
                    "Motion tuning synchronization recovered from an error")
                await asyncio.sleep(self._motion_tuning_sync_interval_s)

    # -------- Event callbacks --------

    def on(self, event: str, callback: Callable):
        """Register an event callback. Events: status, sensors, connection, event."""
        if event in self._event_callbacks:
            self._event_callbacks[event].append(callback)

    async def _emit(self, event: str, *args):
        for cb in self._event_callbacks.get(event, []):
            try:
                if asyncio.iscoroutinefunction(cb):
                    await cb(*args)
                else:
                    cb(*args)
            except Exception:
                logger.exception("Proxy event callback failed: %s", event)

    # -------- Commands --------

    def _begin_pwm_request(self, requested_pwm: List[int]):
        if any(value != self._neutral_us for value in requested_pwm):
            self._neutral_reason_override = None
        self._state.requested_pwm = list(requested_pwm)
        self._state.request_state = "pending"
        self._state.last_command_error = None
        self._state.last_request_at = time.time()

    def _fail_pwm_request(self):
        error = self._transport.last_error or "command failed"
        self._state.last_command_error = error
        self._state.request_state = (
            "rejected" if error.startswith("NACK:") else "timeout")

    def _confirm_pwm_from_report(self, report: Optional[StatusReport]) -> bool:
        if report is None:
            self._state.request_state = "timeout"
            self._state.last_command_error = "status confirmation timeout"
            return False
        if list(report.pwm) == list(self._state.requested_pwm):
            self._state.request_state = "confirmed"
            self._state.last_command_error = None
            return True
        if (report.neutral_reason == NeutralReason.PWM_COMMAND_TIMEOUT
                and all(value == self._neutral_us for value in report.pwm)):
            self._state.requested_pwm = [self._neutral_us] * self._channel_count
            self._state.request_state = "idle"
            self._state.last_command_error = None
            return True
        self._state.last_command_error = "status did not confirm requested PWM"
        self._state.request_state = "timeout"
        return False

    async def _wait_for_pwm_confirmation(
        self, duration_ms: int, expected_pwm: List[int]
    ) -> bool:
        # Keep the confirmation window inside a short pulse's lifetime while
        # allowing at least two STM32 control-task periods.
        timeout = min(
            self._command_confirmation_timeout_s,
            max(0.05, duration_ms / 1000.0 * 0.75),
        )
        deadline = time.monotonic() + timeout
        last_report = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            report = await self.request_status(timeout=min(
                self._report_request_timeout_s, remaining))
            if report is not None:
                last_report = report
                if list(report.pwm) == expected_pwm:
                    self._state.requested_pwm = list(expected_pwm)
                    return self._confirm_pwm_from_report(report)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            await asyncio.sleep(min(
                self._command_confirmation_poll_interval_s, remaining))
        if (last_report is not None
                and last_report.neutral_reason
                == NeutralReason.PWM_COMMAND_TIMEOUT
                and all(value == self._neutral_us for value in last_report.pwm)):
            self._state.requested_pwm = [self._neutral_us] * self._channel_count
            self._state.request_state = "timeout"
            self._state.last_command_error = (
                "PWM pulse ended before active output was confirmed")
            return False
        return self._confirm_pwm_from_report(last_report)

    def _confirm_safety_state(self, report: Optional[StatusReport],
                              expected: tuple[SafetyState, ...],
                              operation: str) -> bool:
        if report is None:
            self._state.last_command_error = (
                f"{operation} status confirmation timeout")
            return False
        if report.safety_state not in expected:
            names = "/".join(state.name for state in expected)
            try:
                actual = SafetyState(report.safety_state).name
            except ValueError:
                actual = str(report.safety_state)
            self._state.last_command_error = (
                f"{operation} expected {names}, confirmed {actual}")
            return False
        self._state.last_command_error = None
        return True

    async def arm(self) -> bool:
        """Request ARM. Returns True if ACKed."""
        self._state.last_command_error = None
        result = await self._transport.send_frame(MsgType.ARM, expect_ack=True)
        ok = result is not None
        if ok:
            self.last_command_sequence = result
        if ok:
            report = await self.request_status()
            ok = self._confirm_safety_state(
                report, (SafetyState.ARMED_IDLE,), "ARM")
        logger.info("ARM %s", "CONFIRMED" if ok else "FAILED")
        return ok

    async def disarm(self) -> bool:
        """Request DISARM."""
        self._begin_pwm_request([self._neutral_us] * self._channel_count)
        result = await self._transport.send_frame(MsgType.DISARM, expect_ack=True)
        ok = result is not None
        if ok:
            self.last_command_sequence = result
        if ok:
            report = await self.request_status()
            pwm_ok = self._confirm_pwm_from_report(report)
            state_ok = self._confirm_safety_state(
                report,
                (SafetyState.DISARMED, SafetyState.EMERGENCY_STOP),
                "DISARM",
            )
            ok = pwm_ok and state_ok
        else:
            self._fail_pwm_request()
        logger.info("DISARM %s", "ACK" if ok else "TIMEOUT")
        return ok

    async def emergency_stop(self) -> bool:
        """Send EMERGENCY_STOP (sent 3 times for reliability)."""
        self._begin_pwm_request([self._neutral_us] * self._channel_count)
        ok = False
        for _ in range(3):
            result = await self._transport.send_frame(
                MsgType.EMERGENCY_STOP, expect_ack=True)
            if result is not None:
                ok = True
                self.last_command_sequence = result
        if ok:
            report = await self.request_status()
            pwm_ok = self._confirm_pwm_from_report(report)
            state_ok = self._confirm_safety_state(
                report, (SafetyState.EMERGENCY_STOP,), "EMERGENCY_STOP")
            ok = pwm_ok and state_ok
        else:
            self._fail_pwm_request()
        logger.critical("EMERGENCY_STOP %s", "ACK" if ok else "FAILED")
        return ok

    async def reset_estop(self) -> bool:
        self._state.last_command_error = None
        result = await self._transport.send_frame(MsgType.RESET_ESTOP, expect_ack=True)
        ok = result is not None
        if ok:
            self.last_command_sequence = result
        if ok:
            report = await self.request_status()
            ok = self._confirm_safety_state(
                report, (SafetyState.DISARMED,), "RESET_ESTOP")
        logger.info("RESET_ESTOP %s", "CONFIRMED" if ok else "FAILED")
        return ok

    async def enter_manual(self) -> bool:
        self._state.last_command_error = None
        result = await self._transport.send_frame(MsgType.ENTER_MANUAL, expect_ack=True)
        if result is not None:
            self.last_command_sequence = result
            report = await self.request_status()
            ok = self._confirm_safety_state(
                report, (SafetyState.MANUAL_TEST,), "ENTER_MANUAL")
        else:
            ok = False
        logger.info("ENTER_MANUAL %s", "CONFIRMED" if ok else "FAILED")
        return ok

    async def exit_manual(self) -> bool:
        self._begin_pwm_request([self._neutral_us] * self._channel_count)
        result = await self._transport.send_frame(MsgType.EXIT_MANUAL, expect_ack=True)
        if result is not None:
            self.last_command_sequence = result
            report = await self.request_status()
            pwm_ok = self._confirm_pwm_from_report(report)
            state_ok = self._confirm_safety_state(
                report, (SafetyState.ARMED_IDLE,), "EXIT_MANUAL")
            return pwm_ok and state_ok
        self._fail_pwm_request()
        return False

    async def set_pwm(self, channel: int, pwm_us: int, duration_ms: int = 500) -> bool:
        """Set a single PWM channel for testing."""
        if channel < 0 or channel >= self._channel_count:
            logger.warning("SET_PWM rejected locally: bad channel %d", channel)
            return False
        if pwm_us < self._pwm_test_min or pwm_us > self._pwm_test_max:
            logger.warning("SET_PWM rejected locally: pwm_us=%d", pwm_us)
            return False
        cmd = SetPwm(channel=channel, pwm_us=pwm_us, timeout_ms=duration_ms)
        if (duration_ms < self._min_test_duration_ms
                or duration_ms > self._max_test_duration_ms):
            logger.warning("SET_PWM rejected locally: duration_ms=%d", duration_ms)
            return False
        desired = [self._neutral_us] * self._channel_count
        desired[channel] = pwm_us
        self._begin_pwm_request(desired)
        result = await self._transport.send_frame(
            MsgType.SET_PWM, cmd.pack(), expect_ack=True)
        self.last_command_sequence = (
            result if result is not None
            else self._transport.last_command_sequence_attempted
        )
        if result is None:
            self._fail_pwm_request()
            logger.warning("SET_PWM CH%d failed: %s", channel,
                           self._state.last_command_error)
            return False
        ok = await self._wait_for_pwm_confirmation(duration_ms, desired)
        if ok:
            logger.info("SET_PWM CH%d=%dus %dms", channel, pwm_us, duration_ms)
        else:
            logger.warning("SET_PWM CH%d not confirmed: %s", channel,
                           self._state.last_command_error)
        return ok

    async def set_all_neutral(self, confirm: bool = True) -> bool:
        self._begin_pwm_request([self._neutral_us] * self._channel_count)
        result = await self._transport.send_frame(
            MsgType.SET_ALL_NEUTRAL, expect_ack=True)
        ok = result is not None
        if ok:
            self.last_command_sequence = result
        if ok and confirm:
            report = await self.request_status()
            ok = self._confirm_pwm_from_report(report)
        elif ok:
            self._state.request_state = "pending"
        else:
            self._fail_pwm_request()
        return ok

    def _set_motion_tuning_error(self, message: str):
        self._state.motion_tuning_synced = False
        self._state.motion_tuning_sync_state = "error"
        self._state.motion_tuning_sync_error = message

    def _confirm_motion_tuning(self, tuning: MotionTuning):
        self._confirmed_motion_tuning = clone_motion_tuning(tuning)
        synced = motion_tuning_equal(
            self._desired_motion_tuning,
            self._confirmed_motion_tuning,
        )
        self._state.motion_tuning_synced = synced
        self._state.motion_tuning_sync_state = (
            "synced" if synced else "mismatch")
        self._state.motion_tuning_sync_error = (
            None if synced else "STM32 tuning differs from persisted tuning")

    async def _synchronize_motion_tuning_locked(self) -> bool:
        state = self.refresh_link_state()
        if not self._transport.connected or not state.stm32_online:
            self._set_motion_tuning_error("STM32 is offline")
            return False
        if state.status_stale:
            report = await self.request_status()
            if report is None:
                self._set_motion_tuning_error(
                    "status confirmation timed out before tuning sync")
                return False
            state = self.refresh_link_state()
        if state.safety_state not in (
            SafetyState.DISARMED,
            SafetyState.ARMED_IDLE,
        ):
            self._state.motion_tuning_synced = False
            self._state.motion_tuning_sync_state = "waiting_for_stop"
            self._state.motion_tuning_sync_error = (
                "Stop motion before applying tuning")
            return False

        self._state.motion_tuning_sync_state = "checking"
        self._state.motion_tuning_sync_error = None
        current = await self.request_motion_tuning()
        if motion_tuning_equal(current, self._desired_motion_tuning):
            self._confirm_motion_tuning(current)
            return True

        self._state.motion_tuning_sync_state = "applying"
        result = await self._transport.send_frame(
            MsgType.SET_MOTION_TUNING,
            self._desired_motion_tuning.pack(),
            expect_ack=True,
        )
        self.last_command_sequence = (
            result if result is not None
            else self._transport.last_command_sequence_attempted
        )
        if result is None:
            self._set_motion_tuning_error(
                self._transport.last_error or "motion tuning ACK timeout")
            return False

        confirmed = await self.request_motion_tuning()
        if confirmed is None:
            self._set_motion_tuning_error(
                "motion tuning report confirmation timeout")
            return False
        self._confirm_motion_tuning(confirmed)
        return self._state.motion_tuning_synced

    async def synchronize_motion_tuning(self) -> bool:
        if not self.config.features.motion_tuning:
            self._set_motion_tuning_error("Motion tuning feature is disabled")
            return False
        async with self._motion_tuning_apply_lock:
            return await self._synchronize_motion_tuning_locked()

    async def ensure_motion_tuning_synced(self) -> bool:
        # Re-read the STM32 before every transition into active body control.
        # A board reset can restore firmware defaults without dropping UART.
        return await self.synchronize_motion_tuning()

    async def set_motion_tuning(
        self,
        tuning: MotionTuning,
        persist: bool = True,
    ) -> bool:
        desired = validate_motion_tuning(clone_motion_tuning(tuning))
        async with self._motion_tuning_apply_lock:
            state = self.refresh_link_state()
            if state.safety_state not in (
                SafetyState.DISARMED,
                SafetyState.ARMED_IDLE,
            ):
                self._state.motion_tuning_sync_state = "waiting_for_stop"
                self._state.motion_tuning_sync_error = (
                    "Stop motion before applying tuning")
                return False
            if persist:
                self._motion_tuning_store.save(desired)
            self._desired_motion_tuning = desired
            self._state.motion_tuning_synced = False
            self._state.motion_tuning_sync_state = "pending"
            self._state.motion_tuning_sync_error = None
            return await self._synchronize_motion_tuning_locked()

    async def enable_body_control(self) -> bool:
        self._state.last_command_error = None
        if not await self.ensure_motion_tuning_synced():
            self._state.last_command_error = (
                self._state.motion_tuning_sync_error
                or "motion tuning is not synchronized")
            return False
        result = await self._transport.send_frame(
            MsgType.BODY_CONTROL_ON, expect_ack=True)
        if result is None:
            self._state.last_command_error = (
                self._transport.last_error or "BODY_CONTROL_ON failed")
            return False
        self.last_command_sequence = result
        report = await self.request_status()
        ok = bool(
            report is not None
            and report.safety_state == SafetyState.ARMED_ACTIVE
            and report.body_control_enabled
        )
        if not ok:
            self._state.last_command_error = (
                "BODY_CONTROL_ON status confirmation timeout")
        return ok

    async def disable_body_control(self) -> bool:
        self._begin_pwm_request([self._neutral_us] * self._channel_count)
        result = await self._transport.send_frame(
            MsgType.BODY_CONTROL_OFF, expect_ack=True)
        if result is None:
            self._fail_pwm_request()
            return False
        self.last_command_sequence = result
        report = await self.request_status()
        pwm_ok = self._confirm_pwm_from_report(report)
        state_ok = self._confirm_safety_state(
            report, (SafetyState.ARMED_IDLE,), "BODY_CONTROL_OFF")
        return pwm_ok and state_ok

    async def send_body_command(self, command: SetBodyCommand) -> bool:
        values = command.values()
        if any(
            not math.isfinite(value) or value < -1.0 or value > 1.0
            for value in values
        ):
            self._state.last_command_error = (
                "body command axes must be finite and within -1.0..1.0")
            return False
        result = await self._transport.send_frame(
            MsgType.SET_BODY_COMMAND,
            command.pack(),
            expect_ack=True,
        )
        self.last_command_sequence = (
            result if result is not None
            else self._transport.last_command_sequence_attempted
        )
        if result is None:
            self._state.last_command_error = (
                self._transport.last_error or "SET_BODY_COMMAND failed")
            return False
        self._state.last_command_error = None
        return True

    async def stop_body_motion(self) -> bool:
        return await self.send_body_command(SetBodyCommand())

    async def force_neutral(self, reason: str, confirm: bool = True,
                            timeout: Optional[float] = None) -> bool:
        """Force all PWM outputs to neutral through the STM32 command path."""
        logger.warning("Force neutral requested: %s", reason)
        if reason == "last_client_disconnected":
            self._neutral_reason_override = NeutralReason.LAST_CLIENT_DISCONNECTED
        operation = self.set_all_neutral(confirm=confirm)
        try:
            ok = await asyncio.wait_for(
                operation,
                timeout if timeout is not None else self._report_request_timeout_s + 1.0,
            )
        except asyncio.TimeoutError:
            self._state.request_state = "timeout"
            self._state.last_command_error = (
                f"force neutral timed out: {reason}")
            ok = False
            logger.error("Force neutral timed out: %s", reason)
        except Exception as exc:
            self._state.request_state = "timeout"
            self._state.last_command_error = str(exc)
            ok = False
            logger.exception("Force neutral failed: %s", reason)
        if not ok:
            self._neutral_reason_override = None
        logger.info("Force neutral result=%s reason=%s", ok, reason)
        return ok

    async def request_status(
        self, timeout: Optional[float] = None
    ) -> Optional[StatusReport]:
        """Request a status report and wait for the response."""
        async with self._status_request_lock:
            return await self._request_status_locked(timeout)

    async def _request_status_locked(
        self, timeout: Optional[float] = None
    ) -> Optional[StatusReport]:
        if not self._transport.connected:
            return None
        future = asyncio.get_running_loop().create_future()

        async def on_status(msg_type, seq, payload):
            if msg_type == MsgType.STATUS_REPORT and not future.done():
                future.set_result(payload)

        self._transport.on_frame(on_status)
        await self._transport.send_frame(MsgType.REQUEST_STATUS)
        try:
            payload = await asyncio.wait_for(
                future,
                timeout if timeout is not None else self._report_request_timeout_s,
            )
            return StatusReport.unpack(payload)
        except (asyncio.TimeoutError, struct.error, ValueError) as exc:
            if not isinstance(exc, asyncio.TimeoutError):
                logger.warning("Invalid STATUS_REPORT payload: %s", exc)
            return None
        finally:
            self._transport.remove_frame_callback(on_status)

    async def request_sensors(self) -> Optional[SensorReport]:
        """Request sensor data and wait for the response."""
        async with self._sensor_request_lock:
            if not self._transport.connected:
                return None
            self._sensor_request_in_flight = True
            try:
                return await self._request_sensors_locked()
            finally:
                self._sensor_request_in_flight = False

    async def _request_sensors_locked(self) -> Optional[SensorReport]:
        future = asyncio.get_running_loop().create_future()

        async def on_sensor(msg_type, seq, payload):
            if msg_type == MsgType.SENSOR_REPORT and not future.done():
                future.set_result(payload)

        self._transport.on_frame(on_sensor)
        await self._transport.send_frame(MsgType.REQUEST_SENSORS)
        try:
            payload = await asyncio.wait_for(
                future, self._report_request_timeout_s)
            return SensorReport.unpack(payload)
        except (asyncio.TimeoutError, struct.error, ValueError) as exc:
            if not isinstance(exc, asyncio.TimeoutError):
                logger.warning("Invalid SENSOR_REPORT payload: %s", exc)
            return None
        finally:
            self._transport.remove_frame_callback(on_sensor)

    async def request_motion_tuning(
        self, timeout: Optional[float] = None
    ) -> Optional[MotionTuning]:
        async with self._motion_tuning_request_lock:
            if not self._transport.connected:
                return None
            future = asyncio.get_running_loop().create_future()

            async def on_tuning(msg_type, seq, payload):
                if (
                    msg_type == MsgType.MOTION_TUNING_REPORT
                    and not future.done()
                ):
                    future.set_result(payload)

            self._transport.on_frame(on_tuning)
            await self._transport.send_frame(MsgType.REQUEST_MOTION_TUNING)
            try:
                payload = await asyncio.wait_for(
                    future,
                    timeout
                    if timeout is not None
                    else self._report_request_timeout_s,
                )
                tuning = MotionTuning.unpack(payload)
                validate_motion_tuning(tuning)
                return tuning
            except (
                asyncio.TimeoutError,
                struct.error,
                ValueError,
                OverflowError,
            ) as exc:
                if not isinstance(exc, asyncio.TimeoutError):
                    logger.warning(
                        "Invalid MOTION_TUNING_REPORT payload: %s", exc)
                return None
            finally:
                self._transport.remove_frame_callback(on_tuning)

    # -------- Frame handler --------

    async def _on_frame(self, msg_type: int, seq: int, payload: bytes):
        """Handle incoming telemetry frames."""
        self._sync_connection_generation()
        emit_event: Optional[tuple] = None
        async with self._lock:
            now_wall = time.time()
            now_mono = time.monotonic()

            if msg_type == MsgType.STATUS_REPORT:
                try:
                    r = StatusReport.unpack(payload)
                except (struct.error, ValueError) as exc:
                    self._transport.rx_errors += 1
                    logger.warning("Invalid STATUS_REPORT payload: %s", exc)
                    return
                self._state.safety_state = r.safety_state
                self._state.flags = r.flags
                self._state.pwm = list(r.pwm)
                self._state.error_count = r.error_count
                self._state.heartbeat_missed = r.heartbeat_missed
                self._state.active_channel = r.active_channel
                if (self._neutral_reason_override is not None
                        and r.neutral_reason == NeutralReason.COMMAND
                        and all(value == self._neutral_us for value in r.pwm)):
                    self._state.neutral_reason = self._neutral_reason_override
                else:
                    self._state.neutral_reason = r.neutral_reason
                    if (r.neutral_reason != NeutralReason.COMMAND
                            or any(value != self._neutral_us for value in r.pwm)):
                        self._neutral_reason_override = None
                self._state.last_status_report_at = now_wall
                self._state._last_status_report_mono = now_mono
                self._state.stm32_online = True
                self._state.serial_connected = self._transport.connected
                if (r.neutral_reason == NeutralReason.PWM_COMMAND_TIMEOUT
                        and all(value == self._neutral_us for value in r.pwm)):
                    self._state.requested_pwm = [self._neutral_us] * self._channel_count
                    self._state.request_state = "idle"
                    self._state.last_command_error = None
                elif (self._state.request_state == "pending"
                      and list(r.pwm) == list(self._state.requested_pwm)):
                    self._state.request_state = "confirmed"
                    self._state.last_command_error = None
                emit_event = ("status", self._state)

            elif msg_type == MsgType.SENSOR_REPORT:
                try:
                    r = SensorReport.unpack(payload)
                except (struct.error, ValueError) as exc:
                    self._transport.rx_errors += 1
                    logger.warning("Invalid SENSOR_REPORT payload: %s", exc)
                    return
                self._state.depth_m = r.depth_m
                self._state.pressure_mbar = r.pressure_mbar
                self._state.water_temp_c = r.water_temp_c
                self._state.yaw = r.yaw
                self._state.pitch = r.pitch
                self._state.roll = r.roll
                self._state.accel = list(r.accel)
                self._state.gyro = list(r.gyro)
                self._state.mag = list(r.mag)
                self._state.last_sensor_report_at = now_wall
                self._state._last_sensor_report_mono = now_mono
                emit_event = ("sensors", self._state)

            elif msg_type == MsgType.MOTION_TUNING_REPORT:
                try:
                    tuning = MotionTuning.unpack(payload)
                    validate_motion_tuning(tuning)
                except (
                    struct.error,
                    ValueError,
                    OverflowError,
                ) as exc:
                    self._transport.rx_errors += 1
                    logger.warning(
                        "Invalid MOTION_TUNING_REPORT payload: %s", exc)
                    return
                self._confirm_motion_tuning(tuning)

            elif msg_type == MsgType.HEARTBEAT_ACK:
                try:
                    r = HeartbeatAck.unpack(payload)
                except (struct.error, ValueError) as exc:
                    self._transport.rx_errors += 1
                    logger.warning("Invalid HEARTBEAT_ACK payload: %s", exc)
                    return
                restarted = (self._last_stm32_uptime_s is not None
                             and r.uptime_s < self._last_stm32_uptime_s)
                self._last_stm32_uptime_s = r.uptime_s
                self._state.safety_state = r.safety_state
                self._state.error_count = r.error_count
                self._state.stm32_online = True
                if restarted:
                    self._state.last_status_report_at = 0.0
                    self._state.last_sensor_report_at = 0.0
                    self._state._last_status_report_mono = 0.0
                    self._state._last_sensor_report_mono = 0.0
                    self._state.requested_pwm = [self._neutral_us] * self._channel_count
                    self._state.request_state = "idle"
                    self._state.last_command_error = None
                    self._state.active_channel = 0xFF
                    self._state.motion_tuning_synced = False
                    self._state.motion_tuning_sync_state = "pending"
                    self._state.motion_tuning_sync_error = None
                    self._confirmed_motion_tuning = None
                    emit_event = ("connection", "stm32_restarted")

            elif msg_type == MsgType.SAFETY_EVENT:
                try:
                    r = SafetyEvent.unpack(payload)
                except (struct.error, ValueError) as exc:
                    self._transport.rx_errors += 1
                    logger.warning("Invalid SAFETY_EVENT payload: %s", exc)
                    return
                logger.warning("Safety event: type=%d reason=%d",
                               r.event_type, r.reason_code)
                emit_event = ("event", msg_type, payload)

        if emit_event is not None:
            await self._emit(emit_event[0], *emit_event[1:])
