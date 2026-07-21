"""
REST API routes for the 1132_bot debug console.

All endpoints validate input and enforce safety constraints before
forwarding commands to the STM32 via Stm32Proxy.
"""

import logging
import asyncio
import time
import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import (
    AliasChoices,
    BaseModel,
    ConfigDict,
    Field,
    model_validator,
)

from opi_console.serial_transport import SerialTransport
from opi_console.stm32_proxy import Stm32Proxy
from opi_console.config import AppConfig, coerce_config
from protocol import SafetyState
from web_backend.ws_manager import WebSocketManager
from web_backend.control_state import ControlState

logger = logging.getLogger("opi_console.api")


# ============================================================================
#  Request/Response models
# ============================================================================

class PwmTestRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    channel: int = Field(strict=True)
    pwm_us: int = Field(strict=True)
    duration_ms: Optional[int] = Field(default=None, strict=True)


class MotorMappingEntry(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="forbid")

    channel: int = Field(..., ge=0, strict=True)
    physical_name: str = Field(
        default="",
        max_length=128,
        validation_alias=AliasChoices("physical_name", "name"),
    )
    direction: str = Field(default="", max_length=64)
    reversed: bool = Field(default=False, strict=True)
    neutral_us: int = Field(default=1500, strict=True)
    safe_min_us: int = Field(
        default=1450,
        strict=True,
        validation_alias=AliasChoices("safe_min_us", "min_us"),
    )
    safe_max_us: int = Field(
        default=1550,
        strict=True,
        validation_alias=AliasChoices("safe_max_us", "max_us"),
    )
    notes: str = Field(default="", max_length=500)

    @model_validator(mode="after")
    def validate_pwm_range(self):
        if self.neutral_us != 1500:
            raise ValueError("motor mapping neutral_us must be 1500")
        if not self.safe_min_us < self.neutral_us < self.safe_max_us:
            raise ValueError(
                "motor mapping must satisfy safe_min_us < neutral_us < safe_max_us")
        return self


class MotorMappingData(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mappings: list[MotorMappingEntry] = Field(default_factory=list)

    @model_validator(mode="after")
    def reject_duplicate_channels(self):
        channels = [entry.channel for entry in self.mappings]
        if len(channels) != len(set(channels)):
            raise ValueError("motor mapping channels must be unique")
        return self


class PwmCapabilitiesResponse(BaseModel):
    neutral_us: int
    min_test_us: int
    max_test_us: int
    min_absolute_us: int
    max_absolute_us: int
    min_test_duration_ms: int
    max_test_duration_ms: int
    default_timeout_ms: int


class FeatureCapabilitiesResponse(BaseModel):
    manual_pwm: bool
    motor_mapping: bool
    sensor_stream: bool
    emergency_stop: bool


class TelemetryCapabilitiesResponse(BaseModel):
    status_hz: float
    sensors_hz: float
    status_stale_timeout_s: float
    sensors_stale_timeout_s: float


class CapabilitiesResponse(BaseModel):
    protocol_version: int
    channel_count: int
    pwm: PwmCapabilitiesResponse
    features: FeatureCapabilitiesResponse
    telemetry: TelemetryCapabilitiesResponse
    sensor_poll_hz: float


# ============================================================================
#  Router factory
# ============================================================================

def create_api_router(
    proxy: Stm32Proxy,
    transport: SerialTransport,
    ws_manager: WebSocketManager,
    control_state: Optional[ControlState] = None,
    config: Optional[AppConfig] = None,
) -> APIRouter:
    router = APIRouter()
    app_config = coerce_config(config or proxy.config)
    pwm_config = app_config.pwm
    control = control_state or ControlState()
    event_log_path = app_config.resolve_path(app_config.logging.event_file)
    mapping_path = app_config.resolve_path(app_config.motor_mapping.file)

    _startup_time = time.time()
    _startup_mono = time.monotonic()
    _recent_errors: list[str] = []

    def _mode() -> str:
        return "SIMULATION" if transport._sim_stm32 else "REAL HARDWARE"

    def _remember_error(message: str):
        _recent_errors.append(f"{time.time():.3f} {message}")
        del _recent_errors[:-20]

    def _online_state():
        state = proxy.refresh_link_state()
        if not transport.connected:
            raise HTTPException(503, "Serial transport is not connected")
        if not state.stm32_online or state.status_stale:
            raise HTTPException(503, "STM32 is offline or telemetry is stale")
        return state

    def _reject_during_safety_transition():
        if control.estop_in_progress:
            raise HTTPException(409, "Emergency stop command is in progress")
        if control.safety_transition_active:
            raise HTTPException(
                409,
                f"Safety transition in progress: {control.safety_transition_reason}",
            )

    def _reject_motion_inhibit():
        if control.motion_inhibited:
            raise HTTPException(
                409,
                "Motion is inhibited after an uncertain safety command: "
                f"{control.motion_inhibit_reason}. Confirm DISARM first.",
            )

    def _command_failure_status() -> int:
        if not transport.connected:
            return 503
        if transport.last_error.startswith("NACK:"):
            return 409
        return 504

    def _failure_reason(default: str) -> str:
        return (
            transport.last_error
            or proxy.refresh_link_state().last_command_error
            or default
        )

    # ---- GET /api/capabilities ----

    @router.get("/capabilities", response_model=CapabilitiesResponse)
    async def capabilities():
        return app_config.capabilities_dict()

    # ---- GET /api/status ----

    @router.get("/status")
    async def get_status():
        state = proxy.refresh_link_state()
        state_dict = state.to_dict()
        return {
            "mode": _mode(),
            **state_dict,
            "tx_frames": transport.tx_frames,
            "rx_frames": transport.rx_frames,
            "rx_errors": transport.rx_errors,
            "crc_errors": transport.crc_errors,
            "ack_timeouts": transport.ack_timeouts,
            "nack_count": transport.nack_count,
            "backend_motion_inhibited": control.motion_inhibited,
            "backend_motion_inhibit_reason": control.motion_inhibit_reason,
            "uptime": time.monotonic() - _startup_mono,
        }

    # ---- GET /api/sensors ----

    @router.get("/sensors")
    async def get_sensors():
        state = proxy.refresh_link_state()
        return {
            **state.sensors_to_dict(),
            "stm32_online": state.stm32_online,
            "serial_connected": transport.connected,
        }

    # ---- POST /api/arm ----

    @router.post("/arm")
    async def arm():
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state == SafetyState.EMERGENCY_STOP:
                raise HTTPException(409, "Cannot ARM: Emergency stop is active. Use /api/reset-estop first.")
            if state.safety_state != SafetyState.DISARMED:
                raise HTTPException(
                    409,
                    f"Cannot ARM from {state.to_dict()['safety_state_name']}",
                )

            ok = await proxy.arm()
            rollback_ok = True
            failure_reason = ""
            failure_status = 504
            if not ok and transport.connected:
                failure_reason = _failure_reason(
                    "ACK or status confirmation timeout")
                failure_status = _command_failure_status()
                rollback_ok = await proxy.disarm()
        if not ok:
            reason = failure_reason or _failure_reason(
                "ACK or status confirmation timeout")
            _remember_error(
                f"ARM failed: {reason}; rollback_disarm={rollback_ok}")
            raise HTTPException(
                failure_status if failure_reason else _command_failure_status(),
                f"ARM command failed: {reason}; "
                f"rollback DISARM={'confirmed' if rollback_ok else 'unconfirmed'}")
        return {"status": "ok", "message": "Armed"}

    # ---- POST /api/disarm ----

    @router.post("/disarm")
    async def disarm():
        async with control.lock:
            ok = await proxy.disarm()
            if ok:
                control.active_test_channel = None
                control.clear_motion_inhibit()
        if not ok:
            reason = _failure_reason("ACK or status confirmation timeout")
            _remember_error(f"DISARM failed: {reason}")
            raise HTTPException(
                _command_failure_status(),
                f"DISARM command failed: {reason}")
        state = proxy.refresh_link_state()
        estop_remains = state.safety_state == SafetyState.EMERGENCY_STOP
        return {
            "status": "ok",
            "message": (
                "Emergency stop remains latched; all PWM neutral"
                if estop_remains else "Disarmed, all PWM neutral"
            ),
            "safety_state": state.to_dict()["safety_state_name"],
        }

    # ---- POST /api/emergency-stop ----

    @router.post("/emergency-stop")
    async def emergency_stop():
        if control.estop_lock.locked():
            raise HTTPException(409, "Emergency stop command is already in progress")
        failure = None
        async with control.estop_lock:
            control.estop_in_progress = True
            control.inhibit_motion("emergency_stop_in_progress")
            try:
                try:
                    ok = await proxy.emergency_stop()
                except Exception:
                    logger.exception("Unexpected ESTOP command failure")
                    ok = False
                if ok:
                    control.active_test_channel = None
                    control.clear_motion_inhibit()
                else:
                    reason = _failure_reason(
                        "ACK or status confirmation timeout")
                    failure_status = _command_failure_status()
                    async with control.lock:
                        neutral_ok = await proxy.force_neutral(
                            "emergency_stop_unconfirmed")
                        disarm_ok = await proxy.disarm()
                        if disarm_ok:
                            control.active_test_channel = None
                            control.clear_motion_inhibit()
                        else:
                            control.inhibit_motion(
                                "emergency_stop_unconfirmed")
                    _remember_error(
                        f"ESTOP failed: {reason}; neutral={neutral_ok}; "
                        f"rollback_disarm={disarm_ok}")
                    failure = (
                        failure_status,
                        f"ESTOP command failed: {reason}; "
                        "forced neutral="
                        f"{'confirmed' if neutral_ok else 'unconfirmed'}; "
                        "rollback DISARM="
                        f"{'confirmed' if disarm_ok else 'unconfirmed'}",
                    )
            finally:
                control.estop_in_progress = False
        if failure is not None:
            raise HTTPException(*failure)
        return {"status": "ok", "message": "EMERGENCY STOP — all PWM neutral, system locked"}

    # ---- POST /api/reset-estop ----

    @router.post("/reset-estop")
    async def reset_estop():
        _reject_during_safety_transition()
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state != SafetyState.EMERGENCY_STOP:
                raise HTTPException(409, "System is not in emergency stop state")
            ok = await proxy.reset_estop()
        if not ok:
            reason = _failure_reason("ACK or status confirmation timeout")
            _remember_error(f"RESET_ESTOP failed: {reason}")
            raise HTTPException(
                _command_failure_status(),
                f"RESET_ESTOP failed: {reason}")
        control.clear_motion_inhibit()
        return {"status": "ok", "message": "ESTOP reset — system DISARMED"}

    # ---- POST /api/pwm/test ----

    @router.post("/pwm/test")
    async def pwm_test(req: PwmTestRequest):
        request_id = uuid.uuid4().hex[:12]
        sequence = None
        command_started = False
        duration_ms = (
            req.duration_ms
            if req.duration_ms is not None
            else pwm_config.default_timeout_ms
        )
        try:
            if not app_config.features.manual_pwm:
                raise HTTPException(403, "Manual PWM feature is disabled")
            _reject_motion_inhibit()
            _reject_during_safety_transition()
            async with control.lock:
                _reject_during_safety_transition()
                state = _online_state()

                if state.safety_state == SafetyState.EMERGENCY_STOP:
                    raise HTTPException(409, "Emergency stop is active")
                if state.safety_state == SafetyState.DISARMED:
                    raise HTTPException(
                        409,
                        "System is DISARMED. ARM first, then enter manual test mode.",
                    )
                if state.safety_state != SafetyState.MANUAL_TEST:
                    raise HTTPException(
                        409,
                        "Must be in MANUAL_TEST mode. Enter manual test mode first.",
                    )

                now = time.monotonic()
                if now - control.last_pwm_test_at < 0.3:
                    raise HTTPException(
                        429, "Rate limited — wait 300ms between PWM commands")

                if req.channel < 0 or req.channel >= pwm_config.channel_count:
                    raise HTTPException(
                        422,
                        f"channel must be 0-{pwm_config.channel_count - 1}",
                    )
                if not pwm_config.min_test_us <= req.pwm_us <= pwm_config.max_test_us:
                    raise HTTPException(
                        422,
                        f"PWM must be {pwm_config.min_test_us}-{pwm_config.max_test_us}us",
                    )
                if not (
                    pwm_config.min_test_duration_ms
                    <= duration_ms
                    <= pwm_config.max_test_duration_ms
                ):
                    raise HTTPException(
                        422,
                        "duration_ms must be "
                        f"{pwm_config.min_test_duration_ms}-"
                        f"{pwm_config.max_test_duration_ms}",
                    )

                # Reconcile process-local ownership after timeout or restart.
                control.active_test_channel = state.to_dict()["active_channel"]
                if (control.active_test_channel is not None
                        and control.active_test_channel != req.channel):
                    neutral_ok = await proxy.force_neutral(
                        f"switch_channel request_id={request_id} "
                        f"previous={control.active_test_channel}")
                    if not neutral_ok:
                        _remember_error(
                            "PWM neutral before switch failed "
                            f"request_id={request_id}: {transport.last_error}")
                        raise HTTPException(
                            _command_failure_status(),
                            "Failed to confirm neutral before switching channels",
                        )
                    control.active_test_channel = None
                    neutral_state = proxy.refresh_link_state()
                    if neutral_state.safety_state == SafetyState.ARMED_IDLE:
                        manual_ok = await proxy.enter_manual()
                        if not manual_ok:
                            rollback_ok = await proxy.disarm()
                            raise HTTPException(
                                _command_failure_status(),
                                "Failed to re-enter MANUAL_TEST after channel "
                                "neutralization; rollback DISARM="
                                f"{'confirmed' if rollback_ok else 'unconfirmed'}",
                            )
                    elif neutral_state.safety_state != SafetyState.MANUAL_TEST:
                        rollback_ok = await proxy.disarm()
                        raise HTTPException(
                            409,
                            "Safety state changed while switching channels; "
                            "rollback DISARM="
                            f"{'confirmed' if rollback_ok else 'unconfirmed'}",
                        )

                command_started = True
                ok = await proxy.set_pwm(
                    req.channel, req.pwm_us, duration_ms)
                sequence = proxy.last_command_sequence
                if not ok:
                    command_error = _failure_reason(
                        "ACK or status confirmation timeout")
                    neutral_ok = await proxy.force_neutral(
                        f"set_pwm_failed request_id={request_id}")
                    control.active_test_channel = None
                    _remember_error(
                        f"SET_PWM failed request_id={request_id}: "
                        f"{command_error}; neutral={neutral_ok}")
                    raise HTTPException(
                        409 if command_error.startswith("NACK:") else 504,
                        f"SET_PWM failed: {command_error}; "
                        f"forced neutral={'confirmed' if neutral_ok else 'unconfirmed'}",
                    )

                post_state = proxy.refresh_link_state()
                interrupted = (
                    control.estop_in_progress
                    or control.motion_inhibited
                    or control.safety_transition_active
                    or post_state.safety_state != SafetyState.MANUAL_TEST
                )
                if interrupted:
                    neutral_ok = await proxy.force_neutral(
                        f"pwm_interrupted request_id={request_id}")
                    control.active_test_channel = None
                    raise HTTPException(
                        409,
                        "PWM command was superseded by a safety transition; "
                        f"forced neutral={'confirmed' if neutral_ok else 'unconfirmed'}",
                    )

                control.last_pwm_test_at = now
                control.active_test_channel = req.channel

            logger.info(
                "PWM test request_id=%s sequence=%s channel=%d pwm_us=%d "
                "duration_ms=%d result=ok reason=ack_and_status_confirmed",
                request_id, sequence, req.channel, req.pwm_us, duration_ms)
            return {
                "status": "ok",
                "request_id": request_id,
                "channel": req.channel,
                "pwm_us": req.pwm_us,
                "duration_ms": duration_ms,
            }
        except asyncio.CancelledError:
            neutral_ok = True
            if command_started:
                transition_started = control.begin_safety_transition(
                    "pwm_request_cancelled")
                try:
                    async with control.lock:
                        neutral_ok = await asyncio.shield(proxy.force_neutral(
                            f"pwm_request_cancelled request_id={request_id}"))
                        control.active_test_channel = None
                finally:
                    if transition_started:
                        control.finish_safety_transition()
            logger.error(
                "PWM test request_id=%s sequence=%s channel=%d pwm_us=%d "
                "duration_ms=%d result=cancelled reason=request_cancelled "
                "forced_neutral=%s",
                request_id, sequence, req.channel, req.pwm_us, duration_ms,
                neutral_ok)
            raise
        except HTTPException as exc:
            headers = dict(exc.headers or {})
            headers["X-Request-ID"] = request_id
            exc.headers = headers
            logger.log(
                logging.ERROR if exc.status_code >= 500 else logging.WARNING,
                "PWM test request_id=%s sequence=%s channel=%d pwm_us=%d "
                "duration_ms=%d result=rejected reason=http_%d:%s",
                request_id, sequence, req.channel, req.pwm_us, duration_ms,
                exc.status_code, exc.detail)
            raise
        except Exception as exc:
            neutral_ok = True
            if command_started:
                transition_started = control.begin_safety_transition(
                    "pwm_unexpected_error")
                try:
                    async with control.lock:
                        neutral_ok = await proxy.force_neutral(
                            f"pwm_unexpected_error request_id={request_id}")
                        control.active_test_channel = None
                finally:
                    if transition_started:
                        control.finish_safety_transition()
            _remember_error(
                f"Unexpected PWM error request_id={request_id}: {exc}")
            logger.exception(
                "PWM test request_id=%s sequence=%s channel=%d pwm_us=%d "
                "duration_ms=%d result=error reason=unexpected "
                "forced_neutral=%s",
                request_id, sequence, req.channel, req.pwm_us, duration_ms,
                neutral_ok)
            raise HTTPException(
                500,
                "Unexpected PWM command failure; forced neutral "
                f"{'confirmed' if neutral_ok else 'unconfirmed'}",
                headers={"X-Request-ID": request_id},
            ) from exc

    # ---- POST /api/pwm/neutral ----

    @router.post("/pwm/neutral")
    async def pwm_neutral():
        async with control.lock:
            ok = await proxy.force_neutral("api_pwm_neutral")
            if ok:
                control.active_test_channel = None
        if not ok:
            reason = _failure_reason("ACK or status confirmation timeout")
            _remember_error(f"PWM neutral failed: {reason}")
            raise HTTPException(
                _command_failure_status(),
                f"Neutral command failed: {reason}")
        return {"status": "ok", "message": "All PWM channels neutral"}

    # ---- POST /api/enter-manual ----

    @router.post("/enter-manual")
    async def enter_manual():
        if not app_config.features.manual_pwm:
            raise HTTPException(403, "Manual PWM feature is disabled")
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state != SafetyState.ARMED_IDLE:
                raise HTTPException(409, "Must be ARMED_IDLE to enter manual test mode")
            ok = await proxy.enter_manual()
            rollback_ok = True
            failure_reason = ""
            failure_status = 504
            if not ok and transport.connected:
                failure_reason = _failure_reason(
                    "ACK or status confirmation timeout")
                failure_status = _command_failure_status()
                await proxy.force_neutral("enter_manual_unconfirmed")
                rollback_ok = await proxy.disarm()
        if not ok:
            reason = failure_reason or _failure_reason(
                "ACK or status confirmation timeout")
            _remember_error(
                f"ENTER_MANUAL failed: {reason}; rollback_disarm={rollback_ok}")
            raise HTTPException(
                failure_status if failure_reason else _command_failure_status(),
                f"ENTER_MANUAL failed: {reason}; "
                f"rollback DISARM={'confirmed' if rollback_ok else 'unconfirmed'}")
        return {"status": "ok", "message": "Manual test mode active — one channel at a time"}

    # ---- POST /api/exit-manual ----

    @router.post("/exit-manual")
    async def exit_manual():
        async with control.lock:
            state = _online_state()
            if state.safety_state != SafetyState.MANUAL_TEST:
                raise HTTPException(409, "System is not in MANUAL_TEST mode")
            ok = await proxy.exit_manual()
            rollback_ok = True
            failure_reason = ""
            failure_status = 504
            if not ok and transport.connected:
                failure_reason = _failure_reason(
                    "ACK or status confirmation timeout")
                failure_status = _command_failure_status()
                await proxy.force_neutral("exit_manual_unconfirmed")
                rollback_ok = await proxy.disarm()
            if ok:
                control.active_test_channel = None
        if not ok:
            reason = failure_reason or _failure_reason(
                "ACK or status confirmation timeout")
            _remember_error(
                f"EXIT_MANUAL failed: {reason}; rollback_disarm={rollback_ok}")
            raise HTTPException(
                failure_status if failure_reason else _command_failure_status(),
                f"EXIT_MANUAL failed: {reason}; "
                f"rollback DISARM={'confirmed' if rollback_ok else 'unconfirmed'}")
        return {"status": "ok"}

    # ---- GET /api/motor-mapping ----

    @router.get("/motor-mapping")
    async def get_motor_mapping():
        if not app_config.features.motor_mapping:
            raise HTTPException(403, "Motor mapping feature is disabled")
        from web_backend.motor_mapping import load_mapping
        return load_mapping(
            mapping_path,
            channel_count=pwm_config.channel_count,
            neutral_us=pwm_config.neutral_us,
            safe_min_us=pwm_config.min_test_us,
            safe_max_us=pwm_config.max_test_us,
            min_absolute_us=pwm_config.min_absolute_us,
            max_absolute_us=pwm_config.max_absolute_us,
        )

    # ---- POST /api/motor-mapping ----

    @router.post("/motor-mapping")
    async def save_motor_mapping(data: MotorMappingData):
        if not app_config.features.motor_mapping:
            raise HTTPException(403, "Motor mapping feature is disabled")
        from web_backend.motor_mapping import save_mapping
        for entry in data.mappings:
            if entry.channel >= pwm_config.channel_count:
                raise HTTPException(
                    422,
                    f"channel must be 0-{pwm_config.channel_count - 1}",
                )
            if (entry.safe_min_us < pwm_config.min_absolute_us
                    or entry.safe_max_us > pwm_config.max_absolute_us):
                raise HTTPException(
                    422,
                    "mapping safe PWM limits exceed configured absolute limits",
                )
        try:
            save_mapping(
                [entry.model_dump() for entry in data.mappings],
                mapping_path,
            )
        except OSError as exc:
            logger.error("Failed to save motor mapping: %s", exc)
            raise HTTPException(500, f"Failed to save motor mapping: {exc}") from exc
        return {"status": "ok", "message": "Mapping saved"}

    # ---- GET /api/logs ----

    @router.get("/logs")
    async def get_logs(lines: int = 100):
        if lines < 1 or lines > 1000:
            raise HTTPException(422, "lines must be 1-1000")
        events = []
        try:
            if event_log_path.exists():
                with event_log_path.open("r", encoding="utf-8", errors="replace") as f:
                    all_lines = f.readlines()
                    events = all_lines[-lines:]
        except Exception as e:
            logger.warning("Failed to read logs: %s", e)
        return {"events": events}

    # ---- GET /api/diagnostics ----

    @router.get("/diagnostics")
    async def diagnostics():
        state = proxy.refresh_link_state()
        tasks = {
            "reader": bool(getattr(transport, "_reader_task_handle", None) and not transport._reader_task_handle.done()),
            "heartbeat": bool(getattr(transport, "_heartbeat_task_handle", None) and not transport._heartbeat_task_handle.done()),
            "reconnect": bool(getattr(transport, "_reconnect_task_handle", None) and not transport._reconnect_task_handle.done()),
            "simulator_tick": bool(getattr(transport, "_sim_tick_task", None) and not transport._sim_tick_task.done()),
            "ws_broadcast": bool(getattr(ws_manager, "_broadcast_task", None) and not ws_manager._broadcast_task.done()),
        }
        return {
            "service_version": "1.0.0",
            "protocol_version": app_config.protocol_version,
            "session_id": ws_manager.session_id,
            "startup_time": _startup_time,
            "uptime_s": time.monotonic() - _startup_mono,
            "mode": _mode(),
            "serial": {
                "connected": transport.connected,
                "port": getattr(transport, "_port", None),
                "baudrate": getattr(transport, "_baudrate", None),
                "last_error": transport.last_error,
                "last_connect_error": transport.last_connect_error,
                "disconnect_count": transport.disconnect_count,
                "reconnect_attempts": transport.reconnect_attempts,
                "connection_generation": transport.connection_generation,
            },
            "stm32": {
                "online": state.stm32_online,
                "status_stale": state.status_stale,
                "sensors_stale": state.sensors_stale,
                "safety_state": state.safety_state,
                "last_update": state.last_update,
            },
            "websocket_client_count": ws_manager.client_count,
            "control": {
                "safety_transition_active": control.safety_transition_active,
                "safety_transition_reason": control.safety_transition_reason,
                "active_test_channel": control.active_test_channel,
                "estop_in_progress": control.estop_in_progress,
                "motion_inhibited": control.motion_inhibited,
                "motion_inhibit_reason": control.motion_inhibit_reason,
                "last_disconnect_result": ws_manager.last_disconnect_safety_result,
            },
            "frames": {
                "tx": transport.tx_frames,
                "rx": transport.rx_frames,
                "rx_errors": transport.rx_errors,
                "crc_errors": transport.crc_errors,
                "ack_timeouts": transport.ack_timeouts,
                "nacks": transport.nack_count,
            },
            "recent_errors": list(_recent_errors),
            "tasks": {
                **tasks,
                "sensor_poll": proxy.sensor_poll_task_running,
                "last_disconnect_safety": bool(
                    getattr(ws_manager, "_last_disconnect_task", None)
                    and not ws_manager._last_disconnect_task.done()),
            },
        }

    return router
