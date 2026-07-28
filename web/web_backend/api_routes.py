"""
REST API routes for the 1132_bot debug console.

All endpoints validate input and enforce safety constraints before
forwarding commands to the STM32 via Stm32Proxy.
"""

import asyncio
import logging
import math
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
from protocol import DepthPidTuning, MotionTuning, SafetyState, SetBodyCommand
from web_backend.ws_manager import WebSocketManager
from web_backend.control_state import ControlState
from web_backend.control_arbiter import (
    ControlMode,
    ControlModeConflict,
)
from web_backend.gamepad_control import (
    GamepadControlError,
    GamepadControlService,
)

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
    motion_tuning: bool
    gamepad_control: bool
    sensor_stream: bool
    emergency_stop: bool
    depth_hold: bool = False


class MotionTuningCapabilitiesResponse(BaseModel):
    axis_order: list[str]
    gain_min: float
    gain_max: float
    axis_max_output_min: float
    axis_max_output_max: float
    global_multiplier_min: float
    global_multiplier_max: float
    pwm_slew_rate_min_us_per_s: int
    pwm_slew_rate_max_us_per_s: int
    command_timeout_min_ms: int
    command_timeout_max_ms: int


class DepthPidCapabilitiesResponse(BaseModel):
    kp_min: float
    kp_max: float
    ki_min: float
    ki_max: float
    kd_min: float
    kd_max: float
    term_limit_min_us: float
    term_limit_max_us: float
    output_limit_min_us: float
    output_limit_max_us: float
    target_depth_min_m: float
    target_depth_max_m: float
    lease_timeout_ms: int


class TelemetryCapabilitiesResponse(BaseModel):
    status_hz: float
    sensors_hz: float
    status_stale_timeout_s: float
    sensors_stale_timeout_s: float


class GamepadCapabilitiesResponse(BaseModel):
    axis_count: int
    min_button_count: int
    max_button_count: int
    send_hz: float
    zero_timeout_ms: int
    disconnect_timeout_ms: int
    deadzone: float
    expo: float
    global_scale: float
    surge_scale: float
    sway_scale: float
    heave_scale: float
    yaw_scale: float
    heave_button_strength: float
    surge_invert: bool
    sway_invert: bool
    yaw_invert: bool


class CapabilitiesResponse(BaseModel):
    protocol_version: int
    channel_count: int
    pwm: PwmCapabilitiesResponse
    features: FeatureCapabilitiesResponse
    motion_tuning: MotionTuningCapabilitiesResponse
    depth_pid: Optional[DepthPidCapabilitiesResponse] = None
    gamepad: GamepadCapabilitiesResponse
    telemetry: TelemetryCapabilitiesResponse
    sensor_poll_hz: float


class MotionTuningRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    axis_gain: list[float] = Field(min_length=6, max_length=6)
    axis_max_output: list[float] = Field(min_length=6, max_length=6)
    global_multiplier: float
    pwm_slew_rate_us_per_s: int = Field(strict=True)
    command_timeout_ms: int = Field(strict=True)


class BodyCommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    surge: float = Field(default=0.0, ge=-1.0, le=1.0)
    sway: float = Field(default=0.0, ge=-1.0, le=1.0)
    heave: float = Field(default=0.0, ge=-1.0, le=1.0)
    roll: float = Field(default=0.0, ge=-1.0, le=1.0)
    pitch: float = Field(default=0.0, ge=-1.0, le=1.0)
    yaw: float = Field(default=0.0, ge=-1.0, le=1.0)


class DepthPidTuningRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    kp: float = Field(ge=0.0, le=100.0)
    ki: float = Field(ge=0.0, le=10.0)
    kd: float = Field(ge=0.0, le=100.0)
    p_limit_us: float = Field(ge=0.0, le=200.0)
    i_limit_us: float = Field(ge=0.0, le=200.0)
    d_limit_us: float = Field(ge=0.0, le=200.0)
    output_limit_us: float = Field(ge=1.0, le=200.0)


class DepthEnableRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    target_depth_m: Optional[float] = Field(default=None, ge=0.0, le=300.0)


class DepthTargetRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    target_depth_m: float = Field(ge=0.0, le=300.0)


# ============================================================================
#  Router factory
# ============================================================================

def create_api_router(
    proxy: Stm32Proxy,
    transport: SerialTransport,
    ws_manager: WebSocketManager,
    control_state: Optional[ControlState] = None,
    config: Optional[AppConfig] = None,
    gamepad_service: Optional[GamepadControlService] = None,
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
        control.reconcile_depth_hold(
            state.safety_state,
            bool(state.flags & 0x02),
            link_ready=bool(
                transport.connected
                and state.stm32_online
                and not state.status_stale
            ),
        )
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

    def _acquire_mode(mode: ControlMode, owner: str = "web"):
        try:
            control.arbiter.acquire(mode, owner)
        except ControlModeConflict as exc:
            raise HTTPException(409, str(exc)) from exc

    def _require_mode(mode: ControlMode, owner: str = "web"):
        try:
            control.arbiter.require(mode, owner)
        except ControlModeConflict as exc:
            raise HTTPException(409, str(exc)) from exc

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

    def _gamepad_service() -> GamepadControlService:
        if gamepad_service is None:
            raise HTTPException(503, "Gamepad control service is unavailable")
        return gamepad_service

    def _depth_feature_enabled() -> bool:
        return bool(getattr(app_config.features, "depth_hold", False))

    def _require_depth_feature():
        if not _depth_feature_enabled():
            raise HTTPException(403, "Depth-hold feature is disabled")

    def _depth_report_value(report, name: str, default=None):
        if isinstance(report, dict):
            return report.get(name, default)
        return getattr(report, name, default)

    def _depth_control_payload(report) -> dict:
        if report is None:
            return {}

        def number(name: str):
            value = _depth_report_value(report, name)
            return (
                float(value)
                if isinstance(value, (int, float)) and math.isfinite(value)
                else None
            )

        def cm_to_m(name: str):
            value = number(name)
            return value / 100.0 if value is not None else None

        return {
            "enabled": bool(_depth_report_value(report, "enabled", False)),
            "depth_sensor_ready": bool(
                _depth_report_value(report, "sensor_ready", False)),
            "depth_sample_valid": bool(
                _depth_report_value(report, "sample_fresh_valid", False)),
            "depth_pid_saturated": bool(
                _depth_report_value(report, "pid_saturated", False)),
            "vertical_saturated": bool(
                _depth_report_value(report, "vertical_saturated", False)),
            "depth_actuator_ready": bool(
                _depth_report_value(report, "actuator_ready", False)),
            "depth_requested_target_m": cm_to_m("requested_target_cm"),
            "depth_active_setpoint_m": cm_to_m("active_setpoint_cm"),
            "depth_measured_m": cm_to_m("measured_depth_cm"),
            "depth_error_m": cm_to_m("error_cm"),
            "depth_pid_p_us": number("p_term_us"),
            "depth_pid_i_us": number("i_term_us"),
            "depth_pid_d_us": number("d_term_us"),
            "depth_pid_output_us": number("output_us"),
            "depth_sample_age_ms": number("sample_age_ms"),
            "fault_reason": _depth_report_value(report, "fault_reason"),
        }

    def _depth_diagnostics_ready(report) -> bool:
        return bool(
            report is not None
            and _depth_report_value(report, "sensor_ready", False)
            and _depth_report_value(report, "sample_fresh_valid", False)
            and _depth_report_value(report, "actuator_ready", False)
        )

    def _depth_measurement_can_seed_target(state) -> bool:
        return bool(
            not state.sensors_stale
            and isinstance(state.depth_m, (int, float))
            and not isinstance(state.depth_m, bool)
            and math.isfinite(state.depth_m)
            and 0.0 <= state.depth_m <= 300.0
        )

    def _depth_state_is_neutral_idle(state) -> bool:
        return bool(
            state.safety_state == SafetyState.ARMED_IDLE
            and not bool(state.flags & 0x02)
            and list(state.pwm) == [pwm_config.neutral_us] * pwm_config.channel_count
        )

    async def _rollback_depth_enable(reason: str) -> dict:
        result = {"disable": False, "neutral": False}
        try:
            result["disable"] = await proxy.disable_depth_hold()
        except Exception:
            logger.exception("Depth-hold rollback disable failed: %s", reason)
        try:
            result["neutral"] = await proxy.force_neutral(
                reason=f"depth_hold_rollback:{reason}",
                confirm=True,
            )
        except Exception:
            logger.exception("Depth-hold rollback neutral failed: %s", reason)
        result["safe"] = bool(
            result["neutral"]
            or (
                result["disable"]
                and _depth_state_is_neutral_idle(
                    proxy.refresh_link_state())
            )
        )
        control.arbiter.force_idle(reason)
        if not result["safe"]:
            control.inhibit_motion(f"{reason}_neutral_unconfirmed")
        return result

    # ---- GET /api/capabilities ----

    @router.get("/capabilities", response_model=CapabilitiesResponse)
    async def capabilities():
        return app_config.capabilities_dict()

    # ---- GET /api/status ----

    @router.get("/status")
    async def get_status():
        state = proxy.refresh_link_state()
        control.reconcile_depth_hold(
            state.safety_state,
            bool(state.flags & 0x02),
            link_ready=bool(
                transport.connected
                and state.stm32_online
                and not state.status_stale
            ),
        )
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
            "control_mode": control.arbiter.mode.value,
            "control_arbiter": control.arbiter.snapshot(),
            "gamepad": (
                gamepad_service.status_snapshot()
                if gamepad_service is not None else None
            ),
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

    # ---- GET/POST /api/gamepad ----

    @router.get("/gamepad/status")
    async def get_gamepad_status():
        return _gamepad_service().status_snapshot()

    @router.post("/gamepad/enable")
    async def enable_gamepad():
        try:
            await _gamepad_service().enable()
        except GamepadControlError as exc:
            raise HTTPException(exc.status_code, exc.detail) from exc
        return {
            "status": "ok",
            "message": "GAMEPAD mode enabled; STM32 remains externally armed",
        }

    @router.post("/gamepad/disable")
    async def disable_gamepad():
        try:
            result = await _gamepad_service().disable()
        except GamepadControlError as exc:
            raise HTTPException(exc.status_code, exc.detail) from exc
        return {
            "status": "ok",
            "message": "GAMEPAD stopped and system disarmed",
            "result": result,
        }

    # ---- GET/POST /api/motion/tuning ----

    @router.get("/motion/tuning")
    async def get_motion_tuning():
        if not app_config.features.motion_tuning:
            raise HTTPException(403, "Motion tuning feature is disabled")
        return proxy.motion_tuning_snapshot()

    @router.post("/motion/tuning")
    async def set_motion_tuning(req: MotionTuningRequest):
        if not app_config.features.motion_tuning:
            raise HTTPException(403, "Motion tuning feature is disabled")
        _reject_during_safety_transition()
        tuning = MotionTuning(
            axis_gain=list(req.axis_gain),
            axis_max_output=list(req.axis_max_output),
            global_multiplier=req.global_multiplier,
            pwm_slew_rate_us_per_s=req.pwm_slew_rate_us_per_s,
            command_timeout_ms=req.command_timeout_ms,
        )
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state not in (
                SafetyState.DISARMED,
                SafetyState.ARMED_IDLE,
            ):
                raise HTTPException(
                    409,
                    "Motion tuning can only be applied while stopped "
                    "(DISARMED or ARMED_IDLE)",
                )
            try:
                ok = await proxy.set_motion_tuning(tuning, persist=True)
            except ValueError as exc:
                raise HTTPException(422, str(exc)) from exc
            except OSError as exc:
                logger.error("Failed to persist motion tuning: %s", exc)
                raise HTTPException(
                    500, f"Failed to persist motion tuning: {exc}") from exc
        if not ok:
            reason = (
                proxy.motion_tuning_snapshot()["sync_error"]
                or _failure_reason("motion tuning confirmation failed")
            )
            raise HTTPException(_command_failure_status(), reason)
        return {
            "status": "ok",
            **proxy.motion_tuning_snapshot(),
        }

    # ---- GET/POST /api/depth ----

    @router.get("/depth/tuning")
    async def get_depth_tuning():
        _require_depth_feature()
        return proxy.depth_pid_tuning_snapshot()

    @router.post("/depth/tuning")
    async def set_depth_tuning(req: DepthPidTuningRequest):
        _require_depth_feature()
        _reject_during_safety_transition()
        tuning = DepthPidTuning(**req.model_dump())
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state not in (
                SafetyState.DISARMED,
                SafetyState.ARMED_IDLE,
            ):
                raise HTTPException(
                    409,
                    "Depth PID tuning can only be applied while stopped "
                    "(DISARMED or ARMED_IDLE)",
                )
            if bool(state.flags & 0x02):
                raise HTTPException(
                    409, "Disable depth hold before applying PID tuning")
            if control.arbiter.mode != ControlMode.IDLE:
                raise HTTPException(
                    409,
                    "Depth PID tuning requires the control arbiter to be IDLE",
                )
            try:
                ok = await proxy.set_depth_pid_tuning(tuning, persist=True)
            except ValueError as exc:
                raise HTTPException(422, str(exc)) from exc
            except OSError as exc:
                logger.error("Failed to persist depth PID tuning: %s", exc)
                raise HTTPException(
                    500, f"Failed to persist depth PID tuning: {exc}") from exc
        if not ok:
            snapshot = proxy.depth_pid_tuning_snapshot()
            reason = (
                snapshot.get("sync_error")
                or _failure_reason("depth PID tuning confirmation failed")
            )
            raise HTTPException(_command_failure_status(), reason)
        return {
            "status": "ok",
            **proxy.depth_pid_tuning_snapshot(),
        }

    @router.get("/depth/control")
    async def get_depth_control():
        _require_depth_feature()
        state = _online_state()
        report = await proxy.request_depth_control()
        if report is None:
            raise HTTPException(
                _command_failure_status(),
                _failure_reason("depth-control report timeout"),
            )
        control.reconcile_depth_hold(
            state.safety_state,
            bool(_depth_report_value(report, "enabled", False)),
            link_ready=bool(
                transport.connected
                and state.stm32_online
                and not state.status_stale
            ),
        )
        return {
            "status": "ok",
            "control_mode": control.arbiter.mode.value,
            "control": _depth_control_payload(report),
        }

    @router.post("/depth/enable")
    async def enable_depth_hold(req: Optional[DepthEnableRequest] = None):
        _require_depth_feature()
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        target_depth_m = req.target_depth_m if req is not None else None
        failure: Optional[tuple[int, str]] = None
        response_report = None
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state != SafetyState.ARMED_IDLE:
                raise HTTPException(
                    409, "Must be ARMED_IDLE to enable depth hold")
            if not _depth_measurement_can_seed_target(state):
                raise HTTPException(
                    409,
                    "Depth sensor telemetry cannot seed a safe 0..300m target",
                )

            diagnostics = await proxy.request_depth_control()
            if diagnostics is None:
                raise HTTPException(
                    _command_failure_status(),
                    _failure_reason("depth-control diagnostics timeout"),
                )
            if not _depth_diagnostics_ready(diagnostics):
                payload = _depth_control_payload(diagnostics)
                raise HTTPException(
                    409,
                    "Depth hold is not ready: "
                    f"sensor_ready={payload['depth_sensor_ready']} "
                    f"sample_valid={payload['depth_sample_valid']} "
                    f"actuator_ready={payload['depth_actuator_ready']} "
                    f"fault={payload['fault_reason']}",
                )

            tuning_ok = await proxy.ensure_depth_pid_tuning_synced()
            if not tuning_ok:
                snapshot = proxy.depth_pid_tuning_snapshot()
                raise HTTPException(
                    _command_failure_status(),
                    snapshot.get("sync_error")
                    or _failure_reason("depth PID tuning is not synchronized"),
                )

            # Synchronization performs serial I/O, so re-check all safety gates
            # immediately before acquiring ownership and enabling thrust.
            state = _online_state()
            if (
                state.safety_state != SafetyState.ARMED_IDLE
                or not _depth_measurement_can_seed_target(state)
            ):
                raise HTTPException(
                    409, "Depth-hold preconditions changed during tuning sync")
            diagnostics = await proxy.request_depth_control()
            if diagnostics is None:
                raise HTTPException(
                    _command_failure_status(),
                    _failure_reason(
                        "depth-control readiness recheck timed out"),
                )
            if not _depth_diagnostics_ready(diagnostics):
                raise HTTPException(
                    409,
                    "Depth-control readiness changed during tuning sync")

            _acquire_mode(ControlMode.DEPTH_HOLD)
            try:
                enabled = await proxy.enable_depth_hold()
                target_ok = True
                if enabled and target_depth_m is not None:
                    target_ok = await proxy.set_depth_cm(
                        target_depth_m * 100.0)
                if not enabled or not target_ok:
                    raise RuntimeError(
                        "FLOAT_ON confirmation failed"
                        if not enabled else "SET_DEPTH confirmation failed")

                response_report = await proxy.request_depth_control()
                if (
                    response_report is None
                    or not _depth_report_value(
                        response_report, "enabled", False)
                ):
                    raise RuntimeError(
                        "depth-control report did not confirm enabled state")
                if target_depth_m is not None:
                    confirmed_cm = _depth_report_value(
                        response_report, "requested_target_cm")
                    if (
                        not isinstance(confirmed_cm, (int, float))
                        or not math.isfinite(confirmed_cm)
                        or not math.isclose(
                            float(confirmed_cm),
                            target_depth_m * 100.0,
                            rel_tol=0.0,
                            abs_tol=0.01,
                        )
                    ):
                        raise RuntimeError(
                            "depth-control report did not confirm target")
            except asyncio.CancelledError:
                await _rollback_depth_enable(
                    "depth_hold_enable_cancelled")
                raise
            except Exception as exc:
                reason = str(exc) or "depth_hold_enable_failed"
                rollback = await _rollback_depth_enable(
                    "depth_hold_enable_failed")
                failure = (
                    _command_failure_status(),
                    f"{reason}; rollback FLOAT_OFF="
                    f"{'confirmed' if rollback['disable'] else 'unconfirmed'}, "
                    "neutral="
                    f"{'confirmed' if rollback['neutral'] else 'unconfirmed'}",
                )

        if failure is not None:
            raise HTTPException(*failure)
        return {
            "status": "ok",
            "message": "Depth hold enabled",
            "control": _depth_control_payload(response_report),
        }

    async def _set_depth_target(
        target_depth_m: float,
        *,
        confirm_report: bool,
        operation: str,
    ) -> dict:
        _require_depth_feature()
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        failure: Optional[tuple[int, str]] = None
        response_report = None
        async with control.lock:
            _reject_during_safety_transition()
            _require_mode(ControlMode.DEPTH_HOLD)
            state = _online_state()
            if (
                state.safety_state != SafetyState.ARMED_ACTIVE
                or not bool(state.flags & 0x02)
            ):
                control.arbiter.force_idle(
                    "firmware_depth_hold_inactive")
                raise HTTPException(409, "Depth hold is not enabled")
            if state.sensors_stale or not math.isfinite(state.depth_m):
                raise HTTPException(
                    409, "Depth sensor telemetry is stale or invalid")

            try:
                ok = await proxy.set_depth_cm(target_depth_m * 100.0)
                if not ok:
                    raise RuntimeError(f"{operation} confirmation failed")
                if confirm_report:
                    response_report = await proxy.request_depth_control()
                    confirmed_cm = _depth_report_value(
                        response_report, "requested_target_cm")
                    if (
                        response_report is None
                        or not _depth_report_value(
                            response_report, "enabled", False)
                        or not isinstance(confirmed_cm, (int, float))
                        or not math.isfinite(confirmed_cm)
                        or not math.isclose(
                            float(confirmed_cm),
                            target_depth_m * 100.0,
                            rel_tol=0.0,
                            abs_tol=0.01,
                        )
                    ):
                        raise RuntimeError(
                            "depth-control report did not confirm target")
            except asyncio.CancelledError:
                await _rollback_depth_enable(
                    f"{operation}_cancelled")
                raise
            except Exception as exc:
                rollback = await _rollback_depth_enable(
                    f"{operation}_failed")
                failure = (
                    _command_failure_status(),
                    f"{exc}; rollback FLOAT_OFF="
                    f"{'confirmed' if rollback['disable'] else 'unconfirmed'}, "
                    "neutral="
                    f"{'confirmed' if rollback['neutral'] else 'unconfirmed'}",
                )

        if failure is not None:
            raise HTTPException(*failure)
        result = {
            "status": "ok",
            "target_depth_m": target_depth_m,
        }
        if response_report is not None:
            result["control"] = _depth_control_payload(response_report)
        return result

    @router.post("/depth/target")
    async def set_depth_target(req: DepthTargetRequest):
        return await _set_depth_target(
            req.target_depth_m,
            confirm_report=True,
            operation="depth_target",
        )

    @router.post("/depth/keepalive")
    async def keepalive_depth_hold(req: DepthTargetRequest):
        return await _set_depth_target(
            req.target_depth_m,
            confirm_report=False,
            operation="depth_keepalive",
        )

    @router.post("/depth/disable")
    async def disable_depth_hold():
        _require_depth_feature()
        _reject_during_safety_transition()
        failure: Optional[tuple[int, str]] = None
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if (
                control.arbiter.mode == ControlMode.IDLE
                and (
                    _depth_state_is_neutral_idle(state)
                    or (
                        state.safety_state in (
                            SafetyState.DISARMED,
                            SafetyState.EMERGENCY_STOP,
                        )
                        and not bool(state.flags & 0x02)
                        and list(state.pwm)
                        == [pwm_config.neutral_us] * pwm_config.channel_count
                    )
                )
            ):
                return {
                    "status": "ok",
                    "message": "Depth hold already disabled and PWM neutral",
                }

            unowned_firmware_active = bool(
                control.arbiter.mode == ControlMode.IDLE
                and state.safety_state == SafetyState.ARMED_ACTIVE
                and bool(state.flags & 0x02)
            )
            if not unowned_firmware_active:
                _require_mode(ControlMode.DEPTH_HOLD)
            if _depth_state_is_neutral_idle(state):
                control.arbiter.release(
                    ControlMode.DEPTH_HOLD,
                    "web",
                    "depth_hold_already_safe",
                )
                return {
                    "status": "ok",
                    "message": "Depth hold already disabled and PWM neutral",
                }

            try:
                command_ok = await proxy.disable_depth_hold()
            except Exception:
                logger.exception("Unexpected FLOAT_OFF failure")
                command_ok = False
            state = proxy.refresh_link_state()
            confirmed = command_ok and _depth_state_is_neutral_idle(state)

            if not confirmed:
                neutral_ok = False
                try:
                    neutral_ok = await proxy.force_neutral(
                        reason="depth_hold_disable_unconfirmed",
                        confirm=True,
                    )
                except Exception:
                    logger.exception(
                        "Force neutral failed after unconfirmed FLOAT_OFF")
                state = proxy.refresh_link_state()
                confirmed = (
                    neutral_ok and _depth_state_is_neutral_idle(state))

            if confirmed:
                if control.arbiter.mode == ControlMode.DEPTH_HOLD:
                    control.arbiter.release(
                        ControlMode.DEPTH_HOLD,
                        "web",
                        "depth_hold_disabled",
                    )
                else:
                    control.arbiter.force_idle(
                        "unowned_depth_hold_disabled")
            else:
                control.inhibit_motion(
                    "depth_hold_disable_unconfirmed")
                failure = (
                    _command_failure_status(),
                    _failure_reason(
                        "FLOAT_OFF did not confirm ARMED_IDLE, float disabled, "
                        "and all PWM neutral"),
                )

        if failure is not None:
            raise HTTPException(*failure)
        return {
            "status": "ok",
            "message": "Depth hold disabled and PWM neutral",
        }

    # ---- POST /api/motion/enable ----

    @router.post("/motion/enable")
    async def enable_motion_control():
        if not app_config.features.motion_tuning:
            raise HTTPException(403, "Motion tuning feature is disabled")
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        async with control.lock:
            _reject_during_safety_transition()
            state = _online_state()
            if state.safety_state != SafetyState.ARMED_IDLE:
                raise HTTPException(
                    409, "Must be ARMED_IDLE to enable six-axis control")
            _acquire_mode(ControlMode.WEB_MOTION)
            ok = await proxy.enable_body_control()
            if not ok:
                control.arbiter.force_idle("web_motion_enable_failed")
                await proxy.disarm()
        if not ok:
            reason = _failure_reason(
                proxy.motion_tuning_snapshot()["sync_error"]
                or "six-axis control confirmation failed")
            raise HTTPException(_command_failure_status(), reason)
        return {"status": "ok", "message": "Six-axis control enabled"}

    # ---- POST /api/motion/command ----

    @router.post("/motion/command")
    async def send_motion_command(req: BodyCommandRequest):
        if not app_config.features.motion_tuning:
            raise HTTPException(403, "Motion tuning feature is disabled")
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        command = SetBodyCommand(**req.model_dump())
        async with control.lock:
            _reject_during_safety_transition()
            _require_mode(ControlMode.WEB_MOTION)
            state = _online_state()
            if (
                state.safety_state != SafetyState.ARMED_ACTIVE
                or not bool(state.flags & 0x20)
            ):
                raise HTTPException(409, "Six-axis control is not enabled")
            ok = await proxy.send_body_command(command)
        if not ok:
            reason = _failure_reason("six-axis command ACK timeout")
            raise HTTPException(_command_failure_status(), reason)
        return {"status": "ok"}

    # ---- POST /api/motion/stop ----

    @router.post("/motion/stop")
    async def stop_motion():
        async with control.lock:
            if control.arbiter.mode != ControlMode.IDLE:
                _require_mode(ControlMode.WEB_MOTION)
            state = _online_state()
            if (
                state.safety_state == SafetyState.ARMED_ACTIVE
                and bool(state.flags & 0x20)
            ):
                ok = await proxy.stop_body_motion()
            else:
                ok = state.safety_state in (
                    SafetyState.DISARMED,
                    SafetyState.ARMED_IDLE,
                    SafetyState.EMERGENCY_STOP,
                )
        if not ok:
            reason = _failure_reason("six-axis stop ACK timeout")
            raise HTTPException(_command_failure_status(), reason)
        return {"status": "ok", "message": "Six-axis command is zero"}

    # ---- POST /api/motion/disable ----

    @router.post("/motion/disable")
    async def disable_motion_control():
        async with control.lock:
            if control.arbiter.mode != ControlMode.IDLE:
                _require_mode(ControlMode.WEB_MOTION)
            state = _online_state()
            if state.safety_state == SafetyState.ARMED_IDLE:
                if control.arbiter.mode == ControlMode.WEB_MOTION:
                    control.arbiter.release(
                        ControlMode.WEB_MOTION,
                        "web",
                        "web_motion_already_disabled",
                    )
                return {
                    "status": "ok",
                    "message": "Six-axis control already disabled",
                }
            if (
                state.safety_state != SafetyState.ARMED_ACTIVE
                or not bool(state.flags & 0x20)
            ):
                raise HTTPException(
                    409, "Six-axis control is not enabled")
            ok = await proxy.disable_body_control()
            if ok:
                control.arbiter.release(
                    ControlMode.WEB_MOTION,
                    "web",
                    "web_motion_disabled",
                )
        if not ok:
            reason = _failure_reason(
                "six-axis disable confirmation failed")
            raise HTTPException(_command_failure_status(), reason)
        return {
            "status": "ok",
            "message": "Six-axis control disabled and PWM neutral",
        }

    # ---- POST /api/arm ----

    @router.post("/arm")
    async def arm():
        _reject_motion_inhibit()
        _reject_during_safety_transition()
        async with control.lock:
            _reject_during_safety_transition()
            if control.arbiter.mode != ControlMode.IDLE:
                raise HTTPException(
                    409,
                    "Cannot ARM while control mode "
                    f"{control.arbiter.mode.value} is active",
                )
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
                control.arbiter.force_idle("api_disarm")
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
            control.arbiter.force_idle("emergency_stop")
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
        control.arbiter.force_idle("reset_estop")
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
                _require_mode(ControlMode.MOTOR_TEST)

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
                control.arbiter.force_idle("api_pwm_neutral")
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
            _acquire_mode(ControlMode.MOTOR_TEST)
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
                control.arbiter.force_idle("enter_manual_failed")
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
            _require_mode(ControlMode.MOTOR_TEST)
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
                if rollback_ok:
                    control.arbiter.force_idle(
                        "exit_manual_failed_disarmed")
            if ok:
                control.active_test_channel = None
                control.arbiter.release(
                    ControlMode.MOTOR_TEST,
                    "web",
                    "manual_test_exited",
                )
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
            "motion_tuning_sync": proxy.motion_tuning_sync_task_running,
            "gamepad_command_pump": bool(
                gamepad_service and gamepad_service.task_running),
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
                "motion_tuning": proxy.motion_tuning_snapshot(),
                "arbiter": control.arbiter.snapshot(),
                "gamepad": (
                    gamepad_service.status_snapshot()
                    if gamepad_service is not None else None
                ),
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
