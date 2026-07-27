"""Validated process-wide configuration for the Orange Pi console."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent

STM32_CHANNEL_COUNT = 8
STM32_PWM_NEUTRAL_US = 1500
STM32_PWM_MIN_US = 1000
STM32_PWM_MAX_US = 2000
STM32_MAX_TEST_DURATION_MS = 2000


class StrictConfigModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class SerialConfig(StrictConfigModel):
    port: str = Field(default="/dev/ttyS5", min_length=1)
    baudrate: int = 115200

    @field_validator("baudrate")
    @classmethod
    def validate_firmware_baudrate(cls, value: int):
        if value != 115200:
            raise ValueError("serial.baudrate must match STM32 firmware: 115200")
        return value


class HeartbeatConfig(StrictConfigModel):
    interval_ms: int = Field(default=200, ge=50, le=5000)
    timeout_ms: int = Field(default=1000, ge=200, le=5000)


class ReconnectConfig(StrictConfigModel):
    min_delay_s: float = Field(default=1.0, gt=0, le=60)
    max_delay_s: float = Field(default=30.0, gt=0, le=300)
    max_retries: int = Field(default=0, ge=0)

    @model_validator(mode="after")
    def validate_delays(self):
        if self.max_delay_s < self.min_delay_s:
            raise ValueError("reconnect.max_delay_s must be >= min_delay_s")
        return self


class PwmConfig(StrictConfigModel):
    channel_count: int = STM32_CHANNEL_COUNT
    neutral_us: int = STM32_PWM_NEUTRAL_US
    min_test_us: int = 1450
    max_test_us: int = 1550
    min_absolute_us: int = STM32_PWM_MIN_US
    max_absolute_us: int = STM32_PWM_MAX_US
    min_test_duration_ms: int = 200
    max_test_duration_ms: int = STM32_MAX_TEST_DURATION_MS
    default_timeout_ms: int = 500

    @model_validator(mode="after")
    def validate_safety_limits(self):
        if self.channel_count != STM32_CHANNEL_COUNT:
            raise ValueError(
                f"pwm.channel_count must be {STM32_CHANNEL_COUNT} for protocol v2")
        if self.neutral_us != STM32_PWM_NEUTRAL_US:
            raise ValueError("pwm.neutral_us must remain 1500")
        if not self.min_test_us < self.neutral_us < self.max_test_us:
            raise ValueError(
                "pwm limits must satisfy min_test_us < neutral_us < max_test_us")
        if self.min_absolute_us > self.min_test_us:
            raise ValueError("pwm.min_absolute_us must be <= min_test_us")
        if self.max_absolute_us < self.max_test_us:
            raise ValueError("pwm.max_absolute_us must be >= max_test_us")
        if self.min_absolute_us < STM32_PWM_MIN_US:
            raise ValueError(
                f"pwm.min_absolute_us cannot be below STM32 limit {STM32_PWM_MIN_US}")
        if self.max_absolute_us > STM32_PWM_MAX_US:
            raise ValueError(
                f"pwm.max_absolute_us cannot exceed STM32 limit {STM32_PWM_MAX_US}")
        if self.min_test_duration_ms <= 0:
            raise ValueError("pwm.min_test_duration_ms must be > 0")
        if self.max_test_duration_ms < self.min_test_duration_ms:
            raise ValueError(
                "pwm.max_test_duration_ms must be >= min_test_duration_ms")
        if self.max_test_duration_ms > STM32_MAX_TEST_DURATION_MS:
            raise ValueError(
                "pwm.max_test_duration_ms cannot exceed STM32 limit 2000")
        if not (
            self.min_test_duration_ms
            <= self.default_timeout_ms
            <= self.max_test_duration_ms
        ):
            raise ValueError(
                "pwm.default_timeout_ms must be within the configured duration range")
        return self


class TelemetryConfig(StrictConfigModel):
    status_hz: float = Field(default=5.0, ge=0.5, le=20.0)
    sensors_hz: float = Field(default=5.0, ge=0.5, le=20.0)
    status_stale_timeout_s: float = Field(default=1.0, gt=0, le=60)
    sensors_stale_timeout_s: float = Field(default=1.0, gt=0, le=60)
    stm32_online_timeout_s: float = Field(default=2.0, gt=0, le=60)
    request_timeout_s: float = Field(default=1.0, gt=0, le=10)
    command_confirmation_timeout_s: float = Field(default=0.5, gt=0, le=2)
    command_confirmation_poll_interval_s: float = Field(
        default=0.02, ge=0.005, le=0.2)


class WebConfig(StrictConfigModel):
    host: str = "0.0.0.0"
    port: int = Field(default=8000, ge=1, le=65535)
    static_dir: str = "web_frontend/dist"
    websocket_send_timeout_s: float = Field(default=1.0, gt=0, le=10)
    cors_origins: list[str] = Field(default_factory=list)


class LoggingConfig(StrictConfigModel):
    level: str = "INFO"
    file: str = "logs/opi_console.log"
    event_file: str = "logs/events.log"
    max_bytes: int = Field(default=10 * 1024 * 1024, gt=0)
    backup_count: int = Field(default=5, ge=0)

    @field_validator("level")
    @classmethod
    def validate_level(cls, value: str):
        normalized = value.upper()
        if normalized not in {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}:
            raise ValueError("logging.level is invalid")
        return normalized


class SimulationConfig(StrictConfigModel):
    enabled: bool = False


class FeatureConfig(StrictConfigModel):
    manual_pwm: bool = True
    motor_mapping: bool = True
    motion_tuning: bool = True
    gamepad_control: bool = True
    sensor_stream: bool = True
    emergency_stop: bool = True


class MotorMappingConfig(StrictConfigModel):
    file: str = "config/motor_mapping.json"


class MotionTuningConfig(StrictConfigModel):
    file: str = "config/motion_tuning.json"
    sync_interval_s: float = Field(default=0.5, ge=0.1, le=10)


class GamepadConfig(StrictConfigModel):
    axis_count: int = 6
    min_button_count: int = 4
    max_button_count: int = 32
    send_hz: float = Field(default=50.0, ge=20.0, le=100.0)
    zero_timeout_ms: int = Field(default=300, ge=100, le=1000)
    disconnect_timeout_ms: int = Field(default=1000, ge=300, le=5000)
    deadzone: float = Field(default=0.08, ge=0.0, lt=0.5)
    expo: float = Field(default=1.0, ge=1.0, le=3.0)
    global_scale: float = Field(default=0.15, ge=0.0, le=1.0)
    surge_scale: float = Field(default=1.0, ge=0.0, le=1.0)
    sway_scale: float = Field(default=1.0, ge=0.0, le=1.0)
    heave_scale: float = Field(default=1.0, ge=0.0, le=1.0)
    yaw_scale: float = Field(default=1.0, ge=0.0, le=1.0)
    heave_button_strength: float = Field(default=0.10, ge=0.0, le=1.0)
    # Target-controller signs: forward=-axis0, right=+axis1 and
    # clockwise/right=+axis4. Verify raw values after changing controllers.
    surge_invert: bool = True
    sway_invert: bool = False
    yaw_invert: bool = False

    @model_validator(mode="after")
    def validate_layout_and_timeouts(self):
        if self.axis_count != 6:
            raise ValueError("gamepad.axis_count must remain 6")
        if self.min_button_count < 4:
            raise ValueError("gamepad.min_button_count must be at least 4")
        if self.max_button_count < self.min_button_count:
            raise ValueError(
                "gamepad.max_button_count must be >= min_button_count")
        if self.disconnect_timeout_ms <= self.zero_timeout_ms:
            raise ValueError(
                "gamepad.disconnect_timeout_ms must exceed zero_timeout_ms")
        return self


class SafetyConfig(StrictConfigModel):
    disconnect_command_timeout_s: float = Field(default=1.5, gt=0, le=10)
    process_lock_file: str = "/tmp/1132_bot_console.lock"


class AppConfig(StrictConfigModel):
    protocol_version: int = 2
    serial: SerialConfig = Field(default_factory=SerialConfig)
    heartbeat: HeartbeatConfig = Field(default_factory=HeartbeatConfig)
    reconnect: ReconnectConfig = Field(default_factory=ReconnectConfig)
    pwm: PwmConfig = Field(default_factory=PwmConfig)
    telemetry: TelemetryConfig = Field(default_factory=TelemetryConfig)
    web: WebConfig = Field(default_factory=WebConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    features: FeatureConfig = Field(default_factory=FeatureConfig)
    motor_mapping: MotorMappingConfig = Field(default_factory=MotorMappingConfig)
    motion_tuning: MotionTuningConfig = Field(default_factory=MotionTuningConfig)
    gamepad: GamepadConfig = Field(default_factory=GamepadConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)

    @model_validator(mode="after")
    def validate_protocol(self):
        if self.protocol_version != 2:
            raise ValueError("protocol_version must be 2")
        return self

    def resolve_path(self, value: str | Path) -> Path:
        path = Path(value).expanduser()
        return path if path.is_absolute() else PROJECT_ROOT / path

    def transport_dict(self) -> dict[str, Any]:
        return self.model_dump(mode="python")

    def capabilities_dict(self) -> dict[str, Any]:
        return {
            "protocol_version": self.protocol_version,
            "channel_count": self.pwm.channel_count,
            "pwm": self.pwm.model_dump(),
            "features": self.features.model_dump(),
            "motion_tuning": {
                "axis_order": [
                    "surge", "sway", "heave", "roll", "pitch", "yaw",
                ],
                "gain_min": 0.0,
                "gain_max": 2.0,
                "axis_max_output_min": 0.0,
                "axis_max_output_max": 1.0,
                "global_multiplier_min": 0.0,
                "global_multiplier_max": 1.0,
                "pwm_slew_rate_min_us_per_s": 100,
                "pwm_slew_rate_max_us_per_s": 5000,
                "command_timeout_min_ms": 200,
                "command_timeout_max_ms": 2000,
            },
            "gamepad": self.gamepad.model_dump(),
            "telemetry": {
                "status_hz": self.telemetry.status_hz,
                "sensors_hz": self.telemetry.sensors_hz,
                "status_stale_timeout_s": self.telemetry.status_stale_timeout_s,
                "sensors_stale_timeout_s": self.telemetry.sensors_stale_timeout_s,
            },
            "sensor_poll_hz": self.telemetry.sensors_hz,
        }


def coerce_config(value: AppConfig | dict[str, Any] | None) -> AppConfig:
    if isinstance(value, AppConfig):
        return value
    return AppConfig.model_validate(value or {})


def load_app_config(config_path: str | Path) -> AppConfig:
    path = Path(config_path).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    if not path.exists():
        raise ValueError(f"Configuration file does not exist: {path}")
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Failed to read configuration {path}: {exc}") from exc
    try:
        return AppConfig.model_validate(raw)
    except ValidationError as exc:
        raise ValueError(f"Invalid configuration {path}: {exc}") from exc
