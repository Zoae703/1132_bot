"""Pure gamepad-to-FRD command mapping used by the Linux forwarder."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Mapping, Sequence

_SURGE_AXIS = 1
_SWAY_AXIS = 0
_YAW_AXIS = 4


@dataclass(frozen=True)
class MappingConfig:
    axis_count: int = 6
    min_button_count: int = 4
    max_button_count: int = 32
    deadzone: float = 0.08
    expo: float = 1.0
    global_scale: float = 0.15
    surge_scale: float = 1.0
    sway_scale: float = 1.0
    heave_scale: float = 1.0
    yaw_scale: float = 1.0
    heave_button_strength: float = 0.10
    surge_invert: bool = True
    sway_invert: bool = False
    yaw_invert: bool = False

    @classmethod
    def from_server(cls, data: Mapping[str, Any]) -> "MappingConfig":
        allowed = {item.name for item in fields(cls)}
        values = {key: data[key] for key in allowed if key in data}
        config = cls(**values)
        config.validate()
        return config

    def validate(self) -> None:
        if self.axis_count != 6:
            raise ValueError("axis_count must be 6")
        if self.min_button_count < 4:
            raise ValueError("min_button_count must be at least 4")
        if self.max_button_count < self.min_button_count:
            raise ValueError("invalid button count limits")
        if not 0.0 <= self.deadzone < 0.5:
            raise ValueError("deadzone must be within 0..0.5")
        if not 1.0 <= self.expo <= 3.0:
            raise ValueError("expo must be within 1..3")
        for name in (
            "global_scale",
            "surge_scale",
            "sway_scale",
            "heave_scale",
            "yaw_scale",
            "heave_button_strength",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and within 0..1")


@dataclass(frozen=True)
class MappedCommand:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {
            "surge": self.surge,
            "sway": self.sway,
            "heave": self.heave,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }


@dataclass(frozen=True)
class MappingResult:
    command: MappedCommand
    heave_conflict: bool


def finite_axis(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("axis must be numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError("axis must be finite")
    return max(-1.0, min(1.0, converted))


def shape_axis(
    value: float,
    *,
    deadzone: float,
    expo: float,
    invert: bool,
) -> float:
    raw = finite_axis(value)
    magnitude = abs(raw)
    if magnitude <= deadzone:
        return 0.0
    normalized = (magnitude - deadzone) / (1.0 - deadzone)
    shaped = math.copysign(normalized ** expo, raw)
    return -shaped if invert else shaped


def map_gamepad(
    axes: Sequence[float],
    buttons: Sequence[int],
    config: MappingConfig,
) -> MappingResult:
    config.validate()
    if len(axes) != config.axis_count:
        raise ValueError(f"expected {config.axis_count} axes")
    if not config.min_button_count <= len(buttons) <= config.max_button_count:
        raise ValueError("button count is incompatible with the server")
    if any(
        isinstance(value, bool)
        or not isinstance(value, int)
        or value not in (0, 1)
        for value in buttons
    ):
        raise ValueError("buttons must contain integer 0 or 1")

    surge = shape_axis(
        axes[_SURGE_AXIS],
        deadzone=config.deadzone,
        expo=config.expo,
        invert=config.surge_invert,
    ) * config.surge_scale * config.global_scale
    sway = shape_axis(
        axes[_SWAY_AXIS],
        deadzone=config.deadzone,
        expo=config.expo,
        invert=config.sway_invert,
    ) * config.sway_scale * config.global_scale
    yaw = shape_axis(
        axes[_YAW_AXIS],
        deadzone=config.deadzone,
        expo=config.expo,
        invert=config.yaw_invert,
    ) * config.yaw_scale * config.global_scale

    a_pressed = buttons[0] == 1
    y_pressed = buttons[3] == 1
    conflict = a_pressed and y_pressed
    if conflict or (not a_pressed and not y_pressed):
        heave = 0.0
    else:
        heave = (
            (1.0 if a_pressed else -1.0)
            * config.heave_button_strength
            * config.heave_scale
            * config.global_scale
        )

    return MappingResult(
        command=MappedCommand(
            surge=surge,
            sway=sway,
            heave=heave,
            roll=0.0,
            pitch=0.0,
            yaw=yaw,
        ),
        heave_conflict=conflict,
    )
