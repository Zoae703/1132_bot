"""Pure Linux gamepad to FRD BodyCommand mapping."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from opi_console.config import GamepadConfig
from protocol import SetBodyCommand

_SURGE_AXIS = 1
_SWAY_AXIS = 0
_YAW_AXIS = 4


@dataclass(frozen=True)
class GamepadMappingResult:
    command: SetBodyCommand
    heave_conflict: bool


def _finite_axis(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("gamepad axis must be numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError("gamepad axis must be finite")
    return max(-1.0, min(1.0, converted))


def shape_axis(
    raw_value: float,
    *,
    deadzone: float,
    expo: float,
    invert: bool,
) -> float:
    """Clamp, remove the center deadzone, then continuously remap to [-1, 1]."""
    value = _finite_axis(raw_value)
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    normalized = (magnitude - deadzone) / (1.0 - deadzone)
    shaped = math.copysign(normalized ** expo, value)
    return -shaped if invert else shaped


def map_gamepad_state(
    axes: Sequence[float],
    buttons: Sequence[int],
    config: GamepadConfig,
) -> GamepadMappingResult:
    if len(axes) != config.axis_count:
        raise ValueError(
            f"expected {config.axis_count} axes, received {len(axes)}")
    if not config.min_button_count <= len(buttons) <= config.max_button_count:
        raise ValueError(
            "button count must be within "
            f"{config.min_button_count}..{config.max_button_count}")

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
    heave_conflict = a_pressed and y_pressed
    if heave_conflict or (not a_pressed and not y_pressed):
        heave = 0.0
    else:
        direction = 1.0 if a_pressed else -1.0
        heave = (
            direction
            * config.heave_button_strength
            * config.heave_scale
            * config.global_scale
        )

    return GamepadMappingResult(
        command=SetBodyCommand(
            surge=surge,
            sway=sway,
            heave=heave,
            roll=0.0,
            pitch=0.0,
            yaw=yaw,
        ),
        heave_conflict=heave_conflict,
    )
