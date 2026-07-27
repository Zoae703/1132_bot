import math

import pytest

from gamepad_mapping import MappingConfig, map_gamepad


CONFIG = MappingConfig()
ZERO_AXES = [0.0] * 6
ZERO_BUTTONS = [0] * 4


@pytest.mark.parametrize(
    ("axes", "field", "sign"),
    [
        ([0.0, -1.0, 0.0, 0.0, 0.0, 0.0], "surge", 1),
        ([0.0, 1.0, 0.0, 0.0, 0.0, 0.0], "surge", -1),
        ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sway", 1),
        ([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sway", -1),
        ([0.0, 0.0, 0.0, 0.0, 1.0, 0.0], "yaw", 1),
        ([0.0, 0.0, 0.0, 0.0, -1.0, 0.0], "yaw", -1),
    ],
)
def test_confirmed_frd_axis_directions(axes, field, sign):
    value = getattr(map_gamepad(axes, ZERO_BUTTONS, CONFIG).command, field)
    assert math.copysign(1, value) == sign


def test_y_is_up_a_is_down_and_conflict_is_zero():
    y = map_gamepad(ZERO_AXES, [0, 0, 0, 1], CONFIG)
    a = map_gamepad(ZERO_AXES, [1, 0, 0, 0], CONFIG)
    conflict = map_gamepad(ZERO_AXES, [1, 0, 0, 1], CONFIG)
    assert y.command.heave < 0
    assert a.command.heave > 0
    assert conflict.command.heave == 0
    assert conflict.heave_conflict is True


def test_unused_inputs_never_affect_motion():
    result = map_gamepad(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0, 1, 1, 0],
        CONFIG,
    )
    assert set(result.command.as_dict().values()) == {0.0}


def test_deadzone_is_zero_and_edge_is_continuous():
    center = map_gamepad(
        [0, -CONFIG.deadzone / 2, 0, 0, 0, 0],
        ZERO_BUTTONS,
        CONFIG,
    )
    just_outside = map_gamepad(
        [0, -(CONFIG.deadzone + 1e-6), 0, 0, 0, 0],
        ZERO_BUTTONS,
        CONFIG,
    )
    assert center.command.surge == 0
    assert 0 < just_outside.command.surge < 1e-5


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_axes_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        map_gamepad([value, 0, 0, 0, 0, 0], ZERO_BUTTONS, CONFIG)


def test_inversion_options_are_effective():
    raw = [-1.0, 1.0, 0.0, 0.0, 1.0, 0.0]
    normal = map_gamepad(raw, ZERO_BUTTONS, CONFIG).command
    inverted = map_gamepad(
        raw,
        ZERO_BUTTONS,
        MappingConfig(
            surge_invert=False,
            sway_invert=True,
            yaw_invert=True,
        ),
    ).command
    assert inverted.surge == -normal.surge
    assert inverted.sway == -normal.sway
    assert inverted.yaw == -normal.yaw
