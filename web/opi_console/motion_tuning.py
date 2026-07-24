"""Validation and atomic persistence for six-axis motion tuning."""

from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

from protocol import MotionTuning


AXIS_NAMES = ("surge", "sway", "heave", "roll", "pitch", "yaw")


def validate_motion_tuning(tuning: MotionTuning) -> MotionTuning:
    if len(tuning.axis_gain) != 6 or len(tuning.axis_max_output) != 6:
        raise ValueError("motion tuning requires exactly six axes")
    if any(
        not math.isfinite(value) or value < 0.0 or value > 2.0
        for value in tuning.axis_gain
    ):
        raise ValueError("axis_gain values must be finite and within 0.0..2.0")
    if any(
        not math.isfinite(value) or value < 0.0 or value > 1.0
        for value in tuning.axis_max_output
    ):
        raise ValueError(
            "axis_max_output values must be finite and within 0.0..1.0")
    if (
        not math.isfinite(tuning.global_multiplier)
        or tuning.global_multiplier < 0.0
        or tuning.global_multiplier > 1.0
    ):
        raise ValueError("global_multiplier must be within 0.0..1.0")
    if (
        isinstance(tuning.pwm_slew_rate_us_per_s, bool)
        or not isinstance(tuning.pwm_slew_rate_us_per_s, int)
        or not 100 <= tuning.pwm_slew_rate_us_per_s <= 5000
    ):
        raise ValueError("pwm_slew_rate_us_per_s must be within 100..5000")
    if (
        isinstance(tuning.command_timeout_ms, bool)
        or not isinstance(tuning.command_timeout_ms, int)
        or not 200 <= tuning.command_timeout_ms <= 2000
    ):
        raise ValueError("command_timeout_ms must be within 200..2000")
    return tuning


def clone_motion_tuning(tuning: MotionTuning) -> MotionTuning:
    return MotionTuning(
        axis_gain=list(tuning.axis_gain),
        axis_max_output=list(tuning.axis_max_output),
        global_multiplier=float(tuning.global_multiplier),
        pwm_slew_rate_us_per_s=int(tuning.pwm_slew_rate_us_per_s),
        command_timeout_ms=int(tuning.command_timeout_ms),
    )


def motion_tuning_equal(
    left: MotionTuning | None,
    right: MotionTuning | None,
    tolerance: float = 1e-5,
) -> bool:
    if left is None or right is None:
        return False
    floats_left = (
        list(left.axis_gain)
        + list(left.axis_max_output)
        + [left.global_multiplier]
    )
    floats_right = (
        list(right.axis_gain)
        + list(right.axis_max_output)
        + [right.global_multiplier]
    )
    return (
        all(
            math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance)
            for a, b in zip(floats_left, floats_right)
        )
        and left.pwm_slew_rate_us_per_s == right.pwm_slew_rate_us_per_s
        and left.command_timeout_ms == right.command_timeout_ms
    )


def motion_tuning_from_dict(raw: Any) -> MotionTuning:
    if not isinstance(raw, dict):
        raise ValueError("motion tuning file must contain a JSON object")
    expected = {
        "axis_gain",
        "axis_max_output",
        "global_multiplier",
        "pwm_slew_rate_us_per_s",
        "command_timeout_ms",
    }
    extra = set(raw) - expected
    missing = expected - set(raw)
    if extra or missing:
        raise ValueError(
            "motion tuning keys mismatch: "
            f"missing={sorted(missing)}, extra={sorted(extra)}")
    axis_gain = raw["axis_gain"]
    axis_max_output = raw["axis_max_output"]
    if not isinstance(axis_gain, list) or not isinstance(axis_max_output, list):
        raise ValueError("axis_gain and axis_max_output must be arrays")
    tuning = MotionTuning(
        axis_gain=[float(value) for value in axis_gain],
        axis_max_output=[float(value) for value in axis_max_output],
        global_multiplier=float(raw["global_multiplier"]),
        pwm_slew_rate_us_per_s=raw["pwm_slew_rate_us_per_s"],
        command_timeout_ms=raw["command_timeout_ms"],
    )
    return validate_motion_tuning(tuning)


class MotionTuningStore:
    def __init__(self, path: Path):
        self.path = path

    def load(self) -> MotionTuning:
        if not self.path.exists():
            return MotionTuning()
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"failed to load motion tuning {self.path}: {exc}") from exc
        try:
            return motion_tuning_from_dict(raw)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"invalid motion tuning {self.path}: {exc}") from exc

    def save(self, tuning: MotionTuning) -> None:
        validated = validate_motion_tuning(clone_motion_tuning(tuning))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.",
            suffix=".tmp",
            dir=self.path.parent,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    validated.to_dict(),
                    stream,
                    ensure_ascii=True,
                    indent=2,
                    allow_nan=False,
                    sort_keys=True,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self.path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
