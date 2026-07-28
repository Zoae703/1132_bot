"""Validation and atomic persistence for depth-hold PID tuning."""

from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

from protocol import DepthPidTuning


_FIELDS = (
    "kp",
    "ki",
    "kd",
    "p_limit_us",
    "i_limit_us",
    "d_limit_us",
    "output_limit_us",
)


def validate_depth_pid_tuning(tuning: DepthPidTuning) -> DepthPidTuning:
    values = tuning.values()
    if any(isinstance(value, bool) or not math.isfinite(value)
           for value in values):
        raise ValueError("depth PID tuning values must be finite numbers")
    if not 0.0 <= tuning.kp <= 100.0:
        raise ValueError("kp must be within 0.0..100.0")
    if not 0.0 <= tuning.ki <= 10.0:
        raise ValueError("ki must be within 0.0..10.0")
    if not 0.0 <= tuning.kd <= 100.0:
        raise ValueError("kd must be within 0.0..100.0")
    for name in ("p_limit_us", "i_limit_us", "d_limit_us"):
        if not 0.0 <= getattr(tuning, name) <= 200.0:
            raise ValueError(f"{name} must be within 0.0..200.0")
    if not 0.0 < tuning.output_limit_us <= 200.0:
        raise ValueError("output_limit_us must be within (0.0, 200.0]")
    return tuning


def clone_depth_pid_tuning(tuning: DepthPidTuning) -> DepthPidTuning:
    return DepthPidTuning(
        kp=float(tuning.kp),
        ki=float(tuning.ki),
        kd=float(tuning.kd),
        p_limit_us=float(tuning.p_limit_us),
        i_limit_us=float(tuning.i_limit_us),
        d_limit_us=float(tuning.d_limit_us),
        output_limit_us=float(tuning.output_limit_us),
    )


def depth_pid_tuning_equal(
    left: DepthPidTuning | None,
    right: DepthPidTuning | None,
    tolerance: float = 1e-5,
) -> bool:
    if left is None or right is None:
        return False
    return all(
        math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance)
        for a, b in zip(left.values(), right.values())
    )


def depth_pid_tuning_from_dict(raw: Any) -> DepthPidTuning:
    if not isinstance(raw, dict):
        raise ValueError("depth PID tuning file must contain a JSON object")
    extra = set(raw) - set(_FIELDS)
    missing = set(_FIELDS) - set(raw)
    if extra or missing:
        raise ValueError(
            "depth PID tuning keys mismatch: "
            f"missing={sorted(missing)}, extra={sorted(extra)}")
    if any(isinstance(raw[name], bool) for name in _FIELDS):
        raise ValueError("depth PID tuning values must be numbers, not booleans")
    tuning = DepthPidTuning(**{
        name: float(raw[name])
        for name in _FIELDS
    })
    return validate_depth_pid_tuning(tuning)


class DepthPidTuningStore:
    def __init__(self, path: Path):
        self.path = path

    def load(self) -> DepthPidTuning:
        if not self.path.exists():
            return DepthPidTuning()
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"failed to load depth PID tuning {self.path}: {exc}") from exc
        try:
            return depth_pid_tuning_from_dict(raw)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"invalid depth PID tuning {self.path}: {exc}") from exc

    def save(self, tuning: DepthPidTuning) -> None:
        validated = validate_depth_pid_tuning(
            clone_depth_pid_tuning(tuning))
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
