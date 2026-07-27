"""Exclusive ownership for all operator-driven motor control modes."""

from __future__ import annotations

from enum import Enum
from typing import Optional


class ControlMode(str, Enum):
    IDLE = "IDLE"
    MOTOR_TEST = "MOTOR_TEST"
    WEB_MOTION = "WEB_MOTION"
    GAMEPAD = "GAMEPAD"


class ControlModeConflict(RuntimeError):
    pass


class ControlArbiter:
    """Tracks one active motor-control owner.

    Callers serialize transitions with ``ControlState.lock`` so ownership and
    the corresponding STM32 state transition remain one operation.
    """

    def __init__(self) -> None:
        self._mode = ControlMode.IDLE
        self._owner: Optional[str] = None
        self._generation = 0
        self._last_release_reason: Optional[str] = None

    @property
    def mode(self) -> ControlMode:
        return self._mode

    @property
    def owner(self) -> Optional[str]:
        return self._owner

    def acquire(self, mode: ControlMode, owner: str) -> None:
        if mode == ControlMode.IDLE:
            raise ValueError("IDLE cannot be acquired")
        if self._mode == mode and self._owner == owner:
            return
        if self._mode != ControlMode.IDLE:
            raise ControlModeConflict(
                f"{self._mode.value} is already owned by {self._owner}")
        self._mode = mode
        self._owner = owner
        self._generation += 1
        self._last_release_reason = None

    def require(self, mode: ControlMode, owner: Optional[str] = None) -> None:
        if self._mode != mode:
            raise ControlModeConflict(
                f"{mode.value} required; current mode is {self._mode.value}")
        if owner is not None and self._owner != owner:
            raise ControlModeConflict(
                f"{mode.value} is owned by {self._owner}, not {owner}")

    def release(
        self,
        mode: ControlMode,
        owner: Optional[str] = None,
        reason: str = "released",
    ) -> None:
        self.require(mode, owner)
        self.force_idle(reason)

    def force_idle(self, reason: str) -> None:
        if self._mode != ControlMode.IDLE:
            self._generation += 1
        self._mode = ControlMode.IDLE
        self._owner = None
        self._last_release_reason = reason

    def snapshot(self) -> dict:
        return {
            "mode": self._mode.value,
            "owner": self._owner,
            "generation": self._generation,
            "last_release_reason": self._last_release_reason,
        }
