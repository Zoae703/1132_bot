"""Process-wide coordination for Web control and safety transitions."""

import asyncio
from typing import Optional

from web_backend.control_arbiter import ControlArbiter, ControlMode


class ControlState:
    def __init__(self):
        self.lock = asyncio.Lock()
        self.estop_lock = asyncio.Lock()
        self.arbiter = ControlArbiter()
        self.estop_in_progress = False
        self.motion_inhibit_reason: Optional[str] = None
        self.safety_transition_active = False
        self.safety_transition_reason: Optional[str] = None
        self.active_test_channel: Optional[int] = None
        self.last_pwm_test_at = 0.0

    def begin_safety_transition(self, reason: str) -> bool:
        if self.safety_transition_active:
            return False
        self.safety_transition_active = True
        self.safety_transition_reason = reason
        return True

    def finish_safety_transition(self):
        self.safety_transition_active = False
        self.safety_transition_reason = None
        self.active_test_channel = None

    @property
    def motion_inhibited(self) -> bool:
        return self.motion_inhibit_reason is not None

    def inhibit_motion(self, reason: str):
        self.motion_inhibit_reason = reason

    def clear_motion_inhibit(self):
        self.motion_inhibit_reason = None

    def reconcile_depth_hold(
        self,
        safety_state: int,
        float_enabled: bool,
        link_ready: bool = True,
        reason: str = "firmware_depth_hold_inactive",
    ) -> bool:
        """Release stale ownership after firmware has fail-closed depth hold.

        Command routes hold ``lock`` while transitioning into or out of depth
        hold. Skipping reconciliation during those transitions prevents a
        concurrently broadcast pre-command status from releasing newly
        acquired ownership.
        """
        if self.arbiter.mode != ControlMode.DEPTH_HOLD:
            return False
        if not link_ready:
            self.arbiter.force_idle("depth_hold_link_unready")
            return True
        if self.lock.locked() or (
            int(safety_state) == 2 and float_enabled
        ):
            return False
        self.arbiter.force_idle(reason)
        return True
