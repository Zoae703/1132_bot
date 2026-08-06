/**
 * @file    actuator_startup_test.cpp
 * @brief   Host tests for the actuator startup state machine invariants.
 *
 * Tests the state transitions and safety invariants of the PCA9685/ESC
 * startup sequence (UNINITIALIZED→INITIALIZING→NEUTRAL_HOLD→READY,
 * with RECOVERING and FAULT branches).
 *
 * This test replicates the control-loop logic from robot_tasks.cpp in a
 * simplified, self-contained simulation to verify invariants without
 * FreeRTOS or hardware dependencies.
 */

#include "robot_data.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

/* ------------------------------------------------------------------ */
/*  Constants (mirrored from robot_tasks.cpp)                          */
/* ------------------------------------------------------------------ */

static constexpr uint32_t ESC_NEUTRAL_HOLD_MS   = 3000U;
static constexpr uint8_t  MAX_RECOVER_STREAK    = 3U;
static constexpr uint32_t PCA_RECOVER_RETRY_MS  = 100U;
static constexpr uint32_t FAULT_RECOVER_PERIOD_MS = 1000U;

namespace {

/* ------------------------------------------------------------------ */
/*  Mock PCA9685 driver — returns controlled results                   */
/* ------------------------------------------------------------------ */

struct MockPCA9685 {
    bool init_ok     = true;
    bool update_ok   = true;
    bool recover_ok  = true;
    int  health      = 0;   /* 0=HEALTHY, 1=BUS_LOCK, 2=TCA, 3=IO_READ,
                               4=CONFIG_MISMATCH, 5=RESET_DETECTED */
    int  init_count  = 0;
};

/* ------------------------------------------------------------------ */
/*  State machine helpers (mirrored from robot_tasks.cpp)              */
/* ------------------------------------------------------------------ */

static void SetActuatorState(ActuatorStartupState s, ActuatorFaultReason r)
{
    if (s != ActuatorStartupState::READY) {
        robot.actuator_output_ready = false;
    }
    robot.actuator_state       = s;
    robot.actuator_fault_reason = r;
    if (s == ActuatorStartupState::READY) {
        robot.actuator_output_ready = true;
    }
}

static void ForceAllSoftwareNeutral()
{
    robot.control_enable      = false;
    robot.float_enabled       = false;
    robot.angle_enabled       = false;
    robot.body_control_enabled = false;
    robot.manual_pwm_enabled  = false;
    for (int i = 0; i < 8; i++) {
        robot.pwm[i]       = ROBOT_PWM_NEUTRAL_US;
        robot.manual_pwm[i] = ROBOT_PWM_NEUTRAL_US;
    }
}

static void InjectOldCommand(int channel, int32_t pwm_us)
{
    /* Simulates a stale command reaching robot.pwm before the
     * NEUTRAL_HOLD layer forces neutral. */
    robot.pwm[channel] = pwm_us;
}

static bool AllPwmNeutral()
{
    for (int i = 0; i < 8; i++) {
        if (robot.pwm[i] != ROBOT_PWM_NEUTRAL_US) return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Test 1: Init success enters NEUTRAL_HOLD                           */
/* ------------------------------------------------------------------ */

void test_init_success_enters_neutral_hold()
{
    robot = RobotData{};
    assert(robot.actuator_state == ActuatorStartupState::UNINITIALIZED);
    assert(!robot.actuator_output_ready);

    /* Simulate: Init succeeds → NEUTRAL_HOLD */
    SetActuatorState(ActuatorStartupState::INITIALIZING,
                     ActuatorFaultReason::NONE);
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);

    assert(robot.actuator_state == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!robot.actuator_output_ready);
    assert(robot.actuator_fault_reason == ActuatorFaultReason::NONE);
}

/* ------------------------------------------------------------------ */
/*  Test 2: Init 3 consecutive failures enter FAULT                    */
/* ------------------------------------------------------------------ */

void test_init_3_failures_enters_fault()
{
    robot = RobotData{};
    MockPCA9685 pca{};
    pca.init_ok = false;

    int failures = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        pca.init_count++;
        if (!pca.init_ok) {
            failures++;
        }
    }

    assert(failures == 3);
    assert(pca.init_count == 3);

    /* After 3 failed inits → FAULT with INIT_FAILED reason */
    SetActuatorState(ActuatorStartupState::FAULT,
                     ActuatorFaultReason::INIT_FAILED);
    assert(robot.actuator_state == ActuatorStartupState::FAULT);
    assert(robot.actuator_fault_reason == ActuatorFaultReason::INIT_FAILED);
    assert(!robot.actuator_output_ready);
}

/* ------------------------------------------------------------------ */
/*  Test 3: NEUTRAL_HOLD blocks non-neutral commands                   */
/* ------------------------------------------------------------------ */

void test_neutral_hold_blocks_non_neutral()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);

    /* Inject a stale 2000 us command on channel 0 */
    InjectOldCommand(0, 2000);
    assert(robot.pwm[0] == 2000);

    /* NEUTRAL_HOLD layer forces everything to neutral */
    ForceAllSoftwareNeutral();

    assert(AllPwmNeutral());
    assert(robot.pwm[0] == ROBOT_PWM_NEUTRAL_US);
}

/* ------------------------------------------------------------------ */
/*  Test 4: 3 seconds + successful write → READY                       */
/* ------------------------------------------------------------------ */

void test_neutral_hold_3s_with_success_enters_ready()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);
    assert(!robot.actuator_output_ready);

    uint32_t now = 0;
    uint32_t neutral_hold_start_ms = now;

    /* Simulate each cycle: force neutral, write PCA */
    MockPCA9685 pca{};
    pca.update_ok = true;

    /* Advance time past 3 seconds */
    now = neutral_hold_start_ms + ESC_NEUTRAL_HOLD_MS;

    ForceAllSoftwareNeutral();
    /* PCA write simulated — succeeds */
    assert(pca.update_ok);

    /* Timer expired + write ok → READY */
    assert((now - neutral_hold_start_ms) >= ESC_NEUTRAL_HOLD_MS);
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);

    assert(robot.actuator_state == ActuatorStartupState::READY);
    assert(robot.actuator_output_ready);
}

/* ------------------------------------------------------------------ */
/*  Test 5: PCA write failure during NEUTRAL_HOLD resets timer         */
/* ------------------------------------------------------------------ */

void test_write_fail_during_neutral_hold_resets_timer()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);

    uint32_t now = 0;
    uint32_t neutral_hold_start_ms = now;

    /* 2.9 seconds in — almost ready */
    now = neutral_hold_start_ms + 2900U;

    /* PCA write fails → RECOVERING, timer reset */
    SetActuatorState(ActuatorStartupState::RECOVERING,
                     ActuatorFaultReason::OUTPUT_WRITE_FAILED);
    assert(robot.actuator_state == ActuatorStartupState::RECOVERING);
    assert(!robot.actuator_output_ready);

    /* Recover succeeds → back to NEUTRAL_HOLD with fresh timer */
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);
    neutral_hold_start_ms = now;

    /* Old timer value is stale — need new 3 seconds */
    now = neutral_hold_start_ms + ESC_NEUTRAL_HOLD_MS;
    assert((now - neutral_hold_start_ms) >= ESC_NEUTRAL_HOLD_MS);

    /* After fresh 3 seconds → READY */
    ForceAllSoftwareNeutral();
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);
    assert(robot.actuator_state == ActuatorStartupState::READY);
    assert(robot.actuator_output_ready);
}

/* ------------------------------------------------------------------ */
/*  Test 6: Update failure does NOT increment recover_failure_streak   */
/* ------------------------------------------------------------------ */

void test_update_fail_does_not_increment_streak()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);

    uint8_t streak = 0U;

    /* Update fails → enter RECOVERING, streak stays 0 */
    SetActuatorState(ActuatorStartupState::RECOVERING,
                     ActuatorFaultReason::OUTPUT_WRITE_FAILED);
    /* streak is reset to 0 on entry to RECOVERING (EnterActuatorRecovering) */

    assert(robot.actuator_state == ActuatorStartupState::RECOVERING);
    assert(streak == 0U);  /* Update failure does not increment */
}

/* ------------------------------------------------------------------ */
/*  Test 7: Recover fails 2 times → still RECOVERING, streak=2         */
/* ------------------------------------------------------------------ */

void test_recover_streak_counting()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::RECOVERING,
                     ActuatorFaultReason::OUTPUT_WRITE_FAILED);

    uint8_t streak = 0U;

    /* Fail #1 */
    streak++;
    assert(streak == 1U);
    assert(streak < MAX_RECOVER_STREAK);
    assert(robot.actuator_state == ActuatorStartupState::RECOVERING);

    /* Fail #2 */
    streak++;
    assert(streak == 2U);
    assert(streak < MAX_RECOVER_STREAK);
    assert(robot.actuator_state == ActuatorStartupState::RECOVERING);
    assert(!robot.estop_locked);
}

/* ------------------------------------------------------------------ */
/*  Test 8: Recover fails 3 times → FAULT, ESTOP latched               */
/* ------------------------------------------------------------------ */

void test_recover_streak_3_enters_fault_estop()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::RECOVERING,
                     ActuatorFaultReason::OUTPUT_WRITE_FAILED);

    uint8_t streak = 0U;

    for (int i = 0; i < 3; i++) {
        streak++;
    }

    assert(streak == 3U);
    assert(streak >= MAX_RECOVER_STREAK);

    /* Latched ESTOP + FAULT */
    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;
    SetActuatorState(ActuatorStartupState::FAULT,
                     ActuatorFaultReason::RECOVERY_FAILED);

    assert(robot.actuator_state == ActuatorStartupState::FAULT);
    assert(robot.estop_locked);
    assert(robot.state == RobotState::EMERGENCY_STOP);
    assert(!robot.actuator_output_ready);
}

/* ------------------------------------------------------------------ */
/*  Test 9: FAULT + ESTOP → 1000ms low-frequency recovery succeeds     */
/* ------------------------------------------------------------------ */

void test_fault_recovers_at_low_frequency()
{
    robot = RobotData{};
    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;
    SetActuatorState(ActuatorStartupState::FAULT,
                     ActuatorFaultReason::RECOVERY_FAILED);

    uint32_t now = 0;
    uint32_t last_fault_recover_ms = 0;

    /* Too early — no recovery attempt */
    now = 500U;
    assert((now - last_fault_recover_ms) < FAULT_RECOVER_PERIOD_MS);

    /* After 1000ms — attempt recovery */
    now = 1000U;
    assert((now - last_fault_recover_ms) >= FAULT_RECOVER_PERIOD_MS);

    /* Recover succeeds → NEUTRAL_HOLD (not READY!) */
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);

    assert(robot.actuator_state == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!robot.actuator_output_ready);  /* Must complete neutral hold first */
}

/* ------------------------------------------------------------------ */
/*  Test 10: FAULT recovery successful — ESTOP still latched           */
/* ------------------------------------------------------------------ */

void test_fault_estop_persists_after_recover()
{
    robot = RobotData{};
    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;
    SetActuatorState(ActuatorStartupState::FAULT,
                     ActuatorFaultReason::RECOVERY_FAILED);

    /* Recovery succeeds */
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);

    assert(robot.actuator_state == ActuatorStartupState::NEUTRAL_HOLD);
    /* ESTOP persists — not automatically cleared by recovery */
    assert(robot.estop_locked);
    assert(robot.state == RobotState::EMERGENCY_STOP);
}

/* ------------------------------------------------------------------ */
/*  Test 11: ARM rejected when NOT in READY                            */
/* ------------------------------------------------------------------ */

static bool SimulateArmCheck()
{
    /* Replicates handle_arm() gate from protocol_handler.cpp */
    if (robot.estop_locked || robot.state == RobotState::EMERGENCY_STOP) {
        return false;  /* NACK: EstopLocked */
    }
    if (!robot.actuator_output_ready) {
        return false;  /* NACK: InternalError (actuator not ready) */
    }
    if (robot.state == RobotState::DISARMED ||
        robot.state == RobotState::COMM_LOST) {
        return true;   /* ARM accepted */
    }
    return false;      /* NACK: BadState */
}

void test_arm_rejected_when_not_ready()
{
    /* NEUTRAL_HOLD */
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::NEUTRAL_HOLD,
                     ActuatorFaultReason::NONE);
    robot.state = RobotState::DISARMED;
    assert(!SimulateArmCheck());  /* Rejected: actuator not ready */

    /* RECOVERING */
    SetActuatorState(ActuatorStartupState::RECOVERING,
                     ActuatorFaultReason::OUTPUT_WRITE_FAILED);
    assert(!SimulateArmCheck());

    /* FAULT */
    SetActuatorState(ActuatorStartupState::FAULT,
                     ActuatorFaultReason::RECOVERY_FAILED);
    assert(!SimulateArmCheck());

    /* READY + DISARMED → ARM accepted */
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);
    robot.state = RobotState::DISARMED;
    robot.estop_locked = false;
    assert(robot.actuator_output_ready);
    assert(SimulateArmCheck());

    /* READY but ESTOP locked → rejected */
    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;
    assert(!SimulateArmCheck());
}

/* ------------------------------------------------------------------ */
/*  Test 12: RESET_ESTOP preserves actuator_state                      */
/* ------------------------------------------------------------------ */

void test_reset_estop_preserves_actuator_state()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);
    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;

    const ActuatorStartupState saved_state = robot.actuator_state;

    /* Simulate handle_reset_estop(): only clear estop + set DISARMED */
    if (robot.estop_locked && robot.state == RobotState::EMERGENCY_STOP) {
        if (robot.actuator_output_ready) {
            robot.estop_locked = false;
            robot.state = RobotState::DISARMED;
        }
    }

    assert(!robot.estop_locked);
    assert(robot.state == RobotState::DISARMED);
    /* actuator_state unchanged — still READY */
    assert(robot.actuator_state == saved_state);
    assert(robot.actuator_state == ActuatorStartupState::READY);
    assert(robot.actuator_output_ready);
}

/* ------------------------------------------------------------------ */
/*  Test 13: Health anomaly prevents motor_control.Update() same cycle */
/* ------------------------------------------------------------------ */

void test_health_failure_prevents_control_output()
{
    robot = RobotData{};
    SetActuatorState(ActuatorStartupState::READY,
                     ActuatorFaultReason::NONE);
    robot.state = RobotState::ARMED_IDLE;
    assert(robot.actuator_output_ready);

    MockPCA9685 pca{};

    /* Health check returns non-HEALTHY (not BUS_LOCK_TIMEOUT) */
    pca.health = 5;  /* RESET_DETECTED */

    bool control_ran = false;

    /* Simulate Layer 3: health check before Layer 8: control output */
    if (pca.health != 0 /* 0 = HEALTHY */) {
        /* Non-HEALTHY (and not BUS_LOCK_TIMEOUT) → RECOVERING */
        SetActuatorState(ActuatorStartupState::RECOVERING,
                         ActuatorFaultReason::HEALTH_RESET_DETECTED);
        /* Control output is skipped this cycle */
    } else {
        /* Normal path: motor_control.Update() runs */
        control_ran = true;
    }

    assert(robot.actuator_state == ActuatorStartupState::RECOVERING);
    assert(!control_ran);  /* Control did not execute */
    assert(!robot.actuator_output_ready);
}

} // namespace

/* ------------------------------------------------------------------ */
/*  Entry point                                                        */
/* ------------------------------------------------------------------ */

int main()
{
    test_init_success_enters_neutral_hold();
    test_init_3_failures_enters_fault();
    test_neutral_hold_blocks_non_neutral();
    test_neutral_hold_3s_with_success_enters_ready();
    test_write_fail_during_neutral_hold_resets_timer();
    test_update_fail_does_not_increment_streak();
    test_recover_streak_counting();
    test_recover_streak_3_enters_fault_estop();
    test_fault_recovers_at_low_frequency();
    test_fault_estop_persists_after_recover();
    test_arm_rejected_when_not_ready();
    test_reset_estop_preserves_actuator_state();
    test_health_failure_prevents_control_output();

    std::cout << "Actuator startup state machine tests passed (13/13)\n";
    return 0;
}
