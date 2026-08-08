#include "ThrusterOutputManager.hpp"

#include "i2c.h"
#include "binary_protocol.h"
#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>
#include <cstring>

/* ======================================================================
 *  Construction
 * ====================================================================== */

ThrusterOutputManager::ThrusterOutputManager(PCA9685Driver &driver)
    : driver_(driver)
{}

/* ======================================================================
 *  Lifecycle
 * ====================================================================== */

void ThrusterOutputManager::Reset()
{
    state_                    = ActuatorStartupState::UNINITIALIZED;
    fault_reason_             = ActuatorFaultReason::NONE;
    init_attempt_             = 0;
    last_init_retry_ms_       = 0;
    neutral_hold_start_ms_    = 0;
    last_health_check_ms_     = 0;
    recovery_failure_streak_  = 0;
    last_recovery_attempt_ms_ = 0;
    last_fault_recovery_ms_   = 0;
    output_write_failure_count_ = 0;
}

void ThrusterOutputManager::Start(uint32_t now_ms)
{
    (void)now_ms;

    /* Reset everything and enter INITIALIZING. */
    state_                    = ActuatorStartupState::UNINITIALIZED;
    fault_reason_             = ActuatorFaultReason::NONE;
    init_attempt_             = 0;
    last_init_retry_ms_       = 0;
    neutral_hold_start_ms_    = 0;
    last_health_check_ms_     = 0;
    recovery_failure_streak_  = 0;
    last_recovery_attempt_ms_ = 0;
    last_fault_recovery_ms_   = 0;
    output_write_failure_count_ = 0;

    EnterState(ActuatorStartupState::INITIALIZING,
               ActuatorFaultReason::NONE, now_ms);

    /* Gate MS5837 from I²C while we initialise. */
    I2C2Bus_SetRecovering(true);
}

void ThrusterOutputManager::Update(uint32_t now_ms)
{
    switch (state_) {
    case ActuatorStartupState::UNINITIALIZED:
        /* Should have been Start()ed.  Enter INITIALIZING as fallback. */
        EnterState(ActuatorStartupState::INITIALIZING,
                   ActuatorFaultReason::NONE, now_ms);
        I2C2Bus_SetRecovering(true);
        ServiceInitializing(now_ms);
        break;

    case ActuatorStartupState::INITIALIZING:
        ServiceInitializing(now_ms);
        break;

    case ActuatorStartupState::NEUTRAL_HOLD:
        ServiceNeutralHold(now_ms);
        break;

    case ActuatorStartupState::READY:
        ServiceReady(now_ms);
        break;

    case ActuatorStartupState::RECOVERING:
        ServiceRecovering(now_ms);
        break;

    case ActuatorStartupState::FAULT:
        ServiceFault(now_ms);
        break;
    }
}

bool ThrusterOutputManager::SubmitOutputs(
    const int32_t *pwm_us, size_t count, uint32_t now_ms)
{
    if (state_ != ActuatorStartupState::READY) {
        return false;
    }
    if (pwm_us == nullptr || count == 0 || count > 16) {
        return false;
    }

    const bool ok = driver_.SetOutputs(pwm_us, count);
    if (!ok) {
        output_write_failure_count_++;
        char msg[48];
        std::snprintf(msg, sizeof(msg),
            "[I2C_FAIL] entering RECOVERING");
        Protocol_LogQueuePush(msg);

        /* Single output failure → RECOVERING immediately.
         * Do NOT increment recovery_failure_streak_ — that is for
         * Recover() failures, not output-write failures. */
        I2C2Bus_SetRecovering(true);
        taskENTER_CRITICAL();
        robot.actuator_output_ready = false;
        taskEXIT_CRITICAL();
        recovery_failure_streak_  = 0;
        last_recovery_attempt_ms_ = 0;
        EnterState(ActuatorStartupState::RECOVERING,
                   ActuatorFaultReason::OUTPUT_WRITE_FAILED, now_ms);
        return false;
    }
    return true;
}

void ThrusterOutputManager::EmergencyNeutral(uint32_t now_ms)
{
    (void)now_ms;
    (void)driver_.SetNeutralOutputs();
}

/* ======================================================================
 *  Queries
 * ====================================================================== */

bool ThrusterOutputManager::IsReady() const
{
    return state_ == ActuatorStartupState::READY;
}

bool ThrusterOutputManager::IsRecovering() const
{
    return state_ == ActuatorStartupState::INITIALIZING ||
           state_ == ActuatorStartupState::RECOVERING  ||
           state_ == ActuatorStartupState::FAULT;
}

bool ThrusterOutputManager::IsFaulted() const
{
    return state_ == ActuatorStartupState::FAULT;
}

ActuatorStartupState ThrusterOutputManager::State() const
{
    return state_;
}

ActuatorFaultReason ThrusterOutputManager::FaultReason() const
{
    return fault_reason_;
}

/* ======================================================================
 *  State transitions
 * ====================================================================== */

void ThrusterOutputManager::EnterState(
    ActuatorStartupState s, ActuatorFaultReason r, uint32_t now_ms)
{
    (void)now_ms;

    /* Leave READY → close ARM gate first */
    if (s != ActuatorStartupState::READY) {
        taskENTER_CRITICAL();
        robot.actuator_output_ready = false;
        taskEXIT_CRITICAL();
    }

    state_        = s;
    fault_reason_ = r;

    /* Enter READY → open ARM gate last */
    if (s == ActuatorStartupState::READY) {
        taskENTER_CRITICAL();
        robot.actuator_output_ready = true;
        taskEXIT_CRITICAL();
    }

    SyncStateToRobot();
}

void ThrusterOutputManager::SyncStateToRobot()
{
    taskENTER_CRITICAL();
    robot.actuator_state       = state_;
    robot.actuator_fault_reason = fault_reason_;
    taskEXIT_CRITICAL();
}

/* ======================================================================
 *  Per-state service routines
 * ====================================================================== */

void ThrusterOutputManager::ServiceInitializing(uint32_t now_ms)
{
    /*
     * Try PCA9685_Init() with timed retries.
     * This is a blocking I²C operation — the I2C2 mutex naturally
     * serialises with MS5837.  I2C2Bus_SetRecovering(true) keeps
     * MS5837 from even attempting the lock.
     */
    if (init_attempt_ >= MAX_INIT_ATTEMPTS) {
        /* All attempts exhausted → FAULT.
         * The module reports IsFaulted(); robot_tasks.cpp decides
         * whether to latch ESTOP. */
        taskENTER_CRITICAL();
        ForceSoftwareNeutral(ProtoNeutral_FAULT);
        taskEXIT_CRITICAL();
        I2C2Bus_SetRecovering(true);
        EnterState(ActuatorStartupState::FAULT,
                   ActuatorFaultReason::INIT_FAILED, now_ms);
        Protocol_LogQueuePush("[PCA_INIT_FAIL]");
        return;
    }

    if (init_attempt_ == 0 ||
        (now_ms - last_init_retry_ms_) >= INIT_RETRY_MS)
    {
        last_init_retry_ms_ = now_ms;
        const bool ok = driver_.Init();
        init_attempt_++;

        if (ok) {
            /* Init succeeded — hardware is already outputting neutral
             * on all sixteen channels.  Sync software targets and
             * enter NEUTRAL_HOLD. */
            taskENTER_CRITICAL();
            ForceSoftwareNeutral(ProtoNeutral_STARTUP);
            taskEXIT_CRITICAL();
            I2C2Bus_SetRecovering(false);
            neutral_hold_start_ms_ = now_ms;
            recovery_failure_streak_ = 0;
            EnterState(ActuatorStartupState::NEUTRAL_HOLD,
                       ActuatorFaultReason::NONE, now_ms);
            Protocol_LogQueuePush("[ESC_NEUTRAL_HOLD]");
        } else if (init_attempt_ >= MAX_INIT_ATTEMPTS) {
            taskENTER_CRITICAL();
            ForceSoftwareNeutral(ProtoNeutral_FAULT);
            taskEXIT_CRITICAL();
            I2C2Bus_SetRecovering(true);
            EnterState(ActuatorStartupState::FAULT,
                       ActuatorFaultReason::INIT_FAILED, now_ms);
            Protocol_LogQueuePush("[PCA_INIT_FAIL]");
        }
        /* else: retry on next cycle (timed by INIT_RETRY_MS) */
    }
}

void ThrusterOutputManager::ServiceNeutralHold(uint32_t now_ms)
{
    /* Force software neutral every cycle.  The hardware already has
     * neutral from Init(), but we keep the software image in sync
     * and write it to hardware so the ESC sees continuous pulses. */
    taskENTER_CRITICAL();
    ForceSoftwareNeutral(ProtoNeutral_STARTUP);
    taskEXIT_CRITICAL();

    /* ---- Periodic health check ---- */
    if ((now_ms - last_health_check_ms_) >= HEALTH_CHECK_PERIOD_MS) {
        last_health_check_ms_ = now_ms;
        const PCA9685HealthStatus hs = driver_.CheckHealth();

        if (hs != PCA9685_HEALTHY) {
            if (hs == PCA9685_HEALTH_BUS_LOCK_TIMEOUT) {
                /* Single lock timeout: record, defer, do NOT trigger recovery. */
                Protocol_LogQueuePush(
                    "[PCA_HEALTH] BUS_LOCK_TIMEOUT (deferred)");
            } else {
                char msg[80];
                std::snprintf(msg, sizeof(msg),
                    "[PCA_HEALTH_FAIL] status=%u",
                    static_cast<unsigned>(hs));
                Protocol_LogQueuePush(msg);

                /* Non-BUS_LOCK health anomaly → RECOVERING.
                 * The module reports IsRecovering(); robot_tasks.cpp
                 * decides whether to DISARM. */
                I2C2Bus_SetRecovering(true);
                taskENTER_CRITICAL();
                robot.actuator_output_ready = false;
                taskEXIT_CRITICAL();
                recovery_failure_streak_  = 0;
                last_recovery_attempt_ms_ = 0;
                EnterState(ActuatorStartupState::RECOVERING,
                           MapHealthToFaultReason(hs), now_ms);
                return;
            }
        }
    }

    /* ---- 3-second neutral hold complete → READY ---- */
    if ((now_ms - neutral_hold_start_ms_) >= ESC_NEUTRAL_HOLD_MS) {
        EnterState(ActuatorStartupState::READY,
                   ActuatorFaultReason::NONE, now_ms);
        Protocol_LogQueuePush("[ESC_READY]");
    }
}

void ThrusterOutputManager::ServiceReady(uint32_t now_ms)
{
    /* ---- Periodic health check ---- */
    if ((now_ms - last_health_check_ms_) >= HEALTH_CHECK_PERIOD_MS) {
        last_health_check_ms_ = now_ms;
        const PCA9685HealthStatus hs = driver_.CheckHealth();

        if (hs != PCA9685_HEALTHY) {
            if (hs == PCA9685_HEALTH_BUS_LOCK_TIMEOUT) {
                Protocol_LogQueuePush(
                    "[PCA_HEALTH] BUS_LOCK_TIMEOUT (deferred)");
            } else {
                char msg[80];
                std::snprintf(msg, sizeof(msg),
                    "[PCA_HEALTH_FAIL] status=%u",
                    static_cast<unsigned>(hs));
                Protocol_LogQueuePush(msg);

                I2C2Bus_SetRecovering(true);
                taskENTER_CRITICAL();
                robot.actuator_output_ready = false;
                taskEXIT_CRITICAL();
                recovery_failure_streak_  = 0;
                last_recovery_attempt_ms_ = 0;
                EnterState(ActuatorStartupState::RECOVERING,
                           MapHealthToFaultReason(hs), now_ms);
            }
        }
    }
}

void ThrusterOutputManager::ServiceRecovering(uint32_t now_ms)
{
    /*
     * Retry Recover() on a fixed period.
     * Only Recover() failures increment the streak — not output-write
     * failures (those are counted separately in output_write_failure_count_).
     */
    if ((now_ms - last_recovery_attempt_ms_) >= RECOVERY_RETRY_MS) {
        last_recovery_attempt_ms_ = now_ms;
        Protocol_LogQueuePush("[PCA_RECOVER_BEGIN]");

        const bool recovered = driver_.Recover();
        if (recovered) {
            /* Recover succeeded → restart NEUTRAL_HOLD from 0.
             * The module reports IsReady()=false; robot_tasks.cpp
             * must not re-ARM automatically. */
            I2C2Bus_SetRecovering(false);
            recovery_failure_streak_  = 0;
            neutral_hold_start_ms_    = now_ms;

            taskENTER_CRITICAL();
            ForceSoftwareNeutral(ProtoNeutral_STARTUP);
            taskEXIT_CRITICAL();

            EnterState(ActuatorStartupState::NEUTRAL_HOLD,
                       ActuatorFaultReason::NONE, now_ms);
            Protocol_LogQueuePush("[PCA_RECOVER_OK]");
            Protocol_LogQueuePush("[ESC_NEUTRAL_HOLD]");
        } else {
            /* Recover failed — increment streak. */
            recovery_failure_streak_++;
            Protocol_LogQueuePush("[PCA_RECOVER_FAIL]");

            if (recovery_failure_streak_ >= MAX_RECOVERY_STREAK) {
                /* Streak exhausted → FAULT.
                 * The module reports IsFaulted(); robot_tasks.cpp
                 * decides to latch ESTOP. */
                I2C2Bus_SetRecovering(true);
                EnterState(ActuatorStartupState::FAULT,
                           ActuatorFaultReason::RECOVERY_FAILED, now_ms);
                Protocol_LogQueuePush(
                    "[PCA_RECOVER_FAIL] ESTOP latched");
            }
        }
    }
}

void ThrusterOutputManager::ServiceFault(uint32_t now_ms)
{
    /* Force software neutral every cycle. */
    taskENTER_CRITICAL();
    ForceSoftwareNeutral(ProtoNeutral_FAULT);
    taskEXIT_CRITICAL();

    /*
     * Low-frequency recovery attempt.
     * The module stays in FAULT even if Recover succeeds (transitions
     * to NEUTRAL_HOLD).  It reports IsFaulted()=false after transition,
     * so robot_tasks.cpp can distinguish.  ESTOP is NOT cleared by
     * the module — robot_tasks.cpp must do that via RESET_ESTOP.
     */
    if ((now_ms - last_fault_recovery_ms_) >= FAULT_RECOVERY_PERIOD_MS) {
        last_fault_recovery_ms_ = now_ms;

        if (driver_.Recover()) {
            I2C2Bus_SetRecovering(false);
            neutral_hold_start_ms_ = now_ms;

            taskENTER_CRITICAL();
            ForceSoftwareNeutral(ProtoNeutral_STARTUP);
            taskEXIT_CRITICAL();

            EnterState(ActuatorStartupState::NEUTRAL_HOLD,
                       ActuatorFaultReason::NONE, now_ms);
            Protocol_LogQueuePush(
                "[PCA_RECOVER_OK] (FAULT→NEUTRAL_HOLD, ESTOP persists)");
        }
    }
}

/* ======================================================================
 *  Helpers
 * ====================================================================== */

void ThrusterOutputManager::ForceSoftwareNeutral(uint8_t neutral_reason)
{
    /*
     * Override all software targets to neutral.
     * This ensures that even if a stale command reaches robot.pwm[],
     * it will be overwritten before the next hardware write.
     *
     * Does NOT clear robot.control_enable / float_enabled / angle_enabled —
     * those are the responsibility of robot_tasks.cpp's safety gate.
     */
    robot.active_test_channel   = 0xFF;
    robot.depth_command_last_ms = 0U;
    robot.last_neutral_reason   = neutral_reason;
    reset_depth_control_runtime(robot);
    robot.depth_control_fault_reason = neutral_reason;
    force_body_output_neutral(robot);
    for (int i = 0; i < 8; i++) {
        robot.manual_pwm[i] = ROBOT_PWM_NEUTRAL_US;
    }
}

bool ThrusterOutputManager::WriteNeutralToHardware()
{
    /* Only attempt hardware write if the chip is initialised. */
    if (!driver_.IsInitialized()) {
        return false;
    }
    return driver_.SetNeutralOutputs();
}

ActuatorFaultReason ThrusterOutputManager::MapHealthToFaultReason(
    PCA9685HealthStatus hs)
{
    switch (hs) {
    case PCA9685_HEALTH_RESET_DETECTED:
        return ActuatorFaultReason::HEALTH_RESET_DETECTED;
    case PCA9685_HEALTH_IO_READ_FAILED:
        return ActuatorFaultReason::HEALTH_IO_READ_FAILED;
    case PCA9685_HEALTH_TCA_SELECT_FAILED:
        return ActuatorFaultReason::HEALTH_TCA_SELECT_FAILED;
    case PCA9685_HEALTH_CONFIG_MISMATCH:
        return ActuatorFaultReason::HEALTH_CONFIG_MISMATCH;
    default:
        return ActuatorFaultReason::HEALTH_CONFIG_MISMATCH;
    }
}
