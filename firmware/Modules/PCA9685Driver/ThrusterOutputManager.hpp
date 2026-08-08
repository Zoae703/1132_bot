#pragma once

#include "include/PCA9685.h"
#include "PCA9685Driver.hpp"
#include "DataBus.hpp"

/*
 * ThrusterOutputManager — ESC / thruster startup & recovery strategy layer.
 *
 * Lives in the same PCA9685 module library as PCA9685Driver but operates at
 * a higher abstraction level:
 *
 *   - Init retry (up to MAX_INIT_ATTEMPTS with INIT_RETRY_MS spacing)
 *   - ESC neutral-hold sequencing (ESC_NEUTRAL_HOLD_MS)
 *   - READY / RECOVERING / FAULT state machine
 *   - Periodic health-check scheduling (HEALTH_CHECK_PERIOD_MS)
 *   - Recovery retry timing & streak counting
 *   - Non-READY rejection of non-neutral output
 *   - Post-recovery neutral-hold restart
 *
 * This class writes robot.actuator_state / actuator_output_ready /
 * actuator_fault_reason (the "read-only mirror" for the rest of the system)
 * and writes neutral to robot.pwm[] during NEUTRAL_HOLD / FAULT.
 *
 * It does NOT directly modify:
 *   robot.estop_locked
 *   robot.state
 *   robot.control_enable
 *   robot.float_enabled
 *   robot.angle_enabled
 *
 * Those remain the responsibility of robot_tasks.cpp, which reads the
 * state queries below and makes top-level safety decisions.
 */
class ThrusterOutputManager {
public:
    explicit ThrusterOutputManager(PCA9685Driver &driver);

    /* ---- Lifecycle ---- */

    /* Reset all internal state to UNINITIALIZED. No hardware I/O. */
    void Reset();

    /* Call once at boot to enter INITIALIZING. */
    void Start(uint32_t now_ms);

    /*
     * Drive the internal state machine.  Must be called every control cycle
     * (~20 ms).  During NEUTRAL_HOLD and FAULT, this method forces
     * robot.pwm[] to neutral.
     */
    void Update(uint32_t now_ms);

    /*
     * Write PWM values to hardware.  Only permitted when state == READY.
     * Returns false if:
     *   - state != READY
     *   - the I²C write fails (state transitions to RECOVERING internally)
     */
    bool SubmitOutputs(const int32_t *pwm_us, size_t count, uint32_t now_ms);

    /*
     * Immediately write neutral to all eight motor channels on the hardware,
     * regardless of internal state.
     */
    void EmergencyNeutral(uint32_t now_ms);

    /* ---- Queries ---- */

    bool IsReady()      const;
    bool IsRecovering() const;
    bool IsFaulted()    const;

    ActuatorStartupState State()        const;
    ActuatorFaultReason  FaultReason()  const;

private:
    /* ---- State transitions ---- */
    void EnterState(ActuatorStartupState s, ActuatorFaultReason r,
                    uint32_t now_ms);
    void SyncStateToRobot();

    /* ---- Per-state service routines ---- */
    void ServiceInitializing(uint32_t now_ms);
    void ServiceNeutralHold(uint32_t now_ms);
    void ServiceReady(uint32_t now_ms);
    void ServiceRecovering(uint32_t now_ms);
    void ServiceFault(uint32_t now_ms);

    /* ---- Helpers ---- */
    void ForceSoftwareNeutral(uint8_t neutral_reason);
    bool WriteNeutralToHardware();
    static ActuatorFaultReason MapHealthToFaultReason(
        PCA9685HealthStatus hs);

    /* ---- Hardware handle ---- */
    PCA9685Driver &driver_;

    /* ---- State ---- */
    ActuatorStartupState state_        = ActuatorStartupState::UNINITIALIZED;
    ActuatorFaultReason  fault_reason_ = ActuatorFaultReason::NONE;

    /* ---- Init retry ---- */
    uint8_t  init_attempt_       = 0;
    uint32_t last_init_retry_ms_ = 0;

    /* ---- Neutral hold ---- */
    uint32_t neutral_hold_start_ms_ = 0;

    /* ---- Health check ---- */
    uint32_t last_health_check_ms_ = 0;

    /* ---- Recovery ---- */
    uint8_t  recovery_failure_streak_  = 0;
    uint32_t last_recovery_attempt_ms_ = 0;

    /* ---- FAULT low-frequency recovery ---- */
    uint32_t last_fault_recovery_ms_ = 0;

    /* ---- Statistics ---- */
    uint32_t output_write_failure_count_ = 0;

    /* ---- Constants (all verified on real hardware — DO NOT CHANGE) ---- */
    static constexpr uint8_t  MAX_INIT_ATTEMPTS      = 3U;
    static constexpr uint32_t INIT_RETRY_MS          = 100U;
    static constexpr uint32_t ESC_NEUTRAL_HOLD_MS    = 3000U;
    static constexpr uint32_t HEALTH_CHECK_PERIOD_MS = 1000U;
    static constexpr uint8_t  MAX_RECOVERY_STREAK    = 3U;
    static constexpr uint32_t RECOVERY_RETRY_MS      = 100U;
    static constexpr uint32_t FAULT_RECOVERY_PERIOD_MS = 1000U;
};
