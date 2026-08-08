#pragma once

#include "PCA9685Driver.hpp"
#include "ThrusterOutputManager.hpp"

extern "C" I2C_HandleTypeDef hi2c2;

/*
 * PCA9685ThrusterModule — unified entry point for the PCA9685-based
 * ESC / thruster actuator subsystem.
 *
 * robot_tasks.cpp talks ONLY to this class.  It does not reach into
 * PCA9685Driver or ThrusterOutputManager directly.
 *
 * Internal layering:
 *
 *   PCA9685ThrusterModule   (facade — this file)
 *       ├── ThrusterOutputManager  (startup state machine & recovery strategy)
 *       └── PCA9685Driver          (chip-level I²C / register / PWM operations)
 */
class PCA9685ThrusterModule {
public:
    PCA9685ThrusterModule(I2C_HandleTypeDef *hi2c = &hi2c2);

    /* ---- Lifecycle ---- */

    /* Resets internal state.  Actual hardware init happens in Start(). */
    void Init();

    /* Call once at boot to begin the startup sequence. */
    void Start(uint32_t now_ms);

    /* Drive the internal state machine.  Must be called every control cycle. */
    void Update(uint32_t now_ms);

    /*
     * Write PWM values to hardware.  Only permitted when ready.
     * Returns false if not ready or the I²C write fails.
     */
    bool SubmitOutputs(const int32_t *pwm_us, size_t count,
                       uint32_t now_ms);

    /* Write neutral to hardware immediately, regardless of state. */
    void EmergencyNeutral(uint32_t now_ms);

    /* ---- Queries ---- */

    bool IsReady()      const;   /* actuator_output_ready gate */
    bool IsRecovering() const;   /* MS5837 gate */
    bool IsFaulted()    const;   /* ESTOP latch decision */

    ActuatorStartupState State()        const;
    ActuatorFaultReason  FaultReason()  const;

    /* ---- Diagnostics ---- */

    void GetDiagnostics(PCA9685Diagnostics *diag) const;

private:
    PCA9685Driver         driver_;
    ThrusterOutputManager manager_;
};
