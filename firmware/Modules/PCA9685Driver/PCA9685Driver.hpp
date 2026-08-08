#pragma once
#include "include/PCA9685.h"
#include <cstddef>
#include <cstdint>

extern "C" I2C_HandleTypeDef hi2c2;

/*
 * PCA9685Driver — chip-level PCA9685 driver.
 *
 * Responsibilities (and nothing else):
 *  - I²C bus access (via the existing C API that owns the bus lock + TCA9548A)
 *  - MODE1 / MODE2 / PRE_SCALE configuration
 *  - PWM µs → count conversion and per-channel / batch writes
 *  - ALL_LED FULL_OFF clearing
 *  - Post-init register read-back verification
 *  - Runtime configuration health checks
 *  - Power-on-reset signature detection
 *  - Chip-level Recover (enter SLEEP, reconfigure, verify, wake)
 *  - Diagnostics counters
 *
 * This class does NOT depend on RobotData, FreeRTOS, RobotState, estop_locked,
 * or any control-enable flags.  It is a pure hardware abstraction.
 */
class PCA9685Driver {
public:
    PCA9685Driver(I2C_HandleTypeDef *hi2c = &hi2c2) : hi2c_(hi2c) {}

    /* ---- Lifecycle ---- */

    /*
     * Full hardware initialisation: force outputs off, configure MODE1/2,
     * PRESCALE, write neutral to all sixteen channels, verify, wake, verify.
     * Returns true when every step passes including read-back verification.
     */
    virtual bool Init() {
        initialized_ = PCA9685_Init(hi2c_);
        last_write_ok_ = initialized_;
        return initialized_;
    }

    /*
     * Chip-level recovery: enter SLEEP, clear ALL_LED full-off,
     * reconfigure registers, write neutral to all channels,
     * verify while SLEEP is asserted, wake, verify after wake.
     * The caller MUST have already discarded old thrust values.
     */
    virtual bool Recover() {
        initialized_ = PCA9685_Recover();
        last_write_ok_ = initialized_;
        return initialized_;
    }

    /* ---- Health ---- */

    virtual PCA9685HealthStatus CheckHealth() {
        if (!initialized_) return PCA9685_HEALTH_IO_READ_FAILED;
        return PCA9685_CheckHealth();
    }

    /* ---- Output ---- */

    /*
     * Write PWM values (in µs, 1000–2000) to channels 0 .. count-1.
     * Uses the unprotected batch-write — the caller must have already
     * validated that the values have not been superseded.
     */
    virtual bool SetOutputs(const int32_t *pwm_us, size_t count) {
        if (!initialized_ || pwm_us == nullptr || count == 0 || count > 16) {
            last_write_ok_ = false;
            return false;
        }
        last_write_ok_ = PCA9685_SetAllPWM(pwm_us);
        if (!last_write_ok_) {
            initialized_ = false;
        }
        return last_write_ok_;
    }

    /*
     * Write neutral (1500 µs) to all eight motor channels.
     */
    virtual bool SetNeutralOutputs() {
        const int32_t neutral[8] = {
            1500, 1500, 1500, 1500,
            1500, 1500, 1500, 1500,
        };
        return SetOutputs(neutral, 8);
    }

    /*
     * Force all sixteen channels to FULL_OFF immediately.
     * Used as a last-resort safety measure.
     */
    virtual bool ForceOutputsOff() {
        const bool ok = PCA9685_ForceOutputsOff();
        if (!ok) {
            initialized_ = false;
            last_write_ok_ = false;
        }
        return ok;
    }

    /* ---- Queries ---- */

    virtual bool IsInitialized()    const { return initialized_; }
    virtual bool LastWriteSucceeded() const { return last_write_ok_; }

    virtual void GetDiagnostics(PCA9685Diagnostics *diag) const {
        PCA9685_GetDiagnostics(diag);
    }

protected:
    I2C_HandleTypeDef *hi2c_;
    bool initialized_  = false;
    bool last_write_ok_ = false;
};
