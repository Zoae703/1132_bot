#pragma once
#include "include/PCA9685.h"
#include "DataBus.hpp"
#include "FreeRTOS.h"
#include "task.h"

extern "C" I2C_HandleTypeDef hi2c2;

class PCA9685Driver {
public:
    PCA9685Driver(float freq = 50.0f) : freq_(freq) {}

    bool Init() {
        (void)freq_;
        initialized_ = PCA9685_Init(&hi2c2);
        last_write_ok_ = initialized_;
        recovery_required_ = !initialized_;
        return initialized_;
    }

    bool Update() {
        if (!initialized_) {
            last_write_ok_ = false;
            return false;
        }

        int32_t pwm[8];
        uint32_t generation;
        taskENTER_CRITICAL();
        for (int i = 0; i < 8; i++) {
            pwm[i] = robot.pwm[i];
        }
        generation = robot.pwm_output_generation;
        taskEXIT_CRITICAL();

        bool superseded = false;
        last_write_ok_ = PCA9685_SetAllPWMGuarded(
            pwm, output_generation_matches,
            &robot.pwm_output_generation, generation, &superseded);
        if (!last_write_ok_) {
            taskENTER_CRITICAL();
            robot.actuator_output_ready = false;
            taskEXIT_CRITICAL();
            initialized_ = false;
            recovery_required_ = true;
            return false;
        }

        taskENTER_CRITICAL();
        superseded =
            superseded || robot.pwm_output_generation != generation;
        taskEXIT_CRITICAL();
        if (superseded) {
            const int32_t neutral[8] = {
                ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
                ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
                ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
                ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
            };
            last_write_ok_ = PCA9685_SetAllPWM(neutral);
            if (!last_write_ok_) {
                taskENTER_CRITICAL();
                robot.actuator_output_ready = false;
                taskEXIT_CRITICAL();
                initialized_ = false;
                recovery_required_ = true;
                return false;
            }
        }
        return true;
    }

    bool RecoverToNeutral() { return Init(); }
    bool is_ready() const { return initialized_; }
    bool last_write_ok() const { return last_write_ok_; }
    bool recovery_required() const { return recovery_required_; }

    PCA9685HealthStatus CheckHealth() {
        if (!initialized_) return PCA9685_HEALTH_IO_READ_FAILED;
        return PCA9685_CheckHealth();
    }

    bool Recover() {
        initialized_ = PCA9685_Recover();
        last_write_ok_ = initialized_;
        recovery_required_ = !initialized_;
        return initialized_;
    }

    void GetDiagnostics(PCA9685Diagnostics *diag) const {
        PCA9685_GetDiagnostics(diag);
    }

private:
    static bool output_generation_matches(
        const void *context, uint32_t expected_generation)
    {
        if (context == nullptr) {
            return false;
        }

        const auto *generation =
            static_cast<const uint32_t *>(context);
        uint32_t current_generation;
        taskENTER_CRITICAL();
        current_generation = *generation;
        taskEXIT_CRITICAL();
        return current_generation == expected_generation;
    }

    float freq_;
    bool initialized_ = false;
    bool last_write_ok_ = false;
    bool recovery_required_ = false;
};
