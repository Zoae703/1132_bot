#pragma once
#include "include/PCA9685.h"
#include "DataBus.hpp"
#include "FreeRTOS.h"
#include "task.h"

extern "C" I2C_HandleTypeDef hi2c2;

class PCA9685Driver {
public:
    PCA9685Driver(float freq = 50.0f) : freq_(freq) {}

    void Init() {
        (void)freq_;
        initialized_ = PCA9685_Init(&hi2c2);
        last_write_ok_ = initialized_;
    }

    void Update() {
        if (!initialized_) {
            return;
        }

        int32_t pwm[8];
        taskENTER_CRITICAL();
        for (int i = 0; i < 8; i++)
            pwm[i] = robot.pwm[i];
        taskEXIT_CRITICAL();
        last_write_ok_ = PCA9685_SetAllPWM(pwm);
    }

    bool is_ready() const { return initialized_; }
    bool last_write_ok() const { return last_write_ok_; }

private:
    float freq_;
    bool initialized_ = false;
    bool last_write_ok_ = false;
};
