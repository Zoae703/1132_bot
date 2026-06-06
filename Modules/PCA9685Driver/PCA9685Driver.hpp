#pragma once
#include "include/PCA9685.h"
#include "DataBus.hpp"

extern "C" I2C_HandleTypeDef hi2c2;

class PCA9685Driver {
public:
    PCA9685Driver(float freq = 50.0f) : freq_(freq) {}

    void Init() {
        PCA9685_Init(&hi2c2);
        // Initialize all 8 channels to neutral
        for (int i = 0; i < 8; i++)
            PCA9685_SetPWM(i, 0, (1610 * 4096 + 10000) / 20000);
    }

    void Update() {
        PCA9685_SetAllPWM(robot.pwm);
    }

private:
    float freq_;
};
