#pragma once
#include "include/IST8310driver.h"
#include "DataBus.hpp"

extern "C" I2C_HandleTypeDef hi2c3;

class IST8310Sensor {
public:
    IST8310Sensor(int read_period_ms = 10)
        : period_(read_period_ms) {}

    void Init() {
        ready_ = (ist8310_init(&hi2c3) == 0);
    }

    void Update() {
        if (!ready_) return;
        uint32_t now = HAL_GetTick();
        if (now - last_read_ < (uint32_t)period_) return;
        last_read_ = now;

        float raw[3];
        ist8310_read_mag(&hi2c3, raw);

        // Apply calibration (ellipsoid correction from reference)
        // MAG_SCALE/MAG_OFFSET from FinsROV — recalibrate for your hardware!
        constexpr float SX = 0.889875f, SY = 0.880329f, SZ = 1.229795f;
        constexpr float OX = 8.418958f, OY = -21.033503f, OZ = -4.048424f;

        robot.mag[0] = (raw[0] - OX) * SX;
        robot.mag[1] = (raw[1] - OY) * SY;
        robot.mag[2] = (raw[2] - OZ) * SZ;
    }

    bool is_ready() const { return ready_; }

private:
    int period_;
    uint32_t last_read_ = 0;
    bool ready_ = false;
};
