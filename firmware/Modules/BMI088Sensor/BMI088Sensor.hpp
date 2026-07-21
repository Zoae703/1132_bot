#pragma once

#include "main.h"
#include "include/BMI088driver.h"
#include "DataBus.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern SPI_HandleTypeDef hspi1;

/**
 * xy_robotkit C++ lifecycle adapter for BMI088 six-axis IMU
 * (gyroscope + accelerometer) via blocking SPI.
 *
 * Default wiring: RoboMaster C-Board SPI1
 *   CS_ACCEL = PA4, CS_GYRO = PB0
 *   SCK = PB3, MISO = PB4, MOSI = PA7
 *
 * Lifecycle:
 *   Init()   — self-test + register configuration, returns via _initialized flag.
 *   Update() — blocking read of gyro + accel + temperature, rate-limited by
 *              read_period_ms.
 */
class BMI088Sensor {
public:
    BMI088Sensor(int read_period_ms = 5,
                 const char* accel_range = "3G",
                 const char* gyro_range = "2000")
        : _period(read_period_ms), _last_read(0), _initialized(false)
    {
        _parse_accel_range(accel_range);
        _parse_gyro_range(gyro_range);
    }

    void Init() {
        uint8_t err = BMI088_init();
        _initialized = (err == BMI088_NO_ERROR);
    }

    void Update() {
        if (!_initialized) return;
        uint32_t now = HAL_GetTick();
        if (now - _last_read < (uint32_t)_period) return;
        _last_read = now;

        BMI088_read(_gyro, _accel, &_temp);

        // Publish to DataBus (accel m/s², gyro rad/s)
        robot.accel[0] = _accel[0];
        robot.accel[1] = _accel[1];
        robot.accel[2] = _accel[2];
        robot.gyro[0]  = _gyro[0];
        robot.gyro[1]  = _gyro[1];
        robot.gyro[2]  = _gyro[2];
        robot.imu_temp = _temp;
    }

    float accel_x() const { return _accel[0]; }
    float accel_y() const { return _accel[1]; }
    float accel_z() const { return _accel[2]; }
    float gyro_x()  const { return _gyro[0]; }
    float gyro_y()  const { return _gyro[1]; }
    float gyro_z()  const { return _gyro[2]; }
    float temperature() const { return _temp; }

    float pitch() const {
        return atan2f(_accel[0], sqrtf(_accel[1]*_accel[1] + _accel[2]*_accel[2])) * 180.0f / M_PI;
    }
    float roll() const {
        return atan2f(_accel[1], _accel[2]) * 180.0f / M_PI;
    }

    bool is_ready() const { return _initialized; }

private:
    int      _period;
    uint32_t _last_read;
    bool     _initialized;
    float    _accel[3] = {0};
    float    _gyro[3]  = {0};
    float    _temp     = 0;

    void _parse_accel_range(const char* r) {
        (void)r; // Range is configured at compile-time in BMI088driver.h
    }
    void _parse_gyro_range(const char* r) {
        (void)r; // Range is configured at compile-time in BMI088driver.h
    }
};
