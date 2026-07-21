#pragma once

#include "include/MS5837.h"
#include "DataBus.hpp"
#include "FreeRTOS.h"
#include "task.h"

/* CubeMX-generated I2C handle. Replace the symbol if you wired the sensor
 * to a different I2C peripheral (e.g. hi2c2 / hi2c3). The MS5837 itself
 * has a fixed 7-bit address 0x76, so multiple sensors require a separate
 * bus or an I2C multiplexer.
 */
extern "C" I2C_HandleTypeDef hi2c2;

/* xy_robotkit C++ lifecycle adapter for the BlueRobotics MS5837-30BA / 02BA
 * pressure & temperature sensor (STM32 HAL port).
 *
 * Lifecycle:
 *   Init()    Calls MS5837::init(&hi2c1), applies fluid density, latches a
 *             "ready" flag. Init() never blocks the loop forever; if the
 *             sensor is not present, ready_ stays false and Update() is a
 *             no-op so the rest of the system can keep running.
 *   Update()  Polls a new pressure + temperature sample at most every
 *             read_period_ms milliseconds. One sample takes ~40 ms because
 *             the underlying library is blocking; pick read_period_ms >=
 *             50 ms to leave time for the rest of the loop.
 *
 * Accessors (call after Update() has run at least once):
 *   pressure_mbar()   Absolute pressure in mbar.
 *   temperature_c()   Temperature in degrees Celsius.
 *   depth_m()         Depth in meters, computed from setFluidDensity().
 *   is_ready()        True once Init() has succeeded.
 */
class MS5837Sensor {
public:
    MS5837Sensor(int fluid_density = 997, int read_period_ms = 100)
        : density_(fluid_density),
          period_(read_period_ms),
          last_read_(0),
          ready_(false) {}

    void Init() {
        ready_ = sensor_.init(&hi2c2);
        if (ready_) {
            sensor_.setFluidDensity(static_cast<float>(density_));
        }
    }

    void Update() {
        if (!ready_) {
            return;
        }
        uint32_t now = HAL_GetTick();
        if (now - last_read_ >= static_cast<uint32_t>(period_)) {
            sensor_.read();
            last_read_ = now;
            float depth_m = sensor_.depth();
            float pressure_mbar = sensor_.pressure();
            float water_temp_c = sensor_.temperature();
            taskENTER_CRITICAL();
            robot.depth_m       = depth_m;
            robot.pressure_mbar = pressure_mbar;
            robot.water_temp_c  = water_temp_c;
            taskEXIT_CRITICAL();
        }
    }

    float pressure_mbar()  { return sensor_.pressure(); }
    float temperature_c()  { return sensor_.temperature(); }
    float depth_m()        { return sensor_.depth(); }
    bool  is_ready() const { return ready_; }

private:
    MS5837   sensor_;
    int      density_;
    int      period_;
    uint32_t last_read_;
    bool     ready_;
};
