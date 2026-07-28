#pragma once

#include "include/MS5837.h"
#include "DataBus.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <cmath>

/* CubeMX-generated I2C handle. Replace the symbol if you wired the sensor
 * to a different I2C peripheral (e.g. hi2c2 / hi2c3). The MS5837 itself
 * has a fixed 7-bit address 0x76, so multiple sensors require a separate
 * bus or an I2C multiplexer.
 */
extern "C" I2C_HandleTypeDef hi2c2;
extern volatile bool pca9685_recovery_in_progress;

/* xy_robotkit C++ lifecycle adapter for the BlueRobotics MS5837-30BA / 02BA
 * pressure & temperature sensor (STM32 HAL port).
 *
 * Lifecycle:
 *   Init()    Tries MS5837::init(&hi2c2) up to three times, rejects unknown
 *             sensor models, applies fluid density, and latches a "ready"
 *             flag. The retry count is bounded so startup cannot block
 *             forever.
 *   Update()  Polls a new pressure + temperature sample at most every
 *             read_period_ms milliseconds. One sample takes ~40 ms because
 *             the underlying library is blocking; pick read_period_ms >=
 *             50 ms to leave time for the rest of the loop. If startup
 *             failed, re-initialization is rate-limited to once per second;
 *             recovery still requires a fresh valid sample before depth hold.
 *
 * Accessors (call after Update() has run at least once):
 *   pressure_mbar()   Absolute pressure in mbar.
 *   temperature_c()   Temperature in degrees Celsius.
 *   depth_m()         Depth in meters, computed from setFluidDensity().
 *   is_ready()        True after Init() or a later retry has succeeded.
 */
class MS5837Sensor {
public:
    MS5837Sensor(int fluid_density = 997, int read_period_ms = 100)
        : density_(fluid_density),
          period_(read_period_ms),
          last_read_(0),
          last_init_attempt_ms_(0),
          ready_(false) {}

    void Init() {
        if (pca9685_recovery_in_progress) {
            ready_ = false;
            taskENTER_CRITICAL();
            robot.depth_sensor_ready = false;
            robot.depth_sample_valid = false;
            taskEXIT_CRITICAL();
            return;
        }
        ready_ = false;
        for (uint8_t attempt = 0U;
             attempt < kInitialAttemptCount && !ready_;
             ++attempt) {
            ready_ = try_init();
            if (!ready_ && (attempt + 1U) < kInitialAttemptCount) {
                HAL_Delay(kInitialRetryDelayMs);
            }
        }
        taskENTER_CRITICAL();
        robot.depth_sensor_ready = ready_;
        robot.depth_sample_valid = false;
        robot.depth_sample_ms = 0U;
        robot.depth_sample_generation = 0U;
        taskEXIT_CRITICAL();
    }

    void Update() {
        if (pca9685_recovery_in_progress) {
            return;
        }
        uint32_t now = HAL_GetTick();
        if (!ready_) {
            if ((now - last_init_attempt_ms_) >= kRuntimeRetryPeriodMs) {
                ready_ = try_init();
                now = HAL_GetTick();
                taskENTER_CRITICAL();
                robot.depth_sensor_ready = ready_;
                robot.depth_sample_valid = false;
                taskEXIT_CRITICAL();
                if (ready_) {
                    /*
                     * Recovery only restores sensor readiness.  A fresh,
                     * validated pressure sample is still required before
                     * depth hold can be enabled again.
                     */
                    last_read_ = now;
                }
            }
            return;
        }
        if (now - last_read_ >= static_cast<uint32_t>(period_)) {
            last_read_ = now;
            const bool read_ok = sensor_.read();
            const float depth_m = sensor_.depth();
            const float pressure_mbar = sensor_.pressure();
            const float water_temp_c = sensor_.temperature();
            const bool sample_ok =
                read_ok &&
                std::isfinite(depth_m) &&
                std::isfinite(pressure_mbar) &&
                std::isfinite(water_temp_c) &&
                depth_m >= ROBOT_DEPTH_VALID_MIN_M &&
                depth_m <= ROBOT_DEPTH_VALID_MAX_M;
            taskENTER_CRITICAL();
            robot.depth_sensor_ready = true;
            if (sample_ok) {
                robot.depth_m = depth_m;
                robot.pressure_mbar = pressure_mbar;
                robot.water_temp_c = water_temp_c;
                robot.depth_sample_ms = now;
                robot.depth_sample_generation++;
                robot.depth_sample_valid = true;
            } else if (robot.depth_sample_generation == 0U ||
                       (now - robot.depth_sample_ms) >
                           ROBOT_DEPTH_SAMPLE_MAX_AGE_MS) {
                robot.depth_sample_valid = false;
            }
            taskEXIT_CRITICAL();
        }
    }

    float pressure_mbar()  { return sensor_.pressure(); }
    float temperature_c()  { return sensor_.temperature(); }
    float depth_m()        { return sensor_.depth(); }
    bool  is_ready() const { return ready_; }

private:
    static constexpr uint8_t kInitialAttemptCount = 3U;
    static constexpr uint32_t kInitialRetryDelayMs = 100U;
    static constexpr uint32_t kRuntimeRetryPeriodMs = 1000U;

    bool try_init() {
        last_init_attempt_ms_ = HAL_GetTick();
        const bool transport_ready = sensor_.init(&hi2c2);
        const uint8_t model = sensor_.getModel();
        const bool model_ready =
            model == MS5837::MS5837_30BA ||
            model == MS5837::MS5837_02BA;
        if (!transport_ready || !model_ready) {
            return false;
        }
        sensor_.setFluidDensity(static_cast<float>(density_));
        return true;
    }

    MS5837   sensor_;
    int      density_;
    int      period_;
    uint32_t last_read_;
    uint32_t last_init_attempt_ms_;
    bool     ready_;
};
