#pragma once
#include <cstdint>

inline constexpr uint32_t ROBOT_MANUAL_PWM_TIMEOUT_MS = 500U;

struct RobotData {
    // === IMU raw data (written by BMI088Sensor, IST8310Sensor) ===
    float accel[3] = {0};     // m/s²
    float gyro[3] = {0};      // rad/s  (already offset-corrected by BMI088 driver)
    float mag[3] = {0};       // µT
    float imu_temp = 0;

    // === AHRS output (written by MahonyAHRS) ===
    float quat[4] = {1,0,0,0};  // w,x,y,z
    float yaw = 0;               // rad
    float pitch = 0;             // rad
    float roll = 0;              // rad
    float yaw_v = 0;             // rad/s (gyro z — angular velocity)
    float pitch_v = 0;           // rad/s (gyro y)
    float roll_v = 0;            // rad/s (gyro x)

    // === Depth sensor (written by MS5837Sensor) ===
    float depth_m = 0;
    float pressure_mbar = 0;
    float water_temp_c = 0;

    // === Targets (written by Communication module) ===
    float target_depth_cm = 30;     // depth target in cm
    float target_yaw = 0;           // yaw target in rad
    float target_roll = 0;          // roll target in rad (always 0 = level)
    float target_pitch = 0;         // pitch target in rad (always 0 = level)

    // === Motor output (written by MotorControl, read by PCA9685Driver) ===
    int32_t pwm[8] = {1610,1610,1610,1610,1610,1610,1610,1610}; // 1000-2000µs

    // === Control flags (written by Communication) ===
    bool float_enabled = false;     // PID float control ON/OFF
    bool angle_enabled = false;     // Yaw angle hold ON/OFF
    bool manual_pwm_enabled = false; // Direct PWM test mode with timeout
    uint32_t manual_pwm_last_ms = 0;
    uint8_t motion_state = 0;       // 0=STOP, 1=FLOAT, 2=FRONT, 3=BACK, 4=LEFT, 5=RIGHT, 6=CLOCKWISE, 7=ANTICLOCKWISE

    // === System ===
    uint32_t loop_count = 0;
};

extern RobotData robot;
