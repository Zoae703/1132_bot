#pragma once
#include <cstdint>

inline constexpr int32_t ROBOT_PWM_NEUTRAL_US = 1500;
inline constexpr int32_t ROBOT_PWM_MIN_US = 1000;
inline constexpr int32_t ROBOT_PWM_MAX_US = 2000;
inline constexpr int32_t ROBOT_PWM_TEST_MIN_US = 1000;
inline constexpr int32_t ROBOT_PWM_TEST_MAX_US = 2000;
inline constexpr uint32_t ROBOT_COMMAND_TIMEOUT_MS = 500U;
inline constexpr uint32_t ROBOT_MANUAL_PWM_TIMEOUT_MS = 500U;
inline constexpr uint32_t ROBOT_HEARTBEAT_TIMEOUT_MS = 1000U;

enum class RobotState : uint8_t {
    DISARMED = 0,
    ARMED_IDLE = 1,
    ARMED_ACTIVE = 2,
    MANUAL_TEST = 3,
    COMM_LOST = 4,
    EMERGENCY_STOP = 5,
    FAULT = 6
};

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
    int32_t pwm[8] = {
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US
    };

    // === Control flags (written by Communication) ===
    bool control_enable = false;    // Master arm/control enable from Orange Pi
    bool float_enabled = false;     // PID float control ON/OFF
    bool angle_enabled = false;     // Yaw angle hold ON/OFF
    bool manual_pwm_enabled = false; // Direct PWM test mode with timeout
    int32_t manual_pwm[8] = {
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US
    };
    uint32_t manual_pwm_last_ms = 0;
    uint32_t last_cmd_tick = 0;
    uint8_t motion_state = 0;       // 0=STOP, 1=FLOAT, 2=FRONT, 3=BACK, 4=LEFT, 5=RIGHT, 6=CLOCKWISE, 7=ANTICLOCKWISE

    // === Safety state machine ===
    RobotState state = RobotState::DISARMED;
    uint32_t state_changed_ms = 0;
    bool     estop_locked = false;

    // === Manual test tracking ===
    uint8_t  active_test_channel = 0xFF;  // 0xFF = none
    uint32_t channel_test_deadline = 0;   // HAL_GetTick() when current test expires
    uint8_t  last_neutral_reason = 0;     // ProtoNeutralReason wire value
    int32_t  pwm_test_min_us = ROBOT_PWM_TEST_MIN_US;
    int32_t  pwm_test_max_us = ROBOT_PWM_TEST_MAX_US;

    // === Heartbeat / link monitoring ===
    uint32_t last_heartbeat_ms = 0;
    uint32_t heartbeat_timeout_ms = ROBOT_HEARTBEAT_TIMEOUT_MS;
    uint16_t heartbeat_missed = 0;

    // === Error counters ===
    uint16_t crc_errors = 0;
    uint16_t frame_errors = 0;
    uint16_t protocol_errors = 0;

    // === System ===
    uint32_t loop_count = 0;
    uint32_t uptime_s = 0;
    uint32_t last_uptime_tick = 0;
};

extern RobotData robot;
