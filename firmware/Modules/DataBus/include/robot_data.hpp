#pragma once
#include <cstdint>

inline constexpr int32_t ROBOT_PWM_NEUTRAL_US = 1500;
inline constexpr int32_t ROBOT_PWM_MIN_US = 1000;
inline constexpr int32_t ROBOT_PWM_MAX_US = 2000;
inline constexpr int32_t ROBOT_PWM_TEST_MIN_US = 1000;
inline constexpr int32_t ROBOT_PWM_TEST_MAX_US = 2000;
inline constexpr uint32_t ROBOT_COMMAND_TIMEOUT_MS = 500U;
inline constexpr uint32_t ROBOT_BODY_COMMAND_TIMEOUT_MS = 500U;
inline constexpr uint32_t ROBOT_MANUAL_PWM_TIMEOUT_MS = 500U;
inline constexpr uint32_t ROBOT_HEARTBEAT_TIMEOUT_MS = 1000U;
inline constexpr uint8_t ROBOT_BODY_AXIS_COUNT = 6U;
inline constexpr uint16_t ROBOT_PWM_SLEW_RATE_MIN_US_PER_S = 100U;
inline constexpr uint16_t ROBOT_PWM_SLEW_RATE_MAX_US_PER_S = 5000U;
inline constexpr uint16_t ROBOT_BODY_COMMAND_TIMEOUT_MIN_MS = 200U;
inline constexpr uint16_t ROBOT_BODY_COMMAND_TIMEOUT_MAX_MS = 2000U;
inline constexpr uint32_t ROBOT_DEPTH_SAMPLE_MAX_AGE_MS = 500U;
inline constexpr float ROBOT_DEPTH_VALID_MIN_M = -10.0F;
inline constexpr float ROBOT_DEPTH_VALID_MAX_M = 300.0F;
inline constexpr float ROBOT_DEPTH_TARGET_MIN_CM = 0.0F;
inline constexpr float ROBOT_DEPTH_TARGET_MAX_CM = 30000.0F;

enum class RobotState : uint8_t {
    DISARMED = 0,
    ARMED_IDLE = 1,
    ARMED_ACTIVE = 2,
    MANUAL_TEST = 3,
    COMM_LOST = 4,
    EMERGENCY_STOP = 5,
    FAULT = 6
};

struct BodyCommand {
    float surge = 0.0F;
    float sway = 0.0F;
    float heave = 0.0F;
    float roll = 0.0F;
    float pitch = 0.0F;
    float yaw = 0.0F;
};

struct MotionTuning {
    float axis_gain[ROBOT_BODY_AXIS_COUNT] = {
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F
    };
    float axis_max_output[ROBOT_BODY_AXIS_COUNT] = {
        0.20F, 0.20F, 0.20F, 0.10F, 0.10F, 0.10F
    };
    float global_multiplier = 1.0F;
    uint16_t pwm_slew_rate_us_per_s = 1000U;
    uint16_t command_timeout_ms = ROBOT_BODY_COMMAND_TIMEOUT_MS;
};

struct DepthPidTuning {
    float kp = 10.0F;
    float ki = 0.02F;
    float kd = 10.0F;
    float p_limit_us = 100.0F;
    float i_limit_us = 50.0F;
    float d_limit_us = 50.0F;
    float output_limit_us = 200.0F;
};

enum class BodyCommandSource : uint8_t {
    None = 0,
    LegacySetMotion = 1,
    BinaryProtocol = 2,
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
    bool depth_sensor_ready = false;
    bool depth_sample_valid = false;
    uint32_t depth_sample_ms = 0;
    uint32_t depth_sample_generation = 0;

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

    // === Unified continuous body command ===
    BodyCommand body_command{};
    BodyCommandSource body_command_source = BodyCommandSource::None;
    uint16_t body_command_sequence = 0;
    uint32_t body_command_last_ms = 0;
    uint32_t body_command_timeout_ms = ROBOT_BODY_COMMAND_TIMEOUT_MS;
    bool body_command_valid = false;
    bool body_control_enabled = false;
    MotionTuning motion_tuning{};
    uint32_t motion_tuning_generation = 0;
    DepthPidTuning depth_pid_tuning{};
    uint32_t depth_pid_tuning_generation = 0;
    uint32_t depth_control_generation = 0;
    uint32_t depth_hold_session_generation = 0;
    uint32_t depth_command_last_ms = 0;
    float depth_active_setpoint_cm = 0.0F;
    float depth_control_measured_cm = 0.0F;
    float depth_error_cm = 0.0F;
    float depth_pid_p_us = 0.0F;
    float depth_pid_i_us = 0.0F;
    float depth_pid_d_us = 0.0F;
    float depth_pid_output_us = 0.0F;
    bool depth_pid_saturated = false;
    uint8_t depth_control_fault_reason = 0;
    float mixed_output[8] = {0.0F};
    bool horizontal_saturated = false;
    bool vertical_saturated = false;
    uint32_t pwm_output_generation = 0;
    // Set only after PCA9685 timing and all channel registers are verified.
    // ARM and ESTOP reset remain blocked while actuator output is unavailable.
    bool actuator_output_ready = false;

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

bool body_command_is_valid(const BodyCommand &command);
bool body_command_is_zero(const BodyCommand &command);
bool motion_tuning_is_valid(const MotionTuning &tuning);
bool depth_pid_tuning_is_valid(const DepthPidTuning &tuning);
bool depth_sample_is_fresh(const RobotData &data, uint32_t now);
void reset_depth_control_runtime(RobotData &data);
void invalidate_body_command(RobotData &data);
void mark_pwm_output_updated(RobotData &data);
void force_body_output_neutral(RobotData &data);

extern RobotData robot;
