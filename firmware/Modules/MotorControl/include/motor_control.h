#pragma once
#include "PID.h"
#include <cstdint>

struct BodyCommand;
struct MotionTuning;

// 8-channel thruster PID cascade + force allocation.
// Reads sensor/AHRS/target data from DataBus, writes robot.pwm[8].
class MotorControl {
public:
    void Init();
    void Update();   // call at 50Hz from ControlTask

private:
    void float_ctrl();
    void angle_ctrl();
    bool calculate_body_outputs(const BodyCommand &command,
                                bool float_enabled,
                                bool angle_enabled,
                                const MotionTuning &tuning,
                                float mixed_output[8],
                                int32_t pwm_output[8],
                                bool &horizontal_saturated,
                                bool &vertical_saturated);
    void apply_pwm_slew(const int32_t desired_pwm[8],
                        const int32_t current_pwm[8],
                        const MotionTuning &tuning,
                        uint32_t now,
                        int32_t pwm_output[8]);
    void set_output_neutral();
    void copy_manual_pwm();

    PID depth_pid_;
    PID roll_out_pid_, roll_in_pid_;
    PID pitch_out_pid_, pitch_in_pid_;
    PID yaw_out_pid_, yaw_in_pid_;

    struct { float depth = 0, roll = 0, pitch = 0, yaw = 0; } pwm_comp_;
    float target_roll_v_ = 0, target_pitch_v_ = 0, target_yaw_v_ = 0;

    int32_t InitPWM_ = 1500;
    uint32_t last_slew_update_ms_ = 0;
};
