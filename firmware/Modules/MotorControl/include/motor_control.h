#pragma once
#include "PID.h"
#include <cstdint>

// 8-motor PID cascade + thrust allocation.
// Reads sensor/AHRS/target data from DataBus, writes robot.pwm[8].
class MotorControl {
public:
    void Init();
    void Update();   // call at 50Hz from ControlTask

private:
    void float_ctrl();
    void angle_ctrl();
    void vertical_allocation();
    void horizontal_allocation();
    void apply_pwm_limits();
    void set_output_neutral();
    void copy_manual_pwm();

    PID depth_pid_;
    PID roll_out_pid_, roll_in_pid_;
    PID pitch_out_pid_, pitch_in_pid_;
    PID yaw_out_pid_, yaw_in_pid_;

    struct { float depth = 0, roll = 0, pitch = 0, yaw = 0; } pwm_comp_;
    float target_roll_v_ = 0, target_pitch_v_ = 0, target_yaw_v_ = 0;

    int32_t InitPWM_ = 1500;
    int8_t  Sign_[8];
    uint8_t InID_[4];
    uint8_t OutID_[4];
    int32_t Compensation_[8];
    int32_t FloatPWM_[4];
    int32_t state_pwm_map_[8][4];
};
