#include "motor_control.h"
#include "robot_data.hpp"
#include <cmath>

#define MC_PI 3.14159265358979323846f

// Motion states (match RobotData::motion_state encoding)
enum { ST_STOP = 0, ST_FLOAT, ST_FRONT, ST_BACK, ST_LEFT, ST_RIGHT, ST_CLOCKWISE, ST_ANTICLOCKWISE };

static float normalize_angle(float angle) {
    angle = fmodf(angle + MC_PI, 2.0f * MC_PI);
    if (angle < 0) angle += 2.0f * MC_PI;
    return angle - MC_PI;
}

void MotorControl::Init() {
    // PID gains (kp, ki, kd, KpMax, KiMax, KdMax, OutMax) — from reference
    depth_pid_.PIDInfo     = PID_Regulator_t(10, 0.02f, 10, 100, 50, 50, 200);
    roll_out_pid_.PIDInfo  = PID_Regulator_t(0.5f, 0.002f, 1, 5, 2.5f, 2.5f, 10);
    roll_in_pid_.PIDInfo   = PID_Regulator_t(8, 0, 0, 100, 50, 50, 200);
    pitch_out_pid_.PIDInfo = PID_Regulator_t(1, 0.005f, 1, 5, 2.5f, 2.5f, 10);
    pitch_in_pid_.PIDInfo  = PID_Regulator_t(8, 0, 0, 100, 50, 50, 200);
    yaw_out_pid_.PIDInfo   = PID_Regulator_t(2, 0.01f, 2, 10, 5, 5, 20);
    yaw_in_pid_.PIDInfo    = PID_Regulator_t(5, 0, 0, 200, 100, 100, 400);

    // Motor layout (from reference)
    const int8_t  sign[8]  = {1, -1, 1, 1, -1, -1, 1, -1};
    const uint8_t inid[4]  = {1, 2, 6, 5};
    const uint8_t outid[4] = {0, 3, 7, 4};
    for (int i = 0; i < 8; i++) { Sign_[i] = sign[i]; Compensation_[i] = 50; }
    for (int i = 0; i < 4; i++) { InID_[i] = inid[i]; OutID_[i] = outid[i]; }

    InitPWM_ = 1610;

    // Vertical neutral PWM with small trim (verbatim from reference)
    FloatPWM_[0] = InitPWM_;
    FloatPWM_[1] = InitPWM_ - Sign_[InID_[1]] * 100;
    FloatPWM_[2] = InitPWM_;
    FloatPWM_[3] = InitPWM_ - Sign_[InID_[3]] * 90;

    const int32_t li = 80, la = 80, ro = 40;  // longitudinal / lateral / rotate speed

    for (int i = 0; i < 4; i++) {
        int32_t s = Sign_[OutID_[i]];
        state_pwm_map_[ST_STOP][i]  = InitPWM_;
        state_pwm_map_[ST_FLOAT][i] = InitPWM_;
        state_pwm_map_[ST_FRONT][i] = InitPWM_ - s * li;
        state_pwm_map_[ST_BACK][i]  = InitPWM_ + s * li;
        state_pwm_map_[ST_CLOCKWISE][i]     = (i < 2) ? (InitPWM_ - s * ro) : (InitPWM_ + s * ro);
        state_pwm_map_[ST_ANTICLOCKWISE][i] = (i < 2) ? (InitPWM_ + s * ro) : (InitPWM_ - s * ro);
    }
    // Left / Right (per-index sign pattern from reference)
    state_pwm_map_[ST_LEFT][0]  = InitPWM_ + Sign_[OutID_[0]] * la;
    state_pwm_map_[ST_LEFT][1]  = InitPWM_ - Sign_[OutID_[1]] * la;
    state_pwm_map_[ST_LEFT][2]  = InitPWM_ - Sign_[OutID_[2]] * la;
    state_pwm_map_[ST_LEFT][3]  = InitPWM_ + Sign_[OutID_[3]] * la;
    state_pwm_map_[ST_RIGHT][0] = InitPWM_ - Sign_[OutID_[0]] * la;
    state_pwm_map_[ST_RIGHT][1] = InitPWM_ + Sign_[OutID_[1]] * la;
    state_pwm_map_[ST_RIGHT][2] = InitPWM_ + Sign_[OutID_[2]] * la;
    state_pwm_map_[ST_RIGHT][3] = InitPWM_ - Sign_[OutID_[3]] * la;
}

void MotorControl::float_ctrl() {
    // Depth PID at full rate (depth_m -> cm)
    pwm_comp_.depth = depth_pid_.PIDCalc(robot.target_depth_cm, robot.depth_m * 100.0f);

    // Outer loops at ~66Hz (every 3rd cycle)
    if (robot.loop_count % 3 == 0) {
        target_roll_v_  = roll_out_pid_.PIDCalc(0.0f, robot.roll);
        target_pitch_v_ = pitch_out_pid_.PIDCalc(0.0f, robot.pitch);
    }
    // Inner loops at full rate
    pwm_comp_.roll  = roll_in_pid_.PIDCalc(target_roll_v_, -robot.roll_v);
    pwm_comp_.pitch = pitch_in_pid_.PIDCalc(target_pitch_v_, robot.pitch_v);
}

void MotorControl::angle_ctrl() {
    if (robot.loop_count % 3 == 0) {
        float yaw_diff = normalize_angle(robot.yaw - robot.target_yaw);
        target_yaw_v_ = yaw_out_pid_.PIDCalc(0.0f, yaw_diff);
    }
    pwm_comp_.yaw = yaw_in_pid_.PIDCalc(target_yaw_v_, robot.yaw_v);
}

void MotorControl::vertical_allocation() {
    if (!robot.float_enabled) return;
    float_ctrl();

    constexpr int8_t factors[4][3] = {
        {-1, -1, -1},  // Motor 0
        {-1, -1,  1},  // Motor 1
        {-1,  1, -1},  // Motor 2
        {-1,  1,  1}   // Motor 3
    };

    for (int i = 0; i < 4; i++) {
        uint8_t idx  = InID_[i];
        int8_t  sign = Sign_[idx];
        int32_t base = FloatPWM_[i];
        int32_t comp = Compensation_[idx];

        float adj = sign * (pwm_comp_.depth * factors[i][0] +
                            pwm_comp_.roll  * factors[i][1] +
                            pwm_comp_.pitch * factors[i][2]);
        int32_t pwm = base - (int32_t)adj;

        if (pwm > InitPWM_)      pwm += comp;
        else if (pwm < InitPWM_) pwm -= comp;
        robot.pwm[idx] = pwm;
    }
}

void MotorControl::horizontal_allocation() {
    if (!robot.float_enabled) return;

    float yaw_comp = 0;
    if (robot.angle_enabled) {
        angle_ctrl();
        yaw_comp = pwm_comp_.yaw;
    }

    int state = robot.motion_state;
    if (state < 0 || state >= 8) state = ST_STOP;

    for (int i = 0; i < 4; i++) {
        uint8_t idx  = OutID_[i];
        int32_t sign = Sign_[idx];
        int32_t comp = Compensation_[idx];
        int32_t base = state_pwm_map_[state][i];

        float adj = ((i < 2) ? sign : -sign) * yaw_comp;
        int32_t pwm = base + (int32_t)adj;

        if (pwm > InitPWM_)      pwm += comp;
        else if (pwm < InitPWM_) pwm -= comp;
        robot.pwm[idx] = pwm;
    }
}

void MotorControl::Update() {
    robot.loop_count++;

    if (robot.float_enabled) {
        vertical_allocation();
        horizontal_allocation();
    } else {
        for (int i = 0; i < 8; i++)
            robot.pwm[i] = InitPWM_;
    }
}
