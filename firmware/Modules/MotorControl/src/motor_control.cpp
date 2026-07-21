#include "motor_control.h"
#include "main.h"
#include "robot_data.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

#define MC_PI 3.14159265358979323846f

// Motion states (match RobotData::motion_state encoding)
enum { ST_STOP = 0, ST_FLOAT, ST_FRONT, ST_BACK, ST_LEFT, ST_RIGHT, ST_CLOCKWISE, ST_ANTICLOCKWISE };

static float normalize_angle(float angle) {
    angle = fmodf(angle + MC_PI, 2.0f * MC_PI);
    if (angle < 0) angle += 2.0f * MC_PI;
    return angle - MC_PI;
}

static int32_t clamp_motor_pwm(int32_t pwm) {
    if (pwm < ROBOT_PWM_MIN_US) return ROBOT_PWM_MIN_US;
    if (pwm > ROBOT_PWM_MAX_US) return ROBOT_PWM_MAX_US;
    return pwm;
}

static void set_all_pwm_neutral(int32_t neutral) {
    for (int i = 0; i < 8; i++)
        robot.pwm[i] = neutral;
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

    InitPWM_ = ROBOT_PWM_NEUTRAL_US;

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
    float depth_m;
    taskENTER_CRITICAL();
    depth_m = robot.depth_m;
    taskEXIT_CRITICAL();
    pwm_comp_.depth = depth_pid_.PIDCalc(robot.target_depth_cm, depth_m * 100.0f);

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
        robot.pwm[idx] = clamp_motor_pwm(pwm);
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
        robot.pwm[idx] = clamp_motor_pwm(pwm);
    }
}

void MotorControl::apply_pwm_limits() {
    for (int i = 0; i < 8; i++) {
        robot.pwm[i] = clamp_motor_pwm(robot.pwm[i]);
    }
}

void MotorControl::set_output_neutral() {
    set_all_pwm_neutral(InitPWM_);
}

void MotorControl::copy_manual_pwm() {
    for (int i = 0; i < 8; i++) {
        robot.pwm[i] = clamp_motor_pwm(robot.manual_pwm[i]);
    }
}

void MotorControl::Update() {
    robot.loop_count++;

    /* === State-machine-gated PWM output === */

    RobotState state;
    bool estop_locked;
    taskENTER_CRITICAL();
    state = robot.state;
    estop_locked = robot.estop_locked;
    taskEXIT_CRITICAL();

    /* Hard stop: ESTOP locked or unsafe states → force neutral */
    if (estop_locked ||
        state == RobotState::DISARMED ||
        state == RobotState::COMM_LOST ||
        state == RobotState::EMERGENCY_STOP ||
        state == RobotState::FAULT)
    {
        taskENTER_CRITICAL();
        robot.manual_pwm_enabled = false;
        robot.float_enabled = false;
        robot.angle_enabled = false;
        robot.motion_state = ST_STOP;
        robot.active_test_channel = 0xFF;
        set_output_neutral();
        taskEXIT_CRITICAL();
        return;
    }

    /* ARMED_IDLE: no PID output, all neutral */
    if (state == RobotState::ARMED_IDLE)
    {
        taskENTER_CRITICAL();
        robot.manual_pwm_enabled = false;
        set_output_neutral();
        taskEXIT_CRITICAL();
        return;
    }

    /* MANUAL_TEST: single-channel PWM only */
    if (state == RobotState::MANUAL_TEST)
    {
        bool manual_enabled;
        uint8_t active_ch;
        taskENTER_CRITICAL();
        manual_enabled = robot.manual_pwm_enabled;
        active_ch = robot.active_test_channel;
        taskEXIT_CRITICAL();

        if (manual_enabled && active_ch < 8U)
        {
            /* Only copy the active channel; all others stay neutral */
            taskENTER_CRITICAL();
            int32_t val = robot.manual_pwm[active_ch];
            taskEXIT_CRITICAL();
            val = clamp_motor_pwm(val);

            taskENTER_CRITICAL();
            for (int i = 0; i < 8; i++)
                robot.pwm[i] = InitPWM_;
            robot.pwm[active_ch] = val;
            taskEXIT_CRITICAL();
        }
        else
        {
            taskENTER_CRITICAL();
            set_output_neutral();
            taskEXIT_CRITICAL();
        }
        return;
    }

    /* === ARMED_ACTIVE: legacy PID cascade === */

    bool control_enable;
    bool manual_pwm_enabled;
    uint32_t manual_pwm_last_ms;
    taskENTER_CRITICAL();
    control_enable = robot.control_enable;
    manual_pwm_enabled = robot.manual_pwm_enabled;
    manual_pwm_last_ms = robot.manual_pwm_last_ms;
    taskEXIT_CRITICAL();

    if (!control_enable) {
        taskENTER_CRITICAL();
        robot.manual_pwm_enabled = false;
        robot.float_enabled = false;
        robot.angle_enabled = false;
        robot.motion_state = ST_STOP;
        set_output_neutral();
        taskEXIT_CRITICAL();
        return;
    }

    if (manual_pwm_enabled) {
        uint32_t elapsed_ms = HAL_GetTick() - manual_pwm_last_ms;
        if (elapsed_ms <= ROBOT_MANUAL_PWM_TIMEOUT_MS) {
            taskENTER_CRITICAL();
            copy_manual_pwm();
            taskEXIT_CRITICAL();
            return;
        }

        taskENTER_CRITICAL();
        robot.manual_pwm_enabled = false;
        bool float_enabled = robot.float_enabled;
        if (!float_enabled) {
            set_output_neutral();
        }
        taskEXIT_CRITICAL();

        if (!float_enabled) {
            return;
        }
    }

    if (robot.float_enabled) {
        vertical_allocation();
        horizontal_allocation();
        taskENTER_CRITICAL();
        apply_pwm_limits();
        taskEXIT_CRITICAL();
    } else {
        taskENTER_CRITICAL();
        set_output_neutral();
        taskEXIT_CRITICAL();
    }
}
