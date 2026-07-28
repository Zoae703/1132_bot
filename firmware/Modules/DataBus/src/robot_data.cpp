#include "robot_data.hpp"

#include <cmath>

RobotData robot;

bool body_command_is_valid(const BodyCommand &command)
{
    const float axes[] = {
        command.surge,
        command.sway,
        command.heave,
        command.roll,
        command.pitch,
        command.yaw,
    };
    for (const float axis : axes)
    {
        if (!std::isfinite(axis) || axis < -1.0F || axis > 1.0F)
        {
            return false;
        }
    }
    return true;
}

bool body_command_is_zero(const BodyCommand &command)
{
    return command.surge == 0.0F &&
           command.sway == 0.0F &&
           command.heave == 0.0F &&
           command.roll == 0.0F &&
           command.pitch == 0.0F &&
           command.yaw == 0.0F;
}

bool motion_tuning_is_valid(const MotionTuning &tuning)
{
    for (uint8_t axis = 0U; axis < ROBOT_BODY_AXIS_COUNT; ++axis)
    {
        if (!std::isfinite(tuning.axis_gain[axis]) ||
            tuning.axis_gain[axis] < 0.0F ||
            tuning.axis_gain[axis] > 2.0F ||
            !std::isfinite(tuning.axis_max_output[axis]) ||
            tuning.axis_max_output[axis] < 0.0F ||
            tuning.axis_max_output[axis] > 1.0F)
        {
            return false;
        }
    }

    return std::isfinite(tuning.global_multiplier) &&
           tuning.global_multiplier >= 0.0F &&
           tuning.global_multiplier <= 1.0F &&
           tuning.pwm_slew_rate_us_per_s >=
               ROBOT_PWM_SLEW_RATE_MIN_US_PER_S &&
           tuning.pwm_slew_rate_us_per_s <=
               ROBOT_PWM_SLEW_RATE_MAX_US_PER_S &&
           tuning.command_timeout_ms >=
               ROBOT_BODY_COMMAND_TIMEOUT_MIN_MS &&
           tuning.command_timeout_ms <=
               ROBOT_BODY_COMMAND_TIMEOUT_MAX_MS;
}

bool depth_pid_tuning_is_valid(const DepthPidTuning &tuning)
{
    return std::isfinite(tuning.kp) &&
           tuning.kp >= 0.0F && tuning.kp <= 100.0F &&
           std::isfinite(tuning.ki) &&
           tuning.ki >= 0.0F && tuning.ki <= 10.0F &&
           std::isfinite(tuning.kd) &&
           tuning.kd >= 0.0F && tuning.kd <= 100.0F &&
           std::isfinite(tuning.p_limit_us) &&
           tuning.p_limit_us >= 0.0F && tuning.p_limit_us <= 200.0F &&
           std::isfinite(tuning.i_limit_us) &&
           tuning.i_limit_us >= 0.0F && tuning.i_limit_us <= 200.0F &&
           std::isfinite(tuning.d_limit_us) &&
           tuning.d_limit_us >= 0.0F && tuning.d_limit_us <= 200.0F &&
           std::isfinite(tuning.output_limit_us) &&
           tuning.output_limit_us >= 1.0F &&
           tuning.output_limit_us <= 200.0F;
}

bool depth_sample_is_fresh(const RobotData &data, uint32_t now)
{
    return data.depth_sensor_ready &&
           data.depth_sample_valid &&
           data.depth_sample_generation != 0U &&
           std::isfinite(data.depth_m) &&
           data.depth_m >= ROBOT_DEPTH_VALID_MIN_M &&
           data.depth_m <= ROBOT_DEPTH_VALID_MAX_M &&
           (now - data.depth_sample_ms) <= ROBOT_DEPTH_SAMPLE_MAX_AGE_MS;
}

void reset_depth_control_runtime(RobotData &data)
{
    data.depth_control_generation++;
    data.depth_active_setpoint_cm = data.target_depth_cm;
    data.depth_control_measured_cm = data.depth_m * 100.0F;
    data.depth_error_cm = 0.0F;
    data.depth_pid_p_us = 0.0F;
    data.depth_pid_i_us = 0.0F;
    data.depth_pid_d_us = 0.0F;
    data.depth_pid_output_us = 0.0F;
    data.depth_pid_saturated = false;
    data.depth_control_fault_reason = 0U;
}

void invalidate_body_command(RobotData &data)
{
    data.body_command = BodyCommand{};
    data.body_command_source = BodyCommandSource::None;
    data.body_command_valid = false;
    for (float &output : data.mixed_output)
    {
        output = 0.0F;
    }
    data.horizontal_saturated = false;
    data.vertical_saturated = false;
}

void mark_pwm_output_updated(RobotData &data)
{
    data.pwm_output_generation++;
}

void force_body_output_neutral(RobotData &data)
{
    invalidate_body_command(data);
    for (int32_t &pwm : data.pwm)
    {
        pwm = ROBOT_PWM_NEUTRAL_US;
    }
    mark_pwm_output_updated(data);
}
