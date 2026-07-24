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
