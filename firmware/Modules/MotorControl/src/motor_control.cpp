#include "motor_control.h"
#include "main.h"
#include "robot_data.hpp"
#include "thruster_config.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <algorithm>
#include <cmath>

#define MC_PI 3.14159265358979323846f

namespace {

constexpr float kHorizontalSpanUs = 450.0F;
constexpr float kVerticalSpanUs = 350.0F;
constexpr uint8_t kNeutralReasonCommand = 1U;
constexpr uint8_t kNeutralReasonFault = 7U;
constexpr uint8_t kNeutralReasonDepthSensor = 8U;

int32_t add_deadzone_compensation(int32_t pwm, int32_t neutral,
                                  uint16_t compensation_us)
{
    if (pwm > neutral) {
        return pwm + static_cast<int32_t>(compensation_us);
    }
    if (pwm < neutral) {
        return pwm - static_cast<int32_t>(compensation_us);
    }
    return pwm;
}

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
    for (const auto &thruster : thruster_config::kThrusters) {
        robot.pwm[thruster.channel] = neutral;
    }
}

void neutralize_computed_output_locked(int32_t neutral)
{
    set_all_pwm_neutral(neutral);
    for (float &output : robot.mixed_output)
    {
        output = 0.0F;
    }
    robot.horizontal_saturated = false;
    robot.vertical_saturated = false;
    mark_pwm_output_updated(robot);
}

bool body_command_is_finite(const BodyCommand &command)
{
    return std::isfinite(command.surge) &&
           std::isfinite(command.sway) &&
           std::isfinite(command.heave) &&
           std::isfinite(command.roll) &&
           std::isfinite(command.pitch) &&
           std::isfinite(command.yaw);
}

bool body_commands_equal(const BodyCommand &lhs, const BodyCommand &rhs)
{
    return lhs.surge == rhs.surge &&
           lhs.sway == rhs.sway &&
           lhs.heave == rhs.heave &&
           lhs.roll == rhs.roll &&
           lhs.pitch == rhs.pitch &&
           lhs.yaw == rhs.yaw;
}

} // namespace

void MotorControl::Init() {
    // PID gains (kp, ki, kd, KpMax, KiMax, KdMax, OutMax) — from reference
    DepthPidTuning depth_tuning{};
    uint32_t depth_tuning_generation = 0U;
    taskENTER_CRITICAL();
    depth_tuning = robot.depth_pid_tuning;
    depth_tuning_generation = robot.depth_pid_tuning_generation;
    taskEXIT_CRITICAL();
    depth_pid_.PIDInfo = PID_Regulator_t(
        depth_tuning.kp,
        depth_tuning.ki,
        depth_tuning.kd,
        depth_tuning.p_limit_us,
        depth_tuning.i_limit_us,
        depth_tuning.d_limit_us,
        depth_tuning.output_limit_us);
    roll_out_pid_.PIDInfo  = PID_Regulator_t(0.5f, 0.002f, 1, 5, 2.5f, 2.5f, 10);
    roll_in_pid_.PIDInfo   = PID_Regulator_t(8, 0, 0, 100, 50, 50, 200);
    pitch_out_pid_.PIDInfo = PID_Regulator_t(1, 0.005f, 1, 5, 2.5f, 2.5f, 10);
    pitch_in_pid_.PIDInfo  = PID_Regulator_t(8, 0, 0, 100, 50, 50, 200);
    yaw_out_pid_.PIDInfo   = PID_Regulator_t(2, 0.01f, 2, 10, 5, 5, 20);
    yaw_in_pid_.PIDInfo    = PID_Regulator_t(5, 0, 0, 200, 100, 100, 400);

    InitPWM_ = ROBOT_PWM_NEUTRAL_US;
    last_slew_update_ms_ = HAL_GetTick();
    last_depth_tuning_generation_ = depth_tuning_generation;
    last_depth_control_generation_ = 0xFFFFFFFFU;
    last_depth_session_generation_ = 0xFFFFFFFFU;
    last_depth_sample_generation_ = 0xFFFFFFFFU;
}

bool MotorControl::float_ctrl(float target_depth_cm,
                              float depth_m,
                              uint32_t sample_generation,
                              uint32_t control_generation,
                              uint32_t session_generation,
                              uint32_t tuning_generation,
                              const DepthPidTuning &tuning) {
    if (!depth_pid_tuning_is_valid(tuning) ||
        !std::isfinite(target_depth_cm) ||
        !std::isfinite(depth_m)) {
        return false;
    }

    const bool new_session =
        session_generation != last_depth_session_generation_;
    if (new_session) {
        roll_out_pid_.Reset();
        roll_in_pid_.Reset();
        pitch_out_pid_.Reset();
        pitch_in_pid_.Reset();
        target_roll_v_ = 0.0F;
        target_pitch_v_ = 0.0F;
        pwm_comp_.roll = 0.0F;
        pwm_comp_.pitch = 0.0F;
        last_depth_session_generation_ = session_generation;
    }

    if (new_session ||
        control_generation != last_depth_control_generation_ ||
        tuning_generation != last_depth_tuning_generation_) {
        depth_pid_.PIDInfo = PID_Regulator_t(
            tuning.kp,
            tuning.ki,
            tuning.kd,
            tuning.p_limit_us,
            tuning.i_limit_us,
            tuning.d_limit_us,
            tuning.output_limit_us);
        depth_pid_.Reset();
        pwm_comp_.depth = 0.0F;
        last_depth_sample_generation_ = 0xFFFFFFFFU;
        last_depth_control_generation_ = control_generation;
        last_depth_tuning_generation_ = tuning_generation;
    }

    // The pressure sensor publishes at 10 Hz while MotorControl runs at 50 Hz.
    // Update the discrete PID exactly once for each successful sensor sample.
    if (sample_generation != last_depth_sample_generation_) {
        pwm_comp_.depth =
            depth_pid_.PIDCalc(target_depth_cm, depth_m * 100.0F);
        if (!std::isfinite(pwm_comp_.depth)) {
            depth_pid_.Reset();
            pwm_comp_.depth = 0.0F;
            return false;
        }
        last_depth_sample_generation_ = sample_generation;
    }

    // Outer loops at ~66Hz (every 3rd cycle)
    if (robot.loop_count % 3 == 0) {
        target_roll_v_  = roll_out_pid_.PIDCalc(0.0f, robot.roll);
        target_pitch_v_ = pitch_out_pid_.PIDCalc(0.0f, robot.pitch);
    }
    // Inner loops at full rate
    pwm_comp_.roll  = roll_in_pid_.PIDCalc(target_roll_v_, -robot.roll_v);
    pwm_comp_.pitch = pitch_in_pid_.PIDCalc(target_pitch_v_, robot.pitch_v);
    return std::isfinite(pwm_comp_.roll) &&
           std::isfinite(pwm_comp_.pitch);
}

void MotorControl::angle_ctrl() {
    if (robot.loop_count % 3 == 0) {
        float yaw_diff = normalize_angle(robot.yaw - robot.target_yaw);
        target_yaw_v_ = yaw_out_pid_.PIDCalc(0.0f, yaw_diff);
    }
    pwm_comp_.yaw = yaw_in_pid_.PIDCalc(target_yaw_v_, robot.yaw_v);
}

bool MotorControl::calculate_body_outputs(
    const BodyCommand &command,
    bool float_enabled,
    bool angle_enabled,
    const MotionTuning &tuning,
    float mixed_output[8],
    int32_t pwm_output[8],
    bool &horizontal_saturated,
    bool &vertical_saturated)
{
    if (!body_command_is_valid(command))
    {
        return false;
    }

    if (!motion_tuning_is_valid(tuning))
    {
        return false;
    }

    if (!float_enabled)
    {
        pwm_comp_.depth = 0.0F;
        pwm_comp_.roll = 0.0F;
        pwm_comp_.pitch = 0.0F;
    }

    if (angle_enabled)
    {
        angle_ctrl();
    }
    else
    {
        pwm_comp_.yaw = 0.0F;
    }

    BodyCommand combined = command;
    combined.heave += pwm_comp_.depth / kVerticalSpanUs;
    combined.roll += pwm_comp_.roll / kVerticalSpanUs;
    combined.pitch += pwm_comp_.pitch / kVerticalSpanUs;
    combined.yaw += pwm_comp_.yaw / kHorizontalSpanUs;
    if (!body_command_is_finite(combined))
    {
        return false;
    }

    float axes[ROBOT_BODY_AXIS_COUNT] = {
        combined.surge,
        combined.sway,
        combined.heave,
        combined.roll,
        combined.pitch,
        combined.yaw,
    };
    bool axis_limited[ROBOT_BODY_AXIS_COUNT] = {
        false, false, false, false, false, false,
    };
    for (uint8_t axis = 0U; axis < ROBOT_BODY_AXIS_COUNT; ++axis)
    {
        const float scaled = axes[axis] * tuning.axis_gain[axis];
        const float limited = std::clamp(
            scaled,
            -tuning.axis_max_output[axis],
            tuning.axis_max_output[axis]);
        axis_limited[axis] = limited != scaled;
        axes[axis] = limited * tuning.global_multiplier;
    }
    combined = BodyCommand{
        axes[0], axes[1], axes[2], axes[3], axes[4], axes[5],
    };

    if (body_command_is_zero(combined))
    {
        for (uint8_t channel = 0U; channel < 8U; ++channel)
        {
            mixed_output[channel] = 0.0F;
            pwm_output[channel] = InitPWM_;
        }
        horizontal_saturated =
            axis_limited[0] || axis_limited[1] || axis_limited[5];
        vertical_saturated =
            axis_limited[2] || axis_limited[3] || axis_limited[4];
        return true;
    }

    float raw_output[8] = {0.0F};
    float horizontal_max = 0.0F;
    float vertical_max = 0.0F;

    for (const auto &thruster : thruster_config::kThrusters)
    {
        float raw = 0.0F;
        if (thruster.orientation ==
            thruster_config::Orientation::HorizontalDiagonal)
        {
            raw =
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::SurgeX)) *
                    combined.surge +
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::SwayY)) *
                    combined.sway +
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::YawZ)) *
                    combined.yaw;
            horizontal_max = std::max(horizontal_max, std::fabs(raw));
        }
        else
        {
            raw =
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::HeaveZ)) *
                    combined.heave +
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::RollX)) *
                    combined.roll +
                static_cast<float>(thruster_config::axis_direction(
                    thruster, thruster_config::ControlAxis::PitchY)) *
                    combined.pitch;
            vertical_max = std::max(vertical_max, std::fabs(raw));
        }

        if (!std::isfinite(raw))
        {
            return false;
        }
        raw_output[thruster.channel] = raw;
    }

    const bool horizontal_group_saturated = horizontal_max > 1.0F;
    const bool vertical_group_saturated = vertical_max > 1.0F;
    horizontal_saturated =
        horizontal_group_saturated ||
        axis_limited[0] || axis_limited[1] || axis_limited[5];
    vertical_saturated =
        vertical_group_saturated ||
        axis_limited[2] || axis_limited[3] || axis_limited[4];
    const float horizontal_scale =
        horizontal_group_saturated ? (1.0F / horizontal_max) : 1.0F;
    const float vertical_scale =
        vertical_group_saturated ? (1.0F / vertical_max) : 1.0F;

    for (const auto &thruster : thruster_config::kThrusters)
    {
        const bool horizontal =
            thruster.orientation ==
            thruster_config::Orientation::HorizontalDiagonal;
        const float scale = horizontal ? horizontal_scale : vertical_scale;
        const float span = horizontal ? kHorizontalSpanUs : kVerticalSpanUs;
        const float normalized = raw_output[thruster.channel] * scale;
        if (!std::isfinite(normalized) ||
            normalized < -1.0F || normalized > 1.0F)
        {
            return false;
        }

        mixed_output[thruster.channel] = normalized;
        const float pwm_delta_f = normalized * span;
        const int32_t pwm_delta = static_cast<int32_t>(
            pwm_delta_f >= 0.0F ? pwm_delta_f + 0.5F
                                : pwm_delta_f - 0.5F);
        int32_t pwm = InitPWM_;
        if (pwm_delta != 0)
        {
            pwm += static_cast<int32_t>(thruster.neutral_trim_us) +
                   pwm_delta;
            pwm = add_deadzone_compensation(
                pwm, InitPWM_, thruster.deadzone_compensation_us);
        }
        pwm_output[thruster.channel] = clamp_motor_pwm(pwm);
    }

    return true;
}

void MotorControl::apply_pwm_slew(const int32_t desired_pwm[8],
                                  const int32_t current_pwm[8],
                                  const MotionTuning &tuning,
                                  uint32_t now,
                                  int32_t pwm_output[8])
{
    uint32_t elapsed_ms = now - last_slew_update_ms_;
    if (elapsed_ms == 0U) elapsed_ms = 20U;
    if (elapsed_ms > 100U) elapsed_ms = 100U;
    uint32_t max_step =
        (static_cast<uint32_t>(tuning.pwm_slew_rate_us_per_s) *
         elapsed_ms + 999U) / 1000U;
    if (max_step == 0U) max_step = 1U;

    for (uint8_t channel = 0U; channel < 8U; ++channel)
    {
        const int32_t current = current_pwm[channel];
        const int32_t desired = desired_pwm[channel];
        if (desired > current)
        {
            pwm_output[channel] = std::min(
                desired, current + static_cast<int32_t>(max_step));
        }
        else if (desired < current)
        {
            pwm_output[channel] = std::max(
                desired, current - static_cast<int32_t>(max_step));
        }
        else
        {
            pwm_output[channel] = desired;
        }
    }
    last_slew_update_ms_ = now;
}

void MotorControl::set_output_neutral() {
    force_body_output_neutral(robot);
}

void MotorControl::copy_manual_pwm() {
    for (const auto &thruster : thruster_config::kThrusters) {
        robot.pwm[thruster.channel] =
            clamp_motor_pwm(robot.manual_pwm[thruster.channel]);
    }
    mark_pwm_output_updated(robot);
}

void MotorControl::Update() {
    taskENTER_CRITICAL();
    robot.loop_count++;
    taskEXIT_CRITICAL();

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
        const bool live_unsafe =
            robot.estop_locked ||
            robot.state == RobotState::DISARMED ||
            robot.state == RobotState::COMM_LOST ||
            robot.state == RobotState::EMERGENCY_STOP ||
            robot.state == RobotState::FAULT;
        if (live_unsafe)
        {
            robot.control_enable = false;
            robot.manual_pwm_enabled = false;
            robot.float_enabled = false;
            robot.depth_command_last_ms = 0U;
            robot.angle_enabled = false;
            robot.body_control_enabled = false;
            robot.active_test_channel = 0xFF;
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    /* ARMED_IDLE: no PID output, all neutral */
    if (state == RobotState::ARMED_IDLE)
    {
        taskENTER_CRITICAL();
        if (robot.state == RobotState::ARMED_IDLE)
        {
            robot.manual_pwm_enabled = false;
            robot.body_control_enabled = false;
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    /* MANUAL_TEST: single-channel PWM only */
    if (state == RobotState::MANUAL_TEST)
    {
        taskENTER_CRITICAL();
        if (robot.state != RobotState::MANUAL_TEST)
        {
            taskEXIT_CRITICAL();
            return;
        }

        invalidate_body_command(robot);
        robot.body_control_enabled = false;
        const uint8_t active_ch = robot.active_test_channel;
        if (robot.manual_pwm_enabled && active_ch < 8U)
        {
            /* Only copy the active channel; all others stay neutral */
            const int32_t val =
                clamp_motor_pwm(robot.manual_pwm[active_ch]);
            for (const auto &thruster : thruster_config::kThrusters) {
                robot.pwm[thruster.channel] = InitPWM_;
            }
            robot.pwm[active_ch] = val;
            mark_pwm_output_updated(robot);
        }
        else
        {
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    /* === ARMED_ACTIVE: unified body command + PID correction === */

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
        if (robot.state == RobotState::ARMED_ACTIVE &&
            !robot.control_enable)
        {
            robot.manual_pwm_enabled = false;
            robot.float_enabled = false;
            robot.depth_command_last_ms = 0U;
            robot.angle_enabled = false;
            robot.body_control_enabled = false;
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    if (manual_pwm_enabled) {
        bool continue_body_control = false;
        taskENTER_CRITICAL();
        if (robot.state != RobotState::ARMED_ACTIVE ||
            !robot.manual_pwm_enabled ||
            robot.manual_pwm_last_ms != manual_pwm_last_ms)
        {
            taskEXIT_CRITICAL();
            return;
        }

        const uint32_t elapsed_ms =
            HAL_GetTick() - robot.manual_pwm_last_ms;
        if (elapsed_ms <= ROBOT_MANUAL_PWM_TIMEOUT_MS)
        {
            invalidate_body_command(robot);
            copy_manual_pwm();
            taskEXIT_CRITICAL();
            return;
        }

        robot.manual_pwm_enabled = false;
        continue_body_control =
            robot.float_enabled ||
            robot.angle_enabled ||
            robot.body_control_enabled;
        if (!continue_body_control)
        {
            set_output_neutral();
        }
        taskEXIT_CRITICAL();

        if (!continue_body_control)
        {
            return;
        }
    }

    BodyCommand command{};
    BodyCommandSource source = BodyCommandSource::None;
    uint16_t sequence = 0U;
    uint32_t last_ms = 0U;
    uint32_t timeout_ms = ROBOT_BODY_COMMAND_TIMEOUT_MS;
    bool command_valid = false;
    bool float_enabled = false;
    bool angle_enabled = false;
    bool body_control_enabled = false;
    MotionTuning tuning{};
    uint32_t tuning_generation = 0U;
    DepthPidTuning depth_tuning{};
    uint32_t depth_tuning_generation = 0U;
    uint32_t depth_control_generation = 0U;
    uint32_t depth_session_generation = 0U;
    uint32_t depth_command_last_ms = 0U;
    uint32_t depth_sample_generation = 0U;
    uint32_t depth_sample_ms = 0U;
    bool depth_sensor_ready = false;
    bool depth_sample_valid = false;
    float depth_m = 0.0F;
    float target_depth_cm = 0.0F;
    taskENTER_CRITICAL();
    command = robot.body_command;
    source = robot.body_command_source;
    sequence = robot.body_command_sequence;
    last_ms = robot.body_command_last_ms;
    timeout_ms = robot.body_command_timeout_ms;
    command_valid = robot.body_command_valid;
    float_enabled = robot.float_enabled;
    angle_enabled = robot.angle_enabled;
    body_control_enabled = robot.body_control_enabled;
    tuning = robot.motion_tuning;
    tuning_generation = robot.motion_tuning_generation;
    depth_tuning = robot.depth_pid_tuning;
    depth_tuning_generation = robot.depth_pid_tuning_generation;
    depth_control_generation = robot.depth_control_generation;
    depth_session_generation = robot.depth_hold_session_generation;
    depth_command_last_ms = robot.depth_command_last_ms;
    depth_sample_generation = robot.depth_sample_generation;
    depth_sample_ms = robot.depth_sample_ms;
    depth_sensor_ready = robot.depth_sensor_ready;
    depth_sample_valid = robot.depth_sample_valid;
    depth_m = robot.depth_m;
    target_depth_cm = robot.target_depth_cm;
    taskEXIT_CRITICAL();

    if (!body_control_enabled && !float_enabled && !angle_enabled)
    {
        taskENTER_CRITICAL();
        if (robot.state == RobotState::ARMED_ACTIVE &&
            !robot.body_control_enabled &&
            !robot.float_enabled &&
            !robot.angle_enabled)
        {
            robot.control_enable = false;
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    if (command_valid && !body_command_is_valid(command))
    {
        taskENTER_CRITICAL();
        if (robot.body_command_valid &&
            robot.body_command_source == source &&
            robot.body_command_sequence == sequence)
        {
            robot.control_enable = false;
            robot.float_enabled = false;
            robot.depth_command_last_ms = 0U;
            robot.angle_enabled = false;
            robot.body_control_enabled = false;
            robot.last_neutral_reason = kNeutralReasonCommand;
            robot.state = RobotState::ARMED_IDLE;
            robot.state_changed_ms = HAL_GetTick();
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (float_enabled)
    {
        const bool depth_lease_fresh =
            (now - depth_command_last_ms) <= ROBOT_COMMAND_TIMEOUT_MS;
        const bool depth_ready =
            depth_sensor_ready &&
            depth_sample_valid &&
            depth_sample_generation != 0U &&
            std::isfinite(depth_m) &&
            depth_m >= ROBOT_DEPTH_VALID_MIN_M &&
            depth_m <= ROBOT_DEPTH_VALID_MAX_M &&
            std::isfinite(target_depth_cm) &&
            target_depth_cm >= ROBOT_DEPTH_TARGET_MIN_CM &&
            target_depth_cm <= ROBOT_DEPTH_TARGET_MAX_CM &&
            (now - depth_sample_ms) <= ROBOT_DEPTH_SAMPLE_MAX_AGE_MS;
        const bool depth_control_ok =
            depth_lease_fresh &&
            depth_ready &&
            float_ctrl(target_depth_cm,
                       depth_m,
                       depth_sample_generation,
                       depth_control_generation,
                       depth_session_generation,
                       depth_tuning_generation,
                       depth_tuning);
        if (!depth_control_ok)
        {
            taskENTER_CRITICAL();
            if (robot.state == RobotState::ARMED_ACTIVE &&
                robot.float_enabled &&
                robot.depth_control_generation ==
                    depth_control_generation &&
                robot.depth_hold_session_generation ==
                    depth_session_generation &&
                robot.depth_command_last_ms ==
                    depth_command_last_ms &&
                robot.depth_sample_generation ==
                    depth_sample_generation &&
                robot.depth_sample_ms == depth_sample_ms &&
                robot.depth_sensor_ready == depth_sensor_ready &&
                robot.depth_sample_valid == depth_sample_valid)
            {
                robot.control_enable = false;
                robot.float_enabled = false;
                robot.depth_command_last_ms = 0U;
                robot.angle_enabled = false;
                robot.body_control_enabled = false;
                reset_depth_control_runtime(robot);
                const uint8_t reason =
                    !depth_lease_fresh
                        ? kNeutralReasonCommand
                        : depth_ready
                        ? kNeutralReasonFault
                        : kNeutralReasonDepthSensor;
                robot.depth_control_fault_reason = reason;
                robot.last_neutral_reason = reason;
                robot.state = RobotState::ARMED_IDLE;
                robot.state_changed_ms = now;
                set_output_neutral();
            }
            else
            {
                /*
                 * A new sample or control command arrived after the snapshot.
                 * Do not fail the newer state from stale evidence, and do not
                 * publish the previous cycle's thrust while it is rechecked.
                 */
                neutralize_computed_output_locked(InitPWM_);
            }
            taskEXIT_CRITICAL();
            return;
        }
    }

    if (command_valid &&
        !body_command_is_zero(command) &&
        (now - last_ms) > timeout_ms)
    {
        taskENTER_CRITICAL();
        if (robot.body_command_valid &&
            robot.body_command_source == source &&
            robot.body_command_sequence == sequence &&
            robot.body_command_last_ms == last_ms &&
            robot.body_command_timeout_ms == timeout_ms &&
            body_commands_equal(robot.body_command, command))
        {
            robot.control_enable = false;
            robot.float_enabled = false;
            robot.depth_command_last_ms = 0U;
            robot.angle_enabled = false;
            robot.body_control_enabled = false;
            robot.last_neutral_reason = kNeutralReasonCommand;
            robot.state = RobotState::ARMED_IDLE;
            robot.state_changed_ms = now;
            set_output_neutral();
        }
        taskEXIT_CRITICAL();
        return;
    }

    const BodyCommand active_command =
        command_valid ? command : BodyCommand{};

    float mixed_output[8] = {0.0F};
    int32_t pwm_output[8] = {
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
        ROBOT_PWM_NEUTRAL_US, ROBOT_PWM_NEUTRAL_US,
    };
    bool horizontal_saturated = false;
    bool vertical_saturated = false;
    if (!calculate_body_outputs(active_command, float_enabled, angle_enabled,
                                tuning, mixed_output, pwm_output,
                                horizontal_saturated, vertical_saturated))
    {
        taskENTER_CRITICAL();
        robot.last_neutral_reason = kNeutralReasonCommand;
        robot.control_enable = false;
        robot.float_enabled = false;
        robot.depth_command_last_ms = 0U;
        robot.angle_enabled = false;
        robot.body_control_enabled = false;
        robot.state = RobotState::ARMED_IDLE;
        robot.state_changed_ms = HAL_GetTick();
        set_output_neutral();
        taskEXIT_CRITICAL();
        return;
    }

    taskENTER_CRITICAL();
    const bool safe_state =
        robot.state == RobotState::ARMED_ACTIVE &&
        !robot.estop_locked &&
        robot.control_enable &&
        (robot.body_control_enabled ||
         robot.float_enabled ||
         robot.angle_enabled);
    const bool command_unchanged =
        robot.body_command_valid == command_valid &&
        (!command_valid ||
         (robot.body_command_source == source &&
          robot.body_command_sequence == sequence &&
          robot.body_command_last_ms == last_ms &&
          robot.body_command_timeout_ms == timeout_ms &&
          body_commands_equal(robot.body_command, command))) &&
        robot.body_control_enabled == body_control_enabled &&
        robot.float_enabled == float_enabled &&
        robot.angle_enabled == angle_enabled &&
        robot.motion_tuning_generation == tuning_generation &&
        (!float_enabled ||
         (robot.depth_pid_tuning_generation ==
              depth_tuning_generation &&
          robot.depth_control_generation ==
              depth_control_generation &&
          robot.depth_hold_session_generation ==
              depth_session_generation &&
          robot.depth_command_last_ms ==
              depth_command_last_ms &&
          robot.depth_sample_generation ==
              depth_sample_generation &&
          robot.depth_sample_ms == depth_sample_ms &&
          robot.depth_sensor_ready == depth_sensor_ready &&
          robot.depth_sample_valid == depth_sample_valid &&
          robot.target_depth_cm == target_depth_cm));

    if (!safe_state)
    {
        set_output_neutral();
    }
    else if (!command_unchanged)
    {
        // A newer valid command arrived while the previous one was being
        // calculated. Never publish the stale result; keep the new command
        // valid so the next control cycle can calculate it.
        neutralize_computed_output_locked(InitPWM_);
    }
    else
    {
        int32_t current_pwm[8];
        int32_t slewed_pwm[8];
        for (uint8_t channel = 0U; channel < 8U; ++channel)
        {
            current_pwm[channel] = robot.pwm[channel];
        }
        apply_pwm_slew(
            pwm_output, current_pwm, tuning, now, slewed_pwm);
        for (const auto &thruster : thruster_config::kThrusters)
        {
            const uint8_t channel = thruster.channel;
            robot.mixed_output[channel] = mixed_output[channel];
            robot.pwm[channel] = slewed_pwm[channel];
        }
        robot.horizontal_saturated = horizontal_saturated;
        robot.vertical_saturated = vertical_saturated;
        if (float_enabled)
        {
            robot.depth_active_setpoint_cm = target_depth_cm;
            robot.depth_control_measured_cm = depth_m * 100.0F;
            robot.depth_error_cm =
                target_depth_cm - robot.depth_control_measured_cm;
            robot.depth_pid_p_us = depth_pid_.PIDInfo.componentKp;
            robot.depth_pid_i_us = depth_pid_.PIDInfo.componentKi;
            robot.depth_pid_d_us = depth_pid_.PIDInfo.componentKd;
            robot.depth_pid_output_us = pwm_comp_.depth;
            robot.depth_pid_saturated =
                std::fabs(pwm_comp_.depth) >=
                    (depth_tuning.output_limit_us - 0.001F);
            robot.depth_control_fault_reason = 0U;
        }
        mark_pwm_output_updated(robot);
    }
    taskEXIT_CRITICAL();
}
