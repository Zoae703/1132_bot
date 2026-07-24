#include "motor_control.h"
#include "robot_data.hpp"
#include "thruster_config.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string_view>

namespace {

using thruster_config::BodyDirection;
using thruster_config::ControlAxis;
using thruster_config::Orientation;
using thruster_config::Position;
using thruster_config::PropellerHand;
using thruster_config::RotationDirection;
using thruster_config::ThrusterConfig;
using thruster_config::Vector3;
using thruster_config::axis_direction;
using thruster_config::kCoordinateSystem;
using thruster_config::kThrusterCount;
using thruster_config::kThrusters;
using thruster_config::position_vector;
using thruster_config::torque_vector;

constexpr float kTolerance = 0.0015F;
uint32_t fake_tick = 1000U;

struct ExpectedThruster {
    uint8_t channel;
    std::string_view name;
    Position position;
    Orientation orientation;
    PropellerHand propeller_hand;
    RotationDirection rotation_above_neutral;
    Vector3 positive_force;
    int neutral_trim_us;
    int deadzone_compensation_us;
    std::array<int8_t, 6> axis_directions;
};

constexpr std::array<ExpectedThruster, 8> kExpected{{
    {0U, "horizontal_rear_right", Position::RearRight,
     Orientation::HorizontalDiagonal, PropellerHand::Normal,
     RotationDirection::Counterclockwise, {-0.707F, -0.707F, 0.0F},
     0, 50, {-1, -1, 0, 0, 0, 1}},
    {1U, "vertical_rear_right", Position::RearRight,
     Orientation::Vertical, PropellerHand::Normal,
     RotationDirection::Counterclockwise, {0.0F, 0.0F, 1.0F},
     0, 50, {0, 0, 1, 1, 1, 0}},
    {2U, "vertical_front_right", Position::FrontRight,
     Orientation::Vertical, PropellerHand::Normal,
     RotationDirection::Counterclockwise, {0.0F, 0.0F, 1.0F},
     -100, 50, {0, 0, 1, 1, -1, 0}},
    {3U, "horizontal_front_right", Position::FrontRight,
     Orientation::HorizontalDiagonal, PropellerHand::Normal,
     RotationDirection::Counterclockwise, {-0.707F, 0.707F, 0.0F},
     0, 50, {-1, 1, 0, 0, 0, 1}},
    {4U, "horizontal_front_left", Position::FrontLeft,
     Orientation::HorizontalDiagonal, PropellerHand::Reverse,
     RotationDirection::Clockwise, {0.707F, 0.707F, 0.0F},
     0, 50, {1, 1, 0, 0, 0, 1}},
    {5U, "vertical_front_left", Position::FrontLeft,
     Orientation::Vertical, PropellerHand::Reverse,
     RotationDirection::Clockwise, {0.0F, 0.0F, -1.0F},
     90, 50, {0, 0, -1, 1, 1, 0}},
    {6U, "vertical_rear_left", Position::RearLeft,
     Orientation::Vertical, PropellerHand::Reverse,
     RotationDirection::Clockwise, {0.0F, 0.0F, -1.0F},
     0, 50, {0, 0, -1, 1, -1, 0}},
    {7U, "horizontal_rear_left", Position::RearLeft,
     Orientation::HorizontalDiagonal, PropellerHand::Reverse,
     RotationDirection::Clockwise, {0.707F, -0.707F, 0.0F},
     0, 50, {1, -1, 0, 0, 0, 1}},
}};

constexpr std::array<ControlAxis, 6> kAxes{{
    ControlAxis::SurgeX,
    ControlAxis::SwayY,
    ControlAxis::HeaveZ,
    ControlAxis::RollX,
    ControlAxis::PitchY,
    ControlAxis::YawZ,
}};

bool nearly_equal(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) <= kTolerance;
}

void expect_vector(const Vector3 &actual, const Vector3 &expected)
{
    assert(nearly_equal(actual.x, expected.x));
    assert(nearly_equal(actual.y, expected.y));
    assert(nearly_equal(actual.z, expected.z));
}

Vector3 cross_product(const Vector3 &position, const Vector3 &force)
{
    return {
        position.y * force.z - position.z * force.y,
        position.z * force.x - position.x * force.z,
        position.x * force.y - position.y * force.x,
    };
}

void test_complete_thruster_table()
{
    static_assert(kThrusterCount == kExpected.size(),
                  "The vehicle must have exactly eight configured thrusters");

    for (std::size_t index = 0; index < kExpected.size(); ++index)
    {
        const ThrusterConfig &actual = kThrusters[index];
        const ExpectedThruster &expected = kExpected[index];

        assert(actual.channel == expected.channel);
        assert(actual.channel == index);
        assert(std::string_view(actual.name) == expected.name);
        assert(actual.position == expected.position);
        assert(actual.orientation == expected.orientation);
        assert(actual.propeller_hand == expected.propeller_hand);
        assert(actual.rotation_above_neutral ==
               expected.rotation_above_neutral);
        expect_vector(actual.positive_force, expected.positive_force);
        assert(actual.neutral_trim_us == expected.neutral_trim_us);
        assert(actual.deadzone_compensation_us ==
               expected.deadzone_compensation_us);
    }
}

void test_coordinate_system()
{
    static_assert(kCoordinateSystem.x_positive == BodyDirection::Forward);
    static_assert(kCoordinateSystem.y_positive == BodyDirection::Right);
    static_assert(kCoordinateSystem.z_positive == BodyDirection::Down);
}

void test_position_and_torque_geometry()
{
    constexpr std::array<Vector3, 4> kExpectedPositions{{
        {-1.0F, 1.0F, 0.0F},
        {1.0F, 1.0F, 0.0F},
        {1.0F, -1.0F, 0.0F},
        {-1.0F, -1.0F, 0.0F},
    }};

    expect_vector(position_vector(Position::RearRight),
                  kExpectedPositions[0]);
    expect_vector(position_vector(Position::FrontRight),
                  kExpectedPositions[1]);
    expect_vector(position_vector(Position::FrontLeft),
                  kExpectedPositions[2]);
    expect_vector(position_vector(Position::RearLeft),
                  kExpectedPositions[3]);

    for (const ThrusterConfig &thruster : kThrusters)
    {
        const Vector3 expected = cross_product(
            position_vector(thruster.position), thruster.positive_force);
        expect_vector(torque_vector(thruster), expected);
    }
}

void test_six_degree_of_freedom_signs()
{
    for (std::size_t thruster_index = 0;
         thruster_index < kExpected.size(); ++thruster_index)
    {
        for (std::size_t axis_index = 0; axis_index < kAxes.size();
             ++axis_index)
        {
            assert(axis_direction(kThrusters[thruster_index],
                                  kAxes[axis_index]) ==
                   kExpected[thruster_index].axis_directions[axis_index]);
        }
    }
}

void test_clockwise_is_positive_yaw()
{
    int horizontal_positive_yaw = 0;
    for (const ThrusterConfig &thruster : kThrusters)
    {
        if (thruster.orientation == Orientation::HorizontalDiagonal)
        {
            const int8_t yaw =
                axis_direction(thruster, ControlAxis::YawZ);
            assert(yaw == 1);
            horizontal_positive_yaw += yaw;
        }
    }

    // In the declared x-forward/y-right/z-down frame, positive yaw is
    // clockwise when viewed from above.
    assert(horizontal_positive_yaw == 4);
}

void set_zero_error_active_fixture(const BodyCommand &command = BodyCommand{})
{
    robot = RobotData{};
    robot.state = RobotState::ARMED_ACTIVE;
    robot.control_enable = true;
    robot.body_control_enabled = true;
    robot.float_enabled = false;
    robot.angle_enabled = false;
    robot.target_depth_cm = 30.0F;
    robot.depth_m = 0.3F;
    robot.target_yaw = 0.0F;
    robot.yaw = 0.0F;
    robot.yaw_v = 0.0F;
    robot.roll = 0.0F;
    robot.roll_v = 0.0F;
    robot.pitch = 0.0F;
    robot.pitch_v = 0.0F;
    robot.body_command = command;
    robot.body_command_source = BodyCommandSource::BinaryProtocol;
    robot.body_command_sequence = 1U;
    robot.body_command_last_ms = fake_tick;
    robot.body_command_timeout_ms = ROBOT_BODY_COMMAND_TIMEOUT_MS;
    robot.body_command_valid = true;
    for (float &limit : robot.motion_tuning.axis_max_output)
    {
        limit = 1.0F;
    }
    robot.motion_tuning.pwm_slew_rate_us_per_s =
        ROBOT_PWM_SLEW_RATE_MAX_US_PER_S;
}

BodyCommand command_for_axis(std::size_t axis_index, float value)
{
    BodyCommand command{};
    switch (axis_index)
    {
    case 0U: command.surge = value; break;
    case 1U: command.sway = value; break;
    case 2U: command.heave = value; break;
    case 3U: command.roll = value; break;
    case 4U: command.pitch = value; break;
    case 5U: command.yaw = value; break;
    default: assert(false); break;
    }
    return command;
}

void test_motor_control_six_pure_axes()
{
    constexpr float kCommand = 0.25F;
    for (std::size_t axis_index = 0U; axis_index < kAxes.size(); ++axis_index)
    {
        MotorControl controller;
        fake_tick = 1000U;
        controller.Init();
        set_zero_error_active_fixture(
            command_for_axis(axis_index, kCommand));
        fake_tick += 100U;
        controller.Update();

        assert(!robot.horizontal_saturated);
        assert(!robot.vertical_saturated);
        for (std::size_t channel = 0U; channel < kExpected.size(); ++channel)
        {
            const float expected =
                static_cast<float>(
                    kExpected[channel].axis_directions[axis_index]) *
                kCommand;
            assert(nearly_equal(robot.mixed_output[channel], expected));
        }
    }
}

void test_motor_control_group_desaturation()
{
    MotorControl controller;
    fake_tick = 1000U;
    controller.Init();
    set_zero_error_active_fixture(
        BodyCommand{1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F});
    fake_tick += 100U;
    controller.Update();

    assert(robot.horizontal_saturated);
    assert(robot.vertical_saturated);
    constexpr std::array<float, 8> kExpectedMixed{{
        -1.0F / 3.0F,
        1.0F,
        1.0F / 3.0F,
        1.0F / 3.0F,
        1.0F,
        1.0F / 3.0F,
        -1.0F / 3.0F,
        1.0F / 3.0F,
    }};
    for (std::size_t channel = 0U; channel < kExpectedMixed.size();
         ++channel)
    {
        assert(nearly_equal(robot.mixed_output[channel],
                            kExpectedMixed[channel]));
    }
}

void test_zero_command_calibration_and_legacy_amplitudes()
{
    {
        MotorControl controller;
        fake_tick = 1000U;
        controller.Init();
        set_zero_error_active_fixture();
        fake_tick += 100U;
        controller.Update();

        for (const int32_t pwm : robot.pwm)
        {
            assert(pwm == ROBOT_PWM_NEUTRAL_US);
        }
    }

    {
        MotorControl controller;
        fake_tick = 1000U;
        controller.Init();
        BodyCommand command{};
        command.surge = 80.0F / 450.0F;
        set_zero_error_active_fixture(command);
        fake_tick += 100U;
        controller.Update();

        assert(robot.pwm[0] == 1370);
        assert(robot.pwm[3] == 1370);
        assert(robot.pwm[4] == 1630);
        assert(robot.pwm[7] == 1630);
    }

    {
        MotorControl controller;
        fake_tick = 1000U;
        controller.Init();
        BodyCommand command{};
        command.yaw = 40.0F / 450.0F;
        set_zero_error_active_fixture(command);
        fake_tick += 100U;
        controller.Update();

        for (const uint8_t channel :
             std::array<uint8_t, 4>{{0U, 3U, 4U, 7U}})
        {
            assert(robot.pwm[channel] == 1590);
        }
    }
}

void expect_all_body_output_neutral()
{
    assert(!robot.body_command_valid);
    assert(robot.body_command_source == BodyCommandSource::None);
    assert(!robot.horizontal_saturated);
    assert(!robot.vertical_saturated);
    for (std::size_t channel = 0U; channel < kExpected.size(); ++channel)
    {
        assert(robot.mixed_output[channel] == 0.0F);
        assert(robot.pwm[channel] == ROBOT_PWM_NEUTRAL_US);
    }
}

void test_invalid_and_timed_out_commands_are_neutral()
{
    {
        MotorControl controller;
        fake_tick = 1000U;
        controller.Init();
        BodyCommand invalid{};
        invalid.surge = NAN;
        set_zero_error_active_fixture(invalid);
        controller.Update();
        expect_all_body_output_neutral();
    }

    MotorControl controller;
    fake_tick = 1000U;
    controller.Init();
    BodyCommand command{};
    command.surge = 0.25F;
    set_zero_error_active_fixture(command);
    fake_tick += ROBOT_BODY_COMMAND_TIMEOUT_MS;
    controller.Update();
    assert(robot.body_command_valid);
    assert(nearly_equal(robot.mixed_output[0], -0.25F));

    fake_tick++;
    controller.Update();
    expect_all_body_output_neutral();
}

void test_pwm_slew_and_hard_stop()
{
    MotorControl controller;
    fake_tick = 1000U;
    controller.Init();
    BodyCommand command{};
    command.surge = 1.0F;
    set_zero_error_active_fixture(command);
    robot.motion_tuning.pwm_slew_rate_us_per_s = 1000U;

    fake_tick += 20U;
    controller.Update();
    assert(robot.pwm[0] == 1480);
    assert(robot.pwm[3] == 1480);
    assert(robot.pwm[4] == 1520);
    assert(robot.pwm[7] == 1520);

    robot.body_command = BodyCommand{};
    robot.body_command_sequence++;
    robot.body_command_last_ms = fake_tick;
    fake_tick += 20U;
    controller.Update();
    for (const uint8_t channel :
         std::array<uint8_t, 4>{{0U, 3U, 4U, 7U}})
    {
        assert(robot.pwm[channel] == ROBOT_PWM_NEUTRAL_US);
    }

    robot.body_command = command;
    robot.body_command_sequence++;
    robot.body_command_last_ms = fake_tick;
    fake_tick += 20U;
    controller.Update();
    assert(robot.pwm[0] == 1480);
    robot.state = RobotState::DISARMED;
    controller.Update();
    expect_all_body_output_neutral();
}

} // namespace

extern "C" uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

int main()
{
    test_coordinate_system();
    test_complete_thruster_table();
    test_position_and_torque_geometry();
    test_six_degree_of_freedom_signs();
    test_clockwise_is_positive_yaw();
    test_motor_control_six_pure_axes();
    test_motor_control_group_desaturation();
    test_zero_command_calibration_and_legacy_amplitudes();
    test_invalid_and_timed_out_commands_are_neutral();
    test_pwm_slew_and_hard_stop();
    std::cout << "thruster config host tests: PASS\n";
    return 0;
}
