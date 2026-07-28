#include "binary_protocol.h"
#include "robot_data.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {
uint32_t fake_tick = 1000U;

struct Response {
    uint8_t type;
    uint16_t frame_sequence;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint16_t payload_length;
};

Response pop_response(BinaryProtocol *bp)
{
    uint8_t frame[PROTO_BUF_SIZE]{};
    uint16_t frame_length = 0U;
    assert(bp_get_tx_frame(bp, frame, sizeof(frame), &frame_length));
    assert(frame_length >= PROTO_FRAME_OVERHEAD + PROTO_CRC_SIZE);
    Response response{};
    response.type = frame[3];
    response.frame_sequence = static_cast<uint16_t>(
        frame[4] | (static_cast<uint16_t>(frame[5]) << 8U));
    response.payload_length = static_cast<uint16_t>(
        frame[6] | (static_cast<uint16_t>(frame[7]) << 8U));
    assert(response.payload_length <= sizeof(response.payload));
    std::memcpy(response.payload, frame + PROTO_FRAME_OVERHEAD,
                response.payload_length);
    return response;
}

ProtoNack as_nack(const Response &response)
{
    assert(response.type == ProtoMsg_NACK);
    assert(response.payload_length == sizeof(ProtoNack));
    ProtoNack nack{};
    std::memcpy(&nack, response.payload, sizeof(nack));
    return nack;
}

void expect_nack(BinaryProtocol *bp, uint16_t rejected_sequence,
                 uint8_t original_type, uint8_t reason)
{
    const ProtoNack nack = as_nack(pop_response(bp));
    assert(nack.rejected_sequence == rejected_sequence);
    assert(nack.original_type == original_type);
    assert(nack.reason == reason);
}

void expect_ack(BinaryProtocol *bp, uint16_t sequence)
{
    const Response response = pop_response(bp);
    assert(response.type == ProtoMsg_ACK);
    assert(response.payload_length == sizeof(ProtoAck));
    ProtoAck ack{};
    std::memcpy(&ack, response.payload, sizeof(ack));
    assert(ack.ack_seq == sequence);
}

void reset_fixture(BinaryProtocol *bp)
{
    bp_init(bp);
    robot = RobotData{};
    // Most protocol tests model a PCA9685 that has already completed its
    // verified neutral initialization.
    robot.actuator_output_ready = true;
    fake_tick = 1000U;
    robot.depth_sensor_ready = true;
    robot.depth_sample_valid = true;
    robot.depth_sample_ms = fake_tick;
    robot.depth_sample_generation = 1U;
    robot.depth_m = 0.42F;
}

void enter_armed_active(BinaryProtocol *bp)
{
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x10U, nullptr, 0U);
    expect_ack(bp, 0x10U);
    bp_dispatch_frame(bp, ProtoMsg_FLOAT_ON, 0x11U, nullptr, 0U);
    expect_ack(bp, 0x11U);
    assert(robot.state == RobotState::ARMED_ACTIVE);
    assert(robot.float_enabled);
}

void enter_body_control(BinaryProtocol *bp)
{
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x10U, nullptr, 0U);
    expect_ack(bp, 0x10U);
    bp_dispatch_frame(bp, ProtoMsg_BODY_CONTROL_ON, 0x11U, nullptr, 0U);
    expect_ack(bp, 0x11U);
    assert(robot.state == RobotState::ARMED_ACTIVE);
    assert(robot.control_enable);
    assert(robot.body_control_enabled);
    assert(!robot.float_enabled);
    assert(!robot.angle_enabled);
}

void assert_body_output_neutral()
{
    assert(!robot.body_command_valid);
    assert(robot.body_command_source == BodyCommandSource::None);
    assert(robot.body_command.surge == 0.0F);
    assert(robot.body_command.sway == 0.0F);
    assert(robot.body_command.heave == 0.0F);
    assert(robot.body_command.roll == 0.0F);
    assert(robot.body_command.pitch == 0.0F);
    assert(robot.body_command.yaw == 0.0F);
    assert(!robot.horizontal_saturated);
    assert(!robot.vertical_saturated);
    for (uint8_t channel = 0U; channel < 8U; ++channel)
    {
        assert(robot.mixed_output[channel] == 0.0F);
        assert(robot.pwm[channel] == ROBOT_PWM_NEUTRAL_US);
    }
}

void test_length_and_unsupported_rejection(BinaryProtocol *bp)
{
    reset_fixture(bp);
    const uint8_t junk = 0x55U;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x101U, &junk, 1U);
    expect_nack(bp, 0x101U, ProtoMsg_ARM,
                ProtoNack_InvalidPayloadLength);
    assert(robot.state == RobotState::DISARMED);
    assert(robot.protocol_errors == 1U);

    bp_dispatch_frame(bp, 0x77U, 0x102U, nullptr, 0U);
    expect_nack(bp, 0x102U, 0x77U, ProtoNack_UnsupportedMessage);
    assert(robot.protocol_errors == 2U);

    bp_dispatch_frame(bp, ProtoMsg_HEARTBEAT, 0x103U, nullptr, 0U);
    expect_nack(bp, 0x103U, ProtoMsg_HEARTBEAT,
                ProtoNack_InvalidPayloadLength);
}

void test_pwm_validation_and_sequence(BinaryProtocol *bp)
{
    reset_fixture(bp);
    bp_dispatch_frame(bp, ProtoMsg_ARM, 1U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    bp_dispatch_frame(bp, ProtoMsg_ENTER_MANUAL, 2U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.state == RobotState::MANUAL_TEST);

    ProtoSetPwm command{2U, 0U, 1530, 200U};
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 3U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command) - 1U);
    expect_nack(bp, 3U, ProtoMsg_SET_PWM,
                ProtoNack_InvalidPayloadLength);
    assert(!robot.manual_pwm_enabled);

    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 4U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.manual_pwm_enabled);
    assert(robot.active_test_channel == 2U);

    command.channel = 3U;
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 5U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    expect_nack(bp, 5U, ProtoMsg_SET_PWM, ProtoNack_ChannelBusy);
    assert(robot.active_test_channel == 2U);

    command.channel = 2U;
    command.reserved = 1U;
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 6U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    expect_nack(bp, 6U, ProtoMsg_SET_PWM, ProtoNack_InvalidValue);

    command.reserved = 0U;
    command.timeout_ms = 199U;
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 7U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    expect_nack(bp, 7U, ProtoMsg_SET_PWM, ProtoNack_InvalidValue);
}

void test_float_value_and_state_validation(BinaryProtocol *bp)
{
    reset_fixture(bp);
    ProtoSetDepth depth{NAN};
    bp_dispatch_frame(bp, ProtoMsg_SET_DEPTH, 0x201U,
                      reinterpret_cast<const uint8_t *>(&depth), sizeof(depth));
    expect_nack(bp, 0x201U, ProtoMsg_SET_DEPTH, ProtoNack_InvalidValue);

    ProtoSetMotion motion{2U};
    bp_dispatch_frame(bp, ProtoMsg_SET_MOTION, 0x202U,
                      reinterpret_cast<const uint8_t *>(&motion), sizeof(motion));
    expect_nack(bp, 0x202U, ProtoMsg_SET_MOTION, ProtoNack_BadState);
    assert_body_output_neutral();
}

void test_depth_hold_health_tuning_and_reports(BinaryProtocol *bp)
{
    reset_fixture(bp);
    robot.depth_sample_valid = false;
    robot.depth_sample_generation = 0U;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x210U, nullptr, 0U);
    expect_ack(bp, 0x210U);
    bp_dispatch_frame(bp, ProtoMsg_FLOAT_ON, 0x211U, nullptr, 0U);
    expect_nack(
        bp, 0x211U, ProtoMsg_FLOAT_ON, ProtoNack_InternalError);
    assert(robot.state == RobotState::ARMED_IDLE);
    assert(!robot.float_enabled);
    assert(robot.last_neutral_reason == ProtoNeutral_DEPTH_SENSOR);
    assert_body_output_neutral();

    reset_fixture(bp);
    robot.depth_sample_ms = fake_tick - ROBOT_DEPTH_SAMPLE_MAX_AGE_MS - 1U;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x212U, nullptr, 0U);
    expect_ack(bp, 0x212U);
    bp_dispatch_frame(bp, ProtoMsg_FLOAT_ON, 0x213U, nullptr, 0U);
    expect_nack(
        bp, 0x213U, ProtoMsg_FLOAT_ON, ProtoNack_InternalError);
    assert(!robot.float_enabled);

    reset_fixture(bp);
    robot.depth_m = -0.05F;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x20EU, nullptr, 0U);
    expect_ack(bp, 0x20EU);
    bp_dispatch_frame(bp, ProtoMsg_FLOAT_ON, 0x20FU, nullptr, 0U);
    expect_nack(
        bp, 0x20FU, ProtoMsg_FLOAT_ON, ProtoNack_InternalError);
    assert(robot.state == RobotState::ARMED_IDLE);
    assert(!robot.float_enabled);
    assert(robot.last_neutral_reason == ProtoNeutral_DEPTH_SENSOR);
    assert_body_output_neutral();

    reset_fixture(bp);
    bp_dispatch_frame(bp, ProtoMsg_REQUEST_DEPTH_PID_TUNING,
                      0x214U, nullptr, 0U);
    Response response = pop_response(bp);
    assert(response.type == ProtoMsg_DEPTH_PID_TUNING_REPORT);
    assert(response.payload_length == sizeof(ProtoDepthPidTuning));
    ProtoDepthPidTuning tuning{};
    std::memcpy(&tuning, response.payload, sizeof(tuning));
    assert(tuning.kp == 10.0F);
    assert(tuning.ki == 0.02F);
    assert(tuning.kd == 10.0F);
    assert(tuning.output_limit_us == 200.0F);

    tuning = ProtoDepthPidTuning{
        8.0F, 0.0F, 4.0F, 90.0F, 25.0F, 30.0F, 150.0F,
    };
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH_PID_TUNING, 0x215U,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_ack(bp, 0x215U);
    assert(robot.depth_pid_tuning.kp == 8.0F);
    assert(robot.depth_pid_tuning.ki == 0.0F);
    assert(robot.depth_pid_tuning.output_limit_us == 150.0F);

    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x216U, nullptr, 0U);
    expect_ack(bp, 0x216U);
    bp_dispatch_frame(bp, ProtoMsg_FLOAT_ON, 0x217U, nullptr, 0U);
    expect_ack(bp, 0x217U);
    assert(robot.target_depth_cm == 42.0F);
    assert(robot.depth_active_setpoint_cm == 42.0F);
    assert(robot.float_enabled);
    assert(robot.depth_hold_session_generation == 1U);
    assert(robot.depth_command_last_ms == fake_tick);

    const uint32_t generation_before_target =
        robot.depth_control_generation;
    ProtoSetDepth depth{125.0F};
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH, 0x218U,
        reinterpret_cast<const uint8_t *>(&depth), sizeof(depth));
    expect_ack(bp, 0x218U);
    assert(robot.target_depth_cm == 125.0F);
    assert(robot.depth_control_generation >
           generation_before_target);
    const uint32_t generation_after_target =
        robot.depth_control_generation;

    fake_tick += 200U;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH, 0x219U,
        reinterpret_cast<const uint8_t *>(&depth), sizeof(depth));
    expect_ack(bp, 0x219U);
    assert(robot.last_cmd_tick == fake_tick);
    assert(robot.depth_command_last_ms == fake_tick);
    assert(robot.depth_control_generation ==
           generation_after_target);

    const uint32_t depth_lease_before_yaw =
        robot.depth_command_last_ms;
    fake_tick += 50U;
    const ProtoSetYaw yaw{15.0F};
    bp_dispatch_frame(
        bp, ProtoMsg_SET_YAW, 0x2191U,
        reinterpret_cast<const uint8_t *>(&yaw), sizeof(yaw));
    expect_ack(bp, 0x2191U);
    assert(robot.last_cmd_tick == fake_tick);
    assert(robot.depth_command_last_ms == depth_lease_before_yaw);
    bp_dispatch_frame(bp, ProtoMsg_ANGLE_OFF, 0x2192U, nullptr, 0U);
    expect_ack(bp, 0x2192U);
    assert(robot.float_enabled);

    bp_dispatch_frame(
        bp, ProtoMsg_REQUEST_DEPTH_CONTROL, 0x21AU, nullptr, 0U);
    response = pop_response(bp);
    assert(response.type == ProtoMsg_DEPTH_CONTROL_REPORT);
    assert(response.payload_length == sizeof(ProtoDepthControlReport));
    ProtoDepthControlReport control{};
    std::memcpy(&control, response.payload, sizeof(control));
    assert(control.requested_target_cm == 125.0F);
    assert(control.active_setpoint_cm == 125.0F);
    assert((control.flags & 0x01U) != 0U);
    assert((control.flags & 0x02U) != 0U);
    assert((control.flags & 0x04U) != 0U);
    assert((control.flags & 0x20U) != 0U);
    assert(control.sample_age_ms ==
           fake_tick - robot.depth_sample_ms);

    tuning.kp = 9.0F;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH_PID_TUNING, 0x21BU,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_nack(
        bp, 0x21BU, ProtoMsg_SET_DEPTH_PID_TUNING,
        ProtoNack_BadState);
    assert(robot.depth_pid_tuning.kp == 8.0F);

    bp_dispatch_frame(bp, ProtoMsg_FLOAT_OFF, 0x21CU, nullptr, 0U);
    expect_ack(bp, 0x21CU);
    assert(robot.state == RobotState::ARMED_IDLE);
    assert(!robot.float_enabled);
    assert(robot.depth_pid_output_us == 0.0F);
    assert_body_output_neutral();

    tuning.kp = NAN;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH_PID_TUNING, 0x21DU,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_nack(
        bp, 0x21DU, ProtoMsg_SET_DEPTH_PID_TUNING,
        ProtoNack_InvalidValue);

    reset_fixture(bp);
    enter_body_control(bp);
    depth.target_depth_cm = 100.0F;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_DEPTH, 0x21EU,
        reinterpret_cast<const uint8_t *>(&depth), sizeof(depth));
    expect_nack(
        bp, 0x21EU, ProtoMsg_SET_DEPTH, ProtoNack_BadState);
}

void test_body_command_acceptance_and_fail_closed_rejection(BinaryProtocol *bp)
{
    reset_fixture(bp);
    enter_body_control(bp);

    const ProtoSetBodyCommand valid{
        0.25F, -0.5F, 0.75F, -1.0F, 1.0F, 0.125F,
    };
    fake_tick = 2345U;
    bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, 0x501U,
                      reinterpret_cast<const uint8_t *>(&valid),
                      sizeof(valid));
    expect_ack(bp, 0x501U);
    assert(robot.body_command_valid);
    assert(robot.body_command_source == BodyCommandSource::BinaryProtocol);
    assert(robot.body_command_sequence == 0x501U);
    assert(robot.body_command_last_ms == fake_tick);
    assert(robot.body_command_timeout_ms == ROBOT_BODY_COMMAND_TIMEOUT_MS);
    assert(robot.last_cmd_tick == fake_tick);
    assert(robot.body_command.surge == valid.surge);
    assert(robot.body_command.sway == valid.sway);
    assert(robot.body_command.heave == valid.heave);
    assert(robot.body_command.roll == valid.roll);
    assert(robot.body_command.pitch == valid.pitch);
    assert(robot.body_command.yaw == valid.yaw);

    ProtoSetBodyCommand invalid_cases[] = {
        valid,
        valid,
        valid,
        valid,
        valid,
    };
    invalid_cases[0].surge = NAN;
    invalid_cases[1].sway = INFINITY;
    invalid_cases[2].heave = -INFINITY;
    invalid_cases[3].roll = 1.001F;
    invalid_cases[4].pitch = -1.001F;

    uint16_t sequence = 0x502U;
    for (const ProtoSetBodyCommand &invalid : invalid_cases)
    {
        bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, sequence,
                          reinterpret_cast<const uint8_t *>(&invalid),
                          sizeof(invalid));
        expect_nack(bp, sequence, ProtoMsg_SET_BODY_COMMAND,
                    ProtoNack_InvalidValue);
        assert_body_output_neutral();
        assert(robot.state == RobotState::ARMED_IDLE);
        assert(!robot.body_control_enabled);

        ++sequence;
        bp_dispatch_frame(bp, ProtoMsg_BODY_CONTROL_ON, sequence, nullptr, 0U);
        expect_ack(bp, sequence);
        ++sequence;
        ++fake_tick;
        bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, sequence,
                          reinterpret_cast<const uint8_t *>(&valid),
                          sizeof(valid));
        expect_ack(bp, sequence);
        ++sequence;
    }

    bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, sequence,
                      reinterpret_cast<const uint8_t *>(&valid),
                      sizeof(valid) - 1U);
    expect_nack(bp, sequence, ProtoMsg_SET_BODY_COMMAND,
                ProtoNack_InvalidPayloadLength);
    assert_body_output_neutral();
    assert(robot.state == RobotState::ARMED_IDLE);

    ++sequence;
    bp_dispatch_frame(bp, ProtoMsg_BODY_CONTROL_ON, sequence, nullptr, 0U);
    expect_ack(bp, sequence);
    ++sequence;
    bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, sequence,
                      reinterpret_cast<const uint8_t *>(&valid),
                      sizeof(valid));
    expect_ack(bp, sequence);
    robot.pwm[0] = 1600;
    robot.mixed_output[0] = 0.5F;
    robot.horizontal_saturated = true;
    robot.state = RobotState::ARMED_IDLE;

    ++sequence;
    bp_dispatch_frame(bp, ProtoMsg_SET_BODY_COMMAND, sequence,
                      reinterpret_cast<const uint8_t *>(&valid),
                      sizeof(valid));
    expect_nack(bp, sequence, ProtoMsg_SET_BODY_COMMAND,
                ProtoNack_BadState);
    assert_body_output_neutral();
}

void test_body_control_mode_and_tuning(BinaryProtocol *bp)
{
    reset_fixture(bp);

    bp_dispatch_frame(
        bp, ProtoMsg_REQUEST_MOTION_TUNING, 0x701U, nullptr, 0U);
    Response response = pop_response(bp);
    assert(response.type == ProtoMsg_MOTION_TUNING_REPORT);
    assert(response.payload_length == sizeof(ProtoMotionTuning));
    ProtoMotionTuning report{};
    std::memcpy(&report, response.payload, sizeof(report));
    assert(report.axis_gain[0] == 1.0F);
    assert(report.axis_max_output[0] == 0.20F);
    assert(report.axis_max_output[3] == 0.10F);
    assert(report.global_multiplier == 1.0F);
    assert(report.pwm_slew_rate_us_per_s == 1000U);
    assert(report.command_timeout_ms == 500U);

    ProtoMotionTuning tuning{};
    for (uint8_t axis = 0U; axis < ROBOT_BODY_AXIS_COUNT; ++axis)
    {
        tuning.axis_gain[axis] = 0.5F + 0.1F * axis;
        tuning.axis_max_output[axis] = 0.15F + 0.05F * axis;
    }
    tuning.global_multiplier = 0.75F;
    tuning.pwm_slew_rate_us_per_s = 1500U;
    tuning.command_timeout_ms = 800U;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_MOTION_TUNING, 0x702U,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_ack(bp, 0x702U);
    assert(robot.motion_tuning.axis_gain[5] == tuning.axis_gain[5]);
    assert(robot.motion_tuning.axis_max_output[5] ==
           tuning.axis_max_output[5]);
    assert(robot.motion_tuning.global_multiplier == 0.75F);
    assert(robot.motion_tuning.pwm_slew_rate_us_per_s == 1500U);
    assert(robot.motion_tuning.command_timeout_ms == 800U);
    assert(robot.motion_tuning_generation == 1U);

    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x703U, nullptr, 0U);
    expect_ack(bp, 0x703U);
    bp_dispatch_frame(bp, ProtoMsg_BODY_CONTROL_ON, 0x704U, nullptr, 0U);
    expect_ack(bp, 0x704U);
    assert(robot.body_control_enabled);

    tuning.global_multiplier = 0.5F;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_MOTION_TUNING, 0x705U,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_nack(
        bp, 0x705U, ProtoMsg_SET_MOTION_TUNING, ProtoNack_BadState);
    assert(robot.motion_tuning.global_multiplier == 0.75F);

    bp_dispatch_frame(bp, ProtoMsg_BODY_CONTROL_OFF, 0x706U, nullptr, 0U);
    expect_ack(bp, 0x706U);
    assert(robot.state == RobotState::ARMED_IDLE);
    assert(!robot.control_enable);
    assert(!robot.body_control_enabled);
    assert_body_output_neutral();

    tuning.axis_gain[0] = NAN;
    bp_dispatch_frame(
        bp, ProtoMsg_SET_MOTION_TUNING, 0x707U,
        reinterpret_cast<const uint8_t *>(&tuning), sizeof(tuning));
    expect_nack(
        bp, 0x707U, ProtoMsg_SET_MOTION_TUNING,
        ProtoNack_InvalidValue);
}

void test_legacy_motion_maps_to_body_command(BinaryProtocol *bp)
{
    reset_fixture(bp);
    enter_armed_active(bp);

    struct LegacyCase {
        uint8_t state;
        float surge;
        float sway;
        float yaw;
    };
    constexpr float surge_scale = 80.0F / 450.0F;
    constexpr float yaw_scale = 40.0F / 450.0F;
    const LegacyCase cases[] = {
        {0U, 0.0F, 0.0F, 0.0F},
        {1U, 0.0F, 0.0F, 0.0F},
        {2U, surge_scale, 0.0F, 0.0F},
        {3U, -surge_scale, 0.0F, 0.0F},
        {4U, 0.0F, -surge_scale, 0.0F},
        {5U, 0.0F, surge_scale, 0.0F},
        {6U, 0.0F, 0.0F, yaw_scale},
        {7U, 0.0F, 0.0F, -yaw_scale},
    };

    uint16_t sequence = 0x601U;
    for (const LegacyCase &entry : cases)
    {
        const ProtoSetMotion motion{entry.state};
        bp_dispatch_frame(bp, ProtoMsg_SET_MOTION, sequence,
                          reinterpret_cast<const uint8_t *>(&motion),
                          sizeof(motion));
        expect_ack(bp, sequence);
        assert(robot.body_command_valid);
        assert(robot.body_command_source ==
               BodyCommandSource::LegacySetMotion);
        assert(robot.body_command_sequence == sequence);
        assert(robot.body_command.surge == entry.surge);
        assert(robot.body_command.sway == entry.sway);
        assert(robot.body_command.heave == 0.0F);
        assert(robot.body_command.roll == 0.0F);
        assert(robot.body_command.pitch == 0.0F);
        assert(robot.body_command.yaw == entry.yaw);
        ++sequence;
    }

    const ProtoSetMotion invalid{8U};
    bp_dispatch_frame(bp, ProtoMsg_SET_MOTION, sequence,
                      reinterpret_cast<const uint8_t *>(&invalid),
                      sizeof(invalid));
    expect_nack(bp, sequence, ProtoMsg_SET_MOTION, ProtoNack_InvalidValue);
    assert_body_output_neutral();
}

void test_estop_remains_latched_across_disarm(BinaryProtocol *bp)
{
    reset_fixture(bp);
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x301U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    bp_dispatch_frame(bp, ProtoMsg_ENTER_MANUAL, 0x302U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);

    ProtoSetPwm command{0U, 0U, 1530, 500U};
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 0x303U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.manual_pwm[0] == 1530);

    bp_dispatch_frame(bp, ProtoMsg_EMERGENCY_STOP, 0x304U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.state == RobotState::EMERGENCY_STOP);
    assert(robot.estop_locked);
    assert(!robot.manual_pwm_enabled);
    for (uint8_t channel = 0U; channel < 8U; ++channel)
    {
        assert(robot.manual_pwm[channel] == ROBOT_PWM_NEUTRAL_US);
        assert(robot.pwm[channel] == ROBOT_PWM_NEUTRAL_US);
    }

    bp_dispatch_frame(bp, ProtoMsg_DISARM, 0x305U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.state == RobotState::EMERGENCY_STOP);
    assert(robot.estop_locked);

    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x306U, nullptr, 0U);
    expect_nack(bp, 0x306U, ProtoMsg_ARM, ProtoNack_EstopLocked);

    bp_dispatch_frame(bp, ProtoMsg_RESET_ESTOP, 0x307U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.state == RobotState::DISARMED);
    assert(!robot.estop_locked);
}

void test_actuator_readiness_blocks_arm_and_estop_reset(BinaryProtocol *bp)
{
    reset_fixture(bp);
    robot.actuator_output_ready = false;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x380U, nullptr, 0U);
    expect_nack(bp, 0x380U, ProtoMsg_ARM, ProtoNack_InternalError);
    assert(robot.state == RobotState::DISARMED);

    robot.actuator_output_ready = true;
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x381U, nullptr, 0U);
    expect_ack(bp, 0x381U);
    bp_dispatch_frame(bp, ProtoMsg_EMERGENCY_STOP, 0x382U, nullptr, 0U);
    expect_ack(bp, 0x382U);

    robot.actuator_output_ready = false;
    bp_dispatch_frame(bp, ProtoMsg_RESET_ESTOP, 0x383U, nullptr, 0U);
    expect_nack(
        bp, 0x383U, ProtoMsg_RESET_ESTOP, ProtoNack_InternalError);
    assert(robot.estop_locked);
    assert(robot.state == RobotState::EMERGENCY_STOP);

    robot.actuator_output_ready = true;
    bp_dispatch_frame(bp, ProtoMsg_RESET_ESTOP, 0x384U, nullptr, 0U);
    expect_ack(bp, 0x384U);
    assert(!robot.estop_locked);
    assert(robot.state == RobotState::DISARMED);

    robot.actuator_output_ready = false;
    robot.state = RobotState::FAULT;
    bp_dispatch_frame(bp, ProtoMsg_DISARM, 0x385U, nullptr, 0U);
    expect_ack(bp, 0x385U);
    assert(robot.state == RobotState::FAULT);
    assert(robot.last_neutral_reason == ProtoNeutral_FAULT);
}

void test_set_all_neutral_exits_manual_mode(BinaryProtocol *bp)
{
    reset_fixture(bp);
    bp_dispatch_frame(bp, ProtoMsg_ARM, 0x401U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    bp_dispatch_frame(bp, ProtoMsg_ENTER_MANUAL, 0x402U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);

    ProtoSetPwm command{1U, 0U, 1520, 500U};
    bp_dispatch_frame(bp, ProtoMsg_SET_PWM, 0x403U,
                      reinterpret_cast<const uint8_t *>(&command),
                      sizeof(command));
    assert(pop_response(bp).type == ProtoMsg_ACK);

    bp_dispatch_frame(bp, ProtoMsg_SET_ALL_NEUTRAL, 0x404U, nullptr, 0U);
    assert(pop_response(bp).type == ProtoMsg_ACK);
    assert(robot.state == RobotState::ARMED_IDLE);
    assert(!robot.manual_pwm_enabled);
    assert(robot.active_test_channel == 0xFFU);
    for (uint8_t channel = 0U; channel < 8U; ++channel)
    {
        assert(robot.manual_pwm[channel] == ROBOT_PWM_NEUTRAL_US);
        assert(robot.pwm[channel] == ROBOT_PWM_NEUTRAL_US);
    }
}

} // namespace

extern "C" uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

int main()
{
    BinaryProtocol *bp = bp_get_instance();
    test_length_and_unsupported_rejection(bp);
    test_pwm_validation_and_sequence(bp);
    test_float_value_and_state_validation(bp);
    test_depth_hold_health_tuning_and_reports(bp);
    test_body_command_acceptance_and_fail_closed_rejection(bp);
    test_body_control_mode_and_tuning(bp);
    test_legacy_motion_maps_to_body_command(bp);
    test_estop_remains_latched_across_disarm(bp);
    test_actuator_readiness_blocks_arm_and_estop_reset(bp);
    test_set_all_neutral_exits_manual_mode(bp);
    std::cout << "stm32 protocol handler host tests: PASS\n";
    return 0;
}
