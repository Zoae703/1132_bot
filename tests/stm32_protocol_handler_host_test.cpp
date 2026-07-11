#include "binary_protocol.h"
#include "robot_data.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

RobotData robot;
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

void reset_fixture(BinaryProtocol *bp)
{
    bp_init(bp);
    robot = RobotData{};
    fake_tick = 1000U;
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
    assert(robot.motion_state == 0U);
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
    test_estop_remains_latched_across_disarm(bp);
    test_set_all_neutral_exits_manual_mode(bp);
    std::cout << "stm32 protocol handler host tests: PASS\n";
    return 0;
}
