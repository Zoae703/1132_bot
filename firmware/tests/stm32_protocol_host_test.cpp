#include "binary_protocol.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

struct ReceivedFrame {
    uint8_t type;
    uint16_t sequence;
    std::vector<uint8_t> payload;
};

std::vector<ReceivedFrame> received;

std::vector<uint8_t> make_frame(uint8_t type, uint16_t sequence,
                                const uint8_t *payload = nullptr,
                                uint16_t payload_len = 0U)
{
    std::vector<uint8_t> frame(PROTO_BUF_SIZE);
    const uint16_t length = proto_encode_frame(
        frame.data(), static_cast<uint16_t>(frame.size()), type, sequence,
        payload, payload_len);
    assert(length > 0U);
    frame.resize(length);
    return frame;
}

void test_stream_parser()
{
    BinaryProtocol bp{};
    bp_init(&bp);

    const ProtoSetPwm pwm{2U, 0U, 1530, 200U};
    const auto first = make_frame(
        ProtoMsg_SET_PWM, 17U,
        reinterpret_cast<const uint8_t *>(&pwm), sizeof(pwm));
    const auto second = make_frame(ProtoMsg_ARM, 18U);

    received.clear();
    bp_feed_bytes(&bp, first.data(), 1U);
    bp_feed_bytes(&bp, first.data() + 1U, 3U);
    assert(received.empty());
    bp_feed_bytes(&bp, first.data() + 4U,
                  static_cast<uint16_t>(first.size() - 4U));
    assert(received.size() == 1U);
    assert(received[0].sequence == 17U);

    std::vector<uint8_t> combined{0x00U, 0x7EU, 0xAAU};
    combined.insert(combined.end(), second.begin(), second.end());
    combined.insert(combined.end(), first.begin(), first.end());
    bp_feed_bytes(&bp, combined.data(), static_cast<uint16_t>(combined.size()));
    assert(received.size() == 3U);
    assert(received[1].type == ProtoMsg_ARM);
    assert(received[2].type == ProtoMsg_SET_PWM);

    auto bad = first;
    bad.back() ^= 0x80U;
    bad.insert(bad.end(), second.begin(), second.end());
    bp_feed_bytes(&bp, bad.data(), static_cast<uint16_t>(bad.size()));
    assert(bp.crc_errors == 1U);
    assert(received.back().type == ProtoMsg_ARM);

    auto inserted = first;
    inserted.insert(inserted.begin() + 10, 0x37U);
    inserted.insert(inserted.end(), second.begin(), second.end());
    const std::size_t before_inserted = received.size();
    bp_feed_bytes(&bp, inserted.data(), static_cast<uint16_t>(inserted.size()));
    assert(received.size() == before_inserted + 1U);
    assert(received.back().type == ProtoMsg_ARM);

    auto bad_version = second;
    bad_version[2] = 0x7FU;
    bad_version.insert(bad_version.end(), second.begin(), second.end());
    const std::size_t before_version = received.size();
    bp_feed_bytes(&bp, bad_version.data(), static_cast<uint16_t>(bad_version.size()));
    assert(received.size() == before_version + 1U);
    assert(received.back().type == ProtoMsg_ARM);

    auto oversized = second;
    oversized[6] = 0xF1U;
    oversized[7] = 0x00U;
    oversized.insert(oversized.end(), second.begin(), second.end());
    const std::size_t before_oversized = received.size();
    bp_feed_bytes(&bp, oversized.data(), static_cast<uint16_t>(oversized.size()));
    assert(received.size() == before_oversized + 1U);
    assert(received.back().type == ProtoMsg_ARM);
}

uint8_t decoded_type(const uint8_t *frame, uint16_t length)
{
    assert(length >= PROTO_FRAME_OVERHEAD + PROTO_CRC_SIZE);
    return frame[3];
}

void test_priority_and_full_reporting()
{
    BinaryProtocol bp{};
    bp_init(&bp);

    for (uint32_t i = 0U; i < BP_TX_NORMAL_QUEUE_DEPTH; ++i)
    {
        assert(bp_send_frame(&bp, ProtoMsg_STATUS_REPORT, nullptr, 0U));
    }
    assert(!bp_send_frame(&bp, ProtoMsg_STATUS_REPORT, nullptr, 0U));
    assert(bp.tx_queue_full_count == 1U);
    assert(bp.tx_dropped_frames == 1U);
    assert(bp.tx_seq == BP_TX_NORMAL_QUEUE_DEPTH);

    assert(bp_send_ack(&bp, 0x1234U));
    assert(bp_send_nack(&bp, 0x4321U, ProtoMsg_SET_PWM,
                        ProtoNack_InvalidPayloadLength));

    uint8_t frame[PROTO_BUF_SIZE]{};
    uint16_t length = 0U;
    assert(bp_get_tx_frame(&bp, frame, sizeof(frame), &length));
    assert(decoded_type(frame, length) == ProtoMsg_ACK);
    assert(bp_get_tx_frame(&bp, frame, sizeof(frame), &length));
    assert(decoded_type(frame, length) == ProtoMsg_NACK);

    const ProtoNack *nack = reinterpret_cast<const ProtoNack *>(
        frame + PROTO_FRAME_OVERHEAD);
    assert(nack->rejected_sequence == 0x4321U);
    assert(nack->original_type == ProtoMsg_SET_PWM);
    assert(nack->reason == ProtoNack_InvalidPayloadLength);

    bp_note_tx_result(&bp, false);
    assert(bp.tx_send_failures == 1U);
}

void test_command_lengths_and_abi()
{
    uint16_t length = 0U;
    assert(PROTO_VERSION == 0x02U);
    assert(ProtoMsg_SET_BODY_COMMAND == 0x37U);
    assert(ProtoMsg_BODY_CONTROL_ON == 0x38U);
    assert(ProtoMsg_BODY_CONTROL_OFF == 0x39U);
    assert(ProtoMsg_SET_MOTION_TUNING == 0x3AU);
    assert(ProtoMsg_REQUEST_MOTION_TUNING == 0x42U);
    assert(ProtoMsg_MOTION_TUNING_REPORT == 0x82U);
    assert(sizeof(ProtoNack) == 4U);
    assert(sizeof(ProtoStatusReport) == 24U);
    assert(sizeof(ProtoSetBodyCommand) == 24U);
    assert(sizeof(ProtoMotionTuning) == 56U);
    assert(bp_expected_command_payload_length(ProtoMsg_SET_PWM, &length));
    assert(length == sizeof(ProtoSetPwm));
    assert(bp_expected_command_payload_length(
        ProtoMsg_SET_BODY_COMMAND, &length));
    assert(length == sizeof(ProtoSetBodyCommand));
    assert(bp_expected_command_payload_length(
        ProtoMsg_SET_MOTION_TUNING, &length));
    assert(length == sizeof(ProtoMotionTuning));
    assert(bp_expected_command_payload_length(
        ProtoMsg_BODY_CONTROL_ON, &length));
    assert(length == 0U);
    assert(bp_expected_command_payload_length(
        ProtoMsg_REQUEST_MOTION_TUNING, &length));
    assert(length == 0U);
    assert(bp_expected_command_payload_length(ProtoMsg_ARM, &length));
    assert(length == 0U);
    assert(!bp_expected_command_payload_length(ProtoMsg_STATUS_REPORT, &length));

    uint8_t output[PROTO_BUF_SIZE]{};
    assert(proto_encode_frame(output, sizeof(output), ProtoMsg_SET_PWM, 1U,
                              nullptr, sizeof(ProtoSetPwm)) == 0U);
}

} // namespace

extern "C" void bp_dispatch_frame(BinaryProtocol *, uint8_t type,
                                  uint16_t sequence, const uint8_t *payload,
                                  uint16_t payload_len)
{
    ReceivedFrame frame{type, sequence, {}};
    if (payload_len > 0U)
    {
        frame.payload.assign(payload, payload + payload_len);
    }
    received.push_back(frame);
}

int main()
{
    test_stream_parser();
    test_priority_and_full_reporting();
    test_command_lengths_and_abi();
    std::cout << "stm32 protocol host tests: PASS\n";
    return 0;
}
