/**
 * @file    binary_protocol.cpp
 * @brief   Non-blocking binary protocol parser — implementation.
 */

#include "binary_protocol.h"
#include <cstring>

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

void bp_init(BinaryProtocol *bp)
{
    if (bp == nullptr) return;
    std::memset(bp, 0, sizeof(*bp));
    bp->rx_state = BP_SYNC1;
}

/* ------------------------------------------------------------------ */
/*  TX queue helpers                                                    */
/* ------------------------------------------------------------------ */

static bool tx_queue_is_full(const BpTxQueue *queue)
{
    return queue->count >= BP_TX_HIGH_QUEUE_DEPTH;
}

static bool tx_queue_is_empty(const BpTxQueue *queue)
{
    return queue->count == 0U;
}

static bool tx_queue_push(BpTxQueue *queue, const uint8_t *data, uint16_t len)
{
    if (queue == nullptr || data == nullptr ||
        tx_queue_is_full(queue) || len > PROTO_BUF_SIZE)
    {
        return false;
    }

    std::memcpy(queue->frames[queue->head], data, len);
    queue->lengths[queue->head] = len;
    queue->head = static_cast<uint8_t>(
        (queue->head + 1U) % BP_TX_HIGH_QUEUE_DEPTH);
    queue->count++;
    return true;
}

static bool tx_queue_pop(BpTxQueue *queue, uint8_t *buf,
                         uint16_t buf_size, uint16_t *out_len)
{
    if (queue == nullptr || buf == nullptr || out_len == nullptr ||
        tx_queue_is_empty(queue))
    {
        return false;
    }

    const uint16_t len = queue->lengths[queue->tail];
    if (len > buf_size)
    {
        return false;
    }

    std::memcpy(buf, queue->frames[queue->tail], len);
    *out_len = len;
    queue->tail = static_cast<uint8_t>(
        (queue->tail + 1U) % BP_TX_HIGH_QUEUE_DEPTH);
    queue->count--;
    return true;
}

/* ------------------------------------------------------------------ */
/*  RX state machine — feed one byte                                   */
/* ------------------------------------------------------------------ */

void bp_feed_byte(BinaryProtocol *bp, uint8_t byte)
{
    if (bp == nullptr) return;

    switch (bp->rx_state)
    {
    case BP_SYNC1:
        if (byte == PROTO_MAGIC_0)
        {
            bp->rx_buf[0] = byte;
            bp->rx_pos = 1;
            bp->rx_state = BP_SYNC2;
        }
        /* else: stay in SYNC1, byte is discarded */
        break;

    case BP_SYNC2:
        if (byte == PROTO_MAGIC_1)
        {
            bp->rx_buf[1] = byte;
            bp->rx_state = BP_VERSION;
        }
        else
        {
            /* False sync — check if this byte starts a new sync */
            bp->rx_state = BP_SYNC1;
            bp->sync_losses++;
            if (byte == PROTO_MAGIC_0)
            {
                bp->rx_buf[0] = byte;
                bp->rx_pos = 1;
                bp->rx_state = BP_SYNC2;
            }
        }
        break;

    case BP_VERSION:
        bp->rx_buf[2] = byte;
        if (byte == PROTO_VERSION)
        {
            bp->rx_state = BP_TYPE;
        }
        else
        {
            bp->rx_state = BP_SYNC1;
            bp->sync_losses++;
            if (byte == PROTO_MAGIC_0)
            {
                bp->rx_buf[0] = byte;
                bp->rx_pos = 1;
                bp->rx_state = BP_SYNC2;
            }
        }
        break;

    case BP_TYPE:
        bp->rx_buf[3] = byte;
        bp->rx_msg_type = byte;
        bp->rx_state = BP_SEQ_L;
        break;

    case BP_SEQ_L:
        bp->rx_buf[4] = byte;
        bp->rx_seq = byte;
        bp->rx_state = BP_SEQ_H;
        break;

    case BP_SEQ_H:
        bp->rx_buf[5] = byte;
        bp->rx_seq |= (uint16_t)(byte << 8);
        bp->rx_state = BP_LEN_L;
        break;

    case BP_LEN_L:
        bp->rx_buf[6] = byte;
        bp->rx_payload_len = byte;
        bp->rx_state = BP_LEN_H;
        break;

    case BP_LEN_H:
        bp->rx_buf[7] = byte;
        bp->rx_payload_len |= (uint16_t)(byte << 8);
        bp->rx_pos = PROTO_FRAME_OVERHEAD;  /* next byte goes into payload */

        if (bp->rx_payload_len > PROTO_MAX_PAYLOAD)
        {
            /* Payload too large — discard frame, resync */
            bp->rx_state = BP_SYNC1;
            bp->sync_losses++;
            if (byte == PROTO_MAGIC_0)
            {
                bp->rx_buf[0] = byte;
                bp->rx_pos = 1;
                bp->rx_state = BP_SYNC2;
            }
        }
        else if (bp->rx_payload_len == 0U)
        {
            /* No payload — skip to CRC */
            bp->rx_state = BP_CRC_L;
        }
        else
        {
            bp->rx_state = BP_PAYLOAD;
        }
        break;

    case BP_PAYLOAD:
        if (bp->rx_pos < PROTO_BUF_SIZE)
        {
            bp->rx_buf[bp->rx_pos] = byte;
        }
        bp->rx_pos++;

        if (bp->rx_pos >= (PROTO_FRAME_OVERHEAD + bp->rx_payload_len))
        {
            bp->rx_state = BP_CRC_L;
        }
        break;

    case BP_CRC_L:
        bp->rx_crc = byte;
        bp->rx_state = BP_CRC_H;
        break;

    case BP_CRC_H:
    {
        bp->rx_crc |= (uint16_t)(byte << 8);

        /* Validate CRC over header + payload */
        uint16_t computed = proto_crc16(bp->rx_buf,
                                         PROTO_FRAME_OVERHEAD + bp->rx_payload_len);

        if (computed == bp->rx_crc)
        {
            bp->good_frames++;
            /* Frame is valid — handler will be called from task context.
             * Store frame info for later dispatch. */
            bp_dispatch_frame(bp, bp->rx_msg_type, bp->rx_seq,
                              bp->rx_buf + PROTO_FRAME_OVERHEAD,
                              bp->rx_payload_len);
        }
        else
        {
            bp->crc_errors++;
        }

        /* Back to hunting for next sync */
        bp->rx_state = BP_SYNC1;
        break;
    }
    }
}

/* ------------------------------------------------------------------ */
/*  Feed buffer                                                         */
/* ------------------------------------------------------------------ */

void bp_feed_bytes(BinaryProtocol *bp, const uint8_t *data, uint16_t len)
{
    if (bp == nullptr || data == nullptr) return;
    for (uint16_t i = 0U; i < len; i++)
    {
        bp_feed_byte(bp, data[i]);
    }
}

/* ------------------------------------------------------------------ */
/*  TX queue read                                                       */
/* ------------------------------------------------------------------ */

bool bp_get_tx_frame(BinaryProtocol *bp, uint8_t *buf, uint16_t buf_size,
                     uint16_t *out_len)
{
    if (bp == nullptr || buf == nullptr || out_len == nullptr) return false;

    if (tx_queue_is_empty(&bp->tx_high) &&
        tx_queue_is_empty(&bp->tx_normal))
    {
        *out_len = 0U;
        return false;
    }

    if (tx_queue_pop(&bp->tx_high, buf, buf_size, out_len))
    {
        return true;
    }
    return tx_queue_pop(&bp->tx_normal, buf, buf_size, out_len);
}

/* ------------------------------------------------------------------ */
/*  Queue outbound frames                                               */
/* ------------------------------------------------------------------ */

bool bp_send_frame(BinaryProtocol *bp, uint8_t type,
                   const uint8_t *payload, uint16_t payload_len)
{
    const BpTxPriority priority =
        (type == ProtoMsg_ACK || type == ProtoMsg_NACK ||
         type == ProtoMsg_HEARTBEAT_ACK || type == ProtoMsg_SAFETY_EVENT)
            ? BP_TX_PRIORITY_HIGH
            : BP_TX_PRIORITY_NORMAL;
    return bp_send_frame_priority(bp, type, payload, payload_len, priority);
}

bool bp_send_frame_priority(BinaryProtocol *bp, uint8_t type,
                            const uint8_t *payload, uint16_t payload_len,
                            BpTxPriority priority)
{
    if (bp == nullptr) return false;
    bp->tx_frames_total++;

    if ((payload_len > 0U && payload == nullptr) ||
        payload_len > PROTO_MAX_PAYLOAD)
    {
        bp->tx_dropped_frames++;
        return false;
    }

    BpTxQueue *queue = (priority == BP_TX_PRIORITY_HIGH)
                           ? &bp->tx_high
                           : &bp->tx_normal;
    if (tx_queue_is_full(queue))
    {
        bp->tx_queue_full_count++;
        bp->tx_dropped_frames++;
        return false;
    }

    uint8_t frame[PROTO_BUF_SIZE];
    const uint16_t total = proto_encode_frame(
        frame, sizeof(frame), type, bp->tx_seq, payload, payload_len);
    if (total == 0U)
    {
        bp->tx_dropped_frames++;
        return false;
    }

    if (!tx_queue_push(queue, frame, total))
    {
        bp->tx_dropped_frames++;
        return false;
    }

    bp->tx_seq++;
    return true;
}

bool bp_send_nack(BinaryProtocol *bp, uint16_t rejected_sequence,
                  uint8_t original_type, uint8_t reason)
{
    ProtoNack nack;
    nack.rejected_sequence = rejected_sequence;
    nack.original_type = original_type;
    nack.reason = reason;
    return bp_send_frame(bp, ProtoMsg_NACK,
                         reinterpret_cast<const uint8_t *>(&nack),
                         sizeof(nack));
}

void bp_note_tx_result(BinaryProtocol *bp, bool success)
{
    if (bp != nullptr && !success)
    {
        bp->tx_send_failures++;
    }
}

bool bp_expected_command_payload_length(uint8_t type, uint16_t *out_len)
{
    if (out_len == nullptr) return false;

    switch (type)
    {
    case ProtoMsg_NOP:
    case ProtoMsg_ARM:
    case ProtoMsg_DISARM:
    case ProtoMsg_EMERGENCY_STOP:
    case ProtoMsg_RESET_ESTOP:
    case ProtoMsg_ENTER_MANUAL:
    case ProtoMsg_EXIT_MANUAL:
    case ProtoMsg_SET_ALL_NEUTRAL:
    case ProtoMsg_FLOAT_ON:
    case ProtoMsg_FLOAT_OFF:
    case ProtoMsg_ANGLE_ON:
    case ProtoMsg_ANGLE_OFF:
    case ProtoMsg_REQUEST_STATUS:
    case ProtoMsg_REQUEST_SENSORS:
        *out_len = 0U;
        return true;

    case ProtoMsg_SET_PWM:
        *out_len = sizeof(ProtoSetPwm);
        return true;
    case ProtoMsg_SET_DEPTH:
        *out_len = sizeof(ProtoSetDepth);
        return true;
    case ProtoMsg_SET_YAW:
        *out_len = sizeof(ProtoSetYaw);
        return true;
    case ProtoMsg_SET_MOTION:
        *out_len = sizeof(ProtoSetMotion);
        return true;
    case ProtoMsg_HEARTBEAT:
        *out_len = sizeof(ProtoHeartbeat);
        return true;
    default:
        return false;
    }
}

bool bp_send_ack(BinaryProtocol *bp, uint16_t ack_seq)
{
    ProtoAck ack;
    ack.ack_seq = ack_seq;
    return bp_send_frame(bp, ProtoMsg_ACK,
                         reinterpret_cast<const uint8_t *>(&ack),
                         sizeof(ack));
}

/* ------------------------------------------------------------------ */
/*  Reset                                                               */
/* ------------------------------------------------------------------ */

void bp_reset_rx(BinaryProtocol *bp)
{
    if (bp == nullptr) return;
    bp->rx_state = BP_SYNC1;
    bp->rx_pos = 0;
}
