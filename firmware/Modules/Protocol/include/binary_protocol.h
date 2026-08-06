/**
 * @file    binary_protocol.h
 * @brief   Non-blocking binary protocol parser for STM32.
 *
 * Feeds bytes one-at-a-time from task context and dispatches
 * complete, CRC-validated frames to the protocol handler.
 */

#ifndef BINARY_PROTOCOL_H
#define BINARY_PROTOCOL_H

#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Parser state                                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    BP_SYNC1 = 0,
    BP_SYNC2,
    BP_VERSION,
    BP_TYPE,
    BP_SEQ_L,
    BP_SEQ_H,
    BP_LEN_L,
    BP_LEN_H,
    BP_PAYLOAD,
    BP_CRC_L,
    BP_CRC_H,
} BpRxState;

/* ------------------------------------------------------------------ */
/*  Parser instance (one per UART)                                     */
/* ------------------------------------------------------------------ */

#define BP_TX_HIGH_QUEUE_DEPTH   8U
#define BP_TX_NORMAL_QUEUE_DEPTH 8U

#if BP_TX_HIGH_QUEUE_DEPTH != BP_TX_NORMAL_QUEUE_DEPTH
#error "BpTxQueue currently requires equal high and normal queue depths"
#endif

typedef enum {
    BP_TX_PRIORITY_NORMAL = 0,
    BP_TX_PRIORITY_HIGH   = 1,
} BpTxPriority;

typedef struct {
    uint8_t  frames[BP_TX_HIGH_QUEUE_DEPTH][PROTO_BUF_SIZE];
    uint16_t lengths[BP_TX_HIGH_QUEUE_DEPTH];
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;
} BpTxQueue;

typedef struct {
    /* RX state machine */
    BpRxState rx_state;
    uint8_t   rx_buf[PROTO_BUF_SIZE];
    uint16_t  rx_pos;
    uint8_t   rx_msg_type;
    uint16_t  rx_seq;
    uint16_t  rx_payload_len;
    uint16_t  rx_crc;

    /* Statistics */
    uint32_t  good_frames;
    uint32_t  crc_errors;
    uint32_t  sync_losses;

    /* Separate response/control and telemetry queues. */
    BpTxQueue tx_high;
    BpTxQueue tx_normal;
    uint16_t  tx_seq;

    /* TX diagnostics.  tx_frames_total counts enqueue attempts. */
    uint32_t  tx_frames_total;
    uint32_t  tx_dropped_frames;
    uint32_t  tx_queue_full_count;
    uint32_t  tx_send_failures;
} BinaryProtocol;

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise the parser instance.
 */
void bp_init(BinaryProtocol *bp);

/**
 * @brief  Feed a single byte into the parser.
 *
 * When a complete frame is received (CRC valid), the frame is dispatched
 * to bp_dispatch_frame().  Call this from the communication task, never from
 * an ISR: dispatch performs state transitions and queues responses.
 *
 * @param bp   Parser instance.
 * @param byte The received byte.
 */
void bp_feed_byte(BinaryProtocol *bp, uint8_t byte);

/**
 * @brief  Feed a buffer of bytes into the parser.
 *
 * Equivalent to calling bp_feed_byte() for each byte.
 */
void bp_feed_bytes(BinaryProtocol *bp, const uint8_t *data, uint16_t len);

/**
 * @brief  Get the next outbound frame to transmit.
 *
 * @param bp       Parser instance.
 * @param buf      Output buffer.
 * @param buf_size Size of output buffer.
 * @param out_len  Receives the frame length (0 if nothing to send).
 * @return true if a frame was copied, false if TX queue is empty.
 */
bool bp_get_tx_frame(BinaryProtocol *bp, uint8_t *buf, uint16_t buf_size,
                     uint16_t *out_len);

/**
 * @brief  Queue an outbound frame for transmission.
 *
 * @param bp          Parser instance.
 * @param type        Message type.
 * @param payload     Payload bytes (may be NULL if len == 0).
 * @param payload_len Payload length.
 * @return true if queued, false if TX queue is full.
 */
bool bp_send_frame(BinaryProtocol *bp, uint8_t type,
                   const uint8_t *payload, uint16_t payload_len);

/** Queue a frame with an explicit priority. */
bool bp_send_frame_priority(BinaryProtocol *bp, uint8_t type,
                            const uint8_t *payload, uint16_t payload_len,
                            BpTxPriority priority);

/**
 * @brief  Queue a NACK response.
 */
bool bp_send_nack(BinaryProtocol *bp, uint16_t rejected_sequence,
                  uint8_t original_type, uint8_t reason);

/**
 * @brief  Queue an ACK response (echoes the sequence number).
 */
bool bp_send_ack(BinaryProtocol *bp, uint16_t ack_seq);

/** Record the result of the physical UART transmit after a frame is popped. */
void bp_note_tx_result(BinaryProtocol *bp, bool success);

/** Return the exact payload size for an inbound host command. */
bool bp_expected_command_payload_length(uint8_t type, uint16_t *out_len);

/** Build state/sensor reports from a consistent RobotData snapshot. */
bool bp_queue_status_report(BinaryProtocol *bp, BpTxPriority priority);
bool bp_queue_sensor_report(BinaryProtocol *bp, BpTxPriority priority);
bool bp_queue_motion_tuning_report(BinaryProtocol *bp,
                                   BpTxPriority priority);
bool bp_queue_depth_pid_tuning_report(BinaryProtocol *bp,
                                     BpTxPriority priority);
bool bp_queue_depth_control_report(BinaryProtocol *bp,
                                  BpTxPriority priority);

/**
 * @brief  Reset the RX parser (e.g. after a timeout / sync loss).
 */
void bp_reset_rx(BinaryProtocol *bp);

/**
 * @brief  Get the shared protocol instance (singleton).
 */
BinaryProtocol *bp_get_instance(void);

/**
 * @brief  Called by the parser when a complete validated frame is received.
 *         Implemented in protocol_handler.cpp.
 */
void bp_dispatch_frame(BinaryProtocol *bp, uint8_t type, uint16_t seq,
                       const uint8_t *payload, uint16_t payload_len);

/* ------------------------------------------------------------------ */
/*  Thread-safe log queue (MPSC — tasks push, CommTask drains)         */
/* ------------------------------------------------------------------ */

#define PROTO_LOG_QUEUE_LENGTH 32U
#define PROTO_LOG_MSG_MAX      128U

typedef struct {
    char message[PROTO_LOG_MSG_MAX];
} ProtocolLogEntry;

/** Initialise the log queue.  Must be called before the scheduler starts
 *  and before any task that may call Protocol_LogQueuePush(). */
bool Protocol_LogQueueInit(void);

/** Push a log message (task context only, zero-wait send).
 *  The message is copied into the queue entry.  On queue full the entry
 *  is dropped and an internal drop counter is incremented. */
void Protocol_LogQueuePush(const char *message);

/** Drain one entry and send it via bp_send_frame_priority.
 *  Only CommTask may call this. */
void Protocol_LogQueueDrainOne(void);

/** Return the number of messages dropped due to a full queue. */
uint32_t Protocol_LogQueueDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BINARY_PROTOCOL_H */
