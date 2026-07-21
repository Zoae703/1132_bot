#pragma once
#include "include/binary_protocol.h"

/**
 * @brief  Protocol module — provides the shared BinaryProtocol singleton
 *         and the bp_dispatch_frame callback.
 *
 * Usage from robot_tasks.cpp:
 *   - Call bp_get_instance() to obtain the parser.
 *   - Feed USART6 RX bytes via bp_feed_bytes().
 *   - Drain TX queue with bp_get_tx_frame() and transmit.
 */
