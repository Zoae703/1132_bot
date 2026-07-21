#pragma once

#include "main.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"

#include <cstddef>
#include <cstdint>

#define COMM_UART_RX_CHUNK_SIZE 128U
#define COMM_RX_STREAM_SIZE     1024U

extern uint8_t CommRxBuffer[COMM_UART_RX_CHUNK_SIZE];

struct CommUartDiagnostics {
    volatile uint32_t rx_callbacks;
    volatile uint32_t rx_bytes_total;
    volatile uint32_t rx_dropped_bytes;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t rx_rearm_failures;
    volatile uint32_t last_uart_error;
    volatile uint16_t last_rx_event_size;
};

/** Create the RX stream and start the one USART6 ReceiveToIdle operation. */
bool Comm_Init();

/** Sole wrapper around HAL_UARTEx_ReceiveToIdle_IT. */
HAL_StatusTypeDef Comm_StartReceiveToIdle();

/** Copy one HAL RX event into the stream.  ISR-safe and non-blocking. */
size_t Comm_OnRxEventFromISR(uint16_t size,
                             BaseType_t *higher_priority_task_woken);

/** Record a UART error and request parser resynchronisation. */
void Comm_OnUartErrorFromISR(uint32_t error_code);

/** Receive raw bytes in CommunicationTask context. */
size_t Comm_ReadRx(uint8_t *buffer, size_t buffer_size,
                   TickType_t timeout_ticks);

/** Atomically consume the parser-resynchronisation request flag. */
bool Comm_TakeRxResyncRequest();

/** Retry a failed ReceiveToIdle arm from task context. */
void Comm_EnsureReceiveArmed();

const CommUartDiagnostics *Comm_GetUartDiagnostics();
