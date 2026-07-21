#include "../include/protocol.h"

#include "task.h"
#include "usart.h"

#include <cstring>

uint8_t CommRxBuffer[COMM_UART_RX_CHUNK_SIZE];

namespace {

StaticStreamBuffer_t rx_stream_control;
uint8_t rx_stream_storage[COMM_RX_STREAM_SIZE];
StreamBufferHandle_t rx_stream = nullptr;

CommUartDiagnostics diagnostics{};
volatile bool rx_resync_requested = false;
volatile bool rx_rearm_pending = true;

} // namespace

HAL_StatusTypeDef Comm_StartReceiveToIdle()
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(
        &huart6, CommRxBuffer, COMM_UART_RX_CHUNK_SIZE);

    /* HAL_BUSY with an active RX operation means another context already
     * completed the requested arm; it is not a receive outage. */
    if (status == HAL_BUSY && huart6.RxState != HAL_UART_STATE_READY)
    {
        rx_rearm_pending = false;
        return HAL_OK;
    }

    rx_rearm_pending = (status != HAL_OK);
    if (status != HAL_OK)
    {
        diagnostics.rx_rearm_failures++;
    }
    return status;
}

bool Comm_Init()
{
    if (rx_stream == nullptr)
    {
        std::memset(&diagnostics, 0, sizeof(diagnostics));
        rx_stream = xStreamBufferCreateStatic(
            COMM_RX_STREAM_SIZE, 1U, rx_stream_storage, &rx_stream_control);
        if (rx_stream == nullptr)
        {
            return false;
        }
    }

    rx_resync_requested = false;
    return Comm_StartReceiveToIdle() == HAL_OK;
}

size_t Comm_OnRxEventFromISR(uint16_t size,
                             BaseType_t *higher_priority_task_woken)
{
    diagnostics.rx_callbacks++;
    diagnostics.last_rx_event_size = size;

    uint16_t copy_size = size;
    if (copy_size > COMM_UART_RX_CHUNK_SIZE)
    {
        copy_size = COMM_UART_RX_CHUNK_SIZE;
    }
    diagnostics.rx_bytes_total += copy_size;

    size_t written = 0U;
    if (rx_stream != nullptr && copy_size > 0U)
    {
        written = xStreamBufferSendFromISR(
            rx_stream, CommRxBuffer, copy_size, higher_priority_task_woken);
    }

    if (written < copy_size)
    {
        diagnostics.rx_overflow_count++;
        diagnostics.rx_dropped_bytes +=
            static_cast<uint32_t>(copy_size - written);
        rx_resync_requested = true;
    }
    return written;
}

void Comm_OnUartErrorFromISR(uint32_t error_code)
{
    diagnostics.uart_error_count++;
    diagnostics.last_uart_error = error_code;
    rx_resync_requested = true;
    rx_rearm_pending = true;
}

size_t Comm_ReadRx(uint8_t *buffer, size_t buffer_size,
                   TickType_t timeout_ticks)
{
    if (rx_stream == nullptr || buffer == nullptr || buffer_size == 0U)
    {
        return 0U;
    }
    return xStreamBufferReceive(
        rx_stream, buffer, buffer_size, timeout_ticks);
}

bool Comm_TakeRxResyncRequest()
{
    bool requested;
    taskENTER_CRITICAL();
    requested = rx_resync_requested;
    rx_resync_requested = false;
    taskEXIT_CRITICAL();
    return requested;
}

void Comm_EnsureReceiveArmed()
{
    if (rx_rearm_pending)
    {
        (void)Comm_StartReceiveToIdle();
    }
}

const CommUartDiagnostics *Comm_GetUartDiagnostics()
{
    return &diagnostics;
}
