/**
 * @file    queue.h
 * @brief   Minimal FreeRTOS queue.h stub for host testing.
 *
 * Provides just enough definitions so that protocol_handler.cpp
 * (which uses the FreeRTOS static queue for the MPSC log queue)
 * compiles in the host test environment.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#ifndef pdFALSE
#define pdFALSE 0
#endif

#ifndef pdTRUE
#define pdTRUE  1
#endif

#ifndef pdPASS
#define pdPASS  1
#endif

#ifndef pdFAIL
#define pdFAIL  0
#endif

#ifndef configASSERT
#define configASSERT(x) ((void)0)
#endif

/* Opaque types — host tests don't exercise the real scheduler. */
typedef void * QueueHandle_t;
typedef void * QueueSetHandle_t;
typedef void * QueueSetMemberHandle_t;

struct StaticQueue_t {
    char opaque[128];
};

/* Stub: xQueueCreateStatic always returns NULL (queue never ready).
 * Existing protocol_handler_host_test does not exercise the log queue
 * path; this stub lets it compile unchanged. */
inline QueueHandle_t xQueueCreateStatic(
    unsigned, unsigned, uint8_t *, StaticQueue_t *)
{
    return nullptr;
}

inline int xQueueSend(QueueHandle_t, const void *, unsigned)
{
    return pdFAIL;
}

inline int xQueueReceive(QueueHandle_t, void *, unsigned)
{
    return pdFAIL;
}
