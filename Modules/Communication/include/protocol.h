#pragma once
#include "main.h"
#include <cstdint>

#define COMM_BUF_LEN 100

// DMA RX target buffer (used by the USART6 RX-event callback in robot_tasks.cpp).
extern uint8_t CommRxBuffer[COMM_BUF_LEN];

void Comm_Init();              // start DMA RX-to-idle on USART6
void Comm_OnRxEvent(uint16_t size);  // called from HAL_UARTEx_RxEventCallback
void Comm_Process();           // parse last command, write to DataBus
