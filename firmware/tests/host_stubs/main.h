#pragma once
#include <cstdint>

typedef struct {
    void *Instance;
} I2C_HandleTypeDef;

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U,
} HAL_StatusTypeDef;

extern "C" {
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(
    I2C_HandleTypeDef *hi2c, uint16_t device_address, uint8_t *data,
    uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef HAL_I2C_Master_Receive(
    I2C_HandleTypeDef *hi2c, uint16_t device_address, uint8_t *data,
    uint16_t length, uint32_t timeout_ms);
}
