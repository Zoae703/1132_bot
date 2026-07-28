#pragma once

#include "main.h"

#include <cstdint>

#define TCA9548A_ADDR (0x70U << 1)

extern "C" {

HAL_StatusTypeDef TCA9548A_SelectChannel(
    I2C_HandleTypeDef *hi2c, uint8_t channel);
HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c);

}
