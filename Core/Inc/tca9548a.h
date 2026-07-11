#ifndef TCA9548A_H
#define TCA9548A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TCA9548A_ADDR (0x70U << 1)

HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);
HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* TCA9548A_H */
