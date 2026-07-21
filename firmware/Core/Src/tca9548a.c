#include "tca9548a.h"

HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel)
{
  if ((hi2c == NULL) || (channel > 7U))
  {
    return HAL_ERROR;
  }

  uint8_t data = (uint8_t)(1U << channel);
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)TCA9548A_ADDR, &data, 1, 10);
}

HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  uint8_t data = 0U;
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)TCA9548A_ADDR, &data, 1, 10);
}
