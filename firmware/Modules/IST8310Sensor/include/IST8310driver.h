#pragma once
#include "main.h"
#include <cstdint>

#define IST8310_IIC_ADDRESS  (0x0E << 1)
#define MAG_SEN  0.3f  // µT per LSB

uint8_t ist8310_init(I2C_HandleTypeDef* hi2c);
void ist8310_read_mag(I2C_HandleTypeDef* hi2c, float mag[3]);
