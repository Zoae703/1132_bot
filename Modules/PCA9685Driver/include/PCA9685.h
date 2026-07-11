#pragma once
#include "main.h"
#include <cstdint>

#define PCA9685_ADDR     0x80
#define PCA9685_MODE1    0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06

bool PCA9685_Init(I2C_HandleTypeDef* hi2c);
bool PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off);
bool PCA9685_SetAllPWM(const int32_t pwm_us[8]);
