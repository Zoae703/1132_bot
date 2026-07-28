#pragma once

#include "main.h"

#include <cstdint>

extern "C" {

extern I2C_HandleTypeDef hi2c2;

bool I2C2_BusLock(uint32_t timeout_ms);
void I2C2_BusUnlock(void);

}
