#include "PCA9685.h"
#include "i2c.h"
#include <cmath>

#define LED0_ON_H  0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA9685_HAL_TIMEOUT_MS 10000U

static I2C_HandleTypeDef* pca_i2c = nullptr;

static bool PCA9685_LockBus(void) {
    if (pca_i2c == &hi2c2) {
        return I2C2_BusLock(PCA9685_HAL_TIMEOUT_MS);
    }
    return true;
}

static void PCA9685_UnlockBus(void) {
    if (pca_i2c == &hi2c2) {
        I2C2_BusUnlock();
    }
}

static uint8_t PCA9685_Read(uint8_t reg) {
    uint8_t tx = reg, buf = 0;
    if (!PCA9685_LockBus()) {
        return 0;
    }
    HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, &tx, 1, PCA9685_HAL_TIMEOUT_MS);
    HAL_I2C_Master_Receive(pca_i2c, PCA9685_ADDR, &buf, 1, PCA9685_HAL_TIMEOUT_MS);
    PCA9685_UnlockBus();
    return buf;
}

static void PCA9685_Write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg, data};
    if (!PCA9685_LockBus()) {
        return;
    }
    HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, tx, 2, PCA9685_HAL_TIMEOUT_MS);
    PCA9685_UnlockBus();
}

static void PCA9685_SetFreq(float freq) {
    freq *= 1.016f;  // empirical correction factor (from reference)
    double prescaleval = 25000000.0 / (4096.0 * freq) - 1.0;
    uint8_t prescale = (uint8_t)floor(prescaleval + 0.5f);
    uint8_t oldmode = PCA9685_Read(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;  // sleep
    PCA9685_Write(PCA9685_MODE1, newmode);
    PCA9685_Write(PCA9685_PRESCALE, prescale);
    PCA9685_Write(PCA9685_MODE1, oldmode);
    HAL_Delay(2);
    PCA9685_Write(PCA9685_MODE1, oldmode | 0xA1);
}

void PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off) {
    PCA9685_Write(LED0_ON_L  + 4 * channel, on);
    PCA9685_Write(LED0_ON_H  + 4 * channel, on >> 8);
    PCA9685_Write(LED0_OFF_L + 4 * channel, off);
    PCA9685_Write(LED0_OFF_H + 4 * channel, off >> 8);
}

void PCA9685_Init(I2C_HandleTypeDef* hi2c) {
    pca_i2c = hi2c;
    PCA9685_Write(PCA9685_MODE1, 0x00);
    PCA9685_SetFreq(50.0f);
}

void PCA9685_SetAllPWM(const int32_t pwm_us[8]) {
    for (int i = 0; i < 8; i++) {
        int32_t pwm = pwm_us[i];
        if (pwm < 1000) pwm = 1000;
        if (pwm > 2000) pwm = 2000;
        int32_t duty = (pwm * 4096 + 10000) / 20000;
        PCA9685_SetPWM(i, 0, duty);
    }
}
