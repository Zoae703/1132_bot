#include "PCA9685.h"

#include "i2c.h"
#include "tca9548a.h"

#include <cmath>

#define LED0_ON_H  0x07U
#define LED0_OFF_L 0x08U
#define LED0_OFF_H 0x09U

#define PCA9685_TCA_CHANNEL 4U
#define PCA9685_HAL_TIMEOUT_MS 20U
#define PCA9685_FIXED_PRESCALE_50HZ 121U
#define PCA9685_ACTUAL_PERIOD_US 19450U
#define PCA9685_PWM_NEUTRAL_US 1500
#define PCA9685_PWM_MIN_US 1300
#define PCA9685_PWM_MAX_US 1700

static I2C_HandleTypeDef *pca_i2c = nullptr;

static bool PCA9685_BeginTransaction(void)
{
    if (pca_i2c == nullptr) {
        return false;
    }

    if (pca_i2c == &hi2c2) {
        if (!I2C2_BusLock(PCA9685_HAL_TIMEOUT_MS)) {
            return false;
        }
        if (TCA9548A_SelectChannel(pca_i2c, PCA9685_TCA_CHANNEL) != HAL_OK) {
            I2C2_BusUnlock();
            return false;
        }
    }

    return true;
}

static void PCA9685_EndTransaction(void)
{
    if (pca_i2c == &hi2c2) {
        I2C2_BusUnlock();
    }
}

static bool PCA9685_ReadUnlocked(uint8_t reg, uint8_t *value)
{
    if ((pca_i2c == nullptr) || (value == nullptr)) {
        return false;
    }

    if (HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, &reg, 1,
                                PCA9685_HAL_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    return HAL_I2C_Master_Receive(pca_i2c, PCA9685_ADDR, value, 1,
                                  PCA9685_HAL_TIMEOUT_MS) == HAL_OK;
}

static bool PCA9685_WriteUnlocked(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    if (pca_i2c == nullptr) {
        return false;
    }

    return HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, tx, sizeof(tx),
                                   PCA9685_HAL_TIMEOUT_MS) == HAL_OK;
}

static bool PCA9685_WritePwmUnlocked(uint8_t channel, uint16_t on, uint16_t off)
{
    if ((pca_i2c == nullptr) || (channel >= 16U)) {
        return false;
    }

    uint8_t tx[5];
    tx[0] = static_cast<uint8_t>(LED0_ON_L + (4U * channel));
    tx[1] = static_cast<uint8_t>(on & 0xFFU);
    tx[2] = static_cast<uint8_t>((on >> 8) & 0x0FU);
    tx[3] = static_cast<uint8_t>(off & 0xFFU);
    tx[4] = static_cast<uint8_t>((off >> 8) & 0x0FU);

    return HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, tx, sizeof(tx),
                                   PCA9685_HAL_TIMEOUT_MS) == HAL_OK;
}

static uint16_t PCA9685_UsToCount(int32_t pwm_us)
{
    uint32_t count;

    if (pwm_us < PCA9685_PWM_MIN_US) {
        pwm_us = PCA9685_PWM_MIN_US;
    }
    if (pwm_us > PCA9685_PWM_MAX_US) {
        pwm_us = PCA9685_PWM_MAX_US;
    }

    count = ((static_cast<uint32_t>(pwm_us) * 4096U) +
             (PCA9685_ACTUAL_PERIOD_US / 2U)) / PCA9685_ACTUAL_PERIOD_US;
    if (count > 4095U) {
        count = 4095U;
    }

    return static_cast<uint16_t>(count);
}

static bool PCA9685_SetFreq50HzUnlocked(void)
{
    uint8_t oldmode = 0U;
    if (!PCA9685_ReadUnlocked(PCA9685_MODE1, &oldmode)) {
        return false;
    }

    const uint8_t sleep_mode = static_cast<uint8_t>((oldmode & 0x7FU) | 0x10U);
    if (!PCA9685_WriteUnlocked(PCA9685_MODE1, sleep_mode)) {
        return false;
    }
    if (!PCA9685_WriteUnlocked(PCA9685_PRESCALE, PCA9685_FIXED_PRESCALE_50HZ)) {
        return false;
    }
    if (!PCA9685_WriteUnlocked(PCA9685_MODE1, oldmode)) {
        return false;
    }

    HAL_Delay(2);
    return PCA9685_WriteUnlocked(PCA9685_MODE1, static_cast<uint8_t>(oldmode | 0xA1U));
}

bool PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off)
{
    bool ok;

    if (!PCA9685_BeginTransaction()) {
        return false;
    }
    ok = PCA9685_WritePwmUnlocked(channel, static_cast<uint16_t>(on),
                                  static_cast<uint16_t>(off));
    PCA9685_EndTransaction();
    return ok;
}

bool PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    bool ok = true;

    pca_i2c = hi2c;
    if (!PCA9685_BeginTransaction()) {
        return false;
    }

    ok = ok && PCA9685_WriteUnlocked(PCA9685_MODE1, 0x00U);
    ok = ok && PCA9685_SetFreq50HzUnlocked();
    for (uint8_t i = 0U; (i < 16U) && ok; i++) {
        ok = PCA9685_WritePwmUnlocked(i, 0U, PCA9685_UsToCount(PCA9685_PWM_NEUTRAL_US));
    }

    PCA9685_EndTransaction();
    return ok;
}

bool PCA9685_SetAllPWM(const int32_t pwm_us[8])
{
    bool ok = true;

    if (pwm_us == nullptr) {
        return false;
    }

    if (!PCA9685_BeginTransaction()) {
        return false;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        ok = ok && PCA9685_WritePwmUnlocked(i, 0U, PCA9685_UsToCount(pwm_us[i]));
    }

    PCA9685_EndTransaction();
    return ok;
}
