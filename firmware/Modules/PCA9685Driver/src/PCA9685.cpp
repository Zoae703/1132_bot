#include "PCA9685.h"

#include "i2c.h"
#include "tca9548a.h"

#include <cmath>

#define LED0_ON_H  0x07U
#define LED0_OFF_L 0x08U
#define LED0_OFF_H 0x09U
#define ALL_LED_OFF_H 0xFDU

#define PCA9685_TCA_CHANNEL 4U
#define PCA9685_BUS_LOCK_TIMEOUT_MS 150U
#define PCA9685_HAL_TIMEOUT_MS 20U
#define PCA9685_FIXED_PRESCALE_50HZ 121U
#define PCA9685_ACTUAL_PERIOD_US 19450U
#define PCA9685_PWM_NEUTRAL_US 1500
#define PCA9685_PWM_MIN_US 1000
#define PCA9685_PWM_MAX_US 2000

namespace {

constexpr uint8_t kMode1Restart = 0x80U;
constexpr uint8_t kMode1ExternalClock = 0x40U;
constexpr uint8_t kMode1AutoIncrement = 0x20U;
constexpr uint8_t kMode1Sleep = 0x10U;
constexpr uint8_t kMode1SubAddressMask = 0x0EU;
constexpr uint8_t kMode1AllCall = 0x01U;
constexpr uint8_t kMode1Configured = kMode1AutoIncrement;
constexpr uint8_t kMode1RelevantMask =
    kMode1ExternalClock | kMode1AutoIncrement |
    kMode1Sleep | kMode1SubAddressMask | kMode1AllCall;
constexpr uint8_t kMode2Invert = 0x10U;
constexpr uint8_t kMode2OutputChangeOnAck = 0x08U;
constexpr uint8_t kMode2TotemPole = 0x04U;
constexpr uint8_t kMode2OutputNotEnabledMask = 0x03U;
constexpr uint8_t kMode2RelevantMask =
    kMode2Invert | kMode2OutputChangeOnAck | kMode2TotemPole |
    kMode2OutputNotEnabledMask;
constexpr uint8_t kMode2Configured = kMode2TotemPole;
constexpr uint8_t kFullOffBit = 0x10U;
constexpr uint8_t kPwmRegisterBytes = 4U;
constexpr uint8_t kPcaChannelCount = 16U;
constexpr uint8_t kMotorChannelCount = 8U;

} // namespace

static I2C_HandleTypeDef *pca_i2c = nullptr;
static PCA9685Diagnostics pca9685_diagnostics{};

static void PCA9685_RecordFailure(PCA9685InitFailurePhase phase)
{
    ++pca9685_diagnostics.init_failures;
    pca9685_diagnostics.last_failure_phase =
        static_cast<uint8_t>(phase);
    if (phase < PCA9685_PHASE_COUNT) {
        ++pca9685_diagnostics.phase_failures[phase];
    }
}

void PCA9685_GetDiagnostics(PCA9685Diagnostics *diagnostics)
{
    if (diagnostics == nullptr) {
        return;
    }
    *diagnostics = pca9685_diagnostics;
}

static PCA9685TransactionStatus PCA9685_BeginTransaction(void)
{
    if (pca_i2c == nullptr) {
        return PCA9685_TXN_TCA_SELECT_FAILED;
    }

    if (pca_i2c == &hi2c2) {
        /*
         * MS5837 shares I2C2 and may hold the bus mutex for a HAL transfer
         * whose timeout is 100 ms. Do not turn normal mutex contention into
         * a false actuator fault; individual PCA transfers remain bounded
         * by the shorter HAL timeout below.
         */
        if (!I2C2_BusLock(PCA9685_BUS_LOCK_TIMEOUT_MS)) {
            return PCA9685_TXN_BUS_LOCK_TIMEOUT;
        }
        if (TCA9548A_SelectChannel(pca_i2c, PCA9685_TCA_CHANNEL) != HAL_OK) {
            I2C2_BusUnlock();
            return PCA9685_TXN_TCA_SELECT_FAILED;
        }
    }

    return PCA9685_TXN_OK;
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

static bool PCA9685_ReadBlockUnlocked(uint8_t start_reg, uint8_t *data,
                                      uint16_t length)
{
    if ((pca_i2c == nullptr) || (data == nullptr) || (length == 0U)) {
        return false;
    }

    if (HAL_I2C_Master_Transmit(pca_i2c, PCA9685_ADDR, &start_reg, 1U,
                                PCA9685_HAL_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    return HAL_I2C_Master_Receive(pca_i2c, PCA9685_ADDR, data, length,
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

uint16_t PCA9685_PwmUsToCount(int32_t pwm_us)
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

static bool PCA9685_WritePwmBatchUnlocked(const int32_t *pwm_us,
                                          uint8_t channel_count)
{
    if ((pwm_us == nullptr) || (channel_count == 0U) ||
        (channel_count > kPcaChannelCount)) {
        return false;
    }

    uint8_t tx[1U + (kPwmRegisterBytes * kPcaChannelCount)]{};
    tx[0] = LED0_ON_L;
    for (uint8_t channel = 0U; channel < channel_count; ++channel) {
        const uint16_t count = PCA9685_PwmUsToCount(pwm_us[channel]);
        const uint16_t offset =
            static_cast<uint16_t>(1U + (kPwmRegisterBytes * channel));
        tx[offset] = 0U;
        tx[offset + 1U] = 0U;
        tx[offset + 2U] = static_cast<uint8_t>(count & 0xFFU);
        tx[offset + 3U] =
            static_cast<uint8_t>((count >> 8U) & 0x0FU);
    }

    const uint16_t tx_length = static_cast<uint16_t>(
        1U + (kPwmRegisterBytes * channel_count));
    return HAL_I2C_Master_Transmit(
               pca_i2c, PCA9685_ADDR, tx, tx_length,
               PCA9685_HAL_TIMEOUT_MS) == HAL_OK;
}

static bool PCA9685_ForceOutputsOffUnlocked()
{
    /* ALL_LED_* writes atomically apply FULL_OFF to all channels. */
    return PCA9685_WriteUnlocked(ALL_LED_OFF_H, kFullOffBit);
}

static bool PCA9685_VerifyConfigurationUnlocked(bool expect_sleep)
{
    uint8_t mode1 = 0U;
    uint8_t mode2 = 0U;
    uint8_t prescale = 0U;
    if (!PCA9685_ReadUnlocked(PCA9685_MODE1, &mode1) ||
        !PCA9685_ReadUnlocked(PCA9685_MODE2, &mode2) ||
        !PCA9685_ReadUnlocked(PCA9685_PRESCALE, &prescale)) {
        return false;
    }

    const uint8_t expected_mode1 = static_cast<uint8_t>(
        kMode1Configured | (expect_sleep ? kMode1Sleep : 0U));
    if ((mode1 & kMode1RelevantMask) != expected_mode1 ||
        (mode2 & kMode2RelevantMask) != kMode2Configured ||
        prescale != PCA9685_FIXED_PRESCALE_50HZ) {
        return false;
    }

    uint8_t pwm_registers[kPwmRegisterBytes * kPcaChannelCount]{};
    if (!PCA9685_ReadBlockUnlocked(
            LED0_ON_L, pwm_registers, sizeof(pwm_registers))) {
        return false;
    }

    const uint16_t neutral_count =
        PCA9685_PwmUsToCount(PCA9685_PWM_NEUTRAL_US);
    for (uint8_t channel = 0U; channel < kPcaChannelCount; ++channel) {
        const uint16_t offset =
            static_cast<uint16_t>(kPwmRegisterBytes * channel);
        if (pwm_registers[offset] != 0U ||
            pwm_registers[offset + 1U] != 0U ||
            pwm_registers[offset + 2U] !=
                static_cast<uint8_t>(neutral_count & 0xFFU) ||
            pwm_registers[offset + 3U] !=
                static_cast<uint8_t>((neutral_count >> 8U) & 0x0FU)) {
            return false;
        }
    }
    return true;
}

bool PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off)
{
    bool ok;

    if (PCA9685_BeginTransaction() != PCA9685_TXN_OK) {
        return false;
    }
    ok = PCA9685_WritePwmUnlocked(channel, static_cast<uint16_t>(on),
                                  static_cast<uint16_t>(off));
    if (!ok) {
        (void)PCA9685_ForceOutputsOffUnlocked();
    }
    PCA9685_EndTransaction();
    return ok;
}

bool PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    ++pca9685_diagnostics.init_attempts;
    pca_i2c = hi2c;
    if (PCA9685_BeginTransaction() != PCA9685_TXN_OK) {
        PCA9685_RecordFailure(PCA9685_PHASE_BEGIN_TRANSACTION);
        return false;
    }

    bool ok = false;
    PCA9685InitFailurePhase failure_phase = PCA9685_PHASE_NONE;
    do {
        /*
         * ALL_LED_OFF_H writes FULL_OFF into all sixteen LEDn_OFF_H
         * registers before MODE1 is changed. This prevents a separately
         * powered PCA9685 from replaying stale channel values while the
         * STM32 is restarting.
         */
        if (!PCA9685_ForceOutputsOffUnlocked()) {
            failure_phase = PCA9685_PHASE_FORCE_OFF;
            break;
        }

        const uint8_t sleep_mode =
            static_cast<uint8_t>(kMode1Configured | kMode1Sleep);
        if (!PCA9685_WriteUnlocked(PCA9685_MODE1, sleep_mode)) {
            failure_phase = PCA9685_PHASE_MODE1_SLEEP;
            break;
        }
        if (!PCA9685_WriteUnlocked(PCA9685_MODE2, kMode2Configured)) {
            failure_phase = PCA9685_PHASE_MODE2;
            break;
        }
        if (!PCA9685_WriteUnlocked(
                PCA9685_PRESCALE, PCA9685_FIXED_PRESCALE_50HZ)) {
            failure_phase = PCA9685_PHASE_PRESCALE;
            break;
        }

        int32_t neutral[kPcaChannelCount]{};
        for (int32_t &value : neutral) {
            value = PCA9685_PWM_NEUTRAL_US;
        }
        if (!PCA9685_WritePwmBatchUnlocked(
                neutral, kPcaChannelCount)) {
            failure_phase = PCA9685_PHASE_NEUTRAL_BATCH;
            break;
        }
        if (!PCA9685_VerifyConfigurationUnlocked(true)) {
            failure_phase = PCA9685_PHASE_VERIFY_SLEEP;
            break;
        }

        if (!PCA9685_WriteUnlocked(
                PCA9685_MODE1, kMode1Configured)) {
            failure_phase = PCA9685_PHASE_WAKE;
            break;
        }
        HAL_Delay(2U);
        if (!PCA9685_WriteUnlocked(
                PCA9685_MODE1,
                static_cast<uint8_t>(
                    kMode1Configured | kMode1Restart))) {
            failure_phase = PCA9685_PHASE_RESTART;
            break;
        }

        /* A single MODE1 read confirms the device woke; the neutral block
         * was already verified while SLEEP was asserted. */
        uint8_t mode1 = 0U;
        if (!PCA9685_ReadUnlocked(PCA9685_MODE1, &mode1) ||
            (mode1 & kMode1RelevantMask) != kMode1Configured) {
            failure_phase = PCA9685_PHASE_VERIFY_WAKE;
            break;
        }

        /*
         * Writing the sixteen individual channel registers while SLEEP was
         * asserted replaced the temporary ALL_LED full-off state. Therefore
         * the first visible pulse after wake/restart is the verified neutral
         * value; never write ALL_LED_* again here.
         */
        ok = true;
    } while (false);

    if (!ok) {
        PCA9685_RecordFailure(failure_phase);
        (void)PCA9685_ForceOutputsOffUnlocked();
    }

    PCA9685_EndTransaction();
    return ok;
}

bool PCA9685_ForceOutputsOff()
{
    if (PCA9685_BeginTransaction() != PCA9685_TXN_OK) {
        return false;
    }
    const bool ok = PCA9685_ForceOutputsOffUnlocked();
    PCA9685_EndTransaction();
    return ok;
}

PCA9685HealthStatus PCA9685_CheckHealth(void)
{
    ++pca9685_diagnostics.health_check_count;

    const PCA9685TransactionStatus txn = PCA9685_BeginTransaction();
    if (txn == PCA9685_TXN_BUS_LOCK_TIMEOUT) {
        ++pca9685_diagnostics.health_bus_lock_timeouts;
        return PCA9685_HEALTH_BUS_LOCK_TIMEOUT;
    }
    if (txn == PCA9685_TXN_TCA_SELECT_FAILED) {
        ++pca9685_diagnostics.health_tca_select_failures;
        return PCA9685_HEALTH_TCA_SELECT_FAILED;
    }

    uint8_t mode1 = 0U, mode2 = 0U, prescale = 0U;
    uint8_t valid_mask = 0U;

    if (PCA9685_ReadUnlocked(PCA9685_MODE1, &mode1)) {
        valid_mask |= 0x01U;
    }
    if (PCA9685_ReadUnlocked(PCA9685_MODE2, &mode2)) {
        valid_mask |= 0x02U;
    }
    if (PCA9685_ReadUnlocked(PCA9685_PRESCALE, &prescale)) {
        valid_mask |= 0x04U;
    }

    pca9685_diagnostics.last_mode1 = mode1;
    pca9685_diagnostics.last_mode2 = mode2;
    pca9685_diagnostics.last_prescale = prescale;
    pca9685_diagnostics.last_snapshot_valid_mask = valid_mask;

    PCA9685_EndTransaction();

    if (valid_mask != 0x07U) {
        ++pca9685_diagnostics.health_io_read_failures;
        return PCA9685_HEALTH_IO_READ_FAILED;
    }

    /* Verify configuration bits using masks that exclude dynamic bits
     * (RESTART is self-clearing) and reserved bits. */
    const bool mode1_ok =
        (mode1 & kMode1RelevantMask) == kMode1Configured;
    const bool mode2_ok =
        (mode2 & kMode2RelevantMask) == kMode2Configured;
    const bool prescale_ok =
        (prescale == PCA9685_FIXED_PRESCALE_50HZ);

    if (mode1_ok && mode2_ok && prescale_ok) {
        return PCA9685_HEALTHY;
    }

    /* Power-on-reset signature: PRE_SCALE=0x1E (default),
     * MODE1=(SLEEP|ALLCALL)≈0x11, MODE2≈0x04. */
    if (prescale == 0x1EU &&
        (mode1 & kMode1RelevantMask) == (kMode1Sleep | kMode1AllCall) &&
        (mode2 & kMode2RelevantMask) == (0x04U & kMode2RelevantMask)) {
        ++pca9685_diagnostics.health_reset_detected;
        return PCA9685_HEALTH_RESET_DETECTED;
    }

    ++pca9685_diagnostics.health_config_mismatches;
    return PCA9685_HEALTH_CONFIG_MISMATCH;
}

bool PCA9685_Recover(void)
{
    ++pca9685_diagnostics.recover_attempts;

    if (PCA9685_BeginTransaction() != PCA9685_TXN_OK) {
        ++pca9685_diagnostics.recover_failures;
        return false;
    }

    bool ok = false;
    PCA9685InitFailurePhase failure_phase = PCA9685_PHASE_NONE;
    do {
        /* Enter SLEEP.  This interrupts PWM output — the upper layer must
         * have already DISARMED and discarded old thrust before calling. */
        const uint8_t sleep_mode =
            static_cast<uint8_t>(kMode1Configured | kMode1Sleep);
        if (!PCA9685_WriteUnlocked(PCA9685_MODE1, sleep_mode)) {
            failure_phase = PCA9685_PHASE_RECOVER_SLEEP;
            break;
        }

        /* Clear ALL_LED global full-off.  A prior write failure calls
         * ForceOutputsOffUnlocked which sets ALL_LED_OFF_H.FULL_OFF=1.
         * Global FULL_OFF overrides per-channel registers — it must be
         * explicitly cleared before channel neutrals take effect. */
        if (!PCA9685_WriteUnlocked(PCA9685_ALL_LED_ON_L,  0x00U) ||
            !PCA9685_WriteUnlocked(PCA9685_ALL_LED_ON_H,  0x00U) ||
            !PCA9685_WriteUnlocked(PCA9685_ALL_LED_OFF_L, 0x00U) ||
            !PCA9685_WriteUnlocked(PCA9685_ALL_LED_OFF_H, 0x00U)) {
            failure_phase = PCA9685_PHASE_FORCE_OFF;
            break;
        }

        /* Configure registers while in SLEEP. */
        if (!PCA9685_WriteUnlocked(PCA9685_MODE2, kMode2Configured)) {
            failure_phase = PCA9685_PHASE_MODE2;
            break;
        }
        if (!PCA9685_WriteUnlocked(
                PCA9685_PRESCALE, PCA9685_FIXED_PRESCALE_50HZ)) {
            failure_phase = PCA9685_PHASE_PRESCALE;
            break;
        }

        /* Write neutral to all sixteen channels while in SLEEP.
         * The PWM registers are reloaded now; after wake the first
         * visible pulse is the verified neutral value.  No RESTART
         * write is needed because we are not restoring old waveforms. */
        int32_t neutral[kPcaChannelCount]{};
        for (int32_t &value : neutral) {
            value = PCA9685_PWM_NEUTRAL_US;
        }
        if (!PCA9685_WritePwmBatchUnlocked(
                neutral, kPcaChannelCount)) {
            failure_phase = PCA9685_PHASE_NEUTRAL_BATCH;
            break;
        }

        /* Read-back verify while still in SLEEP. */
        if (!PCA9685_VerifyConfigurationUnlocked(true)) {
            failure_phase = PCA9685_PHASE_VERIFY_SLEEP;
            break;
        }

        /* Wake: clear SLEEP bit, wait for oscillator to stabilise
         * (≥500 µs per datasheet; 2 ms engineering margin). */
        if (!PCA9685_WriteUnlocked(
                PCA9685_MODE1, kMode1Configured)) {
            failure_phase = PCA9685_PHASE_WAKE;
            break;
        }
        HAL_Delay(2U);

        /* Verify MODE1 after wake using mask (RESTART is self-clearing). */
        uint8_t mode1 = 0U;
        if (!PCA9685_ReadUnlocked(PCA9685_MODE1, &mode1) ||
            (mode1 & kMode1RelevantMask) != kMode1Configured) {
            failure_phase = PCA9685_PHASE_VERIFY_WAKE;
            break;
        }

        ok = true;
    } while (false);

    if (!ok) {
        ++pca9685_diagnostics.recover_failures;
        pca9685_diagnostics.last_recover_failure_phase =
            static_cast<uint8_t>(failure_phase);
        (void)PCA9685_ForceOutputsOffUnlocked();
    } else {
        ++pca9685_diagnostics.recover_successes;
    }

    PCA9685_EndTransaction();
    return ok;
}

bool PCA9685_SetAllPWMGuarded(
    const int32_t pwm_us[8],
    PCA9685_OutputGuard output_guard,
    const void *guard_context,
    uint32_t expected_generation,
    bool *superseded)
{
    bool ok = true;
    bool output_superseded = false;

    if (pwm_us == nullptr) {
        return false;
    }
    if (superseded != nullptr) {
        *superseded = false;
    }

    if (PCA9685_BeginTransaction() != PCA9685_TXN_OK) {
        return false;
    }

    if (output_guard != nullptr &&
        !output_guard(guard_context, expected_generation)) {
        output_superseded = true;
    } else if (!PCA9685_WritePwmBatchUnlocked(
                   pwm_us, kMotorChannelCount)) {
        ok = false;
    }

    if (output_guard != nullptr &&
        !output_guard(guard_context, expected_generation)) {
        output_superseded = true;
    }

    if (output_superseded) {
        int32_t neutral[kMotorChannelCount]{};
        for (int32_t &value : neutral) {
            value = PCA9685_PWM_NEUTRAL_US;
        }
        if (!PCA9685_WritePwmBatchUnlocked(
                neutral, kMotorChannelCount)) {
            ok = false;
        }
    }

    if (!ok) {
        ++pca9685_diagnostics.write_failures;
        (void)PCA9685_ForceOutputsOffUnlocked();
    }

    PCA9685_EndTransaction();
    if (superseded != nullptr) {
        *superseded = output_superseded;
    }
    return ok;
}

bool PCA9685_SetAllPWM(const int32_t pwm_us[8])
{
    return PCA9685_SetAllPWMGuarded(
        pwm_us, nullptr, nullptr, 0U, nullptr);
}
