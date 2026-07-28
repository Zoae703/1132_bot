#pragma once
#include "main.h"
#include <cstdint>

#define PCA9685_ADDR     0x80
#define PCA9685_MODE1    0x00
#define PCA9685_MODE2    0x01
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06

bool PCA9685_Init(I2C_HandleTypeDef* hi2c);

enum PCA9685InitFailurePhase : uint8_t {
    PCA9685_PHASE_NONE = 0U,
    PCA9685_PHASE_BEGIN_TRANSACTION,
    PCA9685_PHASE_FORCE_OFF,
    PCA9685_PHASE_MODE1_SLEEP,
    PCA9685_PHASE_MODE2,
    PCA9685_PHASE_PRESCALE,
    PCA9685_PHASE_NEUTRAL_BATCH,
    PCA9685_PHASE_VERIFY_SLEEP,
    PCA9685_PHASE_WAKE,
    PCA9685_PHASE_RESTART,
    PCA9685_PHASE_VERIFY_WAKE,
    PCA9685_PHASE_COUNT,
};

struct PCA9685Diagnostics {
    uint32_t init_attempts;
    uint32_t init_failures;
    uint32_t phase_failures[PCA9685_PHASE_COUNT];
    uint8_t last_failure_phase;
};

void PCA9685_GetDiagnostics(PCA9685Diagnostics *diagnostics);
uint16_t PCA9685_PwmUsToCount(int32_t pwm_us);
bool PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off);
bool PCA9685_SetAllPWM(const int32_t pwm_us[8]);
bool PCA9685_ForceOutputsOff();

using PCA9685_OutputGuard = bool (*)(const void *context,
                                     uint32_t expected_generation);

bool PCA9685_SetAllPWMGuarded(
    const int32_t pwm_us[8],
    PCA9685_OutputGuard output_guard,
    const void *guard_context,
    uint32_t expected_generation,
    bool *superseded);
