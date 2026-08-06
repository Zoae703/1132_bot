#pragma once
#include "main.h"
#include <cstdint>

#define PCA9685_ADDR     0x80
#define PCA9685_MODE1    0x00
#define PCA9685_MODE2    0x01
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06

#define PCA9685_ALL_LED_ON_L  0xFAU
#define PCA9685_ALL_LED_ON_H  0xFBU
#define PCA9685_ALL_LED_OFF_L 0xFCU
#define PCA9685_ALL_LED_OFF_H 0xFDU

bool PCA9685_Init(I2C_HandleTypeDef* hi2c);

typedef enum {
    PCA9685_HEALTHY = 0,
    PCA9685_HEALTH_BUS_LOCK_TIMEOUT,
    PCA9685_HEALTH_TCA_SELECT_FAILED,
    PCA9685_HEALTH_IO_READ_FAILED,
    PCA9685_HEALTH_CONFIG_MISMATCH,
    PCA9685_HEALTH_RESET_DETECTED,
} PCA9685HealthStatus;

typedef enum {
    PCA9685_TXN_OK = 0,
    PCA9685_TXN_BUS_LOCK_TIMEOUT,
    PCA9685_TXN_TCA_SELECT_FAILED,
} PCA9685TransactionStatus;

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
    PCA9685_PHASE_RECOVER_SLEEP,
    PCA9685_PHASE_RECOVER_WAKE,
    PCA9685_PHASE_HEALTH_BUS_LOCK,
    PCA9685_PHASE_HEALTH_IO_READ,
    PCA9685_PHASE_COUNT,
};

struct PCA9685Diagnostics {
    uint32_t init_attempts;
    uint32_t init_failures;
    uint32_t phase_failures[PCA9685_PHASE_COUNT];
    uint8_t last_failure_phase;

    /* --- recovery statistics (independent of init) --- */
    uint32_t recover_attempts;
    uint32_t recover_successes;
    uint32_t recover_failures;
    uint8_t  last_recover_failure_phase;

    /* --- health-check statistics --- */
    uint32_t health_check_count;
    uint32_t health_bus_lock_timeouts;
    uint32_t health_tca_select_failures;
    uint32_t health_io_read_failures;
    uint32_t health_config_mismatches;
    uint32_t health_reset_detected;

    /* --- runtime write statistics --- */
    uint32_t write_failures;

    /* --- fault snapshots --- */
    uint8_t last_mode1;
    uint8_t last_mode2;
    uint8_t last_prescale;
    uint8_t last_snapshot_valid_mask;  /* bit0=MODE1, bit1=MODE2, bit2=PRESCALE */
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

PCA9685HealthStatus PCA9685_CheckHealth(void);
bool PCA9685_Recover(void);
