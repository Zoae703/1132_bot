/**
 * @file    actuator_startup_test.cpp
 * @brief   Host tests for the ThrusterOutputManager state machine
 *          (20 test cases per the refactoring specification).
 *
 * Uses a MockPCA9685Driver subclass that overrides virtual methods so the
 * state machine can be tested in isolation without hardware dependencies.
 */

#include "ThrusterOutputManager.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <iostream>

/* ------------------------------------------------------------------ */
/*  Constants (mirrored from ThrusterOutputManager — single source)     */
/* ------------------------------------------------------------------ */

static constexpr uint32_t ESC_NEUTRAL_HOLD_MS    = 3000U;
static constexpr uint8_t  MAX_RECOVER_STREAK     = 3U;
static constexpr uint32_t RECOVERY_RETRY_MS      = 100U;
static constexpr uint32_t FAULT_RECOVERY_PERIOD_MS = 1000U;
static constexpr uint32_t HEALTH_CHECK_PERIOD_MS = 1000U;

namespace {

/* ------------------------------------------------------------------ */
/*  Mock PCA9685Driver — controllable test double                       */
/* ------------------------------------------------------------------ */

struct MockDriver : PCA9685Driver {
    MockDriver() : PCA9685Driver(nullptr) {}

    /* Per-call configuration */
    bool init_ok      = false;
    bool recover_ok   = false;
    bool write_ok     = true;
    bool neutral_ok   = true;
    PCA9685HealthStatus health_status = PCA9685_HEALTHY;

    /* Call counters for verification */
    int init_count    = 0;
    int recover_count = 0;
    int health_count  = 0;
    int write_count   = 0;
    int neutral_count = 0;
    int force_off_count = 0;

    /* Internal state */
    bool init_done    = true;
    bool write_good   = true;

    bool Init() override {
        ++init_count;
        set_state(init_ok);
        return init_ok;
    }

    bool Recover() override {
        ++recover_count;
        set_state(recover_ok);
        return recover_ok;
    }

    PCA9685HealthStatus CheckHealth() override {
        ++health_count;
        return health_status;
    }

    bool SetOutputs(const int32_t *, size_t) override {
        ++write_count;
        if (!write_ok) set_state(false);
        return write_ok;
    }

    bool SetNeutralOutputs() override {
        ++neutral_count;
        if (!neutral_ok) set_state(false);
        return neutral_ok;
    }

    bool ForceOutputsOff() override {
        ++force_off_count;
        return true;
    }

    bool IsInitialized() const override { return init_done; }
    bool LastWriteSucceeded() const override { return write_good; }

private:
    void set_state(bool ok) {
        init_done = ok;
        write_good = ok;
    }
};

/* ------------------------------------------------------------------ */
/*  Stub: I2C2Bus recovery flag (consumed by ThrusterOutputManager)     */
/* ------------------------------------------------------------------ */

static bool g_recovering = false;

} // namespace

extern "C" {

void I2C2Bus_SetRecovering(bool r) { g_recovering = r; }
bool I2C2Bus_IsRecovering(void)    { return g_recovering; }

} // extern "C"

/* ------------------------------------------------------------------ */
/*  Dummy PCA9685 C API stubs                                           */
/*                                                                      */
/*  PCA9685Driver.hpp has inline base-class methods that reference      */
/*  these symbols.  The linker needs them even though MockDriver        */
/*  overrides every virtual — the base-class bodies are still emitted.  */
/*                                                                      */
/*  NOTE: PCA9685.h does NOT use extern "C", so these stubs must have   */
/*  C++ linkage to match.  They must also be at global (not anonymous    */
/*  namespace) scope.                                                   */
/* ------------------------------------------------------------------ */

bool PCA9685_Init(I2C_HandleTypeDef *) { return false; }
bool PCA9685_Recover(void) { return false; }
PCA9685HealthStatus PCA9685_CheckHealth(void) { return PCA9685_HEALTHY; }
bool PCA9685_SetAllPWM(const int32_t *) { return false; }
bool PCA9685_SetAllPWMGuarded(const int32_t *, PCA9685_OutputGuard,
                               const void *, uint32_t, bool *) { return false; }
bool PCA9685_ForceOutputsOff() { return false; }
void PCA9685_GetDiagnostics(PCA9685Diagnostics *) {}
uint16_t PCA9685_PwmUsToCount(int32_t) { return 316U; }
bool PCA9685_SetPWM(uint8_t, uint32_t, uint32_t) { return false; }

/* Protocol_LogQueuePush — declared in real binary_protocol.h */
extern "C" void Protocol_LogQueuePush(const char *) { /* no-op in tests */ }

namespace {

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */

static void ResetRobot()
{
    robot = RobotData{};
}

static bool AllPwmNeutral()
{
    for (int i = 0; i < 8; i++) {
        if (robot.pwm[i] != ROBOT_PWM_NEUTRAL_US) return false;
    }
    return true;
}

static bool AllManualPwmNeutral()
{
    for (int i = 0; i < 8; i++) {
        if (robot.manual_pwm[i] != ROBOT_PWM_NEUTRAL_US) return false;
    }
    return true;
}

static void InjectNonNeutralPwm()
{
    for (int i = 0; i < 8; i++) {
        robot.pwm[i] = 1800;
    }
}

/* Advance to READY state */
static void AdvanceToReady(MockDriver &driver,
                           ThrusterOutputManager &mgr,
                           uint32_t &t)
{
    driver.init_ok    = true;
    driver.recover_ok = true;
    driver.write_ok   = true;
    driver.neutral_ok = true;
    driver.health_status = PCA9685_HEALTHY;

    mgr.Start(t);
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);

    t += ESC_NEUTRAL_HOLD_MS + 100;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::READY);
}

/* ================================================================== */
/*  Test 1 — Start enters INITIALIZING                                  */
/* ================================================================== */

void test_01_start_enters_initializing()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);

    assert(mgr.State() == ActuatorStartupState::UNINITIALIZED);
    mgr.Start(0);
    assert(mgr.State() == ActuatorStartupState::INITIALIZING);
    assert(!mgr.IsReady());
}

/* ================================================================== */
/*  Test 2 — Init success enters NEUTRAL_HOLD                           */
/* ================================================================== */

void test_02_init_success_enters_neutral_hold()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = true;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    mgr.Update(0);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.IsReady());
    assert(!mgr.IsFaulted());
    assert(AllPwmNeutral());
    assert(AllManualPwmNeutral());
}

/* ================================================================== */
/*  Test 3 — NOT ready before 3000ms                                    */
/* ================================================================== */

void test_03_not_ready_before_3000ms()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = true;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    mgr.Update(0);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);

    mgr.Update(ESC_NEUTRAL_HOLD_MS - 1);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.IsReady());
}

/* ================================================================== */
/*  Test 4 — 3 seconds + HEALTHY health check → READY                  */
/* ================================================================== */

void test_04_neutral_hold_3s_enters_ready()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = true;
    driver.neutral_ok = true;
    driver.health_status = PCA9685_HEALTHY;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    mgr.Update(0);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);

    uint32_t t = ESC_NEUTRAL_HOLD_MS + 100;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::READY);
    assert(mgr.IsReady());
    assert(robot.actuator_output_ready);
}

/* ================================================================== */
/*  Test 5 — READY allows normal output via SubmitOutputs               */
/* ================================================================== */

void test_05_ready_allows_normal_output()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    assert(mgr.SubmitOutputs(pwm, 8, t + 1));
    assert(driver.write_count > 0);
}

/* ================================================================== */
/*  Test 6 — NEUTRAL_HOLD overwrites non-neutral software targets       */
/* ================================================================== */

void test_06_neutral_hold_overwrites_non_neutral_software()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = true;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    mgr.Update(0);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);

    InjectNonNeutralPwm();
    assert(robot.pwm[0] == 1800);

    mgr.Update(100);
    assert(AllPwmNeutral());
    assert(AllManualPwmNeutral());
}

/* ================================================================== */
/*  Test 7 — Single output write failure enters RECOVERING              */
/* ================================================================== */

void test_07_single_output_failure_enters_recovering()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    assert(!mgr.SubmitOutputs(pwm, 8, t + 1));
    assert(mgr.State() == ActuatorStartupState::RECOVERING);
    assert(!mgr.IsReady());
    assert(mgr.IsRecovering());
    assert(!robot.actuator_output_ready);
}

/* ================================================================== */
/*  Test 8 — Output failure does NOT count toward Recover streak        */
/* ================================================================== */

void test_08_output_failure_not_in_recover_streak()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);

    /* First Recover attempt fails — streak == 1 */
    driver.recover_ok = false;
    t += RECOVERY_RETRY_MS;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);
}

/* ================================================================== */
/*  Test 9 — Recover success restarts NEUTRAL_HOLD                      */
/* ================================================================== */

void test_09_recover_success_restarts_neutral_hold()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);

    driver.recover_ok = true;
    driver.neutral_ok = true;
    t += RECOVERY_RETRY_MS;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.IsReady());
    assert(AllPwmNeutral());
}

/* ================================================================== */
/*  Test 10 — Recover success does NOT restore old PWM values           */
/* ================================================================== */

void test_10_recover_does_not_restore_old_pwm()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    const int32_t old_pwm[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
    mgr.SubmitOutputs(old_pwm, 8, t + 10);

    driver.write_ok = false;
    mgr.SubmitOutputs(old_pwm, 8, t + 20);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);

    driver.recover_ok = true;
    driver.neutral_ok = true;
    t += RECOVERY_RETRY_MS;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(AllPwmNeutral());
}

/* ================================================================== */
/*  Test 11 — Recover success does NOT auto-ARM                         */
/* ================================================================== */

void test_11_recover_does_not_auto_arm()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);

    driver.recover_ok = true;
    driver.neutral_ok = true;
    t += RECOVERY_RETRY_MS;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.IsReady());
    assert(!robot.actuator_output_ready);
}

/* ================================================================== */
/*  Test 12 — 3 consecutive Recover failures enter FAULT                */
/* ================================================================== */

void test_12_consecutive_recover_failures_enter_fault()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);

    driver.recover_ok = false;
    for (int i = 0; i < MAX_RECOVER_STREAK; i++) {
        t += RECOVERY_RETRY_MS;
        mgr.Update(t);
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);
    assert(mgr.IsFaulted());
}

/* ================================================================== */
/*  Test 13 — FAULT low-frequency recovery succeeds → NEUTRAL_HOLD     */
/* ================================================================== */

void test_13_fault_low_freq_recovery_succeeds()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    driver.recover_ok = false;
    for (int i = 0; i < MAX_RECOVER_STREAK; i++) {
        t += RECOVERY_RETRY_MS;
        mgr.Update(t);
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);

    driver.recover_ok = true;
    driver.neutral_ok = true;
    t += FAULT_RECOVERY_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.IsFaulted());
    assert(!mgr.IsReady());
}

/* ================================================================== */
/*  Test 14 — FAULT recovery does NOT clear external ESTOP              */
/* ================================================================== */

void test_14_fault_recovery_does_not_clear_estop()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    robot.estop_locked = true;
    robot.state = RobotState::EMERGENCY_STOP;

    driver.write_ok = false;
    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};
    mgr.SubmitOutputs(pwm, 8, t + 1);
    driver.recover_ok = false;
    for (int i = 0; i < MAX_RECOVER_STREAK; i++) {
        t += RECOVERY_RETRY_MS;
        mgr.Update(t);
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);

    driver.recover_ok = true;
    driver.neutral_ok = true;
    t += FAULT_RECOVERY_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);

    /* ThrusterOutputManager does NOT clear robot.estop_locked */
    assert(robot.estop_locked);
    assert(robot.state == RobotState::EMERGENCY_STOP);
}

/* ================================================================== */
/*  Test 15 — Health BUS_LOCK_TIMEOUT does not trigger RECOVERING      */
/* ================================================================== */

void test_15_bus_lock_timeout_deferred()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.health_status = PCA9685_HEALTH_BUS_LOCK_TIMEOUT;
    t += HEALTH_CHECK_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::READY);
}

/* ================================================================== */
/*  Test 16 — Health RESET_DETECTED triggers RECOVERING                 */
/* ================================================================== */

void test_16_reset_detected_triggers_recovering()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.health_status = PCA9685_HEALTH_RESET_DETECTED;
    t += HEALTH_CHECK_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);
}

/* ================================================================== */
/*  Test 17 — Health CONFIG_MISMATCH triggers RECOVERING                */
/* ================================================================== */

void test_17_config_mismatch_triggers_recovering()
{
    ResetRobot();
    MockDriver driver;
    ThrusterOutputManager mgr(driver);
    uint32_t t = 0;
    AdvanceToReady(driver, mgr, t);

    driver.health_status = PCA9685_HEALTH_CONFIG_MISMATCH;
    t += HEALTH_CHECK_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);
}

/* ================================================================== */
/*  Test 18 — SubmitOutputs rejected in NEUTRAL_HOLD, RECOVERING, FAULT */
/* ================================================================== */

void test_18_submit_outputs_rejected_when_not_ready()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = true;
    ThrusterOutputManager mgr(driver);

    const int32_t pwm[8] = {1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670};

    /* NEUTRAL_HOLD → reject */
    mgr.Start(0);
    mgr.Update(0);
    assert(mgr.State() == ActuatorStartupState::NEUTRAL_HOLD);
    assert(!mgr.SubmitOutputs(pwm, 8, 100));

    /* Go READY, then health failure → RECOVERING */
    uint32_t t = ESC_NEUTRAL_HOLD_MS + 100;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::READY);

    driver.health_status = PCA9685_HEALTH_RESET_DETECTED;
    t += HEALTH_CHECK_PERIOD_MS + 1;
    mgr.Update(t);
    assert(mgr.State() == ActuatorStartupState::RECOVERING);
    assert(!mgr.SubmitOutputs(pwm, 8, t + 1));

    /* Recover streak → FAULT */
    driver.recover_ok = false;
    for (int i = 0; i < MAX_RECOVER_STREAK; i++) {
        t += RECOVERY_RETRY_MS;
        mgr.Update(t);
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);
    assert(!mgr.SubmitOutputs(pwm, 8, t + 1));
}

/* ================================================================== */
/*  Test 19 — Init 3 failures → FAULT                                   */
/* ================================================================== */

void test_19_init_3_failures_enters_fault()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = false;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    uint32_t t = 0;
    for (int i = 0; i < 3; i++) {
        mgr.Update(t);
        t += 100;  /* INIT_RETRY_MS */
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);
    assert(mgr.IsFaulted());
    assert(!robot.actuator_output_ready);
}

/* ================================================================== */
/*  Test 20 — FAULT state forces software neutral every cycle           */
/* ================================================================== */

void test_20_fault_forces_software_neutral()
{
    ResetRobot();
    MockDriver driver;
    driver.init_ok = false;
    ThrusterOutputManager mgr(driver);

    mgr.Start(0);
    uint32_t t = 0;
    for (int i = 0; i < 3; i++) {
        mgr.Update(t);
        t += 100;
    }
    assert(mgr.State() == ActuatorStartupState::FAULT);

    InjectNonNeutralPwm();
    assert(robot.pwm[0] == 1800);

    mgr.Update(t + 100);
    assert(AllPwmNeutral());
    assert(AllManualPwmNeutral());
}

} // namespace

/* ------------------------------------------------------------------ */
/*  Entry point                                                        */
/* ------------------------------------------------------------------ */

int main()
{
    test_01_start_enters_initializing();
    test_02_init_success_enters_neutral_hold();
    test_03_not_ready_before_3000ms();
    test_04_neutral_hold_3s_enters_ready();
    test_05_ready_allows_normal_output();
    test_06_neutral_hold_overwrites_non_neutral_software();
    test_07_single_output_failure_enters_recovering();
    test_08_output_failure_not_in_recover_streak();
    test_09_recover_success_restarts_neutral_hold();
    test_10_recover_does_not_restore_old_pwm();
    test_11_recover_does_not_auto_arm();
    test_12_consecutive_recover_failures_enter_fault();
    test_13_fault_low_freq_recovery_succeeds();
    test_14_fault_recovery_does_not_clear_estop();
    test_15_bus_lock_timeout_deferred();
    test_16_reset_detected_triggers_recovering();
    test_17_config_mismatch_triggers_recovering();
    test_18_submit_outputs_rejected_when_not_ready();
    test_19_init_3_failures_enters_fault();
    test_20_fault_forces_software_neutral();

    std::cout << "Actuator startup state machine tests passed (20/20)\n";
    return 0;
}
