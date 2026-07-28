#include "PCA9685Driver.hpp"
#include "PCA9685.h"
#include "robot_data.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>

namespace {

constexpr uint8_t kMode1 = 0x00U;
constexpr uint8_t kMode2 = 0x01U;
constexpr uint8_t kLed0OnLow = 0x06U;
constexpr uint8_t kLed0OffHigh = 0x09U;
constexpr uint8_t kAllLedOnLow = 0xFAU;
constexpr uint8_t kAllLedOffHigh = 0xFDU;
constexpr uint8_t kPrescale = 0xFEU;
constexpr uint8_t kRestart = 0x80U;
constexpr uint8_t kAutoIncrement = 0x20U;
constexpr uint8_t kSleep = 0x10U;
constexpr uint8_t kFullOff = 0x10U;
constexpr uint8_t kExpectedTcaChannel = 4U;
constexpr uint16_t kExpectedAddress = 0x80U;
constexpr uint32_t kExpectedTimeoutMs = 20U;
constexpr uint32_t kExpectedBusLockTimeoutMs = 150U;
constexpr uint8_t kChannelCount = 16U;
constexpr uint8_t kMotorChannelCount = 8U;
constexpr uint8_t kRegistersPerChannel = 4U;

struct FakePca9685 {
    std::array<uint8_t, 256> registers{};
    uint8_t read_pointer = 0U;
    bool read_pointer_valid = false;
    std::size_t hal_call_count = 0U;
    std::size_t fail_once_call = 0U;
    bool always_fail = false;
    bool failure_triggered = false;
    bool first_data_write_seen = false;
    uint8_t first_write_register = 0U;
    uint8_t first_write_value = 0U;
    uint32_t delay_ms = 0U;
    uint32_t lock_count = 0U;
    uint32_t unlock_count = 0U;
    uint32_t select_count = 0U;
    uint8_t selected_channel = 0xFFU;
    bool bus_locked = false;
    bool fail_bus_lock_once = false;
    bool fail_tca_select_once = false;
    bool drop_write_once = false;
    uint8_t drop_write_register = 0U;
    bool partial_write_failure_armed = false;
    uint16_t partial_write_failure_length = 0U;
    uint16_t partial_write_bytes = 0U;
};

FakePca9685 fake{};
uint32_t fake_tick = 0U;

bool fail_this_hal_call()
{
    ++fake.hal_call_count;
    if (fake.always_fail ||
        (fake.fail_once_call != 0U &&
         fake.hal_call_count == fake.fail_once_call)) {
        fake.failure_triggered = true;
        return true;
    }
    return false;
}

void write_register(uint8_t reg, uint8_t value)
{
    if (fake.drop_write_once && reg == fake.drop_write_register) {
        fake.drop_write_once = false;
        return;
    }

    /*
     * ALL_LED registers are write-only aliases. The real PCA9685 copies an
     * ALL_LED write into the corresponding register of every channel.
     */
    if (reg >= kAllLedOnLow && reg <= kAllLedOffHigh) {
        const uint8_t channel_register_offset =
            static_cast<uint8_t>(reg - kAllLedOnLow);
        for (uint8_t channel = 0U; channel < kChannelCount; ++channel) {
            const uint8_t channel_register = static_cast<uint8_t>(
                kLed0OnLow + (kRegistersPerChannel * channel) +
                channel_register_offset);
            fake.registers[channel_register] = value;
        }
        return;
    }

    if (reg == kMode1 && (value & kRestart) != 0U) {
        value = static_cast<uint8_t>(value & ~kRestart);
    }
    if (reg == kPrescale &&
        (fake.registers[kMode1] & kSleep) == 0U) {
        return;
    }
    fake.registers[reg] = value;
}

void reset_fake(uint8_t dirty_value = 0xA5U)
{
    fake = FakePca9685{};
    fake.registers.fill(dirty_value);
    fake_tick = 0U;
}

uint8_t channel_register(uint8_t channel, uint8_t offset)
{
    return static_cast<uint8_t>(
        kLed0OnLow + (kRegistersPerChannel * channel) + offset);
}

bool channel_is_full_off(uint8_t channel)
{
    return (fake.registers[channel_register(channel, 3U)] & kFullOff) != 0U;
}

bool channel_is_neutral(uint8_t channel)
{
    const uint16_t neutral_count = PCA9685_PwmUsToCount(1500);
    return fake.registers[channel_register(channel, 0U)] == 0U &&
           fake.registers[channel_register(channel, 1U)] == 0U &&
           fake.registers[channel_register(channel, 2U)] ==
               static_cast<uint8_t>(neutral_count & 0xFFU) &&
           fake.registers[channel_register(channel, 3U)] ==
               static_cast<uint8_t>((neutral_count >> 8U) & 0x0FU);
}

void expect_all_channels_full_off()
{
    for (uint8_t channel = 0U; channel < kChannelCount; ++channel) {
        assert(channel_is_full_off(channel));
    }
}

void expect_channels_neutral(uint8_t channel_count)
{
    for (uint8_t channel = 0U; channel < channel_count; ++channel) {
        assert(channel_is_neutral(channel));
    }
}

void expect_transaction_balanced()
{
    assert(!fake.bus_locked);
    assert(fake.lock_count == fake.unlock_count);
}

void test_pwm_conversion_and_clamp()
{
    assert(PCA9685_PwmUsToCount(1000) == 211U);
    assert(PCA9685_PwmUsToCount(1500) == 316U);
    assert(PCA9685_PwmUsToCount(2000) == 421U);
    assert(PCA9685_PwmUsToCount(-5000) ==
           PCA9685_PwmUsToCount(1000));
    assert(PCA9685_PwmUsToCount(5000) ==
           PCA9685_PwmUsToCount(2000));
}

std::size_t test_safe_init_from_dirty_registers()
{
    reset_fake();

    assert(PCA9685_Init(&hi2c2));
    assert(fake.first_data_write_seen);
    assert(fake.first_write_register == kAllLedOffHigh);
    assert(fake.first_write_value == kFullOff);
    assert(fake.selected_channel == kExpectedTcaChannel);
    assert(fake.select_count == 1U);
    assert(fake.registers[kMode1] == kAutoIncrement);
    assert(fake.registers[kMode2] == 0x04U);
    assert(fake.registers[kPrescale] == 121U);
    assert(fake.delay_ms >= 2U);
    expect_channels_neutral(kChannelCount);
    assert(fake.hal_call_count < 55U);
    expect_transaction_balanced();
    return fake.hal_call_count;
}

void test_silent_register_write_corruption_is_detected()
{
    reset_fake();
    fake.drop_write_once = true;
    fake.drop_write_register = kMode2;

    assert(!PCA9685_Init(&hi2c2));
    assert(!fake.drop_write_once);
    PCA9685Diagnostics diagnostics{};
    PCA9685_GetDiagnostics(&diagnostics);
    assert(diagnostics.last_failure_phase == PCA9685_PHASE_VERIFY_SLEEP);
    assert(diagnostics.phase_failures[PCA9685_PHASE_VERIFY_SLEEP] > 0U);
    expect_all_channels_full_off();
    expect_transaction_balanced();
}

void test_partial_init_batch_failure_fails_closed()
{
    reset_fake();
    fake.partial_write_failure_armed = true;
    fake.partial_write_failure_length =
        static_cast<uint16_t>(
            1U + (kRegistersPerChannel * kChannelCount));
    fake.partial_write_bytes = 13U;

    assert(!PCA9685_Init(&hi2c2));
    assert(fake.failure_triggered);
    assert(!fake.partial_write_failure_armed);
    expect_all_channels_full_off();
    expect_transaction_balanced();
}

void test_each_init_hal_failure_fails_closed(
    std::size_t successful_init_hal_calls)
{
    assert(successful_init_hal_calls > 0U);
    for (std::size_t fail_call = 1U;
         fail_call <= successful_init_hal_calls; ++fail_call) {
        reset_fake();
        fake.fail_once_call = fail_call;

        assert(!PCA9685_Init(&hi2c2));
        assert(fake.failure_triggered);
        expect_all_channels_full_off();
        expect_transaction_balanced();
    }
}

void test_mux_and_lock_failures_return_false()
{
    reset_fake();
    fake.fail_bus_lock_once = true;
    assert(!PCA9685_Init(&hi2c2));
    assert(fake.lock_count == 1U);
    assert(fake.unlock_count == 0U);
    assert(!fake.bus_locked);

    reset_fake();
    fake.fail_tca_select_once = true;
    assert(!PCA9685_Init(&hi2c2));
    assert(fake.select_count == 1U);
    expect_transaction_balanced();
}

void test_permanent_i2c_failure_is_not_reported_ready()
{
    reset_fake();
    fake.always_fail = true;
    assert(!PCA9685_Init(&hi2c2));
    assert(fake.failure_triggered);
    expect_transaction_balanced();

    reset_fake();
    fake.always_fail = true;
    robot = RobotData{};
    PCA9685Driver driver;
    assert(!driver.Init());
    assert(!driver.is_ready());
    assert(!driver.last_write_ok());
    assert(driver.recovery_required());
    expect_transaction_balanced();
}

void test_runtime_write_failure_and_recovery()
{
    reset_fake();
    robot = RobotData{};
    PCA9685Driver driver;
    assert(driver.Init());
    assert(driver.is_ready());
    assert(!driver.recovery_required());
    expect_channels_neutral(kChannelCount);

    for (uint8_t channel = 0U; channel < kMotorChannelCount; ++channel) {
        robot.pwm[channel] =
            static_cast<int32_t>(1550U + (5U * channel));
    }
    ++robot.pwm_output_generation;
    robot.actuator_output_ready = true;

    fake.partial_write_failure_armed = true;
    fake.partial_write_failure_length =
        static_cast<uint16_t>(
            1U + (kRegistersPerChannel * kMotorChannelCount));
    fake.partial_write_bytes = 7U;
    assert(!driver.Update());
    assert(fake.failure_triggered);
    assert(!fake.partial_write_failure_armed);
    assert(!driver.is_ready());
    assert(!driver.last_write_ok());
    assert(driver.recovery_required());
    assert(!robot.actuator_output_ready);
    expect_all_channels_full_off();
    expect_transaction_balanced();

    fake.fail_once_call = 0U;
    fake.failure_triggered = false;
    assert(driver.RecoverToNeutral());
    assert(driver.is_ready());
    assert(driver.last_write_ok());
    assert(!driver.recovery_required());
    // ControlTask owns the transition back to protocol-visible readiness.
    assert(!robot.actuator_output_ready);
    expect_channels_neutral(kChannelCount);
    expect_transaction_balanced();
}

struct GuardState {
    uint32_t calls = 0U;
};

bool supersede_after_first_check(
    const void *context, uint32_t expected_generation)
{
    const auto *const_state = static_cast<const GuardState *>(context);
    auto *state = const_cast<GuardState *>(const_state);
    assert(expected_generation == 77U);
    ++state->calls;
    return state->calls == 1U;
}

void test_guard_supersede_rewrites_neutral()
{
    reset_fake();
    assert(PCA9685_Init(&hi2c2));

    const int32_t commanded[kMotorChannelCount] = {
        1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670,
    };
    GuardState guard_state{};
    bool superseded = false;
    assert(PCA9685_SetAllPWMGuarded(
        commanded, supersede_after_first_check, &guard_state, 77U,
        &superseded));
    assert(superseded);
    assert(guard_state.calls == 2U);
    expect_channels_neutral(kMotorChannelCount);
    expect_transaction_balanced();
}

} // namespace

extern "C" {

I2C_HandleTypeDef hi2c2{};

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

void HAL_Delay(uint32_t delay_ms)
{
    fake.delay_ms += delay_ms;
    fake_tick += delay_ms;
}

bool I2C2_BusLock(uint32_t timeout_ms)
{
    assert(timeout_ms == kExpectedBusLockTimeoutMs);
    ++fake.lock_count;
    if (fake.fail_bus_lock_once) {
        fake.fail_bus_lock_once = false;
        return false;
    }
    assert(!fake.bus_locked);
    fake.bus_locked = true;
    return true;
}

void I2C2_BusUnlock(void)
{
    assert(fake.bus_locked);
    fake.bus_locked = false;
    ++fake.unlock_count;
}

HAL_StatusTypeDef TCA9548A_SelectChannel(
    I2C_HandleTypeDef *hi2c, uint8_t channel)
{
    assert(hi2c == &hi2c2);
    ++fake.select_count;
    if (fake.fail_tca_select_once) {
        fake.fail_tca_select_once = false;
        return HAL_ERROR;
    }
    fake.selected_channel = channel;
    return HAL_OK;
}

HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c)
{
    assert(hi2c == &hi2c2);
    fake.selected_channel = 0xFFU;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(
    I2C_HandleTypeDef *hi2c, uint16_t device_address, uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    assert(hi2c == &hi2c2);
    assert(device_address == kExpectedAddress);
    assert(data != nullptr);
    assert(length > 0U);
    assert(timeout_ms == kExpectedTimeoutMs);
    assert(fake.bus_locked);

    if (fail_this_hal_call()) {
        return HAL_ERROR;
    }

    const uint8_t start_register = data[0];
    if (length == 1U) {
        fake.read_pointer = start_register;
        fake.read_pointer_valid = true;
        return HAL_OK;
    }

    if (!fake.first_data_write_seen) {
        fake.first_data_write_seen = true;
        fake.first_write_register = start_register;
        fake.first_write_value = data[1];
    }

    uint16_t data_bytes_to_write = static_cast<uint16_t>(length - 1U);
    const bool partial_failure =
        fake.partial_write_failure_armed &&
        length == fake.partial_write_failure_length;
    if (partial_failure &&
        fake.partial_write_bytes < data_bytes_to_write) {
        data_bytes_to_write = fake.partial_write_bytes;
    }

    for (uint16_t index = 0U; index < data_bytes_to_write; ++index) {
        const bool auto_increment =
            (fake.registers[kMode1] & kAutoIncrement) != 0U;
        const uint8_t reg = static_cast<uint8_t>(
            start_register +
            static_cast<uint8_t>(auto_increment ? index : 0U));
        write_register(reg, data[index + 1U]);
    }
    if (partial_failure) {
        fake.partial_write_failure_armed = false;
        fake.failure_triggered = true;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(
    I2C_HandleTypeDef *hi2c, uint16_t device_address, uint8_t *data,
    uint16_t length, uint32_t timeout_ms)
{
    assert(hi2c == &hi2c2);
    assert(device_address == kExpectedAddress);
    assert(data != nullptr);
    assert(length > 0U);
    assert(timeout_ms == kExpectedTimeoutMs);
    assert(fake.bus_locked);
    assert(fake.read_pointer_valid);

    if (fail_this_hal_call()) {
        return HAL_ERROR;
    }

    const bool auto_increment =
        (fake.registers[kMode1] & kAutoIncrement) != 0U;
    for (uint16_t index = 0U; index < length; ++index) {
        const uint8_t reg = static_cast<uint8_t>(
            fake.read_pointer +
            static_cast<uint8_t>(auto_increment ? index : 0U));
        data[index] = fake.registers[reg];
    }
    if (auto_increment) {
        fake.read_pointer =
            static_cast<uint8_t>(fake.read_pointer + length);
    }
    return HAL_OK;
}

} // extern "C"

int main()
{
    test_pwm_conversion_and_clamp();
    const std::size_t successful_init_hal_calls =
        test_safe_init_from_dirty_registers();
    test_each_init_hal_failure_fails_closed(successful_init_hal_calls);
    test_silent_register_write_corruption_is_detected();
    test_partial_init_batch_failure_fails_closed();
    test_mux_and_lock_failures_return_false();
    test_permanent_i2c_failure_is_not_reported_ready();
    test_runtime_write_failure_and_recovery();
    test_guard_supersede_rewrites_neutral();

    std::cout << "PCA9685 host tests passed; injected all "
              << successful_init_hal_calls
              << " successful-init HAL I2C call positions\n";
    return 0;
}
