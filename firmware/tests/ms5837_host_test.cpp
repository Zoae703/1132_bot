#include "MS5837.h"
#include "MS5837Sensor.hpp"
#include "robot_data.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

constexpr uint8_t kPromRead = 0xA0U;
constexpr uint8_t kAdcRead = 0x00U;

struct FakeMs5837 {
    std::array<uint16_t, 8> prom{};
    uint8_t command = 0U;
    uint8_t adc_read_count = 0U;
    bool fail_next_hal = false;
    bool fail_all_hal = false;
    uint32_t reset_count = 0U;
    uint32_t lock_count = 0U;
    uint32_t unlock_count = 0U;
};

FakeMs5837 fake{};
uint32_t fake_tick = 0U;

uint8_t crc4(std::array<uint16_t, 8> prom)
{
    uint16_t remainder = 0U;
    prom[0] &= 0x0FFFU;
    prom[7] = 0U;
    for (uint8_t index = 0U; index < 16U; ++index)
    {
        remainder ^= (index % 2U) != 0U
                         ? static_cast<uint16_t>(
                               prom[index >> 1U] & 0x00FFU)
                         : static_cast<uint16_t>(
                               prom[index >> 1U] >> 8U);
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            remainder = (remainder & 0x8000U) != 0U
                            ? static_cast<uint16_t>(
                                  (remainder << 1U) ^ 0x3000U)
                            : static_cast<uint16_t>(
                                  remainder << 1U);
        }
    }
    return static_cast<uint8_t>((remainder >> 12U) & 0x0FU);
}

void reset_fake()
{
    fake = FakeMs5837{};
    fake.prom = {
        0U, 34982U, 36352U, 20328U,
        22354U, 26646U, 26146U, 0U,
    };
    fake.prom[0] = static_cast<uint16_t>(
        static_cast<uint16_t>(crc4(fake.prom)) << 12U);
    fake_tick = 0U;
}

bool fail_hal_call()
{
    if (fake.fail_all_hal) return true;
    if (!fake.fail_next_hal) return false;
    fake.fail_next_hal = false;
    return true;
}

void refresh_prom_crc()
{
    fake.prom[0] &= 0x0FFFU;
    fake.prom[0] = static_cast<uint16_t>(
        fake.prom[0] |
        (static_cast<uint16_t>(crc4(fake.prom)) << 12U));
}

void test_low_level_read_success_and_failure()
{
    reset_fake();
    MS5837 sensor;
    assert(!sensor.read());
    assert(sensor.init(&hi2c2));
    assert(sensor.read());
    const float pressure = sensor.pressure();
    const float temperature = sensor.temperature();
    const float depth = sensor.depth();
    assert(std::isfinite(pressure));
    assert(std::isfinite(temperature));
    assert(std::isfinite(depth));

    fake.fail_next_hal = true;
    assert(!sensor.read());
    assert(sensor.pressure() == pressure);
    assert(sensor.temperature() == temperature);
    assert(sensor.depth() == depth);
    assert(fake.lock_count == fake.unlock_count);
}

void test_wrapper_only_publishes_successful_samples()
{
    reset_fake();
    robot = RobotData{};
    MS5837Sensor sensor(997, 100);
    sensor.Init();
    assert(sensor.is_ready());
    assert(robot.depth_sensor_ready);
    assert(!robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 0U);

    fake_tick = 100U;
    sensor.Update();
    assert(robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 1U);
    assert(robot.depth_sample_ms == 100U);
    const float first_depth = robot.depth_m;

    fake_tick = 200U;
    fake.fail_next_hal = true;
    sensor.Update();
    assert(robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 1U);
    assert(robot.depth_sample_ms == 100U);
    assert(robot.depth_m == first_depth);

    fake_tick = 701U;
    fake.fail_next_hal = true;
    sensor.Update();
    assert(!robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 1U);
    assert(robot.depth_sample_ms == 100U);
    assert(robot.depth_m == first_depth);
    assert(fake.lock_count == fake.unlock_count);
}

void test_wrapper_init_retry_and_runtime_recovery()
{
    reset_fake();
    robot = RobotData{};
    fake.fail_next_hal = true;
    MS5837Sensor sensor(997, 100);
    sensor.Init();
    assert(sensor.is_ready());
    assert(robot.depth_sensor_ready);
    assert(fake.reset_count == 1U);

    reset_fake();
    robot = RobotData{};
    fake.fail_all_hal = true;
    MS5837Sensor recovering_sensor(997, 100);
    recovering_sensor.Init();
    assert(!recovering_sensor.is_ready());
    assert(!robot.depth_sensor_ready);
    assert(!robot.depth_sample_valid);

    fake.fail_all_hal = false;
    fake_tick += 999U;
    recovering_sensor.Update();
    assert(!recovering_sensor.is_ready());
    fake_tick += 1U;
    recovering_sensor.Update();
    assert(recovering_sensor.is_ready());
    assert(robot.depth_sensor_ready);
    assert(!robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 0U);

    fake_tick += 100U;
    recovering_sensor.Update();
    assert(robot.depth_sample_valid);
    assert(robot.depth_sample_generation == 1U);
}

void test_wrapper_rejects_unknown_model()
{
    reset_fake();
    robot = RobotData{};
    fake.prom[1] = 20000U;
    refresh_prom_crc();
    MS5837Sensor sensor(997, 100);
    sensor.Init();
    assert(!sensor.is_ready());
    assert(!robot.depth_sensor_ready);
    assert(!robot.depth_sample_valid);
    assert(fake.reset_count == 3U);
}

} // namespace

I2C_HandleTypeDef hi2c2{};
volatile bool pca9685_recovery_in_progress = false;

extern "C" uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

extern "C" void HAL_Delay(uint32_t delay_ms)
{
    fake_tick += delay_ms;
}

extern "C" HAL_StatusTypeDef HAL_I2C_Master_Transmit(
    I2C_HandleTypeDef *, uint16_t, uint8_t *data,
    uint16_t length, uint32_t)
{
    if (fail_hal_call()) return HAL_ERROR;
    assert(data != nullptr);
    assert(length == 1U);
    fake.command = data[0];
    if (fake.command == 0x1EU) ++fake.reset_count;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_I2C_Master_Receive(
    I2C_HandleTypeDef *, uint16_t, uint8_t *data,
    uint16_t length, uint32_t)
{
    if (fail_hal_call()) return HAL_ERROR;
    assert(data != nullptr);
    if (fake.command >= kPromRead && length == 2U)
    {
        const uint8_t index =
            static_cast<uint8_t>((fake.command - kPromRead) / 2U);
        assert(index < 7U);
        data[0] = static_cast<uint8_t>(fake.prom[index] >> 8U);
        data[1] = static_cast<uint8_t>(fake.prom[index] & 0xFFU);
        return HAL_OK;
    }
    assert(fake.command == kAdcRead);
    assert(length == 3U);
    const uint32_t raw =
        (fake.adc_read_count++ % 2U) == 0U
            ? 4958179U
            : 6815414U;
    data[0] = static_cast<uint8_t>((raw >> 16U) & 0xFFU);
    data[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>(raw & 0xFFU);
    return HAL_OK;
}

extern "C" bool I2C2_BusLock(uint32_t)
{
    ++fake.lock_count;
    return true;
}

extern "C" void I2C2_BusUnlock(void)
{
    ++fake.unlock_count;
}

extern "C" HAL_StatusTypeDef TCA9548A_SelectChannel(
    I2C_HandleTypeDef *, uint8_t channel)
{
    assert(channel == 0U);
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef TCA9548A_DisableAll(
    I2C_HandleTypeDef *)
{
    return HAL_OK;
}

int main()
{
    test_low_level_read_success_and_failure();
    test_wrapper_only_publishes_successful_samples();
    test_wrapper_init_retry_and_runtime_recovery();
    test_wrapper_rejects_unknown_model();
    std::cout << "MS5837 host tests: PASS\n";
    return 0;
}
