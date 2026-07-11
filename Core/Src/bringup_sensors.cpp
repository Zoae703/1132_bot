#include "bringup_sensors.h"

#include "BMI088Sensor.hpp"
#include "IST8310Sensor.hpp"
#include "MS5837.h"
#include "MahonyAHRS.hpp"
#include "DataBus.hpp"
#include "i2c.h"
#include "usart.h"

#include <cstdio>
#include <cstring>

namespace {

constexpr uint16_t TCA9548A_ADDR = static_cast<uint16_t>(0x70U << 1);
constexpr uint8_t MS5837_TCA_CHANNEL = 0U;
constexpr uint8_t PCA9685_TCA_CHANNEL = 4U;
constexpr float RAD_TO_DEG = 57.2957795f;

BMI088Sensor bmi088(5, "3G", "2000");
IST8310Sensor ist8310(10);
MahonyAHRSModule ahrs(200.0f, 5.0f, 0.0f);
MS5837 ms5837;

bool bmi088_ready = false;
bool ist8310_ready = false;
bool ahrs_ready = false;
bool ms5837_ready = false;

void SensorUartPrint(const char *s)
{
  HAL_UART_Transmit(&huart6, reinterpret_cast<const uint8_t *>(s),
                    static_cast<uint16_t>(std::strlen(s)), 100);
}

HAL_StatusTypeDef SensorTCA9548A_SelectChannel(uint8_t channel)
{
  if (channel > 7U)
  {
    return HAL_ERROR;
  }

  uint8_t data = static_cast<uint8_t>(1U << channel);
  return HAL_I2C_Master_Transmit(&hi2c2, TCA9548A_ADDR, &data, 1, 10);
}

int32_t FloatToFixed(float value, int32_t scale)
{
  const float scaled = value * static_cast<float>(scale);
  if (scaled >= 0.0f)
  {
    return static_cast<int32_t>(scaled + 0.5f);
  }
  return static_cast<int32_t>(scaled - 0.5f);
}

void PrintFixed3(const char *name, float value, const char *unit)
{
  char msg[96];
  int32_t scaled = FloatToFixed(value, 1000);
  const char *sign = "";

  if (scaled < 0)
  {
    sign = "-";
    scaled = -scaled;
  }

  std::snprintf(msg, sizeof(msg), "%s=%s%ld.%03ld%s\r\n",
                name, sign, static_cast<long>(scaled / 1000),
                static_cast<long>(scaled % 1000), unit);
  SensorUartPrint(msg);
}

void FormatFixed3(char *out, size_t out_size, float value)
{
  int32_t scaled = FloatToFixed(value, 1000);
  const char *sign = "";

  if (scaled < 0)
  {
    sign = "-";
    scaled = -scaled;
  }

  std::snprintf(out, out_size, "%s%ld.%03ld", sign,
                static_cast<long>(scaled / 1000),
                static_cast<long>(scaled % 1000));
}

void PrintVec3(const char *name, const char *unit, const float v[3])
{
  char msg[128];
  char x[20];
  char y[20];
  char z[20];

  FormatFixed3(x, sizeof(x), v[0]);
  FormatFixed3(y, sizeof(y), v[1]);
  FormatFixed3(z, sizeof(z), v[2]);

  std::snprintf(msg, sizeof(msg), "%s x=%s y=%s z=%s%s\r\n",
                name, x, y, z, unit);
  SensorUartPrint(msg);
}

void UpdateImuSamples(void)
{
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if (bmi088_ready)
    {
      bmi088.Update();
    }
    if (ist8310_ready)
    {
      ist8310.Update();
    }
    if (ahrs_ready)
    {
      ahrs.Update();
    }
    HAL_Delay(5);
  }
}

} // namespace

extern "C" void SensorBringup_Init(void)
{
  SensorUartPrint("Sensor bringup init\r\n");

  bmi088.Init();
  bmi088_ready = bmi088.is_ready();
  SensorUartPrint(bmi088_ready ? "BMI088 ready\r\n" : "BMI088 not ready\r\n");

  ist8310.Init();
  ist8310_ready = ist8310.is_ready();
  SensorUartPrint(ist8310_ready ? "IST8310 ready\r\n" : "IST8310 not ready\r\n");

  ahrs.Init();
  ahrs_ready = true;

  if (SensorTCA9548A_SelectChannel(MS5837_TCA_CHANNEL) == HAL_OK)
  {
    ms5837_ready = ms5837.init(&hi2c2);
    if (ms5837_ready)
    {
      ms5837.setFluidDensity(997.0f);
    }
  }
  else
  {
    ms5837_ready = false;
  }
  SensorUartPrint(ms5837_ready ? "MS5837 ready on TCA CH0\r\n" :
                                 "MS5837 not ready on TCA CH0\r\n");

  (void)SensorTCA9548A_SelectChannel(PCA9685_TCA_CHANNEL);
}

extern "C" void SensorBringup_PrintHelp(void)
{
  SensorUartPrint("Sensor commands: IMU, DEPTH, SENS\r\n");
}

extern "C" void SensorBringup_PrintImu(void)
{
  float ypr_deg[3];

  SensorUartPrint("IMU snapshot\r\n");
  UpdateImuSamples();

  if (bmi088_ready)
  {
    PrintVec3("accel[m/s2]", "", robot.accel);
    PrintVec3("gyro[rad/s]", "", robot.gyro);
    PrintFixed3("imu_temp[C]", robot.imu_temp, "");
  }
  else
  {
    SensorUartPrint("BMI088 not ready\r\n");
  }

  if (ist8310_ready)
  {
    PrintVec3("mag[uT]", "", robot.mag);
  }
  else
  {
    SensorUartPrint("IST8310 not ready\r\n");
  }

  ypr_deg[0] = robot.yaw * RAD_TO_DEG;
  ypr_deg[1] = robot.pitch * RAD_TO_DEG;
  ypr_deg[2] = robot.roll * RAD_TO_DEG;
  PrintVec3("ypr[deg]", "", ypr_deg);
}

extern "C" void SensorBringup_PrintDepth(void)
{
  SensorUartPrint("DEPTH snapshot\r\n");

  if (!ms5837_ready)
  {
    SensorUartPrint("MS5837 not ready\r\n");
    (void)SensorTCA9548A_SelectChannel(PCA9685_TCA_CHANNEL);
    return;
  }

  if (SensorTCA9548A_SelectChannel(MS5837_TCA_CHANNEL) != HAL_OK)
  {
    SensorUartPrint("MS5837 TCA CH0 select failed\r\n");
    (void)SensorTCA9548A_SelectChannel(PCA9685_TCA_CHANNEL);
    return;
  }

  ms5837.read();
  robot.pressure_mbar = ms5837.pressure(MS5837::mbar);
  robot.water_temp_c = ms5837.temperature();
  robot.depth_m = ms5837.depth();

  PrintFixed3("pressure[mbar]", robot.pressure_mbar, "");
  PrintFixed3("water_temp[C]", robot.water_temp_c, "");
  PrintFixed3("depth[m]", robot.depth_m, "");

  (void)SensorTCA9548A_SelectChannel(PCA9685_TCA_CHANNEL);
}

extern "C" void SensorBringup_PrintAll(void)
{
  SensorUartPrint("SENS snapshot\r\n");
  SensorBringup_PrintImu();
  SensorBringup_PrintDepth();
}
