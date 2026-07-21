/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bringup_sensors.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART6_BRINGUP_ONLY 0
#define I2C_BRINGUP_ONLY 0
#define SAFE_PWM_BRINGUP_ONLY 0
#define SENSOR_PWM_BRINGUP_ONLY 0

/* PCA9685_ACTUAL_PERIOD_US is from oscilloscope measurement: 19.45 ms.
   PWM_NEUTRAL_US means the target real high pulse is 1500 us.
   If PCA9685 prescale is later tuned to a 20 ms period, change this to 20000. */
#define PCA9685_ACTUAL_PERIOD_US 19450U
#define PWM_NEUTRAL_US 1500U
#define PWM_MAX_OFFSET_US 150
#define PWM_TEST_DURATION_MS 300U
#define PWM_MIN_SAFE_US (PWM_NEUTRAL_US - PWM_MAX_OFFSET_US)
#define PWM_MAX_SAFE_US (PWM_NEUTRAL_US + PWM_MAX_OFFSET_US)

#define TCA9548A_ADDR (0x70U << 1)
#define PCA9685_ADDR (0x40U << 1)
#define PCA9685_MODE1 0x00U
#define PCA9685_PRESCALE 0xFEU
#define PCA9685_LED0_ON_L 0x06U
#define PCA9685_50HZ_PRESCALE 121U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_clock_error_step = 0;
volatile HAL_StatusTypeDef g_clock_hal_status = HAL_OK;
volatile uint32_t g_rcc_cr = 0;
volatile uint32_t g_rcc_cfgr = 0;
volatile uint32_t g_flash_acr = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#if SAFE_PWM_BRINGUP_ONLY || I2C_BRINGUP_ONLY
static void UartPrint(const char *s);
static HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);
#endif
#if SAFE_PWM_BRINGUP_ONLY
static void SafePwmBringupTest(void);
static void SafePwmCommandLoop(void);
static void SafePwmHandleCommand(const char *cmd);
static void SafePwmRunPulseTest(uint8_t channel, uint16_t target_us);
static void SafePwmPrintHelp(void);
static uint8_t SafePwmPollCommandLine(char *line, size_t line_size, uint32_t timeout_ms);
static uint16_t SafePwmClampUs(uint16_t us);
static uint8_t SafePwmParseTestCommand(const char *cmd, uint8_t *channel, uint16_t *target_us);
static HAL_StatusTypeDef PCA9685_ReadReg(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef PCA9685_WriteReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef PCA9685_Init50Hz(void);
static uint16_t PWM_UsToCount(uint16_t us);
static void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);
static void PCA9685_SetPWM_Us(uint8_t channel, uint16_t us);
static void PCA9685_SetAllNeutral(void);
#endif
#if I2C_BRINGUP_ONLY
static void I2C_BringupTest(void);
static void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *bus_name);
#endif
#if USART6_BRINGUP_ONLY
static void USART6_BringupEchoLoop(void);
static void USART6_SendString(const char *text);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
#if SAFE_PWM_BRINGUP_ONLY
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  SafePwmBringupTest();
#elif I2C_BRINGUP_ONLY
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  I2C_BringupTest();
  while (1)
  {
    UartPrint("I2C test alive\r\n");
    HAL_Delay(1000);
  }
#elif USART6_BRINGUP_ONLY
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  USART6_BringupEchoLoop();
#else
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  /* IWDG must start after XYRobotSetup() completes in SensorTaskFunc.
     Do not call MX_IWDG_Init() during early peripheral initialization. */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
#endif

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  g_clock_hal_status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if (g_clock_hal_status != HAL_OK)
  {
    g_clock_error_step = 1;
    g_rcc_cr = RCC->CR;
    g_rcc_cfgr = RCC->CFGR;
    g_flash_acr = FLASH->ACR;
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  g_clock_hal_status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if (g_clock_hal_status != HAL_OK)
  {
    g_clock_error_step = 2;
    g_rcc_cr = RCC->CR;
    g_rcc_cfgr = RCC->CFGR;
    g_flash_acr = FLASH->ACR;
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if SAFE_PWM_BRINGUP_ONLY || I2C_BRINGUP_ONLY
static void UartPrint(const char *s)
{
  HAL_UART_Transmit(&huart6, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

static HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel)
{
  uint8_t data;

  if (channel > 7U)
  {
    return HAL_ERROR;
  }

  data = (uint8_t)(1U << channel);
  return HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)TCA9548A_ADDR, &data, 1, 10);
}
#endif

#if SAFE_PWM_BRINGUP_ONLY
static HAL_StatusTypeDef PCA9685_ReadReg(uint8_t reg, uint8_t *value)
{
  HAL_StatusTypeDef status;

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  status = TCA9548A_SelectChannel(4);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)PCA9685_ADDR, &reg, 1, 10);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_I2C_Master_Receive(&hi2c2, (uint16_t)PCA9685_ADDR, value, 1, 10);
}

static HAL_StatusTypeDef PCA9685_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  HAL_StatusTypeDef status;

  status = TCA9548A_SelectChannel(4);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)PCA9685_ADDR, tx, sizeof(tx), 10);
}

static HAL_StatusTypeDef PCA9685_Init50Hz(void)
{
  uint8_t oldmode = 0U;
  uint8_t sleep_mode;
  HAL_StatusTypeDef status;

  status = PCA9685_WriteReg(PCA9685_MODE1, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_ReadReg(PCA9685_MODE1, &oldmode);
  if (status != HAL_OK)
  {
    return status;
  }

  sleep_mode = (uint8_t)((oldmode & 0x7FU) | 0x10U);

  status = PCA9685_WriteReg(PCA9685_MODE1, sleep_mode);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_WriteReg(PCA9685_PRESCALE, PCA9685_50HZ_PRESCALE);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_WriteReg(PCA9685_MODE1, oldmode);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(2);
  return PCA9685_WriteReg(PCA9685_MODE1, (uint8_t)(oldmode | 0xA1U));
}

static uint16_t PWM_UsToCount(uint16_t us)
{
  uint32_t count;

  count = ((((uint32_t)us * 4096U) + (PCA9685_ACTUAL_PERIOD_US / 2U)) /
           PCA9685_ACTUAL_PERIOD_US);
  if (count > 4095U)
  {
    count = 4095U;
  }

  return (uint16_t)count;
}

static void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
  uint8_t tx[5];

  if (channel >= 16U)
  {
    return;
  }

  if (TCA9548A_SelectChannel(4) != HAL_OK)
  {
    return;
  }

  tx[0] = (uint8_t)(PCA9685_LED0_ON_L + (4U * channel));
  tx[1] = (uint8_t)(on & 0xFFU);
  tx[2] = (uint8_t)((on >> 8) & 0x0FU);
  tx[3] = (uint8_t)(off & 0xFFU);
  tx[4] = (uint8_t)((off >> 8) & 0x0FU);
  (void)HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)PCA9685_ADDR, tx, sizeof(tx), 10);
}

static void PCA9685_SetPWM_Us(uint8_t channel, uint16_t us)
{
  if (channel >= 16U)
  {
    return;
  }

  us = SafePwmClampUs(us);
  PCA9685_SetPWM(channel, 0U, PWM_UsToCount(us));
}

static void PCA9685_SetAllNeutral(void)
{
  for (uint8_t channel = 0U; channel < 16U; channel++)
  {
    PCA9685_SetPWM_Us(channel, PWM_NEUTRAL_US);
  }

  UartPrint("All channels set to neutral\r\n");
}

static void SafePwmPrintHelp(void)
{
  UartPrint("Commands: HELP, NEU, T0+40, T0-40, T0+80, T0-80 ... T7+150, T7-150\r\n");
  UartPrint("Format: T<channel><+|-><offset>, channel 0..7, offset 0..150us\r\n");
  UartPrint("Each test runs one channel for 300ms, then all channels return neutral\r\n");
  SensorBringup_PrintHelp();
}

static uint8_t SafePwmPollCommandLine(char *line, size_t line_size, uint32_t timeout_ms)
{
  static char rx_line[32];
  static size_t rx_len = 0U;
  uint32_t start = HAL_GetTick();
  uint8_t ch;

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (HAL_UART_Receive(&huart6, &ch, 1, 10) != HAL_OK)
    {
      continue;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      if (rx_len == 0U)
      {
        continue;
      }

      if (line_size > 0U)
      {
        size_t copy_len = rx_len;
        if (copy_len >= line_size)
        {
          copy_len = line_size - 1U;
        }
        memcpy(line, rx_line, copy_len);
        line[copy_len] = '\0';
      }
      rx_len = 0U;
      return 1U;
    }

    if (rx_len < (sizeof(rx_line) - 1U))
    {
      rx_line[rx_len] = (char)ch;
      rx_len++;
    }
  }

  return 0U;
}

static uint16_t SafePwmClampUs(uint16_t us)
{
  if (us < PWM_MIN_SAFE_US)
  {
    return PWM_MIN_SAFE_US;
  }
  if (us > PWM_MAX_SAFE_US)
  {
    return PWM_MAX_SAFE_US;
  }

  return us;
}

static uint8_t SafePwmParseTestCommand(const char *cmd, uint8_t *channel, uint16_t *target_us)
{
  char *end = NULL;
  long offset;

  if ((cmd == NULL) || (channel == NULL) || (target_us == NULL))
  {
    return 0U;
  }
  if ((cmd[0] != 'T') || (cmd[1] < '0') || (cmd[1] > '7') ||
      ((cmd[2] != '+') && (cmd[2] != '-')))
  {
    return 0U;
  }
  if ((cmd[3] < '0') || (cmd[3] > '9'))
  {
    return 0U;
  }

  offset = strtol(&cmd[3], &end, 10);
  if ((end == &cmd[3]) || (*end != '\0') || (offset < 0))
  {
    return 0U;
  }
  if (offset > PWM_MAX_OFFSET_US)
  {
    offset = PWM_MAX_OFFSET_US;
  }

  *channel = (uint8_t)(cmd[1] - '0');
  if (cmd[2] == '+')
  {
    *target_us = (uint16_t)(PWM_NEUTRAL_US + (uint16_t)offset);
  }
  else
  {
    *target_us = (uint16_t)(PWM_NEUTRAL_US - (uint16_t)offset);
  }

  *target_us = SafePwmClampUs(*target_us);
  return 1U;
}

static void SafePwmRunPulseTest(uint8_t channel, uint16_t target_us)
{
  char msg[64];

  if (channel > 7U)
  {
    PCA9685_SetAllNeutral();
    return;
  }

  target_us = SafePwmClampUs(target_us);
  PCA9685_SetAllNeutral();

  snprintf(msg, sizeof(msg), "CH%u target pulse %uus for %ums\r\n",
           channel, (unsigned int)target_us, PWM_TEST_DURATION_MS);
  UartPrint(msg);

  PCA9685_SetPWM_Us(channel, target_us);
  HAL_Delay(PWM_TEST_DURATION_MS);
  PCA9685_SetPWM_Us(channel, PWM_NEUTRAL_US);
  PCA9685_SetAllNeutral();

  snprintf(msg, sizeof(msg), "CH%u back to neutral %uus\r\n",
           channel, PWM_NEUTRAL_US);
  UartPrint(msg);
}

static void SafePwmHandleCommand(const char *cmd)
{
  uint8_t channel;
  uint16_t target_us;

  if (strcmp(cmd, "HELP") == 0)
  {
    SafePwmPrintHelp();
    return;
  }

  if (strcmp(cmd, "NEU") == 0)
  {
    PCA9685_SetAllNeutral();
    return;
  }

  if (strcmp(cmd, "IMU") == 0)
  {
    PCA9685_SetAllNeutral();
    SensorBringup_PrintImu();
    return;
  }

  if (strcmp(cmd, "DEPTH") == 0)
  {
    PCA9685_SetAllNeutral();
    SensorBringup_PrintDepth();
    return;
  }

  if (strcmp(cmd, "SENS") == 0)
  {
    PCA9685_SetAllNeutral();
    SensorBringup_PrintAll();
    return;
  }

  if (SafePwmParseTestCommand(cmd, &channel, &target_us) != 0U)
  {
    SafePwmRunPulseTest(channel, target_us);
    return;
  }

  PCA9685_SetAllNeutral();
  UartPrint("Unknown command\r\n");
}

static void SafePwmCommandLoop(void)
{
  char cmd[32];

  while (1)
  {
    if (SafePwmPollCommandLine(cmd, sizeof(cmd), 100) != 0U)
    {
      SafePwmHandleCommand(cmd);
    }
  }
}

static void SafePwmBringupTest(void)
{
  UartPrint("\r\nSAFE PWM BRINGUP START\r\n");
  UartPrint("PCA9685 on TCA9548A CH4\r\n");
  UartPrint("Measured PCA9685 period = 19450us\r\n");
  UartPrint("PWM neutral target actual pulse = 1500us\r\n");

  if ((TCA9548A_SelectChannel(4) != HAL_OK) ||
      (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)PCA9685_ADDR, 2, 10) != HAL_OK))
  {
    UartPrint("PCA9685 not found on TCA CH4\r\n");
    while (1)
    {
      UartPrint("PCA9685 missing\r\n");
      HAL_Delay(1000);
    }
  }

  UartPrint("PCA9685 found on TCA CH4\r\n");

  if (PCA9685_Init50Hz() != HAL_OK)
  {
    UartPrint("PCA9685 init failed\r\n");
    while (1)
    {
      UartPrint("PCA9685 missing\r\n");
      HAL_Delay(1000);
    }
  }

  PCA9685_SetAllNeutral();
  SensorBringup_Init();
  SafePwmPrintHelp();
  SafePwmCommandLoop();
}
#endif

#if I2C_BRINGUP_ONLY
static void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *bus_name)
{
  char msg[128];

  snprintf(msg, sizeof(msg), "Scanning %s\r\n", bus_name);
  UartPrint(msg);

  for (uint8_t addr = 0x03U; addr <= 0x77U; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK)
    {
      snprintf(msg, sizeof(msg), "%s found 0x%02X\r\n", bus_name, addr);
      UartPrint(msg);
    }
  }

  snprintf(msg, sizeof(msg), "%s scan done\r\n", bus_name);
  UartPrint(msg);
}

static void I2C_BringupTest(void)
{
  char msg[128];

  UartPrint("\r\nI2C BRINGUP START\r\n");

  I2C_ScanBus(&hi2c2, "I2C2");

  if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(0x70U << 1), 2, 10) == HAL_OK)
  {
    UartPrint("TCA9548A found at 0x70 on I2C2\r\n");

    for (uint8_t channel = 0U; channel < 8U; channel++)
    {
      if (TCA9548A_SelectChannel(channel) == HAL_OK)
      {
        snprintf(msg, sizeof(msg), "TCA9548A channel %u selected\r\n", channel);
        UartPrint(msg);
        I2C_ScanBus(&hi2c2, "I2C2-CHN");
      }
      else
      {
        snprintf(msg, sizeof(msg), "TCA9548A channel %u select failed\r\n", channel);
        UartPrint(msg);
      }
    }
  }
  else
  {
    UartPrint("TCA9548A not found on I2C2\r\n");
  }

  I2C_ScanBus(&hi2c3, "I2C3");

  UartPrint("I2C BRINGUP DONE\r\n");
}
#endif

#if USART6_BRINGUP_ONLY
static void USART6_SendString(const char *text)
{
  HAL_UART_Transmit(&huart6, (const uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void USART6_BringupEchoLoop(void)
{
  uint8_t ch;
  char line[128];
  size_t line_len = 0U;
  uint8_t last_was_cr = 0U;

  USART6_SendString("\r\nSTM32 USART6 BRINGUP READY\r\n");

  while (1)
  {
    if (HAL_UART_Receive(&huart6, &ch, 1, 100) != HAL_OK)
    {
      continue;
    }

    HAL_UART_Transmit(&huart6, &ch, 1, 100);

    if ((ch == '\r') || (ch == '\n'))
    {
      if ((ch == '\n') && (last_was_cr != 0U))
      {
        last_was_cr = 0U;
        continue;
      }

      line[line_len] = '\0';
      USART6_SendString("RX:");
      USART6_SendString(line);
      USART6_SendString("\r\n");

      if (strcmp(line, "PING") == 0)
      {
        USART6_SendString("PONG\r\n");
      }

      line_len = 0U;
      last_was_cr = (ch == '\r') ? 1U : 0U;
      continue;
    }

    last_was_cr = 0U;
    if (line_len < (sizeof(line) - 1U))
    {
      line[line_len] = (char)ch;
      line_len++;
    }
  }
}
#endif

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
