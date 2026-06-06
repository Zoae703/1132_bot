#include "robot_tasks.h"

#include "cmsis_os.h"
#include "generated_xy_robot_main.hpp"
#include "iwdg.h"

static constexpr uint32_t kCommRxFlag = 1U << 0;
static volatile bool robot_initialized = false;

extern "C" osThreadId_t commTaskHandle;

extern "C" void SensorTaskFunc(void *argument)
{
  (void)argument;

  XYRobotSetup();
  robot_initialized = true;

  for (;;)
  {
    imu.Update();
    depth_sensor.Update();
    ist8310.Update();
    ahrs.Update();
    osDelay(5);
  }
}

extern "C" void ControlTaskFunc(void *argument)
{
  (void)argument;

  while (!robot_initialized)
  {
    osDelay(1);
  }

  for (;;)
  {
    motor_control.Update();
    pca9685.Update();
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(5);
  }
}

extern "C" void CommTaskFunc(void *argument)
{
  (void)argument;

  while (!robot_initialized)
  {
    osDelay(1);
  }

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(kCommRxFlag, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0U)
    {
      comm.Update();
    }
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART6)
  {
    Comm_OnRxEvent(Size);
    if (commTaskHandle != NULL)
    {
      osThreadFlagsSet(commTaskHandle, kCommRxFlag);
    }
  }
}
