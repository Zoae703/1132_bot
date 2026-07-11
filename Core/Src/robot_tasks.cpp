#include "robot_tasks.h"

#include "cmsis_os.h"
#include "generated_xy_robot_main.hpp"
#include "usart.h"
#include "iwdg.h"
#include "binary_protocol.h"

static volatile bool pca9685_ready = false;

/* ------------------------------------------------------------------ */
/*  Unified failsafe neutral                                           */
/* ------------------------------------------------------------------ */

static void SetControlFailsafeNeutral(uint8_t reason)
{
  taskENTER_CRITICAL();
  robot.control_enable = false;
  robot.float_enabled = false;
  robot.angle_enabled = false;
  robot.manual_pwm_enabled = false;
  robot.motion_state = 0;
  robot.active_test_channel = 0xFF;
  robot.last_neutral_reason = reason;
  for (int i = 0; i < 8; i++)
  {
    robot.pwm[i] = ROBOT_PWM_NEUTRAL_US;
    robot.manual_pwm[i] = ROBOT_PWM_NEUTRAL_US;
  }
  taskEXIT_CRITICAL();
}

/* ------------------------------------------------------------------ */
/*  Heartbeat / link monitoring                                        */
/* ------------------------------------------------------------------ */

static void CheckHeartbeat(uint32_t now)
{
  uint32_t last_hb;
  uint32_t timeout;
  RobotState state;

  taskENTER_CRITICAL();
  last_hb = robot.last_heartbeat_ms;
  timeout = robot.heartbeat_timeout_ms;
  state   = robot.state;
  taskEXIT_CRITICAL();

  /* Heartbeat timeout → COMM_LOST
   * Only trigger if we've ever received a heartbeat and are in an armed state */
  if (last_hb != 0U &&
      (state == RobotState::ARMED_IDLE ||
       state == RobotState::ARMED_ACTIVE ||
       state == RobotState::MANUAL_TEST))
  {
    if ((now - last_hb) > timeout)
    {
      SetControlFailsafeNeutral(ProtoNeutral_COMM_LOST);
      taskENTER_CRITICAL();
      if (robot.state != RobotState::COMM_LOST)
      {
        robot.state = RobotState::COMM_LOST;
        robot.state_changed_ms = now;
        robot.heartbeat_missed++;
      }
      taskEXIT_CRITICAL();
    }
  }
}

/* ------------------------------------------------------------------ */
/*  Channel test timeout                                               */
/* ------------------------------------------------------------------ */

static void CheckChannelTimeout(uint32_t now)
{
  uint8_t  active_ch;
  uint32_t deadline;
  bool     manual_enabled;

  taskENTER_CRITICAL();
  active_ch      = robot.active_test_channel;
  deadline       = robot.channel_test_deadline;
  manual_enabled = robot.manual_pwm_enabled;
  taskEXIT_CRITICAL();

  if (manual_enabled && active_ch != 0xFF &&
      static_cast<int32_t>(now - deadline) >= 0)
  {
    taskENTER_CRITICAL();
    robot.pwm[active_ch] = ROBOT_PWM_NEUTRAL_US;
    robot.manual_pwm[active_ch] = ROBOT_PWM_NEUTRAL_US;
    robot.active_test_channel = 0xFF;
    robot.manual_pwm_enabled = false;
    robot.last_neutral_reason = ProtoNeutral_PWM_COMMAND_TIMEOUT;
    taskEXIT_CRITICAL();
  }
}

/* ------------------------------------------------------------------ */
/*  Uptime tracking                                                    */
/* ------------------------------------------------------------------ */

static uint32_t last_uptime_ms = 0;

static void UpdateUptime(uint32_t now)
{
  if ((now - last_uptime_ms) >= 1000U)
  {
    last_uptime_ms = now;
    robot.uptime_s++;
    robot.last_uptime_tick = now;
  }
}

/* ================================================================== */
/*  TASK FUNCTIONS                                                     */
/* ================================================================== */

extern "C" void SensorTaskFunc(void *argument)
{
  (void)argument;

  imu.Init();
  ist8310.Init();
  ahrs.Init();
  depth_sensor.Init();

  /* --- IWDG starts after sensors are initialised --- */
  MX_IWDG_Init();

  TickType_t last_wake = xTaskGetTickCount();
  for (;;)
  {
    imu.Update();
    ist8310.Update();
    ahrs.Update();
    depth_sensor.Update();

    /* Refresh IWDG every sensor cycle */
    HAL_IWDG_Refresh(&hiwdg);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
}

extern "C" void ControlTaskFunc(void *argument)
{
  (void)argument;

  motor_control.Init();
  pca9685.Init();
  pca9685_ready = pca9685.is_ready();
  if (pca9685_ready)
  {
    SetControlFailsafeNeutral(ProtoNeutral_NONE);
    pca9685.Update();
  }

  bool pca_error_logged = false;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    const uint32_t now = HAL_GetTick();

    /* --- Uptime --- */
    UpdateUptime(now);

    /* --- Heartbeat / link monitoring --- */
    CheckHeartbeat(now);

    /* --- Channel test timeout --- */
    CheckChannelTimeout(now);

    /* === ESTOP lock: force neutral regardless of anything else === */
    bool estop_locked;
    RobotState state;
    taskENTER_CRITICAL();
    estop_locked = robot.estop_locked;
    state = robot.state;
    taskEXIT_CRITICAL();

    if (estop_locked && state == RobotState::EMERGENCY_STOP)
    {
      SetControlFailsafeNeutral(ProtoNeutral_EMERGENCY_STOP);
      if (pca9685_ready)
        pca9685.Update();
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
      continue;
    }

    /* --- Command timeout (legacy failsafe, for text protocol) --- */
    bool control_enable;
    uint32_t last_cmd_tick;
    taskENTER_CRITICAL();
    control_enable = robot.control_enable;
    last_cmd_tick  = robot.last_cmd_tick;
    taskEXIT_CRITICAL();

    if (state == RobotState::DISARMED ||
        state == RobotState::COMM_LOST ||
        state == RobotState::FAULT)
    {
      /* Force neutral in unsafe states */
      if (control_enable)
      {
        uint8_t reason = ProtoNeutral_FAULT;
        if (state == RobotState::DISARMED) reason = ProtoNeutral_DISARM;
        if (state == RobotState::COMM_LOST) reason = ProtoNeutral_COMM_LOST;
        SetControlFailsafeNeutral(reason);
      }
    }
    else if (control_enable &&
             ((now - last_cmd_tick) > ROBOT_COMMAND_TIMEOUT_MS))
    {
      SetControlFailsafeNeutral(ProtoNeutral_COMMAND);
    }

    /* --- Motor control --- */
    motor_control.Update();

    /* --- PCA9685 output --- */
    if (pca9685_ready)
    {
      pca9685.Update();
      if (!pca9685.last_write_ok())
      {
        /* PCA9685 I2C write failure → ESTOP */
        SetControlFailsafeNeutral(ProtoNeutral_FAULT);
        taskENTER_CRITICAL();
        robot.estop_locked = true;
        robot.state = RobotState::EMERGENCY_STOP;
        robot.state_changed_ms = now;
        taskEXIT_CRITICAL();
        pca_error_logged = true;
      }
      else
      {
        pca_error_logged = false;
      }
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
  }
}

extern "C" void CommTaskFunc(void *argument)
{
  (void)argument;

  BinaryProtocol *bp = bp_get_instance();
  if (bp == nullptr || !comm.Init())
  {
    Error_Handler();
  }

  uint32_t last_telemetry_ms = HAL_GetTick();
  uint8_t rx_bytes[COMM_UART_RX_CHUNK_SIZE];
  uint8_t tx_frame[PROTO_BUF_SIZE];

  for (;;)
  {
    if (Comm_TakeRxResyncRequest())
    {
      bp_reset_rx(bp);
      while (Comm_ReadRx(rx_bytes, sizeof(rx_bytes), 0U) > 0U)
      {
        /* Discard bytes spanning a detected overflow/UART error. */
      }
    }

    size_t rx_length = Comm_ReadRx(
        rx_bytes, sizeof(rx_bytes), pdMS_TO_TICKS(20));
    if (Comm_TakeRxResyncRequest())
    {
      bp_reset_rx(bp);
      rx_length = 0U;
      while (Comm_ReadRx(rx_bytes, sizeof(rx_bytes), 0U) > 0U)
      {
        /* Finish discarding the damaged stream segment. */
      }
    }
    else if (rx_length > 0U)
    {
      bp_feed_bytes(bp, rx_bytes, static_cast<uint16_t>(rx_length));
    }

    Comm_EnsureReceiveArmed();

    const uint32_t now = HAL_GetTick();
    if ((now - last_telemetry_ms) >= 200U)
    {
      last_telemetry_ms = now;
      (void)bp_queue_status_report(bp, BP_TX_PRIORITY_NORMAL);
    }

    uint16_t tx_length = 0U;
    while (bp_get_tx_frame(
        bp, tx_frame, sizeof(tx_frame), &tx_length))
    {
      const HAL_StatusTypeDef status = HAL_UART_Transmit(
          &huart6, tx_frame, tx_length, 100U);
      bp_note_tx_result(bp, status == HAL_OK);
      if (status != HAL_OK) break;
    }
  }
}

/* ------------------------------------------------------------------ */
/*  USART6 ISR path: copy bytes only; parsing/dispatch stays in task.   */
/* ------------------------------------------------------------------ */

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                            uint16_t Size)
{
  if (huart != nullptr && huart->Instance == USART6)
  {
    BaseType_t higher_priority_task_woken = pdFALSE;
    (void)Comm_OnRxEventFromISR(Size, &higher_priority_task_woken);
    (void)Comm_StartReceiveToIdle();
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != nullptr && huart->Instance == USART6)
  {
    Comm_OnUartErrorFromISR(huart->ErrorCode);
    (void)Comm_StartReceiveToIdle();
  }
}
