#include "robot_tasks.h"

#include "cmsis_os.h"
#include "generated_xy_robot_main.hpp"
#include "usart.h"
#include "iwdg.h"
#include "binary_protocol.h"

static volatile bool pca9685_ready = false;
volatile bool pca9685_recovery_in_progress = true;
static constexpr uint8_t PCA9685_INIT_ATTEMPTS = 3U;
static constexpr uint32_t PCA9685_INIT_RETRY_MS = 100U;
static constexpr uint32_t PCA9685_RECOVERY_RETRY_MS = 750U;

/* ------------------------------------------------------------------ */
/*  Unified failsafe neutral                                           */
/* ------------------------------------------------------------------ */

static void SetControlFailsafeNeutralLocked(uint8_t reason)
{
  robot.control_enable = false;
  robot.float_enabled = false;
  robot.angle_enabled = false;
  robot.body_control_enabled = false;
  robot.manual_pwm_enabled = false;
  robot.active_test_channel = 0xFF;
  robot.depth_command_last_ms = 0U;
  robot.last_neutral_reason = reason;
  reset_depth_control_runtime(robot);
  robot.depth_control_fault_reason = reason;
  force_body_output_neutral(robot);
  for (int i = 0; i < 8; i++)
  {
    robot.manual_pwm[i] = ROBOT_PWM_NEUTRAL_US;
  }
}

static void SetControlFailsafeNeutral(uint8_t reason)
{
  taskENTER_CRITICAL();
  SetControlFailsafeNeutralLocked(reason);
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
      taskENTER_CRITICAL();
      if (robot.last_heartbeat_ms == last_hb &&
          robot.heartbeat_timeout_ms == timeout &&
          robot.state == state &&
          (robot.state == RobotState::ARMED_IDLE ||
           robot.state == RobotState::ARMED_ACTIVE ||
           robot.state == RobotState::MANUAL_TEST) &&
          ((now - robot.last_heartbeat_ms) >
           robot.heartbeat_timeout_ms))
      {
        SetControlFailsafeNeutralLocked(ProtoNeutral_COMM_LOST);
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
    if (robot.manual_pwm_enabled &&
        robot.active_test_channel == active_ch &&
        robot.channel_test_deadline == deadline &&
        static_cast<int32_t>(now - robot.channel_test_deadline) >= 0)
    {
      robot.pwm[active_ch] = ROBOT_PWM_NEUTRAL_US;
      robot.manual_pwm[active_ch] = ROBOT_PWM_NEUTRAL_US;
      robot.active_test_channel = 0xFF;
      robot.manual_pwm_enabled = false;
      robot.last_neutral_reason = ProtoNeutral_PWM_COMMAND_TIMEOUT;
      mark_pwm_output_updated(robot);
    }
    taskEXIT_CRITICAL();
  }
}

/* ------------------------------------------------------------------ */
/*  Unified body command timeout                                      */
/* ------------------------------------------------------------------ */

static void CheckBodyCommandTimeout(uint32_t now)
{
  bool valid;
  uint32_t last_ms;
  uint32_t timeout_ms;
  RobotState state;
  BodyCommandSource source;
  uint16_t sequence;
  BodyCommand command;
  bool body_control_enabled;

  taskENTER_CRITICAL();
  valid = robot.body_command_valid;
  last_ms = robot.body_command_last_ms;
  timeout_ms = robot.body_command_timeout_ms;
  state = robot.state;
  source = robot.body_command_source;
  sequence = robot.body_command_sequence;
  command = robot.body_command;
  body_control_enabled = robot.body_control_enabled;
  taskEXIT_CRITICAL();

  if (valid &&
      !body_command_is_zero(command) &&
      body_control_enabled &&
      state == RobotState::ARMED_ACTIVE &&
      ((now - last_ms) > timeout_ms))
  {
    taskENTER_CRITICAL();
    if (robot.body_command_valid &&
        robot.state == state &&
        robot.body_command_source == source &&
        robot.body_command_sequence == sequence &&
        robot.body_command_last_ms == last_ms &&
        robot.body_command_timeout_ms == timeout_ms &&
        robot.body_control_enabled &&
        !body_command_is_zero(robot.body_command) &&
        ((now - robot.body_command_last_ms) >
         robot.body_command_timeout_ms))
    {
      SetControlFailsafeNeutralLocked(ProtoNeutral_COMMAND);
      robot.state = RobotState::ARMED_IDLE;
      robot.state_changed_ms = now;
    }
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
  pca9685_recovery_in_progress = true;
  taskENTER_CRITICAL();
  robot.actuator_output_ready = false;
  taskEXIT_CRITICAL();

  for (uint8_t attempt = 0U;
       attempt < PCA9685_INIT_ATTEMPTS && !pca9685_ready;
       ++attempt)
  {
    pca9685_ready = pca9685.Init();
    if (!pca9685_ready && (attempt + 1U) < PCA9685_INIT_ATTEMPTS)
    {
      vTaskDelay(pdMS_TO_TICKS(PCA9685_INIT_RETRY_MS));
    }
  }

  taskENTER_CRITICAL();
  if (pca9685_ready)
  {
    /*
     * PCA9685_Init() has already written and read back neutral on all sixteen
     * channels. Keep the software image in sync without adding an un-retried
     * I2C write after the three initialization attempts.
     */
    SetControlFailsafeNeutralLocked(
        robot.estop_locked ? ProtoNeutral_EMERGENCY_STOP
                           : ProtoNeutral_NONE);
    robot.actuator_output_ready = true;
    pca9685_recovery_in_progress = false;
  }
  else
  {
    const bool estop_was_locked =
        robot.estop_locked ||
        robot.state == RobotState::EMERGENCY_STOP;
    SetControlFailsafeNeutralLocked(
        estop_was_locked ? ProtoNeutral_EMERGENCY_STOP
                         : ProtoNeutral_FAULT);
    robot.actuator_output_ready = false;
    robot.estop_locked = estop_was_locked;
    robot.state = estop_was_locked
                      ? RobotState::EMERGENCY_STOP
                      : RobotState::FAULT;
    robot.state_changed_ms = HAL_GetTick();
  }
  taskEXIT_CRITICAL();

  uint32_t last_pca_recovery_ms = HAL_GetTick();
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

    /* --- Unified body command timeout --- */
    CheckBodyCommandTimeout(now);

    /* === ESTOP lock: force neutral regardless of anything else === */
    bool estop_locked;
    RobotState state;
    taskENTER_CRITICAL();
    estop_locked = robot.estop_locked;
    state = robot.state;
    taskEXIT_CRITICAL();

    const bool pca_recovery_needed =
        !pca9685_ready && pca9685.recovery_required();
    if ((estop_locked && state == RobotState::EMERGENCY_STOP) ||
        (state == RobotState::FAULT && pca_recovery_needed))
    {
      SetControlFailsafeNeutral(
          pca9685_ready && estop_locked
              ? ProtoNeutral_EMERGENCY_STOP
                        : ProtoNeutral_FAULT);
      if (pca9685_ready)
      {
        pca9685_ready = pca9685.Update();
        if (!pca9685_ready)
        {
          pca9685_recovery_in_progress = true;
          taskENTER_CRITICAL();
          SetControlFailsafeNeutralLocked(ProtoNeutral_FAULT);
          robot.actuator_output_ready = false;
          robot.estop_locked = true;
          robot.state = RobotState::EMERGENCY_STOP;
          robot.state_changed_ms = now;
          taskEXIT_CRITICAL();
          last_pca_recovery_ms = now;
        }
      }
      else if (pca9685.recovery_required() &&
               (now - last_pca_recovery_ms) >=
                   PCA9685_RECOVERY_RETRY_MS)
      {
        last_pca_recovery_ms = now;
        pca9685_recovery_in_progress = true;
        const bool recovered = pca9685.RecoverToNeutral();
        if (recovered)
        {
          /*
           * RecoverToNeutral() performs the same sixteen-channel neutral
           * write and readback as startup initialization.
           */
          pca9685_recovery_in_progress = false;
          pca9685_ready = true;
          taskENTER_CRITICAL();
          robot.actuator_output_ready = true;
          taskEXIT_CRITICAL();
        }
      }
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
      continue;
    }

    /* --- General control-command timeout --- */
    bool control_enable;
    bool body_control_enabled;
    bool float_enabled;
    uint32_t last_cmd_tick;
    uint32_t depth_command_last_ms;
    taskENTER_CRITICAL();
    control_enable = robot.control_enable;
    body_control_enabled = robot.body_control_enabled;
    float_enabled = robot.float_enabled;
    last_cmd_tick  = robot.last_cmd_tick;
    depth_command_last_ms = robot.depth_command_last_ms;
    taskEXIT_CRITICAL();

    if (state == RobotState::DISARMED ||
        state == RobotState::COMM_LOST ||
        state == RobotState::FAULT)
    {
      /* Re-check under the same lock used to apply the neutral state so a
       * freshly accepted command cannot be cleared by a stale snapshot. */
      taskENTER_CRITICAL();
      const RobotState live_state = robot.state;
      if (robot.control_enable &&
          (live_state == RobotState::DISARMED ||
           live_state == RobotState::COMM_LOST ||
           live_state == RobotState::FAULT))
      {
        uint8_t reason = ProtoNeutral_FAULT;
        if (live_state == RobotState::DISARMED) reason = ProtoNeutral_DISARM;
        if (live_state == RobotState::COMM_LOST) reason = ProtoNeutral_COMM_LOST;
        SetControlFailsafeNeutralLocked(reason);
      }
      taskEXIT_CRITICAL();
    }
    else if (control_enable && !body_control_enabled &&
             ((now - (float_enabled
                          ? depth_command_last_ms
                          : last_cmd_tick)) >
              ROBOT_COMMAND_TIMEOUT_MS))
    {
      taskENTER_CRITICAL();
      if (robot.control_enable &&
          !robot.body_control_enabled &&
          robot.float_enabled == float_enabled &&
          robot.state == state &&
          robot.last_cmd_tick == last_cmd_tick &&
          robot.depth_command_last_ms == depth_command_last_ms &&
          ((now - (robot.float_enabled
                       ? robot.depth_command_last_ms
                       : robot.last_cmd_tick)) >
           ROBOT_COMMAND_TIMEOUT_MS))
      {
        SetControlFailsafeNeutralLocked(ProtoNeutral_COMMAND);
        robot.state = RobotState::ARMED_IDLE;
        robot.state_changed_ms = now;
      }
      taskEXIT_CRITICAL();
    }

    /* --- Motor control --- */
    motor_control.Update();

    /* --- PCA9685 output --- */
    if (pca9685_ready)
    {
      pca9685_ready = pca9685.Update();
      if (!pca9685_ready)
      {
        pca9685_recovery_in_progress = true;
        /*
         * The low-level driver has already attempted ALL_LED full-off.
         * Latch ESTOP and do not permit reset until neutral recovery verifies.
         */
        taskENTER_CRITICAL();
        SetControlFailsafeNeutralLocked(ProtoNeutral_FAULT);
        robot.actuator_output_ready = false;
        robot.estop_locked = true;
        robot.state = RobotState::EMERGENCY_STOP;
        robot.state_changed_ms = now;
        taskEXIT_CRITICAL();
        last_pca_recovery_ms = now;
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
