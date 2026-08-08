#include "robot_tasks.h"

#include "cmsis_os.h"
#include "generated_xy_robot_main.hpp"
#include "usart.h"
#include "iwdg.h"
#include "binary_protocol.h"

#include <cstdio>

/* ------------------------------------------------------------------ */
/*  Forward declarations                                                */
/* ------------------------------------------------------------------ */

static void SetControlFailsafeNeutralLocked(uint8_t reason);

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

  /* Heartbeat timeout -> COMM_LOST
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

  /*
   * The PCA9685ThrusterModule ("pca9685") now owns the full actuator
   * startup state machine:
   *   UNINITIALIZED -> INITIALIZING -> NEUTRAL_HOLD -> READY
   *   with RECOVERING and FAULT branches.
   *
   * robot_tasks.cpp only:
   *   1. Calls pca9685.Update() each cycle to drive the state machine.
   *   2. Syncs the actuator mirror fields (actuator_state / output_ready
   *      / fault_reason) into RobotData for the protocol layer.
   *   3. Decides ESTOP latching when pca9685.IsFaulted().
   *   4. Runs the safety gate and writes hardware neutral when not READY.
   *   5. Runs motor_control and SubmitOutputs() when READY.
   */

  pca9685.Init();

  const uint32_t boot_ms = HAL_GetTick();
  pca9685.Start(boot_ms);

  TickType_t last_wake = xTaskGetTickCount();

  /* ================================================================ */
  /*  Main control loop                                                */
  /* ================================================================ */
  for (;;)
  {
    const uint32_t now = HAL_GetTick();

    /* ---- Layer 1: Uptime + safety monitoring (always runs) ---- */
    UpdateUptime(now);
    CheckHeartbeat(now);
    CheckChannelTimeout(now);
    CheckBodyCommandTimeout(now);

    /* ---- Layer 2: Drive actuator state machine ---- */
    pca9685.Update(now);

    /* ---- Layer 3: Sync actuator mirror fields into RobotData ---- */
    taskENTER_CRITICAL();
    robot.actuator_state       = pca9685.State();
    robot.actuator_output_ready = pca9685.IsReady();
    robot.actuator_fault_reason = pca9685.FaultReason();
    taskEXIT_CRITICAL();

    /* ---- Layer 4: ESTOP decision based on actuator module state ---- */
    {
      bool estop;
      taskENTER_CRITICAL();
      estop = robot.estop_locked;
      taskEXIT_CRITICAL();

      if (pca9685.IsFaulted() && !estop)
      {
        taskENTER_CRITICAL();
        robot.estop_locked = true;
        robot.state = RobotState::EMERGENCY_STOP;
        robot.state_changed_ms = now;
        taskEXIT_CRITICAL();
      }
    }

    /* ---- Layer 5: Safety state gate (ESTOP / DISARMED / COMM_LOST) ---- */
    bool estop_locked;
    RobotState state;
    taskENTER_CRITICAL();
    estop_locked = robot.estop_locked;
    state = robot.state;
    taskEXIT_CRITICAL();

    if (estop_locked || state == RobotState::EMERGENCY_STOP ||
        state == RobotState::DISARMED ||
        state == RobotState::COMM_LOST)
    {
      /* Re-check under lock */
      taskENTER_CRITICAL();
      const RobotState live_state = robot.state;
      const bool live_estop = robot.estop_locked;
      if (live_estop || live_state == RobotState::EMERGENCY_STOP ||
          live_state == RobotState::DISARMED ||
          live_state == RobotState::COMM_LOST)
      {
        if (robot.control_enable || robot.float_enabled ||
            robot.angle_enabled || robot.body_control_enabled)
        {
          uint8_t reason = ProtoNeutral_DISARM;
          if (live_estop || live_state == RobotState::EMERGENCY_STOP)
            reason = ProtoNeutral_EMERGENCY_STOP;
          else if (live_state == RobotState::COMM_LOST)
            reason = ProtoNeutral_COMM_LOST;
          SetControlFailsafeNeutralLocked(reason);
        }
      }
      taskEXIT_CRITICAL();

      /* Write neutral to hardware regardless of actuator state. */
      pca9685.EmergencyNeutral(now);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
      continue;
    }

    /* ---- Layer 6: Normal control (only when actuator READY) ---- */
    if (pca9685.IsReady())
    {
      /* ---- General control-command timeout ---- */
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

      if (state == RobotState::ARMED_IDLE ||
          state == RobotState::ARMED_ACTIVE ||
          state == RobotState::MANUAL_TEST)
      {
        if (control_enable && !body_control_enabled &&
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

        motor_control.Update();
      }
      else
      {
        /* Safety net: not in a controllable RobotState */
        taskENTER_CRITICAL();
        force_body_output_neutral(robot);
        taskEXIT_CRITICAL();
      }

      /* ---- Layer 7: PCA9685 hardware output ---- */
      if (!pca9685.SubmitOutputs(robot.pwm, 8, now))
      {
        /*
         * SubmitOutputs failed -> ThrusterOutputManager internally
         * transitioned to RECOVERING.  Clear control sources and
         * DISARM if armed.
         */
        char msg[48];
        std::snprintf(msg, sizeof(msg),
            "[I2C_FAIL] entering RECOVERING");
        Protocol_LogQueuePush(msg);

        taskENTER_CRITICAL();
        SetControlFailsafeNeutralLocked(
            robot.estop_locked ? ProtoNeutral_EMERGENCY_STOP
                               : ProtoNeutral_FAULT);
        if (!robot.estop_locked &&
            (robot.state == RobotState::ARMED_IDLE ||
             robot.state == RobotState::ARMED_ACTIVE ||
             robot.state == RobotState::MANUAL_TEST))
        {
          robot.state = RobotState::DISARMED;
          robot.state_changed_ms = now;
        }
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        continue;
      }
    }
    else
    {
      /*
       * Actuator not READY (NEUTRAL_HOLD / RECOVERING / FAULT /
       * INITIALIZING).  Ensure software neutral and write it to
       * hardware.  The ThrusterOutputManager has already set
       * robot.pwm[] to neutral in its Update().
       */
      taskENTER_CRITICAL();
      if (robot.control_enable || robot.float_enabled ||
          robot.angle_enabled || robot.body_control_enabled)
      {
        uint8_t reason = ProtoNeutral_FAULT;
        SetControlFailsafeNeutralLocked(reason);
      }
      taskEXIT_CRITICAL();

      pca9685.EmergencyNeutral(now);
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

    /* Drain one log entry per loop iteration. */
    Protocol_LogQueueDrainOne();

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
