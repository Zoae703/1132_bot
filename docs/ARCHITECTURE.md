# 1132_bot System Architecture

## Overview

```
┌─────────────────────────────────────────────────────────┐
│  PC Browser / Tablet                                    │
│  http://orange-pi:8000                                  │
└──────────────────┬──────────────────────────────────────┘
                   │ Ethernet / WiFi
                   ▼
┌─────────────────────────────────────────────────────────┐
│  Orange Pi                                              │
│  ┌───────────────────┐  ┌──────────────────────────────┐│
│  │ opi_console/      │  │ web_backend/ (FastAPI)       ││
│  │ serial_transport  │◄─┤ api_routes + ws_manager      ││
│  │ stm32_proxy       │  │                              ││
│  │ simulated_stm32   │  │ web_frontend/ (React+Vite)   ││
│  └────────┬──────────┘  └──────────────────────────────┘│
│           │ UART (115200 8N1)                           │
└───────────┼─────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────┐
│  STM32F407 (RoboMaster C-Board)                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │ FreeRTOS                                          │   │
│  │  ├── sensorTask (Normal,  10ms)                   │   │
│  │  ├── controlTask (AboveNormal, 20ms)              │   │
│  │  └── commTask (High, event-driven)                │   │
│  ├──────────────────────────────────────────────────┤   │
│  │ Modules:                                          │   │
│  │  DataBus → BMI088 → MS5837 → IST8310              │   │
│  │  → MahonyAHRS → PIDController → PCA9685           │   │
│  │  → MotorControl → Communication → Protocol        │   │
│  └──────────────────────────────────────────────────┘   │
│              │                                           │
│              ▼ I2C2 (TCA9548A)                           │
│  ┌──────────────────────────────────────┐               │
│  │ PCA9685 PWM Board (50Hz, 8ch)        │               │
│  │  → 8x Brushless ESCs → 8x Thrusters  │               │
│  └──────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────┘
```

## Data Flow

1. **Sensors → DataBus**: BMI088 (IMU), MS5837 (depth), IST8310 (magnetometer) write raw data to `robot.accel/gyro/mag/depth_m`

2. **AHRS → DataBus**: MahonyAHRS reads raw IMU → writes `robot.yaw/pitch/roll/*_v`

3. **Commands → DataBus**: USART6 ISR copies raw bytes to a FreeRTOS stream buffer; `commTask` performs streaming parse, validation, dispatch, and DataBus state changes

4. **MotorControl → DataBus → PCA9685**: State-gated PID cascade reads sensors + targets → writes `robot.pwm[8]` → PCA9685Driver → I2C → PWM board

5. **Responses → Orange Pi**: command responses and telemetry enter priority-aware protocol queues; `commTask` is the sole normal-runtime USART6 transmitter

6. **Freshness → Web**: heartbeat, status, sensor, command-ACK, and any-frame timestamps are tracked separately. Link liveness never makes cached status or sensor data fresh.

7. **Command confirmation → Web**: the backend stores requested PWM separately
   from the PWM confirmed by a later STM32 status report. An HTTP `200` is only
   returned for PWM after ACK and matching status confirmation.

8. **WebSocket ordering → Browser**: every process has a random `session_id`;
   every telemetry message carries `type`, `sequence`, `timestamp`, and
   `payload`. The browser rejects duplicate and out-of-order messages within a
   session and resets its cursor after a backend restart.

9. **Last client safety → STM32**: when the final WebSocket client disappears,
   the backend serializes `SET_ALL_NEUTRAL`, `EXIT_MANUAL` when applicable, and
   `DISARM`. A new browser connection does not cancel an in-progress safety
   transition.

## Safety State Machine

```
                  power-on
                     │
                     ▼
    ┌─────────┐  ARM  ┌─────────────┐  FLOAT/ANGLE ON  ┌──────────────┐
    │DISARMED │──────►│ ARMED_IDLE  │─────────────────►│ ARMED_ACTIVE │
    └────┬─────┘       └──────┬──────┘                  └──────┬───────┘
         │                    │                                │
         │      ENTER_MANUAL  │  EXIT_MANUAL                  │
         │           ┌────────▼────────┐                       │
         │           │  MANUAL_TEST    │                       │
         │           │ (1 ch at a time)│                       │
         │           └────────┬────────┘                       │
         │                    │                                │
         │  DISARM (any)      │  DISARM (any)                 │
         ◄────────────────────┴────────────────────────────────┘
         │
         │  ESTOP (any state)     ┌──────────────────┐
         ├───────────────────────►│ EMERGENCY_STOP   │
         │                        │ (latched, locked) │
         │  RESET_ESTOP           └────────┬─────────┘
         ◄─────────────────────────────────┘
         │
         │  heartbeat timeout (armed states)
         ├───────────────────────► COMM_LOST → auto-DISARM on next heartbeat
         │
         │  PCA9685 I2C failure
         ├───────────────────────► FAULT (requires power cycle)
```

## Binary Protocol

See [PROTOCOL.md](PROTOCOL.md) for the complete frame format and message type reference.

USART6 carries only framed binary traffic in the normal firmware. Text logging
uses a separate debug interface; standalone bring-up modes are mutually
exclusive with normal binary operation.

## PWM Safety

- **Default output**: 1500us (neutral, motors stopped)
- **Test range**: 1450-1550us (tightened for safety in MANUAL_TEST)
- **Absolute range**: 1300-1700us (hardware limits)
- **Timeout**: Per-command timeout (default 500ms), heartbeat timeout (1000ms)
- **Single channel**: Only one channel can be non-neutral at a time in test mode
- **ESTOP**: Latched, requires explicit RESET_ESTOP command
- **Independent safety**: Both Web backend and STM32 enforce limits independently
