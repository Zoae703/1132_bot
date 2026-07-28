# 1132_bot System Architecture

## Overview

```
┌─────────────────────────────────────────────────────────┐
│  PC Browser / Tablet + Linux USB gamepad forwarder      │
│  HTTP telemetry/control + /ws/control/gamepad           │
└──────────────────┬──────────────────────────────────────┘
                   │ Ethernet / WiFi
                   ▼
┌─────────────────────────────────────────────────────────┐
│  Orange Pi                                              │
│  ┌───────────────────┐  ┌──────────────────────────────┐│
│  │ opi_console/      │  │ web_backend/ (FastAPI)       ││
│  │ serial_transport  │◄─┤ API + WS + ControlArbiter    ││
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

10. **USB gamepad → existing six-axis mixer**: the Linux forwarder sends raw
    inputs and a mapped BodyCommand to `/ws/control/gamepad`. The backend
    independently validates the map, keeps only the newest frame, acquires the
    exclusive `GAMEPAD` ControlArbiter mode, and periodically emits the existing
    `SET_BODY_COMMAND`. STM32 MotorControl remains the only 6-DOF-to-8-thruster
    mixer.

11. **Gamepad timeout safety**: after 300ms without a new frame the backend
    sends a zero BodyCommand and requires a centered frame before accepting
    movement again. After 1000ms, USB disconnect, disabled client control, or
    WebSocket disconnect, it exits body control and DISARMs.

12. **MS5837 → depth PID → existing mixer**: successful pressure samples carry
    a generation and timestamp. The depth PID advances once per new sample and
    contributes only the existing `heave` correction. A stale/invalid sample or
    expired 500ms depth-command lease disables the loop and returns all outputs
    to neutral.

## Operator Control Arbitration

The Orange Pi has one process-wide motor-control owner:

```text
IDLE -> MOTOR_TEST | WEB_MOTION | GAMEPAD | DEPTH_HOLD
```

Only `IDLE` may transition into a control mode. DISARM, ESTOP, service
disconnect, or a completed mode exit returns ownership to `IDLE`. The gamepad
cannot ARM or RESET_ESTOP and never sends channel PWM. The Web depth-tuning mode
is exclusive and does not automatically take over after a browser/backend
restart.

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
