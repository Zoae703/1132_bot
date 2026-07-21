# CLAUDE.md — 1132_bot 8-Motor Control System

## Project Structure

```
1132_bot/                  # Monorepo root
├── firmware/              # STM32F407 embedded firmware
│   ├── Core/              # HAL user code (main, tasks, interrupts)
│   ├── Drivers/           # HAL drivers (BMI088, MS5837, etc.)
│   ├── Modules/           # xy_robotkit modules (DataBus, PID, AHRS, MotorControl...)
│   ├── protocol/          # Binary protocol (C headers/sources)
│   ├── tests/             # C++ host tests
│   ├── CMakeLists.txt     # Build entry point
│   └── xy_robot.yaml      # Module registry
├── web/                   # Orange Pi web console (上位机)
│   ├── opi_console/       # Serial proxy + web server entry point
│   ├── web_backend/       # FastAPI + WebSocket API
│   ├── web_frontend/      # React/Vite frontend
│   ├── protocol/          # Binary protocol (Python)
│   ├── tests/             # Python tests
│   └── scripts/           # start_web_console.sh
├── scripts/               # CI scripts (run_tests.sh)
├── docs/                  # Architecture, validation, usage docs
└── CLAUDE.md              # This file
```

## Project Context

This is an STM32F407 (RoboMaster C-Board) firmware project for an underwater ROV. Currently:
- BMI088 IMU + MS5837 depth sensor reading works
- Motor PWM (TIM1/TIM8) and CAN bus hardware is configured but unused
- Build: CMake + Ninja + GCC ARM (arm-none-eabi), xy_robotkit modular framework

**Goal**: Port 8-motor control from the FinsROV reference project, adapted so that:
- **Depth**: single MS5837 (not 4 sensors) → DepthPID
- **Attitude (roll/pitch/yaw)**: all from IMU Mahony AHRS (not pressure sensor differentials)
- **Motor output**: PCA9685 I2C PWM board at 50Hz, 8 channels, neutral=1610us
- **Commands**: USART6 protocol (W/A/S/D, ON/OFF, H:, RPY:ON/OFF, etc.)

## Reference Project Paths

The FinsROV reference code is at:
```
C:\Users\40713\Desktop\1132\origin_bot\rov-xy\FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main\Code\Lower_Level_Controller\userCode\
```

Key reference files (read these first before porting):
- `algorithms/Src/PID.cpp` + `algorithms/Inc/PID.h` — PID controller class
- `algorithms/Src/MahonyAHRS.cpp` + `algorithms/Inc/MahonyAHRS.h` — Mahony sensor fusion
- `devices/Src/Propeller.cpp` + `devices/Inc/Propeller.h` — 8-motor control logic (MAIN REFERENCE)
- `devices/Src/Extension.cpp` + `devices/Inc/Extension.h` — PCA9685 + TCA9548A drivers
- `drivers/Src/ist8310driver.cpp` + `drivers/Inc/ist8310driver.h` — IST8310 magnetometer
- `MiddleWares/Src/ist8310driver_middleware.cpp` + `MiddleWares/Inc/ist8310driver_middleware.h` — IST8310 I2C layer
- `devices/Src/IMU.cpp` + `devices/Inc/IMU.h` — get_angle(), attitude_update(), IMU_Attitude_t
- `algorithms/Src/Kalman_Filter.cpp` + `algorithms/Inc/Kalman_Filter.h` — scalar Kalman filter

## Hardware Bus Allocation

```
I2C2 (PF0=SDA, PF1=SCL, 400kHz):
  ├── 0x76 (HAL addr 0xEC) — MS5837 depth sensor (EXISTING)
  └── 0x80                 — PCA9685 PWM board (NEW, no TCA9548A needed)

I2C3 (PC9=SDA, PA8=SCL, 400kHz):
  └── 0x0E (HAL addr 0x1C) — IST8310 magnetometer (NEW)

SPI1 (CS_ACCEL=PA4, CS_GYRO=PB0):
  └── BMI088 IMU (EXISTING)

USART6 (PG14=TX, PG9=RX, 115200baud, DMA RX to idle):
  └── Command protocol (NEW)
```

## Module Architecture

All new modules follow the xy_robotkit pattern:
```
Modules/<ModuleName>/
  <ModuleName>.hpp        # C++ lifecycle wrapper: Init() + Update()
  include/<driver>.h      # Low-level driver header
  src/<driver>.cpp        # Low-level driver implementation
  xy_module.yaml          # Module metadata
```

The build system auto-discovers `Modules/*/src/*.cpp` and `Modules/*/include/`.

### Central Data Exchange: `Modules/DataBus/`

ALL modules communicate through a single global `RobotData` struct. No module calls another module directly — they only read/write the DataBus.

**File: `Modules/DataBus/include/robot_data.hpp`**

```cpp
#pragma once
#include <cstdint>

struct RobotData {
    // === IMU raw data (written by BMI088Sensor, IST8310Sensor) ===
    float accel[3] = {0};     // m/s²
    float gyro[3] = {0};      // rad/s  (already offset-corrected by BMI088 driver)
    float mag[3] = {0};       // µT
    float imu_temp = 0;

    // === AHRS output (written by MahonyAHRS) ===
    float quat[4] = {1,0,0,0};  // w,x,y,z
    float yaw = 0;               // rad
    float pitch = 0;             // rad
    float roll = 0;              // rad
    float yaw_v = 0;             // rad/s (gyro z — angular velocity)
    float pitch_v = 0;           // rad/s (gyro y)
    float roll_v = 0;            // rad/s (gyro x)

    // === Depth sensor (written by MS5837Sensor) ===
    float depth_m = 0;
    float pressure_mbar = 0;
    float water_temp_c = 0;

    // === Targets (written by Communication module) ===
    float target_depth_cm = 30;     // depth target in cm
    float target_yaw = 0;           // yaw target in rad
    float target_roll = 0;          // roll target in rad (always 0 = level)
    float target_pitch = 0;         // pitch target in rad (always 0 = level)

    // === Motor output (written by MotorControl, read by PCA9685Driver) ===
    int32_t pwm[8] = {1610,1610,1610,1610,1610,1610,1610,1610}; // 1000-2000µs

    // === Control flags (written by Communication) ===
    bool float_enabled = false;     // PID float control ON/OFF
    bool angle_enabled = false;     // Yaw angle hold ON/OFF
    uint8_t motion_state = 0;       // 0=STOP, 1=FLOAT, 2=FRONT, 3=BACK, 4=LEFT, 5=RIGHT, 6=CLOCKWISE, 7=ANTICLOCKWISE

    // === System ===
    uint32_t loop_count = 0;
};

extern RobotData robot;
```

**File: `Modules/DataBus/src/robot_data.cpp`**
```cpp
#include "robot_data.hpp"
RobotData robot;
```

**File: `Modules/DataBus/DataBus.hpp`**
```cpp
#pragma once
#include "include/robot_data.hpp"
// DataBus has no Init/Update — it's a passive data store.
// Include this header from any module that needs to read/write robot state.
```

**File: `Modules/DataBus/xy_module.yaml`**
```yaml
name: DataBus
include: DataBus.hpp
description: Central data exchange for all robot modules. No hardware dependency.
```

---

## Implementation Order (7 modules, build sequentially)

### MODULE 1: PIDController

**Goal**: Pure computation module — PID algorithm. No hardware dependency.

Port from reference: `algorithms/PID.cpp` + `algorithms/PID.h`

**Adaptations needed**:
- Replace `#include "Usermain.h"` with `<cstring>` and `<cmath>`
- Replace the `INRANGE` macro with `std::clamp` (C++17)
- Remove the `memcpy`-based copy constructor in `PID_Regulator_t` (use default)
- Keep the same `PID_Regulator_t` struct and `PID` class

**File: `Modules/PIDController/include/PID.h`**
```cpp
#pragma once
#include <cstring>
#include <algorithm>

struct PID_Regulator_t {
    float ref = 0;
    float fdb = 0;
    float err[4] = {0};
    float errSum = 0;
    float kp = 0, ki = 0, kd = 0;
    float componentKp = 0, componentKi = 0, componentKd = 0;
    float componentKpMax = 0, componentKiMax = 0, componentKdMax = 0;
    float output = 0;
    float outputMax = 0;

    PID_Regulator_t() = default;
    PID_Regulator_t(float kp_, float ki_, float kd_, float pM, float iM, float dM, float oM)
        : kp(kp_), ki(ki_), kd(kd_)
        , componentKpMax(pM), componentKiMax(iM), componentKdMax(dM)
        , outputMax(oM) {}
};

class PID {
public:
    PID_Regulator_t PIDInfo{};
    void Reset();
    void Reset(PID_Regulator_t* pidRegulator);
    float PIDCalc(float target, float feedback);
    float PIDCalc(float target, float feedback, float max);
};
```

**File: `Modules/PIDController/src/PID.cpp`** — Copy from reference `algorithms/Src/PID.cpp`, but:
- Replace `INRANGE(val, min, max)` with `val = std::clamp(val, min, max)`
- Add `#include <algorithm>` at top
- Remove reference to `Usermain.h`

**File: `Modules/PIDController/PIDController.hpp`**
```cpp
#pragma once
#include "include/PID.h"
// Thin wrapper — PID is a utility, Init/Update are no-ops or Reset.
class PIDController {
public:
    PIDController() {}
    void Init() {}
    void Update() {}
};
```

---

### MODULE 2: MahonyAHRS

**Goal**: Sensor fusion — gyro + accel + mag → quaternion → Euler angles. No hardware dependency.

Port from reference: `algorithms/MahonyAHRS.cpp` + `algorithms/MahonyAHRS.h`

**Critical adaptations**:
- `sampleFreq` changes from hardcoded `150.0f` to **`200.0f`** (our main loop rate)
- `twoKp` and `twoKi` become class members (not global `volatile`)
- `integralFBx/y/z` become class members
- Add `get_angle()` static method (copied from IMU.cpp line 213-218)
- Keep both `MahonyAHRSupdate()` (with mag) and `MahonyAHRSupdateIMU()` (fallback without mag)
- Keep the Quake fast inverse sqrt: `invSqrt()`

**File: `Modules/MahonyAHRS/include/MahonyAHRS.h`** — Adapted header:
```cpp
#pragma once

class MahonyAHRS {
public:
    MahonyAHRS(float sample_freq = 200.0f, float kp = 5.0f, float ki = 0.0f);

    // Main update: 9-DOF (gyro+accel+mag)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    // Fallback: 6-DOF (gyro+accel only, no mag)
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az);

    // Quaternion → Euler angles (radians). Static — no instance needed.
    static void get_angle(const float q[4], float* yaw, float* pitch, float* roll);

    // Accessors
    float quat_w() const { return q_[0]; }
    float quat_x() const { return q_[1]; }
    float quat_y() const { return q_[2]; }
    float quat_z() const { return q_[3]; }
    void  get_quat(float out[4]) const;
    float yaw()   const;
    float pitch() const;
    float roll()  const;

    void reset();

private:
    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float twoKp_;
    float twoKi_;
    float integralFBx_ = 0, integralFBy_ = 0, integralFBz_ = 0;
    float sampleFreq_;

    static float invSqrt(float x);
};
```

**File: `Modules/MahonyAHRS/src/MahonyAHRS.cpp`** — Adapted from reference:
- Copy `MahonyAHRSupdate()` body, but use member variables `q_`, `twoKp_`, `twoKi_`, `integralFBx_/y/z_`, `sampleFreq_`
- Copy `MahonyAHRSupdateIMU()` body — same adaptations
- Copy `invSqrt()` as-is (the Quake fast inverse sqrt is portable C)
- Implement `get_angle()`:
```cpp
void MahonyAHRS::get_angle(const float q[4], float* yaw, float* pitch, float* roll) {
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                     2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                     2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
```
- **Delete** `MadgwickAHRSupdate()` and `MadgwickAHRSupdateIMU()` — not needed, save flash
- Remove global `volatile float twoKp, twoKi, beta, integralFBx/y/z` — all are now class members

**File: `Modules/MahonyAHRS/MahonyAHRS.hpp`**
```cpp
#pragma once
#include "include/MahonyAHRS.h"
#include "DataBus.hpp"

class MahonyAHRSModule {
public:
    MahonyAHRSModule(float sample_freq = 200.0f, float kp = 5.0f, float ki = 0.0f)
        : ahrs_(sample_freq, kp, ki) {}

    void Init() { ahrs_.reset(); }

    void Update() {
        // Read raw data from DataBus
        float gx = robot.gyro[0], gy = robot.gyro[1], gz = robot.gyro[2];
        float ax = robot.accel[0], ay = robot.accel[1], az = robot.accel[2];
        float mx = robot.mag[0], my = robot.mag[1], mz = robot.mag[2];

        // Run fusion
        ahrs_.update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // Extract quaternion
        ahrs_.get_quat(robot.quat);

        // Extract Euler angles and angular velocities
        MahonyAHRS::get_angle(robot.quat, &robot.yaw, &robot.pitch, &robot.roll);

        // Angular velocities (from gyro, already in rad/s)
        robot.yaw_v   = gz;
        robot.pitch_v = gy;
        robot.roll_v  = gx;
    }

private:
    MahonyAHRS ahrs_;
};
```

---

### MODULE 3: IST8310Sensor

**Goal**: IST8310 magnetometer driver on I2C3. Blocks until data ready.

Port from reference:
- `drivers/ist8310driver.cpp` + `drivers/ist8310driver.h`
- `MiddleWares/ist8310driver_middleware.cpp` + `MiddleWares/ist8310driver_middleware.h`

**MERGE** middleware + driver into a single driver file — no MPU6500 passthrough, we use direct I2C3.

**Pin definitions** (already in `main.h`):
- `IST8310_RSTN_Pin = PG6` (output PP)
- `IST8310_DRDY_Pin = PG3` (EXTI3 input, can poll instead)

**I2C address**: `0x0E << 1 = 0x1C` (HAL 8-bit format)
**WHO_AM_I**: register 0x00, expected value 0x10

**File: `Modules/IST8310Sensor/include/IST8310driver.h`**
```cpp
#pragma once
#include "main.h"
#include <cstdint>

#define IST8310_IIC_ADDRESS  (0x0E << 1)
#define MAG_SEN  0.3f  // µT per LSB

uint8_t ist8310_init(I2C_HandleTypeDef* hi2c);
void ist8310_read_mag(I2C_HandleTypeDef* hi2c, float mag[3]);
```

**File: `Modules/IST8310Sensor/src/IST8310driver.cpp`**

Merge the middleware I2C functions directly into the driver:
```cpp
#include "IST8310driver.h"

// --- Internal I2C helpers (merged from middleware) ---
static I2C_HandleTypeDef* i2c_handle = nullptr;

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t val;
    HAL_I2C_Mem_Read(i2c_handle, IST8310_IIC_ADDRESS, reg,
                     I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

static void i2c_write_reg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(i2c_handle, IST8310_IIC_ADDRESS, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void i2c_read_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
    HAL_I2C_Mem_Read(i2c_handle, IST8310_IIC_ADDRESS, reg,
                     I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

// --- RSTN control ---
static void rst_low()  { HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET); }
static void rst_high() { HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET); }

// --- Init sequence (from reference ist8310driver.cpp) ---
uint8_t ist8310_init(I2C_HandleTypeDef* hi2c) {
    i2c_handle = hi2c;

    static const uint8_t cfg[4][2] = {
        {0x0B, 0x08},  // CTRL_REG3: enable interrupt
        {0x41, 0x09},  // CTRL_REG1: 200Hz, continuous
        {0x42, 0xC0},  // CTRL_REG2: ±1600µT range
        {0x0A, 0x0B}   // STATR_REG: clear status
    };

    // Hardware reset
    rst_low();
    HAL_Delay(50);
    rst_high();
    HAL_Delay(50);

    // WHO_AM_I check
    if (i2c_read_reg(0x00) != 0x10)
        return 0x40;  // IST8310_NO_SENSOR

    HAL_Delay(1);

    for (int i = 0; i < 4; i++) {
        i2c_write_reg(cfg[i][0], cfg[i][1]);
        HAL_Delay(1);
        if (i2c_read_reg(cfg[i][0]) != cfg[i][1])
            return i + 1;
    }
    return 0;
}

void ist8310_read_mag(I2C_HandleTypeDef* hi2c, float mag[3]) {
    i2c_handle = hi2c;
    uint8_t buf[6];
    i2c_read_multi(0x03, buf, 6);
    mag[0] = MAG_SEN * (int16_t)((buf[1] << 8) | buf[0]);
    mag[1] = MAG_SEN * (int16_t)((buf[3] << 8) | buf[2]);
    mag[2] = MAG_SEN * (int16_t)((buf[5] << 8) | buf[4]);
}
```

**File: `Modules/IST8310Sensor/IST8310Sensor.hpp`**
```cpp
#pragma once
#include "include/IST8310driver.h"
#include "DataBus.hpp"

extern I2C_HandleTypeDef hi2c3;

class IST8310Sensor {
public:
    IST8310Sensor(int read_period_ms = 10)
        : period_(read_period_ms) {}

    void Init() {
        ready_ = (ist8310_init(&hi2c3) == 0);
    }

    void Update() {
        if (!ready_) return;
        uint32_t now = HAL_GetTick();
        if (now - last_read_ < (uint32_t)period_) return;
        last_read_ = now;

        float raw[3];
        ist8310_read_mag(&hi2c3, raw);

        // Apply calibration (ellipsoid correction from reference)
        // MAG_SCALE/MAG_OFFSET from FinsROV — recalibrate for your hardware!
        constexpr float SX = 0.889875f, SY = 0.880329f, SZ = 1.229795f;
        constexpr float OX = 8.418958f, OY = -21.033503f, OZ = -4.048424f;

        robot.mag[0] = (raw[0] - OX) * SX;
        robot.mag[1] = (raw[1] - OY) * SY;
        robot.mag[2] = (raw[2] - OZ) * SZ;
    }

    bool is_ready() const { return ready_; }

private:
    int period_;
    uint32_t last_read_ = 0;
    bool ready_ = false;
};
```

---

### MODULE 4: PCA9685Driver

**Goal**: I2C PWM board driver on I2C2 addr 0x80, 50Hz, 16 channels. NO TCA9548A.

Port from reference: `devices/Extension.cpp` + `devices/Extension.h`
- Copy PCA_Write, PCA_Read, PCA_Setfreq, PCA_Setpwm
- DELETE all TCA9548A code (TCA_SetChannel is not needed)

**File: `Modules/PCA9685Driver/include/PCA9685.h`**
```cpp
#pragma once
#include "main.h"
#include <cstdint>

#define PCA9685_ADDR     0x80
#define PCA9685_MODE1    0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L        0x06

void PCA9685_Init(I2C_HandleTypeDef* hi2c);
void PCA9685_SetPWM(uint8_t channel, uint32_t on, uint32_t off);
void PCA9685_SetAllPWM(const int32_t pwm_us[8]);
```

**File: `Modules/PCA9685Driver/src/PCA9685.cpp`**

Port from reference `Extension.cpp`, with these changes:
- Store `I2C_HandleTypeDef* hi2c` as module-local static variable
- `PCA_Write()` → `PCA9685_Write()`: use `hi2c` instead of hardcoded `&hi2c2`
- `PCA_Read()` → `PCA9685_Read()`: same
- `PCA_Setfreq()` → `PCA9685_SetFreq()`: call `PCA9685_Write`/`PCA9685_Read`
- `PCA_Setpwm()` → `PCA9685_SetPWM()`: same
- `PCA9685_SetAllPWM()`: loops over 8 channels, converts µs→duty:
  ```cpp
  void PCA9685_SetAllPWM(const int32_t pwm_us[8]) {
      for (int i = 0; i < 8; i++) {
          int32_t pwm = pwm_us[i];
          if (pwm < 1000) pwm = 1000;
          if (pwm > 2000) pwm = 2000;
          int32_t duty = (pwm * 4096 + 10000) / 20000;
          PCA9685_SetPWM(i, 0, duty);
      }
  }
  ```
- PWM duty formula: `duty_count = (pwm_us * 4096 + 10000) / 20000` (maps 1000-2000µs to 205-410 counts)

**File: `Modules/PCA9685Driver/PCA9685Driver.hpp`**
```cpp
#pragma once
#include "include/PCA9685.h"
#include "DataBus.hpp"

extern I2C_HandleTypeDef hi2c2;

class PCA9685Driver {
public:
    PCA9685Driver(float freq = 50.0f) : freq_(freq) {}

    void Init() {
        PCA9685_Init(&hi2c2);
        // Initialize all 8 channels to neutral
        for (int i = 0; i < 8; i++)
            PCA9685_SetPWM(i, 0, (1610 * 4096 + 10000) / 20000);
    }

    void Update() {
        PCA9685_SetAllPWM(robot.pwm);
    }

private:
    float freq_;
};
```

---

### MODULE 5: Communication

**Goal**: USART6 command protocol parser. Receives commands, writes targets/flags to DataBus.

Port from reference: `Propeller::Receive()` method (Propeller.cpp lines 164-316)

**USART6 init**: Already configured in CubeMX (115200 baud). Need to start DMA RX-to-idle:
```cpp
// In Communication::Init():
HAL_UARTEx_ReceiveToIdle_IT(&huart6, rx_buf_, SERIAL_LENGTH_MAX);
```

**UART RX interrupt**: Add to `Core/Src/main.cpp` or `Core/Src/stm32f4xx_it.c`:
```cpp
extern uint8_t CommRxBuffer[100];  // declared in Communication module

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        // Signal Communication module that data is ready
        // (set a flag or copy buffer)
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, CommRxBuffer, 100);
    }
}
```

**Command set** (from reference, adapted):
| Command | Action |
|---------|--------|
| `ON` | Enable float control, set target_depth = current_depth (clamp min 30cm), init all PWM to 1610 |
| `OFF` | Disable all PID, set all PWM to 1610, target_depth = 30 |
| `W` | motion_state = FRONT |
| `S` | motion_state = BACK |
| `A` | motion_state = LEFT |
| `D` | motion_state = RIGHT |
| `Z` | motion_state = FLOAT (STOP) |
| `E` | motion_state = CLOCKWISE |
| `Q` | motion_state = ANTICLOCKWISE |
| `H:<cm>` | target_depth_cm = atoi(value) / 10.0 → robot.target_depth_cm = atoi(value) (in cm) |
| `UP` | target_depth_cm -= 1 (only when float ON) |
| `DN` | target_depth_cm += 1 (only when float ON) |
| `RPY:ON` | Enable yaw angle hold, set target_yaw = current yaw |
| `RPY:OFF` | Disable yaw angle hold, reset horizontal PWM to neutral |
| `ACL:ON` | Same as RPY:ON |
| `ACL:OF` | Same as RPY:OFF |
| `ANG:<deg>` | target_yaw = deg2rad(atoi(value)) (only when angle ON) |
| `TES:p1,...,p8` | Direct PWM test (only when float OFF && angle OFF), 8 comma-separated µs values |

**File: `Modules/Communication/Communication.hpp`**

The Receive() logic is the main port — copy `Propeller_I2C::Receive()` from reference Propeller.cpp lines 164-316, with these changes:
- Replace `PressureSensor::pressure_sensor.data_depth` → `robot.depth_m * 100.0f` (convert m→cm)
- Replace `IMU::imu.attitude.yaw` → `robot.yaw`
- Replace `Target_depth`, `Target_yaw`, `flag_float`, `flag_angle`, `motion_state` → `robot.target_depth_cm`, `robot.target_yaw`, `robot.float_enabled`, `robot.angle_enabled`, `robot.motion_state`
- Replace `current_PWM[...]` → `robot.pwm[...]`
- Replace `state_PWM_map[motion_state][i]` → needs a local copy of motion state PWM tables (or precomputed in MotorControl)
- Handle the USART6 RX DMA callback — store received buffer, process in Update()

**Key detail**: The motion state PWM tables (FrontPWM, BackPWM, etc.) that Communication uses to set horizontal motor PWMs should be computed in MotorControl and exposed. OR: Communication just sets `motion_state` flag, and MotorControl computes the base PWMs in its Update(). This is cleaner — Communication only sets flags, MotorControl does all PWM computation.

**Simplified approach**: Communication only writes targets and flags. It never directly sets PWM. The MotorControl module reads the flags and motion_state to compute PWMs.

---

### MODULE 6: MotorControl (THE CORE)

**Goal**: 8-motor PID cascade + thrust allocation. Reads sensors/AHRS/targets, writes 8 PWM values.

Port from reference: `Propeller.cpp` + `Propeller.h`

**This is the largest module.** The reference code is at:
`C:\Users\40713\Desktop\1132\origin_bot\rov-xy\FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main\Code\Lower_Level_Controller\userCode\devices\Src\Propeller.cpp`

**Motor layout** (keep from reference):
- 4 vertical (inner): PCA9685 channels 1,2,6,5 — heave, roll, pitch
- 4 horizontal (outer): PCA9685 channels 0,3,7,4 — surge, sway, yaw
- Sign array: `{1, -1, 1, 1, -1, -1, 1, -1}` (clockwise=1, counter-clockwise=-1)
- Neutral: 1610µs
- Dead-zone compensation: 50µs per motor

**PID controllers** (7 total, from reference):

| PID | Kp | Ki | Kd | KpMax | KiMax | KdMax | OutMax | Loop Rate |
|-----|----|----|----|-------|-------|-------|--------|-----------|
| DepthPID | 10 | 0.02 | 10 | 100 | 50 | 50 | 200 | 200Hz |
| RollOutPID | 0.5 | 0.002 | 1 | 5 | 2.5 | 2.5 | 10 | ~66Hz |
| RollInPID | 8 | 0 | 0 | 100 | 50 | 50 | 200 | 200Hz |
| PitchOutPID | 1 | 0.005 | 1 | 5 | 2.5 | 2.5 | 10 | ~66Hz |
| PitchInPID | 8 | 0 | 0 | 100 | 50 | 50 | 200 | 200Hz |
| YawOutPID | 2 | 0.01 | 2 | 10 | 5 | 5 | 20 | ~66Hz |
| YawInPID | 5 | 0 | 0 | 200 | 100 | 100 | 400 | 200Hz |

**Key changes from reference**:
1. Roll/Pitch outer loop feedback: `IMU.attitude.roll` → `robot.roll` (in rad, convert from reference's pressure differential)
2. Depth feedback: `PressureSensor.data_depth` (cm) → `robot.depth_m * 100.0f` (convert m→cm)
3. Outer loop timing: replace `ps_state == CALCULATE` with `loop_count % 3 == 0` (every 3rd iteration at 200Hz = ~66Hz)
4. Remove: 4-sensor averaging, TCA_SetChannel, ps_state gating
5. The reference has `IMU::imu.attitude.neg_roll_v` for roll inner loop — check sign based on your IMU mounting orientation

**vertical_PWM_allocation()** (adapted from reference lines 384-417):
```cpp
void MotorControl::vertical_allocation() {
    if (!robot.float_enabled) return;

    float_ctrl();  // runs PID cascade, fills pwm_comp_

    constexpr int8_t factors[4][3] = {
        {-1, -1, -1},  // Motor 0 (InID[0]=ch1): front-left vertical
        {-1, -1,  1},  // Motor 1 (InID[1]=ch2): rear-left vertical
        {-1,  1, -1},  // Motor 2 (InID[2]=ch6): front-right vertical
        {-1,  1,  1}   // Motor 3 (InID[3]=ch5): rear-right vertical
    };

    for (int i = 0; i < 4; i++) {
        uint8_t idx = InID_[i];           // PCA9685 channel
        int8_t sign = Sign_[idx];          // ±1 for propeller handedness
        int32_t base = FloatPWM_[i];      // neutral + small trim
        int32_t comp = Compensation_[idx]; // dead-zone

        int32_t pwm = base - sign * (
            pwm_comp_.depth * factors[i][0] +
            pwm_comp_.roll  * factors[i][1] +
            pwm_comp_.pitch * factors[i][2]
        );

        if (pwm > InitPWM_) pwm += comp;
        else if (pwm < InitPWM_) pwm -= comp;

        robot.pwm[idx] = pwm;
    }
}
```

**float_ctrl()** (adapted from reference lines 336-356):
```cpp
void MotorControl::float_ctrl() {
    // Depth PID: single loop, full rate (200Hz)
    // depth_m → cm: multiply by 100
    pwm_comp_.depth = depth_pid_.PIDCalc(
        robot.target_depth_cm,
        robot.depth_m * 100.0f
    );

    // Outer loops: run at ~66Hz (every 3rd cycle)
    if (robot.loop_count % 3 == 0) {
        // Roll outer: target_roll (0=level) vs IMU roll angle
        // NOTE: IMU gives radians, PID expects degrees?
        // Reference uses radians internally — keep same convention
        float roll_rad = robot.roll;
        float pitch_rad = robot.pitch;
        target_roll_v_ = roll_out_pid_.PIDCalc(0.0f, roll_rad);
        target_pitch_v_ = pitch_out_pid_.PIDCalc(0.0f, pitch_rad);
    }

    // Inner loops: full rate (200Hz)
    pwm_comp_.roll  = roll_in_pid_.PIDCalc(target_roll_v_, -robot.roll_v);
    pwm_comp_.pitch = pitch_in_pid_.PIDCalc(target_pitch_v_, robot.pitch_v);
}
```

**horizontal_PWM_allocation()** (adapted from reference lines 421-444):
```cpp
void MotorControl::horizontal_allocation() {
    if (!robot.float_enabled) return;

    float yaw_comp = 0;
    if (robot.angle_enabled) {
        angle_ctrl();
        yaw_comp = pwm_comp_.yaw;
    }

    int state = robot.motion_state;
    // state_PWM_map: pre-computed base PWMs for each motion direction
    const auto& base = state_pwm_map_[state];

    for (int i = 0; i < 4; i++) {
        uint8_t idx = OutID_[i];
        int32_t sign = Sign_[idx];
        int32_t comp = Compensation_[idx];

        // Front motors (i<2) and rear motors (i>=2) get opposite yaw
        int32_t pwm = base[i] + ((i < 2) ? sign : -sign) * yaw_comp;

        if (pwm > InitPWM_) pwm += comp;
        else if (pwm < InitPWM_) pwm -= comp;

        robot.pwm[idx] = pwm;
    }
}
```

**angle_ctrl()** (adapted from reference lines 364-372):
```cpp
void MotorControl::angle_ctrl() {
    if (robot.loop_count % 3 == 0) {
        float yaw_diff = normalize_angle(robot.yaw - robot.target_yaw);
        target_yaw_v_ = yaw_out_pid_.PIDCalc(0.0f, yaw_diff);
    }
    pwm_comp_.yaw = yaw_in_pid_.PIDCalc(target_yaw_v_, robot.yaw_v);
}
```

**normalize_angle()** (copy from reference lines 447-452):
```cpp
static float normalize_angle(float angle) {
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle - M_PI;
}
```

**Motion state PWM tables** (copy from reference lines 26-66):
```cpp
// Defined as module-level constants (same values as reference)
constexpr int32_t InitPWM = 1610;
constexpr uint8_t longitudinal_speed = 80;
constexpr uint8_t lateral_speed = 80;
constexpr uint8_t rotate_speed = 40;

// Sign, InID, OutID arrays
constexpr int8_t  Sign[8]  = {1, -1, 1, 1, -1, -1, 1, -1};
constexpr uint8_t InID[4]  = {1, 2, 6, 5};
constexpr uint8_t OutID[4] = {0, 3, 7, 4};

// FloatPWM (vertical neutral with small trim)
// FrontPWM, BackPWM, LeftPWM, RightPWM, ClockwisePWM, AntiClockwisePWM, StopPWM
// ALL copied verbatim from reference Propeller.cpp lines 26-66
```

**MotorControl::Update()** (main entry, called at 200Hz):
```cpp
void MotorControl::Update() {
    robot.loop_count++;

    if (robot.float_enabled) {
        vertical_allocation();
        horizontal_allocation();
    } else {
        // Float OFF: all motors at neutral
        for (int i = 0; i < 8; i++)
            robot.pwm[i] = InitPWM_;
    }
}
```

**MotorControl::Init()**:
- Initialize all 7 PID controllers with reference gains
- Copy Sign, InID, OutID, Compensation, FloatPWM arrays
- Build state_PWM_map (Stop/Front/Back/Left/Right/Clockwise/Anticlockwise)
- Set InitPWM_ = 1610

---

### MODULE 7 (Bonus): KalmanFilter

Optional — only needed if you want to filter the depth sensor. The reference uses 4 Kalman filters (one per sensor). With single sensor, a simple moving average or low-pass filter may suffice. Port from `algorithms/Kalman_Filter.cpp/.h` if desired.

---

## Integration Steps

### Step 1: Create all module directories and files

```
Modules/
  DataBus/
    DataBus.hpp
    include/robot_data.hpp
    src/robot_data.cpp
    xy_module.yaml
  PIDController/
    PIDController.hpp
    include/PID.h
    src/PID.cpp
    xy_module.yaml
  MahonyAHRS/
    MahonyAHRS.hpp
    include/MahonyAHRS.h
    src/MahonyAHRS.cpp
    xy_module.yaml
  IST8310Sensor/
    IST8310Sensor.hpp
    include/IST8310driver.h
    src/IST8310driver.cpp
    xy_module.yaml
  PCA9685Driver/
    PCA9685Driver.hpp
    include/PCA9685.h
    src/PCA9685.cpp
    xy_module.yaml
  Communication/
    Communication.hpp
    include/protocol.h
    src/protocol.cpp
    xy_module.yaml
  MotorControl/
    MotorControl.hpp
    include/motor_control.h
    src/motor_control.cpp
    xy_module.yaml
```

### Step 2: Update `xy_robot.yaml`

```yaml
project:
  name: 1132_bot
  loop_sleep_ms: 5
  flavor: stm32_hal

modules:
  - id: databus
    type: DataBus
    include: DataBus.hpp

  - id: imu
    type: BMI088Sensor
    include: BMI088Sensor.hpp
    args:
      read_period_ms: 5
      accel_range: "3G"
      gyro_range: "2000"

  - id: depth_sensor
    type: MS5837Sensor
    include: MS5837Sensor.hpp
    args:
      fluid_density: 997
      read_period_ms: 100

  - id: ist8310
    type: IST8310Sensor
    include: IST8310Sensor.hpp
    args:
      read_period_ms: 10

  - id: ahrs
    type: MahonyAHRS
    include: MahonyAHRS.hpp
    args:
      sample_freq: 200.0
      kp: 5.0
      ki: 0.0

  - id: pid
    type: PIDController
    include: PIDController.hpp

  - id: pca9685
    type: PCA9685Driver
    include: PCA9685Driver.hpp
    args:
      frequency: 50

  - id: motor_control
    type: MotorControl
    include: MotorControl.hpp

  - id: communication
    type: Communication
    include: Communication.hpp
```

### Step 3: Update `src/generated_xy_robot_main.hpp`

```cpp
#pragma once
// This file was generated by xy_robotkit. Do not edit.

#include "main.h"
#include "DataBus.hpp"
#include "BMI088Sensor.hpp"
#include "MS5837Sensor.hpp"
#include "IST8310Sensor.hpp"
#include "MahonyAHRS.hpp"
#include "PIDController.hpp"
#include "PCA9685Driver.hpp"
#include "MotorControl.hpp"
#include "Communication.hpp"

static BMI088Sensor  imu(5, "3G", "2000");
static MS5837Sensor   depth_sensor(997, 100);
static IST8310Sensor  ist8310(10);
static MahonyAHRSModule ahrs(200.0f, 5.0f, 0.0f);
static PIDController  pid;       // no-op, included for completeness
static PCA9685Driver  pca9685(50);
static MotorControl   motor_control;
static Communication  comm;

inline void XYRobotSetup() {
    imu.Init();
    depth_sensor.Init();
    ist8310.Init();
    ahrs.Init();
    pca9685.Init();
    motor_control.Init();
    comm.Init();
}

inline void XYRobotLoop() {
    // 1. Sensor acquisition
    imu.Update();             // → robot.accel, robot.gyro
    depth_sensor.Update();    // → robot.depth_m (every 100ms)
    ist8310.Update();         // → robot.mag (every 10ms)

    // 2. Attitude estimation
    ahrs.Update();            // reads gyro/accel/mag → robot.quat, robot.yaw/pitch/roll, robot.yaw_v/pitch_v/roll_v

    // 3. Communication
    comm.Update();            // reads USART6 → robot.target_*, robot.flags

    // 4. Motor control
    motor_control.Update();   // reads all sensor/ahrs/target data → PID cascade → robot.pwm[8]

    // 5. Actuator output
    pca9685.Update();         // reads robot.pwm[8] → I2C to PCA9685

    HAL_Delay(5);
}
```

### Step 4: Update `Core/Src/main.cpp`

Add USART6 DMA RX interrupt callback. Find the `HAL_UARTEx_RxEventCallback` (or create it):

```cpp
// At top of main.cpp:
#include "Communication.hpp"  // or wherever CommRxBuffer is

// USART6 DMA RX buffer — defined in Communication module
#define SERIAL_LENGTH_MAX 100
static uint8_t usart6_rx_buf[SERIAL_LENGTH_MAX];

// Override the weak HAL callback
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        // Copy to Communication's buffer for processing
        extern uint8_t CommRxBuffer[SERIAL_LENGTH_MAX];
        extern volatile bool CommRxReady;
        memcpy(CommRxBuffer, usart6_rx_buf, Size < SERIAL_LENGTH_MAX ? Size : SERIAL_LENGTH_MAX);
        CommRxBuffer[Size] = '\0';
        CommRxReady = true;
        // Re-arm DMA
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, usart6_rx_buf, SERIAL_LENGTH_MAX);
    }
}
```

### Step 5: Update `Core/Src/stm32f4xx_it.c`

Ensure USART6 interrupt is handled. If using DMA, the DMA stream interrupt must be enabled.

### Step 6: Update existing `BMI088Sensor` to write to DataBus

Modify `Modules/BMI088Sensor/BMI088Sensor.hpp` Update() to also write to DataBus:
```cpp
void Update() {
    // ...existing read logic...
    BMI088_read(_gyro, _accel, &_temp);
    // Write to DataBus
    robot.accel[0] = _accel[0];
    robot.accel[1] = _accel[1];
    robot.accel[2] = _accel[2];
    robot.gyro[0]  = _gyro[0];
    robot.gyro[1]  = _gyro[1];
    robot.gyro[2]  = _gyro[2];
    robot.imu_temp  = _temp;
}
```

### Step 7: Update existing `MS5837Sensor` to write to DataBus

Modify `Modules/MS5837Sensor/MS5837Sensor.hpp` Update() to also write to DataBus:
```cpp
void Update() {
    // ...existing read logic...
    sensor_.read();
    robot.depth_m       = sensor_.depth();
    robot.pressure_mbar = sensor_.pressure();
    robot.water_temp_c  = sensor_.temperature();
}
```

---

## Build and Verify

```powershell
# Configure
cmake --preset Debug

# Build
cmake --build build/Debug

# Flash (CMSIS-DAP)
cmake --build build/Debug --target flash
```

### Verification Sequence:
1. **Build passes** — all 7 modules compile without errors
2. **PCA9685 output** — scope shows 50Hz PWM on all channels, neutral=1610µs at reset
3. **IMU + AHRS** — serial output yaw/pitch/roll values, verify they change when rotating the ROV
4. **Open-loop control** — send W/A/S/D via USART6, verify correct PWM changes on horizontal channels
5. **Depth hold** — place in water, send ON, verify DepthPID modulates vertical thrusters
6. **Attitude hold** — tilt the ROV, verify roll/pitch PIDs counter-steer

---

## Notes for kiro-cli

- **Read each reference file before porting it.** The reference code paths are listed at the top.
- **Start with DataBus + PIDController + MahonyAHRS** (no hardware deps, can compile standalone)
- **Then IST8310Sensor + PCA9685Driver** (I2C hardware, test each independently)
- **Then Communication** (USART6, can test with serial terminal)
- **Finally MotorControl** (integrates everything, most complex)
- The reference's `INRANGE` macro must become `std::clamp` (C++17, already in project CMakeLists.txt)
- The reference uses `PI` — define `M_PI` from `<cmath>` or use `3.14159265358979323846f`
- The reference's `using namespace std;` → remove, use explicit `std::` prefixes
- All references to `PressureSensor::pressure_sensor.data_depth` become `robot.depth_m * 100.0f`
- All references to `PressureSensor::pressure_sensor.data_roll/data_pitch` are **deleted** (replaced by IMU angles)
- All references to `IMU::imu.attitude.*` become `robot.*` (pitch, roll, yaw, pitch_v, roll_v, yaw_v)
- `ps_state == CALCULATE` gating → `robot.loop_count % 3 == 0`
- `TCA_SetChannel(4)` → **delete** (no mux needed)
- The IST8310 mag calibration constants (MAG_SCALE, MAG_OFFSET) from the reference are for the FinsROV hardware — they WILL need recalibration for your specific magnetometer mounting. Leave them as-is for now; they'll be close enough for initial testing.
