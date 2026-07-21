# BMI088Sensor

BMI088 六轴 IMU（陀螺仪 + 加速度计）模块，阻塞 SPI 读取。

## 硬件

- 芯片：Bosch BMI088（加速度计 + 陀螺仪）
- 接口：SPI（默认 SPI1）
- 默认引脚（RoboMaster 开发板 C 型）：
  - CS_ACCEL: PA4
  - CS_GYRO: PB0
  - SCK: PB3, MISO: PB4, MOSI: PA7

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| read_period_ms | int | 5 | 采样周期（ms），默认 200Hz |
| accel_range | string | "3G" | 加速度计量程：3G/6G/12G/24G |
| gyro_range | string | "2000" | 陀螺仪量程(dps)：2000/1000/500/250/125 |

## 数据访问

- `accel_x/y/z()` — 加速度 m/s²（乘以灵敏度系数后的值）
- `gyro_x/y/z()` — 角速度 rad/s
- `temperature()` — 芯片温度 °C
- `pitch()` / `roll()` — 由加速度计简单计算的姿态角（度），无 AHRS

## CubeMX 工程集成步骤

1. CubeMX 启用 SPI1（Full-Duplex Master），配置 PA4 和 PB0 为 GPIO Output（CS 引脚），重新生成代码。
2. 把 `Modules/BMI088Sensor/`（含 `include/`、`src/`）整体拷进 STM32 工程。
3. 把 `Modules/BMI088Sensor/include`、`Modules/BMI088Sensor/` 加入 include path；把 `src/BMI088driver.cpp` 加入构建源列表。
4. 把 `xy_robotkit generate` 产生的 `src/generated_xy_robot_main.hpp` 拷到 STM32 工程能 include 的位置。
5. main 改成 `.cpp`，调用 `XYRobotSetup()` / `XYRobotLoop()`。

## SPI 句柄硬编码说明

`BMI088Sensor.hpp` 使用 `extern SPI_HandleTypeDef hspi1;`。如果你的板子用 SPI2/SPI3，请修改 `src/BMI088driver.cpp` 中 Middleware 部分的 `hspi1` 引用。

## 注意

- 当前实现为阻塞 SPI 读取，不使用 DMA。
- 不含 IST8310 磁力计。
- 不含椭圆校准和温度 PID。
- `accel_range` / `gyro_range` 参数当前通过编译时宏控制（`BMI088driver.h` 中的 `#define`），运行时参数仅作文档用途。
