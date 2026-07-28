# MS5837Sensor

BlueRobotics MS5837-30BA / 02BA 压力 + 深度传感器的 xy_robotkit 模块封装。

底层 C++ 类来自 `MS5837_STM32_HAL` 的 STM32F4 HAL 移植版（继承自 BlueRobotics MIT 库），本模块只在它之上加一层 `Init()` / `Update()` 生命周期外壳，并把构造参数（流体密度、读取周期）做成 `xy_module.yaml` 声明，方便 `xy_robotkit` 校验和生成。

## 文件结构

```
MS5837Sensor/
  MS5837Sensor.hpp     C++ 生命周期适配器（暴露 Init/Update + 访问器）
  xy_module.yaml       参数 schema
  README.md            本文件
  include/
    MS5837.h           原 HAL 移植头文件
  src/
    MS5837.cpp         原 HAL 移植实现
```

`include/MS5837.h` 和 `src/MS5837.cpp` 与 `MS5837_STM32_HAL/` 中的同名文件保持一致——本模块**自包含**这些源文件，方便 `xy_robotkit export-module` / `pull` 在不同项目之间复用。

## 生命周期

`Init()`：

1. 最多三次调用 `MS5837::init(&hi2c2)`（失败间隔 100ms），做 reset → 读 PROM → CRC 校验。
2. 仅接受已识别的 MS5837-30BA / 02BA；成功后调用 `setFluidDensity(...)`，并把 `is_ready()` 置 true。
3. 三次均失败时 `is_ready()` 保持 false；启动过程有界，不会永久阻塞其它模块。

`Update()`：

- 仅在 `is_ready()` 为 true 时工作。
- 用 `HAL_GetTick()` 节流，每隔 `read_period_ms` 调用一次 `MS5837::read()`（阻塞约 40 ms）；只有完整采样成功且数值有限时才发布新的深度数据。
- 启动失败后每 1s 最多重试一次；恢复时仍保持样本无效，直到首个成功采样，且不会自动恢复 ARM 或定深状态。

## 参数

| 参数 | 类型 | 必填 | 默认 | 说明 |
| --- | --- | --- | --- | --- |
| `fluid_density` | int | 否 | 997 | 流体密度 kg/m³。淡水 997，海水 1029。 |
| `read_period_ms` | int | 否 | 100 | 至少多少毫秒采样一次。建议 ≥ 50 ms，因为单次 `read()` 约 40 ms 阻塞。 |

## 数据访问

`Update()` 完成至少一次成功采样后，可以调用：

```cpp
float p = depth.pressure_mbar();   // 绝对压力 mbar
float t = depth.temperature_c();   // 温度 °C
float d = depth.depth_m();         // 深度 m
bool  k = depth.is_ready();        // sensor.init() 是否成功
```

`depth_m()` 的语义只在水中有效；空气中应使用 `MS5837::altitude()`，本适配器没有暴露，需要时自行扩展或直接用底层类。

## I2C 句柄

`MS5837Sensor.hpp` 顶部写死：

```cpp
extern "C" I2C_HandleTypeDef hi2c1;
```

含义：模块编译期假设 CubeMX 工程里已经配置了 `I2C1` 外设。如果你接到的是 `I2C2` / `I2C3`，请直接修改这一行，例如：

```cpp
extern "C" I2C_HandleTypeDef hi2c2;
...
ready_ = sensor_.init(&hi2c2);
```

将来 xy_robotkit 如果加入 `symbol` / `raw` 参数类型，可以让句柄符号通过 `xy_module.yaml` 配置；当前版本（v1.3）暂未支持这种参数类型，所以靠手工改这一行最简洁。

## CubeMX 工程集成步骤

1. **CubeMX 配置 I2C1**（推荐 400 kHz Fast Mode），重新生成代码。
2. 把 `Modules/MS5837Sensor/`（含 `include/`、`src/`）整个目录拷贝或加入到 STM32 工程，并在 IDE 里把 `Modules/MS5837Sensor/include` 和 `Modules/MS5837Sensor/` 加入 include path，把 `Modules/MS5837Sensor/src/MS5837.cpp` 加入构建源列表。
3. CubeMX 默认是 C 工程；要么把 `main.c` 改名 `main.cpp`，要么加一个自己的 `app_pressure.cpp` 用 `extern "C"` 包装入口供 main.c 调用。
4. 运行 `xy_robotkit generate`（在示例工程目录里），把生成的 `src/generated_xy_robot_main.hpp` 拷或符号链接到 STM32 工程能 include 的位置。
5. 在 main 里：

   ```cpp
   #include "generated_xy_robot_main.hpp"
   int main(void) {
       HAL_Init();
       SystemClock_Config();
       MX_GPIO_Init();
       MX_I2C1_Init();
       XYRobotSetup();
       while (1) { XYRobotLoop(); }
   }
   ```

参考 `MS5837_STM32_HAL/example/main_example.cpp`，那个文件展示了不经过 xy_robotkit 时直接用底层 `MS5837` 类的写法；本模块包是在它之上的"被 xy_robotkit 装配"的版本。

## 复用流程

```bash
# 在某个项目里使用这个模块
xy_robotkit pull MS5837Sensor --from C:\Users\40713\Desktop\xy_robot_modules
xy_robotkit add MS5837Sensor --id depth --arg fluid_density=1029 --arg read_period_ms=50
xy_robotkit check
xy_robotkit generate
```

注意：`xy_robotkit` 不会替你把 `Modules/MS5837Sensor/src/MS5837.cpp` 加到 STM32 工程的构建列表里，这一步仍要手动完成（IDE / Makefile / CMake，看你工程类型）。

## 许可证

继承底层库的 MIT 许可证。版权归 Blue Robotics Inc. (Rustom Jehangir, Adam Šimko) 与本模块贡献者共同所有。
