# 1132_bot 使用文档

本文档覆盖日常开发、固件构建、Orange Pi 调试台启动、Web 控制台使用、串口命令和测试验证。系统安全逻辑详见 [SAFETY.md](SAFETY.md)，系统结构详见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 0. 快速选择

你通常会遇到两种使用场景：

| 场景 | 在哪里运行 | 启动命令 | 浏览器访问 |
| --- | --- | --- | --- |
| 只想先看网页长什么样 | 自己电脑 | `./scripts/start_web_console.sh --simulate` | `http://localhost:8000` |
| 香橙派连接 STM32 真实调试 | 香橙派 | `./scripts/start_web_console.sh` | `http://香橙派IP:8000` |
| 香橙派不接 STM32 先试网页 | 香橙派 | `./scripts/start_web_console.sh --simulate` | `http://香橙派IP:8000` |

关键点：

- 香橙派端只需要跑 `./scripts/start_web_console.sh`。
- 浏览器可以在香橙派本机打开，也可以在同一局域网的电脑、平板、手机打开。
- 服务跑在哪台机器，浏览器就访问哪台机器的 IP。
- 如果服务跑在自己电脑上，浏览器用 `http://localhost:8000`。
- 如果服务跑在香橙派上，浏览器用 `http://香橙派IP:8000`。

## 1. 安全前提

在连接推进器、电调或下水测试前，先满足这些条件：

- 推进器固定可靠，测试人员远离旋转部件。
- 首次上电或改代码后，先断开推进器电源，只看 PWM 信号。
- PCA9685 输出中位应为 `1500us`。
- 手动测试 PWM 范围只使用 `1450-1550us`。
- 任意异常优先执行急停：Web 控制台 `EMERGENCY STOP`、二进制协议 `EMERGENCY_STOP`，或文本串口 `OFF` / `NEU`。
- 急停后系统进入锁定状态，需要确认安全后执行 `RESET ESTOP` 才能重新 ARM。

## 2. 固件构建

在工程根目录执行：

```bash
cmake -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=/home/zoae/stm32_projects/1132_bot/cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE \
  -S /home/zoae/stm32_projects/1132_bot \
  -B /home/zoae/stm32_projects/1132_bot/build/Debug \
  -G Ninja

cmake --build /home/zoae/stm32_projects/1132_bot/build/Debug --
```

构建成功后会生成：

- `build/Debug/1132_bot.elf`
- `build/Debug/1132_bot.hex`
- `build/Debug/1132_bot.bin`

如果需要重新全量构建，可删除 `build/Debug` 后重新执行上面的配置和构建命令。

## 3. 烧录固件

项目提供了 CMake 的 `flash` 目标，使用 DAP-Link/OpenOCD 脚本：

```bash
cmake --build /home/zoae/stm32_projects/1132_bot/build/Debug --target flash
```

前提：

- DAP-Link 已连接 STM32F407。
- `tools/flash-daplink.ps1` 可运行。
- OpenOCD 可被脚本找到，或按 `CMakeLists.txt` 中的 `FLASH_INTERFACE_CFG` / `FLASH_TARGET_CFG` 配置。

如果使用 STM32CubeProgrammer、J-Link 或其他烧录方式，直接烧录 `build/Debug/1132_bot.elf` 或 `build/Debug/1132_bot.hex`。

## 4. Orange Pi 端部署和启动

默认串口配置在 `opi_console/config.yaml`：

- 串口：`/dev/ttyS5`
- 波特率：`115200`
- 心跳间隔：`200ms`
- STM32 心跳超时：`1000ms`
- Web 端口：`8000`

启动脚本不会再用脚本内默认值覆盖 YAML。配置优先级为：

1. 命令行参数
2. `ROV_*` 环境变量
3. YAML 配置

有效配置会在打开串口前整体校验。协议版本必须为 `2`，通道数必须为
`8`，中位必须为 `1500us`，波特率必须和当前固件一致为 `115200`；未知或
拼错的配置键会导致启动失败，不会静默忽略。

进程还会持有 `safety.process_lock_file` 指定的非阻塞文件锁。重复启动会在
打开串口前提示已有实例并退出，防止两个后端绕过进程内控制锁。

### 4.1 第一次放到香橙派

把整个工程目录放到香橙派，例如放在：

```bash
/home/orangepi/1132_bot
```

如果从电脑拷贝到香橙派，可以用类似命令：

```bash
scp -r /path/to/1132_bot orangepi@香橙派IP:/home/orangepi/1132_bot
```

也可以用 U 盘、Git、SFTP 等方式，只要香橙派上有完整工程目录即可。

进入工程目录：

```bash
cd /home/orangepi/1132_bot
```

确保启动脚本可执行：

```bash
chmod +x scripts/start_web_console.sh
```

### 4.2 安装 Python 依赖

推荐使用虚拟环境，避免把项目依赖直接装到系统 Python 里：

运行环境需要 Python `3.10+`。

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

之后 `start_web_console.sh` 会自动检测并激活 `venv`。如果没有虚拟环境，它也会尝试直接用系统 Python 安装依赖。

如果香橙派系统是 Arch Linux / Arch Linux ARM，`python3 -m venv venv` 可能因为系统没有 `pip` 或 `ensurepip` 失败，例如：

```text
ensurepip returned non-zero exit status 1
pip: 未找到命令
```

这种情况下优先安装系统包，并用 `virtualenv` 创建环境：

```bash
sudo pacman -Syu --needed python-pip python-virtualenv python-platformdirs

cd /home/zooae/桌面/1132_bot_orangepi
rm -rf venv
python3 -m virtualenv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

如果不想做整机升级，只是在实验台上临时修复，也可以先做最小包同步：

```bash
sudo pacman -Sy --needed python-pip python-virtualenv python-platformdirs
```

Arch 系统不推荐长期只做部分升级。出现 Python、glibc、expat 这类底层库版本不匹配时，优先在供电和网络稳定时执行完整系统升级。

如果启动真实硬件模式时看到类似错误：

```text
/usr/lib/libc.so.6: version `GLIBC_2.42' not found
```

说明当前 Python 模块比系统 `glibc` 新，串口模块 `termios` 不能加载。先查看版本：

```bash
python3 --version
ldd --version | head -n 1
pacman -Q python glibc expat python-pip python-virtualenv python-platformdirs
```

然后升级底层库：

```bash
sudo pacman -Syu --needed glibc expat
```

修复后重新创建虚拟环境：

```bash
rm -rf venv
python3 -m virtualenv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

验证真实串口能打开：

```bash
./scripts/start_web_console.sh
```

启动日志中看到下面这一行，说明香橙派已经能打开 STM32 串口：

```text
Connected to /dev/ttyS5 at 115200 baud
```

### 4.3 确认网页前端文件

后端会自动提供 `web_frontend/dist` 里的静态网页。

如果你已经在电脑上执行过：

```bash
cd web_frontend
npm run build
```

并把整个工程拷到了香橙派，那么香橙派上通常已经有 `web_frontend/dist`，不需要再装 Node。

如果香橙派上没有 `web_frontend/dist`，有两种选择：

1. 在电脑上构建前端后再拷贝整个工程。
2. 在香橙派上安装 Node.js，然后执行：

```bash
cd web_frontend
npm ci
npm run build
cd ..
```

### 4.4 确认香橙派 IP

在香橙派上执行：

```bash
hostname -I
```

输出里一般会有一个局域网 IP，例如：

```text
192.168.1.23
```

电脑浏览器访问时用：

```text
http://192.168.1.23:8000
```

### 4.5 真实硬件模式启动

真实硬件模式用于香橙派已经接好 STM32 的场景：

```bash
./scripts/start_web_console.sh
```

它会做这些事：

- 打开串口 `/dev/ttyS5`
- 使用 `115200` 波特率和 STM32 通信
- 启动后端 API
- 启动 Web 页面服务
- 周期发送心跳
- 接收 STM32 状态、传感器和 PWM 遥测

启动成功后，终端会显示类似：

```text
MODE: REAL HARDWARE
Serial: /dev/ttyS5 @ 115200 baud
Web: http://0.0.0.0:8000
```

此时在电脑浏览器打开：

```text
http://香橙派IP:8000
```

### 4.6 香橙派上先不接 STM32，只看网页

仿真模式不需要 STM32，也不需要串口：

```bash
./scripts/start_web_console.sh --simulate
```

浏览器仍然访问：

```text
http://香橙派IP:8000
```

页面会显示仿真数据，可以测试解锁、进入手动测试、PWM 测试、急停等流程。

### 4.7 指定串口或端口

如果 STM32 不在 `/dev/ttyS5`，先列出串口设备：

```bash
ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

例如实际串口是 `/dev/ttyS1`：

```bash
./scripts/start_web_console.sh --serial /dev/ttyS1
```

如果 `8000` 端口被占用，换成 `9000`：

```bash
./scripts/start_web_console.sh --port 9000
```

同时指定串口和端口：

```bash
./scripts/start_web_console.sh --serial /dev/ttyS1 --port 9000
```

浏览器对应访问：

```text
http://香橙派IP:9000
```

也可以使用环境变量：

```bash
ROV_SIMULATE=true ROV_WEB_PORT=9000 ./scripts/start_web_console.sh
ROV_SERIAL_PORT=/dev/ttyS1 ./scripts/start_web_console.sh
```

支持的变量为 `ROV_CONFIG`、`ROV_SIMULATE`、`ROV_SERIAL_PORT`、
`ROV_SERIAL_BAUD`、`ROV_WEB_HOST`、`ROV_WEB_PORT` 和 `ROV_LOG_LEVEL`。
YAML 中的 `simulation.enabled: true` 也会直接启用仿真；需要临时强制真实
硬件模式时使用 `--hardware`。

### 4.8 串口权限

如果启动时报串口权限错误，先查看串口权限：

```bash
ls -l /dev/ttyS5
```

常见做法是把当前用户加入串口权限组：

```bash
sudo usermod -aG dialout $USER
```

然后退出 SSH 重新登录，或者重启香橙派。不同系统也可能使用 `tty` 组，可根据 `ls -l /dev/ttyS5` 的组名调整。

临时调试时也可以直接用 `sudo` 启动：

```bash
sudo ./scripts/start_web_console.sh
```

长期使用不建议一直依赖 `sudo`，最好把用户权限配好。

### 4.9 停止服务

启动脚本在前台运行。停止服务按：

```text
Ctrl+C
```

正常停止时后端会先请求 `SET_ALL_NEUTRAL` 并等待状态确认，然后无论回中
是否确认成功都继续尝试 `DISARM`。如果串口已经断开，STM32 仍必须依靠自身
心跳超时独立回中。

日志默认写入：

- `logs/opi_console.log`
- `logs/events.log`

查看实时日志：

```bash
tail -f logs/opi_console.log
```

查看事件和运行诊断：

```bash
tail -f logs/events.log
curl -s http://127.0.0.1:8000/health
curl -s http://127.0.0.1:8000/api/diagnostics
```

诊断接口包含串口/STM32 新鲜度、WebSocket 客户端数、收发帧、CRC 错误、
ACK 超时、NACK、最近错误以及后台任务状态。每条 PWM 操作日志包含
`request_id`、`sequence`、通道、脉宽、持续时间、结果和原因。

## 5. Web 控制台流程

推荐调试顺序：

1. 打开 Web 页面。
2. 顶部状态应显示 `WebSocket 已连接`。
3. 真实硬件模式下，系统状态里应看到串口正常、STM32 在线。
4. 确认当前状态为 `未解锁`，PWM 输出为全通道 `1500us`。
5. 点击 `解锁`，状态应变为 `已解锁待机`。
6. 点击 `进入手动测试`，状态应变为 `手动测试`。
7. 在 PWM 手动测试面板选择通道 `CH0-CH7`。
8. PWM 脉宽只用 `1450-1550us`，持续时间只用 `200-2000ms`。
9. 点击 `输出 CHx` 进行短脉冲测试。
10. 换通道前系统会先让前一个通道回中位。
11. 测试结束后点击 `全部回中`。
12. 不再测试时点击 `上锁并回中`。
13. 出现异常立即点击右上角 `急停`。

页面主要区域：

- 顶部栏：连接状态、当前安全状态、解锁/上锁/急停操作。
- 系统状态：串口、STM32、控制使能、定深、角度闭环、协议错误和心跳丢失。
- 传感器：深度、压力、水温、航向、俯仰、横滚、IMU 三轴数据。
- PWM 手动测试：只在 `手动测试` 状态允许输出。
- PWM 输出：显示 8 个通道当前值和目标值。
- 事件日志：显示连接、控制命令、错误提示。

常用 REST 接口：

```bash
curl http://localhost:8000/api/status
curl http://localhost:8000/api/sensors
curl -X POST http://localhost:8000/api/arm
curl -X POST http://localhost:8000/api/enter-manual
curl -X POST http://localhost:8000/api/pwm/test \
  -H 'Content-Type: application/json' \
  -d '{"channel":0,"pwm_us":1520,"duration_ms":500}'
curl -X POST http://localhost:8000/api/pwm/neutral
curl -X POST http://localhost:8000/api/disarm
curl -X POST http://localhost:8000/api/emergency-stop
curl -X POST http://localhost:8000/api/reset-estop
```

## 6. 电机/推进器调试流程

本节讲的是如何用当前 Web 控制台安全地调试 8 路电机/推进器。下面的“电机”也包括推进器、电调和对应的 PCA9685 PWM 通道。

当前软件限制如下：

- 只有进入 `手动测试` / `MANUAL_TEST` 状态后，Web 页面才允许单通道 PWM 输出。
- Web 后端只接受 `1450-1550us` 的测试 PWM。
- Web 后端只接受 `200-2000ms` 的测试时长。
- 后端限制两次 PWM 测试至少间隔 `300ms`。
- STM32 端一次只允许一个通道非中位，其余通道强制 `1500us`。
- 换通道前，Web 后端会先请求 `SET_ALL_NEUTRAL`，让所有通道回中。
- 通信丢失、急停、未解锁、故障等状态下，STM32 会强制所有 PWM 回到 `1500us`。

### 6.1 调试总原则

按这个顺序调试，不要跳步骤：

1. 不接推进器电源，只启动 STM32、香橙派和 Web 页面。
2. 不接推进器电源，用示波器或逻辑分析仪确认 PCA9685 每路 PWM。
3. 只接一个电调/电机，不装桨或不让推进器产生危险推力，做短脉冲测试。
4. 一个通道确认完，再换下一个通道。
5. 8 个通道全部确认后，记录通道和实物位置映射。
6. 再做方向确认：`1520us` 和 `1480us` 分别对应哪个推力方向。
7. 最后才做组合动作、定深、航向和下水测试。

任何阶段出现异常，优先级从高到低：

1. 点击 Web 页面右上角 `急停`。
2. 点击 `全部回中`。
3. 点击 `上锁并回中`。
4. 断开推进器动力电源。
5. 必要时断开整机电源。

不要在推进器附近用手感受推力方向。方向确认应通过水流方向、固定架受力方向、轻量标记物或安全距离观察完成。

### 6.2 当前代码里的通道约定

PCA9685 当前使用前 8 路输出：

```text
CH0 CH1 CH2 CH3 CH4 CH5 CH6 CH7
```

代码中电机分组位于 `Modules/MotorControl/src/motor_control.cpp`：

```cpp
const uint8_t inid[4]  = {1, 2, 6, 5};
const uint8_t outid[4] = {0, 3, 7, 4};
const int8_t  sign[8]  = {1, -1, 1, 1, -1, -1, 1, -1};
```

含义：

- 垂直/深度/姿态相关电机组：`CH1, CH2, CH6, CH5`
- 水平/前后/左右/旋转相关电机组：`CH0, CH3, CH7, CH4`
- `sign[]` 是软件里当前使用的方向符号，用于把控制量换算成各通道 PWM。

注意：这只是代码约定，不等于实物一定已经按这个位置接好。电机调试的核心任务就是确认：

- PCA9685 的 `CHx` 实际接到了哪个电调。
- 这个电调对应船体哪个推进器。
- `1520us` 时推进器实际推力方向。
- `1480us` 时推进器实际推力方向。
- 软件里的 `sign[]` 是否和实物方向一致。

### 6.3 调试前硬件检查

上电前逐项检查：

- STM32、PCA9685、香橙派、电调、动力电源共地。
- PCA9685 的 VCC、GND、SCL、SDA 接线正确。
- PCA9685 地址和代码一致，当前代码使用 `PCA9685_ADDR = 0x80`，即 7-bit 地址 `0x40` 左移 1 位后的 HAL 地址。
- PCA9685 当前经 TCA9548A 的 `channel 4` 访问。
- 电调信号线接到 PCA9685 的 `CH0-CH7`，不要接到 `CH8-CH15`。
- 推进器固定牢靠，测试时不会移动、撞击、缠线。
- 电机附近没有手、线束、螺丝、工具或松散物。
- 动力电源有保险或限流手段，首次测试建议低电压/限流。
- 已知道如何快速断开动力电源。
- Web 页面已能显示 STM32 在线。

建议准备工具：

- 万用表：检查电源和共地。
- 示波器或逻辑分析仪：检查 PWM 脉宽。
- 标签纸或记号笔：标记 `CH0-CH7` 和实物电机位置。
- 记录表：记录通道、方向、异常。
- 绝缘胶带/扎带：固定信号线和电源线。

### 6.4 无动力检查

目的：确认软件链路正常，不让电机真的转。

操作：

1. 不接推进器动力电源。
2. 只给 STM32、香橙派、PCA9685 控制侧供电。
3. 香橙派启动服务：

   ```bash
   ./scripts/start_web_console.sh
   ```

4. 浏览器打开：

   ```text
   http://香橙派IP:8000
   ```

5. 确认页面顶部显示 `WebSocket 已连接`。
6. 确认系统状态中串口正常、STM32 在线。
7. 确认当前状态为 `未解锁`。
8. 确认 PWM 输出区 8 个通道都是 `1500us`。
9. 点击 `解锁`，状态应变为 `已解锁待机`，PWM 仍应全部 `1500us`。
10. 点击 `进入手动测试`，状态应变为 `手动测试`。
11. 点击 `全部回中`，确认 8 个通道仍为 `1500us`。

如果这一步就不正常，不要接动力电源。先解决串口、STM32 在线、PCA9685 初始化或 Web 状态问题。

### 6.5 PWM 信号检查

目的：确认 PCA9685 输出的是正确频率和脉宽。

建议用示波器或逻辑分析仪接到 PCA9685 的信号输出脚和 GND。

检查 `CH0`：

1. Web 页面保持 `手动测试` 状态。
2. 示波器接 `CH0` 信号脚。
3. 页面选择 `CH0`。
4. PWM 设置 `1500us`，点击 `输出 CH0`。
5. 应看到约 50Hz PWM，脉宽约 `1500us`。
6. PWM 设置 `1520us`，持续 `500ms`，点击 `输出 CH0`。
7. 应看到 `CH0` 短时间变为约 `1520us`，随后自动回到 `1500us`。
8. PWM 设置 `1480us`，持续 `500ms`，点击 `输出 CH0`。
9. 应看到 `CH0` 短时间变为约 `1480us`，随后自动回到 `1500us`。
10. 观察其他通道，应保持 `1500us`。

依次检查 `CH1-CH7`。每个通道都要确认：

- 中位是否为 `1500us`。
- `1520us` 是否只影响当前通道。
- `1480us` 是否只影响当前通道。
- 脉冲结束后是否自动回到 `1500us`。

如果某路没有信号：

- 检查示波器地线。
- 检查 PCA9685 对应通道脚位。
- 检查 I2C 是否正常。
- 检查 TCA9548A 通道是否是当前代码里的 `channel 4`。
- 检查 PCA9685 供电。

### 6.6 单电机首次上电测试

目的：一个通道一个通道确认“哪个 CH 控制哪个电机”。

首次测试建议一次只接一个电调/电机的动力侧。如果必须全部接上，至少保证机械固定可靠，并随时准备断电。

推荐步骤：

1. 点击 `全部回中`。
2. 点击 `上锁并回中`。
3. 接入第一个电调/电机的动力电源。
4. 等电调完成上电自检。
5. Web 页面点击 `解锁`。
6. 点击 `进入手动测试`。
7. 选择 `CH0`。
8. 设置 `1510us`、`200ms`。
9. 点击 `输出 CH0`。
10. 观察是否有电机轻微动作。
11. 如果无明显动作，改为 `1520us`、`200ms`。
12. 仍无动作时，再试 `1530us`、`200ms`。
13. 每次测试后确认 PWM 输出回到 `1500us`。
14. 记录哪个实物电机有反应。
15. 点击 `全部回中`。
16. 换 `CH1`，重复测试。
17. 直到找到这个电调/电机对应的通道。

反向测试：

1. 找到对应通道后，设置 `1490us`、`200ms`。
2. 如果动作太小，再试 `1480us`、`200ms`。
3. 记录 `1480/1490us` 时的推力方向。
4. 再试 `1510/1520us`，记录正向推力方向。

不要一开始就用 `1550us` 或 `1450us`。建议从 `1510/1490us` 开始，逐步扩大。

### 6.7 建议测试档位

手动测试范围虽然允许 `1450-1550us`，但现场建议按下面顺序逐步增加：

| 阶段 | PWM | 时长 | 用途 |
| --- | --- | --- | --- |
| 中位 | `1500us` | `500ms` | 确认不会转 |
| 极轻正向 | `1510us` | `200ms` | 判断是否开始响应 |
| 轻正向 | `1520us` | `200ms` 或 `500ms` | 判断正向推力 |
| 中等正向 | `1530us` | `200ms` | 响应不明显时使用 |
| 极轻反向 | `1490us` | `200ms` | 判断反向响应 |
| 轻反向 | `1480us` | `200ms` 或 `500ms` | 判断反向推力 |
| 中等反向 | `1470us` | `200ms` | 响应不明显时使用 |

只有在固定可靠、方向明确、人员远离、动力系统没有异常时，才短时间试：

- `1540us`
- `1550us`
- `1460us`
- `1450us`

如果出现尖叫、抖动、明显过流、线材发热、推进器松动，立即停止，不要继续加大 PWM。

### 6.8 通道映射记录表

建议边测边填表。不要靠记忆。

| 通道 | 实物位置 | 接线标签 | `1520us` 推力方向 | `1480us` 推力方向 | 启转 PWM | 是否正常 | 备注 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| CH0 |  |  |  |  |  |  |  |
| CH1 |  |  |  |  |  |  |  |
| CH2 |  |  |  |  |  |  |  |
| CH3 |  |  |  |  |  |  |  |
| CH4 |  |  |  |  |  |  |  |
| CH5 |  |  |  |  |  |  |  |
| CH6 |  |  |  |  |  |  |  |
| CH7 |  |  |  |  |  |  |  |

实物位置建议统一命名，例如：

- 前左水平
- 前右水平
- 后左水平
- 后右水平
- 前左垂直
- 前右垂直
- 后左垂直
- 后右垂直

也可以按你机器实际结构命名，但必须保持全队一致。

接线标签建议直接贴在电调信号线上，例如：

```text
PCA_CH0
PCA_CH1
...
PCA_CH7
```

### 6.9 方向判断和修正

调试时要区分两个问题：

1. 通道接错：比如 Web 点 `CH0`，实际动的是你以为的 `CH3`。
2. 方向反了：通道对，但 `1520us` 的推力方向和软件期望相反。

通道接错的处理：

- 优先修改接线或重新贴标签，让实物接线和记录表一致。
- 不建议靠脑子记“CH0 实际是 CH3”，后续会非常容易出错。

方向反了的处理有两种：

1. 硬件修正：交换无刷电机任意两根三相线，电机旋转方向会反过来。
2. 软件修正：修改 `Modules/MotorControl/src/motor_control.cpp` 里的 `sign[8]`。

软件方向符号当前是：

```cpp
const int8_t sign[8] = {1, -1, 1, 1, -1, -1, 1, -1};
```

例如 `CH4` 方向需要反过来，就把第 4 个索引对应的符号改反。数组下标从 0 开始：

```text
CH0 -> sign[0]
CH1 -> sign[1]
CH2 -> sign[2]
CH3 -> sign[3]
CH4 -> sign[4]
CH5 -> sign[5]
CH6 -> sign[6]
CH7 -> sign[7]
```

改软件方向后必须重新构建、烧录并重新做单通道测试。

不要同时改硬件相线和软件 `sign[]`，否则容易改了两次又抵消。推荐原则：

- 如果只是单个电机旋转方向和预期相反，优先交换电机三相线。
- 如果整套布置和软件坐标系相反，优先统一修改 `sign[]` 并记录原因。
- 每次只改一个变量，改完立即复测。

### 6.10 验证垂直电机组

当前代码认为垂直/姿态相关通道是：

```text
CH1, CH2, CH6, CH5
```

建议调试目标：

- 这 4 个通道应对应负责上浮/下潜/横滚/俯仰的推进器。
- `1520us` 和 `1480us` 的推力方向要记录清楚。
- 四个垂直推进器在同一种“上浮”指令下，实际推力方向应一致。
- 用于横滚/俯仰修正时，对角或左右/前后方向应符合船体坐标定义。

验证建议：

1. 在 Web 手动测试中分别测试 `CH1, CH2, CH6, CH5`。
2. 每个通道只用 `1520us` / `1480us`、`200-500ms`。
3. 记录每个通道是“向上推水”还是“向下推水”，或按你的机械定义记录“产生上浮力/下潜力”。
4. 对照结构图确认四个垂直推进器的位置。
5. 如果某个通道明显属于水平推进器，说明接线或通道映射错了。

不要在空气中长时间测试水下推进器。很多水下推进器依赖水冷/水润滑，空气中长时间运行会发热或磨损。

### 6.11 验证水平电机组

当前代码认为水平运动相关通道是：

```text
CH0, CH3, CH7, CH4
```

这些通道参与：

- 前进 `W`
- 后退 `S`
- 左移 `A`
- 右移 `D`
- 顺时针旋转 `E`
- 逆时针旋转 `Q`

建议调试目标：

- `CH0, CH3, CH7, CH4` 应都是水平推进器。
- 每个水平推进器的 `1520us` / `1480us` 推力方向要记录。
- 组合运动前，必须先完成单通道方向确认。

验证建议：

1. 在 Web 手动测试中分别测试 `CH0, CH3, CH7, CH4`。
2. 每个通道只用 `1520us` / `1480us`、`200-500ms`。
3. 记录推力方向，例如“让船体向前/向后/向左/向右/产生顺时针力矩”。
4. 对照船体结构，判断当前 `sign[]` 是否合理。
5. 单通道全部确认后，再考虑用文本命令或二进制协议测试组合动作。

### 6.12 组合动作验证

只有完成 8 个通道的单通道测试和方向记录后，才做组合动作验证。

组合动作可以通过 legacy 文本命令测试：

| 命令 | 目标动作 |
| --- | --- |
| `W` | 前进 |
| `S` | 后退 |
| `A` | 左移 |
| `D` | 右移 |
| `E` | 顺时针转向 |
| `Q` | 逆时针转向 |
| `ON` | 开启定深 |
| `OFF` / `NEU` | 关闭控制并回中 |

组合动作验证建议：

1. 确保机器固定或处于安全水中环境。
2. 先确认 `OFF` / `NEU` 能让所有通道回中。
3. 确认 `PING` 能返回 `PONG`。
4. 短时间发送 `W`，观察水平推进器组合是否产生前进趋势。
5. 发送 `OFF` / `NEU` 回中。
6. 再依次测试 `S/A/D/E/Q`。
7. 每个动作只做短时间，确认方向后立刻回中。

如果组合动作方向不对，不要直接改 PID 参数。先回到单通道表，检查：

- 水平推进器是否接到了 `CH0, CH3, CH7, CH4`。
- 某个推进器方向是否反了。
- `sign[]` 是否和实际方向匹配。
- 船体坐标定义是否和软件一致。

### 6.13 电调校准注意事项

不同电调的校准方式不同，不要在不清楚电调手册的情况下随意做最大/最小油门校准。

当前系统的安全测试范围是 `1450-1550us`，不是完整油门行程。代码里的硬件绝对范围是：

```text
1300-1700us
```

但 Web 手动测试不会让你直接输出 `1300us` 或 `1700us`。这是故意的安全限制。

如果某个电调需要校准：

1. 先查该电调型号的官方校准流程。
2. 明确最大油门、最小油门、中位的要求。
3. 拆掉桨或断开推进器机械负载。
4. 使用专门校准流程，不要混在普通 Web 手动测试流程里。
5. 校准完成后重新确认 `1500us` 中位不会转。

如果你只是要确认通道和方向，不需要做电调校准。

### 6.14 异常现象排查

`点击输出 CHx 后没有任何反应`

- 确认状态是 `手动测试`。
- 确认 Web 页面没有提示链路过期。
- 确认对应电调有动力电源。
- 确认电调信号线接在 PCA9685 对应通道。
- 确认 PCA9685 和电调共地。
- 先用示波器确认该 CH 是否有 `1520us` 脉冲。
- 试 `1530us`、`200ms`，有些电调死区较大。

`页面提示 Must be in MANUAL_TEST`

- 先点击 `解锁`。
- 再点击 `进入手动测试`。
- 只有状态显示 `手动测试` 才能输出 PWM。

`页面提示 PWM 范围错误`

- Web 只允许 `1450-1550us`。
- 不要用 `1300us` 或 `1700us` 做手动测试。

`页面提示 Rate limited`

- 两次 PWM 测试间隔太短。
- 等 `300ms` 以上再点。

`点 CH0，但另一个电机动`

- 接线或标签和记录不一致。
- 用记录表重新确认每个 PCA9685 通道对应哪个电调。
- 优先改标签或接线，不要靠记忆。

`一次有多个电机动`

- 先点 `全部回中`。
- 检查 Web 是否仍在 `手动测试`。
- 检查电调信号线是否短接、插错、共地异常。
- 用示波器确认是不是多个 PCA9685 通道真的输出了非 `1500us`。
- 如果 Web 显示只有一个通道非中位，但多个电机动，优先查硬件接线。

`电机方向反了`

- 如果通道正确但方向相反，交换该电机任意两根三相线。
- 或修改 `sign[]` 对应通道的符号。
- 不要同时改硬件和软件。

`电机抖动、尖叫、不启动`

- 可能 PWM 变化太小，先从 `1510/1490us` 增加到 `1520/1480us`，再到 `1530/1470us`。
- 可能电调未完成上电自检。
- 可能电调中位不认 `1500us`，需要查电调说明。
- 可能电源限流、压降或共地不良。
- 可能推进器卡住或机械阻力过大。

`输出后没有自动回中`

- 立即点 `全部回中` 或 `急停`。
- 查看 Web PWM 输出区是否仍显示非 `1500us`。
- 查看 `logs/opi_console.log` 是否有 ACK 超时或串口断开。
- 如果 Web 显示已回中但示波器没回中，查 PCA9685/I2C/供电。

`急停后无法重新解锁`

- 急停是锁定状态。
- 先确认动力系统安全。
- 点击 `解除急停`。
- 状态回到 `未解锁` 后，再点击 `解锁`。

### 6.15 推荐最终验收表

完成电机调试后，至少确认这些项目：

- [ ] 上电后 8 路 PWM 全部为 `1500us`。
- [ ] `未解锁` 状态下无法输出 PWM。
- [ ] `已解锁待机` 状态下 PWM 仍全部 `1500us`。
- [ ] 只有 `手动测试` 状态能输出单通道 PWM。
- [ ] `CH0-CH7` 每个通道都能被单独识别。
- [ ] 每次只会有一个通道非中位。
- [ ] 每次测试结束后自动回到 `1500us`。
- [ ] 点击 `全部回中` 后 8 路全为 `1500us`。
- [ ] 点击 `急停` 后 8 路全为 `1500us`，且无法继续输出。
- [ ] 串口断开后系统进入安全状态或控制被禁用。
- [ ] 已填写完整通道映射记录表。
- [ ] 已确认 `1520us` / `1480us` 对每个推进器的实际推力方向。
- [ ] 已确认垂直组 `CH1, CH2, CH6, CH5` 对应实物垂直推进器。
- [ ] 已确认水平组 `CH0, CH3, CH7, CH4` 对应实物水平推进器。
- [ ] 已记录所有异常、修正和最终接线。

### 6.16 建议保存的调试记录

建议在 `docs/` 或现场记录本中保存以下内容：

```text
日期：
固件版本/提交：
前端版本/提交：
测试人员：
电池/电源电压：
PCA9685 板号：
电调型号：
推进器型号：

通道映射：
CH0 =
CH1 =
CH2 =
CH3 =
CH4 =
CH5 =
CH6 =
CH7 =

方向结论：
CH0: 1520us ->      1480us ->
CH1: 1520us ->      1480us ->
CH2: 1520us ->      1480us ->
CH3: 1520us ->      1480us ->
CH4: 1520us ->      1480us ->
CH5: 1520us ->      1480us ->
CH6: 1520us ->      1480us ->
CH7: 1520us ->      1480us ->

修改记录：
- 是否交换过三相线：
- 是否修改过 sign[]：
- 是否修改过通道接线：
- 是否发现异常：
```

这份记录比代码注释更重要。现场接线一旦变化，先更新记录，再重新测试。

## 7. 前端开发模式

后端会在存在 `web_frontend/dist` 时提供静态页面。开发前端时可单独启动 Vite：

```bash
cd web_frontend
npm ci
npm run dev
```

当前 Vite 工具链要求 Node.js `20.19+`。开发服务通过代理访问后端，因此默认
不开放跨域来源；如确需独立来源，显式配置 `web.cors_origins`。

构建前端产物：

```bash
cd web_frontend
npm run build
```

快速前端门禁：

```bash
npm test
npm run typecheck
npm run lint
npm audit --audit-level=high
```

## 8. 串口文本协议

USART6 同时保留 legacy 文本命令，串口参数为 `115200 8N1`。每条命令发送 ASCII 文本，建议以换行结尾。

常用命令：

| 命令 | 作用 |
| --- | --- |
| `PING` | 返回 `PONG`，用于连通性检查 |
| `ON` | 开启定深控制，目标深度取当前深度或至少 `30cm` |
| `OFF` / `NEU` | 关闭控制，所有手动 PWM 回中位 |
| `TES:1500,1500,...` | 设置 8 路手动 PWM，最多 8 个逗号分隔值 |
| `UP` | 目标深度减小 `1cm` |
| `DN` | 目标深度增加 `1cm` |
| `H:<cm>` | 设置目标深度，单位 cm |
| `ANG:<deg>` | 设置目标航向角，单位度，并开启角度控制 |
| `ACL ON` / `ACL OFF` | 开关角度闭环 |
| `W` / `S` / `A` / `D` | 前、后、左、右运动 |
| `E` / `Q` | 顺时针、逆时针转向 |

文本协议主要用于调试和兜底；正式上位机推荐使用二进制协议。

## 9. 二进制协议

二进制协议用于 Orange Pi 和 STM32 的主通信链路。共享定义位于：

- `protocol/shared/protocol.h`
- `protocol/shared/protocol.py`

帧格式：

```text
[0xAA 0x55] [version:1] [type:1] [seq:2] [payload_len:2] [payload:N] [crc16:2]
```

常用消息：

- `ARM` / `DISARM`
- `EMERGENCY_STOP` / `RESET_ESTOP`
- `ENTER_MANUAL` / `EXIT_MANUAL`
- `SET_PWM`
- `SET_ALL_NEUTRAL`
- `FLOAT_ON` / `FLOAT_OFF`
- `ANGLE_ON` / `ANGLE_OFF`
- `SET_DEPTH` / `SET_YAW` / `SET_MOTION`
- `REQUEST_STATUS` / `REQUEST_SENSORS`
- `HEARTBEAT`

Python 侧可用 `protocol/shared/protocol.py` 的 `encode_frame()`、`decode_frame()` 和 payload dataclass 生成或解析帧。

## 10. 测试验证

固件构建验证：

```bash
cmake --build /home/zoae/stm32_projects/1132_bot/build/Debug --
```

完整自动化门禁：

```bash
python3 -m pip install -r requirements-dev.txt
./scripts/run_tests.sh
```

包含真实 Chrome 的模拟模式浏览器端到端测试：

```bash
RUN_BROWSER_E2E=1 ./scripts/run_tests.sh
```

若 Chrome/Chromium 不在常见路径，设置：

```bash
ROV_CHROME_PATH=/path/to/chrome npm --prefix web_frontend run test:e2e
```

协议与后端单独测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  -o cache_dir=/tmp/pytest_cache_1132 tests -q
./tests/run_stm32_protocol_host_tests.sh
```

Web/仿真快速验证：

```bash
./scripts/start_web_console.sh --simulate
curl http://localhost:8000/api/status
curl http://localhost:8000/api/capabilities
curl http://localhost:8000/api/diagnostics
curl -X POST http://localhost:8000/api/arm
curl -X POST http://localhost:8000/api/enter-manual
curl -X POST http://localhost:8000/api/pwm/test \
  -H 'Content-Type: application/json' \
  -d '{"channel":0,"pwm_us":1520,"duration_ms":500}'
curl -X POST http://localhost:8000/api/emergency-stop
```

## 11. 常见问题

`STM32 is offline or telemetry is stale`

- 检查 STM32 已烧录并运行。
- 检查 Orange Pi 串口设备名是否正确。
- 检查 USART6 接线和地线。
- 检查波特率是否为 `115200`。
- 先用 `./scripts/start_web_console.sh --simulate` 验证网页本身没问题。

香橙派页面打不开

- 确认脚本还在运行，没有退出。
- 在香橙派执行 `hostname -I`，确认访问的是正确 IP。
- 确认电脑和香橙派在同一个局域网。
- 确认端口是启动时显示的端口，默认 `8000`。
- 如果改成 `--port 9000`，浏览器也要访问 `http://香橙派IP:9000`。

`pip: 未找到命令` 或 `ensurepip returned non-zero exit status 1`

- Arch Linux / Orange Pi 上先安装系统包：`sudo pacman -Syu --needed python-pip python-virtualenv python-platformdirs`。
- 重新创建虚拟环境：`rm -rf venv && python3 -m virtualenv venv`。
- 激活后安装依赖：`source venv/bin/activate && python3 -m pip install -r requirements.txt`。
- 详细步骤见 `4.2 安装 Python 依赖`。

`GLIBC_2.42 not found` 或 Python 串口模块加载失败

- 这是 Python 包和系统 `glibc` 版本不匹配。
- 先查版本：`python3 --version && ldd --version | head -n 1 && pacman -Q python glibc expat`。
- 在供电和网络稳定时升级系统库：`sudo pacman -Syu --needed glibc expat`。
- 升级后重新创建 `venv` 并安装依赖。
- 修好后真实模式日志应出现：`Connected to /dev/ttyS5 at 115200 baud`。

串口打不开或 Permission denied

- 检查串口设备是否存在：`ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null`。
- 检查权限：`ls -l /dev/ttyS5`。
- 把用户加入串口组后重新登录：`sudo usermod -aG dialout $USER`。
- 临时调试可用：`sudo ./scripts/start_web_console.sh`。

`SET_PWM failed` 或 Web PWM 测试被拒绝

- 必须先 `ARM`，再进入 `MANUAL_TEST`。
- PWM 只能在 `1450-1550us`。
- 测试持续时间只能在 `200-2000ms`。
- 急停状态下需要先 `RESET ESTOP`。

构建报找不到 ARM 工具链

- 确认已安装 `arm-none-eabi-gcc` / `arm-none-eabi-g++`。
- 确认 `cmake/gcc-arm-none-eabi.cmake` 中的工具链路径匹配当前系统。

Web 页面打不开

- 确认 `./scripts/start_web_console.sh` 没有退出。
- 确认访问的是 Orange Pi 的实际 IP 和端口。
- 如果前端静态文件不存在，先在 `web_frontend` 执行 `npm run build`。
