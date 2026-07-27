# Linux USB 手柄转发器

本程序在操作电脑运行，数据路径为：

```text
USB 手柄
  -> 本程序
  -> WebSocket / 网口 / 电力载波
  -> 香橙派 FastAPI
  -> ControlArbiter
  -> STM32 SET_BODY_COMMAND
  -> MotorControl 六轴混控
  -> 8 路推进器
```

本程序不会发送 CH0-CH7 PWM，不会 ARM，不会解除 ESTOP，也不包含第二套
推进器混控。

## 安装

Ubuntu/Debian：

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-tk

cd gamepad_forwarder_linux
chmod +x run.sh
./run.sh
```

`run.sh` 首次运行会创建 `.venv` 并安装：

- `pygame`
- `websocket-client`

手柄权限不足时：

```bash
sudo usermod -aG input "$USER"
```

注销并重新登录后检查：

```bash
ls -l /dev/input/js* /dev/input/event* 2>/dev/null
```

## 连接

1. 香橙派先启动 `./scripts/start_web_console.sh`。
2. 本程序填写香橙派 IP，默认端口 `8000`。
3. WebSocket 路径保持 `/ws/control/gamepad`。
4. 发送频率建议 `30-50Hz`，默认 `50Hz`。
5. 点击“启动连接”。
6. 确认 USB 手柄和香橙派都显示已连接。
7. 观察原始 axis0、axis1、axis4，核对真实正负方向。
8. 所有摇杆回中并松开 A/Y。
9. 打开“控制开启”。
10. 在 Web 页面手动 ARM，然后进入“手柄控制”页启用 GAMEPAD。

网络设置保存在：

```text
~/.config/gamepad-forwarder/settings.json
```

## 映射

| 输入 | BodyCommand | FRD 正方向 |
| --- | --- | --- |
| axis 1 左杆上下 | surge | 前推为正 |
| axis 0 左杆左右 | sway | 右推为正 |
| axis 4 右杆左右 | yaw | 右推为正，俯视顺时针 |
| button 3，Y | heave | 负，上浮 |
| button 0，A | heave | 正，下潜 |
| axis 3 | 未使用 | 始终为零 |
| button 1，B | 保留 | 不影响运动 |
| button 2，X | 保留 | 不影响运动 |

Y 和 A 同时按下时 heave 强制为零并显示冲突。roll 和 pitch 第一版始终为
零。

香橙派 `opi_console/config.yaml` 是映射的权威来源。WebSocket 建立后，程序
读取服务端下发的 deadzone、expo、倍率和 invert 配置，并用同样的配置生成
`mapped_command`。香橙派会独立重算并拒绝不一致的结果。

默认方向设置：

```yaml
surge_invert: true
sway_invert: false
yaw_invert: false
```

它对应 axis1 前推为负、axis0 右推为正、axis4 右推为正。不同手柄或驱动可能
不同，必须先看界面的原始轴值，再修改香橙派配置并重启后端。

## 界面状态

程序显示：

- 手柄连接状态和名称。
- 香橙派连接状态。
- 本地控制开关和服务端 ControlArbiter 模式。
- 原始 axes、完整 buttons、按下按钮和 hats。
- 映射后的 surge/sway/heave/roll/pitch/yaw。
- A/Y 冲突。
- 实际发送频率和 sequence。
- 最近 ACK、服务端接受/拒绝原因和 RTT。
- 最近连接、字段布局或协议错误。

手柄拔出、网络发送异常或字段布局不是 6 轴/至少 4 按钮时，本地“控制开启”
会自动关闭。

## 安全行为

- 同时只允许一个转发器持有 `/ws/control/gamepad` 租约。
- 香橙派只保留最新一帧，不排队重放。
- sequence 重复或倒退会被拒绝。
- 超过 300ms 无新帧，香橙派发送六轴全零。
- 300ms 超时后，即使网络恢复也必须先回中，旧非零输入不会恢复。
- 超过 1000ms 无新帧，香橙派退出 GAMEPAD 并 DISARM。
- 关闭程序、关闭本地控制、拔手柄或 WebSocket 断开会立即触发安全退出。
- 重新连接不会自动 ARM，也不会自动重新进入 GAMEPAD。

## 自动测试

在该目录执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=. \
python3 -m pytest -q test_gamepad_mapping.py
```

测试覆盖 FRD 方向、Y/A、按键冲突、未使用输入、死区连续性、NaN/Inf 和三个
invert 配置。测试不驱动真实推进器。

完整项目手柄使用、超时和拆桨低功率验证步骤见：

```text
docs/USAGE.md 的 5.8 节
```
