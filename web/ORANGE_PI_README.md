# 1132_bot 香橙派运行包

真实硬件启动：

```bash
cd /home/orangepi/1132_bot
./scripts/start_web_console.sh
```

只看网页/仿真模式：

```bash
cd /home/orangepi/1132_bot
./scripts/start_web_console.sh --simulate
```

浏览器访问：

```text
http://香橙派IP:8000
```

“运动调参”页面依赖当前版本 STM32 六轴协议。部署本运行包后，还必须烧录配套
STM32 固件；页面显示“参数已由 STM32 回读确认”后才能进入六轴控制。参数保存在
`config/motion_tuning.json`，服务重启后会自动重新加载和同步。

Linux USB 手柄通过下面的端点连接：

```text
ws://香橙派IP:8000/ws/control/gamepad
```

手柄转发程序在操作电脑运行，不在香橙派运行。香橙派负责唯一控制租约、
ControlArbiter 模式互斥、300ms 归零、1000ms 退出上锁，并复用
`SET_BODY_COMMAND` 六轴链路。状态接口：

```bash
curl http://127.0.0.1:8000/api/gamepad/status
```

映射、死区、expo、倍率和方向反转配置位于 `opi_console/config.yaml` 的
`gamepad` 节。修改后必须重启服务。

首次部署必须先准备前端生产文件。构建机需要 Node.js `20.19+`：

```bash
cd web_frontend
npm ci
npm run build
cd ..
```

健康和诊断接口：

```bash
curl http://127.0.0.1:8000/health
curl http://127.0.0.1:8000/api/diagnostics
```

详细说明见：

```text
docs/USAGE.md
```
