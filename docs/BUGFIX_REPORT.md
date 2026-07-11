# Web 上位机稳定性修复报告

日期：2026-07-11。本文记录本轮可复现问题、根因、修复和验证边界。
真实 STM32、PCA9685、电调和推进器未连接，因此所有“需硬件验证”项目仍须按
[HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md) 执行，不能视为硬件通过。

## 修复前系统与运行结果

数据流：React 页面通过 REST 下发命令、通过 WebSocket 接收 5Hz 遥测；FastAPI
调用 `Stm32Proxy`，再由单一 `SerialTransport` 使用 UART 115200 发送协议 v2
帧；STM32 协议处理器更新安全状态和手动 PWM，20ms 控制任务把值交给
PCA9685。仿真模式只替换 UART 对端，Web、API、代理和协议帧路径不变。

模块职责：`web_frontend` 负责显示、输入和本地 fail-closed；`web_backend`
负责 API、全局控制锁、WebSocket 和持久化；`opi_console` 负责配置、生命周期、
串口、确认状态和模拟器；`Modules/Protocol` 是 STM32 最终协议安全门；
MotorControl/PCA9685 保持原有输出职责，本轮没有重写其输出逻辑。

修复前复现结果：Python 基线 68 项通过，前端 23 项通过，前端生产构建通过，
STM32 Debug 构建通过；但缺少 `typecheck`/`lint` 脚本，协议主机测试脚本无执行
权限。实际 ASGI 运行中 `/health`、`/api/capabilities`、SPA 子路径为 404，传感器
轮询未随应用启动。真实硬件模式因当前环境没有 STM32 串口而未执行电气验证。

## Bug 清单

### WEB-P0-01 最后一个浏览器断开后后端继续保持 MANUAL/ARM

- 严重度：P0；复现：MANUAL 下输出 CH0 后关闭唯一 WebSocket。
- 预期：立即确认回中并 DISARM；实际：仅移除连接，Orange Pi 心跳继续维持状态。
- 涉及：`web_backend/ws_manager.py`、`control_state.py`、`app.py`。
- 根因：连接计数没有绑定安全转换，关闭处理也没有共享控制锁。
- 修复：最后客户端后台串行执行统一 `force_neutral()`、按状态退出 MANUAL、无论前步结果都尝试 DISARM；新连接不取消转换。
- 硬件验证：是，需关闭浏览器并用示波器确认 8 路 1500us。

### WEB-P0-02 HTTP 取消、SET_PWM 与 DISARM/ESTOP 并发可留下不确定输出

- 严重度：P0；复现：延迟 ACK 后取消 HTTP，或 SET_PWM 处理中同时急停。
- 预期：安全命令优先且任何不确定路径回中；实际：取消异常可跳过回中，前端普通请求还会禁用急停。
- 涉及：`api_routes.py`、`serial_transport.py`、`App.tsx`。
- 根因：仅用 React state 防重入，ACK future 取消未清理，ESTOP 和普通命令共用页面 busy 门。
- 修复：引用级 single-flight、独立 ESTOP 通道、取消 pending ACK、取消/异常路径加运动转换锁并确认回中；后端二次检查安全转换。
- 硬件验证：是，需在延迟/丢 ACK 条件下测最终中位。

### WEB-P0-03 急停未确认后后端仍可能依据旧 MANUAL 缓存放行

- 严重度：P0；复现：让三次 ESTOP ACK 全部丢失后再次 ARM/PWM。
- 预期：后端 fail-closed；实际：旧缓存短期仍可能是 MANUAL。
- 涉及：`control_state.py`、`api_routes.py`、`ws_manager.py`、`status.ts`、`App.tsx`。
- 根因：没有“安全命令结果不确定”的持久运动锁存。
- 修复：ESTOP 发起即锁存运动；失败后回中并继续 DISARM，未确认时保持锁存并同步到 REST/WS/UI；DISARM/RESET 确认后才清除。
- 硬件验证：是，需真实断开 TX/RX 分别验证。

### WEB-P0-04 模拟器 DISARM 会错误清除 ESTOP

- 严重度：P0；复现：仿真 ESTOP 后直接 DISARM。
- 预期：仍为 EMERGENCY_STOP；实际：模拟器回到 DISARMED，和固件相反。
- 涉及：`simulated_stm32.py`、`stm32_protocol_handler_host_test.cpp`。
- 根因：模拟状态机没有镜像固件的 `estop_locked` 分支。
- 修复：DISARM 只回中，不解除锁存；增加固件主机测试验证 ARM NACK 和 RESET_ESTOP 唯一恢复路径。
- 硬件验证：是，确认真实固件状态报告与主机测试一致。

### WEB-P0-05 两个后端进程可绕过进程内全局控制锁

- 严重度：P0；复现：重复执行启动脚本。
- 预期：第二实例在串口前退出；实际：可能同时打开串口，最后才遇到 Web 端口冲突。
- 涉及：`config.py`、`config.yaml`、`main.py`、`start_web_console.sh`。
- 根因：控制锁只存在于单个 Python 进程。
- 修复：增加内核 `flock` 单实例锁，错误包含占用 PID 和锁文件；异常退出自动释放。
- 硬件验证：否；实际脚本重复启动已验证。

### WEB-P0-06 通道切换在模拟成功、真实固件 NACK

- 严重度：P0；复现：CH0 输出后切到 CH1。
- 预期：CH0 回中、重新进入 MANUAL、只输出 CH1；实际：固件 `SET_ALL_NEUTRAL` 会回到 ARMED_IDLE，旧模拟器不会，后续 SET_PWM 在真机被拒绝。
- 涉及：`simulated_stm32.py`、`api_routes.py`、协议处理器主机测试。
- 根因：模拟器遗漏固件状态转换，后端假设回中不退出 MANUAL。
- 修复：模拟器对齐固件；后端确认回中状态，必要时重新 ENTER_MANUAL，失败则 DISARM。
- 硬件验证：是，逐通道切换并测其他 7 路中位。

### WEB-P0-07 重连后旧状态可被心跳重新标为新鲜，sequence 也被复用

- 严重度：P0；复现：状态刚更新后拔插串口并快速收到心跳。
- 预期：必须等新连接的 STATUS_REPORT；实际：断线前缓存仍在新鲜窗口，主机 sequence 又从 0 开始。
- 涉及：`serial_transport.py`、`stm32_proxy.py`、`simulated_stm32.py`。
- 根因：没有连接 generation，连接重置混合了时间戳清理和命令 sequence 清零。
- 修复：每次连接递增 generation 并使状态/传感器过期；清除旧输入和模拟队列；命令 sequence 跨重连连续，断线取消旧回调。
- 硬件验证：是，至少连续拔插 5 次并观察 stale/online。

### WEB-P0-08 ACK 到达时真实控制任务尚未应用 PWM

- 严重度：P0；复现：真机 ACK 后立即 REQUEST_STATUS。
- 预期：等到 STM32 报告请求值；实际：20ms controlTask 尚未复制，单次状态可能仍为 1500，模拟器无法复现。
- 涉及：`stm32_proxy.py`、`config.py`、`config.yaml`、相关回归测试。
- 根因：把一次即时状态采样当成最终确认。
- 修复：在短脉冲有效期内按 controlTask 的 20ms 配置节拍做有截止时间的条件轮询；从未匹配则失败并回中。
- 硬件验证：是，重点验证 200ms 最短脉冲。

### WEB-P1-01 静态根挂载遮蔽健康/API，SPA 子路径 404

- 严重度：P1；复现：GET `/health`、`/api/capabilities`、`/dashboard`。
- 预期：JSON/JSON/index；实际：404 或被静态路由处理。
- 涉及：`web_backend/app.py`。
- 根因：`/` StaticFiles 注册顺序过早且无 SPA fallback。
- 修复：API/WS/health 先注册，静态最后；仅无扩展名路径回退 index，缺失资源保持 404，未知 API 保持 JSON 404。
- 硬件验证：否。

### WEB-P1-02 传感器轮询任务未随应用生命周期运行

- 严重度：P1；复现：启动模拟 Web 后 GET `/api/sensors`。
- 预期：持续更新；实际：默认零值/过期。
- 涉及：`stm32_proxy.py`、`app.py`。
- 根因：代理已有请求方法但没有唯一、可关闭的后台轮询生命周期。
- 修复：FastAPI lifespan 幂等启动/取消一个轮询任务；离线不排队，异常自恢复，重连恢复请求。
- 硬件验证：是，确认真实传感器刷新率和串口负载。

### WEB-P1-03 启动配置被脚本覆盖，simulation.enabled 无效，异常路径漏清理

- 严重度：P1；复现：修改 YAML 后用脚本启动，或让 Web 启动抛错。
- 预期：CLI/env/YAML 明确优先且总是关闭资源；实际：脚本固定值覆盖，配置模式未使用，异常可跳过清理。
- 涉及：`config.py`、`main.py`、`logger.py`、`start_web_console.sh`。
- 根因：配置在多处以 dict 默认值读取，清理不在外层 finally。
- 修复：严格 Pydantic 配置、未知键拒绝、统一覆盖后复验；绝对路径；启动/关闭均确认回中再 DISARM；缺依赖/静态文件清晰失败。
- 硬件验证：是，验证实际 `/dev/ttyS5` 权限和热插拔。

### WEB-P1-04 WebSocket 慢客户端阻塞全体，缺少稳定会话标识

- 严重度：P1；复现：一个客户端停止读取，或后端重启后 sequence 回到 1。
- 预期：慢客户端被独立清理，前端识别新会话；实际：串行 send 阻塞，旧 sequence 可压掉新数据。
- 涉及：`ws_manager.py`、`wsProtocol.ts`、`App.tsx`。
- 根因：广播逐客户端无超时，只有 sequence 没有进程 session。
- 修复：并发发送、配置超时、发送锁、死连接清理；统一 type/session_id/sequence/timestamp/payload，前端按会话和无符号环回排序。
- 硬件验证：否。

### WEB-P1-05 API 缺少统一能力契约、严格校验和正确失败状态

- 严重度：P1；复现：字符串/布尔 PWM 参数、重复 ARM、离线/NACK/ACK timeout。
- 预期：422/409/503/504 可区分；实际：配置分叉、类型被强制转换或命令只看 ACK。
- 涉及：`api_routes.py`、`capabilities.ts`、`App.tsx`。
- 根因：无 `/api/capabilities`，请求模型宽松，状态命令未等待确认报告。
- 修复：能力接口从同一验证配置生成；请求 strict/extra-forbid；状态命令 ACK 后确认状态，未确认 ARM/MANUAL 自动回滚 DISARM。
- 硬件验证：是，检查真实 ACK/NACK reason 和状态报告。

### WEB-P1-06 并发状态请求会消费同一报告并交叉确认

- 严重度：P1；复现：SET_PWM 等状态时并发 ESTOP。
- 预期：PWM 被安全转换覆盖；实际：两个 waiter 可能用同一旧报告各自成功。
- 涉及：`stm32_proxy.py`、`api_routes.py`。
- 根因：STATUS_REPORT 没有请求关联字段，代理也没有序列化状态请求。
- 修复：单一 status request lock；PWM 返回前再检查 ESTOP、断开转换、运动锁和最终安全状态。
- 硬件验证：是，注入 60-100ms ACK 延迟并发验证。

### WEB-P1-07 前端请求可永久 pending 或同一渲染周期重复提交

- 严重度：P1；复现：同步触发两次 click，或 fetch 不返回。
- 预期：只发一次且超时恢复；实际：React state 更新前可重复，按钮可能永久 busy。
- 涉及：`App.tsx`、`uiState.ts`、浏览器 E2E。
- 根因：只依赖异步 state，没有 ref gate/AbortController 超时。
- 修复：普通命令和 PWM 分别 single-flight，6s HTTP 超时，卸载/断线清理；ESTOP 独立且完成后必须等更新遥测才解锁。
- 硬件验证：否。

### WEB-P2-01 NaN/Inf 传感器破坏 REST/WS JSON

- 严重度：P2；复现：缓存写入 NaN/Inf 后读取传感器。
- 预期：页面显示无数据；实际：REST 可 500，WebSocket 产生浏览器无法解析的 NaN token。
- 涉及：`stm32_proxy.py`、`ws_manager.py`、`status.ts`。
- 根因：Python JSON 默认允许非标准 NaN，边界未清洗。
- 修复：仅 JSON 快照把非有限数转为 null，并强制 `allow_nan=False`；前端显示 `--`。
- 硬件验证：否，可通过故障帧复核。

### WEB-P2-02 电机映射损坏文件可返回危险中位/范围，保存非原子

- 严重度：P2；复现：手工写入 neutral=1700 或写盘中断。
- 预期：安全默认或完整旧文件；实际：错误字段被合并，直接写可能留下半文件。
- 涉及：`motor_mapping.py`、`api_routes.py`。
- 根因：持久化内容被当作可信，缺少原子 replace。
- 修复：加载时验证类型、唯一通道、1500 中位和绝对范围；临时文件 fsync + replace + 目录 fsync，损坏整体回退默认。
- 硬件验证：否。

### WEB-P2-03 日志/诊断不足，配置轮转参数未生效

- 严重度：P2；复现：修改 max_bytes，制造 CRC/NACK/PWM 失败后查看日志。
- 预期：可关联请求和后台状态；实际：轮转固定、CRC 无独立计数、失败上下文不完整。
- 涉及：`logger.py`、`serial_transport.py`、`api_routes.py`、`ws_manager.py`。
- 根因：日志参数硬编码且统计未贯穿诊断接口。
- 修复：配置化轮转；PWM 日志含 request_id/sequence/channel/value/duration/result/reason；诊断增加 generation、重连、CRC、任务和运动锁。
- 硬件验证：否。

### WEB-P2-04 前端缺少 lint/typecheck 命令，旧 Vite 有已知公告

- 严重度：P2；复现：`npm run typecheck`/`npm run lint`，再执行 `npm audit`。
- 预期：门禁可执行且无高危公告；实际：脚本不存在，Vite 5 报路径处理漏洞。
- 涉及：`package.json`、`package-lock.json`、`eslint.config.js`、`run_tests.sh`。
- 根因：项目仅依赖 build 间接执行 tsc，工具链长期未更新。
- 修复：增加独立 typecheck/lint/E2E；升级到 Vite 7.3.6 与兼容插件；审计为 0 漏洞。
- 硬件验证：否。

## 最终验证

- Python 协议、后端、模拟器：117 passed。
- 前端 Node 单元测试：24 passed。
- TypeScript、ESLint、生产构建：通过；Vite 7.3.6，产物 JS 约 172kB。
- `npm audit --audit-level=high`：0 vulnerabilities。
- Chrome 模拟 E2E：通过；覆盖断线重连、API 错误、同步双击、ESTOP 优先、最后客户端关闭后 DISARM/8 路 1500us。
- STM32 两组主机协议测试：PASS；Debug 固件构建通过，FLASH 117592B、RAM 50120B。
- 重复启动：第二实例在串口前明确报告已有 PID；Ctrl+C 日志确认 shutdown neutral=True、disarm=True。

## 修改文件

- `opi_console/config.py`、`config.yaml`：单一严格配置、能力/时序/锁文件参数。
- `opi_console/main.py`、`logger.py`：环境覆盖、单实例、启动/关闭安全、可靠日志路径和轮转。
- `opi_console/serial_transport.py`：ACK/CRC/sequence、连接 generation、重连和任务清理。
- `opi_console/stm32_proxy.py`：requested/confirmed、状态请求串行化、条件确认、stale 和 JSON 安全快照。
- `opi_console/simulated_stm32.py`：急停、回中状态转换、重连清理和持续时间契约对齐。
- `web_backend/app.py`、`control_state.py`、`ws_manager.py`：lifespan、路由、全局安全锁、会话化并发广播、最后客户端策略。
- `web_backend/api_routes.py`、`motor_mapping.py`：严格 API、正确状态码/确认、诊断、原子映射。
- `web_frontend/src/App.tsx`、`status.ts`、`uiState.ts`、`capabilities.ts`：fail-closed 状态、single-flight、ESTOP 优先、能力重载和缺失值显示。
- `web_frontend/package.json`、`package-lock.json`、`eslint.config.js`：测试/类型/lint/E2E 和无公告构建工具链。
- `Modules/Protocol/src/protocol_handler.cpp`：仅把手动 PWM 最短持续时间收紧到 200ms；未改 PCA9685 输出逻辑。
- `tests/test_web_backend.py`、`test_web_completion.py`、前端测试、STM32 host tests：回归覆盖。
- `scripts/start_web_console.sh`、`run_tests.sh`、协议测试脚本权限：可靠启动和统一门禁。
- `requirements.txt`、`requirements-dev.txt`、`ORANGE_PI_README.md`、`docs/*.md`：依赖、运行、排障和硬件验收说明。

新增测试重点包括：最后客户端 1/2/重复断开，安全动作失败仍 DISARM，新客户端不取消安全转换，HTTP 取消回中，PWM/ESTOP 并发，ESTOP 不确定锁存，通道切换重新进入 MANUAL，20ms 延迟应用确认，ACK/状态超时回中，串口噪声/CRC/sequence/generation，传感器任务异常恢复，慢 WebSocket 隔离，多会话排序，配置/映射/日志/路由，以及完整 Chrome 用户流程。

## 尚未完成与风险

- 未连接真实 STM32/PCA9685/电调/推进器；物理脉宽、UART 热插拔、Orange Pi 断电和 STM32 watchdog 必须现场验证。
- HTTP/WS 当前没有用户认证或 TLS，应只放在受控调试网络；加入认证属于功能扩展，本轮未做。
- 协议 v2 的 STATUS_REPORT 没有 request-id；软件通过单 waiter、条件确认和安全二次检查规避交叉确认，仍需真机延迟/噪声测试。
- 直接使用 REST 而不建立 WebSocket 不具备“最后浏览器断开”租约语义；脚本/自动化调用者必须显式 DISARM。
