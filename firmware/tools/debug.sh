#!/usr/bin/env bash
#
# 一键调试: 启动 OpenOCD GDB 服务 + 连接 gdb-multiarch 并下载固件
#
# 用法:
#   cmake --build build/Debug --target debug
#   或直接: tools/debug.sh build/Debug/1132_bot.elf openocd.cfg
#
# 流程:
#   1. 清理残留的 OpenOCD 实例 (防止端口占用导致连接失败)
#   2. 后台启动 OpenOCD (GDB 服务, 默认端口 3333)
#   3. 等待端口就绪, 确认 OpenOCD 进程存活
#   4. 启动 gdb 交互调试: 连接 -> 下载固件 -> 复位 -> 停在 main
#   5. 退出 gdb 后自动关闭 OpenOCD
set -euo pipefail

ELF="${1:?用法: $0 <elf文件> [openocd配置文件]}"
CFG="${2:-${ELF%/build/*}/openocd.cfg}"
PORT="${OPENOCD_PORT:-3333}"
GDB="${GDB:-gdb-multiarch}"

# 日志放在 build 目录, 避免干扰终端
BUILD_DIR="$(dirname "$ELF")"
LOG="${BUILD_DIR}/openocd.log"

# 1. 清理残留的 OpenOCD 实例。
#    旧实例会占用 3333/6666 端口: 新实例绑定端口失败会直接退出,
#    且端口检测会误连到旧实例 (连接很快被关闭, load/monitor 全部失败)。
for port in 3333 6666; do
    stale_pids="$(ss -tlnpH 2>/dev/null | sed -n "s/.*:$port .*pid=\([0-9][0-9]*\).*/\1/p" | sort -u)"
    if [ -z "$stale_pids" ] && command -v lsof >/dev/null 2>&1; then
        stale_pids="$(lsof -t -iTCP:$port -sTCP:LISTEN 2>/dev/null || true)"
    fi
    for pid in $stale_pids; do
        echo "==> 关闭残留 OpenOCD (PID $pid, 占用端口 $port)"
        kill "$pid" 2>/dev/null || true
    done
done
if [ -n "${stale_pids:-}" ]; then
    sleep 1
fi

# 2. 启动 OpenOCD (后台)
echo "==> 启动 OpenOCD: $CFG (日志: $LOG)"
"${OPENOCD:-openocd}" -f "$CFG" >"$LOG" 2>&1 &
OPENOCD_PID=$!
trap 'kill "$OPENOCD_PID" 2>/dev/null || true' EXIT

# 确认 OpenOCD 正常启动 (绑定端口失败等会立即退出)
sleep 1
if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
    echo "==> 错误: OpenOCD 启动失败, 日志:"
    tail -n 20 "$LOG" || true
    exit 1
fi

# 3. 等待 GDB 服务就绪 (最多 15 秒)
ready=0
for _ in $(seq 1 30); do
    if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
        echo "==> 错误: OpenOCD 中途退出, 日志:"
        tail -n 20 "$LOG" || true
        exit 1
    fi
    if (echo >"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then
        ready=1
        break
    fi
    sleep 0.5
done

if [ "$ready" -ne 1 ]; then
    echo "==> 错误: OpenOCD GDB 服务 ($PORT) 未就绪, 最近日志:"
    tail -n 20 "$LOG" || true
    exit 1
fi

echo "==> GDB 服务就绪, 启动 gdb 调试 $(basename "$ELF")"
echo "    (Ctrl+C 暂停程序, quit 退出并关闭 OpenOCD)"

# 4. 交互式调试
"$GDB" "$ELF" \
    -ex "target extended-remote :$PORT" \
    -ex "monitor reset halt" \
    -ex "load" \
    -ex "monitor reset halt" \
    -ex "break main" \
    -ex "continue" \
    || true   # gdb 退出码不一定为 0, 交给 EXIT trap 清理 OpenOCD

echo "==> gdb 已退出, 关闭 OpenOCD"
