# 1132_bot — STM32F407 Underwater ROV

Monorepo for the 1132_bot underwater ROV: STM32F407 firmware, Orange Pi web console, and tools.

## Project Structure

```
1132_bot/
├── firmware/              # STM32F407 embedded firmware (CMake + ARM GCC)
├── web/                   # Orange Pi web console (FastAPI + React)
├── scripts/               # CI and utility scripts
├── docs/                  # Architecture, usage, and safety docs
├── gamepad_forwarder_linux/  # USB gamepad → WebSocket forwarder
└── 1132_bot.code-workspace  # VS Code workspace file
```

## Quick Start

### Firmware

```bash
cd firmware

# Check your environment
python3 scripts/check_env.py

# Build (Debug)
cmake --preset Debug
cmake --build --preset Debug

# Flash via DAP-Link (connects to hardware!)
cmake --build --preset Debug --target flash
```

### Web Console (Orange Pi / Local)

```bash
./scripts/start_web_console.sh --simulate   # No hardware needed
./scripts/start_web_console.sh              # With STM32 connected
```

## Development Environment

### What You Install Once

| Tool | Purpose |
|------|---------|
| ARM GNU Toolchain (`arm-none-eabi-gcc`) | Cross-compiler for Cortex-M |
| CMake (>= 3.21) | Build system generator |
| Ninja | Build executor |
| OpenOCD (>= 0.12.0) | Debug probe server |
| Python 3 | Scripts and web backend |
| Node.js | Web frontend |
| VS Code | Editor (open `1132_bot.code-workspace`) |

### VS Code

Open `1132_bot.code-workspace` to get:
- Correct CMake source directory
- Recommended extensions auto-prompt
- Pre-configured build, flash, and debug tasks

### GDB / Debugging

- **Linux**: Uses `gdb-multiarch` (system package). Select `STM32: Debug (gdb-multiarch)`.
- **Windows**: Uses `arm-none-eabi-gdb` from ARM GNU Toolchain. Select `STM32: Debug (ARM GDB)`.

### STM32CubeMX

The firmware includes `1132_bot.ioc` — the real CubeMX project file.
Install [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) to edit pin assignments and regenerate peripheral code.

## License

[Your license here]
