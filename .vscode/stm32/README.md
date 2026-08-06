# STM32 VS Code Profile

VS Code Profile 名称：`stm32`

## 调试入口

在 Run and Debug 中选择：

```text
STM32F407 DAPLink OpenOCD Debug
```

该调试入口定义在当前工程的 `.vscode/launch.json`，不是 VS Code 默认 Profile 的全局调试配置。

## 路径

| 项目 | 路径 |
|------|------|
| OpenOCD 配置 | `firmware/openocd.cfg` |
| ELF | `firmware/build/Debug/1132_bot.elf` |
| OpenOCD | 由系统 PATH 自动查找（`openocd` 命令） |
| GDB | `gdb-multiarch`（Linux）或 `arm-none-eabi-gdb`（Windows/ARM 工具链） |

## OpenOCD 配置

当前使用无线 DAPLink，NRST 不可靠，所以 OpenOCD 配置中使用：

```tcl
reset_config none
```

不要改成 `srst_only`。

## 硬件注意事项

无线 DAPLink 需要目标板和目标端都供电，否则 OpenOCD 可能无法读取 CMSIS-DAP 信息，或在连接、复位、烧录时超时。

## 命令行烧录

```bash
# 使用 CMake preset（推荐）
cmake --build --preset Debug --target flash

# 或手动调用 OpenOCD
openocd -f firmware/openocd.cfg \
  -c "program firmware/build/Debug/1132_bot.elf verify reset exit"
```
