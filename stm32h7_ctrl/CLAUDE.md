# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32H723VGT6 (Cortex-M7, 480 MHz) embedded firmware for a custom control board, generated with STM32CubeMX v6.18.0 and built with CMake + GCC ARM. Runs FreeRTOS v10.6.2 (CMSIS-RTOS V2) with a five-layer architecture.

Peripherals: 3× FDCAN, 4× USART + 1× UART, 3× SPI, 1× I2C, USB CDC Virtual COM Port, OCTOSPI flash, ADC, PWM timers (TIM1/2/3/12), RNG.

Hardware schematics: `原理图/`.

### Architecture (Five-Layer)

```
tasks/          — FreeRTOS 任务层，sw_timer 驱动，编排驱动+服务
service/        — 纯业务逻辑，不自建定时器（srv_xxx_step 由 task 层调用）
device_drivers/ — HAL 语义化封装（drv_xxx），唯一接触 HAL 句柄的层
m_middlewares/  — 平台无关库 ← ../public_layer/m_middlewares/ (add_subdirectory)
Core/ + HAL/    — CubeMX 生成层 + STM32H7 HAL + CMSIS
```

### Layer Details

**tasks/** — FreeRTOS 任务编排

| Module | Init | Description |
|--------|------|-------------|
| `can_task` | `can_task_init()` | CAN 通信：主机上报 + 从板控制 + RX 接收 |
| `daemon_task` | `daemon_task_init()` | 守护监控：9 电机反馈超时看门狗 |
| `led_task` | `led_task_init()` | LED 状态指示：蓝色+红色呼吸模式 (TIM1 PWM) |
| `log_task` | `log_task_init()` | 日志输出：UART / SEGGER RTT 后端，运行时切换 |

All tasks expose `void xxx_task_init(void)` and are driven by `sw_timer` (not by polling in a while loop).

**service/** — 业务逻辑

| Module | Input | Output | Description |
|--------|-------|--------|-------------|
| `srv_can` | CAN FD 控制帧 (0x100) | 反馈帧 (0x101) + 状态帧 (0x102) | 9 轴电机控制协议：控制帧解析 + 反馈/状态帧打包。使用回调注入硬件接口。 |
| `srv_led` | 异步命令队列 (kfifo) | PWM 输出（回调注入） | 多实例 LED 控制器：ON/OFF/编码闪烁/呼吸。内部使用 FSM + clist 实例链表。 |

Config/context 分离设计：`srv_xxx_config_t` 初始化时设定，`srv_xxx_context_t` 持有运行时状态。

**device_drivers/** — HAL 语义化封装

| Module | Description |
|--------|-------------|
| `drv_can` | FDCAN 驱动。dlc 始终为实际字节数(0-64)，内部自转 FDCAN DLC。RX 回调注册。 |
| `drv_uart` | 通用串口驱动。DMA TX + DMA circular + IDLE 中断 RX (ping-pong 双缓冲)。多实例，`drv_uart_init()` 无参数。 |
| `drv_systick` | 微秒/毫秒级延时 (`delay_us`, `delay_ms`, `micros`, `millis`) |
| `hal_flash/` | Flash 硬件抽象层。单例模式，编译时宏 `HAL_FLASH_CHIP_STM32H7` 选型。内嵌 drv_stm32f4_flash / drv_stm32g4_flash 底层。 |

Driver conventions (`e1-firmware-rules` skill):
- `drv_xxx_init(void)` **无参数** — HAL 句柄通过内置硬件配置表绑定
- HAL 弱回调在 driver `.c` 内重写，只做"解析通道、更新缓冲"最小工作
- 中断上下文通过回调直接上抛数据，不经过任务队列

**m_middlewares/** — `../public_layer/m_middlewares/` (add_subdirectory)

Platform-agnostic static library. All modules are statically allocated, no malloc. Include via `#include "public.h"`.

| Group | Modules |
|-------|---------|
| `algorithm/` | PID / gimbal_pid / MIT controller, PT1/IIR filters, PLL, 3D math, CRC8/16 |
| `framework/` | sw_timer (三级), fsm (表驱动), daemon (看门狗), event, msg_fifo |
| `utils/` | clist (侵入式链表), kfifo (无锁环形缓冲) — 底层基石 |
| `log/` | ESP32-style multi-level log (E/W/I/D/T), kfifo buffered |
| `key_base/` | Multi-instance key detection (click/double/long hold etc.) |
| `protocol_tools/` | Packer / parser (kfifo buffered) |
| `Third_Party/` | CmBacktrace, SEGGER RTT, LwMEM, mpaland_printf, ring_storage |

## Build Commands

```bash
# Configure and build (Debug)
cmake --preset Debug && cmake --build build/Debug

# Release
cmake --preset Release && cmake --build build/Release

# Direct (if already configured)
ninja -C build/Debug

# Clean
cmake --build build/Debug --target clean
```

Output: `build/{preset}/stm32h7_ctrl.elf`, `.hex`, `.bin`, `.map`, `compile_commands.json`.

Post-build: `.elf` → `.hex` + `.bin` automatically via `objcopy`.

**Toolchain:** `arm-none-eabi-gcc` on `$PATH`. Flags: `-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard`, linker script `STM32H723xG_flash.ld`, `nano.specs`.

**Clangd:** `.clangd` → `build/Debug/compile_commands.json`. Run `cmake --preset Debug` first.

## Project Configuration (STM32CubeMX)

- `stm32h7_ctrl.ioc` — master config. Open in CubeMX GUI to modify peripherals/clock/pins, then regenerate.
- CubeMX may update `cmake/stm32cubemx/CMakeLists.txt` with new sources/includes.
- Key compile symbols: `USE_PWR_LDO_SUPPLY`, `USE_HAL_DRIVER`, `STM32H723xx`.
- User compile defines (in root `CMakeLists.txt`): `HAL_FLASH_CHIP_STM32H7`, `PRINTF_DISABLE_SUPPORT_FLOAT`, `PRINTF_DISABLE_SUPPORT_EXPONENTIAL`.

## Clock Tree

- HSE 24 MHz → PLL1 → 480 MHz SYSCLK
- AHB = 240 MHz, APB1/2/3/4 = 120 MHz
- LDO supply, VOS Scale0

## Memory Map

| Region | Address | Size |
|--------|---------|------|
| Flash | — | 1024 KB |
| DTCMRAM | 0x20000000 | 128 KB |
| RAM_D1 (AXI SRAM) | 0x24000000 | 320 KB |
| RAM_D2 | 0x30000000 | 32 KB |
| RAM_D3 | 0x38000000 | 16 KB |
| ITCMRAM | 0x00000000 | 64 KB |

## Debugging

- **SEGGER Ozone:** `h7_debug.jdebug` — SWD 12 MHz, loads `build/Debug/stm32h7_ctrl.elf`.
- **VS Code:** `.vscode/launch.json` — JLink / ST-Link GDB configurations.
- **GDB:** Connect `arm-none-eabi-gdb` to GDB server on port 2331.

## Project-Level Skills

- `/e1-firmware-rules` — 五层架构约束、驱动自包含规范、服务解耦规约、WebKit C 编码规范
- `/mcu-pin-documenter` — 从原理图 PDF 生成 MCU 引脚汇总表
- `/protocol-doc-generator` — 嵌入式通信协议文档标准化生成
