# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

- **Toolchain**: `arm-none-eabi-gcc` (default preset). STM32CubeIDE wraps cmake via `cube-cmake` and also supports `starm-clang` (see `cmake/starm-clang.cmake`).
- **Generator**: Ninja

```bash
cmake --preset Debug
cmake --build build/Debug
```

Four presets are available: `Debug`, `Release` (`-Os -g0`), `RelWithDebInfo`, `MinSizeRel`. Post-build, `.hex` and `.bin` files are generated via `arm-none-eabi-objcopy`. `compile_commands.json` is exported to the build directory for clangd.

### Convenience scripts

- **Windows**: `build.bat [-t Debug|Release]` — auto-locates ARM toolchain (STM32CubeIDE bundle or PATH), cmake, and ninja; configures and builds.
- **Linux/macOS**: `build.sh [-t Debug|Release]` — same logic, searches common toolchain paths.

### clangd

`.clangd` points `CompilationDatabase` to `build/RelWithDebInfo`. For IDE intellisense, use the RelWithDebInfo preset so `compile_commands.json` is available at that path. (To switch, edit `.clangd` and reconfigure with the matching preset.)

## Flash layout

The STM32G474 has 128 KB of internal flash. The linker script (`STM32G474XX_FLASH.ld`) maps the **bootloader's own 64 KB** region — App partitions are accessed at runtime via absolute-address flash operations, not linker symbols.

| Region | Address | Size | Notes |
|--------|---------|------|-------|
| Bootloader | `0x08000000` | 64 KB | Linked region; contains this project |
| App A | `0x08010000` | 24 KB | Active/standby firmware slot |
| App B | `0x08016000` | 24 KB | Alternate firmware slot |
| Metadata | `0x0801C000` | 16 KB | Managed via `ring_storage` KV; stores `boot_metadata_t` |

> **Note:** Sizes are board-parameterized in the shared [`boot_flash.h`](../public_layer/service/boot/boot_flash.h) (`#ifndef`-guarded, F407 defaults). This project's G474 values are set in [`CMakeLists.txt`](CMakeLists.txt): `BOOT_FLASH_BOOT_SIZE = 0x10000`, `BOOT_FLASH_APP_SIZE = 0x6000`, `BOOT_FLASH_META_SIZE = 0x4000`.

**Linker script note:** [`STM32G474XX_FLASH.ld`](STM32G474XX_FLASH.ld) maps `FLASH` as 64 KB (`LENGTH = 64K`), matching the bootloader region. Do not enlarge the linker FLASH region without understanding the partition boundary.

The flash driver (`drv_stm32g4_flash`) auto-detects single-bank (4 KB pages) vs. dual-bank (2 KB pages) mode at init by reading the `FLASH->OPTR` DBANK bit, and handles cross-bank erase by splitting operations at bank boundaries.

## Project structure

| Directory | Ownership | Notes |
|---|---|---|
| `Core/` | CubeMX **generated** | HAL init, `main.c`, peripheral config (`gpio.c`, `fdcan.c`, `spi.c`, etc.). Contains `USER CODE BEGIN`/`END` guards. |
| `Drivers/` | **Vendor** (ST) | HAL + CMSIS. Do not modify. |
| `cmake/stm32cubemx/` | CubeMX **generated** | CMake config for HAL sources. Regenerated with CubeMX. |
| `tasks/` | **User** | Application glue layer. `app_main` lives here; each task owns `sw_timer` instances and wires drivers to services. Only project-local tasks remain (`app_main`, `led_task`) — the boot/log tasks are shared. |
| `device_drivers/` | **User** | Hardware abstraction layer (CAN, log UART, systick, LED). Direct HAL usage lives here. Flash HAL is **shared** — see `../public_layer/device_drivers/hal_flash`. |
| `../public_layer/` | **User (shared)** | Cross-project shared code. This project compiles `device_drivers/hal_flash/`, `m_middlewares/`, the boot stack (`service/boot/*`, `task/boot_task`, `task/log_task`), and `service/srv_signal.{c,h}` (drives the running LED in `led_task`) from here. There is no local `service/` directory. |
| `m_middlewares/public.h` | **User** | Central include umbrella under `extern "C"` — application code includes only this to get all middleware headers. |
| `updata_tool/` | **User** | Host-side Python/PySide6 flashing tool that drives the bootloader over CAN (via CANable USB-CAN adapter). |
| `stm32_g474_boot.ioc` | CubeMX **config** | Source of truth for pin mux, clocks, and peripheral assignment. |

## Multi-project workspace

This project is part of a larger workspace at `stm32_workspace_/`. **The git repository root is `stm32_workspace_/` — one single repo spans all projects plus `public_layer/`** (there is no per-project `.git`). Commits can and do span multiple projects, and `git status` run inside this project also reports sibling-project changes.

| Directory | Description |
|---|---|
| `stm32_g474_boot` | **This project** — G474 bootloader |
| `E1_Hand_G474` | G474 dexterous hand firmware |
| `E1_Master_Power_Manage` | Power management firmware (F407). Shares the same `public_layer` boot stack, parameterized for its layout. |
| `public_layer/` | **Shared cross-project code** — `m_middlewares/`, `device_drivers/hal_flash/` (incl. `ring_storage_port_hal.h`), `service/boot/`, `service/srv_signal.{c,h}`, `task/boot_task.{c,h}`, `task/log_task.{c,h}` |

### Shared middleware pattern

Shared code lives in `../public_layer/` (not locally). This project's `CMakeLists.txt` consumes it three ways:
- `m_middlewares/` as a static library via `add_subdirectory(../public_layer/m_middlewares m_middlewares)` — built once per project.
- `device_drivers/hal_flash/` via an explicit `aux_source_directory(../public_layer/device_drivers/hal_flash ...)`.
- The **boot stack** (`service/boot/*.c`, `task/boot_task.c`, `task/log_task.c`) via an explicit source list.

Modifications to anything under `public_layer/` affect all projects. **Do not duplicate any of it locally.** The boot stack is board-parameterized: `boot_flash.h`/`boot_task.c` wrap their board-specific constants in `#ifndef` with F407 defaults, and each project passes its own values via `target_compile_definitions` (G474's are in [`CMakeLists.txt`](CMakeLists.txt) — partition sizes, `BOOT_HW_COMPAT_ID=0x0001U`, App flash range).

## Architecture

### Boot sequence

```
Reset_Handler (startup_stm32g474xx.s)
  → SystemInit()
  → Copy .data to RAM, zero .bss
  → main()                          [Core/Src/main.c]
       → HAL_Init()
       → SystemClock_Config()       (160 MHz from HSE + PLL)
       → MX_GPIO/DMA/UART/SPI/FDCAN/TIM_Init()
       → app_main()                 [tasks/app_main.c]
            → delay_init()
            → log_task_init()
            → boot_task_try_boot_app()   // Validate App checksum, jump if valid
            → boot_task_init()           // Enter bootloader mode if no valid App
            → while(1) { sw_timer_tick(); sw_timer_task() }
```

`main.c`'s own `while(1)` after the `app_main()` call is **unreachable dead code** — `app_main()` contains its own infinite loop. Only add new peripheral init calls inside `USER CODE BEGIN 2` in `main.c`; the loop body under `USER CODE BEGIN WHILE` is unused.

There is **no RTOS**. All periodic work runs through `sw_timer` — a cooperative scheduler driven by `sw_timer_task()` in the main loop. SysTick ISR calls `sw_timer_tick()` to mark expired timers ready; the main loop dispatches their callbacks by priority (HIGH → NORMAL → LOW). Adding periodic work always means creating a new `sw_timer`.

### Four-layer architecture

```
tasks/            → Glue: owns sw_timers, wires drivers to services
service/ (shared) → Domain logic (boot FSM, flash ops, transport; srv_signal) — lives in ../public_layer/service
device_drivers/   → HAL calls, DMA, interrupts, kfifo buffering
m_middlewares/    → Reusable frameworks and algorithms
```

Tasks are thin — they own timers and stitch layers together. Services contain business logic but never touch HAL directly (they receive callback function pointers instead). Device drivers wrap HAL and expose `init()`/`deinit()`/`is_initialized()` lifecycles.

### Bootloader design

This project implements a **dual A/B partition** firmware upgrade system over CAN. The protocol is fully specified in [`boot_protocol_spec.md`](boot_protocol_spec.md). The CAN FD DLC padding fix is documented in [`can_fd_dlc_padding_fix.md`](can_fd_dlc_padding_fix.md).

**The entire boot stack is shared** — it lives in `../public_layer/service/boot/` + `../public_layer/task/` and is compiled into this project (and `E1_Master_Power_Manage`) from the same sources. Do **not** create local copies. The shared code is board-parameterized via `#ifndef`-guarded macros whose defaults are the F407 layout; this project supplies its own values in [`CMakeLists.txt`](CMakeLists.txt):
- `BOOT_FLASH_BOOT_SIZE=0x10000U`, `BOOT_FLASH_APP_SIZE=0x6000U`, `BOOT_FLASH_META_SIZE=0x4000U`
- `BOOT_HW_COMPAT_ID=0x0001U`, `BOOT_APP_FLASH_START=0x08010000U`, `BOOT_APP_FLASH_END=0x08016000U`
- `BOOT_CAN_ID_HOST_TO_NODE=0x701U`, `BOOT_CAN_ID_NODE_TO_HOST=0x702U` (can be changed per board/host)
- Metadata sector defaults to `hal_flash_get_caps()->erase_size` (G4 uniform pages).

**Protocol basics:**
- CAN IDs `0x701` (Host→Node) and `0x702` (Node→Host) — Host ID configurable in the GUI, Node ID = Host ID + 1
- 2-byte header (Command + Sequence) per frame
- 1 KB block checksums (16-bit additive) with checksum at fixed offset (Byte 2-3) to avoid CAN FD padding issues
- Full-image verification: 32-bit additive checksum (`sum(fw_data) & 0xFFFFFFFF`) instead of CRC32
- Commands: `START` (0x01), `METADATA` (0x02), `DATA` (0x03), `VERIFY` (0x04), `REBOOT` (0x05), `CANCEL` (0x06), `DATA_START` (0x07), `DATA_END` (0x08), `ACK` (0x10), `NACK` (0x11)
- **Classic CAN only on G474 today**: the shared `boot_transport` advertises only the 8-byte frame size (`{8}`) because it was adapted for the F407's bxCAN. The former local G474 transport negotiated the full CAN FD set `{8,12,16,20,24,32,48,64}`; restoring CAN FD means widening the shared transport's `s_supported_frame_sizes`.

**Boot service layer (`../public_layer/service/boot/`):**

| Component | File | Role |
|---|---|---|
| Transport | `boot_transport.c` | Stateless frame encode/decode. Parses all 10 command types and builds ACK/NACK responses. |
| FSM | `boot_fsm.c` | Upgrade state machine (5 states: IDLE → START → DATA_TRANSFER → VERIFY_PENDING → REBOOT_PENDING). Uses the `fsm` library's guard matrix for transition validation. Drives the upgrade through callbacks — never touches hardware directly. |
| Flash | `boot_flash.c` | Partition-aware flash manager. Erases/writes/verifies App partitions (size defined by `BOOT_FLASH_APP_SIZE`), manages the metadata page, computes checksums over flash. Delegates to the shared `hal_flash` layer. |

**Upgrade flow (4 phases):**
1. **Handshake**: START (negotiate frame size, validate HW compat ID, erase target partition) → METADATA (32-bit additive checksum + version)
2. **Data transfer**: Per 1 KB block: DATA_START (block index handshake) → N × DATA frames → DATA_END (16-bit additive checksum). Block-level retry (up to 3×) on checksum failure. Double-verified: checksum on wire + byte-by-byte read-back after flash write. CANCEL (0x06) accepted at any time to abort and return to IDLE.
3. **Verify**: Full-image 32-bit additive checksum computed over the written flash partition, compared with host-provided value.
4. **Commit & reboot**: Write metadata page (magic `0x424F4F54`, partition, version, checksum), then NVIC system reset.

**CAN FD DLC padding caveat:** CAN FD data lengths are discrete (`{8, 12, 16, 20, 24, 32, 48, 64}`). When a DATA_END frame doesn't exactly fill a discrete length, the host pads with zero bytes. The board-side parser must **cap `rem_len` by free buffer space** rather than trusting the DLC-derived length — see [`can_fd_dlc_padding_fix.md`](can_fd_dlc_padding_fix.md) for the full analysis.

**CAN Bus-Off auto recovery:** The boot task polls `drv_can_is_bus_off()` every ~100 ms (via `sw_timer` tick counter). When Bus-Off is detected, `drv_can_recover()` is called to re-initialize the CAN controller, allowing recovery from cable faults without a hardware reset.

**Boot decision (`boot_task_try_boot_app`):**
1. Read metadata page at `0x0801C000` (start of the `BOOT_FLASH_META_SIZE` ring_storage area, computed as `boot_flash_base() + BOOT_FLASH_BOOT_SIZE + BOOT_FLASH_APP_SIZE * 2`)
2. If `magic != 0x424F4F54` or `upgrade_flag != 0` or `fw_size == 0` or App 32-bit additive checksum mismatch → enter bootloader
3. Otherwise → jump to App (**enabled** in the shared `../public_layer/task/boot_task.c`): promote partition B→A if active, validate the vector table (SP in RAM, PC within `[BOOT_APP_FLASH_START, BOOT_APP_FLASH_END)`), `log_task_flush()`, `__disable_irq()`, `__set_MSP()`, then jump.
4. **Rollback safety** (shared task): after a failed/cancelled upgrade session (2 s idle) or a 12 s initial idle with a valid previous version, it clears `upgrade_flag` and resets back to the last-good App.

> **⚠ App-link caveat:** the G474 App (`E1_Hand_G474`) currently links at flash base `0x08000000` with no VTOR relocation — it does **not** sit in partition A (`0x08010000`). The shared boot task's vector-table check would therefore reject it and the bootloader stays in upgrade mode. Boot-to-App stays inactive until the App is relinked to partition A with `USER_VECT_TAB_ADDRESS` set.

**A/B swap logic:** New firmware always goes to the *opposite* partition of the currently-active App. The old partition is never erased until the new firmware is fully verified and committed, ensuring an unbrickable update.

### hal_flash abstraction layer

The flash subsystem uses a multi-platform abstraction with compile-time chip selection. It lives in the **shared layer** at `../public_layer/device_drivers/hal_flash/` and is compiled into this project via `CMakeLists.txt`:

```
../public_layer/device_drivers/hal_flash/
├── hal_flash.h           → Public API (hal_flash_dev, read/write/erase/lock)
├── hal_flash_base.h      → Base types (hal_flash_ops_t, lock-depth)
├── hal_flash.c           → Singleton dispatch, lock management
├── drv_stm32g4_flash.c   → STM32G4 ops (64-bit program, dual-bank auto-detect)
├── drv_stm32g4_flash.h   → G4 private types
├── drv_stm32f4_flash.c   → STM32F4 ops
├── drv_stm32h7_flash.c   → STM32H7 ops
└── ring_storage_port.c   → Ring storage flash port (key-value storage on flash)
```

Select chip at compile time via `target_compile_definitions`:
- `HAL_FLASH_CHIP_STM32G4` (this project)
- `HAL_FLASH_CHIP_STM32F4`
- `HAL_FLASH_CHIP_STM32H7` (driver exists — shared with other projects)

Key design features:
- **Singleton**: `hal_flash_dev()` returns the single device instance — no `dev` parameter in public API
- **Lock depth**: Reentrant lock with depth counter (nested lock/unlock pairs), defaults to `__disable_irq()`/`__enable_irq()`; custom lock callbacks via `hal_flash_set_lock_cb()`
- **Operations table**: `hal_flash_ops_t` struct with function pointers for init/read/write/erase/wait — each chip driver fills this
- **Compile-time polymorphism**: No vtable overhead — which driver compiles in is decided by `#ifdef HAL_FLASH_CHIP_STM32G4` in `hal_flash.c`

### Middleware ecosystem

- **`sw_timer`** — Software timers with priority levels. Single-shot, N-repeat, or infinite. Backbone of all periodic work.
- **`fsm`** — Flat state machine with guard-matrix transition validation. Used by the shared `../public_layer/service/boot/boot_fsm` and `../public_layer/service/srv_signal` (the FSM-based LED/buzzer/GPIO output module that `led_task` uses to drive the running LED). Not included in `public.h` (include manually).
- **`event`** — ISR-safe 32-bit event flags for main-loop polling.
- **`daemon`** — Task watchdog with online/offline callbacks and debounce.
- **`kfifo`** — Lock-free power-of-2 ring buffer. Single-producer/single-consumer safe from ISR. Used for all DMA buffering.
- **`clist`** — Intrusive circular doubly-linked list (Linux-kernel style). Backbone of `daemon`, `sw_timer`, `key_base` instance registries.
- **`msg_fifo`** — Typed message queue layered on `kfifo`. Used to decouple CAN ISR from main-loop FSM processing (256-slot queue).
- **`log`** — ESP32-style colored logging (`LOG_E`/`W`/`I`/`D`/`T` macros). Output buffered through a `kfifo` for non-blocking DMA TX.
- **`protocol_packer`/`protocol_parser`** — Stateless frame builder / stateful frame de-serializer with configurable header/footer/checksum callbacks. Generic — not bootloader-specific.
- **`key_base`** — Button debounce with multi-event detection (click, double-click, long press, etc.).
- **`ring_storage`** (Third_Party) — Flash-backed key-value storage with wear leveling. Used for persistent configuration.
- **Algorithms**: PID (position/incremental), gimbal PID (cascade + gyro feedforward), MIT impedance control, PT1/Biquad/Slew/LMA filters, fast trig (`sin_approx`, `atan2_approx`), median filters, software PLL, CRC8/CRC16/CRC32.

**Third_Party middleware also available** (not in `public.h` umbrella — include directly):
- **CmBacktrace**: ARM Cortex-M hard fault analyzer — dumps call stack, registers, and fault status on crash. `#include "cm_backtrace.h"`, invoke `cm_backtrace_fault()` from HardFault_Handler.
- **lwmem**: Lightweight dynamic memory allocator (only use in non-bootloader projects — this project avoids `malloc`).
- **SEGGER_RTT**: Real-time terminal I/O via JTAG/SWD. Log output can be toggled at runtime between UART and RTT via `log_task_set_output()`.

### Log system

Logs can output to **UART** (via `drv_log_uart`, DMA TX), **SEGGER_RTT** (via JTAG/SWD), or be **disabled** — switched at runtime by `log_task_set_output(LOG_OUTPUT_UART | LOG_OUTPUT_RTT | LOG_OUTPUT_NONE)`. Default is UART. The log module uses an internal `kfifo` for non-blocking writes; a 20 ms `sw_timer` periodically drains the FIFO to the selected output.

## Host-side flashing tool (`updata_tool/`)

A Python/PySide6 desktop application that drives the bootloader over CAN via a CANable 2.5 USB-CAN adapter (Candlelight/ElmueSoft firmware).

```
updata_tool/
├── flash_gui.py              ← Convenience launcher at project root
├── canable_sdk/              ← USB-CAN driver (ZDTCanable, CANFrame, bitrate config)
├── flash_tool/               ← The flashing application
│   ├── __main__.py           ← Entry: python -m updata_tool.flash_tool
│   ├── main_window.py        ← PySide6 QMainWindow (device panel + firmware panel)
│   ├── protocol.py           ← Protocol codec (frame builders, parsers, checksum/CRC)
│   ├── worker.py             ← FlashWorker QThread — the 4-phase flashing state machine
│   └── widgets/              ← DevicePanel + FirmwarePanel
└── cangui/                   ← Separate CAN bus monitor GUI (not part of flashing)
```

To run: `python flash_gui.py` or `python -m updata_tool.flash_tool`. Requires PySide6, pyusb, and the bundled `libusb-1.0.dll` (Windows). On Linux, run `install_udev.sh` for USB permissions.

The `FlashWorker` state machine mirrors the board-side FSM: handshake → data transfer (with 3-retry block-level error recovery) → verify → reboot.

## Coding conventions

Follow **`MODULE_CODING_GUIDE.md`** for all new code. Key rules:

- **Style**: WebKit (4-space indent, Allman braces for functions, K&R braces for `if`/`for`/`while`). Max 100 chars per line.
- **Standard**: C11 (`-std=gnu11`), MISRA C:2012 compliance expected.
- **Naming**: `snake_case` with module prefix. Types end in `_t` (e.g. `protocol_parser_config_t`). Error enums start with `MODULE_OK = 0`. Static globals use `s_` prefix.
- **config-in-context pattern**: Every module has a `xxx_config_t` (immutable parameters + callbacks) nested inside a `xxx_context_t` (runtime state + copy of config + `initialized` flag). Init copies config, sets `initialized = true`. All public functions guard with `if (!ctx->initialized) return ERROR_UNINITIALIZED`.
- **Static allocation**: No `malloc` in middleware or services. Callers provide memory; modules use `xxx_register_static()` patterns.
- **`__malloc` zero-init**: If `__malloc` is used, it **must** be immediately followed by `memset(ptr, 0, size)`. Uninitialized memory can contain garbage values for `initialized` flags and pointers, causing crashes when `_deinit()` dereferences a garbage pointer. See `MODULE_CODING_GUIDE.md` § "动态内存分配必须初始化" for root-cause analysis.
- **Comments**: Chinese Doxygen-style (`@brief`/`@param`/`@return`) for public API. Source comments are mixed Chinese (GBK in CubeMX files, UTF-8 in user files).
- **Types**: Always use fixed-width integers (`uint8_t`, `uint16_t`, `uint32_t`) and `bool` from `<stdbool.h>`.
- **Section ordering**: `/* Includes */` → `/* Private constants */` → `/* Private variables */` → `/* Private function prototypes */` → `/* Exported functions */` → `/* Private functions */`.

## CubeMX code regeneration

Re-generating from the `.ioc` file **overwrites** `Core/` and `cmake/stm32cubemx/`. Code inside `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` guards in `Core/Src/main.c` is preserved; everything outside is lost. Only add code between the guards in CubeMX-managed files. User directories (`tasks/`, `device_drivers/`) and the shared `../public_layer/` are unaffected.

## Hardware

- **MCU**: STM32G474RBTx — Cortex-M4 with FPU, 128 KB Flash (`0x08000000`), 128 KB RAM (`0x20000000`)
- **Clock**: 160 MHz from HSE + PLL (Voltage Scale 1 + Boost, Flash latency 4)
- **Linker**: `STM32G474XX_FLASH.ld` — heap 512 B, stack 1024 B, newlib-nano, `--gc-sections`. Maps the bootloader's 64 KB flash region (`LENGTH = 64K`).
- **Toolchain flags**: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
- **Peripherals**: SPI1, FDCAN1 (PA11/PA12), FDCAN2 (PB12/PB13), USART1 (DMA TX + IDLE-line RX), TIM1, TIM15, GPIO, DMA
- No tests, no CI, no formatter/linter configured.
