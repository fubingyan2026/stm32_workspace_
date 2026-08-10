# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

**stm32_g0_boot** — E1 系统中**从电源板**的 CAN 固件升级 bootloader，MCU 为 STM32G0B1CBT6（Cortex-M0+，64 MHz，128KB Flash / 144KB RAM，LQFP48）。由 STM32CubeMX 骨架工程 `E1-fw_slave_power_g0` **原地改造**而来，参照 `../stm32_g474_boot` 的架构。工程名/产物为 `stm32_g0_boot`（`.elf`/`.hex`/`.bin`）。

从电源板的主/从板协议详见 `../E1_Master_Power_Manage/docs/protocol_slaver.md`。App 固件（从电源板业务）后续另建工程，链接到 App 分区 `0x08010000`。

## Flash 布局（单分区）

G0 与 G474 同为 128KB Flash，复用 G474 布局（`BOOT_SINGLE_PARTITION` 单分区）：

| 区域 | 地址 | 大小 | 说明 |
|------|------|------|------|
| Bootloader | `0x08000000` | 64 KB | 本工程链接区（`STM32G0B1XX_BOOT.ld`，FLASH LENGTH=64K） |
| App | `0x08010000` | 56 KB | 从电源板固件（待建） |
| Metadata | `0x0801E000` | 8 KB | `ring_storage` KV，存 `boot_metadata_t` |

板级分区参数经根 CMakeLists 的 `target_compile_definitions` 覆写共享层默认值：`BOOT_FLASH_BOOT_SIZE=0x10000U` / `BOOT_FLASH_APP_SIZE=0xE000U` / `BOOT_FLASH_META_SIZE=0x2000U`。

**关键**：G0 RAM 为 **144K**（`0x20000000-0x20024000`），必须在编译定义中覆写 `BOOT_RAM_START/BOOT_RAM_END`，否则 boot_task 的 App 向量表 SP 校验会把合法 SP（0x20024000）误判为越界。

## 构建

CMake 3.22+ / Ninja 生成器，`arm-none-eabi` GCC（`-mcpu=cortex-m0plus`，无 FPU）。工具链文件：`cmake/gcc-arm-none-eabi.cmake`（工具链须在 PATH 中；本机在 `%LOCALAPPDATA%\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin`，cmake/ninja 亦在 bundle 下，手动加 PATH 即可）。

```bash
cmake --preset Debug
cmake --build build/Debug

cmake --preset Release
cmake --build build/Release
```

- 构建产物：`build/<preset>/stm32_g0_boot.elf` + `.map` + `.hex` + `.bin`（POST_BUILD objcopy）。
- **烧录建议用 Release**：Debug `-O0` 下 FLASH 占用接近 64KB 上限（约 98%，因保留 ADC/TIM 中断代码），Release `-Os` 约 58%。
- 根 CMakeLists 设 `CMAKE_EXPORT_COMPILE_COMMANDS TRUE`，clangd 索引用 `build/Debug/compile_commands.json`（`.clangd` 已指向 `build/Debug`）。
- 本工程**没有测试设施**。

## 架构（复用 public_layer 共享 boot 栈）

```
tasks/app_main.c → delay_init(); log_task_init();
                → boot_task_try_boot_app()   // 校验 App 有效则跳转
                → boot_task_init()           // 否则进入升级模式
                → while(1){ sw_timer_tick(millis()); sw_timer_task(); }
```

| 目录/层 | 说明 |
|---------|------|
| `tasks/` | `app_main.c`（本地）——启动决策 + 主循环，无 LED 任务 |
| `device_drivers/` | `drv_can`（绑定 **FDCAN2** PB12/13）、`drv_log_uart`（USART1 PB6/7 DMA）、`drv_systick`（`SystemCoreClock` 自动适配 64MHz） |
| `../public_layer/service/boot/` | **共享**：`boot_transport`（0x701/0x702 协议编解码）、`boot_fsm`（升级状态机）、`boot_flash`（分区管理 + Metadata） |
| `../public_layer/task/` | **共享**：`boot_task`（启动决策 + CAN 升级接收 + 回滚）、`log_task`（UART DMA 日志） |
| `../public_layer/device_drivers/hal_flash/` | **共享**：`hal_flash` 抽象层 + `drv_stm32g0_flash`（**本工程新增的 G0 驱动**，2K 页 / 64-bit 双字 / DBANK 检测） |
| `../public_layer/m_middlewares/` | 静态库 `m_middlewares`（sw_timer/fsm/msg_fifo/kfifo/log/ring_storage/crc/SEGGER_RTT/mpaland_printf） |

**CAN**：FDCAN2（PB12/PB13，1 Mbps），经典 CAN 8 字节帧（与共享 `boot_transport` 的 `{8}` 帧长集合匹配）。升级 CAN ID 覆写为 `0x701`（Host→Node）/ `0x702`（Node→Host）。

**硬件兼容 ID**：`BOOT_HW_COMPAT_ID=0x0003U`（G0 从电源板专用；G474=0x0001，E1_Master=0x0002）。上位机工具据此确认目标板，需同步到 `E1_Master_Power_Manage/updata_tool/` 配置。

**Boot 行为**：
- `boot_task_try_boot_app()`：读 Metadata → 无有效 App / 校验失败 / 升级标志置位 → 进升级模式；否则跳转 App。
- 升级流程：START 握手 → METADATA → 1KB 块传输（DATA_START/DATA/DATA_END，块校验 + 重试）→ VERIFY（32-bit 累加和）→ REBOOT。
- 回滚安全：会话失败/取消 2s 无新会话 → 清 `upgrade_flag` 复位回滚；初始 IDLE 12s 无指令且有有效旧版本 → 回滚。

## 共享层改动注意

- **本工程新增共享文件**：`../public_layer/device_drivers/hal_flash/drv_stm32g0_flash.{c,h}`，并在 `hal_flash.c/h` 加入 `HAL_FLASH_CHIP_STM32G0` 选型分支。改动影响所有兄弟工程，但 `#ifdef` 隔离保证现有 F4/G4/H7 选型行为不变。
- 修改 `../public_layer/` 任何文件会影响所有工程（共享层），改动前需谨慎并验证兄弟工程。

## 工程约定

- **改硬件外设**：优先编辑 `E1-fw_slave_power_g0.ioc`，再用 CubeMX 重新生成，避免配置与代码不同步（CubeMX 会覆盖 `Core/Src` USER CODE 块之外的内容）。
  - **注意**：本工程保留 ADC1/TIM2/3/6/14 外设代码（main.c 不调用其初始化）。若 CubeMX 重新生成，main.c 的初始化序列会恢复所有 `MX_*_Init()` 调用且丢失 `app_main()`——需在 USER CODE 块外重新裁剪、USER CODE 2 恢复 `app_main()`。如需彻底精简，可编辑 `.ioc` 移除 ADC/TIM 后重新生成。
- **CubeMX 重新生成后**：`Core/` 文件被覆盖，自定义代码只在 USER CODE 块内保留；新增 `Core/Src/*.c` / `Inc/*.h` 需手工加入 `cmake/stm32cubemx/CMakeLists.txt`。根 CMakeLists、CMakePresets、链接脚本、本文件不会被重新生成。
- **编码规范**（`e1-firmware-rules` skill，全 workspace 生效）：WebKit 风格（4 空格缩进，函数 Allman、控制流 K&R），行宽 ≤100；命名 `module_name_` 前缀 + snake_case，类型后缀 `_t`；固定宽度类型（`uint8_t`/`uint16_t`/`uint32_t`/`int32_t`/`bool`）；公共 API 与复杂逻辑中文注释 + Doxygen `@brief`/`@param`/`@return`；未用参数 `(void)param;`。
- **文档**：统一放各工程 `Docs/` 目录（中文命名）。主/从板协议见 `E1_Master_Power_Manage/docs/protocol_slaver.md`，升级协议见 `../stm32_g474_boot/boot_protocol_spec.md`。

## 偏好

1. 问答语言优先使用中文，生成的文档名称和内容尽量使用中文。
2. 生成的文档统一放在工程根目录 `Docs/` 下（可按需建子文件夹）。
3. 修改硬件相关源码时优先修改 cubemx 配置文件，再提示用户用 CubeMX 重新生成工程。
