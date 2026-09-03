# AGENTS.md

STM32 固件 monorepo 工作区（git 根目录）：多个独立的 CubeMX 工程共享仓库外的 `public_layer/` 原件。均为裸机（无 RTOS）、C11、CMake/Ninja 构建。

## 工作区布局
- 独立固件工程（各自有 CubeMX 工程 + CMake 配置）：
  - `E1_Hand_G474/` — 9 自由度灵巧手（STM32G474，CAN FD + 多路 UART 电机）。最成熟工程，架构/协议看其 `AGENTS.md` + `CLAUDE.md` + `docs/`。
  - `E1_Master_Power_Manage/` — 主控电源板（STM32F407），看其 `README.md`、`CLAUDE.md`、`docs/`。
  - `G0_hand_ctrl_sample/` — G0 遥操作手套工程（STM32F103xB/Cortex-M3），已适配分层架构（device_drivers/tasks/service + public_layer），CAN 协议为骨架占位；`log_task` 为工程内本地副本（不复用 public_layer 版本）；`host/g0_host.py` 为串口上位机（PySide6 + pyserial，协议见 `docs/G0_Hand 串口通信协议规范.md`，帧格式 `z-cmd-len-payload-crc-\n`，CRC8 多项式 0x31 初值 0xFF）。
  - `stm32_g0b1_boot/`、`stm32_g474_boot/` — 引导程序。`stm32_g474_boot/MODULE_CODING_GUIDE.md` 是全局 C 模块编码规范。
  - `e1_dual_battery_hot_swappable/` — 双电池热插拔（git 未跟踪的新工程）。
- `public_layer/` — **共享代码原件**（`device_drivers/`、`m_middlewares/`、`service/`、`task/`）。各工程 CMake 通过 `../public_layer/` 直接引用，本目录没有本地副本。

## 构建（每个工程独立，Windows）
- 一键：`build.bat`（`E1_Hand_G474/`、`E1_Master_Power_Manage/`、`G0_hand_ctrl_sample/` 有；自动在 `%LOCALAPPDATA%\stm32cube\bundles` 下找 ARM GCC/CMake/Ninja；默认 RelWithDebInfo）。其他工程直接 `cmake --preset Debug && cmake --build --preset Debug`。
- 产物在 `build/{Debug|RelWithDebInfo}/`：`*.elf/.hex/.bin`。
- clangd 读 `build/.../compile_commands.json`（各工程 `.clangd` 指定的目录），**先构建再索引**。
- **不要修改任何 `CMakeLists.txt`（用户明确要求）**。

## 共享代码约定
- 改共享代码一律编辑 `public_layer/` 里的原件，不要复制进工程。
- 新代码先查 `public_layer/m_middlewares/`：framework（sw_timer/fsm/event/msg_fifo/daemon）、utils（kfifo/clist）、algorithm（pid/filter/crc）、log；`#include "public.h"` 可一次性引入全部中间件。

## 分层架构（严格只向下依赖）
```
applications/   app_* 业务策略（如 app_rgb_status：LED 灯效），依赖 service
tasks/          xxx_task_init() + sw_timer 回调
service/        srv_* 业务逻辑（FSM/协议/算法）
m_middlewares/  平台无关通用模块——禁止含 HAL
device_drivers/ drv_* HAL 薄封装——唯一允许引用 CubeMX 句柄的层
Core/           CubeMX 生成代码——只在 USER CODE 块内修改
```
- 全程静态分配（无 malloc），调用者提供句柄内存；`*_init()` 一律无参。

## 编码规范（见 `stm32_g474_boot/MODULE_CODING_GUIDE.md`）
- WebKit 风格（4 空格缩进、函数 Allman 大括号、控制语句 K&R）+ MISRA C:2012。
- 前缀：driver=`drv_`、service=`srv_`、task=`xxx_task_`；类型 `_t`/`_cb_t`；枚举大写蛇形。
- 公共 API + 复杂逻辑用中文 Doxygen（`@brief/@param/@return`）；公共函数首行校验 NULL/未初始化/越界。
- Config-in-context：`xxx_config_t` 嵌入 handle，每实例缓冲放 handle 内，禁止模块级 static 单例。
