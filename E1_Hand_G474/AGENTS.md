# AGENTS.md

STM32G474 (Cortex-M4, 128KB SRAM) 9 自由度灵巧手固件：裸机（无 RTOS）、C11、CMake/Ninja 构建。
完整架构与中间件速查见 `CLAUDE.md`（本文件是其精简版），大改动前先读它。

## 构建（Windows）
- 一键：`build.bat`（自动检测 `%LOCALAPPDATA%\stm32cube\bundles` 下的 ARM GCC/CMake/Ninja）；`build.bat -t Release` 出 Release。
- 手动：`cmake --preset Debug && cmake --build --preset Debug`。
- 产物在 `build/{Debug|Release}/`：`E1_Hand_G474.elf/.hex/.bin`。
- **依赖同级 `../public_layer/`，缺失即构建失败**。clangd 读 `build/Debug/compile_commands.json`——先构建再索引。

## 共享代码在仓库之外
- 本工程没有本地 `public_layer/`，共享代码原件在 `../public_layer/`（workspace 根）。CMake 直接 `add_subdirectory` `m_middlewares`，并显式引用 `service/srv_signal.c`、`service/srv_log_flash.c`、`task/log_task.c`、`device_drivers/hal_flash/`。
- **改共享代码一律编辑 `../public_layer/` 里的原件**，不要复制进本工程。

## 分层架构（严格只向下依赖）
```
tasks/          xxx_task_init() + sw_timer 回调，拥有主循环
service/        srv_* 业务逻辑（FSM/协议/算法）
m_middlewares/  平台无关通用模块（sw_timer/fsm/kfifo/…）——禁止含 HAL
device_drivers/ drv_* HAL 薄封装——唯一允许引用 main.h/CubeMX 句柄的层
Core/           CubeMX 生成代码——只在 USER CODE 块内修改
```
- 全程静态分配（无 malloc），调用者提供句柄内存；`*_init()` 一律无参。
- 写新代码前**先查 `../public_layer/m_middlewares/`**：framework（sw_timer/fsm/event/msg_fifo/daemon）、utils（kfifo/clist）、algorithm（pid/filter/crc）、log.h；`#include "public.h"` 可一次性引入。
- UART 走 per-instance HAL 回调（`USE_HAL_UART_REGISTER_CALLBACKS=1`）。

## 运行时接线事实
- 主循环（`tasks/app_main.c`）：`drv_uart_rx_restart(CH_2/CH_3)` → `sw_timer_tick/task` → `srv_motor_step()` 全速轮询（不走 sw_timer，为满足 300 Hz）。
- `behavior_task_init()` 与 `daemon_task_init()` **当前在 app_main 中被注释**；启用顺序必须 behavior → daemon（daemon 依赖电机句柄已注册）。
- 电机 UART：USART2/3 @ 500000 bps；组 A = USART2（5 电机，ID 1-5），组 B = USART3（4 电机，ID 1-4）。
- **USART1（PC4/PC5, 115200）= 控制台，由 `drv_log_uart` 独占**（日志 TX DMA + RX DMA circular/IDLE → kfifo）；其热路径/回调禁止打印日志。主循环每轮须 `drv_uart_rx_restart` 重新武装 IDLE 接收。
- CAN（FDCAN1, PA11/PA12）：经典 CAN 2.0A @ 1Mbps。测试模块由 `service/srv_motor_test_select.h` 的 `SRV_MOTOR_TEST_SELECT` 宏三选一（`SRV_MOTOR_TEST_HT_TORQUE`=苇熠位置往复、`SRV_MOTOR_TEST_HT_TEMP`=苇熠速度模式、`SRV_MOTOR_TEST_TONGZHI`=良志ODrive位置往复），各模块源文件始终编译、同一时刻只激活一个。旧 CAN FD 上位机协议（0x100/0x101/0x102）保留在 `srv_can`。

## 编码规范
- 风格：WebKit（4 空格缩进，函数 Allman 大括号，控制语句 K&R）+ MISRA C:2012；完整规范见同级 `../stm32_g474_boot/MODULE_CODING_GUIDE.md`。
- 前缀：driver=`drv_`，service=`srv_`，task=`xxx_task_`；类型 `_t`/`_cb_t`；枚举大写蛇形 + 模块前缀。
- 公共 API + 复杂逻辑用中文 Doxygen（`@brief/@param/@return`）；公共函数首行校验 NULL/未初始化/越界。
- Config-in-context：`xxx_config_t` 嵌入 handle，每实例缓冲放 handle 内，禁止模块级 static 单例。
- 编译宏：`STM32G474xx`、`USE_HAL_DRIVER`、`HAL_FLASH_CHIP_STM32G4`、`PRINTF_DISABLE_SUPPORT_FLOAT`、`PRINTF_DISABLE_SUPPORT_EXPONENTIAL`。

## 文档
- `docs/uart_protocol.md`（UART 20B 定长帧）、`docs/can_protocol.md`（旧 CAN FD，已非主用）、`docs/motor_control_api.md`（srv_motor 调度/FSM）、`docs/苇熠电机can协议文档.md`（HT 测试协议）。
