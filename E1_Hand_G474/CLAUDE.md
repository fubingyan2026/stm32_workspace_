# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**E1_Hand_G474** — 基于 STM32G474 (Cortex-M4, 128KB SRAM) 的 9 自由度灵巧手嵌入式固件，裸机（无 RTOS），C11，CMake 构建。

**关键设计决策**：
- 控制板经 FDCAN1 接收上位机指令 → 转换 2 路电机 UART (USART2/3, 500000 bps) 广播控制 9 个关节电机（5+4 分组）
- **USART1 为独立控制台串口**（115200 bps，`drv_log_uart` 独占）：日志 DMA 输出 + 命令 DMA circular/IDLE 接收
- 控制频率目标 300 Hz；`srv_motor_step()` 在主循环全速轮询（不走 sw_timer），DMA 空闲即发帧
- 全程静态分配，无 malloc
- **共享层架构**：中间件 (`m_middlewares`) 与通用服务/任务 (`service`/`task`) 原件维护在 `../public_layer/`，CMake 直接 `add_subdirectory(../public_layer/m_middlewares)` + 显式列源文件引用

> ⚠️ 路径注意：本地目录 `public_layer/` **当前不存在**（无 junction 目录），构建直接引用同级目录 `../public_layer/`。共享代码一律编辑 `../public_layer/` 里的原件。

## 代码生成黄金规则

### 优先复用 `public_layer/m_middlewares/` 下已有的通用模块，禁止重复造轮子

在新增任何功能之前，**必须优先检查 `../public_layer/m_middlewares/` 中是否已有可复用的模块**。包括：

- **框架原语**：定时器（`sw_timer`）、状态机（`fsm`）、事件标志（`event`）、消息队列（`msg_fifo`）、看门狗（`daemon`）
- **数据结构**：无锁 FIFO（`kfifo`）、侵入式链表（`clist`）
- **算法**：PID/云台 PID、MIT 控制器、CRC、PT1 滤波、PLL 锁相环、数学工具（三角函数/插值/限幅）
- **协议**：协议打包器（`protocol_packer`）、协议解析器（`protocol_parser`）
- **日志**：带时间戳的 log 模块（`kfifo` 缓冲 + 多级宏 + Flash 落盘 sink）
- **输入**：按键扫描（`key_base`）
- **共享服务/任务**：`srv_signal`（LED/蜂鸣器）、`srv_log_flash`（日志落 Flash）、`log_task`（UART/RTT 输出）、`boot_task`（升级）

> `public_layer/m_middlewares/public.h` 一行 `#include` 即可引入所有中间件。

### Build & Development

### Prerequisites
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- CMake >= 3.22
- 依赖同级共享层 `../public_layer/`（缺失时构建失败）

### Build commands

优先使用 `build.bat` 一键构建（自动检测 ARM GCC 工具链、CMake、Ninja）：

```bat
# Debug 构建（默认）
build.bat

# Release 构建
build.bat -t Release

# 从其他目录指定项目路径
build.bat D:\path\to\project -t Debug
```

手动构建（需自行配置工具链环境变量）：

```bash
# 使用 CMakePresets（推荐手动方式）
cmake --preset Debug && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release

# 或完整命令
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -GNinja
ninja -C build/Debug
```

构建产物（位于 `build/{Debug|Release}/`）：
- `E1_Hand_G474.elf` — ELF 可执行
- `E1_Hand_G474.hex` — Intel HEX
- `E1_Hand_G474.bin` — 纯二进制

### MCU 信息
- STM32G474 (Cortex-M4, FPv4-SP)
- 编译宏：`STM32G474xx`, `USE_HAL_DRIVER`, `HAL_FLASH_CHIP_STM32G4`, `PRINTF_DISABLE_SUPPORT_FLOAT`, `PRINTF_DISABLE_SUPPORT_EXPONENTIAL`
- 启动文件：`startup_stm32g474xx.s`
- Flash: 512 KB, SRAM: 128 KB

## 架构：五层分层结构

```
tasks/           应用编排层    xxx_task_init() / sw_timer       ← 拥有 sw_timer、注入回调
service/         业务逻辑层    srv_*  FSM / 协议 / 算法          ← 硬件无关或绑定单一 driver
m_middlewares/   通用中间件    sw_timer/fsm/msg_fifo/滤波…      ← 平台无关、可复用、静态分配
device_drivers/  设备驱动层    drv_*  HAL 薄封装 + 引脚自包含    ← 唯一接触 main.h/CubeMX 句柄
Core/ (CubeMX)   HAL 生成层    main.h / hfdcan1 / htimX / …    ← 只在 USER CODE 块内改
```

**依赖规则**：只能向下，禁止向上或跨层反向。

| 层 | 可 include | 禁止 include |
|----|-----------|-------------|
| tasks | service, device_drivers, middleware, Core | — |
| service | middleware、其绑定的 device_drivers、Core 类型 | 其他 service（除非明确数据依赖）、tasks |
| middleware | 其他 middleware、C 标准库 | device_drivers, service, tasks, Core/HAL |
| device_drivers | Core（main.h/HAL/外设句柄）、middleware | service, tasks |

### 关键原则
- **middleware 必须平台无关**（不含 `stm32g4xx_hal.h`）
- **device_drivers 是唯一允许引用 `main.h` 引脚宏和 CubeMX 句柄的层**
- **全程静态分配**（无 malloc），调用者提供句柄内存
- **`init` 一律无参**（driver 层内部包含 HAL 句柄表或宏）
- **UART 回调采用 per-instance 注册**：`USE_HAL_UART_REGISTER_CALLBACKS=1` 已开启，`drv_uart` 接管 `huart2/3`，`drv_log_uart` 独占 `huart1`，互不冲突

## 目录结构

```
Core/                   CubeMX 生成代码（Inc + Src）
  Inc/main.h            引脚宏定义、外设句柄全局声明
  Inc/fdcan.h / tim.h / usart.h / gpio.h / dma.h
  Src/main.c            MX_xxx_Init() → app_main()
  Src/stm32g4xx_it.c    中断入口
device_drivers/         设备驱动层（drv_ 前缀）+ hal_flash（从 middleware 迁入）
  drv_can.c/h           CAN 驱动 (FDCAN1, PA11/PA12) — 支持经典 CAN + CAN FD，DLC 自动编解码
  drv_uart.c/h          UART 通用驱动（USART2/3 — PA2/PA3, PB10/PB11, 500000 bps）
                        TX: DMA normal 双状态检忙；RX: DMA normal + IDLE 事件 ping-pong 双缓冲
                        CH_1 (USART1) 已剥离给 drv_log_uart，s_inst 置空跳过
  drv_log_uart.c/h      USART1 控制台驱动（PC4/PC5, 115200 bps）— 日志 TX DMA + RX DMA
                        circular + IDLE → kfifo；per-instance HAL 回调，热路径禁止打日志
  drv_systick.c/h       SysTick 延时/时间戳 (delay_us/millis/micros)
  drv_hw_timer.c/h      硬件定时器
  drv_stm32g4_flash.c/h  STM32G4 Flash 操作接口
  hal_flash/            Flash 抽象层（位于 ../public_layer/device_drivers/hal_flash）
service/                业务逻辑层（本地 + public_layer/service 共同编译）
  srv_motor.c/h         电机串口通信服务 — 多实例（每电机一个 handle 注册），广播控制 + 分时轮询
                        组A=USART2 (DRV_UART_CH_2, 5 电机 ID 1-5)，组B=USART3 (DRV_UART_CH_3, 4 电机 ID 1-4)
                        全局索引 0-4=A, 5-8=B；电机名 "motorA1".."motorB4"（daemon 注册名单一来源）
  srv_motor_behavior.c/h 电机行为协调 FSM — 单 fsm 管理 9 电机组生命周期
                        （OFFLINE→IDLE→CALIB→RUNNING⇄FAULT，CALIB=上电到位找零；在线判定来自 daemon）
  srv_can.c/h           旧 CAN FD 上位机协议 — 控制帧 (0x100) 解析 + 反馈/状态帧打包 (0x101/0x102)；
                        srv_can_on_rx 先将测试协议帧路由给 srv_ht_temp_test
  srv_ht_temp_test.c/h  苇熠(HT) 伺服执行器 CAN 测试协议 — 经典 CAN 2.0A 1Mbps，总线扫描 (0x00 握手) +
                        使能/速度/报警/电压查询循环；独立于 srv_motor 体系
tasks/                  应用编排层
  app_main.c/h          主入口：init 顺序 → for(;;) { uart rx restart; sw_timer_tick/task; srv_motor_step(); }
  can_task.c/h          CAN 任务（10ms 周期 sw_timer）— drv_can_poll_status + srv_can_process + 测试步进
  led_task.c/h          LED 状态指示任务（10ms 刷新）
  behavior_task.c/h     电机行为任务（1ms sw_timer）— srv_motor_init + srv_motor_behavior_init，驱动
                        srv_motor_behavior_step（当前在 app_main 中被注释禁用）
  daemon_task.c/h       守护任务（1ms sw_timer）— 9 电机反馈超时看门狗（daemon 中间件），掉线/恢复仅打日志
                        （当前在 app_main 中被注释禁用；依赖 behavior_task 先完成电机注册）
../public_layer/        共享层（同级目录，原件所在地）
  m_middlewares/        通用中间件（sw_timer/fsm/daemon/kfifo/PID/log/…，独立 CMakeLists.txt）
  service/              srv_signal (LED/蜂鸣器), srv_log_flash (日志 Flash 持久化), boot/
  task/                 log_task (UART/RTT 日志输出), boot_task (固件升级)
  device_drivers/       hal_flash
docs/                   协议文档
  can_protocol.md       CAN FD 上位机协议规范 (V1.1.0) — 当前已非主用（见 srv_can 测试模式）
  uart_protocol.md      关节模组通讯协议 (UART 20B 定长帧)
  motor_control_api.md  srv_motor API 参考 — FSM 状态转移矩阵、帧调度时序、CRC 覆盖区间
  plan_task.md          规划文档/设计笔记
```

## 核心中间件速查表

**新增代码前先查下表**——看 `m_middlewares/` 是否有可直接使用的模块：

### 框架原语 (`m_middlewares/framework/`)

| 模块 | Include | API 前缀 | 能力 |
|------|---------|----------|------|
| `sw_timer` | `"sw_timer.h"` | `sw_timer_` | 软件定时器，3 优先级（HIGH ISR / NORMAL LOW 主循环），静态分配 |
| `fsm` | `"fsm.h"` | `fsm_` | 扁平转移矩阵状态机（state² guard 表）+ entry/exit 回调 + user_data |
| `event` | `"event.h"` | `event_` | ISR 安全的 32-bit 事件标志，ISR↔主循环生产者消费者 |
| `msg_fifo` | `"msg_fifo.h"` | `msg_fifo_` | 基于 kfifo 的定长消息队列，单生产者单消费者 |
| `daemon` | `"daemon.h"` | `daemon_` | 任务看门狗：注册名 + 超时 + 掉线回调 + 启动初始等待 |

### 数据结构 (`m_middlewares/utils/`)

| 模块 | Include | API 前缀 | 能力 |
|------|---------|----------|------|
| `kfifo` | `"kfifo.h"` | `kfifo_` | 无锁 2 的幂字节 FIFO（所有 FIFO/队列的底层依赖） |
| `clist` | `"clist.h"` | `clist_` | 侵入式循环双向链表（多实例管理） |

### 算法 (`m_middlewares/algorithm/`)

| 模块 | 路径 | 能力 |
|------|------|------|
| PID | `controller/pid.h` | 位置式/增量式 PID |
| 云台 PID | `controller/gimbal_pid.h` | 云台专用 PID |
| MIT 控制器 | `controller/mit.h` | 力矩控制 |
| CRC | `crc.h` | CRC8/CRC16 计算/校验/追加 |
| PT1 滤波 | `filter/filter.h` | 一阶低通 + 双二阶 IIR + Slew Rate |
| 数学工具 | `math/maths.h`, `utils_math.h`, `utils.h` | 三角函数、插值、限幅、常用数学 |
| PLL | `pll/pll.h` | 锁相环 |

### 其他 (`m_middlewares/`)

| 模块 | Include | 能力 |
|------|---------|------|
| 日志 | `"log.h"` | kfifo 缓冲 + 时间戳 + 多级宏 `LOG_E/W/I/D/T(tag, ...)` + `LOG_HEXDUMP`；`LOG_ENABLED=0` 编译期剥离；可注册 Flash sink（srv_log_flash） |
| 协议打包 | `"protocol_packer.h"` | 结构化协议打包器 |
| 协议解析 | `"protocol_parser.h"` | 结构化协议解析器 |
| 按键 | `"key_base.h"` | 按键扫描/消抖/连击识别 |

### 第三方库 (`m_middlewares/Third_Party/`)

| 库 | 用途 | 集成方式 |
|----|------|----------|
| CmBacktrace | ARM Cortex HardFault 自动诊断（栈回溯/寄存器/调用栈） | 需实现 `cmb_printf` 输出重定向到 log/UART |
| SEGGER_RTT | J-Link 实时终端输出 | `log_task` 集成为第二输出后端（`LOG_OUTPUT_RTT`），默认 UART |
| lwmem | 轻量动态内存管理 | 已入库，当前业务代码未使用（项目原则仍是静态分配） |
| mpaland_printf | 轻量 printf | 编译宏禁用浮点输出（G474 无硬件双精度） |

## 初始化顺序

在 `app_main()` 中按依赖顺序调用（**当前实际接线**）：

1. `delay_init()` → SysTick 时基
2. `drv_uart_init()` → 电机 UART 驱动（USART2/3；USART1 已被 drv_log_uart 接管，跳过）
3. `drv_log_uart_init()` → USART1 控制台驱动（DMA circular + kfifo）
4. `log_task_init()` → 日志模块 + 输出任务（10ms sw_timer，默认 UART 输出，可切 RTT）— 来自 `../public_layer/task/log_task.c`
5. `srv_log_flash_init()` → 警告/错误日志 Flash 持久化（依赖 log 已初始化）— 来自 `../public_layer/service/`
6. `can_task_init()` → CAN 通信（注册 RX 回调，启动 10ms 定时器）
7. `led_task_init()` → LED
8. ~~`behavior_task_init()` / `daemon_task_init()`~~ → **当前被注释禁用**（过渡期），启用顺序为：behavior_task → daemon_task（daemon 依赖电机句柄已注册）

主循环：
```c
for (;;) {
    drv_uart_rx_restart(DRV_UART_CH_2); // 检查并重启电机 UART 的 RX DMA
    drv_uart_rx_restart(DRV_UART_CH_3);
    sw_timer_tick(millis());            // 更新定时器时基
    sw_timer_task();                    // 派发到期 NORMAL/LOW 定时器
    srv_motor_step();                   // 全速轮询电机控制/反馈（无固定定时器）
}
```

> **注意**：`srv_motor_step()` 是唯一不走 sw_timer 的周期性工作 —— 直接在主循环全速轮询。
> 多实例注册的电机按组构建广播帧（位置+速度）并分时轮询反馈，通过 `drv_uart_is_tx_busy()` 控制发送间隔。
> 这是为满足 300 Hz 控制频率需求的设计决策。详细调度时序见 [docs/motor_control_api.md](docs/motor_control_api.md)。

## 通信架构概览

```
上位机 (CAN, FDCAN1)
    │
    ├─ 主用（测试模式）：经典 CAN 2.0A @ 1Mbps，苇熠伺服执行器协议（srv_ht_temp_test）
    │      CAN-ID 低 8 位=设备地址，≤8B，srv_ht_temp_test_step() 每 10ms 驱动（扫 ID/使能/正反转/查报警）
    │
    └─ 旧 CAN FD 上位机协议（保留，can_task 中上报暂注释）：
          0x100 控制帧 (55B) → srv_can_on_rx() → srv_can_process() → srv_motor_behavior_set_setpoint()
          0x101 反馈帧 (64B, 100ms)  ← srv_can_send_feedback()
          0x102 状态帧 (48B, 500ms)  ← srv_can_send_status()

控制板内部:
    srv_motor_behavior (FSM) → srv_motor_set_setpoint() → UART 广播/标准帧 (500000 bps)
                                                              ├─ USART2 组A (5 电机, ID 1-5)
                                                              └─ USART3 组B (4 电机, ID 1-4)
控制台（调试/日志）:
    USART1 (115200 bps) — 日志 DMA TX + 控制台命令 DMA circular/IDLE RX (drv_log_uart)
```

协议细节见 [docs/can_protocol.md](docs/can_protocol.md)（CAN FD 旧协议）、[docs/uart_protocol.md](docs/uart_protocol.md) 和 [docs/motor_control_api.md](docs/motor_control_api.md)。苇熠测试协议组帧定义在 `srv_ht_temp_test.c` 头部注释，协议规范见 [docs/苇熠电机can协议文档.md](docs/苇熠电机can协议文档.md)。

## 编码规范

### 编码规范文档

`agent_standards/` 目录当前**不存在**于本仓库。代码规范参照同级 boot 工程 [../stm32_g474_boot/MODULE_CODING_GUIDE.md](../stm32_g474_boot/MODULE_CODING_GUIDE.md)（WebKit + MISRA C:2012 风格）。

核心要点速查：

| 规范 | 要求 |
|------|------|
| **代码风格** | WebKit（4 空格缩进，函数 Allman 大括号，控制语句 K&R） + MISRA C:2012 |
| **命名前缀** | driver=`drv_`，service=`srv_`，task=`xxx_task_`，middleware=模块名 |
| **类型后缀** | `_t` 结构体/枚举，`_cb_t` 回调 |
| **枚举值** | 大写蛇形 + 模块前缀（`MODULE_NAME_OK = 0`, 负数为错误） |
| **注释** | 公共 API + 复杂逻辑用中文 Doxygen（`@brief/@param/@return`） |
| **文件结构** | 头注释 → Includes → 私有常量 → 私有变量 → 私有原型 → 导出函数 → 私有函数 |
| **Config-in-context** | `xxx_config_t`（含回调）嵌入 `xxx_context_t`/handle，运行时状态分离 |
| **参数校验** | 公共函数首行校验 NULL / 未初始化 / 越界 |
| **内存** | 全程静态分配（无 malloc），调用者提供句柄内存 |
| **多实例** | 每个实例独立的缓冲区/队列放入句柄结构体，禁止模块级 static 单例 |
| **服务设计** | 风格 A（回调注入，共享总线/异构）或风格 B（直连 drv_，1:1 绑定）|
| **热路径禁日志** | ISR / 高速收发路径禁止打印日志（如 drv_log_uart 的 send/回调），避免回灌自引用 |

### 硬件 / 引脚参考

引脚配置以 `Core/Inc/main.h` 与 `E1_Hand_G474.ioc` 中的 CubeMX 宏为准。当前串口分配：
- **USART1** PC4(TX)/PC5(RX) @ 115200 — 控制台/日志（drv_log_uart）
- **USART2** PA2(TX)/PA3(RX) @ 500000 — 电机组 A
- **USART3** PB10(TX)/PB11(RX) @ 500000 — 电机组 B
- **FDCAN1** PA11(RX)/PA12(TX) — 上位机通信
