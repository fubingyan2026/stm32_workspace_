# CAN FD 电机控制实现文档

## 1. 概述

本文档说明如何通过 **STM32G474 的 FDCAN1** 控制一台 CAN FD 电机（如 0x10/0x20 帧协议的单轴电机模组），
并描述本工程中用于 **24 小时耐久测试** 的自动正反转循环实现。

- 控制通道：`FDCAN1`（`PA11`=RX / `PA12`=TX）
- 帧格式：CAN FD，标准帧（11-bit ID），64 字节满帧
- 位速率切换（BRS）：仲裁段 **1 Mbps**，数据段 **5 Mbps**
- 实现文件：`service/srv_can.c`（测试模式）、`device_drivers/drv_can.c`（FDCAN 驱动）

---

## 2. 硬件与总线配置

### 2.1 时钟

| 项 | 值 | 说明 |
|----|-----|------|
| HSE | 8 MHz | 外部晶振 |
| SYSCLK | 160 MHz | `PLLN = 40`，`PLLM = DIV1`，`PLLP = DIV2` |
| PCLK1 | 160 MHz | FDCAN 内核时钟来源（`RCC_FDCANCLKSOURCE_PCLK1`） |

> 采用 160 MHz 内核是为了让 FDCAN **数据段能精确到 5.000 Mbps**（160 / 32 = 5）。
> 168 MHz 内核时 5 Mbps 无法整除（168/5 = 33.6 tq），只能得到 4.941 Mbps。

### 2.2 FDCAN 位时序（`Core/Src/fdcan.c`）

| 段 | Prescaler | SJW | TimeSeg1 | TimeSeg2 | 总 tq | 实际波特率 | 采样点 |
|----|-----------|-----|----------|----------|-------|-----------|--------|
| 仲裁段 | 8 | 4 | 15 | 4 | 20 | 1.000 MHz | 80% |
| 数据段 | 2 | 2 | 13 | 2 | 16 | 5.000 MHz | 87.5% |

- `FrameFormat = FDCAN_FRAME_FD_BRS`（启用位速率切换）
- 发送帧头 `BitRateSwitch = FDCAN_BRS_ON`（`drv_can.c`）
- GPIO `PA11/PA12`：`VERY_HIGH` 输出速度 + `PULLUP`（5 Mbps 需要足够沿速率）

> **注意**：5 Mbps 数据段要求对端电机模组也配置为 **CAN FD + BRS，仲裁 1M / 数据 5M**。
> 若模组为 `FD_NO_BRS` 或经典 CAN，将无法解码数据段，导致无 ACK、发送失败、总线静默。

---

## 3. 电机控制协议（帧定义）

三种命令，均为 **64 字节 CAN FD 帧**。使能/失能帧 ID 为 `0x10`，速度帧 ID 为 `0x20`。

### 3.1 使能帧（ID `0x10`）

每 8 字节一组，组内末字节为 `0xFC`，共 8 组：

```
FF FF FF FF FF FF FF FC  × 8   （64 字节）
```

### 3.2 失能帧（ID `0x10`）

每 8 字节一组，组内末字节为 `0xFD`：

```
FF FF FF FF FF FF FF FD  × 8   （64 字节）
```

> 当前测试循环**不使用失能帧**（停留/停止用速度 0 帧代替）；失能帧格式保留备用。

### 3.3 速度帧（ID `0x20`）

8 字节模式重复 8 次填满 64 字节。**前 2 字节（Byte0-1）为速度相关字段**，方向不同则取值不同：

| 模式 | 8B 模式（×8） | 说明 |
|------|---------------|------|
| 正转 | `7F FF BF F0 00 33 37 FF` | 正向速度 |
| 反转 | `7F FF 3F F0 00 33 37 FF` | 反向速度 |
| 速度 0（停止） | `7F FF 7F F0 00 05 17 FF` | 停止，**不**失能 |

对应代码中的三个模式数组（`srv_can.c`）：

```c
static const uint8_t pattern[8]        = { 0x7F, 0xFF, 0xBF, 0xF0, 0x00, 0x33, 0x37, 0xFF }; // 正转
static const uint8_t pattern_invert[8] = { 0x7F, 0xFF, 0x3F, 0xF0, 0x00, 0x33, 0x37, 0xFF }; // 反转
static const uint8_t pattern_zero[8]   = { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x05, 0x17, 0xFF }; // 速度0
```

> 修改速度或方向时，只需改动这 3 个数组的字节值（须与电机模组协议一致）。

---

## 4. 测试循环时序（24h 耐久测试）

测试启动后按以下阶段无限循环，**24 小时后自动停止**：

| 阶段 | 时长 | 发送内容 | 发送频率 |
|------|------|----------|----------|
| 正转 | 30 s | `pattern`（正转速度帧） | 每 1 s |
| 停留 | 30 s | `pattern_zero`（速度 0 帧） | 每 1 s |
| 反转 | 30 s | `pattern_invert`（反转速度帧） | 每 1 s |
| 停留 | 30 s | `pattern_zero`（速度 0 帧） | 每 1 s |

- 一个循环 = 120 s；24 h ≈ 720 个循环
- **每次进入正转/反转阶段时，先补发一次使能帧**（`0x10` + `FC`），确保停留后电机仍可响应速度命令
- **24 h 到点**：发送一次速度 0 帧，停止循环
- 上电自动启动（`SRV_CAN_TEST_AUTO_START = 1`）

---

## 5. 固件实现流程

### 5.1 初始化调用链

```
main()
 └─ SystemClock_Config()          // SYSCLK = 160 MHz (PLLN=40)
 └─ MX_FDCAN1_Init()              // FDCAN1: FD_BRS, 1M/5M
     └─ HAL_FDCAN_MspInit()       // 时钟源 PCLK1, GPIO VERY_HIGH+PULLUP
 └─ app_main()
     └─ can_task_init()
         ├─ drv_can_init()        // FDCAN Start + RX 中断 + 收发器 STB 拉低
         └─ srv_can_init()
             └─ srv_can_test_start()   // 发使能帧，启动 10ms 周期定时器
```

### 5.2 周期驱动（`tasks/can_task.c`）

`can_task` 注册一个 **10 ms** 的 `sw_timer`，回调中调用：

```c
srv_can_process();   // 处理上位机 0x100 控制帧（正常模式）
srv_can_test_step(); // 测试模式：驱动阶段循环 + 周期发送（本测试）
```

### 5.3 测试模式核心函数（`service/srv_can.c`）

| 函数 | 职责 |
|------|------|
| `srv_can_test_start()` | 置运行标志、阶段=正转、记录起始时间、发一次使能帧 |
| `srv_can_test_step()` | 每 10 ms：24h 判断 → 阶段切换 → 按阶段发对应帧 |
| `srv_can_test_stop()` | 停止循环，发一次速度 0 帧 |
| `srv_can_test_send_cmd(fill)` | 发 `0x10` 使能/失能帧（`0xFC`=使能，`0xFD`=失能） |
| `srv_can_test_send_speed(pat)` | 发 `0x20` 速度帧（按传入 8B 模式 ×8） |

### 5.4 关键常量（改参数改这里）

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `SRV_CAN_TEST_AUTO_START` | `1` | 1=上电自动启动测试；0=手动调 `srv_can_test_start()` |
| `SRV_CAN_TEST_ID_EN` | `0x10` | 使能/失能帧 ID |
| `SRV_CAN_TEST_ID_SPD` | `0x20` | 速度帧 ID |
| `SRV_CAN_TEST_SPD_PERIOD_MS` | `1000` | 速度帧发送周期 (ms) |
| `SRV_CAN_TEST_PHASE_MS` | `30000` | 每阶段时长 (ms)，30 s |
| `SRV_CAN_TEST_DURATION_MS` | `86400000` | 测试总时长 (ms)，24 h |
| `pattern` / `pattern_invert` / `pattern_zero` | — | 正转/反转/速度0 的 8B 模式 |

---

## 6. 调试与验证

1. **CAN 分析仪**：抓 `0x10`/`0x20` 帧，确认帧交替出现、方向字节按循环变化。
2. **RTT 日志**（SEGGER RTT）：`drv_can` 在发送失败时会打印：
   - `ch1 tx fail xN` —— 无 ACK，帧没发出去
   - `ERROR-PASSIVE` —— 错误帧累积（多为数据段不匹配）
   - `txfifo FULL ... lec=x` —— TX FIFO 打满，所有帧被丢弃
3. **无数据排查**：若总线完全静默，优先检查电机模组的 CAN FD/BRS/波特率配置是否与本端（1M/5M）一致，以及总线终端电阻、接线长度（5 Mbps 对物理层要求高）。

---

## 7. 相关文件

| 文件 | 作用 |
|------|------|
| `service/srv_can.c` / `srv_can.h` | 测试模式 + CAN 协议打包/解析 |
| `tasks/can_task.c` | 10 ms 周期任务，驱动 `srv_can_test_step()` |
| `device_drivers/drv_can.c` / `drv_can.h` | FDCAN 驱动（发送/接收/错误诊断） |
| `Core/Src/fdcan.c` | FDCAN1 初始化（FD_BRS 1M/5M） |
| `Core/Src/main.c` | 系统时钟（SYSCLK 160 MHz） |
