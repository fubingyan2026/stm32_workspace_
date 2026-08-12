# CAN 电机控制实现文档

## 1. 概述

本文档说明如何通过 **STM32G474 的 FDCAN1** 控制一台 **苇熠伺服执行器（SCA）**，
并描述本工程中用于 **24 小时耐久测试** 的自动正反转循环实现。

- 控制通道：`FDCAN1`（`PA11`=RX / `PA12`=TX）
- 帧格式：**经典 CAN 2.0A**，标准帧（11-bit ID，低 8 位 = 设备地址），每帧 ≤ 8 字节
- 波特率：**1 Mbps**（FDCAN 仲裁段速率，经典 CAN 帧即用此速率）
- 协议依据：`docs/苇熠电机can协议文档.md`（基于 V1.04）
- 启动流程：**先扫描总线电机 ID**（握手 `0x00`），只对检测到的电机下发控制命令
- 日志：通过 `log.c` 日志库输出扫描/检测/阶段/停止等详细事件日志（SEGGER RTT 默认通道）
- 实现文件：`service/srv_can.c`（测试模式）、`device_drivers/drv_can.c`（FDCAN 驱动）

---

## 2. 硬件与总线配置

### 2.1 时钟

| 项 | 值 | 说明 |
|----|-----|------|
| HSE | 8 MHz | 外部晶振 |
| SYSCLK | 160 MHz | `PLLN = 40`，`PLLM = DIV1`，`PLLP = DIV2` |
| PCLK1 | 160 MHz | FDCAN 内核时钟来源（`RCC_FDCANCLKSOURCE_PCLK1`） |

### 2.2 FDCAN 位时序（`Core/Src/fdcan.c`）

| 段 | Prescaler | SJW | TimeSeg1 | TimeSeg2 | 总 tq | 实际波特率 | 采样点 |
|----|-----------|-----|----------|----------|-------|-----------|--------|
| 仲裁段 | 8 | 4 | 15 | 4 | 20 | **1.000 MHz** | 80% |
| 数据段 | 2 | 2 | 13 | 2 | 16 | 5.000 MHz | 87.5% |

> **注意**：苇熠协议要求 **1 Mbit/s 经典 CAN**。本工程 `FrameFormat = FDCAN_FRAME_FD_BRS`，
> 但发送时每帧通过 `drv_can_msg_t.is_fd = false` 指定为 **经典 CAN 帧**（`FDFormat = FDCAN_CLASSIC_CAN`），
> 经典帧只走仲裁段 1.000 MHz，与协议一致。数据段 5 Mbps 仅用于 FD 帧。

---

## 3. 苇熠电机控制协议（帧定义）

### 3.1 帧格式

- **CAN-ID**：设备地址（低 8 位），由启动扫描自动发现（`0x01`~`0x3F`，`0x00` 为广播且无返回）。
- **数据负载**：`data[0] = 指令符`，`data[1..] = 参数`，多字节参数为**大端**（高字节在前）。
- **DLC** = 1 + 参数字节数（≤ 8）。

### 3.2 用到的指令（详见协议文档 §6）

| 功能 | 指令 | 参数 | DLC | 帧数据示例（CAN-ID=0x02） |
|------|------|------|-----|---------------------------|
| 握手（扫描探测） | `0x00` | — | 1 | `02 00`（电机返回 `[00][01]`） |
| 使能 | `0x2A` | `0x01` | 2 | `02 2A 01` |
| 失能 | `0x2A` | `0x00` | 2 | `02 2A 00` |
| 设置速度模式 | `0x07` | `0x02` | 2 | `02 07 02` |
| 设置速度 = 100 RPM | `0x09` | IQ24 大端 | 5 | `02 09 00 04 44 44` |
| 设置速度 = -100 RPM | `0x09` | IQ24 大端 | 5 | `02 09 FF FB BB BD` |
| 设置速度 = 0（停止） | `0x09` | 0 | 5 | `02 09 00 00 00 00` |
| 打开抱闸（可选） | `0xF4` | `0x01 0x00` | 3 | `02 F4 01 00` |
| 查询报警 | `0xFF` | — | 1 | `02 FF`（**实测返回 `[FF][4B 大端报警码]`，5B**，与文档 §6.1.2"返回3字节"不符） |
| 读取供电电压 | `0x87` | — | 1 | `02 87`（**实测返回 `[87][状态]`，2B 写命令应答，不含电压数据，已禁用**） |

> **实测结论**：本电机 0xFF 报警应答为 5 字节（读取指令3 的 IQ24 格式），解析按 `[0xFF][data[1..4] 大端]`；
> 0x87 在本电机固件里不返回电压（返回写命令应答），电压读取功能已通过 `SRV_CAN_TEST_VOLTAGE_QUERY_ENABLE=0` 关闭。

> 表中帧数据 = `[CAN-ID][DLC][指令][参数…]`，供 CAN 分析仪比对。
> 实际发送时 CAN-ID 在帧头，数据负载为 `[指令][参数…]`。
> 帧示例以电机地址 0x02 为例；扫描时地址由总线探测结果决定。

### 3.3 IQ24 转换（速度）

- 速度满量程 = **6000 RPM**，归一化 `IQ = rpm / 6000 × 2²⁴`（取整）。
- 正转 = 正 IQ 值，反转 = 负 IQ 值（补码），0 = 停止（不失能）。
- 示例：`100 RPM → 0x00044444`；`-100 RPM → 0xFFFBBBBD`。

---

## 4. 测试循环时序（24h 耐久测试）

测试启动后**先扫描总线电机 ID**，再对检测到的电机无限循环执行正反转，**24 小时后自动停止**：

### 4.1 阶段 0：扫描总线电机 ID

逐地址发送握手 `0x00`（探测 `0x01`~`0x3F`，每地址间隔 `SRV_CAN_SCAN_PROBE_PERIOD_MS`），
收到应答帧（CAN-ID=设备地址）即记为已检测电机。
扫描完成后，对检测到的电机下发控制命令；**若未检测到任何电机，回退到默认地址
`SRV_CAN_TEST_DEFAULT_MOTOR_ADDR` 继续控制**（打 WARNING 日志，不中止测试）。

### 4.2 阶段 1：正转/停留/反转/停留循环

| 阶段 | 时长 | 发送内容 | 发送频率 |
|------|------|----------|----------|
| 正转 | `SRV_CAN_TEST_PHASE_MS` | 速度 +`SRV_CAN_TEST_SPEED_RPM` | 每 1 s |
| 停留 | `SRV_CAN_TEST_PHASE_MS` | 速度 0（不失能） | 每 1 s |
| 反转 | `SRV_CAN_TEST_PHASE_MS` | 速度 -`SRV_CAN_TEST_SPEED_RPM` | 每 1 s |
| 停留 | `SRV_CAN_TEST_PHASE_MS` | 速度 0（不失能） | 每 1 s |

- 一个循环 = 4 × `SRV_CAN_TEST_PHASE_MS`；默认每阶段 6 s（循环 24 s）
- **扫描完成时**：使能 → 速度模式 → 正转速度帧（可选：打开抱闸，`SRV_CAN_TEST_RELEASE_BRAKE`）
- **每次进入正转/反转阶段时**：补发使能 + 速度模式，确保停留后电机仍可响应速度命令
- **控制开始 1 s 内**：每 100 ms 补发使能 + 速度模式（首帧可能因 TX FIFO 未就绪被丢弃）
- **报警监控**：每 1 s 主动查询电机报警（`0xFF`，电机报警需主动查询、不会自动上报）；
  实测应答为 5 字节 `[FF][4B 大端报警码]`（24 位码），**只在报警状态变化时打印**：
  报警出现/变化/消除各打印一次（WARN + 逐位解码），持续报警与稳态无报警均静默（错误码见协议文档 §8）
- **电机无响应检测**：报警查询与速度/控制帧**错开**发送，避免多包同时发出导致丢应答；
  **超过 3 s 未收到电机任何帧**（任意指令的应答均算）打 WARN（`长时间无响应`，掉线或断电）
- **电压监控**：已禁用（`SRV_CAN_TEST_VOLTAGE_QUERY_ENABLE=0`）。实测 0x87 在本电机返回
  `[87][状态]`（写命令应答），不含电压数据；且 0x87 在写命令形态下是"设置 Home 值"，
  自动轮询有风险。欠压检测改由 0xFF 报警查询承担
- **24 h 到点**：发送速度 0 帧 + **失能**（`0x2A 00`，防止零位丢失）
- 上电自动启动（`SRV_CAN_TEST_AUTO_START = 1`）

---

## 5. 固件实现流程

### 5.1 初始化调用链

```
main()
 └─ SystemClock_Config()          // SYSCLK = 160 MHz (PLLN=40)
 └─ MX_FDCAN1_Init()              // FDCAN1: FD_BRS, 仲裁 1M
     └─ HAL_FDCAN_MspInit()       // 时钟源 PCLK1, GPIO VERY_HIGH+PULLUP
 └─ app_main()
     └─ can_task_init()
         ├─ drv_can_init()        // FDCAN Start + RX 中断 + 收发器 STB 拉低
         └─ srv_can_init()
             └─ srv_can_test_start()   // 使能 + 速度模式 + 正转，启动 10ms 周期定时器
```

### 5.2 周期驱动（`tasks/can_task.c`）

`can_task` 注册一个 **10 ms** 的 `sw_timer`，回调中调用：

```c
srv_can_process();   // 处理上位机 0x100 控制帧（旧 CAN FD 协议，当前未用）
srv_can_test_step(); // 测试模式：驱动阶段循环 + 周期发送（本测试）
```

### 5.3 测试模式核心函数（`service/srv_can.c`）

| 函数 | 职责 |
|------|------|
| `srv_can_test_start()` | 置运行标志、记录起始时间、进入**扫描阶段** |
| `srv_can_test_step()` | 每 10 ms：24h 判断 → 扫描阶段逐地址探测 → 阶段切换 → 按阶段发对应速度帧 |
| `srv_can_test_stop()` | 停止循环，发速度 0 + 失能（发往所有检测到的电机） |
| `srv_can_test_scan_record(addr)` | 记录握手应答的电机地址（去重，ISR 中调用） |
| `srv_can_test_scan_done()` | 扫描结束：打印结果，使能+速度模式+正转，进入正转阶段 |
| `srv_can_test_send_handshake(addr)` | 发 `0x00` 握手探测帧（经典 CAN 1B） |
| `srv_can_test_send_enable(addr, enable)` | 发 `0x2A` 使能/失能帧（经典 CAN 2B） |
| `srv_can_test_set_speed_mode(addr)` | 发 `0x07 02` 速度模式帧（经典 CAN 2B） |
| `srv_can_test_send_speed(addr, rpm)` | 发 `0x09` 速度设定帧（IQ24，经典 CAN 5B） |
| `srv_can_test_cmd_*_all(...)` | 批量下发到所有检测到的电机 |
| `srv_can_test_send_release_brake(addr)` | 发 `0xF4 0100` 打开抱闸帧（经典 CAN 3B，可选） |
| `srv_can_test_send_query_alarm(addr)` | 发 `0xFF` 报警查询帧（经典 CAN 1B） |
| `srv_can_test_query_alarm_all()` | 对所有检测到的电机发送报警查询 |
| `srv_can_alarm_print(addr, code)` | 打印报警查询结果（解码 24 位错误码） |
| `srv_can_test_send_query_voltage(addr)` | 发 `0x87` 电压读取帧（`VOLTAGE_QUERY_ENABLE=1` 时编译） |
| `srv_can_test_query_voltage_all()` | 对所有检测到的电机发送电压读取（同上） |
| `srv_can_voltage_print(addr, volt)` | 打印供电电压（×100 单位 V，同上） |

### 5.4 关键常量（改参数改这里，`service/srv_can.c`）

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `SRV_CAN_TEST_AUTO_START` | `1` | 1=上电自动启动测试；0=手动调 `srv_can_test_start()` |
| `SRV_CAN_TEST_SPEED_RPM` | `600` | 正转/反转转速 (RPM)，满量程 6000 |
| `SRV_CAN_TEST_CMD_PERIOD_MS` | `1000` | 速度帧发送周期 (ms) |
| `SRV_CAN_TEST_ALARM_PERIOD_MS` | `1000` | 报警查询周期 (ms)，主动 0xFF 查询 |
| `SRV_CAN_TEST_ALARM_OFFSET_MS` | `500` | 报警查询相对控制帧的时间偏移 (ms)，错开防冲突 |
| `SRV_CAN_TEST_NORESP_PERIOD_MS` | `3000` | 电机无响应判定周期 (ms)，超时判掉线/断电 |
| `SRV_CAN_TEST_VOLTAGE_QUERY_ENABLE` | `0` | 1=周期读供电电压（0x87）；0=关闭（实测 0x87 不返回电压） |
| `SRV_CAN_TEST_PHASE_MS` | `6000` | 每阶段时长 (ms)，6 s |
| `SRV_CAN_TEST_DURATION_MS` | `86400000` | 测试总时长 (ms)，24 h |
| `SRV_CAN_TEST_RELEASE_BRAKE` | `0` | 1=使能后/进入旋转阶段时打开抱闸；0=不处理 |
| `SRV_CAN_SCAN_ADDR_MIN` | `0x01` | 扫描地址下限（含） |
| `SRV_CAN_SCAN_ADDR_MAX` | `0x3F` | 扫描地址上限（含），协议推荐地址范围 |
| `SRV_CAN_SCAN_PROBE_PERIOD_MS` | `20` | 每地址探测间隔 (ms)，留出握手应答时间 |
| `SRV_CAN_MAX_MOTORS` | `63` | 最多同时控制的电机数 |
| `SRV_CAN_TEST_DEFAULT_MOTOR_ADDR` | `0x21` | 未扫描到电机时的回退设备地址 |

---

## 6. 调试与验证

1. **CAN 分析仪**：扫描阶段抓握手帧（`0x00`，逐地址探测 0x01~0x3F）与应答；
   运行阶段确认使能/模式帧启动时出现，速度帧每 1 s 交替为正/零/负/零，方向按循环变化。
2. **RTT 日志**（SEGGER RTT）：`srv_can` 打印扫描结果（检测到的电机 ID 列表）、
   阶段切换（FORWARD/DWELL/REVERSE）、24h 停止等事件；`drv_can` 在发送失败时会打印：
   - `ch1 tx fail xN` —— 无 ACK，帧没发出去
   - `ERROR-PASSIVE` —— 错误帧累积（波特率/帧格式不匹配）
   - `txfifo FULL ... lec=x` —— TX FIFO 打满，所有帧被丢弃
3. **电机无响应排查**：确认电机地址在扫描范围（`SRV_CAN_SCAN_ADDR_MIN/MAX`）内、
   波特率 1 Mbps（FDCAN 仲裁段）、帧为经典 CAN（非 FD）、总线终端电阻与接线正常；
   若扫描日志显示 "NO servo detected"，重点检查供电与接线。
4. **电机不转但无报错**：确认已使能（`0x2A 01`）并处于速度模式（`0x07 02`）；
   若电机带抱闸且默认锁定，将 `SRV_CAN_TEST_RELEASE_BRAKE` 置 1。

---

## 7. 相关文件

| 文件 | 作用 |
|------|------|
| `docs/苇熠电机can协议文档.md` | 苇熠伺服执行器 CAN 协议规范（V1.04） |
| `service/srv_can.c` / `srv_can.h` | 测试模式（苇熠协议组帧）+ 旧 CAN FD 反馈帧打包 |
| `tasks/can_task.c` | 10 ms 周期任务，驱动 `srv_can_test_step()` |
| `device_drivers/drv_can.c` / `drv_can.h` | FDCAN 驱动（发送/接收/错误诊断，支持经典 CAN 帧） |
| `Core/Src/fdcan.c` | FDCAN1 初始化（FD_BRS，仲裁 1M） |
| `Core/Src/main.c` | 系统时钟（SYSCLK 160 MHz） |
