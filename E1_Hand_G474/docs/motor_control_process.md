# 当前电机控制过程

| 项目 | 信息 |
|------|------|
| **文档版本** | V1.0.0 |
| **适用分支** | `test_ht_motor`（2026-08-12，commit `94fc573` 完成 HT 电机测试控制） |
| **目标 MCU** | STM32G474 (Cortex-M4, 512KB Flash / 128KB SRAM) |
| **依赖协议** | [关节模组通讯协议](uart_protocol.md)（UART 关节电机）、[苇熠电机 CAN 协议](苇熠电机can协议文档.md)（CAN HT 电机） |

> **本文档描述的是当前分支实际运行的电机控制过程**，与代码 `service/srv_motor.c`、`service/srv_can.c` 一一对应。
> 注意：`docs/motor_control_api.md`、`docs/can_motor_control.md` 中的 FSM 状态与常量部分已落后于当前代码，以本文档及源码为准。

---

## 1. 概述

当前固件**并行存在两条相互独立的电机控制通道**：

| 通道 | 目标电机 | 传输层 | 服务模块 | 驱动周期 |
|------|----------|--------|----------|----------|
| **A. UART 关节电机** | 灵巧手 9 个关节模组（组A 5 + 组B 4） | USART2/USART3 @500000 bps | `srv_motor`（每路 UART 一个 FSM） | 主循环全速 poll `srv_motor_step()` |
| **B. CAN HT 电机** | 苇熠伺服执行器（SCA）耐久测试 | FDCAN1 经典 CAN 2.0A @1 Mbps | `srv_can`（测试模式 FSM） | `can_task` 10 ms 周期 |

两条通道互不依赖：UART 通道控制手部关节，CAN 通道独立驱动 HT 电机做正反转耐久测试。两条通道都在当前分支**处于激活状态**（UART 通道负责 9 个关节的稳态位置广播 + 反馈轮询；CAN 通道上电自动启动耐久测试）。

### 1.1 当前分支启用 / 禁用状态速览

| 模块 | 状态 | 说明 |
|------|------|------|
| `srv_motor_step()` | ✅ 启用 | 主循环全速驱动 UART 关节电机控制 |
| `srv_can_test_step()` | ✅ 启用 | `can_task` 10 ms 驱动 CAN HT 电机耐久测试（上电自启 `SRV_CAN_TEST_AUTO_START=1`） |
| `behavior_task_init()` | ⛔ 注释 | 行为协调 FSM（`srv_motor_behavior`）**未运行** → 零点校准/上电找零/RUNNING 门控全部失效 |
| `daemon_task_init()` | ⛔ 注释 | 9 电机反馈超时看门狗未运行 |
| 反馈帧 `0x101` / 状态帧 `0x102` | ⛔ 注释 | `can_task` 中周期上报已注释，上位机暂收不到反馈 |
| 旧上位机控制帧 `0x100` | ⚠️ 惰性 | `srv_can_process()` 仍在调用，但 `srv_motor_behavior` 未初始化 → 目标值被丢弃，链路实际断开 |

---

## 2. 系统初始化与任务调度

### 2.1 初始化顺序（`app_main()`）

```
delay_init()              SysTick 时基（millis/micros）
drv_uart_init()           USART2/3 DMA 收发（USART1 由 drv_log_uart 控制台接管）
drv_log_uart_init()       USART1 控制台（日志 TX + 命令 RX）
log_task_init()           日志输出任务
srv_log_flash_init()      告警/错误日志 Flash 持久化
can_task_init()           drv_can_init + srv_can_init(自动启动测试) + 10ms 周期任务
led_task_init()           LED 状态指示
// behavior_task_init()   已注释 → srv_motor_behavior/daemon 未启用
// daemon_task_init()     已注释
```

### 2.2 主循环（协程式，无 RTOS）

```
for (;;) {
    drv_uart_rx_restart(DRV_UART_CH_2);   // RX DMA 兜底重启
    drv_uart_rx_restart(DRV_UART_CH_3);
    sw_timer_tick(millis());              // 更新定时器时基
    sw_timer_task();                      // 派发到期 NORMAL/LOW 定时器
    srv_motor_step();                     // UART 关节电机全速 poll（非定时）
}
```

### 2.3 周期任务（`sw_timer`）

| 任务 | 周期 | 回调内容 |
|------|------|----------|
| `can_task` | 10 ms | `drv_can_poll_status`（Bus-Off 恢复/错误告警）→ `srv_can_process` → `srv_can_test_step`（HT 测试推进） |
| `led_task` | 10 ms | LED 状态指示刷新 |
| `log_task` | 20 ms | 日志输出（UART DMA 控制台） |

> `srv_motor_step()` 不走定时器，直接在主循环全速轮询；每路 UART 的发送由 FSM 在 DMA 空闲时自动推进，通过 `micros()` 做帧间隔门控。

---

## 3. 通道 A：UART 关节电机控制（`srv_motor`）

### 3.1 硬件与分组

| 组 | UART 通道 | 引脚 | 波特率 | 电机 | dev_id |
|----|-----------|------|--------|------|--------|
| 组A | `DRV_UART_CH_2`（USART2） | PA2(TX) / PA3(RX) | 500000 | motorA1~A5 | 1~5 |
| 组B | `DRV_UART_CH_3`（USART3） | PB10(TX) / PB11(RX) | 500000 | motorB1~B4 | 1~4 |

> 通道可通过编译宏 `SRV_MOTOR_GRPA_UART` / `SRV_MOTOR_GRPB_UART` 重定向（仅限 CH_2/CH_3 互切；USART1 已被控制台占用，不可作电机通道）。

### 3.2 每路 UART 一个分时发送 FSM

状态集（当前代码，7 态）：

```
STARTUP → BCAST_EN → BCAST_POS → POLL → BCAST_CUR → BCAST_LIM → IDLE →(周期到)→ BCAST_EN
   ↑                                                                             │
   └─────────────────── 每 MOTOR_BCAST_PERIOD_MS 启动新一轮广播序列 ──────────────┘
```

| 状态 | 行为 | 下一状态 |
|------|------|----------|
| `STARTUP` | 分 3 步：①逐电机发 ENABLE → ②等 1500 ms 编码器自校准 → ③逐电机发 SET_MODE(位置模式) | 完成后 → `IDLE` |
| `BCAST_EN` | 模式脏则逐电机发 SET_MODE；使能脏则发 `0xB155` 使能广播；否则直接跳过 | → `BCAST_POS` |
| `BCAST_POS` | 构建 `0xAB55` 位置广播帧（8 槽位，含 dev 1~6）下发 | → `POLL` |
| `POLL` | 轮询 1 个电机反馈（round-robin，见 3.6） | → `BCAST_CUR` |
| `BCAST_CUR` | 配置 1 个电机的 `MAX_CUR`(index=18) | → `BCAST_LIM` |
| `BCAST_LIM` | 配置同一电机的 `SPD_LIMIT`(index=30)，推进游标 | → `IDLE` |
| `IDLE` | 等待距 `cycle_start` 满 `MOTOR_BCAST_PERIOD_MS` | → `BCAST_EN` |

> 所有 handler 都在 `drv_uart_is_tx_busy()` 为假（DMA 空闲）时才实际发送；DMA 忙则返回当前状态滞留，下个 `srv_motor_step()` 重试。

### 3.3 STARTUP 启动序列（`fsm_startup`）

1. **ENABLE**：对每个已注册电机发标准帧 `[dev_id, 0,0,0,0,0, CMD_ENABLE(1), 0]`（can_id=`0xA0`），并记录 `enable_time`。
2. **等待编码器校准**：`millis() - enable_time >= 1500` ms 后进入下一步（不阻塞主循环）。
3. **SET_MODE**：对每个电机发标准帧 `[dev_id, CTRL_MODE(7), 0,0, POSITION(2), 0,0,0]`（can_id=`0xA0`）。
4. 全部完成 → `cycle_start = millis()`，进入 `IDLE`。

> 初始参考值（`srv_motor_init` 设置）：`pos_ref=0`（停原点）、`spd_ref=6000`（SPD_LIMIT ≈15%）、`cur_ref=1000`（MAX_CUR ≈1.37A）。

### 3.4 稳态广播周期

每个周期（`MOTOR_BCAST_PERIOD_MS` ≈ 10 ms）执行一轮：

1. **位置广播**：`0xAB55` + `int16 pos_ref[8]`（20B）。注：当前代码只广播位置（`0xAB55`），不再广播速度帧。
2. **反馈轮询**：`motor_poll_one()` — round-robin 游标选 1 个电机，发 `0xA0` 标准查询帧 `[dev_id, index]`：
   - 默认查 `INFO_02_R`(index=2)（角度/速度/DQ 电流，高频）；每 `POLL_01_INTERVAL=3` 次插入一次 `INFO_01_R`(index=1)（状态/故障/温度/电压）。
3. **电流限制配置**：`MAX_CUR`(index=18) → `SPD_LIMIT`(index=30)，每次 1 个电机（`cur_cursor` 游标轮转）。

### 3.5 帧格式与 CRC

**标准帧（20B）** — 用于配置/使能/轮询/零点：

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4-7 | Byte 8-15 | Byte 16-17 | Byte 18-19 |
|--------|--------|--------|--------|----------|-----------|------------|------------|
| `0x55` | `0xAA` | `0x00` | `0x14` | can_id（LE） | data[8] | CRC16 | 保留 0 |

- CRC16-CCITT-FALSE 覆盖 **can_id + data 共 12 字节**（byte 4~15），帧头不计入。

**位置广播帧（20B）**：

| Byte 0-1 | Byte 2-17 | Byte 18-19 |
|----------|-----------|------------|
| head2 `0xAB55`（LE） | int16 pos[8]（LE） | CRC16 |

- CRC16-CCITT-FALSE 覆盖 **value 共 16 字节**（byte 2~17），不含 head2。

**使能广播帧（20B）**：

| Byte 0-1 | Byte 2-17 | Byte 18-19 |
|----------|-----------|------------|
| head2 `0xB155`（LE） | en_states[16]（每 dev 1 字节：0/1=无/使能） | CRC16 |

- CRC16-CCITT-FALSE 覆盖 byte 0~17 共 18 字节。

**反馈解析关键**（`motor_parse_reply`）：校验帧头 `55 AA 00 14` + CRC 后，`dev_id = frame[5]`、`cfg_index = frame[6]`：
- `INFO_02_R`：`d_cur`=B8、`q_cur`=B10、`speed_fb`=B12、`angle_fb`=B14（各 int16 LE）→ 更新 `fb.time_fb`。
- `INFO_01_R`：`fsm_state`=B8、`err_code`=B9、`temp`=B11、`angle_fb`=B12、`vbus`=B14 → 更新 `fb.time_status`。

**量纲换算**：`angle_fb` Q7（raw = 度×128）；`speed_fb` Q15（满量程 ≈50 krpm）；`q_cur`/`d_cur` Q15（满量程 5.625 A）；`vbus` raw/128 = V；`temp` 物理°C = raw − 50。

### 3.6 接收管道（ISR → 主循环）

```
UART RX DMA ping-pong 双缓冲(20B) + IDLE 事件
  └─ HAL_UARTEx_RxEventCallback → 切换缓冲并重启 DMA，上抛本次数据（中断上下文）
       └─ motor_rx_cb → 按 20B 定长切帧 → msg_fifo_push（每路 UART 一个 rx_queue，512B≈25 帧）
            └─ srv_motor_step() 内 motor_drain_rx → motor_parse_reply → 更新对应 fb
```

- RX DMA 缓冲 20B（`DRV_UART_RX_BUF_SIZE`），IDLE 事件天然按帧分块，错位自愈。
- 主循环 `drv_uart_rx_restart()` 仅作 ISR 内重启失败时的兜底。

### 3.7 时序与频率

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR_ONE_FRAME_TIME_US` | 400 | 单帧传输时间估算 |
| `MOTOR_POST_DLY_US` | 600 | 帧间额外间隔 |
| 帧间隔门控 | 1000 µs | `srv_motor_step` 中 `micros()` 差值 ≥ 1000 µs 才推进 FSM（每路 UART） |
| `MOTOR_BCAST_PERIOD_MS` | 10 | 位置广播周期 ≈ 100 Hz/组（由帧时间预算推导） |
| 反馈轮询率 | 1 电机/周期 | 组A 5 电机 → 每台约 20 Hz；每 3 次轮询插 1 次状态查询 |

> 顶层注释所称"广播 300 Hz"为早期目标值，当前常量下实际广播周期约 10 ms。

### 3.8 关键常量（改参数改这里）

| 宏 | 默认 | 含义 |
|----|------|------|
| `SRV_MOTOR_GRPA_UART` | `DRV_UART_CH_2` | 组A 串口通道 |
| `SRV_MOTOR_GRPB_UART` | `DRV_UART_CH_3` | 组B 串口通道 |
| `MOTOR_PER_GROUP` | 6 | 每组最大电机数（广播帧槽位数） |
| `POLL_01_INTERVAL` | 3 | 每 N 次 `INFO_02_R` 后插一次 `INFO_01_R` |
| `MOTOR_BCAST_PERIOD_MS` | 10 | 广播周期（ms） |
| 初始 `spd_ref` / `cur_ref` | 6000 / 1000 | SPD_LIMIT ≈15% / MAX_CUR ≈1.37A |

---

## 4. 通道 B：CAN HT 电机测试控制（`srv_can`）

> 该通道是当前分支（`test_ht_motor`）的主要工作内容，对应 commit `94fc573`。目标为**苇熠伺服执行器（SCA）**，上电自动进入 24h 正反转耐久测试。

### 4.1 硬件与总线

| 项 | 值 |
|----|-----|
| 通道 | FDCAN1，PA11(RX) / PA12(TX) |
| 帧格式 | 经典 CAN 2.0A，标准帧（11-bit ID，低 8 位 = 设备地址），每帧 ≤ 8 B |
| 波特率 | 仲裁段 1.000 MHz（Prescaler=8，tq=20，采样点 80%） |
| 指令语义 | `data[0] = 指令符`，`data[1..] = 参数`（多字节**大端**） |
| 协议依据 | `docs/苇熠电机can协议文档.md` |

### 4.2 启动流程

```
can_task_init()
 ├─ drv_can_init()         FDCAN Start + RX 中断 + 收发器 STB 拉低
 └─ srv_can_init()
     └─ srv_can_test_start()   ← SRV_CAN_TEST_AUTO_START=1 时上电自动调用
         ├─ 置运行标志、记录 24h 计时起点
         └─ 进入【阶段 0：总线扫描】
```

### 4.3 阶段 0：总线电机 ID 扫描

- 逐地址发送握手帧 `0x00`（经典 CAN 1B），探测 `0x01`~`0x3F`，每地址间隔 `SRV_CAN_SCAN_PROBE_PERIOD_MS`=20 ms。
- 收到应答帧（CAN-ID = 电机地址）即记为已检测电机（去重，`srv_can_test_scan_record`，ISR 中执行）。
- **扫描完成**（`srv_can_test_scan_done`）：
  - 若未检测到任何电机 → 回退默认地址 `SRV_CAN_TEST_DEFAULT_MOTOR_ADDR`=`0x21` 继续测试（打 WARN）。
  - 对检测到的电机下发：**使能** `0x2A 01` → **速度模式** `0x07 02` →（可选）打开抱闸 `0xF4 0100`。
  - 不直接发全速：转速由缓启动斜坡从 0 线性爬升，避免阶跃过大报警。
  - 进入【阶段 1】正转。

### 4.4 阶段 1：正转 / 停留 / 反转 / 停留 循环

| 阶段 | 时长（`SRV_CAN_TEST_PHASE_MS`=30000） | 目标转速 |
|------|------|----------|
| 正转 | 30 s | +1600 RPM |
| 停留 | 30 s | 0 RPM（不失能） |
| 反转 | 30 s | −1600 RPM |
| 停留 | 30 s | 0 RPM（不失能） |

**缓启动斜坡**（`SRV_CAN_TEST_RAMP_TIME_MS`=3000）：目标变化时以当前下发转速为新段起点，按时间线性插值，3000 ms 内到达目标；阶段切换后重新计时。速度帧每 `SRV_CAN_TEST_CMD_PERIOD_MS`=100 ms 下发一次（IQ24 大端，满量程 6000 RPM）。

**补发策略**：
- 进入正转/反转阶段时补发使能 + 速度模式（停留期间可能掉使能/掉模式）。
- 控制开始 1 s 内每 `SRV_CAN_TEST_STARTUP_RETRY_MS`=100 ms 补发一次（首帧可能因 TX FIFO 未就绪被丢弃）。

### 4.5 运行监控

| 监控项 | 指令 | 周期/偏移 | 行为 |
|--------|------|-----------|------|
| 报警查询 | `0xFF` | 每 1000 ms，偏移 500 ms | 应答 `[FF][4B 大端报警码]`（24 位），**只在状态变化时打印**（出现/变化/消除各一次，WARN + 逐位解码） |
| 供电电压 | `0x87` | 每 5000 ms，偏移 250 ms | `[87][V高][V低]` ×100 = V，打印电压值 |
| 无响应检测 | — | 超时 3000 ms | 超过 3 s 未收到电机任何帧 → 打 WARN"长时间无响应（掉线或断电）"，每电机只告警一次 |
| 24h 到时 | — | `SRV_CAN_TEST_DURATION_MS` | 发速度 0 帧 + **失能** `0x2A 00`（防止零位丢失），自动停止 |

> 报警/电压查询与速度控制帧**错开**发送（偏移 250/500 ms），避免同节拍多包同时发出导致丢应答。

### 4.6 当前参数表（与 `service/srv_can.c` 一致）

| 宏 | 值 | 含义 |
|----|-----|------|
| `SRV_CAN_TEST_AUTO_START` | `1` | 上电自动启动测试 |
| `SRV_CAN_TEST_SPEED_RPM` | `1600` | 正/反转目标转速 (RPM)，满量程 6000 |
| `SRV_CAN_TEST_RAMP_TIME_MS` | `3000` | 缓启动斜坡时长 (ms) |
| `SRV_CAN_TEST_CMD_PERIOD_MS` | `100` | 速度帧发送周期 (ms) |
| `SRV_CAN_TEST_ALARM_PERIOD_MS` | `1000` | 报警查询周期 (ms) |
| `SRV_CAN_TEST_ALARM_OFFSET_MS` | `500` | 报警查询相对控制帧偏移 (ms) |
| `SRV_CAN_TEST_NORESP_PERIOD_MS` | `3000` | 无响应判定超时 (ms) |
| `SRV_CAN_TEST_VOLTAGE_QUERY_ENABLE` | `1` | 周期读供电电压（0x87） |
| `SRV_CAN_TEST_VOLTAGE_PERIOD_MS` | `5000` | 电压读取周期 (ms) |
| `SRV_CAN_TEST_VOLTAGE_OFFSET_MS` | `250` | 电压读取相对控制帧偏移 (ms) |
| `SRV_CAN_TEST_PHASE_MS` | `30000` | 每阶段时长 (ms)，30 s |
| `SRV_CAN_TEST_DURATION_MS` | `86400000` | 测试总时长 (ms)，24 h |
| `SRV_CAN_TEST_STARTUP_RETRY_MS` | `100` | 控制开始 1 s 内补发周期 (ms) |
| `SRV_CAN_TEST_RELEASE_BRAKE` | `0` | 1=打开抱闸；0=不处理（无抱闸电机） |
| `SRV_CAN_LOG_RAW_RX_ENABLE` | `1` | 记录原始接收帧到环形缓冲（调试，打印已注释） |
| `SRV_CAN_SCAN_ADDR_MIN/MAX` | `0x01`/`0x3F` | 扫描地址范围（含） |
| `SRV_CAN_SCAN_PROBE_PERIOD_MS` | `20` | 每地址探测间隔 (ms) |
| `SRV_CAN_MAX_MOTORS` | `63` | 最多同时控制电机数 |
| `SRV_CAN_TEST_DEFAULT_MOTOR_ADDR` | `0x21` | 未扫描到电机时的回退地址 |

### 4.7 指令帧速查（经典 CAN，CAN-ID = 设备地址）

| 功能 | 指令 | 参数 | DLC | 帧示例（addr=0x02） |
|------|------|------|-----|---------------------|
| 握手（扫描） | `0x00` | — | 1 | `02 00` |
| 使能 | `0x2A` | `0x01` | 2 | `02 2A 01` |
| 失能 | `0x2A` | `0x00` | 2 | `02 2A 00` |
| 设置速度模式 | `0x07` | `0x02` | 2 | `02 07 02` |
| 设置速度 +1600 RPM | `0x09` | IQ24 大端 | 5 | `02 09 00 44 44 44` |
| 设置速度 0 | `0x09` | 0 | 5 | `02 09 00 00 00 00` |
| 查询报警 | `0xFF` | — | 1 | `02 FF`（应答 `[FF][4B 大端报警码]`，5B） |
| 读取供电电压 | `0x87` | — | 1 | `02 87`（应答 `[87][V高][V低]`） |
| 打开抱闸（可选） | `0xF4` | `0x01 0x00` | 3 | `02 F4 01 00` |

> IQ24 转换：`IQ = rpm / 6000 × 2²⁴`（取整，大端）。1600 RPM → `0x00444444`。

---

## 5. 数据流汇总

```
┌──────────────────────────── 主循环（全速） ────────────────────────────┐
│  srv_motor_step()                                                     │
│    ├─ motor_drain_rx(组A/组B)   收电机回复 → 更新 fb                   │
│    └─ 帧间隔≥1ms 时 fsm_step    按 STARTUP/BCAST_EN/BCAST_POS/POLL/    │
│                                 BCAST_CUR/BCAST_LIM/IDLE 推进，DMA     │
│                                 空闲才发帧 → USART2/3 → 9 关节电机      │
└────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────── can_task（10 ms） ─────────────────────────┐
│  drv_can_poll_status()    Bus-Off 恢复 / 错误告警                       │
│  srv_can_process()        旧 0x100 控制帧（惰性，behavior 未初始化）     │
│  srv_can_test_step()      HT 测试：扫描→正/反转循环→斜坡→报警/电压/无响应  │
│                            监控，经 FDCAN1 → 苇熠伺服执行器             │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 6. 当前分支已禁用功能与影响

| 禁用项 | 位置 | 影响 |
|--------|------|------|
| `behavior_task_init()` | `app_main.c:52` 注释 | `srv_motor_behavior` FSM 不运行：无上电找零（180° 到位设零）、无 RUNNING 态门控、`srv_motor_behavior_set_setpoint()` 因 `s_initialized=false` 直接返回 |
| `daemon_task_init()` | `app_main.c:55` 注释 | 9 电机反馈超时看门狗不运行，`any_online()` 恒 false |
| `srv_can_send_feedback()` / `srv_can_send_status()` | `can_task.c:70-77` 注释 | 上位机收不到 `0x101` 反馈 / `0x102` 状态帧 |
| 旧 CAN FD 上位机控制 | `srv_can.c` 保留 | `s_rx` 缓存仍在解析 `0x100`，但 `srv_can_process` 下发的目标被 behavior 层丢弃 |

> **结论**：当前分支上，手部 9 个关节电机由 `srv_motor` 自主跑启动序列并稳定在原点（`pos_ref=0`）；真正"被控制"、可观测的运动是 CAN 通道的 HT 电机耐久测试。

---

## 7. 相关文件

| 文件 | 作用 |
|------|------|
| `service/srv_motor.c` / `.h` | UART 关节电机分时 FSM、帧构建、反馈解析 |
| `service/srv_can.c` / `.h` | CAN HT 电机测试模式（扫描/循环/监控）+ 旧 CAN FD 反馈帧打包 |
| `tasks/can_task.c` | 10 ms 周期任务，驱动 `srv_can_test_step()` |
| `tasks/app_main.c` | 初始化顺序 + 主循环（`behavior_task`/`daemon` 已注释） |
| `device_drivers/drv_uart.c` / `.h` | USART2/3 DMA 收发（TX DMA + RX ping-pong + IDLE） |
| `device_drivers/drv_can.c` / `.h` | FDCAN1 驱动（经典 CAN + CAN FD，Bus-Off 恢复） |
| `docs/uart_protocol.md` | 关节模组通讯协议（20B 定长帧） |
| `docs/苇熠电机can协议文档.md` | 苇熠伺服执行器 CAN 协议（V1.04） |
