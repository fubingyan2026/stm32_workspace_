# srv_ht_torque_test 实现说明

## 1. 概述

`srv_ht_torque_test.c`（`service/`）是苇熠(HT) 伺服执行器的**速度模式多圈往复耐久测试**服务：令电机在位置 `0` 与 `+POS_LIMIT_DEG`（当前 `29*360 = 10440° = 29 圈`）之间连续往复旋转。

- 走**速度模式**（`0x07 02`）下发转速（`0x09`），方向变化时用**固件线性斜坡**平滑加减速——这是消除"一卡一卡"（位置模式下电机对每个目标都执行"加速→匀速→减速→停"，到点即停）的关键。
- 每 `POS_POLL_PERIOD_MS`(10ms) 用 `0x06` 读回位置，**任一台电机到达端点即翻转全局方向**。
- 附带报警查询、电压读取、掉线检测、恢复重发、30 天在线自动停止。
- 激活方式：`srv_ht_test_mode.h` 中 `SRV_HT_TEST_MODE_TORQUE = 1`（`can_task` / `srv_can` 按此宏接线）。

## 2. 通信协议

经典 CAN 2.0A，1 Mbps，CAN-ID 低 8 位 = 设备地址，`data[0]` = 指令，多字节参数大端。

| 指令 | 说明 | 帧 |
|------|------|----|
| `0x00` | 握手（总线扫描探测） | 1B |
| `0x2A 01/00` | 使能 / 失能 | 2B |
| `0x07 02` | 设置速度模式 | 2B |
| `0x09` + IQ24 | 设置转速（RPM，归一化 × 满量程 6000，正=正转 负=反转 0=停） | 5B |
| `0x06` | 读当前位置（返回 `[0x06][4B 大端 IQ24]`，单位 R） | 1B → 5B |
| `0xFF` | 读报警（实测返回 `[0xFF][4B 大端报警码]`） | 1B → 5B |
| `0x87` | 读供电电压（`[0x87][V高][V低]`，×100 V） | 1B → 3B |

- 位置 IQ24：`IQ = 转数 × 2^24`（满量程 ±127R）。`POS_LIMIT_IQ = 29 << 24 = 0x1D000000`。
- 速度 IQ24：`IQ = rpm / 6000 × 2^24`（`SRV_HT_TORQUE_TEST_SPEED_FULL_SCALE = 6000`）。

## 3. 分层与接线

- `tasks/can_task.c`：每 `TASK_PERIOD_MS`(5ms) 调用 `srv_ht_torque_test_step()`；RX 回调 `can_rx_callback → srv_can_on_rx → srv_ht_torque_test_on_rx()`（测试帧优先消费，返回 true 后旧 CAN FD 协议不再解析）。
- 模块**不依赖** `srv_motor` / `srv_motor_behavior`，直接经 `drv_can` 收发。
- 全静态分配、无 malloc，`*_init()` 无参（符合工程规范）。

## 4. 运行流程

### 4.1 上电启动 `srv_ht_torque_test_init()`
`SRV_HT_TORQUE_TEST_AUTO_START = 1` → 自动调 `srv_ht_torque_test_start()`。

### 4.2 `srv_ht_torque_test_start()`
- 置运行标志、重置速度斜坡状态（`s_dir = +1`、当前下发转速 0）。
- 进入扫描阶段。

### 4.3 总线扫描（step 中）
- 每 `SCAN_PROBE_PERIOD_MS`(20ms) 向地址 `0x01~0x3F` 逐地址发握手 `0x00`；
- 应答帧的 CAN-ID 记入电机列表（去重）；
- 一个都没扫到 → 回退到默认地址 `SRV_HT_TORQUE_TEST_DEFAULT_MOTOR_ADDR = 0x21`。

### 4.4 `srv_ht_torque_test_scan_done()`
- 下发 `使能(0x2A 01)` + `速度模式(0x07 02)`；
- 首个速度值不在此下发，由 step() 的斜坡逻辑从 0 平滑爬升，避免上电转速突变。
- 此后 `startup retry`：控制开始 1s 内每 `STARTUP_RETRY_MS`(100ms) 补发使能+速度模式（防首帧被 TX FIFO 丢弃）。

### 4.5 主循环 `srv_ht_torque_test_step()`（每 5ms）
顺序执行：

1. 30 天在线计时：全部电机在线时累计 `s_online_ms`，到 `DURATION_MS` 自动 `stop()`。
2. 报警应答处理：状态变化才打印（`alarm_print`，LOG_W）。
3. 电压应答打印（LOG_I，默认关闭）。
4. 启动补发使能/速度模式（仅前 1s）。
5. **位置轮询**：每 10ms 向各电机发 `0x06`。
6. **端点反向判定**：消费位置应答，`pos ≥ POS_LIMIT_IQ - REACH_IQ` → `s_dir = -1`；`pos ≤ REACH_IQ` → `s_dir = +1`。
7. **速度斜坡发送**：每 `CMD_PERIOD_MS`(100ms) 计算并下发当前转速。
8. 报警查询（每 `ALARM_PERIOD_MS`(100ms)）、电压查询（每 `VOLTAGE_PERIOD_MS`(3000ms)）。
9. 掉线检测（`NORESP_PERIOD_MS`=10s 无任何帧 → 告警一次）与恢复在线（重发使能+速度模式）。

## 5. 核心控制算法

### 5.1 速度平滑斜坡（关键）

方向不变时转速目标恒定；方向翻转（端点反向）时，目标转速在 `RAMP_MS`(2000ms) 内**线性插值**过渡，每 `CMD_PERIOD_MS`(100ms) 重发一次：

```
target_rpm = s_dir > 0 ? +SRV_HT_TORQUE_TEST_SPEED_RPM : -SRV_HT_TORQUE_TEST_SPEED_RPM
if (target_rpm != s_ramp_target_rpm)   // 目标变化，开新斜坡段
    s_ramp_target_rpm = target_rpm; s_ramp_from_rpm = s_speed_rpm; s_ramp_start_ms = now;
elapsed = now - s_ramp_start_ms
send_rpm = elapsed >= RAMP_MS ? target_rpm
                               : s_ramp_from_rpm + (target_rpm - s_ramp_from_rpm) * elapsed / RAMP_MS
```

效果：0 → +300 RPM 用 2s 线性爬升；反向时 +300 → -300 用 2s 平滑过零，全程无转速阶跃 → 无卡顿。

### 5.2 端点反向判定

- 以**位置反馈**为准（10ms 轮询 `0x06`），任一台电机到端点即翻转全局方向（所有电机广播同一速度）。
- 端点阈值 `REACH_IQ` 对应 3°。

### 5.3 反转过冲（已知特性）

端点反向触发后，电机还需按斜坡减速（约半段 `RAMP_MS/2`），会越过端点一段距离：

```
过冲 ≈ 转速 × (RAMP_MS/2) / 60000   [圈]
     = 300 × 1000 / 60000 ≈ 2.5 圈
```

即单向往返实际行程约 `29 + 2.5 ≈ 31.5 圈`。耐久测试可接受；如需精停在端点，需在接近端点时提前切回位置模式精停（当前未实现）。

## 6. 接收处理 `srv_ht_torque_test_on_rx()`（ISR 上下文）

- **扫描阶段**：任何应答帧的 CAN-ID 记为电机地址。
- **已知电机帧**：刷新 `last_seen`（掉线检测）；若此前被判掉线，置"恢复在线"事件。
- `0x06` 位置应答 → 存 `s_motor_pos_iq[idx]` + 置 `s_motor_pos_pending`。
- `0xFF` 报警应答 → 存 `s_motor_alarm[idx]` + 置 `s_alarm_pending`（主循环打印）。
- `0x87` 电压应答 → 存 `s_motor_volt[idx]`。
- ISR 只记录/置标志，**不打日志**；主循环消费。

## 7. 关键参数表（当前值）

| 参数 | 值 | 含义 |
|------|----|------|
| `POS_LIMIT_DEG` | `29*360 = 10440` | 正极限角度（29 圈） |
| `POS_LIMIT_IQ` | `29 << 24`（const int32_t） | 端点位置 IQ24 |
| `REACH_DEG` / `REACH_IQ` | 3° | 端点判定阈值 |
| `POS_POLL_PERIOD_MS` | 10 | 位置轮询周期 |
| `SPEED_RPM` | 300 | 巡航转速 |
| `RAMP_MS` | 2000 | 加减速斜坡时长 |
| `CMD_PERIOD_MS` | 100 | 速度帧重发周期 |
| `SPEED_FULL_SCALE` | 6000 | 速度满量程（文档 §4.2） |
| `ALARM_PERIOD_MS` | 100 | 报警查询周期 |
| `NORESP_PERIOD_MS` | 10000 | 掉线判定 |
| `VOLTAGE_PERIOD_MS` | 3000 | 电压读取周期 |
| `DURATION_MS` | 2,592,000,000 | 30 天在线自动停止 |
| `DEFAULT_MOTOR_ADDR` | 0x21 | 扫描失败回退地址 |
| `MAX_MOTORS` | 63 | 最多电机数 |

注：`POS_LIMIT_IQ` 用文件级 `const int32_t` 定义（非宏），仅本文件使用，若其他文件需要请改为 `#define` 或 `static const`。

## 8. 日志

- `SRV_HT_TORQUE_TEST_LOG_W`：启用（tag `torque_test`）——打印报警、掉线、恢复在线。
- `SRV_HT_TORQUE_TEST_LOG_I`：已注释关闭——扫描信息、电压等不打印（如需调试打开第 37 行 `//`）。

## 9. 调参指南

| 目标 | 调整 |
|------|------|
| 更快/更慢往复 | `SPEED_RPM`（当前 300 RPM） |
| 更陡/更缓加减速 | `RAMP_MS`（当前 2000ms，越小越陡；太大会反应迟钝、过冲大） |
| 摆幅（圈数） | `POS_LIMIT_DEG = 圈数*360` |
| 反向更灵敏 | 调小 `POS_POLL_PERIOD_MS`（受 can_task 5ms 步进下限约束） |
| 精停在端点（去过冲） | 需在端点前切位置模式精停（未实现） |

## 10. 注意事项

- 断电前务必调用 `stop()`（发速度 0 + 失能 `0x2A 00`），否则零位可能丢失。
- 多电机共用总线时，所有电机收到同一广播速度，反向由"任一台到位"触发，电机位置差异大时行为按此简化模型。
- 与旧 CAN FD 上位机协议（0x100 控制帧）共存：测试帧先被本模块消费，其余交由 `srv_can`。
