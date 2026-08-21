# 四电机扭力耐久测试方案与实现文档

> 版本：2026-08-20 ｜ 工程：E1_Hand_G474（STM32G474，裸机，CMake/Ninja）
>
> 本文档覆盖四条总线上 4 种伺服电机的往复耐久（扭力）测试模块的完整方案与实现：
> 苇熠 HT（CAN1 / CAN2 两路）、良志 ODrive（CAN1）、Motorevo PA430（CAN2）。
> 所有模块均为上电自动启动的往复运动耐久测试，累计在线满 30 天自动停止并失能。

---

## 1. 概述

| 模块源文件 | 电机/总线 | 通道 | 控制方式 | 往复形式 |
|---|---|---|---|---|
| `srv_ht_torque_test.c` | 苇熠 HT（CAN1） | FDCAN1 | 速度模式连续旋转 | 初始化位置 ±6 转（速度模式多圈） |
| `srv_ht_can2_torque_test.c` | 苇熠 HT（CAN2） | FDCAN2 | 速度模式连续旋转 | 初始化位置 ±6 转（CAN1 版克隆） |
| `srv_tongzhi_torque_test.c` | 良志 ODrive（CAN1） | FDCAN1 | 位置模式 + 梯形轨迹 | 首次闭环位置 ±6.5 转 |
| `srv_pa430_torque_test.c` | Motorevo PA430（CAN2） | FDCAN2 | MIT 力位混合 | 首次反馈位置 ±1.6 rad |

**模块选择**：`service/srv_motor_test_select.h`

- CAN1（`SRV_MOTOR_TEST_SELECT`，三选一，默认 `SRV_MOTOR_TEST_TONGZHI`）：
  - `SRV_MOTOR_TEST_HT_TORQUE = 0` 苇熠 HT 位置往复（速度模式）
  - `SRV_MOTOR_TEST_HT_TEMP   = 1` 苇熠 HT 速度模式原测试（旧）
  - `SRV_MOTOR_TEST_TONGZHI   = 2` 良志 ODrive 位置往复
- CAN2（`SRV_MOTOR_TEST_SELECT_CAN2`，二选一，默认 `SRV_MOTOR_TEST_PA430`）：
  - `SRV_MOTOR_TEST_HT_CAN2 = 0` 苇熠 HT CAN2 版
  - `SRV_MOTOR_TEST_PA430   = 1` Motorevo PA430 MIT 测试

各模块源文件**始终参与编译**，同一时刻每路只激活一个；CAN1 与 CAN2 并行运行互不干扰。

**运行时接线**（`tasks/can_task.c`）：`can_task_init()` 中 `drv_can_init()` 后按选择宏调用
对应模块 `*_init()`，注册 RX 回调；5ms 周期 `can_timer_cb` 依次调用两路 `*_step()`，
`drv_can_poll_status` 负责 Bus-Off 恢复与错误态告警。

---

## 2. 苇熠 HT（CAN1）— `srv_ht_torque_test.c`

### 2.1 协议与接线

- 协议文档：`docs/苇熠电机can协议文档.md`；经典 CAN 2.0A @ 1 Mbps。
- 帧格式：CAN-ID 低 8 位 = 设备地址（0x01~0x3F）；`data[0]` = 指令，`data[1..]` = 参数，多字节大端。
- 通道：FDCAN1（PA11 RX / PA12 TX）。

### 2.2 往复控制方案

1. **扫描**：启动后按 `0x00` 握手逐地址探测 `0x01~0x3F`（20ms/地址），发现电机后接管；
   未发现任何电机时回退到默认地址 `0x21`（`SRV_HT_TORQUE_TEST_DEFAULT_MOTOR_ADDR`）继续控制。
   每 3s 重扫一次（热插拔的电机晚于控制板上电时也能被发现接管）。
2. **使能 + 模式**：`0x2A` 使能、`0x07 0x02` 速度模式、`0x58` 电流限制（`0x01000000`=满量程，拉满扭矩上限）。
3. **往复**：以各电机**启动后首次读回位置为自身 0 点**，在 `中心 ± SRV_HT_TORQUE_TEST_POS_LIMIT_DEG`（±6 转）
   之间多圈往复。每 20ms 用 `0x06` 读当前位置，`|位置 − 端点| ≤ 3°` 判定到达端点并反向。
4. **速度与斜坡**：巡航 `SRV_HT_TORQUE_TEST_SPEED_RPM = 300 RPM`；方向变化时在
   `RAMP_MS = 2000ms` 内线性斜坡平滑加减速（速度模式 0x09 电机侧锁存，斜坡期间按 100ms 重发）。
5. **掉线/恢复**：10s 无任何应答帧判定掉线（速度模式自主运转，阈值放宽避免误判）；
   恢复在线时重新锁存该电机 0 点并重新下发使能/速度模式。
6. **反向兜底**：60s 内未发生端点反向（位置反馈冻结/丢帧/到位偏置）时强制反向。
7. **告警**：每 1s 主动 `0xFF` 查询报警码（与位置轮询错开 500ms）。

### 2.3 关键参数

| 参数 | 宏 | 当前值 | 说明 |
|---|---|---|---|
| 往复半幅 | `SRV_HT_TORQUE_TEST_POS_LIMIT_DEG` | `6*360`（6 转） | 中心 ± 该角度 |
| 到位阈值 | `SRV_HT_TORQUE_TEST_REACH_DEG` | `3` ° | 端点反向判定 |
| 巡航转速 | `SRV_HT_TORQUE_TEST_SPEED_RPM` | `300` RPM | 正负表方向 |
| 斜坡时长 | `SRV_HT_TORQUE_TEST_RAMP_MS` | `2000` ms | 加减速平滑 |
| 位置轮询 | `SRV_HT_TORQUE_TEST_POS_POLL_PERIOD_MS` | `20` ms | 0x06 读位置 |
| 电流限制 | `SRV_HT_TORQUE_TEST_CUR_LIMIT_IQ` | `0x01000000` | 满量程 |
| 掉线判定 | `SRV_HT_TORQUE_TEST_NORESP_PERIOD_MS` | `10000` ms | |
| 反向兜底 | `SRV_HT_TORQUE_TEST_FLIP_TIMEOUT_MS` | `60000` ms | |
| 状态日志 | `SRV_HT_TORQUE_TEST_STATUS_LOG_MS` | `60000` ms | 1 分钟 1 条 |
| 耐久时长 | `SRV_HT_TORQUE_TEST_DURATION_MS` | `2592000000` ms | 30 天，0 禁用 |

---

## 3. 苇熠 HT（CAN2）— `srv_ht_can2_torque_test.c`

### 3.1 说明

`srv_ht_torque_test` 在 FDCAN2（PB12 RX / PB13 TX）上的完整克隆，运行逻辑、参数、协议帧
与 CAN1 版**完全一致**，用于 CAN1 被占用时在独立总线控制另一组苇熠电机。符号统一加
`_CAN2_` 前缀（如 `SRV_HT_CAN2_TORQUE_TEST_SPEED_RPM = 300`）。

### 3.2 与 CAN1 版的差异

- 全部指令走 `DRV_CAN_CH_2`；RX 由 `can_task` 按 CH_2 分发到 `srv_ht_can2_torque_test_on_rx()`，
  非测试帧直接丢弃（CAN2 为专用总线）。
- 新增**使能保活补发**：在线但查询到未使能的电机，每 `2s`（起始偏移 11ms）补发
  使能+速度模式，修复「电机晚于控制板上电、错过启动补发窗口后永久失能」问题；
  11ms 偏移与位置/速度/报警帧分时错开，保证单 tick 帧数不超 FDCAN TX FIFO 深度。
- 同样下发电流限制 `0x58`（满量程），保证速度模式扭矩输出上限。

---

## 4. 良志 ODrive（CAN1）— `srv_tongzhi_torque_test.c`

### 4.1 协议与接线

- 协议文档：`docs/良志电机can协议.md`；经典 CAN 2.0A @ 1 Mbps。
- 帧格式：`CAN-ID = (node_id<<5)|cmd_id`，8 字节，**小端**，IEEE-754 单精度浮点。
- 通道：FDCAN1，与 HT CAN1 通过 `SRV_MOTOR_TEST_SELECT` 三选一。

### 4.2 往复控制方案

1. **电机发现**：ODrive 无主机握手命令，上电后电机**主动周期推送心跳（0x01，默认 100ms）**，
   模块监听心跳被动发现 node_id 并动态收录（去重，上限 8）。另以 200ms 周期对候选 node
   主动发 `Get_Error(0x03)` 探测静默电机；启动 2s 探测窗口内无发现则回退对 node 0 盲发。
2. **初始化序列（5 帧，逐帧下发）**：
   1. `0x18` 清错（清残留错误，避免闭环被旧错误阻塞）
   2. `0x0B` Set_Controller_Mode（control=3 位置，input=5 梯形轨迹）
   3. `0x11` Set_Traj_Vel_Limit（`SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS = 20` 转/s）
   4. `0x12` Set_Traj_Accel_Limits（加/减速 `8` 转/s²）
   5. `0x07` Set_Axis_State（8 = 闭环）
   序列发完仍未闭环（1s 宽限假定闭环）则从头重发。
3. **往复**：以各电机**首次进入闭环时的编码器位置为自身零点**（`s_center_turns[]`，掉线恢复
   重新锁存；未锁存前按绝对 ±AMP 下发），目标 = 零点 ± `SRV_TONGZHI_POS_AMP_TURNS`（±6.5 转）
   交替。`Set_Input_Pos`（0x0C）为**锁存式目标**（ODrive 梯形规划器自动跑到位并保持），
   无需高频持续下发；500ms 重发同目标作安全 no-op，防首帧/丢帧漏目标。
4. **到位判定**：靠电机周期推送的编码器位置（0x09，默认 10ms），全部已闭环电机
   `|enc − 目标| ≤ SRV_TONGZHI_ARRIVE_THRESH_TURNS`（0.25 转）后目标翻转；
   未闭环/掉线/无编码器反馈的电机不参与到位判定。
5. **兜底**：到位超时 60s 强制翻转（防反馈冻结/到位偏置永久停摆）；
   无位置反馈的盲发电机按 `SRV_TONGZHI_TRAVEL_EST_MS`（2.5s）定时翻转。

### 4.3 关键参数

| 参数 | 宏 | 当前值 | 说明 |
|---|---|---|---|
| 往复半幅 | `SRV_TONGZHI_POS_AMP_TURNS` | `6.5` 转 | 零点 ± 该值 |
| 到位阈值 | `SRV_TONGZHI_ARRIVE_THRESH_TURNS` | `0.25` 转 | |
| 梯形限速 | `SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS` | `20` 转/s | 调快转速上限 |
| 梯形加速度 | `SRV_TONGZHI_TRAJ_ACCEL_TPS2` | `8` 转/s² | 实际主导速度 |
| 梯形减速度 | `SRV_TONGZHI_TRAJ_DECEL_TPS2` | `8` 转/s² | |
| 目标重发 | `SRV_TONGZHI_TARGET_RESEND_MS` | `500` ms | 锁存式目标防丢帧 |
| 到位超时兜底 | `SRV_TONGZHI_ARRIVE_TIMEOUT_MS` | `60000` ms | |
| 定时翻转估计 | `SRV_TONGZHI_TRAVEL_EST_MS` | `2500` ms | 无反馈盲发用 |
| 状态日志 | `SRV_TONGZHI_STATUS_LOG_MS` | `5000` ms | |
| 耐久时长 | `SRV_TONGZHI_DURATION_MS` | `2592000000` ms | 30 天 |

---

## 5. Motorevo PA430（CAN2）— `srv_pa430_torque_test.c`

### 5.1 协议与接线

- 协议文档：`docs/Motorevo电机CAN协议文档.md`；CAN FD 广播帧 `0x10`（使能）/`0x20`（控制），
  DLC 64，每电机槽 `(ID-1)*8` 字节，多字节大端。
- 通道：FDCAN2（PB12 RX / PB13 TX），与 HT CAN2 通过 `SRV_MOTOR_TEST_SELECT_CAN2` 二选一。
- 控制：MIT 力位混合模式（Control Mode = 0x2），`T_out = Kp×(θ_ref−θ) + Kd×(V_ref−V) + T_ref`，
  每 5ms 重发广播 `0x20` 保持刚度（MIT 需持续下发）。

### 5.2 往复控制方案

1. **运动中心**：各电机**首次收到反馈帧时锁存该位置为自身中心**（`s_motor_center_raw[]`，
   仅一次），目标 = 中心 ± `SRV_PA430_ANGLE_AMP_RAD`（±1.6 rad）；中心锁存前目标保持
   中性（0x8000）等待。掉线恢复时对该电机**重新锁存**当前反馈位置为新中心再重启往复。
2. **目标递进限速**：`θ_ref` 每控制周期（5ms）向终点最多移动
   `SRV_PA430_TARGET_VEL_RADPS × 0.005` rad（默认 **1.5 rad/s**），指令速度受限→电机实际
   转速≈递进速度；`SRV_PA430_TARGET_VEL_RADPS = 0` 时关闭递进（直接跳变）。与 Kd 阻尼正交叠加限速。
3. **到位判定**：指令已递进到位 **且** 所有已收反馈电机 `|θ − 终点| ≤ SRV_PA430_ARRIVE_THRESHOLD_RAD`
   （0.5 rad）后翻转终点。
4. **Control Mode 自愈**（`SRV_PA430_CONFIGURE_MODE=1`）：启动读回 Index 11，若非 MIT(2)，
   状态机执行 失能 → 写 Control Mode=2 → 等 300ms 生效 → 读回校验 → 恢复使能；
   **不发送保存命令**（仅 RAM 生效，避免电机 Flash 擦写），校验失败仅告警。
5. **使能自愈**：在线但反馈使能位（Bit0）=0 的电机每 200ms 重发使能，直到 Bit0=1 确认
   （有活动错误位时跳过，故障消除后自动重新使能）。
6. **掉线/恢复**：2s 无反馈判定掉线；恢复在线时重新锁存中心并重启往复。
7. **启动诊断**：读回 Index 11（Control Mode）/21（Torque Limit）/69（Protocol，旧固件无此参数）/
   10（Firmware Version）并打印；STATUS 周期日志（默认 6s）含 θ/V/T/目标/使能/错误，便于确认电机在动。

### 5.3 关键参数

| 参数 | 宏 | 当前值 | 说明 |
|---|---|---|---|
| 往复半幅 | `SRV_PA430_ANGLE_AMP_RAD` | `1.6` rad | 中心 ± 该值 |
| MIT 刚度 | `SRV_PA430_KP_NMPR` | `6.5` Nm/rad | 调大更"硬"更快 |
| MIT 阻尼 | `SRV_PA430_KD_NMPRPDS` | `50` Nm/(rad/s) | 调大更慢更平稳 |
| 到位阈值 | `SRV_PA430_ARRIVE_THRESHOLD_RAD` | `0.5` rad | |
| 目标递进限速 | `SRV_PA430_TARGET_VEL_RADPS` | `1.5` rad/s | 0=关闭递进 |
| 控制周期 | `SRV_PA430_CTRL_PERIOD_MS` | `5` ms | 广播 0x20 重发 |
| 掉线判定 | `SRV_PA430_NORESP_PERIOD_MS` | `2000` ms | |
| 状态日志 | `SRV_PA430_STATUS_LOG_MS` | `6000` ms | 0=关闭 |
| 耐久时长 | `SRV_PA430_DURATION_MS` | `2592000000` ms | 30 天 |

### 5.4 已知坑（勿再踩）

- 电机固件早于 260617（如 `0x180426EF`）时**不支持单机寻址**（0x100+ID/0x200+ID），
  只能用广播 `0x10/0x20`；Index 69 无此参数（读回超时）。且此类电机可能拒绝使能态改写
  Control Mode，需先失能再写（状态机已处理）。
- 电机若为伺服模式（Index 11=1），MIT 组包会被按伺服字段解析（byte2≈0 → V_ref 限幅 0、
  byte3=0 → Kp_pos=0），电机使能但不动、无错误——务必先确认/改写为 MIT。
- `Set_Limits`/Torque Limit 在线改写曾导致电机不动，已撤回；不要通过参数帧在线限流。

---

## 6. 调参速查

| 想达到 | 模块 | 修改宏 |
|---|---|---|
| HT 转速快/慢 | srv_ht_* | `SRV_HT(_CAN2)_TORQUE_TEST_SPEED_RPM`（巡航转速）|
| HT 加减速平缓 | srv_ht_* | `SRV_HT(_CAN2)_TORQUE_TEST_RAMP_MS` |
| HT 行程 | srv_ht_* | `SRV_HT(_CAN2)_TORQUE_TEST_POS_LIMIT_DEG` |
| 良志转速快/慢 | srv_tongzhi | `SRV_TONGZHI_TRAJ_ACCEL_TPS2` / `SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS` |
| 良志行程 | srv_tongzhi | `SRV_TONGZHI_POS_AMP_TURNS` |
| PA430 速度 | srv_pa430 | `SRV_PA430_TARGET_VEL_RADPS`（指令递进限速）或 `SRV_PA430_KD_NMPRPDS` |
| PA430 行程 | srv_pa430 | `SRV_PA430_ANGLE_AMP_RAD` |

> 说明：HT 是速度模式（速度直接可控）；良志是梯形轨迹（速度=限速、加减速=限加）；
> PA430 是 MIT（无显式速度环，靠 Kd 阻尼 + 目标递进间接限速）。

### 6.1 每分钟往复次数估算（当前参数）

> **口径**：1 次（1 周期）= 一个完整往返 = **摆动两次（去 + 回）**。
> 若按单程（摆动一次）计数，表中次数翻倍。

| 电机 | 每程时间 | 周期（往返） | 单程次数/分 | 往返次数/分 |
|---|---|---|---|---|
| 苇熠 HT（CAN1/CAN2） | 斜坡 2s + 12 转@300RPM=2.4s → 4.4s | 8.8s | ≈ **13.6** | ≈ **6.8** |
| 良志 ODrive | 三角型 2×√(6.5/8)=1.80s | 3.6s | ≈ **33.3** | ≈ **16.7** |
| Motorevo PA430 | 2×AMP(3.2rad)/递进1.5rad/s=2.13s | 4.27s | ≈ **28.1** | ≈ **14** |

- HT 反向斜坡 +300→−300RPM 过零时电机在端点侧 overshoot 约 2.5 转后回到端点再巡航，
  故每程 = 斜坡 2s + 12 转巡航 2.4s。
- 良志峰值速度 10.2 转/s 未达限速 20，轨迹为三角型，单程时间 = 2×√(半幅/加速度)。
- PA430 稳态每程 = 端点间距 2×AMP 除以递进限速；首次锁存后首程为 1.07s，之后均按全幅。
- 提速方向：HT 调 `SPEED_RPM`/缩短 `RAMP_MS`；良志调 `TRAJ_ACCEL_TPS2`；
  PA430 调 `TARGET_VEL_RADPS`。

---

## 7. 故障排查要点

1. **电机不动 / 无日志**：先确认选择宏指向正确模块、总线/供电正常；PA430 优先看启动
   `参数回读` 的 Index 11 是否 = MIT(2)，再看 STATUS 日志中 `T` 是否偏离 0x800（0 Nm）。
2. **`chX ERROR-PASSIVE / BUS-OFF`**：`drv_can` 告警含 act/lec/tec/rec。lec=3 ACK 错（无人应答，
   查接线/终端电阻/检测器挂接）；lec=5 bit0（收发器未驱动/开路）；lec=4 bit1（短路或接反）。
   CAN FD 5M 数据段问题详见 `docs/can_fd_tdc_troubleshooting.md`。
3. **HT 只往一个方向跑**：查 `FLIP_TIMEOUT` 兜底日志、0x06 位置应答是否正常、
   `POS_LIMIT` 是否超出电机机械行程。
4. **良志闭环失败**：查 init 序列 5 帧是否发全、`0x07` 闭环确认是否收到；轴状态非 8 时
   每 200ms 从头重发；排查文档 `docs/tongzhi_motor_test_troubleshooting.md`。
5. **PA430 使能后不动**：确认 Control Mode=MIT；确认广播 0x20 帧被电机接收（无 `tx fail` 日志）；
   到位阈值/行程不超过机械限位。

---

## 8. 相关文件

- 模块实现：`service/srv_ht_torque_test.c/.h`、`service/srv_ht_can2_torque_test.c/.h`、
  `service/srv_tongzhi_torque_test.c/.h`、`service/srv_pa430_torque_test.c/.h`
- 选择宏：`service/srv_motor_test_select.h`
- 任务接线：`tasks/can_task.c`
- 驱动：`device_drivers/drv_can.c/.h`
- 协议文档：`docs/苇熠电机can协议文档.md`、`docs/良志电机can协议.md`、
  `docs/Motorevo电机CAN协议文档.md`
- 排查文档：`docs/can_fd_tdc_troubleshooting.md`、`docs/tongzhi_motor_test_troubleshooting.md`
