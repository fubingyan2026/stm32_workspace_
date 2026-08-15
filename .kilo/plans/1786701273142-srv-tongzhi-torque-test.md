# 新增 srv_tongzhi_torque_test（良志/ODrive 电机位置往复耐久测试）

## 目标

为良志(ODrive CAN Simple)伺服电机新增一个往复耐久测试服务 `srv_tongzhi_torque_test.c/h`，
参考 `srv_pa430_torque_test.c`（位置闭环来回）与 `srv_ht_torque_test.c`（往复耐久）的工程范式，
在 **FDCAN1（DRV_CAN_CH_1）** 上驱动多台电机做 **位置模式梯形轨迹往复** 耐久测试。

## 已确认决策（来自 Q&A）

| 项 | 决策 |
|----|------|
| CAN 总线 | 复用 FDCAN1（DRV_CAN_CH_1），与苇熠 HT 测试通过编译开关**二选一** |
| 控制模式 | 位置模式 + 梯形轨迹（`Set_Controller_Mode` control=3/input=5 + `Set_Input_Pos` 交替目标） |
| 波特率 | 1 Mbps（FDCAN1 现配即 1M，**不改 .ioc / drv_can.c**；电机侧参数需设为 1M） |
| 电机数量 | 多台；**心跳被动发现** node_id（监听周期心跳 0x01，动态收录在线电机） |

## 协议速查（docs/良志电机can协议.md）

- 经典 CAN 2.0A，8 字节帧，标准 11 位 ID，**小端**，IEEE-754 单精度浮点。
- **CAN ID = (node_id << 5) | cmd_id**，node_id 0~63，cmd_id 0~31。
- 主机→电机（均 DLC 8，小端）：
  - `0x07 Set_Axis_State`：`[0..3]=axis_state uint32`（8=CLOSED_LOOP_CONTROL），`[4..7]=0`
  - `0x0B Set_Controller_Mode`：`[0..3]=control_mode(3=POSITION)`，`[4..7]=input_mode(5=TRAP_TRAJ)`
  - `0x0C Set_Input_Pos`：`[0..3]=pos float32(转)`，`[4..5]=vel_ff int16(0.001 转/s)`，`[6..7]=torque_ff int16(0.001 Nm)`
  - `0x11 Set_Traj_Vel_Limit`：`[0..3]=vel_limit float32(转/s)`
  - `0x12 Set_Traj_Accel_Limits`：`[0..3]=accel float32(转/s²)`，`[4..7]=decel float32(转/s²)`
  - `0x18 Clear_Errors`：8×0
- 电机→主机（周期推送，无需主动轮询）：
  - `0x01 Heartbeat`（默认 100ms）：`[0..3]=axis_error uint32`，`[4]=axis_state`，`[5]=flags`(bit7=轨迹完成)，`[6]=temp int8`，`[7]=life`
  - `0x09 Get_Encoder_Estimates`（默认 10ms，仅闭环有效）：`[0..3]=pos float32(转)`，`[4..7]=vel float32(转/s)`

> 关键差异 vs PA430：ODrive `Set_Input_Pos` 是**锁存式目标**（梯形规划器自动跑到位并保持），
> **不需要**像 MIT 那样每 5ms 持续下发保持刚度。控制循环是事件驱动（到位翻转）+ 周期
> 重新确认闭环/模式，而非高频刚度维持。

## 改动清单（按顺序执行）

1. **新建 `service/srv_tongzhi_torque_test.h`**
   - 头文件保护、`#include "drv_can.h"`、`stdbool/stdint`。
   - `#ifndef SRV_TONGZHI_TORQUE_TEST_ENABLE` → `#define ... 0`（默认关闭，可在命令行 `-D` 覆盖）。
   - 导出 5 个 API：`init/start/stop/step/on_rx`（签名对齐 `srv_pa430_torque_test.h`）。
   - 中文 Doxygen 说明与「与苇熠二选一」的接线约定。

2. **新建 `service/srv_tongzhi_torque_test.c`**（主体，见下「模块设计」）
   - CMake 用 `aux_source_directory(service)` 自动收录，**无需改 CMakeLists**。

3. **修改 `tasks/can_task.c`**（FDCAN1 二选一接线）
   - 顶部加：`#if SRV_TONGZHI_TORQUE_TEST_ENABLE` 时 `#include "srv_tongzhi_torque_test.h"` 并令
     `CAN1_TEST_INIT/STEP/ON_RX` 指向良志模块；`#else` 走现有苇熠选择（`SRV_HT_TEST_MODE_TORQUE`）。
   - `can_task_init()`：`srv_can_init()` 仅在 `!SRV_TONGZHI_TORQUE_TEST_ENABLE` 时调用；`HT_TEST_INIT()` 替换为 `CAN1_TEST_INIT()`。
   - `can_timer_cb()`：`srv_can_process()` 同样用 `#if !SRV_TONGZHI_TORQUE_TEST_ENABLE` 包住；`HT_TEST_STEP()` → `CAN1_TEST_STEP()`。
   - `can_rx_callback()`：CH_1 分支在 `SRV_TONGZHI_TORQUE_TEST_ENABLE` 时调 `srv_tongzhi_torque_test_on_rx(msg)`，否则 `srv_can_on_rx(msg)`。
   - PA430（CH_2）路径保持不变（`SRV_PA430_TORQUE_TEST_ENABLE` 仍独立控制）。

## 模块设计要点（srv_tongzhi_torque_test.c）

### 配置宏（物理单位，可调）
```c
SRV_TONGZHI_AUTO_START           1     // 上电自动启动
SRV_TONGZHI_MAX_MOTORS           8     // 总线最多电机（node_id 上限 63，本模块限 8）
SRV_TONGZHI_POS_AMP_TURNS        2.5f  // 往复半幅（转）：目标在 ±AMP 交替
SRV_TONGZHI_ARRIVE_THRESH_TURNS  0.05f // 到位阈值（转）
SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS   2.0f  // 梯形限速（转/s）
SRV_TONGZHI_TRAJ_ACCEL_TPS2      4.0f  // 梯形加速度（转/s²）
SRV_TONGZHI_TRAJ_DECEL_TPS2      4.0f  // 梯形减速度（转/s²）
SRV_TONGZHI_NORESP_PERIOD_MS     2000U // 掉线判定（心跳 100ms，20 帧）
SRV_TONGZHI_ENABLE_RETRY_MS      200U  // 未闭环电机重发 闭环+模式+梯形参数
SRV_TONGZHI_TARGET_RESEND_MS     500U  // 周期重发当前目标（锁存式，重发同目标为安全 no-op）
SRV_TONGZHI_DURATION_MS          2592000000U // 累计在线 30 天自动停止
```

### 帧封装辅助
- `srv_tongzhi_can_id(node, cmd)` = `((node & 0x3F) << 5) | (cmd & 0x1F)`。
- `pack_float_le(uint8_t* dst, float v)`：`memcpy(dst, &v, 4)`（Cortex-M4 小端即 LE），
  避免 union（MISRA）；`pack_i16_le(dst, v)` / `pack_u32_le(dst, v)` 同理。
- 解包：`unpack_float_le/unpack_u32_le` 反向 memcpy。
- 所有发送帧：`is_extended=false, is_fd=false, dlc=8`，经 `drv_can_tx_ready(DRV_CAN_CH_1)` 门控。

### 状态与数据结构（静态分配，镜像 PA430）
- `s_running`、`s_dir`(1→+AMP)、`s_target_turns`(float)。
- `s_motor_ids[MAX]` + `s_motor_cnt`：心跳收录的 node_id 列表。
- 每电机：`last_seen_ms`、`nresp_latch`、`online_evt_pending`、`axis_state`、`axis_error`、
  `err_pending`、`encoder_turns`(float)、`have_encoder`、`mode_confirmed`(bool)。
- 全局：`start_ms/online_ms/online_last_ms`、`last_enable_retry_ms`、`last_target_resend_ms`。

### 流程
- **init**：`SRV_TONGZHI_AUTO_START=1` → `start()`。
- **start**：`running=true`，清零所有状态，`s_dir=1, s_target_turns=+AMP`；不主动发帧
  （电机上电后自推心跳，靠被动发现收录）。
- **on_rx（ISR，只记录不打日志）**：
  - 仅消费标准帧 DLC≥8；`node = id>>5, cmd = id&0x1F`；`node ≥ MAX_MOTORS` 丢弃。
  - 任何帧刷新 `last_seen_ms[node]`；若 `nresp_latch` 曾置位 → 清锁存 + `online_evt_pending=true`。
  - 未知 node 的心跳 → 记录到 `s_motor_ids`（`s_motor_cnt` 满则忽略），由主循环初始化。
  - `cmd==0x01`：解析 `axis_error(u32 LE)`、`axis_state`、`flags`、`life`；axis_error 变化置 `err_pending`。
  - `cmd==0x09`：解析 `pos float32 LE` → `encoder_turns`、`have_encoder=true`。
- **step（主循环）**：
  1. 持续在线计时（全部在线才累加）→ 满 `DURATION_MS` 自动 `stop()`。
  2. 新收录电机初始化：发 `Set_Axis_State(8=闭环)` + `Set_Controller_Mode(3,5)` +
     `Set_Traj_Vel_Limit` + `Set_Traj_Accel_Limits`；置 `mode_confirmed=false` 待心跳确认。
  3. 错误打印：`err_pending` 消费，raw hex + 已知位解码（文档 §5.3 表），变化即打印。
  4. 到位翻转：对每台「在线 && axis_state==8(闭环) && have_encoder」电机判
     `|encoder_turns − target| ≤ THRESH`；**全部到位**则 `s_dir` 翻转、`s_target_turns=±AMP`，
     对每台闭环电机发 `Set_Input_Pos(target)`（日志打印翻转，转数换算成整数 ×1000 打印，因 PRINTF 禁浮点）。
  5. 保持闭环：`axis_state != 8` 且在线、无活动错误的电机，每 `ENABLE_RETRY_MS` 重发
     闭环 + 模式 + 梯形参数（镜像 PA430 的使能重试；含超时告警）。
  6. 周期重发目标：每 `TARGET_RESEND_MS` 对闭环电机重发当前 `Set_Input_Pos`（安全 no-op，防漏帧）。
  7. 掉线检测：`now − last_seen ≥ NORESP` 且未锁存 → 告警 + `nresp_latch=true`。
  8. 恢复在线：`online_evt_pending` 消费 → 重发 闭环+模式+梯形参数 + 当前目标，打印日志。
- **stop**：发 `Clear_Errors` 可选，`Set_Axis_State(1=IDLE)`（或直接停发目标并置
  `running=false`）；打印停止日志。

### 日志
- 用 `SRV_TONGZHI_TORQUE_TEST_LOG_I/W/E` 宏 + `LOG_I("tongzhi_test", ...)`；`PRINTF_DISABLE_SUPPORT_FLOAT`
  已定义，**所有浮点转整数 ×1000** 后打印。

## 边界 / 风险

1. **node_id 冲突与低 ID**：node_id=0 时 CAN ID 仅为 cmd_id（0x01/0x09…），与 0x10/0x20
   不冲突（PA430 在 CH_2），旧 0x100 协议在 `SRV_TONGZHI_TORQUE_TEST_ENABLE` 时已停用，无冲突。
2. **闭环前置条件**：ODrive 需先完成电机+编码器校准并保存，`Set_Axis_State(8)` 才能直接进闭环；
   否则电机报错。**假设电机已预校准保存**（良志为成品执行器）；未闭环时模块按「保持闭环重试」
   周期重发并在超时后告警，不阻塞。
3. **编码器仅在闭环有效**：到位判定以 `axis_state==8` 为前提，未闭环电机不参与「全部到位」判定。
4. **浮点打印禁用**：日志全部定点化，避免链接 mpaland_printf 浮点分支。
5. **命名**：用户指定前缀 `srv_tongzhi_torque_test`，但协议文档标题为「良志」；若需改前缀
   （liangzhi）仅涉及文件名与符号重命名，无架构影响。

## 验证

1. `cmd /c build.bat`（项目根目录）编译通过，无新增告警（关注 MISRA/未用变量）。
2. 默认 `SRV_TONGZHI_TORQUE_TEST_ENABLE=0` 时构建应**不引入**新符号进入 can_task 热路径，
   现有苇熠/PA430 行为不变（回归）。
3. 置 `SRV_TONGZHI_TORQUE_TEST_ENABLE=1` 烧录，检测器挂 CAN1（1M 经典）观察：
   - 电机上电后固件日志出现「发现电机 node_id=…」与「已确认闭环」。
   - 目标在 ±AMP 之间翻转，`Set_Input_Pos` 帧（ID=(node<<5)|0x0C）周期出现。
   - 拔电/断电后日志出现掉线告警，恢复后自动重发闭环并继续往复。
4. 验证 CAN ID 编码：确认检测器收到的帧 ID = `(node_id<<5)|cmd`，数据段为小端 float。

## 遗留（可后置）

- `Get_Error`(0x03) 详细错误码查询未纳入首版，先用心跳 `axis_error` uint32 解码。
- 梯形 `Set_Traj_Inertia`(0x13) 未纳入，采用默认惯量。
- 校准/回零自动化未纳入（依赖电机预配置）。

## 故障诊断：电机无反应、启动日志后无任何后续（2026-08-14 现场）

### 已确认的事实
- `SRV_TONGZHI_TORQUE_TEST_ENABLE=1` 已生效，`build/RelWithDebInfo`(18:19) 固件中
  `can_task.o` 引用 `srv_tongzhi_torque_test_*`（苇熠路径已编译掉），模块**确实在跑**；
  FDCAN1 RX 中断链路完好（stm32g4xx_it.c 有 FDCAN1_IT0/IT1 → HAL_FDCAN_IRQHandler）。
- 控制台能见到「良志(ODrive) 往复启动」日志 → 模块 running=true，step 每 5ms 执行。
- **无**「发现电机 node_id=…」→ 从未收到任何心跳/编码器帧。
- **无** `ch1 ERROR-PASSIVE`/`ch1 BUS-OFF!`/`ch1 tx fail` 日志 → 总线上也**没有不匹配波特率
  的杂散信号**（若电机在 500k 发帧、主机听 1M，REC 会在 1~2s 内冲上 128/256，必然出现 EP/BO 日志）。

### 根因判定
固件侧无问题；**良志电机没有往 FDCAN1 总线发任何东西**。候选原因（按概率）：
1. 电机未供电 / CAN 未使能（`odrv0.can.config.enable`、`comm_intf_mux`=CAN、`heartbeat_rate_ms`>0）。
2. 电机接到的是 **FDCAN2（PB12/PB13）那条总线**（原 PA430 用的），而非 FDCAN1（PA11/PA12，原苇熠用的）——
   tongzhi 模块只听 CH_1，自然收不到。
3. CANH/CANL 接反、未接终端电阻（120Ω）、收发器到电机连接断开。
4. 电机波特率确实为 500k 且不发送（需用分析仪确认，见下）。

### 下一步动作（按顺序）
1. **确认物理接线**：良志电机必须接在 FDCAN1（PA11/PA12）对应的 CAN 连接器上（原苇熠电机的位置），
   不是 FDCAN2（原 PA430）的位置；检查 CANH/CANL 与终端电阻。
2. **确认电机供电**：驱动器母线供电（24/48V）是否正常、电流是否足够。
3. **总线抓帧定位波特率**：用 USB-CAN 分析仪（CANable）分别以 500k 与 1M 挂到电机总线上抓帧：
   - 500k 抓到心跳(0x01)/编码器(0x09)帧 → 电机在 500k 运行 → **决策点**：把电机参数改为 1M，
     或把固件 FDCAN1 名义位时序改为 500k（见下）；
   - 1M 抓到帧 → 波特率没问题，问题在主机侧 RX/接线，需进一步查；
   - 500k/1M 都抓不到 → 电机未发数据（供电/CAN 未使能/接线/终端电阻）。
4. **可选固件诊断增强**（若 3 无法执行）：在模块中加「启动后 X 秒仍未发现电机」周期日志，
   及每 N 秒输出 CH_1 `REC/TEC/RX帧计数`（drv_can 目前只在 EP/BO 边沿与 TX FIFO 满时打日志），
   区分「总线完全无声」vs「有信号但帧错误」。

### 波特率决策点（待抓帧结果确认）
- 电机若在 **500k** 且用户不便改电机参数：
  - 方案 A（首选）：通过厂商/ODrive 工具把电机波特率改为 1M，固件不变。
  - 方案 B：把 FDCAN1 名义位时序改为 500k（改 `.ioc` 的 NominalPrescaler=16 并重新生成，
    或 drv_can_init 里按宏 MODIFY_REG NBTP 覆盖），同时苇熠 1M 将无法复用同一固件。
- 电机若在 **1M** 且 500k/1M 都无帧：回到第 1/2 步查供电/接线/CAN 使能。

## 现场诊断结论（2026-08-14 二次确认）

- 控制台只有一条「良志(ODrive) 往复启动」，之后永远沉默；无「发现电机」、无 `ch1 ERROR-PASSIVE/BUS-OFF/tx fail`。
- CANable 在 **500k 和 1M 下抓 FDCAN1 总线均完全无声**。
- 结论：电机已供电、接对 FDCAN1，但 **CAN 已使能只是不发周期帧（`heartbeat_rate_ms=0`/`encoder_rate_ms=0`），
  或 CAN 完全未使能**。当前模块**纯被动心跳发现** → 对这种电机永远发现不了、永远静默。
- 用户决策：**不改电机配置，直接加固件主动探测**。

## 固件增强任务：主动探测 + 静默电机适配（srv_tongzhi_torque_test.c）

### 新常量
```c
SRV_TONGZHI_PROBE_INTERVAL_MS   200U   // 主动探测周期：每周期对下一个候选 node 发 Get_Error
SRV_TONGZHI_FALLBACK_MS         2000U  // 发现窗口：启动后该时长内探测 0~MAX-1；结束后若仍无发现则回退
SRV_TONGZHI_FALLBACK_NODE       0U     // 回退默认 node_id（镜像 HT 的 DEFAULT_MOTOR_ADDR）
SRV_TONGZHI_INIT_GRACE_MS       1000U  // init 下发后该时长内无心跳 axis_state=8 确认 → 假定已闭环
SRV_TONGZHI_TRAVEL_EST_MS       2500U  // 无位置反馈时的单程移动估计时间（按 amp/vel/accel 粗调）
SRV_TONGZHI_NO_MOTOR_LOG_MS     5000U  // 未发现电机时周期告警日志间隔
```

### 新状态
```c
s_probe_idx, s_last_probe_ms, s_probe_done, s_fallback_active;
s_last_init_ms[MAX];   // 每电机最近一次下发 init 的时间（用于假定闭环计时）
s_assumed_closed[MAX]; // 假定已闭环标志（无心跳确认时置位）
s_last_flip_ms;        // 最近一次目标翻转时间（定时翻转用）
s_last_no_motor_log_ms;
```

### 新函数
- `srv_tongzhi_torque_test_send_get_error(node)`：`0x03`，`data[0]=0`(error_type=motor)，DLC 8。
- `srv_tongzhi_torque_test_probe_step(now)`：每 `PROBE_INTERVAL_MS` 对 `s_probe_idx` 发 Get_Error，
  `s_probe_idx` 在 0~MAX-1 轮转；已发现的 node 跳过；`now-start ≥ FALLBACK_MS` 后置 `s_probe_done=true` 停止探测
  （限制失败帧数：≤10 次 × TEC+8 = 80 < 128，避免探测把主机打进 BUS-OFF）。
- `srv_tongzhi_torque_test_fallback(now)`：`s_probe_done && s_motor_cnt==0` 时，把 `FALLBACK_NODE` 加入列表、
  打印告警、走 `motor_init`（盲发），`s_fallback_active=true`。

### on_rx 改动（可选）
- 现有逻辑已实现「任意帧即收录 node」→ Get_Error 回包(0x03) 会自动完成发现，无需改发现路径。
- 可选：`cmd==0x03` 回包取 `data[0..3]` 作为 axis_error 小端喂入 `s_motor_err/err_pending`。

### step() 改动
1. **主动探测**：插入 `srv_tongzhi_torque_test_probe_step(now)`（在在线计时之后、错误打印之前）。
2. **回退盲发**：插入 `srv_tongzhi_torque_test_fallback(now)`。
3. **假定闭环**：对每台已 init 电机，`now-last_init_ms ≥ INIT_GRACE_MS && axis_state!=8` → `s_assumed_closed[i]=true`（打印一次）。
4. **到位翻转改双模式**：
   - 「受控」= `axis_state==8`（心跳确认）或 `s_assumed_closed[i]`。
   - 若所有受控电机都有 `have_encoder` → 沿用现有位置到位判定（受控电机参与）。
   - 否则（存在无反馈电机）→ **定时翻转**：`now-s_last_flip_ms ≥ TRAVEL_EST_MS` 且至少一台受控电机 → 翻转。
   - 翻转时更新 `s_last_flip_ms` 并下发新目标。
5. **保持闭环重试**：条件改为 `axis_state!=8 && !s_assumed_closed[i] && err==0`，避免假定闭环后反复重发。
6. **周期重发目标**：目标重发对象由 `axis_state==8` 改为「受控」电机。
7. **未发现电机周期日志**：`s_motor_cnt==0` 且每 `NO_MOTOR_LOG_MS` 打印
   「仍未发现电机，已主动探测 node 0~%u」。
8. 被动心跳发现、掉线检测、恢复在线逻辑保留不变（静默电机掉线仅告警一次，不影响盲发驱动）。

### 验证（此固件烧录后）
1. 控制台每 5s 出现「仍未发现电机…」→ 模块探测路径存活。
2. 若出现 `ch1 tx fail`/`ERROR-PASSIVE`/`BUS-OFF` → 电机在 1M 下无 ACK：
   - 大概率电机实际在 **500k**（文档默认）→ 进入「波特率决策点」：试把 FDCAN1 改 500k（见下）。
   - 或电机 CAN 完全未使能 → 必须用厂商工具配置，固件无法解决。
3. 若出现「发现电机 node_id」→ Get_Error 回包被发现 → 继续观察是否进入闭环/往复；
   若仍无反馈帧 → 走 `s_assumed_closed` + 定时翻转，电机应往复摆动。
4. 确认电机开始 ±AMP 往复、且无持续 BO 抖动。

### 波特率快速试验（条件性，若 1M 无 ACK）
- 在 `drv_can.c` 加编译宏 `DRV_CAN_CH1_NOMINAL_500K`（默认 0）：置 1 时在 `drv_can_init`
  中对 FDCAN1 用 `MODIFY_REG(NBTP, NSJW|NTSEG1|NTSEG2|NBRP, ...)` 把名义位时序改为 500k
  （NBRP 翻倍：160MHz/(16×20)=500k），仅当 `SRV_TONGZHI_TORQUE_TEST_ENABLE=1` 生效。
- 用该宏与 1M 两版分别烧录，配合第 2 条日志判断电机实际波特率。
- 注意：500k 与苇熠 1M 互斥，属诊断用途，定位后改回 1M 或按需保留。

## 实现状态（2026-08-14 18:56）

**已完成并构建通过**（build/RelWithDebInfo 18:56:32，RAM 23584B / FLASH 46480B）：

- 新增常量：`PROBE_INTERVAL_MS/FALLBACK_MS/FALLBACK_NODE/INIT_GRACE_MS/TRAVEL_EST_MS/NO_MOTOR_LOG_MS`、`CMD_GET_ERROR`。
- 新增状态：`s_probe_idx/s_last_probe_ms/s_probe_done/s_fallback_active/s_last_init_ms[]/s_assumed_closed[]/s_last_flip_ms/s_last_no_motor_log_ms`。
- 新增函数：`send_get_error`（0x03 error_type=0）、`probe_step`（发现窗口内轮转探测 0~MAX-1，窗口结束置 probe_done 限帧防 BUS-OFF）、`fallback`（无发现回退默认 node 盲发+假定受控）、`in_control`。
- `on_rx`：Get_Error 回包解析 axis_error（兼作在线信号）；心跳 axis_state≠8 时撤销假定闭环。
- `step()`：探测→回退→无电机周期告警→假定闭环→双模式翻转（全有反馈=位置判定，否则定时）→
  目标重发与保持闭环均按「受控」判定。
- 语法检查 `-Wall` 与完整构建均无告警。

**待现场验证**：
1. 烧录 build/RelWithDebInfo/E1_Hand_G474.hex（18:56:32，tongzhi 已编入）。
2. 若电机在 1M：应出现「发现电机 node_id」→ init →（有反馈走位置翻转 / 无反馈走定时翻转往复）。
3. 若出现「仍未发现电机」+ `ch1 tx fail/EP/BO`：电机不在 1M 应答 → 实施「波特率快速试验」宏
   （drv_can.c 加 `DRV_CAN_CH1_NOMINAL_500K` 切 500k 再试）。

## 现场验证进展（2026-08-14 19:02）：通信已打通，电机带活动错误 0x40

### 已确认
- 主动探测**成功**：控制台出现 `tongzhi_test: 电机 node=1 错误：0x00000040`。
- 这说明 **1M 通信完全正常**（Get_Error 回包发现 node=1；之前总线静默正是因为电机
  `heartbeat_rate_ms=0`，只应答命令、不推周期帧）。
- 电机当前有**活动错误 0x40**：error_type=0 请求的是 motor error(64bit)，
  ODrive `MOTOR_ERROR_ENCODER_VELOCITY_LIMIT`（0x40）；若按 axis.error 解释则为
  `FET_THERMISTOR_OVER_TEMP`。很可能是上次运行遗留的锁存错误。

### 直接根因（固件缺陷）
- keep-alive 的 `need_enable` 条件含 `s_motor_err[i] == 0U`：电机带活动错误时**拒发
  motor_init（内含 Clear_Errors 0x18）** → 错误永远清不掉 → 永远进不了闭环 → 永远不动。
  （该门控是从 PA430「保护中跳过避免反复顶撞」照搬的，对 ODrive「必须 Clear_Errors 才能
  恢复」不适用。）

### 修复任务（srv_tongzhi_torque_test.c）
1. **移除 keep-alive 的 err==0 门控**：`need_enable` 与发送循环改为
   `axis_state!=8 && !assumed_closed`（去掉 `err==0`），使带错误的电机也能收到
   motor_init（清错→闭环→模式→梯形），完成锁存错误恢复。
2. **Clear_Errors 帧格式改为 ODrive 标准**：`send_clear_errors` 发送
   `data[0..3]=1`(clear_errors=true)、`data[4..7]=0`(不清其它轴)。
   - 注：良志文档写「8 字节 0」，但 ODrive 实现要求 clear_errors 标志置 1，文档存疑；
     先按 ODrive 标准（1），若错误仍不消除则回退试 8×0。
3. **发现窗口后周期回查已发现电机的错误状态**：扩展 `probe_step`——
   - 窗口内（未 probe_done）：照旧轮转探测未发现候选 node 0~MAX-1；
   - 窗口后（probe_done）：轮转对已发现电机周期发 Get_Error（新增 `s_err_query_idx`
     状态，复用 `PROBE_INTERVAL_MS` 节奏）。作用：错误状态变化可打印（on_rx 已按 0x03
     回包更新 err）、静默电机 last_seen 持续刷新（不再误报掉线）。
4. **err_map 增加 0x40 等 ODrive 位解码**：motor.error 0x40=ENCODER_VELOCITY_LIMIT
   （编码器速度超限）；保留原始 hex 打印，未知位不崩溃。

### 验证（重烧后）
1. 期望日志顺序：`发现电机 node=1` →（Clear_Errors 生效）`电机 node=1 错误已消除，恢复正常`
   → 进入闭环（有心跳则「已确认闭环」，无心跳则 1s 后「假定已闭环」）→ 开始 ±2.5 转往复。
2. 若 0x40 **持续存在**（Clear_Errors 无效）：
   - 试 Clear_Errors 改回 8×0（文档格式）；
   - 或电机未校准（文档 §七「首次使用需电机+编码器校准并保存」）→ 需厂商工具校准；
   - 或 0x40 为真实故障（板温/编码器速度配置），需查电机侧配置。
3. 若电机动起来：确认定时翻转（无反馈）或位置翻转（有编码器帧）正常往复，收尾。

## 现场问题（2026-08-14 19:20）：已确认闭环但电机完全不动

### 现场日志
```
I (5) tongzhi_test:   发现电机：node_id = 1
I (5) tongzhi_test: 电机 node=1 已确认闭环
之后就没有运动了！（无「目标翻转」日志）
```

### 已确认事实
- 电机 node=1 通过**心跳（0x01）被动发现**，且首帧心跳 `axis_state` 已经是 8（两条日志同 tick t=5ms）。
- 心跳在收、无 BUS-OFF/EP/tx fail、无「错误：0x…」日志。
- 无「目标翻转 → …（定时）」日志 → **编码器帧(0x09)在收**，`all_have_encoder=true`，走位置到位判定；
  目标 ±12.5 转从未被电机执行，`diff` 恒 > 0.05 → 永不翻转、永不发翻转日志。与现象完全一致。

### 根因（固件缺陷）
`step()` 的 keep-alive 门控（srv_tongzhi_torque_test.c:506-533）把「下发 motor_init」限制在
`axis_state != 8` 的电机上：
- motor_init 内含 `Set_Controller_Mode(3=POSITION, 5=TRAP_TRAJ)` + `Set_Traj_Vel_Limit` +
  `Set_Traj_Accel_Limits`（这是 Set_Input_Pos 能被执行的**前置条件**）。
- 电机若以 `startup_axis_state=CLOSED_LOOP` 启动（或上次会话遗留闭环未断电），发现时心跳已报 8 →
  `need_enable=false` → **模式/梯形参数永不发送**。
- 电机停留在 ODrive 默认控制模式（VELOCITY + VELOCITY_RAMP），`Set_Input_Pos(0x0C)` 被忽略 → 完全不动。
- 另外 ODrive trap_traj 的 `trap_traj_vel_limit/accel/decel` 默认值可能是 0，即使进了位置模式，
  不显式下发 0x11/0x12 也可能 0 速。因此**无论电机当前是否闭环，新发现电机都必须完整 init 一次**。

### 修复任务（srv_tongzhi_torque_test.c）
1. **keep-alive 门控加入「从未 init」条件**：`need_enable` 判定与发送循环均改为
   `!s_assumed_closed[i] && ((axis_state != 8) || (s_last_init_ms[i] == 0U))`：
   - 已闭环但从未 init（s_last_init_ms==0）的电机 → 补发一次完整 motor_init（清错/闭环/模式/梯形限速），
     之后 s_last_init_ms 置位、axis_state==8 → 不再重发；
   - 未闭环电机行为不变（周期重发直到确认闭环）。
2. **首次 init 补一条日志**：`s_last_init_ms[i]==0 → 置位` 时打印
   `SRV_TONGZHI_TORQUE_TEST_LOG_I("电机 node=%u 首次下发初始化（补发模式/梯形参数）", ...)`，
   便于现场确认补发路径已走。
3. **确认重发节奏**：保持 `ENABLE_RETRY_MS`(200ms) 不变；补发后下个 tick 条件自然失效，不会重复刷帧。

### 验证（重烧后）
1. 期望日志顺序：
   `发现电机 node_id=1` → `电机 node=1 首次下发初始化（补发模式/梯形参数）` →
   `电机 node=1 已确认闭环` → `目标翻转 → 12500 毫转（位置到位）` → 开始 ±12.5 转往复。
2. 若补发后仍不动：用 CANable 抓 0x2B(Set_Controller_Mode)、0x31/0x32(梯形限速) 帧是否真发、
   电机是否回错误；核对 `SRV_TONGZHI_POS_AMP_TURNS=12.5`、`TRAJ_VEL_LIMIT_TPS=0.5`（单程 25s，偏慢但可见）。
3. 确认往复正常后按需把限速调快（如 2.0 转/s）再收尾。

## 现场问题（2026-08-15）：能往复但「很容易」就静默停摆，无错误无翻转日志

### 现场现象（用户确认）
- 初始化检测到电机、有反馈数据（心跳 + 编码器 0x09 在收），开始正反转正常。
- 运行一段时间后**静默停摆**：不再正反转，控制台**无任何错误日志**、也不再打印
  「目标翻转」的运行状态。用户明确：停摆那一刻**无「错误：0x…」**。

### 已排除
- 电机报错掉出闭环（0x40/0x01000000）→ 排除了（无错误日志，且 axis_state 应仍为 8）。
- keep-alive 门控问题（上一条已修）→ 不是本次直接原因。

### 根因（固件缺陷，高置信）
到位翻转走**位置判定分支**（`all_have_encoder=true`，因编码器帧在收）：
```c
should_flip = all_arrived;   // 仅当 |encoder − target| ≤ 0.05 转
```
一旦该条件永久不满足，`should_flip` 恒为 false → 不再发新目标、不再打印「目标翻转」→
**完全静默停摆**，且没有任何超时兜底。触发「条件永久不满足」的候选：
1. 编码器 0x09 帧停发/冻结（`s_motor_encoder_turns` 停在旧值，`diff` 恒 > 0.05）；
2. 电机停在端点但稳态位置误差 > 0.05 转（TRAP_TRAJ 到位后位置环增益不足/负载所致）；
3. 编码器零位与 ±12.5 转假设不一致（到达「目标+偏置」，`|enc−target|` 恒 = 偏置 > 0.05）。
- 当前**没有任何诊断日志**，停摆时无法区分以上三种；也没有超时强制翻转，一旦停就永远停。

### 修复任务（srv_tongzhi_torque_test.c）
1. **到位超时强制翻转兜底**（核心）：位置判定分支改为
   `should_flip = all_arrived || ((now - s_last_flip_ms) >= SRV_TONGZHI_ARRIVE_TIMEOUT_MS)`；
   新增宏 `SRV_TONGZHI_ARRIVE_TIMEOUT_MS`（默认 60000U ≈ 2× 单程 25s）。触发超时时用
   `SRV_TONGZHI_TORQUE_TEST_LOG_W("目标到达超时，强制翻转（可能反馈冻结/到位偏置）")` 打印一次，
   使往复永不永久停摆。
2. **周期诊断日志**（定位真正的停摆原因，否则永远靠猜）：新增 `SRV_TONGZHI_STATUS_LOG_MS`
   （默认 5000U）。每该周期对每台受控电机打印一条：
   `node / axis_state / error / encoder 毫转 / target 毫转 / have_encoder / last_seen 距今年龄`。
   停摆时据此判定是「反馈冻结」「掉出闭环」「到位偏置」哪一种。
3. **评估到位阈值与轨迹完成标志**（可选，依赖第 2 步日志结论）：
   - 若日志显示稳态误差 > 0.05 转 → 放宽 `SRV_TONGZHI_ARRIVE_THRESH_TURNS`（0.05→0.1）；
   - 若显示编码器有固定偏置 → 目标改用「增量式」（相对当前位置 ±AMP）而非绝对 ±AMP；
   - 若 ODrive 心跳 data[5] bit7「轨迹完成」可靠 → 作为到位判定的更优信号（`traj_done` 为
     权威「规划器跑完」指示，可 OR 进 `all_arrived`），减少对位置阈值的依赖。

### 验证（重烧后）
1. 期望：往复长时间持续；若停摆，控制台出现「目标到达超时，强制翻转」→ 往复自动恢复；
2. 5s 周期诊断日志在停摆瞬间打印出 `encoder/target/axis_state/error`，据此锁定第 3 步
   应采用的最终修法（放宽阈值 / 增量目标 / traj_done 判定）。
3. 反复上电跑 10 分钟确认不再静默停摆。

## 现场问题（2026-08-15）：上电顺序决定电机行为（先板后机=能使能但不动；先机后板=能转但超速）

### 现场现象（用户确认）
- **控制板先上电、电机后上电**：电机能被使能（闭环），但**不能运行**（Set_Input_Pos 不生效，不往复）。
- **电机先上电、控制板后上电**：电机能转，但**会报超速错误**（0x40 ENCODER_VELOCITY_LIMIT / 编码器速度超限）。

### 根因 1（决定性）：TX FIFO 只有 3 深，motor_init 的 5 帧连发丢 2 帧 → 梯形限速/限加永不生效
- G4 FDCAN 消息 RAM 常量 `SRAMCAN_TFQ_NBR = 3U`（stm32g4xx_hal_fdcan.c:216）→ **TX FIFO 深度固定 3**。
- `drv_can_tx_ready()` 只判 `free > 0`（drv_can.c:349-355），`drv_can_send()` 在 FIFO 满时静默丢弃。
- `srv_tongzhi_torque_test_motor_init()`（srv_tongzhi_torque_test.c:926-932）**一次调用连发 5 帧**：
  清错(0x18)→闭环(0x07)→控制模式(0x0B)→`Set_Traj_Vel_Limit(0x11)`→`Set_Traj_Accel_Limits(0x12)`。
  同一 tick 内连发，FIFO 装入前 3 帧后满，**第 4/5 帧（梯形限速/限加）被静默丢弃**。
- 结果：电机进入「位置+TRAP_TRAJ」模式但**梯形速度/加速度限制从未下发**，按 ODrive 默认
  （较高的）trap_traj_vel_limit/accel 跑 ±12.5 转目标 → 编码器速度超限 → **0x40 超速错误**。
- 该缺陷同时解释了「先机后板能转但超速」（初始化下发时电机已在，但限速帧丢了）。

### 根因 2：fallback 在电机上电前就 `assumed_closed=true`，电机出现后 keep-alive 不再下发 init
- 发现窗口（FALLBACK_MS=2000ms）在电机上电前结束 → `fallback()`（srv_tongzhi_torque_test.c:751-736）
  记录 FALLBACK_NODE 并直接置 `s_assumed_closed=true`、`s_last_init_ms=now`，blind 发 motor_init。
- 电机随后上电且以闭环状态启动（startup_axis_state=CLOSED_LOOP），心跳 axis_state=8：
  - `on_rx`（srv_tongzhi_torque_test.c:624-626）**只在 axis_state≠8 时撤销假定** → 心跳报 8 时假定保留；
  - keep-alive 门控 `!s_assumed_closed[i]`（line 545）→ **永远 false** → `motor_init` 永不再发；
  - 电机停留在 ODrive 默认（速度）控制模式，`Set_Input_Pos` 被忽略 → **能闭环但不能运行**。

### 修复任务（srv_tongzhi_torque_test.c）
1. **motor_init 拆成逐帧步进（每 200ms 一帧），保证 5 帧全部可靠送达**
   - 新增 `s_init_step[SRV_TONGZHI_MAX_MOTORS]`（0..4=下一帧序号，5=完成）。
   - 把 `send_traj_limits()` 拆成 `send_traj_vel_limit()`(0x11) 与 `send_traj_accel_limits()`(0x12)，各单帧。
   - 新增 `motor_init_step(node, step)`：step0=清错、step1=控制模式、step2=梯形限速、step3=梯形限加、
     step4=闭环(Set_Axis_State 8)。**按 ODrive 推荐顺序：先配置模式/限速再进闭环**，避免进闭环后
     短暂按默认限速运行。
   - keep-alive 循环改为每 `ENABLE_RETRY_MS` 对每台电机推进 1 帧；`step==5 && 未闭环` → 重置 step=0
     重发；首次 init 置 `s_last_init_ms` 并打印（保留现有日志）。
   - start()/scan_record/恢复在线 均重置 `s_init_step[i]=0`。
2. **目标下发/翻转以「init 完成」为门槛**
   - 新增判定：电机可下发目标 = `in_control && (s_assumed_closed[i] || s_init_step[i] >= 5)`；
     翻转与目标重发都按此门槛，避免 init 未完成（梯形限制未就位）时提前发 Set_Input_Pos。
3. **心跳到达即撤销 fallback 假定并强制完整重新 init（修根因 2）**
   - `on_rx` 心跳分支：若 `s_assumed_closed[idx]` 为 true（fallback 盲发节点）→ 置 false、
     `s_last_init_ms[idx]=0`、`s_init_step[idx]=0`（心跳证明电机真实在线，按心跳状态重新完整 init）。
4. **清理 fallback 幽灵节点（可选加固）**：`s_fallback_active` 且发现真实电机（node≠FALLBACK_NODE）
   时，从列表中移除/忽略 fallback 幽灵节点，避免其（无编码器）把翻转拖入定时模式并误导到位判定。

### 验证（重烧后）
1. 控制板先上电、稍后电机上电：应出现「发现电机 node_id」→「首次下发初始化」→「已确认闭环」→
   「目标翻转」→ 正常 ±12.5 转往复，无超速。
2. 电机先上电、控制板后上电：同样完整 init，往复正常，**不再报 0x40 超速**（梯形限速 0.5 转/s 已送达）。
3. 用 CANable 抓总线确认 `Set_Traj_Vel_Limit(0x11)`/`Set_Traj_Accel_Limits(0x12)` 帧确实发出、
   `Set_Input_Pos(0x0C)` 在 init 完成后才出现。
4. 两种上电顺序各跑 10 分钟，确认行为一致、无超速/静默停摆。

