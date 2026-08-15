# 良志(ODrive) 往复耐久测试问题排查与修复总结

## 1. 问题现象

控制板（STM32G474，FDCAN1 PA11/PA12，经典 CAN 2.0A @ 1Mbps）驱动良志(ODrive) 伺服做
±12.5 转梯形轨迹往复耐久测试（`service/srv_tongzhi_torque_test.c`），出现以下现象：

1. **初始化检测到电机、有反馈数据，但电机不动**（先板后机 / 或直接发现后不运行）。
2. **电机先上电、控制板后上电：能转但报超速错误**（0x40 ENCODER_VELOCITY_LIMIT / 编码器速度超限）。
3. **控制板先上电、电机后上电：能被使能（闭环）但完全不能运行**。
4. 运行一段时间后**静默停摆**：不再正反转，控制台无错误、也不再打印「目标翻转」日志。

## 2. 根因分析

### 2.1 根因一（决定性）：init 5 帧连发超出 FDCAN TX FIFO 深度，梯形限速/限加永不生效

- STM32G4 FDCAN 消息 RAM 常量 `SRAMCAN_TFQ_NBR = 3U`（stm32g4xx_hal_fdcan.c:216）
  → **TX FIFO 深度固定为 3**。
- `drv_can_tx_ready()` 只判 `free > 0`，`drv_can_send()` 在 FIFO 满时**静默丢弃**该帧。
- 原 `srv_tongzhi_torque_test_motor_init()` 在一次调用内**连发 5 帧**：
  清错(0x18)→闭环(0x07)→控制模式(0x0B)→`Set_Traj_Vel_Limit`(0x11)→`Set_Traj_Accel_Limits`(0x12)。
  同一 tick 内连发，FIFO 装入前 3 帧后满，**第 4/5 帧（梯形限速/限加）被静默丢弃**。
- 结果：电机进入「位置+TRAP_TRAJ」模式，但梯形速度/加速度限制**从未下发**，按 ODrive
  默认（较高）的 `trap_traj_vel_limit/accel` 跑 ±12.5 转目标 → 编码器速度超限 → **0x40 超速错误**。
- 该缺陷同时解释了「先机后板能转但超速」。

### 2.2 根因二：fallback 在电机上电前就 `assumed_closed=true`，电机出现后 keep-alive 不再下发 init

- 发现窗口（`SRV_TONGZHI_FALLBACK_MS`=2000ms）在电机上电前结束 → `fallback()`
  （srv_tongzhi_torque_test.c:767）记录默认 node 并直接置 `s_assumed_closed=true`、
  `s_last_init_ms=now`，blind 发 init。
- 电机随后上电且以闭环状态启动（`startup_axis_state=CLOSED_LOOP`），心跳 axis_state=8：
  - `on_rx` 只在 `axis_state≠8` 时撤销假定 → 心跳报 8 时假定保留；
  - keep-alive 门控 `!s_assumed_closed[i]` → **永远 false** → `motor_init` 永不再发；
  - 电机停留在 ODrive 默认（速度）控制模式，`Set_Input_Pos` 被忽略 → **能闭环但不能运行**。

### 2.3 根因三：到位判定无超时兜底，反馈冻结/到位偏置导致静默停摆

- 到位翻转在 `all_have_encoder=true` 时走**位置判定分支**：
  `should_flip = all_arrived`，仅当 `|encoder − target| ≤ 0.05 转`。
- 若编码器 0x09 帧停发/冻结、或电机稳态位置误差 > 0.05 转（限位/负载/零位偏置），
  `should_flip` 恒为 false → 不再发新目标、不再打印「目标翻转」→ **完全静默停摆，且无超时兜底**。

## 3. 修复方法（`service/srv_tongzhi_torque_test.c`）

### 3.1 init 拆帧逐次下发（修根因一）

- 把 `send_traj_limits()` 拆成单帧 `send_traj_vel_limit()`(0x11) 与 `send_traj_accel_limits()`(0x12)。
- `motor_init()` 改为 `motor_init_step(node, step)`：5 帧按 **ODrive 推荐顺序**逐帧下发：
  ```
  step0 = Clear_Errors(0x18)
  step1 = Set_Controller_Mode(3=POSITION, 5=TRAP_TRAJ)
  step2 = Set_Traj_Vel_Limit(0x11)
  step3 = Set_Traj_Accel_Limits(0x12)
  step4 = Set_Axis_State(8=闭环)   ← 最后进闭环
  ```
- 新增每电机步进状态 `s_init_step[SRV_TONGZHI_MAX_MOTORS]`（0..4=待发帧序号，5=已发完）。
- keep-alive 每 `ENABLE_RETRY_MS`（200ms）**只推进 1 帧**（全局轮转），保证 5 帧全部进入
  TX FIFO 不被丢弃；序列发完仍未闭环则从头重发；已闭环则停止。
- `start()` / `scan_record()` / 恢复在线 均重置 `s_init_step`。

### 3.2 心跳到达即撤销 fallback 假定并强制重新 init（修根因二）

- `on_rx` 心跳分支：若 `s_assumed_closed[idx]` 为 true（fallback 盲发节点）→ 置 false、
  `s_last_init_ms[idx]=0`、`s_init_step[idx]=0`（心跳证明电机真实在线，按心跳重新走完整 init）。
- 恢复在线路径：撤销假定 + 重置 init 步进，让 keep-alive 按序重发。

### 3.3 到位超时强制翻转 + 周期诊断日志（修根因三）

- 位置判定分支改为 `should_flip = all_arrived || (now − s_last_flip_ms) ≥ SRV_TONGZHI_ARRIVE_TIMEOUT_MS`
  （新宏，默认 60000ms ≈ 2× 单程 25s）；触发超时时打印
  `目标到达超时，强制翻转（可能反馈冻结/到位偏置）`。
- 翻转日志区分 `到达超时 / 位置到位 / 定时`。
- 新增 `SRV_TONGZHI_STATUS_LOG_MS`（默认 5000ms）周期打印受控电机
  `node / axis_state / err / encoder 毫转 / target 毫转 / enc_ok / last_seen 年龄`，
  停摆时据此定位是「反馈冻结」「掉出闭环」还是「到位偏置」。

### 3.4 目标下发以 init 完成为门槛（加固）

- 新增 `srv_tongzhi_torque_test_target_ready(idx)`：
  受控（闭环或假定闭环）且（已假定闭环 或 `s_init_step ≥ 5`）。
- 翻转下发与周期目标重发均按此门槛，避免梯形限速/限加未就位时提前发 `Set_Input_Pos`
  导致电机按默认高速限制跑。

## 4. 关键诊断方法（经验）

1. **TX FIFO 深度**：G4 FDCAN TX FIFO 固定 3（`SRAMCAN_TFQ_NBR=3`），一次函数调用连发
   >3 帧必丢。逐帧下发/每周期限 1 帧是通用解法。
2. **超速 0x40 定位**：`0x00000040` 解码为 `ENCODER_VELOCITY_LIMIT`（编码器速度超限）。
   电机能转但报超速 → 优先怀疑梯形限速(0x11)/限加(0x12) 帧没送达，按默认高速跑。
3. **fallback 假定的副作用**：盲发节点假定闭环会永久阻断后续 init——任何「能闭环但不动」
   的现象先检查 `s_assumed_closed` 是否被提前置位。
4. **静默停摆判定**：无错误 + 无「目标翻转」日志 → 位置到位判定卡住；用周期状态日志
   （enc/target/axis/err）区分根因。
5. **CANable 抓帧**：确认 `0x11/0x12`（梯形限制）确实发出、`0x0C`（Set_Input_Pos）在
   init 完成后才出现。

## 5. 验证

- 两种上电顺序（先板后机 / 先机后板）均能完整 init → 正常 ±12.5 转往复，无超速、无静默停摆。
- 长期往复正常，超时兜底保证即使反馈异常也能自动恢复翻转。

## 6. 相关文件

- `service/srv_tongzhi_torque_test.c`（本次所有修改）
- `service/srv_tongzhi_torque_test.h`
- 协议参考：`docs/良志电机can协议.md`
