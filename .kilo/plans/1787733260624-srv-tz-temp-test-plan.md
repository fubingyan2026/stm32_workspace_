# Plan: 新增 srv_tz_temp_test 服务 — 良志(ODrive) 速度模式 24H 耐久测试

## 目标

新建 `service/srv_tz_temp_test.c/h`：控制良志(ODrive) 电机速度模式，自动扫描总线电机 ID、
读取实时反馈（编码器位置/速度、轴状态、错误码），按「正转 30s → 停 30s → 反转 30s → 停 30s」
循环运行，持续在线累计满 24h 自动结束。参考 `service/srv_tongzhi_torque_test.c` 的协议/架构。

## 已确认决策

| 项 | 决定 |
|----|------|
| 测试转速 | 20 转/s（Set_Input_Vel float32，正转 `+20.0f`，反转 `-20.0f`） |
| 启停斜坡 | 主机侧 2s 线性斜坡（PASS_THROUGH 输入模式，主机逐帧插值，不依赖电机端 ramp 配置） |
| 激活方式 | `srv_motor_test_select.h` 新增第 4 个 CAN1 选项 `SRV_MOTOR_TEST_TZ_TEMP = 3`，旧 TONGZHI 保留可切换 |
| 24h 计时 | 与参考模块一致：仅当所有已发现电机在线时累计 `s_online_ms`，满 `SRV_TZ_TEMP_DURATION_MS = 86400000` 自动停止 |
| 日志语言 | 中文（与 srv_tongzhi_torque_test 一致；「英文日志」约束仅针对巨蟹/扭矩传感器） |

## 协议事实（docs/良志电机can协议.md）

- 经典 CAN 2.0A，1 Mbps，标准 11 位 ID：`CAN-ID = (node_id<<5) | cmd_id`，DLC 8，小端，float32 IEEE-754。
- 本模块走 FDCAN1（`DRV_CAN_CH_1`），与 srv_tongzhi_torque_test 互斥（select 宏）。
- 速度模式用帧：
  - `0x0B Set_Controller_Mode`：data[0..3]=control_mode=2(VELOCITY_CONTROL)，data[4..7]=input_mode=1(PASS_THROUGH)
  - `0x0D Set_Input_Vel`：data[0..3]=vel float32(转/s)，data[4..7]=torque_ff float32=0
  - `0x07 Set_Axis_State`：8=闭环，1=IDLE
  - `0x18 Clear_Errors`：data[0..3]=1（ODrive 标准要求标志置 1 才清错）
  - `0x03 Get_Error`：主动探测用（data[0]=error_type=0，电机回错误码帧）
- RX：`0x01 Heartbeat`（data[0..3]=axis_error u32 LE，data[4]=axis_state，data[5]=flags，data[6]=temp，data[7]=life）；
  `0x09 Get_Encoder_Estimates`（data[0..3]=pos float32 转，data[4..7]=vel float32 转/s）；
  `0x03` 回包 = 错误码 u32 LE。
- ODrive 目标速度为锁存式，无需高频持续下发；100~500ms 周期重发同速度为安全 no-op。

## 改动清单（4 个文件）

### 1. 新建 `service/srv_tz_temp_test.h`

镜像 `srv_tongzhi_torque_test.h` 结构，导出：

```c
void srv_tz_temp_test_init(void);
void srv_tz_temp_test_start(void);
void srv_tz_temp_test_stop(void);
void srv_tz_temp_test_step(void);
bool srv_tz_temp_test_on_rx(const drv_can_msg_t* msg);
```

头文件注释说明：FDCAN1、速度模式 24H 耐久、由 `SRV_MOTOR_TEST_SELECT = SRV_MOTOR_TEST_TZ_TEMP` 激活、can_task 把 CH_1 帧直连 `srv_tz_temp_test_on_rx`、srv_can 停用。

### 2. 新建 `service/srv_tz_temp_test.c`

整体架构克隆 srv_tongzhi_torque_test.c，去掉位置往复/零点锁存，改为速度阶段 FSM + 主机侧斜坡。

**模块级静态数组（ISR 写/主循环读，均按 `SRV_TZ_TEMP_MAX_MOTORS 8U`）：**
`s_motor_ids[]`、`s_motor_cnt`、`s_scan_log_cnt`、`s_motor_axis_state[]`、`s_axis_state_prev[]`、
`s_motor_err[]`、`s_err_pending[]`、`s_motor_encoder_turns[]`、`s_motor_encoder_vel_tps[]`、`s_motor_have_encoder[]`、
`s_motor_last_seen_ms[]`、`s_motor_nresp_latch[]`、`s_online_evt_pending[]`、`s_last_init_ms[]`、
`s_assumed_closed[]`、`s_init_step[]`。

**FSM/计时静态变量：** `s_running`、`s_stopping`、`s_stop_start_ms`、`s_phase`、`s_phase_start_ms`、
`s_ctrl_start_ms`、`s_dir`、`s_cycle`、`s_online_ms`、`s_online_last_ms`、`s_start_ms`、
`s_last_cmd_ms`、`s_ramp_target_tps`、`s_ramp_from_tps`、`s_ramp_start_ms`、`s_last_enable_retry_ms`、
`s_enable_stall_since_ms`、`s_enable_warned`、`s_last_status_log_ms`、`s_last_no_motor_log_ms`、
`s_probe_idx`、`s_err_query_idx`、`s_last_probe_ms`、`s_probe_done`、`s_fallback_active`。

**阶段枚举：**
```c
typedef enum { SRV_TZ_TEMP_PHASE_FORWARD=0, SRV_TZ_TEMP_PHASE_DWELL_F, SRV_TZ_TEMP_PHASE_REVERSE, SRV_TZ_TEMP_PHASE_DWELL_R, SRV_TZ_TEMP_PHASE_NUM } srv_tz_temp_test_phase_t;
```

**关键宏：**
- `SRV_TZ_TEMP_SPEED_TPS 20.0f`、`SRV_TZ_TEMP_RAMP_TIME_MS 2000U`、`SRV_TZ_TEMP_CMD_PERIOD_MS 100U`
- `SRV_TZ_TEMP_PHASE_MS 30000U`、`SRV_TZ_TEMP_DURATION_MS 86400000U`、`SRV_TZ_TEMP_DURATION_ENABLE 1`（可临时置 0 或改小验证）
- 发现/保持：`SRV_TZ_TEMP_NORESP_PERIOD_MS 2000U`、`SRV_TZ_TEMP_ENABLE_RETRY_MS 200U`、`SRV_TZ_TEMP_ENABLE_TIMEOUT_MS 2000U`、
  `SRV_TZ_TEMP_PROBE_INTERVAL_MS 200U`、`SRV_TZ_TEMP_FALLBACK_MS 2000U`、`SRV_TZ_TEMP_FALLBACK_NODE 0U`、`SRV_TZ_TEMP_INIT_GRACE_MS 1000U`
- 日志：`SRV_TZ_TEMP_STATUS_LOG_MS 5000U`、`SRV_TZ_TEMP_NO_MOTOR_LOG_MS 5000U`、`SRV_TZ_TEMP_LOG_ENABLE 1`、
  `SRV_TZ_TEMP_AUTO_START 1`
- 指令常量：`SRV_TZ_TEMP_CMD_*`（0x01/0x03/0x07/0x09/0x0B/0x0D/0x18）、`SRV_TZ_TEMP_CONTROL_MODE_VELOCITY 2U`、
  `SRV_TZ_TEMP_INPUT_MODE_PASS_THROUGH 1U`、`SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP 8U`、`SRV_TZ_TEMP_AXIS_STATE_IDLE 1U`
- 错误码描述表 `s_err_map[]` 照抄 srv_tongzhi_torque_test.c 的 10 项。

**init 序列（逐帧，每 `ENABLE_RETRY_MS` 发 1 帧，规避 FDCAN TX FIFO 深度 3）：**
step 0=Clear_Errors → step 1=Set_Controller_Mode(2/1) → step 2=Set_Axis_State(8)。
完成标志 `s_init_step[idx] >= 3U`。init_step 值域 0..2（发完未闭环则从 0 重发）。

**send 函数（全部判 `drv_can_tx_ready(DRV_CAN_CH_1)`）：**
`send_clear_errors`、`send_controller_mode`、`send_axis_state`、`send_input_vel(node, float tps)`、
`send_get_error`；小端 pack/unpack 辅助照抄参考文件。

**step() 主流程（每 5ms 由 can_task 调用）：**
1. `if (!s_running) return;`
2. **停止序列**：`s_stopping` 时把当前速度目标强制为 0（斜坡自然回落），`(now - s_stop_start_ms) >= RAMP_TIME_MS + 1000U` 后向所有电机发 IDLE，`s_running=false`，返回。
3. 在线计时：`all_online()` 则 `s_online_ms += now - s_online_last_ms`；`s_online_last_ms = now`；
   `>= SRV_TZ_TEMP_DURATION_MS` 时打日志 → 进入停止序列（`s_stopping=true`），返回。
4. `probe_step(now)`（发现窗口内轮转 Get_Error 探测 node 0~7；窗口后周期回查已发现电机错误/保活）→ `fallback(now)`（无发现回退 node 0 盲发）。
5. 无电机周期告警；`scan_log_new()`；错误变化打印（沿用 tongzhi 的 s_err_pending 模式）。
6. 闭环确认日志（axis_state 0→8 上升沿一次）；假定闭环宽限（`INIT_GRACE_MS`）。
7. 周期保持：任一电机未闭环/未完成 init → 维护 `s_enable_stall_since_ms`/超时告警，每个 `ENABLE_RETRY_MS`
   对第一台待初始化电机推进 1 帧 init_step。
8. **阶段切换**：`(now - s_phase_start_ms) >= SRV_TZ_TEMP_PHASE_MS` 时推进到下一阶段（FORWARD→DWELL_F→REVERSE→DWELL_R→FORWARD），
   更新 `s_phase_start_ms`、目标速度（`+SPEED/0/-SPEED/0`），`s_cycle` 在 DWELL_R 结束回 FORWARD 时 +1；打阶段日志（含阶段名、目标转/s、循环序号）。
9. **斜坡+发速度**：每 `CMD_PERIOD_MS` 计算当前阶段目标转/s（`s_ramp_target_tps`），目标变化时以当前下发速度为 `s_ramp_from_tps` 重起计时；
   按 `RAMP_TIME_MS` 线性插值得到下发速度，对所有 `target_ready(idx)`（受控 且 已假定闭环或 init_step>=3）电机发 `Set_Input_Vel`。
10. **周期状态诊断日志（5s）**：每受控电机打印 `node/axis/err/vel 毫转/s/enc 毫转/tgt 毫转/s/enc_ok/last_seen/在线时长`；同时打印 `s_online_ms` 与 `s_cycle`。
11. 掉线检测（`NORESP_PERIOD_MS`，latch 一次）；恢复在线事件：撤销假定闭环、重置 init 序列（保持 IDLE 时模式可能丢失，需重走 清错→模式→闭环）、打印。

**on_rx()（ISR，不打日志）**：镜像 tongzhi：标准帧+DLC≥8 才消费；node 越界消费丢弃；未知 node 收录；
刷新 last_seen + 掉线恢复标志；Heartbeat(0x01) 记录 axis_state/err、纠正 assumed_closed；
Get_Error 回包(0x03) 记录 err；Encoder(0x09) 记录 pos/vel float；其余帧刷新 last_seen 后消费。

**stop()：** 置 `s_running` 不变，置 `s_stopping=true`、`s_stop_start_ms=millis()`（走步骤 2 的软停止：先斜坡到 0 再 IDLE）。

**start()/init()：** 镜像 tongzhi：重置全部状态、发现数组、计时，进入发现+FORWARD 阶段。

### 3. 编辑 `service/srv_motor_test_select.h`

- 枚举追加：`SRV_MOTOR_TEST_SEL_TZ_TEMP = 3, /* 良志(ODrive) 速度模式 24H 耐久（srv_tz_temp_test） */`
- 预处理值追加：`#define SRV_MOTOR_TEST_TZ_TEMP 3`
- 便捷宏追加：`#define SRV_MOTOR_TEST_IS_TZ_TEMP (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_TZ_TEMP)`
- 顶部注释块补充第 4 个 CAN1 选项说明。

### 4. 编辑 `tasks/can_task.c`

- 顶部追加 `#include "srv_tz_temp_test.h"`。
- 在 `#elif SRV_MOTOR_TEST_IS_HT_TEMP` 分支后、`#else #error` 前追加：

```c
#elif SRV_MOTOR_TEST_IS_TZ_TEMP
#define CAN1_TEST_INIT srv_tz_temp_test_init
#define CAN1_TEST_STEP srv_tz_temp_test_step
#define CAN1_TEST_ON_RX srv_tz_temp_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
```

- 更新文件头接线注释（新增良志速度模式选项）。

CMake 无需改动（`aux_source_directory(service)` 自动收集新 .c）。

## 激活与构建

```bat
cmake --preset Debug -DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_TZ_TEMP
cmake --build --preset Debug
```
- 若 build/Debug 已存在，必须先重新 configure 让宏生效（cache 持久）；否则 ninja 只重编不改宏。
- 替代方案：临时把 `srv_motor_test_select.h` 默认值改为 `SRV_MOTOR_TEST_TZ_TEMP`，完成后改回。

## 验证

1. `cmake --build --preset Debug` 无告警/错误（新文件编译通过）。
2. 回归：`-DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE` 重建仍通过（select 分支无破坏）。
3. 硬件（推荐先把 `SRV_TZ_TEMP_DURATION_MS` 临时改小或 `SRV_TZ_TEMP_STATUS_LOG_MS` 调短）：
   - 日志出现电机 node 发现（心跳/Get_Error 探测）；无电机时 2s 后回退 node 0 盲发。
   - 每 30s 阶段切换（正转/停/反转/停），速度斜坡约 2s 爬升/回落，周期状态日志反馈 vel/enc/axis/err 正常。
   - 拔掉某电机 2s 后出现掉线告警、恢复后重新走 init。
   - 24h 自动停止：到点后先回落速度 0 → IDLE，日志打印累计在线时长与循环次数（预期 720 循环）。

## 风险与注意

- Set_Input_Vel 为锁存目标：停留阶段持续发 0 并保持闭环（不 IDLE），电机带零速保持。
- 停止序列先软停再 IDLE，避免 20 转/s 下直接 IDLE 造成惯性空转。
- FDCAN TX FIFO 深度 3：init 逐帧下发、发速度时逐台串行发送，避免同 tick 连发被丢。
- 速度回显单位 转/s（0x09 vel 为 float32），状态日志统一 ×1000 打毫转/s。
- 错误电机同样持续收到 init（第一步清错），避免锁存错误阻塞闭环。
