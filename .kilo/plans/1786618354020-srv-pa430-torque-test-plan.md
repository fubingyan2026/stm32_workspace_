# srv_pa430_torque_test 实现计划

## 目标

参考 `service/srv_ht_torque_test.c` 的结构与分层，新建 `srv_pa430_torque_test.c/.h`，
用 **Motorevo 协议**（`docs/Motorevo电机CAN协议文档.md`）驱动 pa430 电机做「来回运动」：

- 控制模式：**MIT 力位混合（Control Mode = 0x2）**
- 传输：**CAN FD 广播帧**（状态帧 `0x10` / 控制帧 `0x20`，DLC 64，每电机槽 `(ID-1)*8`）
- 反向判定：**位置反馈闭环判到**（读反馈帧 θ，`|θ_raw − θ_ref_raw| ≤ 阈值` 后反向）

不依赖 `srv_motor` / `srv_motor_behavior`，与 `srv_ht_torque_test` 一样通过 `on_rx`/`step`/`init` 三个接口接入。

---

## 已锁定的设计决策

| 项 | 决策 |
|---|---|
| 协议 | Motorevo（文档 §1–§4） |
| 控制模式 | MIT（`Control Mode = 0x2`，公式 `T_out = Kp(θ_ref−θ) + Kd(V_ref−V) + T_ref`） |
| 寻址 | 广播帧：状态 `0x10`、控制 `0x20`，DLC 64，CAN FD（`is_fd=true, dlc=64`） |
| 反馈 | 每电机以自身 ID（1~8）回复标准帧 DLC 8，解析 θ/速度/扭矩/温度/错误码 |
| 反向判定 | 位置反馈闭环：读 θ_raw 与目标 θ_ref_raw 比较，≤阈值反向 |
| 编译开关 | 新宏 `SRV_PA430_TORQUE_TEST_ENABLE`（默认 0，启用后接管 CAN 总线，HT 测试保持原样） |

## 协议关键数据（供实现直接引用）

### MIT 广播控制槽（`0x20`，槽偏移 `(ID-1)*8`，8 字节大端封包）

```
byte0 = pos_raw >> 8
byte1 = pos_raw & 0xFF
byte2 = vel_raw >> 4
byte3 = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)
byte4 = kp_raw & 0xFF
byte5 = kd_raw >> 4
byte6 = ((kd_raw & 0x0F) << 4) | ((tq_raw >> 8) & 0x0F)
byte7 = tq_raw & 0xFF
```

默认通讯范围（文档 §8）：θ = ±12.5 rad、V = ±10 rad/s、Kp = 0~250、Kd = 0~50、T = ±50 Nm。

- `pos_raw`（16bit）：θ 映射，+12.5 rad → `0xFFFF`，−12.5 rad → `0x0000`
- `vel_raw`（12bit）：V=0 → `0x800`；本模块恒发 `0x800`（目标速度 0）
- `kp_raw`（12bit）：0 → `0x000`，250 → `0xFFF`
- `kd_raw`（12bit）：0 → `0x000`，50 → `0xFFF`
- `tq_raw`（12bit）：T=0 → `0x800`（前馈扭矩 0）

### 广播使能/失能帧（`0x10`，DLC 64，槽偏移 `(ID-1)*8`）

```
byte0..byte6 = 0xFF（固定）
byte7 = 0xFC 使能 / 0xFD 失能 / 0xFF 无命令（未配置槽）
```

### 反馈帧（标识符 = Motor ID 1~8，标准帧 DLC 8，大端）

```
byte0..1 : θ_raw（16bit，与 pos_raw 同映射）
byte2 + byte3[4:7] : V（12bit）
byte3[0:3] + byte4  : T（12bit）
byte5    : 温度（0x00=−40℃，0xFF=215℃）
byte6..7 : 错误码（16bit）：Bit0 使能、Bit1 母线过压、Bit2 相过流、Bit3 线圈过温、
           Bit4 超速、Bit8 堵转、Bit10 板温过温、Bit11 母线欠压、Bit12 位置超限、Bit13 CAN 超时
```

θ 反馈与 θ_ref 使用同一 16bit 标度 → 反向判定纯整数计算，无浮点（工程禁用 float）。

---

## 文件改动

### 新建 `service/srv_pa430_torque_test.h`

镜像 `srv_ht_torque_test.h`，导出：

```c
void srv_pa430_torque_test_init(void);   /* AUTO_START=1 时自动 start */
void srv_pa430_torque_test_start(void);  /* 使能广播 → 启动往复 */
void srv_pa430_torque_test_stop(void);   /* θ_ref 回中 + 失能广播 */
void srv_pa430_torque_test_step(void);   /* 由 can_task 每 5ms 调用 */
bool srv_pa430_torque_test_on_rx(const drv_can_msg_t* msg);
```

同时在本头文件定义接线开关：

```c
#ifndef SRV_PA430_TORQUE_TEST_ENABLE
#define SRV_PA430_TORQUE_TEST_ENABLE 0   /* 1=启用 pa430 测试并接管 CAN；0=保留 HT 测试 */
#endif
```

### 新建 `service/srv_pa430_torque_test.c`

**可配置常量（带默认值，上电前须按实际确认/整定）**

```c
#define SRV_PA430_AUTO_START         1
#define SRV_PA430_MOTOR_COUNT        1     /* 总线电机数 1~8（确认） */
/* 电机 ID 列表：{1}（默认），可扩展为 {1,2,...} 对应广播槽 (ID-1)*8 */
static const uint8_t SRV_PA430_MOTOR_IDS[] = { 1 };

/* 端点（16bit 原始值，默认用满量程 = CAN COM Theta MAX/MIN） */
#define SRV_PA430_RAW_POS            0xFFFFU  /* +CAN_COM_MAX（+12.5 rad ≈ +716°） */
#define SRV_PA430_RAW_NEG            0x0000U  /* CAN_COM_MIN（−12.5 rad） */
#define SRV_PA430_RAW_MID            0x8000U  /* θ=0，用于停车回中 */
#define SRV_PA430_ARRIVE_THRESHOLD   0x0100U  /* 到达阈值（原始 16bit 单位，≈全量程/256，确认） */

/* MIT 刚度/阻尼（12bit 原始值，须按负载整定） */
#define SRV_PA430_KP_RAW             0x0333U  /* Kp≈50 Nm/rad（确认） */
#define SRV_PA430_KD_RAW             0x0066U  /* Kd≈2  Nm/(rad/s)（确认） */

/* 周期 */
#define SRV_PA430_CTRL_PERIOD_MS     5U    /* 控制帧 0x20 重发周期（对齐 can_task 5ms） */
#define SRV_PA430_ARRIVE_SETTLE_MS   200U  /* 到点后驻留去抖时间（确认是否需要） */
#define SRV_PA430_NORESP_PERIOD_MS   2000U /* 无反馈判掉线阈值 */
#define SRV_PA430_DURATION_MS        2592000000U /* 30 天自动停止；0=禁用 */
```

**运行状态（静态，config-in-context 沿用现有单例风格）**

```c
static bool  s_running;
static uint8_t s_dir;                      /* +1 朝 RAW_POS，−1 朝 RAW_NEG */
static uint16_t s_target_raw;              /* 当前目标 θ_ref_raw */
static uint16_t s_motor_theta_raw[8];      /* 每电机最新 θ（ISR 写） */
static bool s_motor_arrived[8];            /* 到点标志（ISR 写） */
static bool s_arrive_pending[8];           /* 新到点待处理（ISR 写） */
static uint16_t s_motor_err[8];            /* 错误码（ISR 写） */
static bool s_err_pending[8];              /* 错误变化待打印 */
static uint32_t s_motor_last_seen_ms[8];   /* 掉线检测 */
static bool s_nresp_latch[8];              /* 掉线锁存（避免刷屏） */
static uint32_t s_last_ctrl_ms;            /* 控制帧重发计时 */
static uint32_t s_start_ms, s_online_ms, s_online_last_ms; /* 30 天计时 */
```

**关键函数实现要点**

1. `srv_pa430_torque_test_send_enable(bool enable)` — 组 `0x10` DLC 64，`is_fd=true`；
   配置电机槽 `byte7=0xFC/0xFD`，未配置槽 `byte7=0xFF`（`drv_can_tx_ready` 守卫）。
2. `srv_pa430_torque_test_send_control(void)` — 组 `0x20` DLC 64，`is_fd=true`；
   对每个配置电机槽写入 MIT 封包（`pos=s_target_raw, vel=0x800, kp=KP_RAW, kd=KD_RAW, tq=0x800`）；
   未配置槽写中性包（`pos=0x8000, vel=0x800, kp=0, kd=0, tq=0x800`）。
3. `srv_pa430_torque_test_start()` — `s_running=true`；`s_dir=+1; s_target_raw=RAW_POS`；
   记录 `s_start_ms`；`send_enable(true)`；进入 MIT 控制（Control Mode 由宏 `SRV_PA430_CONFIGURE_MODE` 决定是否软件写入，见下）。
4. `srv_pa430_torque_test_step()` — 周期（`CTRL_PERIOD_MS`）重发 `0x20`；
   消费 `s_arrive_pending[]`：所有配置电机 `|θ−target|≤阈值` 后反向（`s_target_raw` 在 `RAW_POS`/`RAW_NEG` 间翻转，`s_dir` 取反）；
   消费 `s_err_pending[]` 打印错误码变化；掉线检测 + 恢复在线重新使能；
   `s_online_ms` 累计满 `DURATION_MS`（若启用）自动 `stop()`。
5. `srv_pa430_torque_test_stop()` — `s_target_raw=RAW_MID`，发一帧控制帧回中 → `send_enable(false)`。
6. `srv_pa430_torque_test_on_rx(msg)` — **ISR 上下文，只记录不打日志**：
   - 过滤：`msg->is_extended==false` 且 `msg->id` ∈ 配置电机 ID（1~8）且 `msg->dlc>=8` → 解析反馈，返回 `true`；否则返回 `false`（交旧 `0x100` 协议）。
   - 更新 `s_motor_theta_raw[i]`、`s_motor_err[i]`、`s_motor_last_seen_ms[i]`；
     判到点置 `s_arrive_pending[i]`；错误变化置 `s_err_pending[i]`；掉线恢复清 `s_nresp_latch`。

**可选：Control Mode 软件配置**（宏 `SRV_PA430_CONFIGURE_MODE`，默认 0=假设电机已配置 MIT）

- 置 1 时，`start()` 阶段对每台电机经 `0x600+ID`（DLC 8，单机帧）写 Index 11 = 2（MIT），
  随后 Index 0 保存（触发 Flash 擦写，须供电稳定 ≥1s，见文档 §5.2 警告）。
- 默认关闭，避免 Flash 擦写风险；上电前人工确认电机 Control Mode=2。

### 修改 `tasks/can_task.c`

```c
#if SRV_PA430_TORQUE_TEST_ENABLE
#include "srv_pa430_torque_test.h"
#define HT_TEST_INIT srv_pa430_torque_test_init
#define HT_TEST_STEP srv_pa430_torque_test_step
#else
/* 现有 HT 选择保持不变 */
#endif
```
（`HT_TEST_INIT`/`HT_TEST_STEP` 命名沿用，仅新增一个优先分支。）

### 修改 `service/srv_can.c`

在 `#if SRV_HT_TEST_MODE_TORQUE` 之前加：

```c
#if SRV_PA430_TORQUE_TEST_ENABLE
#include "srv_pa430_torque_test.h"
#define HT_TEST_ON_RX srv_pa430_torque_test_on_rx
#elif SRV_HT_TEST_MODE_TORQUE
...
#endif
```

## 编译

`aux_source_directory(.../service)` 自动收集新 `.c`，**无需改 CMakeLists.txt**。构建：`cmd /c build.bat`。

---

## 前置条件 / 风险（须上电前确认，多数属硬件配置，不在 .c/.h 代码内）

1. **FDCAN 波特率不匹配（必须处理）**：当前 `Core/Src/fdcan.c` 名义 1.0625 MHz / 数据 5.3125 MHz，
   而 Motorevo 5M 版为 1 MHz / 5 MHz、4M 版为 1 MHz / 3.953 MHz。广播 DLC 64 必须走 CAN FD（BRS）。
   需在 CubeMX 中把 FDCAN1 位时序改为与 pa430 的 Protocol Type（Index 69，0=5M/1=4M/2=CAN）一致，
   否则会产生位错误 / ERROR-PASSIVE / 丢帧。**这是启用 `SRV_PA430_TORQUE_TEST_ENABLE=1` 前必须完成的 CubeMX 改动。**
2. **Control Mode=2**：电机 flash 需已配置为 MIT（或置 `SRV_PA430_CONFIGURE_MODE=1` 由固件写入+保存）。
3. **CAN COM Theta 范围（Index 24/25）**：默认 ±12.5 rad。端点 `RAW_POS/RAW_NEG` 默认取满量程；
   若实际范围不同，须改 `SRV_PA430_RAW_POS/NEG` 或重配参数。
4. **Kp/Kd 整定**：`SRV_PA430_KP_RAW/KD_RAW` 默认值须按实际负载整定，过大可能振荡、过小不到位。
5. **电机数量/ID**：`SRV_PA430_MOTOR_COUNT` 与 `SRV_PA430_MOTOR_IDS[]` 须与实际总线一致（默认 1 台 ID=1）。
6. **广播槽中性化**：未配置槽发中性包（Kp=0），避免误动其它 ID。

## 验证方案

1. `cmd /c build.bat` 通过（无告警/错误）。
2. 上电观察日志：使能广播、首帧 θ_ref=+RAW_POS、反馈帧 θ 单调接近目标。
3. 到点后方向翻转（目标切到 RAW_NEG），θ 反向运动；错误码无异常位。
4. 手动调 `srv_pa430_torque_test_stop()`：θ_ref 回中 + 失能，电机松劲。
5. 断掉电机反馈（拔线/断电）验证掉线告警与恢复在线重新使能。
6. 确认 FDCAN 位时序已改后，无 ERROR-PASSIVE / BUS-OFF 日志。

## 待确认清单（实施前/上电前）

- [ ] 电机实际数量与 ID 列表
- [ ] pa430 的 Protocol Type（5M / 4M）→ 对应 CubeMX FDCAN 位时序
- [ ] 端点角度（是否用满量程 ±12.5 rad，还是更小范围）
- [ ] Kp / Kd 目标值
- [ ] 是否需要 30 天自动停止（`SRV_PA430_DURATION_MS`）
- [ ] Control Mode 是否已预配置为 MIT（否则启用软件写入路径）
