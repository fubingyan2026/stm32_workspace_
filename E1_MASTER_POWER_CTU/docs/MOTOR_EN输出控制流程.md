# MOTOR_EN 输出控制流程（E1_MASTER_POWER_CTU）

- **适用工程**：E1_MASTER_POWER_CTU（STM32F103RC 主电源板）
- **更新**：2026-09-21
- **说明**：描述 `MOTOR_POWER_EN`（PC11）从“上层策略判断”到“实际拉高/拉低”的完整程序流程，
  并包含同样会关断 MOTOR 的**输入过压保护**（独立锁存）。
  全文用中文名称描述功能，括号内为对应代码符号/接口，便于对照源码。

---

## 1. 角色分工（先看这个）

| 模块 | 名称 | 职责 |
|------|------|------|
| 应用层 | 故障保护策略（`app_fault_policy_step`） | 每 10ms 判断：急停是否按下、MOTOR 是否故障、是否允许使能；调用“设置 MOTOR”接口 |
| 服务层 | 电源控制服务（`srv_pwr_ctrl`） | 直接驱动 MOTOR_EN 引脚；对“使能请求”做延后处理（必须已上电成功） |
| 驱动层 | 电源轨驱动（`drv_power`） | 把 `DRV_POWER_RAIL_MOTOR` 写到 PC11 引脚 |
| 状态来源 | 电源状态监测（`srv_pwr_det_read`）+ 电源控制状态（`srv_pwr_ctrl_get_state`） | 提供：有效急停、MOTOR_PGD、上电完成、各轨使能标志 |

关键约束：
- **急停/故障只关 MOTOR**，VIN_DC-DC / 24V / AUX 三路不受急停影响；
- MOTOR **必须在上电流程成功（POWERED）后**才允许使能；未上电时的使能请求只记录、不动作；
- MOTOR 故障判据带 **使能稳定窗口 150ms + 掉电去抖 50ms**，避免使能瞬间误关断；
- **输入过压保护（独立于 MOTOR 锁存）同样会关断 MOTOR**：已上电后 VIN ≥60V 持续 5s → 关断全部输出（含 MOTOR）并锁存，输入回落 ≤58V 后自动清除并重新上电；过压期间**急停释放无效**（只能靠输入恢复）。
- 低压（VIN <36V）在已上电后**不关断**输出，仅监测。

---

## 2. 程序流程图

> 已导出静态矢量图：[`MOTOR_EN输出控制流程.svg`](./MOTOR_EN输出控制流程.svg)（与本文件同目录，可直接插入文档/PPT）。

```mermaid
flowchart TD
    subgraph APP["应用层：故障保护策略（每 10ms 执行一次）"]
        START["开始：读取系统状态<br/>读急停/各轨 PGD（srv_pwr_det_read）<br/>读电源控制状态（srv_pwr_ctrl_get_state）"]
        CHK_LATCH{"已锁存且急停仍按下？"}
        ENSURE_OFF1["确保 MOTOR 关闭<br/>设置 MOTOR = 关"]
        RET1["本周期结束<br/>（保持关闭，等待急停释放）"]
        CHK_TRIP{"急停有效，或<br/>（已上电成功且 MOTOR 已故障）？"}
        DO_TRIP["置故障锁存<br/>设置 MOTOR = 关<br/>风扇强制满速并记录触发原因"]
        CHK_RELEASE{"检测到急停释放沿？<br/>（有效急停 1 变 0）"}
        DO_RESET["解除故障锁存"]
        CHK_WANT{"允许使能条件：<br/>未锁存 且 已上电成功 且 未急停"}
        DO_ON["请求使能 MOTOR<br/>设置 MOTOR = 开"]
        RET2["本周期结束"]

        START --> CHK_LATCH
        CHK_LATCH -- 是 --> ENSURE_OFF1 --> RET1
        CHK_LATCH -- 否 --> CHK_TRIP
        CHK_TRIP -- 是 --> DO_TRIP --> CHK_RELEASE
        CHK_TRIP -- 否 --> CHK_RELEASE
        CHK_RELEASE -- 是 --> DO_RESET --> CHK_WANT
        CHK_RELEASE -- 否 --> CHK_WANT
        CHK_WANT -- 是 --> DO_ON --> RET2
        CHK_WANT -- 否 --> RET2
    end

    subgraph CRIT["MOTOR 故障判据（fault_policy_critical）"]
        C1["前提：MOTOR 当前已使能"]
        C2["使能后先等待 150ms 稳定窗口"]
        C3{"稳定窗口结束后<br/>MOTOR_PGD 为低且持续 ≥50ms？"}
        C4["判为 MOTOR 故障（进入上面 DO_TRIP）"]
        C5["判为正常"]
        C1 --> C2 --> C3
        C3 -- 是 --> C4
        C3 -- 否 --> C5
    end
    CHK_TRIP -. 需要故障判据 .-> CRIT

    subgraph OVP["输入过压保护（独立锁存，服务层电源 FSM）"]
        OV1["上电成功（POWERED）后每 1ms 采样 VIN"]
        OV2{"VIN ≥60V 累计满 5s？"}
        OV3["关断全部输出（含 MOTOR）<br/>置过压锁存 ov_latched<br/>回到待机（IDLE）"]
        OV4["待机中：VIN 回落 ≤58V？"]
        OV5["清除过压锁存<br/>重新执行上电流程"]
        OV6["保持全关<br/>（等待输入恢复）"]
        OV1 --> OV2
        OV2 -- 是 --> OV3 --> OV4
        OV2 -- 否 --> OV1
        OV4 -- 是 --> OV5
        OV4 -- 否 --> OV6 --> OV4
    end

    subgraph SVC["服务层：电源控制服务（MOTOR 唯一直接操作者）"]
        MS_ON["“设置 MOTOR = 开”接口"]
        MS_DESIRE["记录使能请求 motor_desired = 真"]
        MS_CHK{"当前是否已上电成功？"}
        MS_EN["立即使能：<br/>motor_on = 真<br/>驱动 MOTOR_EN = 高"]
        MS_DEFER["延后：仅保留请求<br/>待上电成功后自动开启"]
        MS_OFF["“设置 MOTOR = 关”接口<br/>清除请求并立即关闭<br/>驱动 MOTOR_EN = 低"]
        MS_PWR["上电成功状态（POWERED）每 1ms 步进"]
        MS_PWRCHK{"存在使能请求且尚未使能？"}
        MS_PWREN["自动使能：驱动 MOTOR_EN = 高"]

        MS_ON --> MS_DESIRE --> MS_CHK
        MS_CHK -- 是 --> MS_EN
        MS_CHK -- 否 --> MS_DEFER
        MS_PWR --> MS_PWRCHK
        MS_PWRCHK -- 是 --> MS_PWREN
        MS_PWRCHK -- 否 --> MS_PWR
    end

    PIN["实际输出：MOTOR_POWER_EN（PC11）<br/>高 = 电机电源使能，低 = 关断"]

    ENSURE_OFF1 -. 调“设置 MOTOR = 关” .-> MS_OFF
    DO_TRIP -. 调“设置 MOTOR = 关” .-> MS_OFF
    DO_ON -. 调“设置 MOTOR = 开” .-> MS_ON
    MS_EN --> PIN
    MS_PWREN --> PIN
    MS_OFF --> PIN
    OV3 --> PIN
    OV3 -. 上电状态=false，使能门控不通过 .-> CHK_WANT
```

---

## 3. 流程文字说明（按执行顺序）

1. **周期开始**：读取系统状态（有效急停 `estop_on`、MOTOR 电源正常 `motor_power_ok`）与电源控制状态快照（是否已上电成功 `powered_on`、MOTOR 是否已使能 `motor_en`）。
2. **已锁存且急停仍按下**：保持 MOTOR 关闭（若有异常打开也强制关闭），本周期直接结束。
3. **触发判断**（满足任一即触发）：
   - 有效急停 `estop_on` 为真；
   - 或：已上电成功且 MOTOR 故障判据成立（见第 4 点）。
   触发后：置锁存、**立即关闭 MOTOR**、风扇满速、记录原因。
4. **MOTOR 故障判据**：仅当 MOTOR 已使能时生效；使能后先给 150ms 稳定窗口，窗口之后 MOTOR_PGD 持续为低 ≥50ms 才判为故障（防使能瞬间软启动未就绪误判）。
5. **急停释放沿**：检测到“有效急停 1→0”时解除锁存。
6. **允许使能判断**：`未锁存 且 已上电成功 且 未急停` 三者同时满足 → 请求使能 MOTOR；否则保持原状。
7. **服务层使能处理**：
   - 已上电成功 → 立即拉高 MOTOR_EN；
   - 未上电成功 → 仅记录请求（延后），由“上电成功状态”每 1ms 检查并自动补使能；
   - 关闭请求 → 任何时候立即拉低，并清除延后请求。
8. **最终输出**：`MOTOR_POWER_EN`（PC11），高=电机电源使能，低=关断。
9. **输入过压保护（独立于上述锁存，但同样影响 MOTOR_EN）**：
   - 已上电成功状态下持续监测 VIN：≥60V 累计满 5s → **关断全部输出（含 MOTOR）**、置过压锁存、回待机；
   - 待机中 VIN 回落 ≤58V → 自动清除锁存并重新执行上电流程；
   - 过压锁存期间 `powered_on=false`，第 6 步“允许使能判断”不会通过，因此**急停释放也无法让 MOTOR 恢复**，只能等输入电压回到允许范围。
   - 已上电后 VIN <36V（低压）不关断输出，仅监测。

---

## 4. 触发表（便于查代码）

| 条件 | 代码位置 | 动作 |
|------|----------|------|
| 已锁存 且 急停按下 | `app_fault_policy_step` 起始分支 | 确保 MOTOR 关闭 |
| 有效急停 | `st.estop_on` | 锁存 + 关 MOTOR + 风扇满速 |
| MOTOR 使能后 PGD 丢失（150ms 稳定 + 50ms 去抖） | `fault_policy_critical` | 锁存 + 关 MOTOR |
| 急停释放沿 | `utils_edge_detect(...) == UTILS_EDGE_FALLING` | 解除锁存 |
| 未锁存 && 已上电 && 未急停 | `motor_want` | 请求使能 MOTOR |
| 未上电时的使能请求 | `srv_pwr_ctrl_motor_set(true)` | 延后记录，POWERED 后自动开启 |
| 任何关闭请求 | `srv_pwr_ctrl_motor_set(false)` | 立即关闭并清除延后请求 |
| **已上电后 VIN ≥60V 持续 5s（过压保护）** | `pwr_state_powered` → `pwr_rails_all_off()` + `ov_latched` | 关断全部输出（含 MOTOR）回待机；VIN ≤58V 自动清锁存重新上电 |
| 已上电后 VIN <36V | 无动作（仅监测） | 不关断输出 |
