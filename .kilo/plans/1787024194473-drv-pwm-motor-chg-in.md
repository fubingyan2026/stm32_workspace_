# 预充电状态机增加【阶段0：预偏置检测】

## 目标

在 `srv_pwr_ctrl.c` 的预充电 FSM 中新增「阶段0：预偏置检测」，根据
`ratio = motor_power_mv / vin_mv` 自适应选择：
- **冷机**（ratio < 10%）：走全流程（阶段二 → 三 → 四）。
- **暖机**（10% ≤ ratio < 95%）：跳过阶段二/三，直接切入阶段四 RAMP_DUTY，
  起始 duty = ratio（钳位 100~900‰），并按**同斜率折算虚拟已走时间**继续爬升到 90%。
- **近满**（ratio ≥ 95%）：直接进入 STEADY（done），无需再预充电。

> 注：用户所称 `Precharge_1ms_Process` 即 `precharge_step()`（srv_pwr_ctrl.c:389）。
> 预偏置逻辑以新增 FSM 状态 `PREBIAS` + handler 承载，`precharge_step()` 本身不改
> （它已通过 `fsm_step()` 派发新状态）。

## 决策（用户拍板）

1. 冷机 = ratio < 10%（100‰）；暖机切入阶段四（duty=ratio，钳位 100~900‰）；≥95% → STEADY。
2. 暖机切入后按同斜率折算剩余时间（虚拟 phase_elapsed 预置）。

## 状态流转图（更新后）

```
IDLE --start--> EN_CLEAR --5ms--> PREBIAS(阶段0) ─┬─ ratio<10%(冷机) ──> RAMP_TON(阶段二)
        │                                        ├─ 10%≤r<95%(暖机) ─> RAMP_DUTY(阶段四, 切入 duty=r)
        │                                        └─ r≥95%(近满) ─────> STEADY(稳态)
        │
        └─ 任意活动态 OCP ──> OCP_RECOVER(10ms) ─┬─ 重试<3 ──> EN_CLEAR
                                                └─ ≥3 ──────> FAULT

RAMP_TON(阶段二 0→8‰,50kHz) ─> RAMP_FREQ(阶段三 8→100‰,50k→600k) ─> RAMP_DUTY(阶段四 100→900‰,600kHz) ─> STEADY(900‰)
RAMP_DUTY(暖机切入点) ─> STEADY
```

## 改动清单（`service/srv_pwr_ctrl.c`，按序）

### 1. 状态枚举（L92-103）插入 `PREBIAS`

```c
PRECHARGE_STATE_EN_CLEAR, /**< 阶段一：EN 保持低 5ms 清除 OCP 锁存 */
PRECHARGE_STATE_PREBIAS,  /**< 阶段0：预偏置检测（EN 低采样，决定冷/暖/近满） */
PRECHARGE_STATE_RAMP_TON, /**< 阶段二：50kHz 恒频，脉宽爬升 */
```
（后续枚举自动前移；本文件全部用枚举名 + designated initializer，重编号安全。）

### 2. `s_precharge_state_names[]` 增加一行

```c
[PRECHARGE_STATE_PREBIAS] = "PREBIAS",
```

### 3. 新增常量（`PWR_PRECHARGE_NO_LOAD_MV` 附近）

```c
/** @brief 冷机判定：预偏置比 < 10%（100‰）→ 走全流程 */
#define PWR_PRECHARGE_COLD_RATIO_PERMILLE (100U)
```

### 4. `precharge_ctx_t` 增加字段

```c
uint32_t ramp_seed; /**< 暖机切入阶段四的虚拟已走时间 (ms)，冷机为 0 */
```

### 5. `precharge_reset()` 增加一行（清 seed）

```c
s_precharge.ramp_seed = 0;
```

### 6. `precharge_state_en_clear()` 出口改为 `PREBIAS`

```c
if (c->phase_elapsed_ms >= PWR_PRECHARGE_EN_CLEAR_MS) {
    return PRECHARGE_STATE_PREBIAS;
}
```

### 7. 新增 `precharge_state_prebias()` handler（核心代码）

```c
/**
 * @brief 阶段0：预偏置检测——按 motor_power_mv/vin 比值决定冷机/暖机/近满
 */
static fsm_state_t precharge_state_prebias(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    const uint32_t vin = c->last_vin_mv;
    if (vin == 0U) {
        /* 母线电压无效：按冷机走全流程（安全兜底） */
        SRV_PWR_CTRL_LOG_W("预偏置检测: VIN 无效 → 冷机全流程");
        return PRECHARGE_STATE_RAMP_TON;
    }

    const uint32_t ratio_permille = c->last_motor_bus_mv * 1000U / vin;

    if (ratio_permille < PWR_PRECHARGE_COLD_RATIO_PERMILLE) {
        SRV_PWR_CTRL_LOG_I("预偏置检测: 冷机 (ratio=%u‰<10%%) → 全流程",
            (unsigned)ratio_permille);
        return PRECHARGE_STATE_RAMP_TON;
    }

    if (ratio_permille >= PWR_PRECHARGE_EARLY_STEADY_PCT * 10U) {
        SRV_PWR_CTRL_LOG_I("预偏置检测: 近满 (ratio=%u‰≥95%%) → 直接稳态",
            (unsigned)ratio_permille);
        return PRECHARGE_STATE_STEADY;
    }

    /* 暖机：切入阶段四，起始 duty = ratio（钳位 100~900‰），按同斜率折算虚拟已走时间 */
    uint32_t duty = ratio_permille;
    if (duty < PWR_PRECHARGE_DUTY_P3_END) {
        duty = PWR_PRECHARGE_DUTY_P3_END;
    } else if (duty > PWR_PRECHARGE_DUTY_P4_END) {
        duty = PWR_PRECHARGE_DUTY_P4_END;
    }
    /* 冷机斜坡 duty(t) = 100 + 800*t/450 → t = (duty-100)*450/800 */
    c->ramp_seed = (duty - PWR_PRECHARGE_DUTY_P3_END) * PWR_PRECHARGE_PHASE4_MS
        / (PWR_PRECHARGE_DUTY_P4_END - PWR_PRECHARGE_DUTY_P3_END);
    SRV_PWR_CTRL_LOG_I("预偏置检测: 暖机 (ratio=%u‰) → 切入阶段四 duty=%u‰",
        (unsigned)ratio_permille, (unsigned)duty);
    return PRECHARGE_STATE_RAMP_DUTY;
}
```

### 8. 注册 handler + 声明原型

- 原型区（L179 附近）：`static fsm_state_t precharge_state_prebias(fsm_t* fsm);`
- `precharge_init()`：`s_precharge_handlers[PRECHARGE_STATE_PREBIAS] = precharge_state_prebias;`

### 9. `precharge_entry_cb()` 两处

- 新增 `PREBIAS` case（与 EN_CLEAR 相同：EN 低 + PWM 0）：
  ```c
  case PRECHARGE_STATE_PREBIAS:
      /* 阶段0：保持 EN 低 + PWM 0，仅采样，不驱动输出 */
      drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
      drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
      break;
  ```
- `RAMP_DUTY` case 补拉高 EN + 预置虚拟进度（暖机路径跳过了 RAMP_TON，那里才拉高 EN）：
  ```c
  case PRECHARGE_STATE_RAMP_DUTY:
      /* 暖机切入可能跳过 RAMP_TON，这里补拉高 EN；按折算虚拟已走时间预置进度 */
      drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, true);
      drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_FREQ_HIGH_HZ);
      c->phase_elapsed_ms = c->ramp_seed; /* 覆盖 entry_cb 顶部的 =0 */
      break;
  ```

### 10. 无需改动项（确认）

- `precharge_step()`：`total_elapsed` 范围 `EN_CLEAR..STEADY` 已含 PREBIAS；OCP 活动态
  列表不含 PREBIAS（EN 低无电流），均无需改。
- `precharge_state_ocp_recover()` 重试仍返回 `EN_CLEAR`（→ PREBIAS 重新检测）。
- 阶段四 `precharge_state_ramp_duty()` 的 NO_LOAD / 95% 提前稳态 / 到点转稳态逻辑不变
  （暖机切入后从 seed 继续，最终仍走「到 90% → STEADY」或「≥95% 提前 STEADY」）。

## 验证

- `cmd /c build.bat`（Release）无新增错误/告警。
- 逻辑走查（纯软件/台架）：
  - 冷机（motor_power_mv≈0）：日志「冷机 → 全流程」，走 EN_CLEAR→PREBIAS→RAMP_TON→RAMP_FREQ→RAMP_DUTY→STEADY，时序与改动前一致。
  - 暖机（如 ratio=40%）：日志「暖机 ratio=400‰ → 切入阶段四 duty=400‰」，直接进 RAMP_DUTY，
    首拍 duty≈400‰、freq 600kHz、EN 高，随后同斜率爬升到 90% 转 STEADY。
  - 近满（ratio≥95%）：日志「近满 → 直接稳态」，进 STEADY（done，EN 保持低），
    电源 FSM 随即转 MOTOR 移交主回路。
  - VIN=0：日志「VIN 无效 → 冷机全流程」。
  - OCP 重试：OCP_RECOVER→EN_CLEAR→PREBIAS 重新检测，行为正确。
- grep `PRECHARGE_STATE_PREBIAS` 覆盖：枚举、state_names、handler 注册、entry_cb、handler 定义。

## 风险 / 备注

- **阶段0 物理位置**：PREBIAS 放在 EN_CLEAR（阶段一，5ms EN 低清 OCP）之后、斜坡之前，
  两者都保持 EN 低——采样时预充电未驱动，读到的 motor_power_mv 是真实残余电压；且清 OCP
  是硬性安全前置。若需严格把「阶段0」置于 EN_CLEAR 之前，需在 context 存 `prebias_decision`
  并在 EN_CLEAR 出口分支（略增复杂度），当前方案更简洁。
- **近满路径 EN 保持低**：ratio≥95% 时直接 done、不启动预充电半桥，交由主回路接管
  （与既有「≥95% 提前转稳态」语义一致）。
- **ratio 整数精度**：`motor_power_mv*1000/vin`（uint32，上限 ~24M 不溢出）；vin 极小
  （噪声）时 ratio 偏大可能误判近满，实际 VIN≈24V 不会出现，必要时可加 vin 下限阈值。
- **状态枚举重编号**：插入 PREBIAS 后 RAMP_TON..FAULT 前移，本文件全部用枚举名引用，
  `total_elapsed`/OCP 列表按名字判断，安全；实现后建议 grep 确认无裸数字下标。
