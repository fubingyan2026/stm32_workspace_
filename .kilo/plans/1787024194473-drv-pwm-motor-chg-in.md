# MOTOR 移交：按电压条件延迟关闭预充电（避免 MOTOR_POWER_PGD 故障）

## 问题背景

`pwr_entry_cb` 的 `PWR_STATE_MOTOR` 分支在使能 `MOTOR_EN + HSD` 的同时立即调用
`precharge_reset()`。此时母线仍由预充电维持（稳态 duty=90% ≈ 0.9×vin），主回路刚闭合
尚未把母线抬到接近 VIN，预充电一断母线瞬间跌落 → `DRV_STATUS_MOTOR_PGD`（电机电源
PGOOD）判定异常 → 触发 `fault_policy_critical` 紧急断电。

## 方案（用户拍板）

**按电压条件 + 超时兜底**：MOTOR 态持续检查母线电压，`motor_power_mv ≥ vin×95%`
才执行 `precharge_reset()`（一次）；带超时兜底（200ms，母线抬不上来也强制关闭，防止
死等）；移交完成前不进入 DONE。主回路先把母线抬到接近 VIN，再断预充电，PGD 不跌落。

## 改动清单（`service/srv_pwr_ctrl.c`）

### 1. 常量（`STEADY_TIME_MS` 附近）

```c
#define STEADY_TIME_MS (50U)

/** @brief MOTOR 移交：母线达到 VIN 该比例后关闭预充电半桥 */
#define PWR_MOTOR_HANDOVER_PCT (95U)
/** @brief MOTOR 移交超时兜底 (ms)：母线未抬到阈值也强制关闭预充电 */
#define PWR_MOTOR_HANDOVER_TIMEOUT_MS (200U)
```

### 2. `power_ctrl_ctx_t` 增加标志

```c
typedef struct {
    bool power_on_requested;
    uint16_t steady_ms;
    bool precharge_off_done; /**< MOTOR 态是否已完成预充电移交关闭 */
} power_ctrl_ctx_t;
```

### 3. `pwr_entry_cb` 的 `PWR_STATE_MOTOR`：去掉立即 `precharge_reset()`

```c
case PWR_STATE_MOTOR:
    p->precharge_off_done = false;
    drv_power_set(DRV_POWER_RAIL_MOTOR_EN, true);
    drv_power_set(DRV_POWER_RAIL_HSD1_12V, true);
    drv_power_set(DRV_POWER_RAIL_HSD1_24V, true);
    drv_power_set(DRV_POWER_RAIL_HSD2_24V, true);
    /* 预充电保持导通，等母线抬到接近 VIN 后再关闭（见 pwr_state_motor） */
    break;
```

### 4. `pwr_state_motor`：移交条件 + 超时兜底，完成后才 DONE

```c
static fsm_state_t pwr_state_motor(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);

    /* 移交：主回路就绪（母线接近 VIN）后再关闭预充电半桥，避免 MOTOR_POWER_PGD 跌落 */
    if (!p->precharge_off_done) {
        const bool handover_ok = (s_precharge.last_vin_mv > 0
            && s_precharge.last_motor_bus_mv
                >= s_precharge.last_vin_mv * PWR_MOTOR_HANDOVER_PCT / 100U);

        if (handover_ok || p->steady_ms >= PWR_MOTOR_HANDOVER_TIMEOUT_MS) {
            SRV_PWR_CTRL_LOG_I("MOTOR 移交: 关闭预充电 (bus=%umV, vin=%umV)",
                (unsigned)s_precharge.last_motor_bus_mv, (unsigned)s_precharge.last_vin_mv);
            precharge_reset();
            p->precharge_off_done = true;
        }
    }

    /* 移交完成且母线稳定后进入 DONE */
    return (p->precharge_off_done && p->steady_ms >= STEADY_TIME_MS)
        ? PWR_STATE_DONE : PWR_STATE_MOTOR;
}
```

> 说明：
> - `s_precharge.last_motor_bus_mv / last_vin_mv` 由 `precharge_step → precharge_sample_voltages`
>   每 1ms 刷新，MOTOR handler 直接读取即可，无需再查 srv_adc。
> - 移交期间预充电 FSM 处于 STEADY，其 OCP 全局检测不覆盖 STEADY；即使移交瞬间硬件
>   OCP 锁存，`precharge_reset()`（EN 拉低）也会在进入 DONE 前清除锁存，不会遗留
>   到 DONE 后被 app_fault_policy 误判。

### 5. 不改动项

- `precharge_reset()`、预充电 FSM、`app_fault_policy` 不变。
- 急停路径 `emergency_off → precharge_reset()` 保持立即关断（安全语义不变）。

## 验证

- `cmd /c build.bat`（Release）无新增错误/告警。
- 逻辑走查：
  - 冷机上电：PRECHARGE 完成 → MOTOR（MOTOR_EN+HSD 开，预充电保持 90% 导通）→ 母线被
    主回路抬到 ≥95%vin → `precharge_reset()` → 稳定后 DONE；`MOTOR_POWER_PGD` 全程不跌落。
  - 母线抬不上来：200ms 超时兜底强制 `precharge_reset()` → DONE（PGD 是否故障交由
    app_fault_policy 判定）。
  - 再次 request_on：MOTOR entry 重新 `precharge_off_done=false`，移交逻辑可重复。

## 备注

- 阈值 95% 与既有 `PWR_PRECHARGE_EARLY_STEADY_PCT` 一致；如需“很接近 VIN”可调高
  （如 98%），超时 200ms 可按母线建立时间调整。
- 若移交窗口内出现 MOTOR_CHG_OCP 误报（硬件锁存但 FSM 不查 STEADY），可在进入 DONE
  前由 `precharge_reset()` 的 EN 拉低统一清除；当前设计已覆盖。
