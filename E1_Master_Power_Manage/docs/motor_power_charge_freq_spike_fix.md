# 电机预充电「频率切换瞬间 10A 电流尖峰」问题修复记录

- **现象等级**：严重（浪涌电流 10A，可能损伤半桥/母线）
- **涉及模块**：`device_drivers/drv_pwm.c`（`drv_pwm_set_frequency`）
- **触发条件**：预充电阶段三（RAMP_FREQ 变频 50k→600k）期间每次变频瞬间
- **修复方式**：PWM 不停机在线改频（去除 `HAL_TIM_PWM_Stop/Init/Start` 重启序列）

---

## 1. 问题背景

电机电源预充电软启动（`docs/motor_power_charge_step.md`）采用 Buck 半桥拓扑，分四阶段
平滑建立后级母线电压：

```
阶段一 EN_CLEAR(清 OCP) → 阶段二 RAMP_TON(脉宽爬升) → 阶段三 RAMP_FREQ(变频 50k→600k)
→ 阶段四 RAMP_DUTY(占空比主升) → 稳态
```

阶段三（`srv_pwr_ctrl.c` `precharge_state_ramp_freq`）在 **25ms 内每 1ms 变频一次**
（步进约 22kHz/ms），阶段四入口再设一次 600kHz。台架实测发现：**每次频率切换的瞬间，
预充电电流冲高至约 10A**，远超软启动预期的限流目标，存在损伤功率器件的风险。

## 2. 根因分析

### 2.1 直接原因：PWM 停机重启毛刺

`drv_pwm_set_frequency()` 原实现每次变频都执行：

```c
HAL_TIM_PWM_Stop(htim, channel);      /* ① 停 PWM：CCxE 清零，TIM3_CH3 输出释放/悬空 */
htim->Init.Prescaler = psc;
htim->Init.Period    = arr;
HAL_TIM_PWM_Init(htim);               /* ② 重初始化定时器基 */
__HAL_TIM_SET_COMPARE(htim, channel, compare);
HAL_TIM_PWM_Start(htim, channel);     /* ③ 重新启动 PWM */
```

阶段三变频 25 次 + 阶段四入口 1 次，**共 26 次 Stop→Init→Start**。每次停机间隙：

1. `CCxE` 被清零，TIM3_CH3 输出不再由定时器驱动（输出释放/悬空）；
2. 半桥驱动输入若被悬空误判为高电平，**高侧 FET 在停机间隙持续导通**；
3. 电感电流按 `I = V_in / L × Δt` 快速上升，停机间隙结束时已累积出 10A 级尖峰；
4. PWM 重启后恢复，但间隙内电流冲击已发生。

斜坡本身（恒 Ton≈167ns，单脉冲峰值电流很小）不会产生 10A 电流，尖峰完全来自
**PWM 停摆重启的毛刺窗口**。

### 2.2 为什么阶段三最明显

阶段三占空比已从 8‰ 爬升至 100‰（非零占空比运行），每次变频都有真实电流流过，
停机间隙的误导通造成的冲击最显著；阶段二占空比接近 0、阶段四频率恒定，故现象集中在
阶段三。

## 3. 解决方案

### 3.1 思路

**PWM 全程不停机**：PSC 恒 0 时频率只由 ARR 决定，变频仅需写 ARR（并按新 ARR 重算
CCR 保持占空比），再触发一次软件更新事件（`TIM_EGR_UG`）让新周期从 CNT=0 干净开始。
PWM 输出连续，无停机/悬空窗口，电流尖峰从根源消除。

### 3.2 改动前（`drv_pwm_set_frequency` 重配置段）

```c
HAL_TIM_PWM_Stop(ctx->route->htim, ctx->route->channel);
ctx->route->htim->Init.Prescaler = psc;
ctx->route->htim->Init.Period = arr;
HAL_TIM_PWM_Init(ctx->route->htim);

uint32_t compare = (uint32_t)ctx->duty_permille * (arr + 1U) / 1000U;
__HAL_TIM_SET_COMPARE(ctx->route->htim, ctx->route->channel, compare);
HAL_TIM_PWM_Start(ctx->route->htim, ctx->route->channel);
```

### 3.3 改动后（在线改频，不停机）

```c
/* 在线改频（不停机）：PSC 恒 0，仅更新 ARR，触发更新事件从新周期起点装载，
 * 避免 HAL Stop/Init/Start 重启毛刺（频率切换瞬间电流尖峰） */
ctx->route->htim->Init.Prescaler = psc;
ctx->route->htim->Init.Period = arr;

__HAL_TIM_SET_AUTORELOAD(ctx->route->htim, arr);     /* 写 ARR（PSC=0 场景即时生效） */
__HAL_TIM_SET_COUNTER(ctx->route->htim, 0);          /* 计数清零，防止新旧周期拼接超长脉冲 */
ctx->route->htim->Instance->EGR = TIM_EGR_UG;        /* 更新事件：重装 PSC/ARR 影子、清 UIF */

uint32_t compare = (uint32_t)ctx->duty_permille * (arr + 1U) / 1000U;
__HAL_TIM_SET_COMPARE(ctx->route->htim, ctx->route->channel, compare);
```

### 3.4 关键点说明

| 项 | 说明 |
| :--- | :--- |
| `__HAL_TIM_SET_AUTORELOAD` | PSC 恒 0（ARR 无影子、即时生效），变频即改 ARR |
| `__HAL_TIM_SET_COUNTER(0)` | 计数清零，避免新旧 ARR 拼接产生超长/异常脉冲 |
| `EGR = TIM_EGR_UG` | 软件更新事件：重装 PSC/ARR、清 UIF，新周期从 CNT=0 干净开始 |
| `__HAL_TIM_SET_COMPARE` | 按新 ARR 与已存 `duty_permille` 重算 CCR，占空比保持不变 |
| 保留 `htim->Init` 字段更新 | 与 `drv_pwm_get_frequency`（读寄存器）一致，信息不丢失 |

### 3.5 其他路径核查

- `HAL_TIM_PWM_Start` 仅保留在 `drv_pwm_init`（启动 PWM）；`HAL_TIM_PWM_Stop` 仅保留在
  `drv_pwm_deinit_all`（反初始化）。正常运行路径不再触碰 Stop/Start。
- `drv_pwm_set_duty` 不受影响；预充电状态机（srv_pwr_ctrl.c）无改动。

## 4. 验证

- **编译**：`cmd /c build.bat`（Release）通过，无新增错误/告警。
- **台架（建议）**：示波器观察 `MOTOR_CHG_IN`（TIM3_CH3）——
  - 变频瞬间波形不再出现停摆/毛刺，频率平滑切换；
  - 预充电电流峰值回落至软启动预期范围，10A 尖峰消失。
- **功能回归**：`drv_pwm_set_frequency(600kHz)` 后 `drv_pwm_get_frequency` 回读 600kHz；
  `set_duty` 按新 ARR 正常换算。

## 5. 相关文件

- `device_drivers/drv_pwm.c` — 本次修复（`drv_pwm_set_frequency`）
- `service/srv_pwr_ctrl.c` — 预充电双 FSM（阶段三变频驱动方，未改动）
- `docs/motor_power_charge_step.md` — 预充电软启动设计文档

## 6. 备注

- 若台架后续仍希望进一步平滑变频，可把阶段三频率步长加密（当前约 22kHz/ms）或对频率
  做缓变处理，但本次修复后已无停机毛刺，通常不再需要。
- `TIM_EGR_UG` / `__HAL_TIM_SET_AUTORELOAD` / `__HAL_TIM_SET_COUNTER` 均为 STM32F4 HAL
  标准宏/寄存器，无需额外依赖。
