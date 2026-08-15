# 计划：PA430 恢复后"使能成功但不往复"修复（目标重同步 + 到位阈值放宽）

## 背景与根因

现象（用户已确认"无错误、无翻转日志"）：
```
W (53955) 恢复在线
I (53961) 已确认使能
I (55370) 已确认使能   ← 使能位 1→0→1 反复，但之后不再往复
```

使能链路已修好（上一轮"重试直到确认"生效），但**运动没有恢复**。根因在到位判定
（`E1_Hand_G474/service/srv_pa430_torque_test.c`）：

1. **恢复后运动状态未复位**：断电期间 `step()` 仍用冻结的 `s_motor_theta_raw` 跑翻转逻辑，
   `s_target_raw` 保持旧值、`s_motor_have_fb` 永不清除。恢复后固件用"旧目标+旧反馈状态"
   继续驱动，不确定。
2. **到位阈值偏紧（主因）**：前几轮为降速把 Kp 从 50 降到 12.5 Nm/rad。MIT 下稳态位置
   误差 δ = 负载扭矩/Kp，Kp 越小 δ 越大。当 δ > `SRV_PA430_ARRIVE_THRESHOLD_RAD`（0.1 rad）
   时，电机稳定在目标差一点点处，`is_arrived()` 恒 false → 永不翻转 → "使能成功但不再往复"，
   且**无错误码**（不是保护，只是到位判定差）。"有时候"取决于负载/姿态。

## 决策（已与用户确认）

- 恢复在线时**重同步运动目标**到电机当前反馈位置（钳位到测试端点范围）。
- 到位阈值 `SRV_PA430_ARRIVE_THRESHOLD_RAD` 默认 0.1 → **0.2 rad**（行程 ±2.5 rad 的 8%）。
- 恢复日志附带 θ 与重同步后的目标，便于后续确认。

## 改动（均在 `E1_Hand_G474/service/srv_pa430_torque_test.c`，非共享代码）

### 1. 放宽到位阈值默认值

```c
/** @brief 到位判定阈值（rad） */
#define SRV_PA430_ARRIVE_THRESHOLD_RAD (0.2f)   /* 0.1f → 0.2f */
```
（`s_arrive_thresh_raw` 由 `delta_theta_to_raw` 自动换算为 ≈524 counts，无需其它改动。）

### 2. `step()` 恢复在线块：重同步运动目标

将现有"恢复在线"循环（打印 `电机 ID=%u 恢复在线`）改为同时重同步目标：

```c
/* 恢复在线：把运动目标重同步到电机当前位置，避免旧目标/旧反馈造成不翻转；
   实际使能仍交给下方"保持使能"重试循环 */
for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
    if (s_online_evt_pending[i]) {
        s_online_evt_pending[i] = false;
        uint16_t theta = s_motor_theta_raw[i];
        /* 钳位到测试端点 [NEG, POS]，防止反馈异常值作为目标 */
        if (theta < s_pos_raw_neg) {
            theta = s_pos_raw_neg;
        } else if (theta > s_pos_raw_pos) {
            theta = s_pos_raw_pos;
        }
        s_target_raw = theta;   /* 目标 = 当前位置 → 下一 tick 到位即翻转，从当前位置重启往复 */
        s_dir = 1;
        SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 恢复在线，θ=0x%04X，目标重同步 0x%04X",
            (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i], (unsigned)s_target_raw);
    }
}
```

### 3. 注释同步

- `step()` `@note` 与文件头 `@brief`：补充"恢复在线后重同步 θ_ref 到当前位置再重启往复"。

## 边界 / 失败模式

- **重同步后立即到位**：target=当前位置 → `is_arrived` 为 true → 下一 tick 翻转 → 电机从
  当前位置摆向另一端，往复恢复（约 5~10ms 内）。
- **反馈异常值**：θ 钳位到 `[s_pos_raw_neg, s_pos_raw_pos]`，避免 encoder 未就绪时的
  0x0000/0xFFFF 被当作目标。
- **多电机**：当前 `SRV_PA430_MOTOR_COUNT=1`，`s_target_raw` 全局仅对应单机；多机需
  每机目标（超出本次范围，注释中注明）。
- **阈值放宽副作用**：0.2 rad 下可能在距端点约 8% 处提前翻转，摆幅略小，属可接受。

## 验证计划

1. 构建：E1_Hand_G474 根目录 `cmd /c build.bat`。
2. 断电重上电复现：观察恢复日志应含 `θ=0xXXXX，目标重同步 0xXXXX`；随后应出现
   `θ_ref 翻转为 0xXXXX`（翻转恢复），电机重新往复。
3. 观察新诊断：若仍卡住，从日志 θ 与目标差值可判定是"稳态误差>阈值"（θ 接近目标）还是
   其它（θ 远离目标）——前者继续放宽阈值，后者另查。
4. 回归：正常往复不受影响（到位阈值 0.2 rad 在连续运行下翻转正常）。

## 范围外

- 不改使能重试逻辑（上一轮已实现并生效）。
- 不新增运行时调参命令；阈值仍为编译期宏 `SRV_PA430_ARRIVE_THRESHOLD_RAD` 可调。
