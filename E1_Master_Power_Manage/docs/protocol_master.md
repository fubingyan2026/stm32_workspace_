# 主电源板 → 主机 CAN 上报协议

## 总线

主电源板与主机之间通过 **P_CAN**（CAN1，1 Mbps）进行双向通信：

| 方向 | CAN ID | 说明 |
|------|--------|------|
| 主电源板 → 主机 | 0x001, 0x011-0x015 | 系统状态 + 电池数据上报 |
| 主机 → 主电源板 | 0x001 | 反馈请求 + RGB/蜂鸣器/输出控制 |
| 主机 → 主电源板 | 0x003 | 请求进入升级模式（Boot） |

---

## 帧 ID 总览

| CAN ID | 帧名 | 方向 | 触发方式 | 说明 |
|--------|------|------|----------|------|
| 0x001 | 系统状态 | 主电源板 → 主机 | **始终发送** | 急停/电源轨错误/电池简况 |
| 0x011 | 电池基础 | 主电源板 → 主机 | feedback_select bit0 | 双电池容量 + 循环 + 充电 |
| 0x012 | 电池电压电流 | 主电源板 → 主机 | feedback_select bit1 | 双电池电压 + 电流 |
| 0x013 | 电池版本 | 主电源板 → 主机 | feedback_select bit2 | 双电池 HW/SW 版本 |
| 0x014 | 故障码 | 主电源板 → 主机 | feedback_select bit3 | 双电池详细故障 |
| 0x015 | 故障等级+预警+温度 | 主电源板 → 主机 | feedback_select bit4 | 严重等级 + 预警 + 温度 |
| 0x003 | 进入升级模式 | 主机 → 主电源板 | 主机主动发送 | 请求 App 进入 Bootloader 升级（见 [boot_upgrade.md](boot_upgrade.md)） |

所有帧固定 **8 字节**（DLC=8），标准 11-bit ID，多字节字段 **小端字节序**。

---

## 0x001 — 系统状态帧  [主电源板 → 主机]

| 字节 | 位 | 字段 | 描述 |
|------|-----|------|------|
| 0 | 0 | `stop_key_state` | 急停：0=释放, 1=按下 |
| | 1 | `battery_key_state` | 电池开关：0=关, 1=开 |
| | 2 | `battery_charging` | 充电（任一电池）：0=放电, 1=充电 |
| | 3 | `battery_temp_error` | 电池温度异常 |
| | 4 | `device_online_slaver` | 副电源管理控制板在线 |
| | 5 | `device_online_dual` | 双电池控制板在线 |
| | 6 | `device_online_bat1` | 电池1 在线 |
| | 7 | `device_online_bat2` | 电池2 在线 |
| 1 | 0 | `err_vin` | 主输入电压异常 (48V) |
| | 1 | `err_vin_dcdc` | DCDC 输出异常 |
| | 2 | `err_12v_int` | 内部 12V 异常 |
| | 3 | `err_5v_int` | 内部 5V 异常 |
| | 4 | `err_12v_ext` | 外部 12V 异常 |
| | 5 | `err_24v_ext` | 外部 24V 异常 |
| | 6 | `err_12v_user` | 用户 12V 异常 |
| | 7 | `err_24v_user` | 用户 24V 异常 |
| 2 | 0 | `err_24v_comp` | 工控机 24V 异常 |
| | 1 | `err_power` | 从板电源异常 |
| | 2 | `err_motor` | 电机电源异常 |
| | 3 | `err_chg_out` | 预充电异常 |
| | 4 | `err_hsd1_12v` | HSD1 12V 异常 |
| | 5 | `err_hsd2_12v` | HSD2 12V 异常 |
| | 6 | `err_hsd3_12v` | HSD3 12V 异常 |
| | 7 | `err_dbr` | 制动电阻异常 |
| 3 | 0 | `err_hsd1_24v` | HSD1 24V 异常 |
| | 1 | `err_hsd2_24v` | HSD2 24V 异常 |
| | 2 | `err_hsd3_24v` | HSD3 24V 异常 |
| | 3 | `err_lsd1_24v` | LSD1 24V 异常 |
| | 4 | `err_lsd2_24v` | LSD2 24V 异常 |
| | 5 | `err_fan0` | 风扇0 异常 |
| | 6 | `err_fan1` | 风扇1 异常 |
| | 7 | `byte3_fixed1` | 协议固定为 1 |
| 4 | 0-7 | `err_ntc[0..7]` | 8路 NTC 温度异常 |
| 5 | 0 | `a_in1_io` | A_IN1_IO 模拟输入 |
| | 1 | `a_in2_io` | A_IN2_IO 模拟输入 |
| | 2 | `a_in3_io` | A_IN3_IO 模拟输入 |
| | 3 | `seq_vin_fault` | VIN_DCDC 上电故障 |
| | 4 | `seq_chg_fault` | 预充电时序故障 |
| | 5 | `seq_motor_fault` | 电机上电故障 |
| | 6-7 | 保留 | |
| 6 | | `bat1_soc` | 电池1 SOC（0-100%） |
| 7 | | `bat2_soc` | 电池2 SOC（0-100%，0=无电池2） |

---

## 0x011 — 电池基础参数  [主电源板 → 主机]

SOC 已由 0x001 上报，本帧不再重复。

| 字节 | 字段 | 单位 | 描述 |
|------|------|------|------|
| 0-1 | `bat1_capacity` | 256 mAh | 电池1 设计容量高16位 (÷256) |
| 2-3 | `bat2_capacity` | 256 mAh | 电池2 设计容量高16位 (÷256) |
| 4 | `bat1_cycle` | 次 | 电池1 循环次数低8位 |
| 5 | `bat2_cycle` | 次 | 电池2 循环次数低8位 |
| 6 | 充电标志 | | bit0=bat1_charging, bit1=bat2_charging |
| 7 | 保留 | | |

---

## 0x012 — 电池电压 + 电流  [主电源板 → 主机]

| 字节 | 字段 | 单位 | 描述 |
|------|------|------|------|
| 0-1 | `bat1_voltage` | 0.1V (uint16 LE) | 电池1 电压 |
| 2-3 | `bat2_voltage` | 0.1V (uint16 LE) | 电池2 电压 |
| 4-5 | `bat1_current` | 0.1A (int16 LE) | 电池1 电流（正=放电） |
| 6-7 | `bat2_current` | 0.1A (int16 LE) | 电池2 电流（正=放电） |

---

> 电流、温度不再单独成帧，已并入 0x012（电压+电流）与 0x015（等级+预警+温度）。

---

## 0x013 — 电池版本  [主电源板 → 主机]

| 字节 | 字段 | 描述 |
|------|------|------|
| 0-1 | `bat1_hw_version` | 电池1 硬件版本 (uint16 LE) |
| 2-3 | `bat1_sw_version` | 电池1 软件版本 (uint16 LE) |
| 4-5 | `bat2_hw_version` | 电池2 硬件版本 (uint16 LE) |
| 6-7 | `bat2_sw_version` | 电池2 软件版本 (uint16 LE) |

---

## 0x014 — 双电池故障码  [主电源板 → 主机]

### 电池1 (CAN 原生)，字节 0-3

**Byte 0 — 电压类故障**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `cell_ov` | 电芯过压保护 |
| 1 | `total_ov` | 总压过压保护 |
| 2 | `fully_charged` | 充满保护 |
| 3 | `cell_uv` | 电芯欠压保护 |
| 4 | `total_uv` | 总压欠压保护 |
| 5-7 | 预留 | |

**Byte 1 — 电流/短路故障**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `short_circuit` | 放电短路保护 |
| 1 | `dischg_oc` | 放电过流保护 |
| 2 | `chg_oc` | 充电过流保护 |
| 3-7 | 预留 | |

**Byte 2 — 温度故障**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `chg_ov_temp` | 充电高温保护 |
| 1 | `dischg_ov_temp` | 放电高温保护 |
| 2 | `mos_ov_temp` | MOS 过温保护 |
| 3 | `amb_ov_temp` | 环境高温保护 |
| 4 | `amb_low_temp` | 环境低温保护 |
| 5-7 | 预留 | |

**Byte 3 — 硬件/采集故障**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `temp_sensor_fail` | 温度采集失效 |
| 1 | `volt_sensor_fail` | 电压采集失效 |
| 2 | `dischg_mos_fail` | 放电 MOS 失效 |
| 3 | `chg_mos_fail` | 充电 MOS 失效 |
| 4 | `cell_imbalance` | 电芯不均衡告警 |
| 5-7 | 预留 | |

### 电池2 (RYDER)，字节 4-7

**Byte 4 — 温度保护/告警**

| 位 | 字段 | 说明 |
|----|------|------|
| 0-1 | `chg_temp_prot` | 充电高/低温保护（2位编码） |
| 2-3 | `dischg_temp_prot` | 放电高/低温保护（2位编码） |
| 4-5 | `chg_temp_warn` | 充电高/低温告警（2位编码） |
| 6-7 | `dischg_temp_warn` | 放电高/低温告警（2位编码） |

**Byte 5 — 电压保护**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `chg_ov_prot` | 充电过压保护 |
| 1 | `chg_ov_hw` | 充电过压保护（硬件） |
| 2 | `chg_ov_second` | 充电过压二次保护 |
| 3 | `dischg_uv_prot` | 放电欠压保护 |
| 4 | `dischg_uv_hw` | 放电欠压保护（硬件） |
| 5-7 | 预留 | |

**Byte 6 — 电流/硬件故障**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `chg_oc_prot` | 充电过流保护 |
| 1 | `short_circuit` | 短路保护 |
| 2 | `dischg_oc_prot` | 放电过流保护 |
| 3 | `dischg_oc_second` | 放电过流二次保护 |
| 4 | `dischg_oc_hw` | 放电过流硬件保护 |
| 5 | `hw_defected` | 电池硬件损坏 |
| 6-7 | 预留 | |

**Byte 7 — 告警与 FET/保险**

| 位 | 字段 | 说明 |
|----|------|------|
| 0 | `chg_ov_warn` | 充电过压告警 |
| 1 | `dischg_uv_warn` | 放电欠压告警 |
| 2 | `chg_oc_warn` | 充电过流告警 |
| 3 | `dischg_oc_warn` | 放电过流告警 |
| 4 | `chg_fet_fail` | 充电 FET 失效 |
| 5 | `dischg_fet_fail` | 放电 FET 失效 |
| 6 | `fuse_blown` | 三端保险丝熔断 |
| 7 | 预留 | |

---

## 0x015 — 故障等级 + 预警 + 温度  [主电源板 → 主机]

| 字节 | 字段 | 描述 |
|------|------|------|
| 0-1 | `bat1_warnings` | 电池1 其他预警位掩码 (uint16 LE) |
| 2 | `bat1_fault_level` | 电池1 严重等级 (0=正常,1=轻微,2=严重,3=致命) |
| 3 | `bat2_fault_level` | 电池2 严重等级 |
| 4-5 | `bat2_extra_warn` | 电池2 其他告警位掩码 (uint16 LE) |
| 6 | `bat1_temp` | 电池1 电芯温度 (°C, int8) |
| 7 | `bat2_temp` | 电池2 电芯温度 (°C, int8) |

---

## 主机 → 主电源板 控制协议

主机通过同一条 P_CAN 总线向主电源板发送控制指令，触发数据上报和输出控制。

| CAN ID | 帧名 | 方向 | 帧长度 | 说明 |
|--------|------|------|--------|------|
| 0x001 | 控制指令 | 主机 → 主电源板 | 7 字节 | 反馈请求 + RGB/蜂鸣器/输出控制 |
| 0x003 | 进入升级模式 | 主机 → 主电源板 | 1 字节 | 请求 App 置 upgrade_flag 复位进 Bootloader |

### 0x001 — 控制指令帧

| 字节 | 字段 | 描述 |
|------|------|------|
| 0 | `feedback_select` | 位掩码，选择请求哪些电池数据帧（见上文 feedback_select 表） |
| 1 | `rgb_mode` | RGB 灯效模式 |
| 2 | `rgb_color.R` | RGB 颜色——R 分量 |
| 3 | `rgb_color.G` | RGB 颜色——G 分量 |
| 4 | `rgb_color.B` | RGB 颜色——B 分量 |
| 5 | `buzzer_duty` | 蜂鸣器占空比 0-100 |
| 6 | `ctrl_byte` | 输出控制位（见下表） |

**Byte6 控制位：**

| 位 | 名称 | 描述 |
|----|------|------|
| 7 | `valid_hsd1_12v` | 1=bit6 有效 |
| 6 | `hsd1_12v_on` | HSD1 12V 输出：1=开, 0=关 |
| 5 | `valid_hsd2_12v` | 1=bit4 有效 |
| 4 | `hsd2_12v_on` | HSD2 12V 输出：1=开, 0=关 |
| 3 | `valid_lsd1_24v` | 1=bit2 有效 |
| 2 | `lsd1_24v_on` | LSD1 24V 输出：1=开, 0=关 |
| 1 | `valid_lsd2_24v` | 1=bit0 有效 |
| 0 | `lsd2_24v_on` | LSD2 24V 输出：1=开, 0=关 |

**要点：**
- 每对控制位（valid + value）独立有效：valid=1 时才更新对应输出，valid=0 时忽略该位
- 主机发送 0x001 后，主电源板按 `feedback_select` 打包状态帧和电池帧回复
- 帧长度必须为 **7 字节**，否则丢弃

### 0x003 — 进入升级模式帧

| 字节 | 字段 | 描述 |
|------|------|------|
| 0 | `magic` | 固定 `0x01`，表示请求进入升级模式 |

- 主电源板收到后置 BOOT 分区 `upgrade_flag=1` 并系统复位；复位后 Boot 镜像判定 `upgrade_flag ≠ 0` 进入升级模式，等待主机通过 0x701/0x702 协议烧写新固件。
- 升级协议与流程见 [boot_upgrade.md](boot_upgrade.md)。
- 帧长度必须为 **1 字节**，否则忽略。

### 交互流程

| 步骤 | 主机 | 方向 | 主电源板 |
|------|------|------|---------|
| 1 | 发送 0x001 (feedback_select=0x1F, buzzer=50, hsd1=ON) | → | |
| 2 | | ← | 0x001 系统状态帧（始终回复） |
| 3 | | ← | 0x011 电池基础（因反馈请求 bit0） |
| 4 | | ← | 0x012 电池电压+电流（因反馈请求 bit1） |
| 5 | | ← | 0x013 电池版本（因反馈请求 bit2） |
| 6 | | ← | 0x014 故障码（因反馈请求 bit3） |
| 7 | | ← | 0x015 等级+预警+温度（因反馈请求 bit4） |

> `feedback_select=0x00` 时仅返回 0x001 系统状态帧，不返回电池数据帧。

---

## 通讯时序

### 上报触发机制

**0x001 系统状态帧**每 100ms 自动发送。电池数据帧（0x011-0x015）**按需上报**——仅当主机通过 0x001 控制帧指定 `feedback_select` 位时才回复对应帧。`feedback_select=0x00` 时只返回 0x001 状态帧。

### 参数

| 参数 | 值 | 说明 |
|------|----|------|
| TASK_PERIOD_MS | 10 ms | 主循环周期 |
| REPORT_INTERVAL_MS | 100 ms | 自动上报周期 |
| 发送策略 | 每周期 1 帧 | 队列逐帧发送，TX 忙时等待 |

### feedback_select 位掩码

| 位 | 宏 | 请求帧 |
|----|-----|--------|
| bit0 | `SRV_CAN_MST_FEEDBACK_BAT_BASE` | 0x011 |
| bit1 | `SRV_CAN_MST_FEEDBACK_BAT_VOLTAGE` | 0x012（电压+电流） |
| bit2 | `SRV_CAN_MST_FEEDBACK_BAT_STATUS` | 0x013 |
| bit3 | `SRV_CAN_MST_FEEDBACK_BAT_ERROR` | 0x014 |
| bit4 | `SRV_CAN_MST_FEEDBACK_BAT_COUNTER` | 0x015（等级+预警+温度） |
| bit5-7 | 保留 | |
