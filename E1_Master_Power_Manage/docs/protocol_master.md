# 主电源板(CTU) → 主机 CAN 上报协议

- **作者**：maximillian
- **日期**：2026-08-21
- **版本**：V1.0.0
- **摘要**：定义主电源板（STM32F407）与上位机之间的 CAN 通信协议。上报帧（主电源板 → 主机）：0x010 系统状态帧（2 字节，急停/电源轨/风扇/模拟输入/NTC 连接）、0x011 温度帧（NTC1/NTC2/MCU）、0x012 电源电压+预充故障帧（VIN/MOTOR/AUX + 电机预充故障码），每 100ms 周期上报。主机控制帧（主机 → 主电源板）：0x001 统一控制帧（6 字节，蜂鸣器 + HSD 输出 + LED RGB，led_index 选通道）、0x003 请求进入升级模式。

---

## 总线

主电源板与主机之间通过 **P_CAN**（CAN1，1 Mbps）进行双向通信：

| 方向 | CAN ID | 说明 |
|------|--------|------|
| 主电源板 → 主机 | 0x010, 0x011, 0x012 | 系统状态 + 温度 + 电源电压/预充故障 |
| 主机 → 主电源板 | 0x001 | 蜂鸣器/输出控制 + LED RGB |
| 主机 → 主电源板 | 0x003 | 请求进入升级模式（Boot） |

---

## 帧 ID 总览

| CAN ID | 帧名 | 方向 | 触发方式 | 说明 |
|--------|------|------|----------|------|
| 0x010 | 系统状态 | 主电源板 → 主机 | **始终发送** | 急停/电源轨错误/风扇/模拟输入/NTC 连接 |
| 0x011 | 温度 | 主电源板 → 主机 | 始终发送 | NTC1/NTC2/MCU 温度 |
| 0x012 | 电源电压+预充故障 | 主电源板 → 主机 | 始终发送 | VIN/MOTOR/AUX 电压 + 电机预充故障码 |
| 0x001 | 统一控制指令 | 主机 → 主电源板 | 主机主动发送 | 蜂鸣器 + 输出控制 + LED RGB（led_index 选通道） |
| 0x003 | 进入升级模式 | 主机 → 主电源板 | 主机主动发送 | 请求 App 进入 Bootloader 升级 |

标准 11-bit ID，多字节字段 **小端字节序**。0x010 为 2 字节帧，0x011/0x012 为 8 字节帧，0x001 控制帧为 6 字节。

---

## 0x010 — 系统状态帧  [主电源板 → 主机]

状态帧压缩为 **2 字节**（状态位 + NTC 连接状态）。

| 字节 | 位 | 字段 | 描述 |
|------|-----|------|------|
| 0 | 0 | `stop_key_state` | 急停：0=释放, 1=按下 |
| | 1 | `err_12v_ext` | 外部 12V 输出异常 |
| | 2 | `err_24v_ext` | 外部 24V 输出异常 |
| | 3 | `err_24v_computer` | 工控机 24V 输出异常 |
| | 4 | `err_aux_power` | 辅电电源异常（AUX PGD） |
| | 5 | `err_motor_power` | 电机电源异常（MOTOR PGD） |
| | 6 | `err_chg_out` | 预充电异常（CHG OCP） |
| | 7 | `err_hsd_fault` | HSD 公用通道异常 |
| 1 | 0 | `err_dbr` | 制动电阻过流（DBR OCP） |
| | 1 | `a_in1_io` | A_IN1_IO 模拟输入 |
| | 2 | `a_in2_io` | A_IN2_IO 模拟输入 |
| | 3 | `a_in3_io` | A_IN3_IO 模拟输入 |
| | 4 | `err_fan0` | 风扇0 异常 |
| | 5 | `err_fan1` | 风扇1 异常 |
| | 6 | `err_ntc1` | NTC1 未连接 |
| | 7 | `err_ntc2` | NTC2 未连接 |

---

## 0x011 — 温度帧  [主电源板 → 主机]

随 0x010 每 100ms 周期上报。

| 字节 | 字段 | 单位 | 描述 |
|------|------|------|------|
| 0-1 | `ntc1_temp` | 0.01°C (int16 LE) | NTC1 外部温度（°C×100） |
| 2-3 | `ntc2_temp` | 0.01°C (int16 LE) | NTC2 外部温度（°C×100） |
| 4-5 | `mcu_temp` | 0.01°C (int16 LE) | MCU 内部温度（°C×100） |
| 6-7 | 保留 | | 恒 0 |

---

## 0x012 — 电源电压 + 预充故障帧  [主电源板 → 主机]

随 0x010 每 100ms 周期上报。

| 字节 | 字段 | 单位 | 描述 |
|------|------|------|------|
| 0-1 | `vin_mv` | mV (uint16 LE) | 主输入电压 |
| 2-3 | `motor_power_mv` | mV (uint16 LE) | 电机电源电压 |
| 4-5 | `aux_power_mv` | mV (uint16 LE) | 辅助电源电压 |
| 6 | `precharge_fault` | | 电机预充故障码：0=无, 1=后级短路, 2=未接负载或者二极管断路 |
| 7 | 保留 | | 恒 0 |

---

## 主机 → 主电源板 控制协议

主机通过同一条 P_CAN 总线向主电源板发送控制指令。

| CAN ID | 帧名 | 方向 | 帧长度 | 说明 |
|--------|------|------|--------|------|
| 0x001 | 统一控制指令 | 主机 → 主电源板 | 6 字节 | 蜂鸣器 + 输出控制 + LED RGB |
| 0x003 | 进入升级模式 | 主机 → 主电源板 | 1 字节 | 请求 App 置 upgrade_flag 复位进 Bootloader |

### 0x001 — 统一控制指令帧

| 字节 | 字段 | 描述 |
|------|------|------|
| 0 | `buzzer_duty` | 蜂鸣器占空比 0-50 |
| 1 | `ctrl_byte`   | 输出控制位（见下表） |
| 2 | `led_index` | LED 索引：0-31=通道1(RGB1/SPI1)，32-63=通道2(RGB2/SPI3) |
| 3 | `led_r` | LED 红亮度 0-255 |
| 4 | `led_g` | LED 绿亮度 0-255 |
| 5 | `led_b` | LED 蓝亮度 0-255 |

**Byte1 控制位：**

| 位 | 名称 | 描述 |
|----|------|------|
| 5 | `hsd1_12v_en` | 1=bit4 有效 |
| 4 | `hsd1_12v_in` | HSD1 12V 输出：1=开, 0=关 |
| 3 | `hsd1_24v_en` | 1=bit2 有效 |
| 2 | `hsd1_24v_in` | HSD1 24V 输出：1=开, 0=关 |
| 1 | `hsd2_24v_en` | 1=bit0 有效 |
| 0 | `hsd2_24v_in` | HSD2 24V 输出：1=开, 0=关 |

**要点：**
- 每对控制位（valid + value）独立有效：valid=1 时才更新对应输出，valid=0 时忽略该位
- LED 通过 `led_index` 区分通道：0-31=通道1(RGB1/SPI1)，32-63=通道2(RGB2/SPI3)；一帧控制一个 LED，分别发帧可分别控制两通道
- 索引超出对应通道 LED 数时忽略该灯
- 收到含 LED RGB 的 0x001 后灯带退出彗星动画，进入手动控制模式；后续帧覆盖对应 LED 并刷新
- 帧长度必须为 **6 字节**，否则丢弃

### 0x003 — 进入升级模式帧

| 字节 | 字段 | 描述 |
|------|------|------|
| 0 | `magic` | 固定 `0x01`，表示请求进入升级模式 |

- 主电源板收到后置 BOOT 分区 `upgrade_flag=1` 并系统复位；复位后 Boot 镜像判定 `upgrade_flag ≠ 0` 进入升级模式，等待主机通过 0x701/0x702 协议烧写新固件。
- 帧长度必须为 **1 字节**，否则忽略。

### 交互流程

| 步骤 | 主机 | 方向 | 主电源板 |
|------|------|------|---------|
| 1 | 发送 0x001 (buzzer=50, hsd1=ON) | → | |
| 2 | | ← | 0x010 系统状态帧（始终回复） |
| 3 | | ← | 0x011 温度帧（始终回复） |
| 4 | | ← | 0x012 电源电压+预充故障（始终回复） |

---

## 代码映射（数据链接与同层解耦）

协议帧与工程内数据结构的对应关系如下。`srv_can_mst`（service 层）遵循**回调注入**模式，
仅持有回调指针，**不直接调用任何 `drv_*` 设备驱动或业务模块**；所有硬件/业务耦合由
task 层（`can_task.c`）通过回调注入，保持 service 与应用/驱动的同层解耦。

### 上报帧（主电源板 → 主机）

| 帧 | 工程结构体（字段顺序 = 协议位序） | 数据填充位置（read_data 回调） |
|----|----------------------------------|------------------------------|
| 0x010 系统状态 | `srv_can_mst_status_frame_t`（位域 union，2 字节） | `app_status_report_fill()` 聚合：`srv_pwr_det_read()` 提供急停/电源轨/HSD/DBR，`srv_adc_read_ain()` 提供 A_IN1~3_IO，`srv_fan_ctrl_is_fault()` 提供风扇，`srv_adc_get_latest()` 提供 NTC 连接态 |
| 0x011 温度 | `srv_can_mst_volt_temp_frame_t`（8 字节） | `srv_adc_get_latest()` 的 NTC1/NTC2/MCU 温度（°C×100，int16 LE） |
| 0x012 电压+预充 | `srv_can_mst_power_fault_frame_t`（8 字节） | VIN/MOTOR/AUX 电压来自 `srv_adc_get_latest()`；`precharge_fault` 来自 `srv_pwr_ctrl_get_precharge_fault()`（0=无, 1=后级短路 SHORT_CIRCUIT, 2=未接负载/二极管断路 NO_LOAD） |

打包链路：`srv_can_mst_request()` 同步调用 `read_data` → `cm_build_0x001 / cm_build_volt_temp / cm_build_power_fault` 入队；`srv_can_mst_task()` 经 `send_frame` 回调（`can_task.c:can_send_frame`，底层 `drv_can_send`）逐帧发送。

### 控制帧（主机 → 主电源板）

| 帧 | 工程结构体 / 函数 | 消费位置与解耦方式 |
|----|------------------|------------------|
| 0x001 统一控制 | `srv_can_mst_cmd_t`（解析于 `srv_can_mst_process_rx`，6 字节） | byte0 `buzzer_duty`（0-50）、byte1 `ctrl_byte`（3 对 valid+value）、byte2 `led_index`、byte3-5 LED RGB |
| 0x001 HSD 输出 | `set_output` 回调（`srv_can_mst_set_output_cb_t`） | `srv_can_mst_process_rx()` 仅在 valid 位置位时调用 `s_config.set_output(out, on)`；task 层 `can_task.c:can_set_output()` 将抽象通道映射为 `drv_power_set(DRV_POWER_RAIL_HSD1_12V_DIAG / HSD1_24V_DIAG / HSD2_24V_DIAG, on)`。**service 层不直连 `drv_power`，同层解耦** |
| 0x001 LED RGB | `srv_can_mst_get_cmd()` + `srv_ws2812b_set_pixel()` | RX 解析在 ISR（`can_rx_callback`），LED 应用延后到主循环 `can_timer_cb`（避免 ISR 内 SPI DMA） |
| 0x003 进 Boot | `srv_boot_ctrl_request_boot()` | RX 仅置 `s_enter_boot_requested` 标志，主循环 `can_timer_cb` 消费并调用 |

### 同层解耦要点
- `srv_can_mst`（service 层）只持有 `read_data` / `send_frame` / `set_output` 三个回调指针，不含任何 `drv_*` 或业务模块调用。
- `can_task.c`（task 层）是唯一同时接触 CAN 驱动、电源驱动与上报服务的边界：它实现上述回调，把抽象协议通道翻译为具体硬件动作（`drv_power_set`、`srv_ws2812b_set_pixel`、`srv_boot_ctrl_request_boot`）。

---

## 通讯时序

### 上报触发机制

**0x010 系统状态帧**、**0x011 温度帧**、**0x012 电源电压+预充故障帧**每 100ms 自动周期上报（一帧一拍，队列逐帧发送）。

### 参数

| 参数 | 值 | 说明 |
|------|----|------|
| TASK_PERIOD_MS | 10 ms | 主循环周期 |
| REPORT_INTERVAL_MS | 100 ms | 自动上报周期 |
| 发送策略 | 每周期 1 帧 | 队列逐帧发送，TX 忙时等待 |
