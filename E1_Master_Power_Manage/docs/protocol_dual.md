
# 双电池 CAN 上行通信协议文档

## 1. 概述
- **数据长度**：所有上行 CAN 帧载荷固定为 **8 字节**。
- **字节序**：多字节字段采用 **小端模式**（Little-Endian）。
- **对齐方式**：强制 1 字节对齐，无填充字节。
- **帧 ID 映射**：

| CAN ID | 帧类型 | 发送周期/触发方式 |
|--------|--------|-------------------|
| 0x200  | 单电池核心动态帧 | 高频 MUX，100ms |
| 0x201  | 固化/低频信息帧 | 低频 MUX / 请求响应 |
| 0x202  | 独立详细故障数据包 | 事件触发 + 心跳 MUX |

---

## 2. 帧 ID: 0x200 – 单电池核心动态帧

| 字节偏移 | 字段名 | 数据类型 | 分辨率/单位 | 说明 |
|---------|--------|----------|-------------|------|
| Byte 0  | `bat_id` | uint8_t | - | 电池编号：0x01 = 电池1，0x02 = 电池2 |
| Byte 1-2 | `voltage` | uint16_t | 0.01 V/bit | 单包总电压 |
| Byte 3-4 | `current` | int16_t | 0.01 A/bit | 单包总电流（有符号，正为放电） |
| Byte 5  | `soc` | uint8_t | 1% / bit | 荷电状态，0~100 |
| Byte 6  | `cell_temp` | int8_t | 1 ℃ / bit | 电芯温度（有符号） |
| Byte 7  | `is_charging` | uint8_t | - | 充电状态：0 = 未充电，1 = 充电中 |

---

## 3. 帧 ID: 0x201 – 固化/低频信息帧

| 字节偏移 | 字段名 | 数据类型 | 说明 |
|---------|--------|----------|------|
| Byte 0  | `info_key` | uint8_t | 信息键值，决定后续载荷解析方式 |

### 3.1 Key = 0x01 – 容量信息

| 字节偏移 | 字段名 | 数据类型 | 分辨率/单位 | 说明 |
|---------|--------|----------|-------------|------|
| Byte 1-4 | `design_cap` | uint32_t | 1 mAh / bit | 设计容量 |
| Byte 5-6 | `full_cap` | uint16_t | 1 mAh / bit | 满充容量 |
| Byte 7  | `reserved1` | uint8_t | - | 预留 |

### 3.2 Key = 0x03 – 版本与循环信息（示例）

| 字节偏移 | 字段名 | 数据类型 | 分辨率/单位 | 说明 |
|---------|--------|----------|-------------|------|
| Byte 1-2 | `hw_version` | uint16_t | - | 硬件版本 |
| Byte 3-4 | `sw_version` | uint16_t | - | 软件版本 |
| Byte 5-6 | `cycle_count` | uint16_t | 次 | 循环次数 |
| Byte 7  | `reserved2` | uint8_t | - | 预留 |

---

## 4. 帧 ID: 0x202 – 独立详细故障数据包

| 字节偏移 | 字段名 | 数据类型 | 说明 |
|---------|--------|----------|------|
| Byte 0  | `bat_id` | uint8_t | 电池编号：0x01 = 电池1，0x02 = 电池2 |
| Byte 1-7 | `faults` | 联合体 | 根据电池类型选择不同故障结构（见下文） |

### 4.1 电池1（CAN 原生）故障结构 – `Bat_can_Detailed_Fault_t`

#### Byte 1 – 电压类故障

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `cell_ov` | 电芯过压保护 |
| 1 | `total_ov` | 总压过压保护 |
| 2 | `fully_charged` | 充满保护 |
| 3 | `cell_uv` | 电芯欠压保护 |
| 4 | `total_uv` | 总压欠压保护 |
| 5-7 | `volt_rsv` | 预留 |

#### Byte 2 – 电流/短路故障

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `short_circuit` | 放电短路保护 |
| 1 | `dischg_oc` | 放电过流保护 |
| 2 | `chg_oc` | 充电过流保护 |
| 3-7 | `curr_rsv` | 预留 |

#### Byte 3 – 温度故障

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `chg_ov_temp` | 充电高温保护 |
| 1 | `dischg_ov_temp` | 放电高温保护 |
| 2 | `mos_ov_temp` | MOS 过温保护 |
| 3 | `amb_ov_temp` | 环境高温保护 |
| 4 | `amb_low_temp` | 环境低温保护 |
| 5-7 | `temp_rsv` | 预留 |

#### Byte 4 – 硬件/采集故障

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `temp_sensor_fail` | 温度采集失效 |
| 1 | `volt_sensor_fail` | 电压采集失效 |
| 2 | `dischg_mos_fail` | 放电 MOS 失效 |
| 3 | `chg_mos_fail` | 充电 MOS 失效 |
| 4 | `cell_imbalance` | 电芯不均衡告警 |
| 5-7 | `hw_rsv` | 预留 |

#### Byte 5-6 – 其他预警

| 字节 | 字段名 | 类型 | 说明 |
|------|--------|------|------|
| 5-6 | `warnings` | uint16_t | 其他轻微预警状态集合（位掩码） |

#### Byte 7 – 严重等级

| 值 | 含义 |
|----|------|
| 0 | 正常 |
| 1 | 轻微 |
| 2 | 严重 |
| 3 | 致命 |

---

### 4.2 电池2（RYDER）故障结构 – `Bat_RYDER_Detailed_Fault_t`

#### Byte 1 – 温度保护/告警

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0-1 | `chg_temp_prot` | 充电高/低温保护（2位编码） |
| 2-3 | `dischg_temp_prot` | 放电高/低温保护（2位编码） |
| 4-5 | `chg_temp_warn` | 充电高/低温告警（2位编码） |
| 6-7 | `dischg_temp_warn` | 放电高/低温告警（2位编码） |

#### Byte 2 – 电压保护

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `chg_ov_prot` | 充电过压保护 |
| 1 | `chg_ov_hw` | 充电过压保护（硬件） |
| 2 | `chg_ov_second` | 充电过压二次保护（硬件） |
| 3 | `dischg_uv_prot` | 放电欠压保护 |
| 4 | `dischg_uv_hw` | 放电欠压保护（硬件） |
| 5-7 | `volt_rsv` | 预留 |

#### Byte 3 – 电流/硬件故障

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `chg_oc_prot` | 充电过流保护 |
| 1 | `short_circuit` | 短路保护 |
| 2 | `dischg_oc_prot` | 放电过流保护 |
| 3 | `dischg_oc_second` | 放电过流二次保护 |
| 4 | `dischg_oc_hw` | 放电过流硬件保护 |
| 5 | `hw_defected` | 电池硬件损坏 |
| 6-7 | `curr_rsv` | 预留 |

#### Byte 4 – 告警与 FET/保险

| 位 | 字段名 | 说明 |
|----|--------|------|
| 0 | `chg_ov_warn` | 充电过压告警 |
| 1 | `dischg_uv_warn` | 放电欠压告警 |
| 2 | `chg_oc_warn` | 充电过流告警 |
| 3 | `dischg_oc_warn` | 放电过流告警 |
| 4 | `chg_fet_fail` | 充电 FET（MOS）失效 |
| 5 | `dischg_fet_fail` | 放电 FET（MOS）失效 |
| 6 | `fuse_blown` | ★ 三端保险丝熔断 |
| 7 | `fuse_rsv` | 预留 |

#### Byte 5-6 – 其他告警

| 字节 | 字段名 | 类型 | 说明 |
|------|--------|------|------|
| 5-6 | `extra_warnings` | uint16_t | 其他告警状态（位掩码） |

#### Byte 7 – 严重等级

同 4.1 节 Byte 7 定义。

---

## 5. 统一载荷联合体

所有上行 CAN 帧均可通过以下联合体访问：

```c
typedef union {
    uint8_t data[8];                        // 原始字节数组
    Uplink_BatCore_t        bat_core;       // ID: 0x200
    Uplink_StaticInfo_t     static_info;    // ID: 0x201
    Uplink_Detailed_Fault_t detailed_fault; // ID: 0x202
} Uplink_CAN_Payload_u;
```

- 发送时根据 CAN ID 选择对应的结构体填充，然后以 `data[8]` 写入硬件发送寄存器。
- 接收端根据 CAN ID 解析对应的结构体。

---

> **注意**：所有多字节字段（如 `voltage`、`current`、`design_cap` 等）在内存中均为 **小端字节序**。