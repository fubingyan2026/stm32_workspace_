# 橘虾(巨蟹) MIT 电机 + Mz 扭矩传感器 — 串口控制协议

> 版本：V2.1.0 ／ 2026-08-26
> 控制板：E1_Hand_G474（STM32G474）
> 串口：USART1（PC4 TX / PC5 RX，**1M bps**，8N1）——专属上位机交互命令端口
> 固件日志：SEGGER RTT 输出（J-Link RTT Viewer 查看），不占用 USART1

---

## 目录

- [1. 通信基础配置](#1-通信基础配置)
- [2. 帧格式与 CRC](#2-帧格式与-crc)
- [3. 命令定义（主机 → 控制板）](#3-命令定义主机--控制板)
- [4. 应答定义（控制板 → 主机）](#4-应答定义控制板--主机)
- [5. 电机 MIT 载荷与参数](#5-电机-mit-载荷与参数)
- [6. 电机使用流程（重要）](#6-电机使用流程重要)
- [7. 扭矩传感器使用流程](#7-扭矩传感器使用流程)
- [8. 状态位与错误说明](#8-状态位与错误说明)
- [9. Python 完整示例](#9-python-完整示例)
- [10. 附录：CAN 侧协议速查](#10-附录can-侧协议速查)

---

## 1. 通信基础配置

| 项目 | 值 |
| :--- | :--- |
| 串口 | USART1（PC4=TX，PC5=RX） |
| 波特率 | 1 000 000 bps（1M），8 数据位 / 无校验 / 1 停止位 |
| 帧格式 | 20 字节定长帧 |
| 数据方向 | 主机 ↔ 控制板双向（一问一答 + 周期上报） |
| CAN1 | 橘虾(巨蟹)电机，CAN FD + BRS，仲裁段=数据段 1M |
| CAN2 | Mz 扭矩传感器，经典 CAN，1M |
| 控制频率 | 电机 MIT 控制帧 500Hz、传感器查询 500Hz（固件内部限频） |

> 控制板上电后：电机默认**失能 + 抱闸吸合 + 零刚度零力矩**（Kp=Kd=Tq=0），不会突动。
> 电机反馈帧由控制板发送的 MIT 控制帧自动触发，主机无需单独轮询电机 CAN 侧。

---

## 2. 帧格式与 CRC

| 偏移 | 长度 | 字段 | 说明 |
| :--- | :---: | :--- | :--- |
| 0~3 | 4B | head | 固定 `0x1400AA55`（小端字节序：`55 AA 00 14`） |
| 4~7 | 4B | cmd_id | 命令/应答 ID（小端），见 §3/§4 |
| 8~15 | 8B | data | 数据段 |
| 16~19 | 4B | crc | CRC16_CCITT_FALSE，低 2 字节有效、高 2 字节为 0 |

- CRC 计算范围：`cmd_id + data`（第 4~15 字节，共 12 字节）。
- CRC 参数：多项式 `0x1021`，初值 `0xFFFF`，无反射、无 XOR-out。
- 解析方按帧头 `55 AA 00 14` 重同步，CRC 错帧自动丢弃（不阻塞后续帧）。
- **字段命名说明**：`cmd_id` 是串口协议的**命令/应答标识符**，与 CAN 总线报文 ID（如电机的 `0x111`/`0x301`）**无关**；上位机代码中对应 `CID_*` 常量。

```python
import struct

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def pack(cid: int, data: bytes = b"") -> bytes:
    """构造 20 字节发送帧"""
    payload = struct.pack("<I", cid) + bytes(data).ljust(8, b"\x00")
    c = crc16_ccitt_false(payload)
    return b"\x55\xaa\x00\x14" + payload + struct.pack("<I", c)
```

---

## 3. 命令定义（主机 → 控制板）

| cmd_id | 命令 | data[8] | 应答 |
| :--- | :--- | :--- | :--- |
| `0x000001D0` | MIT 控制 | juxie 载荷 Byte[1..8]（见 §5） | 不逐帧应答 |
| `0x000001D1` | 电机配置 | `data[0]=param`，`data[1..2]=int16`（小端）值（见 §5.2） | ACK |
| `0x000001D2` | 电机标零 | 忽略 | ACK |
| `0x000001E0` | 读电机反馈 | `data[0]=电机ID(=1)` | 反馈帧 |
| `0x000001E1` | 读电机状态 | `data[0]=电机ID(=1)` | 状态帧 |
| `0x000000E2` | 读 Mz | 忽略 | Mz 数据帧 |
| `0x000000E3` | 流式控制 | `data[0]=0/1`（关/开），`data[1]=间隔 ms`（1~1000；填 0 时固件默认 100ms，上位机默认 10ms） | ACK |
| `0x000000E4` | Mz 标零 | 忽略 | ACK |

> **应答可靠性**：控制板应答帧经 8 帧 TX 队列排队发送，`get_fb + get_mz` 等背靠背命令的应答不再因 TX 忙而丢失。

---

## 4. 应答定义（控制板 → 主机）

| cmd_id | 内容 | data[8]（多字节均为小端） |
| :--- | :--- | :--- |
| `0x00E00100` | 电机反馈 | `pos(2B 0.01°) + speed(2B rpm) + iq(2B mA) + tq(2B 0.01Nm)` |
| `0x00E10100` | 电机状态 | `err(2B) + temp(2B 0.1℃) + mode(1B) + status(1B) + 预留(2B)` |
| `0x00E20000` | Mz 数据 | `mz(4B float32) + seq(2B) + flags(1B) + 预留(1B)` |
| `0x00E30000` | 流式帧 | `mz(4B float32) + pos(2B 0.01°) + tq(2B 0.01Nm)` |
| `0x00FF0000` | ACK/NAK | `data[0]=1(OK)/0(NAK)`，`data[1]=命令字节回显` |

字段说明：

- **电机反馈 `0xE00100`**：`pos`=负载端实际位置（0.01°，饱和 ±327.67°）；`speed`=电机端速度(rpm)；`iq`=实际电流(mA)；`tq`=力矩反馈（0.01Nm，由电机 0.05Nm/LSB 换算）。
- **电机状态 `0xE10100`**：`err`=电机错误码；`temp`=线圈温度(0.1℃)；`mode`=控制模式（0x06=MIT）；`status`=状态位（见 §8）。
- **Mz 数据 `0xE20000`**：`mz`=IEEE-754 float32（N·m）；`seq`=有效读取序号（**16bit 循环计数**，用于判断刷新率，回绕属正常）；`flags`：`bit0=传感器在线`、`bit1=标零完成`。
- **流式帧 `0xE30000`**：开启流式后按设定间隔主动上报最新 Mz + 电机位置 + 电机力矩，无需逐条查询。

---

## 5. 电机 MIT 载荷与参数

### 5.1 MIT 控制帧载荷（0x000001D0，data[8] = juxie CAN 帧 Byte[1..8]）

| 位宽 | 字段 | 范围 ↔ 物理量 |
| :--- | :--- | :--- |
| 16bit | 目标位置 pos | `[0..65535] ↔ (-Pos_Max ~ +Pos_Max)`（大端高字节在前） |
| 12bit | 目标速度 vel | `[0..4095] ↔ (-Vel_Max ~ +Vel_Max)` |
| 12bit | 位置增益 Kp | `[0..4095] ↔ [0..500]` |
| 12bit | 速度增益 Kd | `[0..4095] ↔ [0..5]` |
| 12bit | 目标力矩 tq | `[0..4095] ↔ (-T_Max ~ +T_Max)` |

> 控制板每 2ms（500Hz）将最近一次主机目标组帧下发到电机（CAN ID `0x110|Dev_ID`，CAN FD）。
> 电机帧 Byte[0] 控制指令头由控制板合成：`bit7 使能 / bit6 抱闸 / bit5 清错 / bit[4:1]=0x06(MIT)`，
> 使能与抱闸来自 §5.2 配置——**这就是“使能 + 抱闸释放后 MIT 指令才有效”的原因**。

原始值换算公式：

```
pos_raw = (pos_deg / Pos_Max + 1) / 2 * 65535      # 饱和 [0..65535]
vel_raw = (vel_rpm / Vel_Max + 1) / 2 * 4095        # 饱和 [0..4095]
tq_raw  = (tq_Nm  / T_Max   + 1) / 2 * 4095         # 饱和 [0..4095]
kp_raw  = kp / 500 * 4095                            # 饱和 [0..4095]
kd_raw  = kd / 5   * 4095                            # 饱和 [0..4095]
```

### 5.2 电机配置参数（0x000001D1）

| param | 含义 | value |
| :---: | :--- | :--- |
| 1 | 使能控制 | `1`=使能，`0`=失能 |
| 2 | 清错误 | `1`=请求清错（脉冲，发完一帧自动复位） |
| 3 | 抱闸 | `1`=释放，`0`=吸合 |
| 4 | Pos_Max | 0.1°，int16（如 1800 = 180.0°） |
| 5 | Vel_Max | rpm，int16 |
| 6 | T_Max | 0.01Nm，int16（如 5000 = 50.00Nm） |

量程默认值：`Pos_Max=180.0°`、`Vel_Max=3000 rpm`、`T_Max=50.00 Nm`。

---

## 6. 电机使用流程（重要）

> 核心原则：**先使能 + 抱闸释放，MIT 控制指令才会生效**。
> 控制板始终以 500Hz 下发 MIT 帧，但帧内使能/抱闸位未置位时，电机不施加力矩。

### 6.1 启动流程

```
① 连接串口（1M）并确认联机
② （可选）设置量程           → 0x1D1 param=4/5/6
③ 读电机反馈/状态确认在线     → 0xE0 / 0xE1（读取当前角度，判断是否需要标零）
④ （可选）电机标零           → 0x1D2（当前角度置零；建议先在零位或可控状态执行）
⑤ 先下发零力矩目标（防突动） → 0x1D0：kp=kd=tq=0，pos=当前角度
⑥ 使能电机                  → 0x1D1 param=1, value=1
⑦ 抱闸释放                  → 0x1D1 param=3, value=1
⑧ 下发 MIT 控制目标         → 0x1D0：pos/vel/kp/kd/tq（见 §5.1）
⑨ 运行中随时更新目标（含动态调 Kp/Kd）→ 0x1D0
```

### 6.2 停机流程（安全顺序，严禁带力直接断电）

```
① 下发零力矩                → 0x1D0：kp=kd=tq=0
② 抱闸吸合                  → 0x1D1 param=3, value=0
③ 失能电机                  → 0x1D1 param=1, value=0
```

### 6.3 注意事项

- Kp/Kd 从**小值起步**（如 Kp=20、Kd=0.5），逐步加大，避免冲击/振荡。
- 位置目标应参考反馈 `pos`（0xE0 返回）设置，避免从当前位置直接跳变。
- 若电机报错（status.bit5=1），先发 `0x1D1 param=2, value=1` 清错，确认后再运行。
- 电机标零（0x1D2）在电机未使能时也可执行，但须确保电机处于已知位置（否则“零点”含义不明）。

---

## 7. 扭矩传感器使用流程

```
① 读 Mz 确认在线       → 0xE2（flags.bit0=1 在线）
② （可选）传感器标零   → 0xE4（写寄存器 0x4604=1.0，通道1；应答后 flags.bit1=1）
③ 单次读取             → 0xE2
④ 连续观测（推荐）     → 0xE3：data[0]=1 开流式，data[1]=间隔ms（如 10）
                          控制板按间隔上报 0xE30000（Mz + 电机位置 + 电机力矩）
⑤ 关闭流式             → 0xE3：data[0]=0
```

控制板以 500Hz 查询传感器（CAN2 `0x510→0x410`），最新值经 `0xE2`/`0xE3` 上报，主机无需直接操作 CAN。

---

## 8. 状态位与错误说明

### 8.1 电机状态字节（0xE10100 的 status）

| 位 | 含义 |
| :---: | :--- |
| bit7 | 使能状态（1=已使能） |
| bit6 | 抱闸状态（1=已释放） |
| bit5 | 报错状态（1=有错误） |
| bit4 | 位置到位（1=到位） |
| bit3~0 | 预留 |

### 8.2 Mz 数据 flags（0xE20000）

| 位 | 含义 |
| :---: | :--- |
| bit0 | 传感器在线（1=在线） |
| bit1 | 标零完成（1=最近一次标零已确认） |

### 8.3 ACK/NAK（0x00FF0000）

- `data[0]=1`：命令已接受并执行；`data[0]=0`：拒绝（参数非法、总线忙等）。
- `data[1]`：回显命令 `cmd_id` 低 8 位，便于多命令并发时对应。

---

## 9. Python 完整示例

```python
# -*- coding: utf-8 -*-
"""串口控制：使能 → 抱闸释放 → MIT 控制 → 停机
复用 host/juxie_host.py 的协议函数（pack/mit_ctrl/motor_cfg 等）。"""
import time
import serial

from juxie_host import (mit_ctrl, motor_cfg, pack,
    CID_GET_FB, CID_GET_MZ, CID_STREAM,
    to_pos_raw, to_vel_raw, to_tq_raw, to_kp_raw, to_kd_raw,
    PARAM_ENABLE, PARAM_BRAKE, PARAM_POS_MAX, PARAM_VEL_MAX, PARAM_TQ_MAX)

ser = serial.Serial("COM5", 1000000, timeout=0.05)
cfg = {"pos_max": 180.0, "vel_max": 3000.0, "tq_max": 50.0}   # 默认量程

# ---- 启动流程 ----
ser.write(motor_cfg(PARAM_POS_MAX, int(cfg["pos_max"] * 10)))  # 量程(0.1°)
ser.write(motor_cfg(PARAM_VEL_MAX, int(cfg["vel_max"])))        # 量程(rpm)
ser.write(motor_cfg(PARAM_TQ_MAX,  int(cfg["tq_max"] * 100)))   # 量程(0.01Nm)
ser.write(pack(CID_GET_FB))                                     # 读当前角度
ser.write(motor_cfg(PARAM_ENABLE, 1))                           # ① 使能
ser.write(motor_cfg(PARAM_BRAKE,  1))                           # ② 抱闸释放
# ③ 下发 MIT 目标（使能+抱闸释放后才会生效）
ser.write(mit_ctrl(
    to_pos_raw(30.0,  cfg["pos_max"]),   # 位置 30°
    to_vel_raw(0.0,   cfg["vel_max"]),   # 速度 0
    to_kp_raw(50.0),                     # Kp=50
    to_kd_raw(0.5),                      # Kd=0.5
    to_tq_raw(0.0,    cfg["tq_max"])))   # 力矩 0
time.sleep(2)

# ---- 停机流程 ----
ser.write(mit_ctrl(0, 0, 0, 0, 0))       # ① 零力矩
ser.write(motor_cfg(PARAM_BRAKE, 0))     # ② 抱闸吸合
ser.write(motor_cfg(PARAM_ENABLE, 0))    # ③ 失能
```

> 完整可运行上位机见 `host/juxie_host.py`（含位置滑块、连续发送、反馈显示、流式、标零），
> 默认波特率 1M、流式间隔 10ms、自动刷新 50ms；监控页「查询率(/s)」实时显示实际收到的 Mz/流式帧数。
> 协议自检见 `host/test_protocol.py`。

---

## 10. 附录：CAN 侧协议速查

### 10.1 橘虾(巨蟹)电机（CAN1，FD+BRS，1M）

- MIT 单轴控制：`0x110|Dev_ID`（=0x111），DLC 9，Byte[0]=控制头，Byte[1..8]=§5.1 载荷。
- 反馈（控制帧自动触发）：`0x300|Dev_ID`（=0x301），DLC 16（带力矩）或 12。
- 标零：SDO 写 `0x601`，数据 `23 31 25 00 01 00 00 00`（Index 0x2531 Sub 0 = 1）。
- 同步帧：`0x80`（控制板每 10ms 发送，用于触发反馈广播）。

### 10.2 Mz 扭矩传感器（CAN2，经典 CAN 1M）

- 查询：`0x510`，DLC 7，`04 00 00 00 00 00 00`。
- 应答：`0x410`，`04 00 00` + 4B 大端 IEEE-754 float32（N·m）。
- 标零：`0x510`，DLC 7，`10 46 04 3F 80 00 00`（写 0x4604=1.0，通道 1）。

---

## 修订记录

| 版本 | 日期 | 说明 |
| :--- | :--- | :--- |
| V1.0.0 | 2026-08-26 | 初版（命令/应答定义） |
| V2.0.0 | 2026-08-26 | 补充电机使用流程（使能→抱闸释放→MIT 才有效）、停机流程、频率参数、完整示例 |
| V2.1.0 | 2026-08-26 | 应答 TX 队列（背靠背应答不丢帧）、seq 循环计数说明、上位机默认参数（1M/流式 10ms/自动刷新 50ms/查询率显示） |
