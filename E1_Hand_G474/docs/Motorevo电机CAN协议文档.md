# Motorevo CANFD/CAN 电机驱动模组完整协议文档

本文档由《Motorevo CANFD\&CAN协议模组使用说明书》整理提炼而成，覆盖电机通信所需的全部协议内容：通信基础、状态切换、控制报文、反馈帧、参数读写、在线升级(OTA)。

---

## 1. 通信基础

### 1.1 寻址方式

电机支持两种寻址方式，数据端格式均为**高字节在前、低字节在后**（大端序）：

| 方式 | 报文标识符 | DLC | 帧类型 | 电机ID范围 | 一条总线电机数 |
|---|---|---|---|---|---|
| 广播方式 | 状态帧 `0x10`，控制帧 `0x20` | 64 | 标准帧 | 1\~8 | 最多8个关节 |
| 单独方式 | 状态帧 `0x100+ID`，控制帧 `0x200+ID` | 8 | 标准帧 | 1\~255 | 可超过8台 |

> **注意：单独方式仅在固件 260617（及以后）才支持。**

### 1.2 广播方式 64 字节布局

| Byte | 含义 |
|---|---|
| Byte0\-Byte7 | 主机发送给 ID=1 关节的 8Bytes 指令数据 |
| Byte8\-Byte15 | 主机发送给 ID=2 关节的 8Bytes 指令数据 |
| Byte16\-Byte23 | 主机发送给 ID=3 关节的 8Bytes 指令数据 |
| Byte24\-Byte31 | 主机发送给 ID=4 关节的 8Bytes 指令数据 |
| Byte32\-Byte39 | 主机发送给 ID=5 关节的 8Bytes 指令数据 |
| Byte40\-Byte47 | 主机发送给 ID=6 关节的 8Bytes 指令数据 |
| Byte48\-Byte55 | 主机发送给 ID=7 关节的 8Bytes 指令数据 |
| Byte56\-Byte63 | 主机发送给 ID=8 关节的 8Bytes 指令数据 |

即每个电机的 8 字节槽位位于 `Byte[(ID-1)*8]` 起始处。

### 1.3 FDCAN 波特率配置

| | 5MHz 版 | 4MHz 版 |
|---|---|---|
| CAN 时钟主频 | 170MHz | 170MHz |
| 仲裁域 BPR | 10 | 10 |
| 仲裁域 Seg1 | 13 | 13 |
| 仲裁域 Seg2 | 3 | 3 |
| 仲裁域 Sync Jump Width | 3 | 3 |
| 仲裁域波特率 | 1MHz | 1MHz |
| 仲裁域采样率 | 82.3% | 82.3% |
| 数据域 BPR | 1 | 1 |
| 数据域 Seg1 | 25 | 31 |
| 数据域 Seg2 | 8 | 11 |
| 数据域 Sync Jump Width | 4 | 11 |
| 数据域波特率 | 5MHz | 3.95MHz |
| 数据域采样率 | 76.47% | 74.41% |

- 两个版本的波特率在老固件上**不支持修改参数切换，需要 OTA 固件切换**；260617 后的固件允许通过参数 `Protocol Type`（Index 69）切换：`0 = CANFD 5M`，`1 = CANFD 4M`，`2 = CAN`。

### 1.4 总线终端电阻

串联多个电机时，**总线首端和尾端**需并入 120Ω 电阻做阻抗匹配。首端或尾端未接 120Ω 电阻可能导致某些电机丢包严重，甚至总线错误引起 `Buff OFF`。位于总线末端的电机可用驱动板上自带终端电阻焊盘短接。

### 1.5 CAN 经典协议

CAN 协议下的指令与 CANFD 协议下**单独控制方式指令完全相同，仅波特率不同**。

---

## 2. 状态定义与状态切换报文

### 2.1 状态定义

| 状态 | 描述 | 指示灯 |
|---|---|---|
| 失能状态 | 三相PWM不开关，电机不运行FOC控制 | 仅黄灯亮 |
| 使能状态 | 三相PWM发波，电机运行FOC控制 | 黄灯蓝灯一起亮 |
| 保护状态 | 使能状态下检测到错误进入保护模式，电机下使能，三相PWM关断 | 黄灯红灯一起亮 |

### 2.2 状态切换报文

#### 广播方式（标识符 `0x10`，DLC 64，标准帧）

每个电机的 8 字节槽位定义（偏移 = `(ID-1)*8`）：

| Byte | 字节格式 | 含义 |
|---|---|---|
| Byte[(ID-1)\*8+0] | 0xFF | 固定 |
| Byte[(ID-1)\*8+1] | 0xFF | 固定 |
| Byte[(ID-1)\*8+2] | 0xFF | 固定 |
| Byte[(ID-1)\*8+3] | 0xFF | 固定 |
| Byte[(ID-1)\*8+4] | 0xFF | 固定 |
| Byte[(ID-1)\*8+5] | 0xFF | 固定 |
| Byte[(ID-1)\*8+6] | 0xFF | 固定 |
| Byte[(ID-1)\*8+7] | 0xFC | **使能命令**：命令电机进入使能状态 |
| | 0xFD | **失能命令**：命令进入失能状态 |
| | 0xFE | **设零命令**：设置当前位置为零位，并保存Flash |
| | 0xFB | **清错命令**：确认故障排除后清除故障码 |
| | 0xFA | **查询命令**：查询电机当前状态，电机不执行任何操作，但会回复一帧报文响应（需固件 ≥ 260617） |

#### 单独方式（标识符 `0x100+Motor ID`，DLC 8，标准帧）

| Byte | 字节格式 | 含义 |
|---|---|---|
| Byte0 | 0xFF | 固定 |
| Byte1 | 0xFF | 固定 |
| Byte2 | 0xFF | 固定 |
| Byte3 | 0xFF | 固定 |
| Byte4 | 0xFF | 固定 |
| Byte5 | 0xFF | 固定 |
| Byte6 | 0xFF | 固定 |
| Byte7 | 0xFC / 0xFD / 0xFE / 0xFB / 0xFA | 同广播方式定义 |

#### 示例：广播方式给 8 个关节发送使能指令

标识符 `0x10`，DLC 64：每个槽位数据均为 `0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC`。

#### 示例：单独方式给 2 号电机发送使能指令

标识符 `0x102`，DLC 8，数据：`0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC`。

> **⚠️ 重要警告：**
> - 发送 `0xFE` 设零命令后，电机将对片上 Flash 进行擦写，**发送后 1 秒内必须确保电机供电稳定**，否则可能发生 Flash 丢失（电角度信息、编码器校准信息永久丢失，回归默认参数），只能重新配置参数并重新校准。
> - 设零命令发 1\~2 帧即可，**不要连续发送几百上千帧**，会大大增加参数丢失风险。

---

## 3. 电机控制报文

控制报文标识符均为 `0x20`（广播）或 `0x200+Motor ID`（单独）。不同 ID 的电机根据内部配置的 **Control Mode（Index 11）** 解析报文。

> **注意：控制前务必确认报文是按照对应电机的控制模式封包，否则可能让电机跑飞！**
>
> 支持混合模式：例如 1\~3 号电机为 MIT、4\~6 号为 Servo Position、7\~8 号为 Velocity，则可通过一条 `0x20` 报文，Byte0\~23 用 MIT 封包、Byte24\~47 用 Servo Position 封包、Byte48\~63 用 Velocity 封包，同时控制 8 个模式不同的电机。

### 3.1 MIT 模式（力位混合控制模式，Control Mode = 0x2）

电机扭矩输出公式：

```
T_out = Kp × (θ_ref - θ) + Kd × (V_ref - V) + T_ref
```

| 参数 | 单位 | 含义 | 来源 |
|---|---|---|---|
| Kp | Nm/rad | 刚度 | 主机通过 CAN 发送 |
| Kd | Nm/(rad/s) | 阻尼 | 主机通过 CAN 发送 |
| θ_ref | rad | 目标位置 | 主机通过 CAN 发送 |
| V_ref | rad/s | 目标速度 | 主机通过 CAN 发送 |
| T_ref | Nm | 前馈扭矩 | 主机通过 CAN 发送 |
| θ | rad | 当前电机实际位置 | 电机内部采集 |
| V | rad/s | 当前电机实际转速 | 电机内部采集 |
| T_out | Nm | 控制器输出扭矩 | 限幅在 ±Torque Limit(Nm) 后除以 Torque Constant(Nm/A) 得到 I_q_ref，送入电流PI控制器 |

#### 广播方式 8 字节封包（`0x20`，DLC 64，槽位偏移 `(ID-1)*8`）

| Byte | 位宽 | 含义 | 映射 |
|---|---|---|---|
| Byte[(ID-1)\*8+0] | 8bits | θ_ref（高8位） | 0x0000 ↔ θ_ref=CAN COM Theta MIN；0xFFFF ↔ θ_ref=CAN COM Theta MAX |
| Byte[(ID-1)\*8+1] | 8bits | θ_ref（低8位） | （16bit 位置，范围 0~65535） |
| Byte[(ID-1)\*8+2] | 8bits | V_ref（高8位） | Byte2[0-7] 为速度高8位，Byte3[4-7] 为速度低4位；0x000 ↔ V_ref=CAN COM Velocity MIN；0xFFF ↔ V_ref=CAN COM Velocity MAX |
| Byte[(ID-1)\*8+3] | 高4bits | V_ref（低4位） | |
| | 低4bits | Kp（高4位） | Byte3[0-3] 为 Kp 高4位，Byte4[0-7] 为 Kp 低8位；0x000 ↔ Kp=CAN COM Kp MIN；0xFFF ↔ Kp=CAN COM Kp MAX |
| Byte[(ID-1)\*8+4] | 8bits | Kp（低8位） | |
| Byte[(ID-1)\*8+5] | 8bits | Kd（高8位） | Byte5[0-7] 为 Kd 高8位，Byte6[4-7] 为 Kd 低4位；0x000 ↔ Kd=CAN COM Kd MIN；0xFFF ↔ Kd=CAN COM Kd MAX |
| Byte[(ID-1)\*8+6] | 高4bits | Kd（低4位） | |
| | 低4bits | T_ref（高4位） | Byte6[0-3] 为 T_ref 高4位，Byte7[0-7] 为 T_ref 低8位；0x000 ↔ T_ref=CAN COM Torque MIN；0xFFF ↔ T_ref=CAN COM Torque MAX |
| Byte[(ID-1)\*8+7] | 8bits | T_ref（低8位） | |

#### 单独方式 8 字节封包（`0x200+Motor ID`，DLC 8）

字节布局与广播方式相同，只是 Byte0\~Byte7 直接对应各字段（无槽位偏移）。

#### 示例：8 个电机全部发送 `Kp=0, Kd=0, θ_ref=0, V_ref=0, T_ref=0`

每个槽位数据：`0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF`

#### MIT 封包 Python 代码（官方示例）

```python
# 通信范围常量
CAN_COM_THETA_MIN = -12.5
CAN_COM_THETA_MAX = 12.5
CAN_COM_VELOCITY_MIN = -10.0
CAN_COM_VELOCITY_MAX = 10.0
CAN_COM_KP_MIN = 0.0
CAN_COM_KP_MAX = 250.0
CAN_COM_KD_MIN = 0.0
CAN_COM_KD_MAX = 50.0
CAN_COM_TORQUE_MIN = -50.0
CAN_COM_TORQUE_MAX = 50.0

def pack_motor_command_8Bytes(
    target_position: float,
    target_velocity: float,
    target_torque: float,
    kp: float,
    kd: float,
) -> bytes:
    """
    字节布局:
        Byte0-1: θ_ref (16bits) - 位置
        Byte2[0-7] + Byte3[4-7]: V_ref (12bits) - 速度
        Byte3[0-3] + Byte4[0-7]: K_p (12bits) - 比例
        Byte5[0-7] + Byte6[4-7]: K_d (12bits) - 微分
        Byte6[0-3] + Byte7[0-7]: T_ref (12bits) - 力矩
    """
    FULL_SCALE_16BIT = 65535.0  # 2^16-1
    FULL_SCALE_12BIT = 4095.0   # 2^12-1
    pos_raw = int((target_position - CAN_COM_THETA_MIN) /
                  (CAN_COM_THETA_MAX - CAN_COM_THETA_MIN) * FULL_SCALE_16BIT)
    pos_raw = max(0, min(int(FULL_SCALE_16BIT), pos_raw))

    vel_raw = int((target_velocity - CAN_COM_VELOCITY_MIN) /
                  (CAN_COM_VELOCITY_MAX - CAN_COM_VELOCITY_MIN) * FULL_SCALE_12BIT)
    vel_raw = max(0, min(int(FULL_SCALE_12BIT), vel_raw))

    kp_raw = int((kp - CAN_COM_KP_MIN) /
                 (CAN_COM_KP_MAX - CAN_COM_KP_MIN) * FULL_SCALE_12BIT)
    kp_raw = max(0, min(int(FULL_SCALE_12BIT), kp_raw))

    kd_raw = int((kd - CAN_COM_KD_MIN) /
                 (CAN_COM_KD_MAX - CAN_COM_KD_MIN) * FULL_SCALE_12BIT)
    kd_raw = max(0, min(int(FULL_SCALE_12BIT), kd_raw))

    torque_raw = int((target_torque - CAN_COM_TORQUE_MIN) /
                     (CAN_COM_TORQUE_MAX - CAN_COM_TORQUE_MIN) * FULL_SCALE_12BIT)
    torque_raw = max(0, min(int(FULL_SCALE_12BIT), torque_raw))

    byte0 = (pos_raw >> 8) & 0xFF          # Position[15:8]
    byte1 = pos_raw & 0xFF                  # Position[7:0]
    byte2 = (vel_raw >> 4) & 0xFF          # Velocity[11:4]
    byte3 = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)  # Velocity[3:0] | Kp[11:8]
    byte4 = kp_raw & 0xFF                  # Kp[7:0]
    byte5 = (kd_raw >> 4) & 0xFF           # Kd[11:4]
    byte6 = ((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F)  # Kd[3:0] | Torque[11:8]
    byte7 = torque_raw & 0xFF              # Torque[7:0]

    return bytes([byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7])

def pack_64byte_message(
    motor1: tuple, motor2: tuple, motor3: tuple, motor4: tuple,
    motor5: tuple, motor6: tuple, motor7: tuple, motor8: tuple,
) -> bytes:
    """将8个电机的指令封装为64字节CAN报文，每个参数为
    (target_position, target_velocity, target_torque, kp, kd) 的元组"""
    motors = [motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8]
    result = b''
    for m in motors:
        cmd = pack_motor_command_8Bytes(*m)
        result += cmd
    return result
```

### 3.2 Servo Position 模式（伺服模式，Control Mode = 0x1）

控制结构：外环位置环计算速度环目标速度，经过限幅后进入速度环：

```
V_ref = Kp_pos × (θ_ref - θ) - Kd_pos × V
V_ref = clip(V_ref, -V_ref_limit, +V_ref_limit)
T_out = Ki_vel × ∫(V_ref - V)dt + Kp_vel × (V_ref - V)
```

| 参数 | 单位 | 含义 |
|---|---|---|
| Kp_pos | 1/s | 位置环比例系数 |
| Kd_pos | / | 位置环微分系数 |
| Ki_vel | Nm/(rad/s) | 速度环积分系数 |
| Kp_vel | Nm/(rad/s) | 速度环比例系数 |
| θ_ref | rad | 目标位置 |
| V_ref_limit | rad/s | 目标速度限幅 |
| θ | rad | 当前电机实际位置 |
| V | rad/s | 当前电机实际转速 |
| T_out | Nm | 控制器输出扭矩（限幅 ±Torque Limit，除以 Torque Constant 得到 I_q_ref） |

#### 广播方式 8 字节封包（`0x20`，DLC 64，槽位偏移 `(ID-1)*8`）

| Byte | 位宽 | 含义 | 映射 |
|---|---|---|---|
| Byte[(ID-1)\*8+0] | 8bits | θ_ref（高8位） | 0x0000 ↔ CAN COM Theta MIN；0xFFFF ↔ CAN COM Theta MAX |
| Byte[(ID-1)\*8+1] | 8bits | θ_ref（低8位） | |
| Byte[(ID-1)\*8+2] | 8bits | V_ref | 0x00 ↔ CAN COM Velocity MIN；0xFF ↔ CAN COM Velocity MAX |
| Byte[(ID-1)\*8+3] | 8bits | Kp_pos | 0x00 ↔ CAN COM Kp MIN；0xFF ↔ CAN COM Kp MAX |
| Byte[(ID-1)\*8+4] | 8bits | Kd_pos | 0x00 ↔ CAN COM Kd MIN；0xFF ↔ CAN COM Kd MAX |
| Byte[(ID-1)\*8+5] | 8bits | Kp_vel | 0x00 ↔ CAN COM Kp MIN；0xFF ↔ CAN COM Kp MAX |
| Byte[(ID-1)\*8+6] | 8bits | Kd_vel | 0x00 ↔ CAN COM Kd MIN；0xFF ↔ CAN COM Kd MAX |
| Byte[(ID-1)\*8+7] | 8bits | Ki_vel | 0x00 ↔ CAN COM Ki MIN；0xFF ↔ CAN COM Ki MAX |

#### 单独方式 8 字节封包（`0x200+Motor ID`，DLC 8）

Byte0\~Byte7 直接对应各字段，无槽位偏移。

#### 示例：8 个电机全部发送 `Kp_pos=0, Kd_pos=0, θ_ref=0, V_ref=0, Kp_vel=0, Kd_vel=0, Ki_vel=0`

每个槽位数据：`0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x00,0x00`

### 3.3 Velocity 模式（速度模式，Control Mode = 0x3）

电机扭矩输出公式：

```
T_out = Kp_vel × (V_ref - V) + ∫ Ki_vel × (V_ref - V) dt
```

| 参数 | 单位 | 含义 |
|---|---|---|
| Ki_vel | Nm/(rad/s) | 积分刚度 |
| Kp_vel | Nm/(rad/s) | 阻尼 |
| V_ref | rad/s | 目标速度 |
| V | rad/s | 当前电机实际转速 |
| T_out | Nm | 控制器输出扭矩（限幅 ±Torque Limit，除以 Torque Constant 得到 I_q_ref） |

#### 广播方式 8 字节封包（`0x20`，DLC 64，槽位偏移 `(ID-1)*8`）

| Byte | 位宽 | 含义 | 映射 |
|---|---|---|---|
| Byte[(ID-1)\*8+0] | 8bits | V_ref（高8位） | 0x0000 ↔ CAN COM Velocity MIN；0xFFFF ↔ CAN COM Velocity MAX |
| Byte[(ID-1)\*8+1] | 8bits | V_ref（低8位） | |
| Byte[(ID-1)\*8+2] | 8bits | Kp_vel（高8位） | Byte2[0-7] 为高8位，Byte3[4-7] 为低4位；0x000 ↔ CAN COM Kp MIN；0xFFF ↔ CAN COM Kp MAX |
| Byte[(ID-1)\*8+3] | 高4bits | Kp_vel（低4位） | |
| | 低4bits | Kd_vel（高4位） | Byte3[0-3] 为高4位，Byte4[0-7] 为低8位；0x000 ↔ CAN COM Kd MIN；0xFFF ↔ CAN COM Kd MAX |
| Byte[(ID-1)\*8+4] | 8bits | Kd_vel（低8位） | |
| Byte[(ID-1)\*8+5] | 8bits | Ki_vel（高8位） | Byte5[0-7] 为高8位，Byte6[4-7] 为低4位；0x000 ↔ CAN COM Ki MIN；0xFFF ↔ CAN COM Ki MAX |
| Byte[(ID-1)\*8+6] | 高4bits | Ki_vel（低4位） | |
| | 低4bits | 无意义 | |
| Byte[(ID-1)\*8+7] | 8bits | **0xAC** | **固定字节，必须发 0xAC，否则将忽略本帧报文** |

#### 单独方式 8 字节封包（`0x200+Motor ID`，DLC 8）

| Byte | 位宽 | 含义 | 映射 |
|---|---|---|---|
| Byte0 | 8bits | V_ref（高8位） | 0x0000 ↔ CAN COM Velocity MIN；0xFFFF ↔ CAN COM Velocity MAX |
| Byte1 | 8bits | V_ref（低8位） | |
| Byte2 | 8bits | Kp_vel（高8位） | Byte2[0-7] 为高8位，Byte3[4-7] 为低4位 |
| Byte3 | 高4bits | Kp_vel（低4位） | |
| | 低4bits | Kd_vel（高4位） | Byte3[0-3] 为高4位，Byte4[0-7] 为低8位 |
| Byte4 | 8bits | Kd_vel（低8位） | |
| Byte5 | 8bits | Ki_vel（高8位） | Byte5[0-7] 为高8位，Byte6[4-7] 为低4位 |
| Byte6 | 高4bits | Ki_vel（低4位） | |
| | 低4bits | 无意义 | |
| Byte7 | 8bits | **0xAC** | **固定字节，必须发 0xAC，否则将忽略本帧报文** |

---

## 4. 电机反馈帧

- 报文标识符：固件 260617 以前固定为电机 Motor ID。260617 以后可由参数 **CAN_MASTER（Index 67）** 自定义：当 CAN_MASTER < 0 时，标识符为 Motor ID；当 CAN_MASTER ≥ 0 时，标识符为 CAN_MASTER 值。
- DLC：8，帧类型：标准帧，数据大端。

| Byte | 位宽 | 含义 | 映射 |
|---|---|---|---|
| Byte0 | 8bits | 当前电机角度 θ(rad)（高8位） | 0x0000 ↔ CAN COM Theta MIN；0xFFFF ↔ CAN COM Theta MAX。**饱和处理**：θ > Theta MAX 反馈 0xFFFF，θ < Theta MIN 反馈 0x0000 |
| Byte1 | 8bits | θ（低8位） | |
| Byte2 | 8bits | 当前电机速度 V(rad/s)（高8位） | Byte2[0-7] 为速度高8位，Byte3[4-7] 为速度低4位；0x000 ↔ CAN COM Velocity MIN；0xFFF ↔ CAN COM Velocity MAX。**饱和处理**同上 |
| Byte3 | 高4bits | V（低4位） | |
| | 低4bits | 当前电机输出扭矩 T(Nm)（高4位） | Byte3[0-3] 为 T 高4位，Byte4[0-7] 为 T 低8位；0x000 ↔ CAN COM Torque MIN；0xFFF ↔ CAN COM Torque MAX。**饱和处理**同上 |
| Byte4 | 8bits | T（低8位） | |
| Byte5 | 8bits | 当前电机线圈温度(℃) | 0x00 ↔ -40℃；0xFF ↔ 215℃ |
| Byte6 | 8bits（高8位） | 错误码&使能指示，共16位 | 见下表 |
| Byte7 | 8bits（低8位） | | |

### 错误码 & 使能指示位定义（Byte6 = 高8位，Byte7 = 低8位）

| Bit | 含义 |
|---|---|
| Bit 0 | 使能指示位：1 = 电机在使能状态；0 = 不在使能状态（报错保护下使能后也会置0） |
| Bit 1 | 母线过压：母线电压超过 BUS_OV_LOCK 阈值触发 |
| Bit 2 | 相电流过流：三相电流任意一相超过 PHASE_OC_LOCK 阈值触发 |
| Bit 3 | 电机线圈过温：线圈温度超过 Motor_OT_LOCK 阈值触发 |
| Bit 4 | 电机超速 |
| Bit 5 | 暂未定义 |
| Bit 6 | 暂未定义 |
| Bit 7 | 暂未定义 |
| Bit 8 | 电机堵转报警：Iq 电流大于 Stuck_Current，同时速度小于 Stuck_Velocity，持续时间超过 Stuck_Time 时触发；任一条件不成立则计时重置 |
| Bit 9 | 暂未定义 |
| Bit 10 | 驱动板板温过温：板温超过 PCB_OT_LOCK 阈值触发 |
| Bit 11 | 母线欠压：母线电压低于 BUS_UV_LOCK 阈值触发 |
| Bit 12 | 位置超限 |
| Bit 13 | CAN 通信超时：在 CAN_TIME_OUT 时间内没有收到报文触发 |
| Bit 14 | 暂未定义 |
| Bit 15 | 暂未定义 |

---

## 5. 修改电机参数

### 5.1 参数表（Index 10\~76）

| 参数名 | Index(Dec) | 数据类型 | 单位 | 解释 |
|---|---|---|---|---|
| Firmware Version | 10 | Int32 | / | 固件版本号，例如 0xEF260325 表示 2026-03-25 发布的 EF 版本 |
| Control Mode | 11 | Int32 | / | 0x1 = Servo Position 模式；0x2 = MIT 模式；0x3 = Velocity 模式 |
| Id Controller Kp | 12 | Float32 | V/A | FOC 电流环 D 轴 PI 比例项 Kp |
| Id Controller Ki | 13 | Float32 | Hz | FOC 电流环 D 轴 PI 积分项 Ki |
| Iq Controller Kp | 14 | Float32 | V/A | FOC 电流环 Q 轴 PI 比例项 Kp |
| Iq Controller Ki | 15 | Float32 | Hz | FOC 电流环 Q 轴 PI 积分项 Ki |
| Current DeadZone | 16 | Float32 | A | 电流环死区，DQ 轴电流 error 小于此值置 0 |
| Velocity DeadZone | 17 | Float32 | rad/s | 速度环死区 |
| Position DeadZone | 18 | Float32 | rad | 位置环死区 |
| Current Integral Limit | 19 | Float32 | / | 电流环积分限幅系数，限幅在 Current Integral Limit × V_bus 范围内 |
| Velocity Integral Limit | 20 | Float32 | Nm | 速度环积分限幅 |
| Torque Limit | 21 | Float32 | Nm | 控制器力限保护，任何模式下输出扭矩都不会超过此值 |
| Electric Angle Offset | 22 | Float32 | rad | 电角度偏移。**严禁随意修改，值错误会导致电机烧毁或失控跑飞！仅由校准命令生成** |
| Machine Angle Offset | 23 | Float32 | rad | 驱动器机械零位，输出角度 = 测量值 - Machine Angle Offset |
| CAN COM Theta MIN | 24 | Float32 | rad | CAN 通讯数据范围 |
| CAN COM Theta MAX | 25 | Float32 | rad | CAN 通讯数据范围 |
| CAN COM Velocity MIN | 26 | Float32 | rad/s | CAN 通讯数据范围 |
| CAN COM Velocity MAX | 27 | Float32 | rad/s | CAN 通讯数据范围 |
| CAN COM Kp MIN | 28 | Float32 | Nm/rad | CAN 通讯数据范围 |
| CAN COM Kp MAX | 29 | Float32 | Nm/rad | CAN 通讯数据范围 |
| CAN COM Kd MIN | 30 | Float32 | Nm/(rad/s) | CAN 通讯数据范围 |
| CAN COM Kd MAX | 31 | Float32 | Nm/(rad/s) | CAN 通讯数据范围 |
| CAN COM Ki MIN | 32 | Float32 | / | CAN 通讯数据范围 |
| CAN COM Ki MAX | 33 | Float32 | / | CAN 通讯数据范围 |
| CAN COM Torque MIN | 34 | Float32 | Nm | CAN 通讯数据范围 |
| CAN COM Torque MAX | 35 | Float32 | Nm | CAN 通讯数据范围 |
| Motor ID | 36 | Int32 | / | 本机 Motor ID，范围 1\~8 |
| CAN COM Time Out | 37 | Int32 | ms | CAN 中断超时报警时间 |
| Default Position Kp | 38 | Float32 | / | EtherCAT 模式伺服位置环 Kp |
| Default Position Kd | 39 | Float32 | / | EtherCAT 模式伺服位置环 Kd |
| Default Velocity Kp | 40 | Float32 | / | EtherCAT 模式伺服速度环 Kp |
| Default Velocity Ki | 41 | Float32 | / | EtherCAT 模式伺服速度环 Ki |
| Acceleration | 42 | Float32 | rad/s² | Servo Position 模式下的加速度值 |
| Torque slope | 43 | Float32 | / | 力矩上升率限制。值过大会导致很高瞬间冲击电流，损坏驱动器或电机 |
| NPP | 44 | Int32 | / | 电机极对数，必须与实际极对数一致，**错误会导致驱动或电机烧毁** |
| Gear Ratio | 45 | Float32 | / | 减速器减速比，与实际不符会导致位置显示不准确 |
| Torque Constant | 46 | Float32 | Nm/A | 扭矩常数，出厂时设定好 |
| Rotate Dir | 47 | Int32 | / | 电机运行正方向，只能为 0 或 +1。**修改后必须重新进行电角度校准和输出轴编码器校准** |
| Encoder Config | 48 | Int32 | / | |
| BUS_OV_LOCK | 49 | Float32 | V | 母线过压阈值 |
| BUS_UV_LOCK | 50 | Float32 | V | 母线欠压阈值 |
| PHASE_OC_LOCK | 51 | Float32 | A | 相电流过流保护阈值 |
| PCB_OT_LOCK | 52 | Float32 | ℃ | 驱动板温度保护阈值 |
| Stuck Current | 53 | Float32 | A | 堵转保护：Iq > Stuck Current 且速度 < Stuck Velocity 且持续 > Stuck Time |
| Stuck Velocity | 54 | Float32 | rad/s | 堵转保护参数 |
| Stuck Time | 55 | Float32 | s | 堵转保护参数 |
| Protect Switchs | 56 | Int32 | / | 保护开关，bit=1 打开：Bit0 位置超限、Bit1 超速、Bit2 相电流过流、Bit3 过温、Bit4 过压欠压、Bit5 堵转、Bit6 输入轴编码器自检、Bit7 CAN 通讯中断、Bit8 线圈温度超温。全开 = 0xFFFFFFFF；关位置超限+超速 = 0xFFFFFFFC |
| Encoder Gain | 57 | / | / | **编码器参数，用户严禁修改** |
| Brake Action DTC | 58 | / | / | |
| Brake Action Time | 59 | / | / | |
| Brake Hold DTC | 60 | / | / | |
| Current Filter Bandwidth | 61 | Int32 | Hz | 电流滤波器带宽，0 = 关闭 |
| Velocity Filter Bandwidth | 62 | Int32 | Hz | 速度滤波器带宽，0 = 关闭 |
| Position Filter Bandwidth | 63 | Int32 | Hz | 位置滤波器带宽，0 = 关闭 |
| V_Calibration | 64 | Float32 | / | 电角度校准相关系数，无特殊情况无需修改 |
| Dual Encoder Config | 65 | / | / | **编码器参数，用户严禁修改** |
| Motor OT Lock | 66 | Float32 | ℃ | 电机绕组温度保护阈值 |
| CAN Master | 67 | Int32 | / | 电机反馈帧报文标识符，参考反馈章节 |
| Nonius Flag | 68 | Int32 | / | 编码器校准标识符，做过校准为 1，否则为 0。**68 及以后参数均为 260617 版本之后新加入** |
| Protocol Type | 69 | Int32 | / | 0 = CANFD 5M；1 = CANFD 4M；2 = CAN |
| Error Code | 70 | Int32 | / | 错误码 |
| Error Information | 71 | float | / | 错误反馈信息，如触发过压则该值为触发瞬间电压值 |
| Ld | 72 | float | mH | D 轴电感 |
| Lq | 73 | float | mH | Q 轴电感 |
| Rs | 74 | float | mOhm | 相电阻 |
| Flux | 75 | float | mWb | 磁通 |
| BOOTLOADER Firmware Version | 76 | Int32 | / | Bootloader 版本号，如 0xAA260325 |

### 5.2 通过 CAN 协议修改参数

#### 帧定义（参数读写帧）

标识符：`0x600 + Motor ID`，DLC 8，标准帧：

| Byte | 内容 | 含义 |
|---|---|---|
| Byte0 | 0x67 | 帧头 |
| Byte1 | Index | 索引，确定读写对象（见参数表 Index） |
| Byte2 | Data0 | 数据第1字节 |
| Byte3 | Data1 | 数据第2字节 |
| Byte4 | Data2 | 数据第3字节 |
| Byte5 | Data3 | 数据第4字节 |
| Byte6 | R/W | 读命令 = 0x04；写命令 = 0x15 |
| Byte7 | 0x76 | 帧尾 |

#### 电机反馈（读命令或写命令均反馈一帧；写命令反馈新参数）

标识符：`0x600 + Motor ID`，DLC 8，标准帧：

| Byte | 内容 | 含义 |
|---|---|---|
| Byte0 | Motor ID | 电机 ID |
| Byte1 | Index | 索引 |
| Byte2 | Data0 | 数据第1字节 |
| Byte3 | Data1 | 数据第2字节 |
| Byte4 | Data2 | 数据第3字节 |
| Byte5 | Data3 | 数据第4字节 |
| Byte6 | 0x00 | 固定为 0x00 |
| Byte7 | 0xFF | 帧尾 |

#### 保存命令

发送完所有修改后，发送 **Index = 0x00**（Data0\~Data3 任意，R/W 任意）的保存命令，将新参数存入 Flash。

> **⚠️ 发送保存命令后需确保电机供电稳定至少 1 秒以上**，等待电机完成 Flash 存储；若存储过程中供电不稳，很可能发生 Flash 丢失，一旦丢失则无法继续使用电机。

#### 示例（Motor ID = 6）

| 内容 | 报文（标识符 0x606） |
|---|---|
| 设置 [12] Id Controller Kp 为 0.3（0x3E99999A） | 0x67 0x0C 0x9A 0x99 0x99 0x3E 0x15 0x76 |
| 设置 [13] Id Controller Ki 为 0.15（0x3E19999A） | 0x67 0x0D 0x9A 0x99 0x19 0x3E 0x15 0x76 |
| 设置 [43] Torque Slop 为 5.0（0x40A00000） | 0x67 0x2B 0x00 0x00 0xA0 0x40 0x15 0x76 |
| 设置 [51] PHASE-OC-LOCK 为 30.0（0x41F00000） | 0x67 0x33 0x00 0x00 0xF0 0x41 0x15 0x76 |
| 保存参数 | 0x67 0x00 0x00 0x00 0x00 0x00 0x15 0x76 |

> 浮点数按 IEEE-754 编码，大端发送。

---

## 6. 在线升级 OTA

> ⚠️ OTA 有一定风险，请在供电、通信稳定的情况下使用。请严格按以下步骤执行。

### 步骤 1：Jump（主机 → 电机）

标识符 `0x600+Motor ID`，DLC 8：

```
0x67 0x04 0xA0 0xA1 0xA2 0xA3 0x00 0x76
```

### 步骤 2：电机回复 "Jump完毕"

标识符 `Motor ID`，DLC 8：

```
0x8A 0xA0 0x00 0x00 0x00 0x00 0x00 0xA8
```

### 步骤 3：预备发送（主机 → 电机）

标识符 `0x500+Motor ID`，DLC 8：

```
0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xEF
```

### 步骤 4：电机回复 "可以发送"

- 开辟存储区成功，标识符 `Motor ID`：

```
0x8A 0xA2 0x00 0x00 0x00 0x00 0x00 0xA8
```

- 开辟存储区失败，标识符 `Motor ID`（可重新发送预备发送指令重试）：

```
0x8A 0xA1 0x00 0x00 0x00 0x00 0x00 0xA8
```

### 步骤 5：发送新固件

标识符 `0x400+Motor ID`，DLC 64，标准帧，数据为固件内容。按顺序发送，**建议每帧之间间隔 10ms 防止丢包**。

### 步骤 6：SUM 校验（主机 → 电机）

发送完固件后发送 SUM 校验报文。标识符 `0x500+Motor ID`，DLC 8：

| Byte | 定义 |
|---|---|
| Byte0 | 0x00 |
| Byte1 | 0x00 |
| Byte2 | SUM 校验值（高字节在前）。例如校验和为 0x12345678，则 Byte2=0x12, Byte3=0x34, Byte4=0x56, Byte5=0x78 |
| Byte3 | |
| Byte4 | |
| Byte5 | |
| Byte6 | 0x00 |
| Byte7 | 0xFE |

### 步骤 7：电机回复 "校验结果"

标识符 `Motor ID`，DLC 4：

- 校验通过：Byte0\~3 = SUM 校验和
- 校验不通过：Byte0\~3 = 全 0xFF（此时程序停留在 boot，需检查校验和、检查连线并重新执行步骤 3\~5）

### 步骤 8：OTA 完成

校验通过后电机会开始 Flash 擦写，**此时务必确保供电稳定，且不要主动发送 CAN 报文**。擦写完成后电机回复（标识符 `Motor ID`，DLC 8）：

```
0x8A 0xA3 0x00 0x00 0x00 0x00 0x00 0xA8
```

随后电机回到失能状态，可下电重启，正式完成 OTA 流程。

---

## 7. 反馈帧解析 Python 代码（官方示例）

```python
def parse_torque_12bit(byte6_str, byte7_str, t_min, t_max):
    """解析电机实际扭矩反馈值（12bits CAN 数据）
    Byte6[0-3]（高4位）+ Byte7[0-7]（低8位）组合为 12bits 原始值，
    再线性映射为物理单位扭矩（Nm）。"""
    if pd.isna(byte6_str) or pd.isna(byte7_str):
        return None
    try:
        b6 = int(byte6_str, 16)
        b7 = int(byte7_str, 16)
    except (ValueError, TypeError):
        return None
    raw_12 = ((b6 & 0x0F) << 8) | b7
    torque = t_min + (raw_12 / 4096.0) * (t_max - t_min)
    return torque

def parse_position_16bit(data0_str, data1_str, p_min, p_max):
    """解析电机实际位置反馈值（16bits CAN 数据）
    Byte0（高8位）+ Byte1（低8位）组合为 16bits 原始值，
    再线性映射为物理单位位置（rad）。"""
    if pd.isna(data0_str) or pd.isna(data1_str):
        return None
    try:
        high_byte = int(data0_str, 16)
        low_byte = int(data1_str, 16)
    except (ValueError, TypeError):
        return None
    position_raw = (high_byte << 8) | low_byte
    position = p_min + (position_raw / 65536.0) * (p_max - p_min)
    return position

def parse_velocity_12bit(byte2_str, byte3_str, v_min, v_max):
    """解析电机实际速度反馈值（12bits CAN 数据）
    Byte2[0-7]（高8位）+ Byte3[4-7]（低4位）组合为 12bits 原始值，
    再线性映射为物理单位速度（rad/s）。"""
    if pd.isna(byte2_str) or pd.isna(byte3_str):
        return None
    try:
        b2 = int(byte2_str, 16)
        b3 = int(byte3_str, 16)
    except (ValueError, TypeError):
        return None
    raw_12 = ((b2 & 0xFF) << 4) | ((b3 >> 4) & 0x0F)
    velocity = v_min + (raw_12 / 4096.0) * (v_max - v_min)
    return velocity
```

---

## 8. 常用参数与限制速查

- MIT 模式封包默认通讯范围：θ = ±12.5 rad，V = ±10.0 rad/s，Kp = 0\~250，Kd = 0\~50，T = ±50 Nm（对应参数 Index 24\~35）。
- 电角度校准（Electric Angle Offset）、输出轴编码器校准为出厂时完成，用户通常无需操作；**校准失败或错误波形时电机绝不能继续使用，否则会烧毁或跑飞**。
- 修改 Rotate Dir（Index 47）后必须重新进行电角度校准和输出轴编码器校准。
- 驱动板最大输入电压 60V，推荐工作电压 48V，峰值相电流 60A，编码器位数 14 位，最大工作温度 50℃。
