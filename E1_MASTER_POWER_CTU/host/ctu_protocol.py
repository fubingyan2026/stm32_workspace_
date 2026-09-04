# -*- coding: utf-8 -*-
"""CTU 主电源板 RS485 协议编解码 — 与 E1_MASTER_POWER_CTU/docs/protocol_master_485.md 对齐

帧格式: [z][cmd][data_len][payload][crc][\n]
  - 帧头 0x7A ('z')
  - cmd 1B（应答帧 cmd = 0x80 | 下行命令码）
  - data_len 1B
  - payload data_len B
  - crc 1B (CRC8, 多项式 0x31, 初值 0xFF, 覆盖 帧头~payload)
  - 帧尾 0x0A ('\n')
帧总长 = data_len + 5；多字节小端。
"""

from __future__ import annotations

# ---- 命令字（主机 -> 板）----
CMD_READ_STATUS = 0x01
CMD_READ_TEMP = 0x02
CMD_READ_VOLT = 0x03
CMD_CTRL = 0x10
CMD_UPGRADE = 0x11

CMD_REPLY_FLAG = 0x80
CMD_ERR = 0x7F

CMD_NAME = {
    CMD_READ_STATUS: "读系统状态",
    CMD_READ_TEMP: "读温度",
    CMD_READ_VOLT: "读电压",
    CMD_CTRL: "控制",
    CMD_UPGRADE: "升级请求",
}

# ---- 错误码（0x7F 应答 payload[0]）----
ERR_TEXT = {
    0x00: "无错误",
    0x01: "未知命令",
    0x02: "功能暂不支持",
    0x03: "帧长度与命令不匹配",
}

# ---- CRC8（与固件 m_middlewares crc.c 一致）----
_CRC8_TABLE: list[int] = []
for _i in range(256):
    _crc = _i
    for _ in range(8):
        if _crc & 0x01:
            _crc = ((_crc >> 1) ^ 0x8C) & 0xFF
        else:
            _crc = (_crc >> 1) & 0xFF
    _CRC8_TABLE.append(_crc)


def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """按协议打包一帧"""
    if not 0 <= len(payload) <= 255:
        raise ValueError("payload 超长")
    frame = bytes([0x7A, cmd & 0xFF, len(payload) & 0xFF]) + payload
    return frame + bytes([crc8(frame), 0x0A])


class FrameParser:
    """流式帧解析：喂入字节流，吐出完整合法帧"""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        frames: list[bytes] = []
        while True:
            idx = self._buf.find(0x7A)
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 4:
                break
            plen = self._buf[2]
            total = plen + 5
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            if frame[-1] == 0x0A and crc8(frame[:-2]) == frame[-2]:
                frames.append(frame)
        return frames


def parse_cmd(frame: bytes) -> int:
    return frame[1]


def parse_payload(frame: bytes) -> bytes:
    plen = frame[2]
    return frame[3:3 + plen]


def unpack_i16_le(data: bytes, off: int) -> int:
    v = data[off] | (data[off + 1] << 8)
    return v - 0x10000 if v >= 0x8000 else v


def unpack_u16_le(data: bytes, off: int) -> int:
    return data[off] | (data[off + 1] << 8)


# ---- 状态帧解码（0x81 payload 2B）----
RAIL_ERR_NAMES = [
    "12V 异常",        # bit1
    "24V 异常",        # bit2
    "VIN_DC-DC 异常",  # bit3
    "AUX 异常",        # bit4
    "MOTOR 异常",      # bit5
]
AUX_ERR_NAMES = [
    "风扇0 异常",  # byte1 bit0
    "风扇1 异常",  # bit1
    "NTC1 断开",   # bit2
    "NTC2 断开",   # bit3
]


def decode_status(payload: bytes) -> tuple[bool, str]:
    """返回 (急停有效?, 错误描述字符串)。payload 长度不足时返回占位。"""
    if len(payload) < 2:
        return False, "数据不足"
    b0, b1 = payload[0], payload[1]
    estop = bool(b0 & 0x01)
    errors: list[str] = []
    for i, name in enumerate(RAIL_ERR_NAMES):
        if b0 & (1 << (i + 1)):
            errors.append(name)
    for i, name in enumerate(AUX_ERR_NAMES):
        if b1 & (1 << i):
            errors.append(name)
    text = "、".join(errors) if errors else ("急停触发" if estop else "正常")
    return estop, text


STATUS_KEYS = ("estop", "r12", "r24", "rvin", "raux", "rmotor",
               "fan0", "fan1", "ntc1", "ntc2")


def status_items(payload: bytes) -> dict[str, bool]:
    """逐位解析状态帧：每个键 true=有效(急停/异常/断开)

    键含义: estop 急停有效; r12/r24/rvin/raux/rmotor 电源轨异常;
            fan0/fan1 风扇异常; ntc1/ntc2 NTC 未连接。
    """
    items = {k: False for k in STATUS_KEYS}
    if len(payload) < 2:
        return items
    b0, b1 = payload[0], payload[1]
    items["estop"] = bool(b0 & 0x01)
    items["r12"] = bool(b0 & 0x02)
    items["r24"] = bool(b0 & 0x04)
    items["rvin"] = bool(b0 & 0x08)
    items["raux"] = bool(b0 & 0x10)
    items["rmotor"] = bool(b0 & 0x20)
    items["fan0"] = bool(b1 & 0x01)
    items["fan1"] = bool(b1 & 0x02)
    items["ntc1"] = bool(b1 & 0x04)
    items["ntc2"] = bool(b1 & 0x08)
    return items


def decode_temp(payload: bytes) -> str:
    """0x82 温度应答：ntc1/ntc2/mcu int16 LE ×100℃"""
    if len(payload) < 6:
        return "数据不足"
    vals = [
        unpack_i16_le(payload, 0) / 100.0,
        unpack_i16_le(payload, 2) / 100.0,
        unpack_i16_le(payload, 4) / 100.0,
    ]
    return (f"NTC1={vals[0]:.2f}°C  NTC2={vals[1]:.2f}°C  "
            f"MCU={vals[2]:.2f}°C")


def decode_volt(payload: bytes) -> str:
    """0x83 电压应答：vin/vin_dcdc uint16 LE mV"""
    if len(payload) < 4:
        return "数据不足"
    vin = unpack_u16_le(payload, 0)
    vin_dcdc = unpack_u16_le(payload, 2)
    return f"VIN={vin}mV  VIN_DC-DC={vin_dcdc}mV"


def build_read_status() -> bytes:
    return build_frame(CMD_READ_STATUS)


def build_read_temp() -> bytes:
    return build_frame(CMD_READ_TEMP)


def build_read_volt() -> bytes:
    return build_frame(CMD_READ_VOLT)


def build_ctrl(buzzer_duty: int) -> bytes:
    """构建 0x10 控制帧；buzzer_duty 为导通占空比 0-50（上限 50%）"""
    duty = min(max(int(buzzer_duty), 0), 50)
    return build_frame(CMD_CTRL, bytes([duty]))


def frame_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)
