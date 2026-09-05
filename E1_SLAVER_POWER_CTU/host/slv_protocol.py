# -*- coding: utf-8 -*-
"""E1_SLAVER_POWER_CTU RS485 协议编解码 — 与 docs/protocol_slaver_485.md 对齐

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

# ---- 命令字（主机 -> 从板）----
CMD_READ_STATUS = 0x01
CMD_READ_VOLT = 0x02
CMD_READ_TEMP = 0x03
CMD_CTRL = 0x10
CMD_RESET_LATCH = 0x11
CMD_UPGRADE = 0x1F

CMD_REPLY_FLAG = 0x80
CMD_ERR = 0x7F

CMD_NAME = {
    CMD_READ_STATUS: "读系统状态",
    CMD_READ_VOLT: "读电压",
    CMD_READ_TEMP: "读温度/VDDA",
    CMD_CTRL: "输出控制",
    CMD_RESET_LATCH: "清除故障锁存",
    CMD_UPGRADE: "升级请求",
}

# ---- 输出掩码位（0x10 控制帧 ctrl）----
MASK_24V = 0x01
MASK_12V = 0x02
MASK_LSD1 = 0x04
MASK_LSD2 = 0x08

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


# ---- 0x81 系统状态应答（2B 位域）----
# byte0: 故障位（1=异常）
FAULT_KEYS = ("err_24v", "err_12v", "err_aux", "err_motor", "err_lsd1", "err_lsd2")
# byte1: 输出/锁存位
OUT_KEYS = ("out_24v", "out_12v", "out_lsd1", "out_lsd2", "latch_active")

STATUS_KEYS = FAULT_KEYS + OUT_KEYS


def status_items(payload: bytes) -> dict[str, bool]:
    """逐位解析状态帧，返回 {位名: bool}（true = 异常/输出有效/锁存存在）"""
    items = {k: False for k in STATUS_KEYS}
    if len(payload) < 2:
        return items
    b0, b1 = payload[0], payload[1]
    for i, key in enumerate(FAULT_KEYS):
        items[key] = bool(b0 & (1 << i))
    for i, key in enumerate(OUT_KEYS):
        items[key] = bool(b1 & (1 << i))
    return items


def decode_status_text(payload: bytes) -> str:
    """0x81 应答 → 一句话描述（异常列表/正常）"""
    if len(payload) < 2:
        return "数据不足"
    items = status_items(payload)
    names = {
        "err_24v": "24V 异常",
        "err_12v": "12V_ISO 异常",
        "err_aux": "AUX 输入缺失",
        "err_motor": "MOTOR 输入缺失",
        "err_lsd1": "LSD1 异常",
        "err_lsd2": "LSD2 异常",
    }
    errors = [names[k] for k in FAULT_KEYS if items.get(k)]
    if not errors:
        return "正常"
    if items.get("latch_active"):
        errors.append("有锁存")
    return "、".join(errors)


# ---- 0x82 电压应答（8B）----
VOLT_KEYS = ("aux_mv", "motor_mv", "lsd1_mv", "lsd2_mv")


def decode_volt(payload: bytes) -> dict[str, int]:
    """0x82 → {aux/motor/lsd1/lsd2 mV}"""
    vals = {k: 0 for k in VOLT_KEYS}
    if len(payload) < 8:
        return vals
    for i, key in enumerate(VOLT_KEYS):
        vals[key] = unpack_u16_le(payload, i * 2)
    return vals


def decode_volt_text(payload: bytes) -> str:
    if len(payload) < 8:
        return "数据不足"
    v = decode_volt(payload)
    return (f"AUX={v['aux_mv']}mV  MOTOR={v['motor_mv']}mV  "
            f"LSD1={v['lsd1_mv']}mV  LSD2={v['lsd2_mv']}mV")


# ---- 0x83 温度/VDDA 应答（4B）----
def decode_temp(payload: bytes) -> dict[str, float | int]:
    """0x83 → {mcu_temp_c, vdda_mv}"""
    if len(payload) < 4:
        return {"mcu_temp_c": 0.0, "vdda_mv": 0}
    temp_c = unpack_i16_le(payload, 0) / 100.0
    vdda_mv = unpack_u16_le(payload, 2)
    return {"mcu_temp_c": temp_c, "vdda_mv": vdda_mv}


def decode_temp_text(payload: bytes) -> str:
    if len(payload) < 4:
        return "数据不足"
    d = decode_temp(payload)
    return f"MCU={d['mcu_temp_c']:.2f}°C  VDDA={d['vdda_mv']}mV"


# ---- 构建下行帧 ----
def build_read_status() -> bytes:
    return build_frame(CMD_READ_STATUS)


def build_read_volt() -> bytes:
    return build_frame(CMD_READ_VOLT)


def build_read_temp() -> bytes:
    return build_frame(CMD_READ_TEMP)


def build_ctrl(output_mask: int, fill_duty: int) -> bytes:
    """构建 0x10 输出控制帧；fill_duty 0..1000（uint16 LE）"""
    mask = int(output_mask) & 0x0F
    duty = min(max(int(fill_duty), 0), 1000)
    return build_frame(CMD_CTRL, bytes([mask, 0x00, duty & 0xFF, (duty >> 8) & 0xFF]))


def build_reset_latch() -> bytes:
    """构建 0x11 清除故障锁存命令（magic 0x01）"""
    return build_frame(CMD_RESET_LATCH, bytes([0x01]))


def frame_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)
