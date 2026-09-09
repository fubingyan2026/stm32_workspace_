# -*- coding: utf-8 -*-
"""E1 CTU 电源板 RS485 协议编解码（E1_MASTER_POWER_CTU / E1_SLAVER_POWER_CTU 合并版）

两兄弟板共用同一帧头/信封（避免帧头差异导致解析失败），设备用 payload 首字节 ID 区分/定向：
  [z][cmd][data_len][payload][crc][\n]
  - 帧头 0x7A ('z')
  - cmd 1B（应答帧 cmd = 0x80 | 下行命令码）
  - data_len 1B
  - payload data_len B；下行 payload[0]=目标设备 ID，上行 payload[0]=源设备 ID
  - crc 1B (CRC8, 多项式 0x31, 初值 0xFF, 覆盖 帧头~payload)
  - 帧尾 0x0A ('\n')
帧总长 = data_len + 5；多字节小端。

命令码与 E1_SLAVER 统一（两板 0x01 状态 / 0x02 电压 / 0x03 温度 / 0x04 控制 /
0x05 清除故障锁存 / 0x06 升级预留），仅负载数据段按板不同（mst_/slv_ 前缀解码）。
协议见 E1_MASTER_POWER_CTU/docs/protocol_master_485.md 与
E1_SLAVER_POWER_CTU/docs/protocol_slaver_485.md。
"""

from __future__ import annotations

# ---- 设备 ID（E1_MASTER=0x01 / E1_SLAVER=0x02，经 payload 首字节定向/标识）----
DEV_ADDR_MASTER = 0x01
DEV_ADDR_SLAVER = 0x02

DEV_NAMES = {
    DEV_ADDR_MASTER: "E1_MASTER (0x01)",
    DEV_ADDR_SLAVER: "E1_SLAVER (0x02)",
}

# ---- 命令字（主机 -> 板，两板统一）----
CMD_READ_STATUS = 0x01
CMD_READ_VOLT = 0x02
CMD_READ_TEMP = 0x03
CMD_CTRL = 0x04
CMD_RESET_LATCH = 0x05
CMD_UPGRADE = 0x06

CMD_REPLY_FLAG = 0x80
CMD_ERR = 0x7F

CMD_NAME = {
    CMD_READ_STATUS: "读系统状态",
    CMD_READ_VOLT: "读电压",
    CMD_READ_TEMP: "读温度",
    CMD_CTRL: "控制",
    CMD_RESET_LATCH: "清除故障锁存",
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
    """按协议打包一帧（帧头统一无地址；设备 ID 已含在 payload 首字节）"""
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


def parse_addr(frame: bytes) -> int:
    """帧内设备 ID（= payload 首字节；下行=目标 ID，上行=源 ID）"""
    if len(frame) < 5:
        return 0
    return frame[3]


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


# =============================================================================
# E1_MASTER 数据段（0x81 状态 / 0x82 电压 / 0x83 温度）
# =============================================================================

MST_STATUS_KEYS = ("estop", "r12", "r24", "rvin", "raux", "rmotor",
                   "fan0", "fan1", "ntc1", "ntc2")


def decode_mst_status(payload: bytes) -> dict[str, bool]:
    """0x81 主控板状态位域 → {键: bool}（true=有效：急停/异常/断开）

    byte0: estop | r12 | r24 | rvin | raux | rmotor
    byte1: fan0 | fan1 | ntc1 | ntc2
    """
    items = {k: False for k in MST_STATUS_KEYS}
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


def decode_mst_volt(payload: bytes) -> dict[str, int]:
    """0x82 主控板电压应答 → {vin_mv, vin_dcdc_mv}"""
    if len(payload) < 4:
        return {"vin_mv": 0, "vin_dcdc_mv": 0}
    return {"vin_mv": unpack_u16_le(payload, 0),
            "vin_dcdc_mv": unpack_u16_le(payload, 2)}


def decode_mst_temp(payload: bytes) -> dict[str, float]:
    """0x83 主控板温度应答 → {ntc1_c, ntc2_c, mcu_c} (°C)"""
    if len(payload) < 6:
        return {"ntc1_c": 0.0, "ntc2_c": 0.0, "mcu_c": 0.0}
    return {"ntc1_c": unpack_i16_le(payload, 0) / 100.0,
            "ntc2_c": unpack_i16_le(payload, 2) / 100.0,
            "mcu_c": unpack_i16_le(payload, 4) / 100.0}


# =============================================================================
# E1_SLAVER 数据段（0x81 状态 / 0x82 电压 / 0x83 温度+VDDA）
# =============================================================================

SLV_STATUS_KEYS = ("err_24v", "err_12v", "err_aux", "err_motor", "err_lsd1",
                   "err_lsd2", "out_24v", "out_12v", "out_lsd1", "out_lsd2",
                   "latch_active")


def decode_slv_status(payload: bytes) -> dict[str, bool]:
    """0x81 副板状态位域 → {键: bool}

    byte0: err_24v | err_12v | err_aux | err_motor | err_lsd1 | err_lsd2
    byte1: out_24v | out_12v | out_lsd1 | out_lsd2 | latch_active
    """
    items = {k: False for k in SLV_STATUS_KEYS}
    if len(payload) < 2:
        return items
    b0, b1 = payload[0], payload[1]
    fault_keys = ("err_24v", "err_12v", "err_aux", "err_motor",
                  "err_lsd1", "err_lsd2")
    out_keys = ("out_24v", "out_12v", "out_lsd1", "out_lsd2", "latch_active")
    for i, key in enumerate(fault_keys):
        items[key] = bool(b0 & (1 << i))
    for i, key in enumerate(out_keys):
        items[key] = bool(b1 & (1 << i))
    return items


def decode_slv_volt(payload: bytes) -> dict[str, int]:
    """0x82 副板电压应答 → {aux_mv, motor_mv, lsd1_mv, lsd2_mv}"""
    if len(payload) < 8:
        return {"aux_mv": 0, "motor_mv": 0, "lsd1_mv": 0, "lsd2_mv": 0}
    return {"aux_mv": unpack_u16_le(payload, 0),
            "motor_mv": unpack_u16_le(payload, 2),
            "lsd1_mv": unpack_u16_le(payload, 4),
            "lsd2_mv": unpack_u16_le(payload, 6)}


def decode_slv_temp(payload: bytes) -> dict[str, float | int]:
    """0x83 副板温度/VDDA 应答 → {mcu_c, vdda_mv}"""
    if len(payload) < 4:
        return {"mcu_c": 0.0, "vdda_mv": 0}
    return {"mcu_c": unpack_i16_le(payload, 0) / 100.0,
            "vdda_mv": unpack_u16_le(payload, 2)}


# =============================================================================
# 构建下行帧（帧头统一；payload[0]=目标设备 ID）
# =============================================================================

def build_read_status(addr: int = DEV_ADDR_MASTER) -> bytes:
    """0x01 读系统状态（目标 addr，payload=[id]）"""
    return build_frame(CMD_READ_STATUS, bytes([addr]))


def build_read_volt(addr: int = DEV_ADDR_MASTER) -> bytes:
    """0x02 读电压（目标 addr，payload=[id]）"""
    return build_frame(CMD_READ_VOLT, bytes([addr]))


def build_read_temp(addr: int = DEV_ADDR_MASTER) -> bytes:
    """0x03 读温度（目标 addr，payload=[id]）"""
    return build_frame(CMD_READ_TEMP, bytes([addr]))


def build_mst_ctrl(buzzer_duty: int, addr: int = DEV_ADDR_MASTER) -> bytes:
    """0x04 主控板控制帧：payload=[id, buzzer_duty 0-50%（超限截断）]"""
    duty = min(max(int(buzzer_duty), 0), 50)
    return build_frame(CMD_CTRL, bytes([addr, duty]))


def build_slv_ctrl(output_mask: int, fill_duty: int,
                   addr: int = DEV_ADDR_SLAVER) -> bytes:
    """0x04 副板输出控制帧：payload=[id, 输出位域, 预留, 补光亮度 uint16 LE]"""
    mask = int(output_mask) & 0x0F
    duty = min(max(int(fill_duty), 0), 1000)
    return build_frame(CMD_CTRL,
                       bytes([addr, mask, 0x00, duty & 0xFF, (duty >> 8) & 0xFF]))


def build_reset_latch(addr: int = DEV_ADDR_MASTER) -> bytes:
    """0x05 清除故障锁存：payload=[id, magic 0x01]（两板通用）"""
    return build_frame(CMD_RESET_LATCH, bytes([addr, 0x01]))


def frame_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)
