# -*- coding: utf-8 -*-
"""G0 串口协议编解码 — 与 docs/G0_Hand 串口通信协议规范.md 对齐

帧格式: [z][cmd][data_len][payload][crc][\n]
  - 帧头 0x7A ('z')
  - cmd 1B
  - data_len 1B
  - payload data_len B
  - crc 1B (CRC8, 多项式 0x31, 初值 0xFF, 覆盖 帧头~payload)
  - 帧尾 0x0A ('\n')
帧总长 = data_len + 5
"""

from __future__ import annotations

import struct

# ---- 命令字 ----
CMD_HEARTBEAT = 0x00            # G0 -> 上位机  心跳上报 payload=2B tick
CMD_KEY_EVENT = 0x01            # G0 -> 上位机  按键事件 payload=2B [key_index][event]
CMD_MOTOR_SET_TARGET = 0x02     # 上位机 -> G0  电机目标 payload=20B 5xfloat LE
CMD_MOTOR_REQ_FEEDBACK = 0x03   # 上位机 -> G0  请求反馈 payload=0
CMD_MOTOR_FEEDBACK_REPORT = 0x04  # G0 -> 上位机 反馈上报 payload=21B [state][5xfloat]

# ---- 电机目标量程 ----
POS_MIN, POS_MAX = 0.0, 0.464
VEL_MIN, VEL_MAX = -30.0, 30.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
TOR_MIN, TOR_MAX = -0.1, 0.1

# ---- 电机状态含义（低 4 位）----
MOTOR_STATE_TEXT = {
    0x0: "失能",
    0x1: "使能",
    0x2: "电机侧未识别",
    0x3: "输出轴未识别",
    0x5: "读取编码器错误",
    0x7: "读取编码器错误",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失",
    0xE: "过载",
}

# ---- 按键事件含义 ----
KEY_EVENT_TEXT = {
    0x00: "按下",
    0x01: "松开",
    0x02: "空闲按下",
    0x03: "短按松开",
    0x04: "点击",
    0x05: "单击",
    0x06: "双连击",
    0x07: "三连击",
    0x08: "重复点击",
    0x09: "长按",
    0x0A: "长按松开",
}

KEY_NAME = {0: "KEY1", 1: "KEY2"}

# ---- CRC8 (反射多项式, LSB-first; 多项式 0x31 的反射值 = 0x8C, 初值 0xFF) ----
# 与固件 public_layer/m_middlewares/algorithm/crc.c 的 CRC8_table 一致:
#   crc = table[crc ^ byte]，初值 0xFF，无最终异或。
_CRC8_TABLE = []
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
    frame = bytes([0x7A, cmd & 0xFF, len(payload) & 0xFF]) + payload
    frame += bytes([crc8(frame), 0x0A])
    return frame


class FrameParser:
    """流式帧解析：喂入字节流，吐出完整合法帧"""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        frames = []
        while True:
            # 找帧头
            idx = self._buf.find(0x7A)
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 4:  # 头+cmd+len 已足够判断长度
                break
            plen = self._buf[2]
            total = plen + 5
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            # 校验帧尾 + CRC
            if frame[-1] == 0x0A and crc8(frame[:-2]) == frame[-2]:
                frames.append(frame)
        return frames


def parse_cmd(frame: bytes) -> int:
    return frame[1]


def unpack_float_le(data: bytes) -> float:
    return struct.unpack("<f", data)[0]


def build_motor_target(pos: float, vel: float, kp: float, kd: float, tor: float) -> bytes:
    """构建 0x02 电机目标设定帧（各参数已限幅）"""
    pos = min(max(pos, POS_MIN), POS_MAX)
    vel = min(max(vel, VEL_MIN), VEL_MAX)
    kp = min(max(kp, KP_MIN), KP_MAX)
    kd = min(max(kd, KD_MIN), KD_MAX)
    tor = min(max(tor, TOR_MIN), TOR_MAX)
    payload = struct.pack("<5f", pos, vel, kp, kd, tor)
    return build_frame(CMD_MOTOR_SET_TARGET, payload)


def build_req_feedback() -> bytes:
    """构建 0x03 请求反馈帧"""
    return build_frame(CMD_MOTOR_REQ_FEEDBACK)
