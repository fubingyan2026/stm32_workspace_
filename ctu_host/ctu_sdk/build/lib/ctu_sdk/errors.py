# -*- coding: utf-8 -*-
"""SDK 异常类型：所有可预期的失败都派生自 :class:`CtuError`。"""

from __future__ import annotations

from .protocol import CMD_NAME, ERR_TEXT


class CtuError(Exception):
    """SDK 基础异常。"""


class CtuNotConnected(CtuError):
    """串口未打开就发起了请求。"""


class CtuTimeout(CtuError):
    """在超时时间内没有收到符合预期的应答。"""


class CtuFramingError(CtuError):
    """收到的数据无法构成合法帧（帧头/帧尾/CRC 校验失败）。"""


class CtuDeviceError(CtuError):
    """设备返回 0x7F 错误应答。

    @ivar device: 设备名（master/slaver）
    @ivar code:   错误码（参见 ``ERR_TEXT``）
    """

    def __init__(self, device: str, code: int) -> None:
        self.device = device
        self.code = code
        self.text = ERR_TEXT.get(code, f"未知错误码 0x{code:02X}")
        super().__init__(f"{device} 错误应答 err=0x{code:02X} ({self.text})")


class CtuUnexpectedReply(CtuError):
    """收到的应答命令码与请求不匹配。"""

    def __init__(self, expected_cmd: int, actual_cmd: int) -> None:
        self.expected_cmd = expected_cmd
        self.actual_cmd = actual_cmd
        name = CMD_NAME.get(expected_cmd, f"0x{expected_cmd:02X}")
        super().__init__(
            f"应答命令不符：期望 {name}(0x{expected_cmd:02X})，"
            f"实际 0x{actual_cmd:02X}")


class FirmwareImageError(CtuError):
    """固件镜像未通过预检（非 AppA 链接镜像、超容量、读取失败等）。"""


class UpgradeError(CtuError):
    """固件升级过程中失败（SELECT/START/DATA/END 未被设备确认）。"""
