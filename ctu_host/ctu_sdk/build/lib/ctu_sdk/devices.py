# -*- coding: utf-8 -*-
"""设备标识：把友好的设备名映射到应用协议地址与 Boot 设备 ID。"""

from __future__ import annotations

from .firmware import DEVICE_MASTER as BOOT_DEVICE_MASTER
from .firmware import DEVICE_SLAVER as BOOT_DEVICE_SLAVER
from .protocol import DEV_ADDR_MASTER, DEV_ADDR_SLAVER

#: 应用协议地址（帧 payload 首字节）
DEVICE_ADDR = {
    "master": DEV_ADDR_MASTER,
    "slaver": DEV_ADDR_SLAVER,
}

#: Boot 协议设备 ID（boot_protocol 使用的字符串）
DEVICE_BOOT_ID = {
    "master": BOOT_DEVICE_MASTER,
    "slaver": BOOT_DEVICE_SLAVER,
}

MASTER = "master"
SLAVER = "slaver"

ALL_DEVICES = (MASTER, SLAVER)


def normalize_device(device: object) -> str:
    """把多种写法归一化为 ``"master"`` / ``"slaver"``。

    接受 ``"master"``/``"slaver"``、``"0x01"``/``"0x02"``、``1``/``2``、
    或设备中文别名（``"主"``/``"副"``）。
    """
    if isinstance(device, bool):  # bool 是 int 子类，先挡掉
        raise ValueError(f"非法设备: {device!r}")
    if isinstance(device, int):
        for name, addr in DEVICE_ADDR.items():
            if addr == device:
                return name
        raise ValueError(f"非法设备地址: {device}")
    if isinstance(device, str):
        text = device.strip().lower()
        if text in DEVICE_ADDR:
            return text
        if text in ("0x01", "01", "1", "主", "主控", "master板"):
            return MASTER
        if text in ("0x02", "02", "2", "副", "副板", "slaver板"):
            return SLAVER
    raise ValueError(f"非法设备: {device!r}")


def device_addr(device: object) -> int:
    """应用协议地址（0x01 / 0x02）。"""
    return DEVICE_ADDR[normalize_device(device)]


def boot_device_id(device: object) -> str:
    """Boot 协议设备 ID（``"master"`` / ``"slaver"``）。"""
    return DEVICE_BOOT_ID[normalize_device(device)]
