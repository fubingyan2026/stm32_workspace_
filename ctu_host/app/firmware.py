# -*- coding: utf-8 -*-
"""固件升级协议适配：复用 ``E1_CTU_BOOT/host/boot_protocol.py``（单一来源）。

Boot 工程自带完整的 z 帧寻址分块升级实现，此处只负责把它加入 import 路径
并再导出，向上层（transport / ui）提供统一的固件检查入口。
"""

from __future__ import annotations

import sys
from pathlib import Path

_BOOT_HOST_DIR = Path(__file__).resolve().parents[2] / "E1_CTU_BOOT" / "host"
if str(_BOOT_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_BOOT_HOST_DIR))

from boot_protocol import (  # noqa: E402
    DATA_MAX,
    DEVICE_BOOT,
    DEVICE_MASTER,
    DEVICE_NAMES,
    DEVICE_SLAVER,
    MAX_FW_SIZE,
    BootProtoSender,
    check_app_image,
    request_boot,
)

__all__ = [
    "DATA_MAX", "DEVICE_BOOT", "DEVICE_MASTER", "DEVICE_NAMES",
    "DEVICE_SLAVER", "MAX_FW_SIZE", "BootProtoSender", "check_app_image",
    "inspect_firmware", "request_boot",
]


def inspect_firmware(path: str) -> tuple[bool, int, str | None]:
    """读取并预检固件文件。

    @return ``(是否通过, 字节数, 失败原因或 None)``
    """
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError as exc:
        return False, 0, f"读取失败: {exc}"

    size = len(data)
    if size > MAX_FW_SIZE:
        return False, size, f"超出 App 分区容量 96KB（当前 {size}B）"
    reason = check_app_image(data)
    return reason is None, size, reason
