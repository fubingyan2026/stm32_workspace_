# -*- coding: utf-8 -*-
"""固件升级协议适配：复用 ``E1_CTU_BOOT/host/boot_protocol.py``（单一来源）。

Boot 工程自带完整的 z 帧寻址分块升级实现，本模块只负责把它加入 import 路径并
再导出，同时提供统一的固件镜像检查入口。

定位 Boot 协议层的顺序：

1. 环境变量 ``CTU_BOOT_PROTOCOL_DIR`` 指定的目录；
2. 仓库内相对路径 ``<repo>/E1_CTU_BOOT/host``（开发 / WSL 场景）。

独立部署到 Linux 时，只需把 ``boot_protocol.py`` 放到任意目录并设置该环境变量，
或直接 ``pip install`` 一个提供 ``boot_protocol`` 的包。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _boot_host_dir() -> Path | None:
    """按优先级搜索存放 ``boot_protocol.py`` 的目录。"""
    env_dir = os.environ.get("CTU_BOOT_PROTOCOL_DIR")
    if env_dir:
        candidate = Path(env_dir).expanduser()
        if (candidate / "boot_protocol.py").is_file():
            return candidate
        return None

    # ctu_sdk/firmware.py -> parents[2] 即仓库根目录
    candidate = Path(__file__).resolve().parents[2] / "E1_CTU_BOOT" / "host"
    return candidate if (candidate / "boot_protocol.py").is_file() else None


def _ensure_boot_path() -> None:
    boot_dir = _boot_host_dir()
    if boot_dir is not None and str(boot_dir) not in sys.path:
        sys.path.insert(0, str(boot_dir))


_ensure_boot_path()

try:
    from boot_protocol import (  # noqa: E402
        CMD_ABORT,
        CMD_DATA,
        CMD_END,
        CMD_SELECT,
        CMD_START,
        DATA_MAX,
        DEVICE_BOOT,
        DEVICE_MASTER,
        DEVICE_NAMES,
        DEVICE_SLAVER,
        MAX_FW_SIZE,
        BootProtoSender,
        check_app_image,
        crc16_xmodem,
        request_boot,
    )
    from boot_protocol import ERR_NONE as BOOT_ERR_NONE  # noqa: E402
    from boot_protocol import ERR_TEXT as BOOT_ERR_TEXT  # noqa: E402
except ImportError as exc:  # pragma: no cover - 仅在缺少 Boot 协议层时触发
    raise ImportError(
        "未找到 boot_protocol（Boot 升级协议层）。请设置环境变量 "
        "CTU_BOOT_PROTOCOL_DIR 指向 E1_CTU_BOOT/host，或确认仓库结构完整。"
    ) from exc

__all__ = [
    # Boot 协议层导出
    "DATA_MAX", "DEVICE_BOOT", "DEVICE_MASTER", "DEVICE_NAMES",
    "DEVICE_SLAVER", "MAX_FW_SIZE", "BootProtoSender", "check_app_image",
    "request_boot", "crc16_xmodem",
    "CMD_SELECT", "CMD_START", "CMD_DATA", "CMD_END", "CMD_ABORT",
    "BOOT_ERR_NONE", "BOOT_ERR_TEXT",
    # 本模块
    "inspect_firmware", "boot_host_dir",
]


def boot_host_dir() -> str | None:
    """返回实际使用的 Boot 协议层目录（未找到时返回 ``None``）。"""
    found = _boot_host_dir()
    return None if found is None else str(found)


def inspect_firmware(path: str) -> tuple[bool, int, str | None]:
    """读取并预检固件镜像（向量表 sanity + 容量）。

    @param path  固件 ``.bin`` 路径
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
