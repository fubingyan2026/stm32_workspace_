# -*- coding: utf-8 -*-
"""测试夹具：pty 虚拟串口 + 虚拟电源板 + 假固件构造。"""

from __future__ import annotations

import contextlib
import os
import pty
import struct
import unittest

from ..client import CtuClient
from ..devices import MASTER
from .fake_board import FakeCtuBoard


def make_firmware_image(size: int = 1000) -> bytes:
    """构造一个能通过预检的假固件（向量表指向 AppA，SP 落在 RAM）。"""
    header = struct.pack("<II", 0x20001000, 0x08008101)
    if size < len(header):
        size = len(header)
    return header + bytes(size - len(header))


def write_firmware(tmp_dir: str, size: int = 1000, tag: str = "fw") -> str:
    """把假固件写入临时文件，返回路径（由调用方负责清理）。"""
    path = os.path.join(tmp_dir, f"ctu_{tag}_{os.getpid()}_{size}.bin")
    with open(path, "wb") as handle:
        handle.write(make_firmware_image(size))
    return path


@contextlib.contextmanager
def virtual_board(device: str = MASTER, timeout: float = 0.3):
    """建立一个 pty 对：SDK 用 slave 端，虚拟板用 master 端。"""
    if os.name != "posix":  # pragma: no cover
        raise unittest.SkipTest("ctu_sdk 回环测试需要 POSIX pty")

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    os.close(slave_fd)
    stream = os.fdopen(master_fd, "r+b", buffering=0)
    board = FakeCtuBoard(stream, device=device)
    client = CtuClient(slave_name, baud=115200, timeout=timeout)
    board.start()
    try:
        yield board, client
    finally:
        client.close()
        board.close()


@contextlib.contextmanager
def virtual_port(device: str = MASTER):
    """只提供端口名 + 虚拟板，由调用方自行打开串口（用于验证 CLI 等）。"""
    if os.name != "posix":  # pragma: no cover
        raise unittest.SkipTest("ctu_sdk 回环测试需要 POSIX pty")

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    # 保留 slave_fd 打开：避免 master 端写入时因无对端而 EIO；
    # 该 fd 不参与读取，不会与调用方的读取抢数据。
    stream = os.fdopen(master_fd, "r+b", buffering=0)
    board = FakeCtuBoard(stream, device=device)
    board.start()
    try:
        yield board, slave_name
    finally:
        board.close()
        try:
            os.close(slave_fd)
        except OSError:
            pass
