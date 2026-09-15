# -*- coding: utf-8 -*-
"""串口传输层：pyserial 薄封装 + z 帧流式解析（纯 Python，无 Qt 依赖）。

设计为**同步阻塞**风格：调用方 ``write()`` 后按需 ``read_frame()`` 等待应答。
一次完整的事务由上层 :class:`~ctu_sdk.client.CtuClient` 持有 ``lock`` 完成，
因此多线程（例如 :class:`~ctu_sdk.poller.CtuPoller`）可以安全并发调用客户端。
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable

import serial

from .protocol import FrameParser, frame_hex

#: 日志回调签名：(文本, 级别)，级别取值 info/warn/error/tx/rx
LogCallback = Callable[[str, str], None]

_POLL_SLEEP_S = 0.002


class SerialTransport:
    """串口收发 + 帧解析。

    @ivar lock: 可重入锁；上层在一次“发送-等待应答”事务期间持有它，避免交错。
    """

    def __init__(self, port: str, baud: int = 115200,
                 write_timeout: float = 0.5,
                 logger: LogCallback | None = None) -> None:
        self._port = port
        self._baud = int(baud)
        self._write_timeout = write_timeout
        self._log = logger or (lambda _t, _l: None)
        self._serial: serial.Serial | None = None
        self._parser = FrameParser()
        self._frames: deque[bytes] = deque()
        self.lock = threading.RLock()

    # ------------------------------------------------------------------ 属性
    @property
    def port(self) -> str:
        return self._port

    @property
    def baud(self) -> int:
        return self._baud

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    @property
    def raw(self) -> serial.Serial:
        """底层 pyserial 句柄（供 Boot 升级 :class:`BootProtoSender` 复用）。"""
        if self._serial is None:
            raise serial.SerialException("串口未打开")
        return self._serial

    # ------------------------------------------------------------------ 打开
    def open(self) -> None:
        if self.is_open:
            return
        self._serial = serial.Serial(self._port, self._baud, timeout=0,
                                     write_timeout=self._write_timeout)
        self.reset_parser()
        self._log(f"已打开 {self._port} @ {self._baud}", "info")

    def close(self) -> None:
        with self.lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:  # noqa: BLE001
                    pass
                self._serial = None
            self.reset_parser()

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # ------------------------------------------------------------------ 收发
    def write(self, frame: bytes) -> None:
        """发送一帧（已含 CRC 与帧尾的完整帧）。"""
        if not self.is_open:
            raise serial.SerialException("串口未打开")
        with self.lock:
            self._serial.write(frame)  # type: ignore[union-attr]
            self._log(f"TX  {frame_hex(frame)}", "tx")

    def read_frame(self, timeout: float) -> bytes | None:
        """在 ``timeout`` 秒内等待一个合法帧；超时返回 ``None``。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            if self._frames:
                frame = self._frames.popleft()
                self._log(f"RX  {frame_hex(frame)}", "rx")
                return frame
            if not self.is_open:
                return None
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            if not self._pump():
                time.sleep(min(_POLL_SLEEP_S, remaining))

    def drain(self, timeout: float = 0.0) -> list[bytes]:
        """读取 ``timeout`` 秒内到达的所有帧（默认只取已缓冲的）。"""
        frames: list[bytes] = []
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            frame = self.read_frame(max(0.0, deadline - time.monotonic()))
            if frame is None:
                return frames
            frames.append(frame)

    def flush_input(self) -> None:
        """丢弃接收方向的滞留数据（半双工总线发新命令前清理）。"""
        if not self.is_open:
            return
        with self.lock:
            try:
                pending = self._serial.in_waiting  # type: ignore[union-attr]
                if pending:
                    self._serial.read(pending)  # type: ignore[union-attr]
            except Exception:  # noqa: BLE001
                pass
            self.reset_parser()

    def reset_parser(self) -> None:
        """清空帧解析缓冲与已解析队列。"""
        self._parser = FrameParser()
        self._frames.clear()

    # ------------------------------------------------------------------ 内部
    def _pump(self) -> bool:
        """把串口缓冲区里的字节喂给解析器；有数据返回 True。"""
        try:
            pending = self._serial.in_waiting  # type: ignore[union-attr]
        except Exception as exc:  # noqa: BLE001
            self._log(f"串口读取错误: {exc}", "error")
            return False
        if not pending:
            return False
        data = self._serial.read(pending)  # type: ignore[union-attr]
        if data:
            self._frames.extend(self._parser.feed(data))
        return True
