# -*- coding: utf-8 -*-
"""串口传输层：负责实际 I/O 的 Qt 工作线程。

- :class:`SerialWorker`   常驻收发线程，流式解析 z 帧并在读到的第一时间打时间戳。
- :class:`UpgradeWorker`  Boot 固件升级线程（独占串口，与轮询互斥）。

固件升级协议层直接复用 ``E1_CTU_BOOT/host/boot_protocol.py``（单一来源）。
"""

from __future__ import annotations

import time

import serial

from PyQt6.QtCore import QThread, pyqtSignal

from .firmware import BootProtoSender, request_boot
from .protocol import FrameParser


class SerialWorker(QThread):
    """常驻串口收发线程（0 超时轮询 + 流式帧解析）。"""

    connected = pyqtSignal(bool, str)
    frame_rx = pyqtSignal(bytes, float)  # (帧, 接收时间戳 perf_counter)

    def __init__(self, port: str, baud: int, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._parser = FrameParser()
        self._running = False

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self._port, self._baud,
                                         timeout=0, write_timeout=0.5)
        except Exception as exc:  # noqa: BLE001
            self.connected.emit(False, f"连接失败: {exc}")
            return

        self.connected.emit(True, f"已连接 {self._port} @ {self._baud}")
        self._running = True

        try:
            while self._running:
                try:
                    pending = self._serial.in_waiting
                except Exception as exc:  # noqa: BLE001
                    self.connected.emit(False, f"串口错误: {exc}")
                    break
                if pending > 0:
                    data = self._serial.read(pending)
                    # 在底层读到数据的第一时间打时间戳，消除 Qt 跨线程分发延迟
                    rx_ts = time.perf_counter()
                    for frame in self._parser.feed(data):
                        self.frame_rx.emit(frame, rx_ts)
                    continue  # 有数据不休眠，尽快吞完
                time.sleep(0.0005)  # 空闲时轻微让出 CPU
        finally:
            self._close()

    def send(self, data: bytes) -> bool:
        if self._serial is None:
            return False
        try:
            self._serial.write(data)
            return True
        except Exception as exc:  # noqa: BLE001
            self.connected.emit(False, f"发送失败: {exc}")
            return False

    def _close(self) -> None:
        try:
            if self._serial:
                self._serial.close()
        except Exception:  # noqa: BLE001
            pass
        self._serial = None


class UpgradeWorker(QThread):
    """Boot 寻址分块升级：0x06 邀请 → SELECT/START/DATA/END。

    与轮询互斥：调用前须停止 :class:`SerialWorker` 释放串口，本线程独占串口；
    完成后再由上层重新连接并恢复轮询，避免半双工总线上帧交错冲突。
    """

    log_line = pyqtSignal(str, str)   # (文本, 级别)
    progress = pyqtSignal(int)        # 0..100
    phase = pyqtSignal(str)
    done = pyqtSignal(bool, str)      # (成功, 描述)

    def __init__(self, port: str, baud: int, device: str,
                 filepath: str, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._device = device
        self._filepath = filepath
        self._serial: serial.Serial | None = None
        self._sender: BootProtoSender | None = None

    def cancel(self) -> None:
        if self._sender:
            self._sender.cancel()

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self._port, self._baud,
                                         timeout=0, write_timeout=0.5)
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"升级串口打开失败: {exc}", "error")
            self.done.emit(False, f"串口打开失败: {exc}")
            return

        ok = False
        try:
            # 统一先发一帧 0x06：App→复位进 Boot；已在 Boot→SELECT（幂等）
            self.phase.emit("发送 0x06 邀请 / 等待 Boot")
            self.log_line.emit("发送 0x06 邀请（App 复位进 Boot / Boot 选中）", "info")
            if not request_boot(self._serial, self._device,
                                lambda t, l: self.log_line.emit(t, l)):
                self.done.emit(False, "0x06 邀请发送失败")
                return
            time.sleep(0.5)

            self._sender = BootProtoSender(
                self._serial,
                log_cb=lambda t, l: self.log_line.emit(t, l),
                phase_cb=lambda t: self.phase.emit(t),
                progress_cb=lambda f: self.progress.emit(int(f * 100)))
            ok = self._sender.transfer(self._device, self._filepath)
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"升级异常: {exc}", "error")
        finally:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None
            self._sender = None

        self.done.emit(ok, "升级完成，板端复位运行新固件" if ok else "升级失败")
