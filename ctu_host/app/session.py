# -*- coding: utf-8 -*-
"""会话层：串口连接生命周期 + 等间隔轮询调度 + 丢包统计。

本层不含任何界面代码，仅通过信号对外广播状态；界面（ui）订阅信号并调用
``connect_port`` / ``send`` / ``set_auto_poll`` 等方法完成交互。
"""

from __future__ import annotations

import time
from collections import deque
from typing import Callable

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal

from .protocol import (
    CMD_ERR,
    CMD_REPLY_FLAG,
    DEV_NAMES,
    frame_hex,
    parse_addr,
    parse_cmd,
)
from .transport import SerialWorker

#: 严格应答超时阈值（ms）
REPLY_TIMEOUT_MS = 10

#: 统计刷新周期（ms）
STATS_REFRESH_MS = 200

PollSupplier = Callable[[], list[bytes]]


class SerialSession(QObject):
    """串口会话：连接、发送、自动轮询与通信统计。"""

    connection_changed = pyqtSignal(bool, str)   # (已连接, 描述)
    auto_poll_changed = pyqtSignal(bool)         # 自动轮询开关实际状态
    frame_received = pyqtSignal(bytes, float)    # (帧, 接收时间戳)
    log_message = pyqtSignal(str, str)           # (文本, 级别)
    stats_changed = pyqtSignal(dict)             # 统计快照

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._worker: SerialWorker | None = None
        self._port = ""
        self._baud = 115200
        self._connected = False

        self._poll_suppliers: list[PollSupplier] = []
        self._auto_poll = False
        self._poll_interval_ms = 100
        self._seq_index = 0

        self._pending_id: int | None = None
        self._pending_expected: tuple[int, int] | None = None
        self._pending_tx_ts = 0.0
        self._id_counter = 0

        self._tx_count = 0
        self._loss_count = 0
        self._rx_total = 0
        self._rx_window: deque[float] = deque()
        self._last_rx_ts: dict[int, float] = {}

        #: 是否在日志中显示原始收发帧（由界面开关控制）
        self.log_frames = True

        self._poll_timer = QTimer(self)
        self._poll_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._poll_timer.timeout.connect(self._on_poll_step)

        self._loss_timer = QTimer(self)
        self._loss_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._loss_timer.setSingleShot(True)
        self._loss_timer.timeout.connect(self._on_loss_timeout)

        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._emit_stats)
        self._stats_timer.start(STATS_REFRESH_MS)

    # ------------------------------------------------------------------ 属性
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def auto_poll(self) -> bool:
        return self._auto_poll

    @property
    def poll_interval_ms(self) -> int:
        return self._poll_interval_ms

    @property
    def port(self) -> str:
        return self._port

    @property
    def baud(self) -> int:
        return self._baud

    def set_poll_suppliers(self, suppliers: list[PollSupplier]) -> None:
        """注册轮询请求提供者（返回本板待发帧列表，空列表=跳过）。"""
        self._poll_suppliers = list(suppliers)

    def frames_log_enabled(self) -> bool:
        """轮询期间不刷原始帧日志，避免刷屏。"""
        return self.log_frames and not self._auto_poll

    def seconds_since_rx(self, addr: int) -> float | None:
        ts = self._last_rx_ts.get(addr)
        return None if ts is None else time.perf_counter() - ts

    # ------------------------------------------------------------ 连接管理
    def connect_port(self, port: str, baud: int) -> None:
        if self._worker and self._worker.isRunning():
            return
        if not port:
            self.log_message.emit("请先选择串口", "warn")
            return
        self._port, self._baud = port, int(baud)
        worker = SerialWorker(port, int(baud))
        worker.connected.connect(self._on_worker_connected)
        worker.frame_rx.connect(self._on_worker_frame)
        self._worker = worker
        worker.start()

    def disconnect(self) -> None:
        if self._auto_poll:
            self.set_auto_poll(False)
        self._poll_timer.stop()
        self._loss_timer.stop()
        self._pending_id = None
        self._pending_expected = None

        worker, self._worker = self._worker, None
        if worker:
            try:
                worker.connected.disconnect()
                worker.frame_rx.disconnect()
            except (RuntimeError, TypeError):
                pass
            worker.stop()
            worker.wait(2000)

        if self._connected:
            self._connected = False
            self.connection_changed.emit(False, "已断开")

    def send(self, frame: bytes, tag: str = "") -> bool:
        worker = self._worker
        if worker is None or not worker.isRunning():
            self.log_message.emit("未连接串口", "warn")
            return False
        if self.frames_log_enabled():
            text = f"TX  {tag}: {frame_hex(frame)}" if tag else f"TX  {frame_hex(frame)}"
            self.log_message.emit(text, "tx")
        if not worker.send(frame):
            self.log_message.emit(f"发送失败: {tag or frame_hex(frame)}", "error")
            return False
        return True

    # ------------------------------------------------------------ 自动轮询
    def set_auto_poll(self, enabled: bool) -> None:
        if enabled and not self._connected:
            self.log_message.emit("未连接串口，无法开启轮询", "warn")
            self._auto_poll = False
            self.auto_poll_changed.emit(False)
            return

        self._auto_poll = enabled
        if enabled:
            self._seq_index = 0
            self._poll_timer.start(self._poll_interval_ms)
            self._on_poll_step()
        else:
            self._poll_timer.stop()
            self._loss_timer.stop()
            self._pending_id = None
            self._pending_expected = None
        self.auto_poll_changed.emit(enabled)

    def set_poll_interval(self, ms: int) -> None:
        self._poll_interval_ms = int(ms)
        if self._auto_poll and self._poll_timer.isActive():
            self._poll_timer.setInterval(int(ms))

    def _build_sequence(self) -> list[bytes]:
        """交错各板请求（M,S,M,S...），保证同一总线上的查询均匀分布。"""
        lists = [supplier() for supplier in self._poll_suppliers]
        sequence: list[bytes] = []
        width = max((len(reqs) for reqs in lists), default=0)
        for i in range(width):
            for reqs in lists:
                if i < len(reqs):
                    sequence.append(reqs[i])
        return sequence

    def _on_poll_step(self) -> None:
        if not self._connected:
            return
        sequence = self._build_sequence()
        if not sequence:
            return
        if self._seq_index >= len(sequence):
            self._seq_index = 0
        frame = sequence[self._seq_index]
        self._seq_index = (self._seq_index + 1) % len(sequence)

        self._tx_count += 1
        self._id_counter += 1
        self._pending_id = self._id_counter
        self._pending_expected = (parse_addr(frame),
                                  parse_cmd(frame) & ~CMD_REPLY_FLAG)
        self._pending_tx_ts = time.perf_counter()

        worker = self._worker
        if worker:
            worker.send(frame)
        self._loss_timer.start(REPLY_TIMEOUT_MS)

    def _on_loss_timeout(self) -> None:
        """超时未收到应答：记一次丢包并清理待决状态。"""
        if self._pending_id is not None:
            self._loss_count += 1
            self._pending_id = None
            self._pending_expected = None

    # ------------------------------------------------------------ 接收处理
    def _on_worker_connected(self, ok: bool, text: str) -> None:
        self._connected = ok
        if ok:
            self.reset_stats()
            self.log_message.emit(text, "info")
        else:
            self.log_message.emit(text, "error")
            worker, self._worker = self._worker, None
            if worker:
                try:
                    worker.connected.disconnect()
                    worker.frame_rx.disconnect()
                except (RuntimeError, TypeError):
                    pass
                worker.wait(500)
        self.connection_changed.emit(ok, text)

    def _on_worker_frame(self, frame: bytes, rx_ts: float) -> None:
        addr = parse_addr(frame)
        cmd = parse_cmd(frame)
        self._rx_total += 1
        self._rx_window.append(rx_ts)
        self._last_rx_ts[addr] = rx_ts

        if self._auto_poll and self._pending_id is not None:
            exp_addr, exp_cmd = self._pending_expected  # type: ignore[misc]
            if addr == exp_addr and ((cmd & ~CMD_REPLY_FLAG) == exp_cmd
                                     or cmd == CMD_ERR):
                elapsed_ms = (rx_ts - self._pending_tx_ts) * 1000.0
                self._loss_timer.stop()
                if elapsed_ms > float(REPLY_TIMEOUT_MS):
                    self._loss_count += 1
                self._pending_id = None
                self._pending_expected = None

        self.frame_received.emit(frame, rx_ts)

    # ------------------------------------------------------------ 统计
    def reset_stats(self) -> None:
        self._tx_count = 0
        self._loss_count = 0
        self._rx_total = 0
        self._rx_window.clear()
        self._pending_id = None
        self._pending_expected = None
        self._emit_stats()

    def _emit_stats(self) -> None:
        now = time.perf_counter()
        while self._rx_window and (now - self._rx_window[0] > 1.0):
            self._rx_window.popleft()
        tx, loss = self._tx_count, self._loss_count
        self.stats_changed.emit({
            "fps": len(self._rx_window),
            "tx": tx,
            "loss": loss,
            "rx": self._rx_total,
            "loss_rate": (loss / tx * 100.0) if tx else 0.0,
        })

    @staticmethod
    def device_name(addr: int) -> str:
        return DEV_NAMES.get(addr, f"addr=0x{addr:02X}")
