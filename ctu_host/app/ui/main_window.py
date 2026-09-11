# -*- coding: utf-8 -*-
"""主窗口：顶栏（连接 + 轮询控制）、侧栏导航与多页堆叠。

页面：
    概览 / E1_MASTER / E1_SLAVER / 固件升级 / 通信日志

主窗口是唯一的“编排者”：持有 :class:`SerialSession`，负责把会话信号分发给
各页面，并在固件升级时暂停轮询、独占串口、完成后自动恢复。
"""

from __future__ import annotations

import os

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from ..protocol import (
    DEV_ADDR_MASTER,
    DEV_ADDR_SLAVER,
    describe_frame,
    frame_hex,
    parse_addr,
)
from ..session import SerialSession
from ..theme import ERROR, OK, WARN
from ..transport import UpgradeWorker
from .log_page import LogPage
from .master_page import MasterPage
from .overview_page import OverviewPage
from .slaver_page import SlaverPage
from .upgrade_page import UpgradePage
from .widgets import ConnectionBar, StatusChip

DEFAULT_POLL_MS = 100
RECONNECT_DELAY_MS = 1500


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("E1 CTU 电源板 RS485 调试上位机")
        self.resize(1200, 780)
        self.setMinimumSize(820, 560)

        self._session = SerialSession(self)
        self._upgrade_worker: UpgradeWorker | None = None
        self._upgrade_port = ""
        self._upgrade_baud = 115200
        self._upgrade_was_connected = False
        self._restore_poll = False
        self._pending_restore_poll = False

        self._build_ui()
        self._wire_session()
        self._update_connection_ui(False, "")

    # ==================================================================== UI
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_header())

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        self._nav = QListWidget()
        self._nav.setObjectName("nav")
        self._nav.setMinimumWidth(150)
        self._nav.setMaximumWidth(210)
        for name in ("系统概览", "E1_MASTER 主控板", "E1_SLAVER 副电源板",
                     "固件升级", "通信日志"):
            self._nav.addItem(name)
        self._nav.currentRowChanged.connect(self._on_nav_changed)
        body.addWidget(self._nav)

        self._stack = QStackedWidget()
        self._overview_page = OverviewPage(self._session)
        self._master_page = MasterPage(self._session)
        self._slaver_page = SlaverPage(self._session)
        self._upgrade_page = UpgradePage()
        self._log_page = LogPage()
        for page in (self._overview_page, self._master_page, self._slaver_page,
                     self._upgrade_page, self._log_page):
            self._stack.addWidget(page)
        body.addWidget(self._stack, 1)

        root.addLayout(body, 1)
        self.setCentralWidget(central)

        self._build_status_bar()
        self._nav.setCurrentRow(0)

        self._master_page.log.connect(self._append_log)
        self._slaver_page.log.connect(self._append_log)
        self._upgrade_page.start_requested.connect(self._on_upgrade_start)
        self._upgrade_page.cancel_requested.connect(self._on_upgrade_cancel)

        self._session.set_poll_suppliers(
            [self._master_page.poll_requests, self._slaver_page.poll_requests])

    def _build_header(self) -> QFrame:
        header = QFrame()
        header.setObjectName("header")
        layout = QVBoxLayout(header)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(6)

        row1 = QHBoxLayout()
        title = QLabel("E1 CTU 电源板 RS485 调试上位机")
        title.setObjectName("appTitle")
        subtitle = QLabel("MASTER + SLAVER 同总线 · 严格 10ms 应答判定")
        subtitle.setObjectName("appSubtitle")
        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        row1.addLayout(title_box)
        row1.addStretch(1)

        self._conn_bar = ConnectionBar()
        self._conn_bar.connect_requested.connect(self._on_connect)
        self._conn_bar.disconnect_requested.connect(self._on_disconnect)
        row1.addWidget(self._conn_bar)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        self._auto_poll = QCheckBox("自动轮询")
        self._auto_poll.toggled.connect(self._session.set_auto_poll)
        row2.addWidget(self._auto_poll)

        row2.addWidget(QLabel("间隔:"))
        self._poll_ms = QSpinBox()
        self._poll_ms.setRange(5, 10000)
        self._poll_ms.setValue(DEFAULT_POLL_MS)
        self._poll_ms.setSuffix(" ms")
        self._poll_ms.valueChanged.connect(self._session.set_poll_interval)
        row2.addWidget(self._poll_ms)

        self._show_frames = QCheckBox("显示收发帧")
        self._show_frames.setChecked(True)
        self._show_frames.toggled.connect(self._on_show_frames_toggled)
        row2.addWidget(self._show_frames)

        reset_btn = QPushButton("清零统计")
        reset_btn.setObjectName("neutral")
        reset_btn.clicked.connect(self._session.reset_stats)
        row2.addWidget(reset_btn)
        row2.addStretch(1)
        layout.addLayout(row2)
        return header

    def _build_status_bar(self) -> None:
        bar = QStatusBar()
        self.setStatusBar(bar)

        self._conn_chip = StatusChip("未连接", "idle")
        bar.addWidget(self._conn_chip)

        self._lbl_fps = QLabel()
        self._lbl_loss = QLabel()
        self._lbl_tx = QLabel()
        self._lbl_rx = QLabel()
        for label in (self._lbl_tx, self._lbl_rx, self._lbl_fps, self._lbl_loss):
            bar.addPermanentWidget(label)
        self._on_stats({"fps": 0, "tx": 0, "loss": 0, "rx": 0, "loss_rate": 0.0})

    # ================================================================= 信号
    def _wire_session(self) -> None:
        self._session.connection_changed.connect(self._on_connection_changed)
        self._session.auto_poll_changed.connect(self._on_auto_poll_changed)
        self._session.stats_changed.connect(self._on_stats)
        self._session.log_message.connect(self._append_log)
        self._session.frame_received.connect(self._on_frame)

    def _on_nav_changed(self, index: int) -> None:
        self._stack.setCurrentIndex(index)

    def _on_connect(self) -> None:
        self._session.connect_port(self._conn_bar.port(), self._conn_bar.baud())

    def _on_disconnect(self) -> None:
        self._session.disconnect()

    def _on_connection_changed(self, ok: bool, text: str) -> None:
        self._update_connection_ui(ok, text)
        if ok and self._pending_restore_poll:
            self._pending_restore_poll = False
            self._restore_poll = False
            self._auto_poll.setChecked(True)

    def _update_connection_ui(self, ok: bool, text: str) -> None:
        self._conn_bar.set_connected(ok, text)
        self._conn_chip.set_state("已连接" if ok else "未连接",
                                  "ok" if ok else "idle")
        if ok and text:
            self.setWindowTitle(f"E1 CTU 电源板 RS485 调试上位机 ({text})")
        else:
            self.setWindowTitle("E1 CTU 电源板 RS485 调试上位机")
        if not ok:
            self._auto_poll.blockSignals(True)
            self._auto_poll.setChecked(False)
            self._auto_poll.blockSignals(False)

    def _on_auto_poll_changed(self, enabled: bool) -> None:
        self._auto_poll.blockSignals(True)
        self._auto_poll.setChecked(enabled)
        self._auto_poll.blockSignals(False)

    def _on_show_frames_toggled(self, checked: bool) -> None:
        self._session.log_frames = checked

    # ------------------------------------------------------------ 接收路由
    def _on_frame(self, frame: bytes, _rx_ts: float) -> None:
        if self._session.frames_log_enabled():
            addr = parse_addr(frame)
            self._append_log(
                f"RX {self._session.device_name(addr)}: {frame_hex(frame)} "
                f"[{describe_frame(frame)}]", "rx")

        addr = parse_addr(frame)
        if addr == DEV_ADDR_MASTER:
            self._master_page.handle_frame(frame)
        elif addr == DEV_ADDR_SLAVER:
            self._slaver_page.handle_frame(frame)
        else:
            self._append_log(
                f"收到未知设备帧 addr=0x{addr:02X}: {frame_hex(frame)}", "warn")

    def _on_stats(self, stats: dict) -> None:
        loss_rate = stats["loss_rate"]
        loss_color = OK if loss_rate == 0 else (WARN if loss_rate < 5 else ERROR)
        self._lbl_fps.setText(
            f"RX 帧率: <b style='color:#3E7BB6;'>{stats['fps']:.1f}</b> fps")
        self._lbl_loss.setText(
            f"丢包率: <b style='color:{loss_color};'>{loss_rate:.2f}%</b> "
            f"({stats['loss']} / {stats['tx']})")
        self._lbl_tx.setText(f"轮询发包: <b>{stats['tx']}</b>")
        self._lbl_rx.setText(f"接收总数: <b>{stats['rx']}</b>")

    # ------------------------------------------------------------ 日志
    def _append_log(self, text: str, level: str = "info") -> None:
        self._log_page.append_log(text, level)
        if self._upgrade_page is not None:
            self._upgrade_page.append_log(text, level)

    # ================================================================= 升级
    def _on_upgrade_start(self, device: str, filepath: str) -> None:
        if self._upgrade_worker and self._upgrade_worker.isRunning():
            self._append_log("升级进行中", "warn")
            return

        self._upgrade_port = self._conn_bar.port()
        self._upgrade_baud = self._conn_bar.baud()
        if not self._upgrade_port:
            self._append_log("请先选择串口", "warn")
            return

        name = os.path.basename(filepath)
        answer = QMessageBox.warning(
            self, "确认升级",
            f"将通过 Bootloader 升级 {self._upgrade_page.device_display()}\n"
            f"固件: {name}\n\n"
            "升级期间将暂停轮询并独占串口，完成后自动恢复。是否继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if answer != QMessageBox.StandardButton.Yes:
            return

        # 暂停轮询并释放串口，升级线程独占
        self._restore_poll = self._session.auto_poll
        was_connected = self._session.is_connected
        if self._session.auto_poll:
            self._session.set_auto_poll(False)
        self._session.disconnect()

        self._append_log(f"开始升级: {name} → "
                         f"{self._upgrade_page.device_display()}", "info")
        worker = UpgradeWorker(self._upgrade_port, self._upgrade_baud,
                               device, filepath)
        worker.log_line.connect(self._append_log)
        worker.phase.connect(self._upgrade_page.set_phase)
        worker.progress.connect(self._upgrade_page.set_progress)
        worker.done.connect(self._on_upgrade_done)
        self._upgrade_worker = worker

        self._upgrade_page.set_running(True)
        self._conn_bar.set_enabled(False)
        self._upgrade_was_connected = was_connected
        worker.start()

    def _on_upgrade_cancel(self) -> None:
        if self._upgrade_worker and self._upgrade_worker.isRunning():
            self._upgrade_worker.cancel()
            self._append_log("请求中止升级...", "warn")

    def _on_upgrade_done(self, ok: bool, text: str) -> None:
        self._append_log(text, "info" if ok else "error")
        self._upgrade_page.finish(ok)
        self._conn_bar.set_enabled(True)
        self._upgrade_worker = None
        if self._upgrade_was_connected:
            QTimer.singleShot(RECONNECT_DELAY_MS, self._reconnect_after_upgrade)
        self._upgrade_was_connected = False

    def _reconnect_after_upgrade(self) -> None:
        self._pending_restore_poll = self._restore_poll
        self._restore_poll = False
        self._session.connect_port(self._upgrade_port, self._upgrade_baud)

    # ================================================================= 退出
    def closeEvent(self, event) -> None:  # noqa: N802
        if self._upgrade_worker and self._upgrade_worker.isRunning():
            self._upgrade_worker.cancel()
            self._upgrade_worker.wait(2000)
        self._session.disconnect()
        super().closeEvent(event)
