# -*- coding: utf-8 -*-
"""固件升级页：选择目标与固件、预检、进度与阶段展示。"""

from __future__ import annotations

import os

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..firmware import (
    DEVICE_MASTER,
    DEVICE_SLAVER,
    inspect_firmware,
)
from ..theme import ACCENT_TEXT, ERROR, OK, TEXT_DIM
from .widgets import Card, LogView


class UpgradePage(QWidget):
    """Boot 固件升级操作页（与轮询互斥，由主窗口负责编排）。"""

    start_requested = pyqtSignal(str, str)   # (目标设备, 固件路径)
    cancel_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._filepath = ""
        self._app_ok = False

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        heading = QLabel("固件升级 · Bootloader 分块传输")
        heading.setStyleSheet("font-size: 14px; font-weight: bold;")
        root.addWidget(heading)

        root.addWidget(self._settings_card())
        root.addWidget(self._progress_card())
        root.addWidget(self._log_card(), 1)

    # -------------------------------------------------------------- 卡片
    def _settings_card(self) -> Card:
        card = Card("升级设置")
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        grid.addWidget(QLabel("目标设备:"), 0, 0)
        self._device_cb = QComboBox()
        self._device_cb.addItem("E1_MASTER (0x01)", DEVICE_MASTER)
        self._device_cb.addItem("E1_SLAVER (0x02)", DEVICE_SLAVER)
        grid.addWidget(self._device_cb, 0, 1, 1, 2)

        grid.addWidget(QLabel("固件文件:"), 1, 0)
        self._file_lbl = QLabel("尚未选择")
        self._file_lbl.setStyleSheet(f"color: {TEXT_DIM};")
        self._file_lbl.setMinimumWidth(240)
        grid.addWidget(self._file_lbl, 1, 1)
        browse_btn = QPushButton("选择 .bin")
        browse_btn.setObjectName("neutral")
        browse_btn.clicked.connect(self._on_browse)
        grid.addWidget(browse_btn, 1, 2)

        self._info_lbl = QLabel("")
        grid.addWidget(self._info_lbl, 2, 1, 1, 2)
        grid.setColumnStretch(1, 1)
        card.add(grid)
        return card

    def _progress_card(self) -> Card:
        card = Card("升级进度")
        buttons = QHBoxLayout()
        buttons.setSpacing(8)

        self._start_btn = QPushButton("开始升级")
        self._start_btn.clicked.connect(self._on_start)
        self._cancel_btn = QPushButton("中止")
        self._cancel_btn.setObjectName("danger")
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.clicked.connect(self.cancel_requested.emit)
        buttons.addWidget(self._start_btn)
        buttons.addWidget(self._cancel_btn)
        buttons.addStretch(1)
        card.add(buttons)

        self._phase_lbl = QLabel("待命")
        self._phase_lbl.setStyleSheet(
            f"color: {ACCENT_TEXT}; font-weight: bold;")
        card.add(self._phase_lbl)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        card.add(self._progress)
        return card

    def _log_card(self) -> Card:
        card = Card("升级日志")
        self._log = LogView(max_blocks=3000)
        self._log.setMinimumHeight(160)
        card.add(self._log)
        return card

    # -------------------------------------------------------------- 对外接口
    def filepath(self) -> str:
        return self._filepath

    def device(self) -> str:
        return self._device_cb.currentData()

    def device_display(self) -> str:
        return self._device_cb.currentText()

    def is_firmware_ready(self) -> bool:
        return bool(self._filepath) and self._app_ok

    def set_running(self, running: bool) -> None:
        self._start_btn.setEnabled(not running)
        self._cancel_btn.setEnabled(running)
        self._device_cb.setEnabled(not running)
        if running:
            self._progress.setValue(0)
            self._phase_lbl.setText("准备...")

    def set_phase(self, text: str) -> None:
        self._phase_lbl.setText(text)

    def set_progress(self, value: int) -> None:
        self._progress.setValue(value)

    def finish(self, ok: bool) -> None:
        self.set_running(False)
        self._phase_lbl.setText("完成" if ok else "失败")
        if not ok:
            self._progress.setValue(0)

    def append_log(self, text: str, level: str = "info") -> None:
        self._log.append_line(text, level)

    # -------------------------------------------------------------- 内部
    def _on_browse(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self, "选择固件", "", "固件 (*.bin *.hex *.elf);;所有文件 (*)")
        if not path:
            return
        self._filepath = path
        self._file_lbl.setText(os.path.basename(path))
        self._file_lbl.setToolTip(path)
        self._file_lbl.setStyleSheet("")

        ok, size, reason = inspect_firmware(path)
        self._app_ok = ok
        if ok:
            self._info_lbl.setText(
                f"<span style='color:{OK};'>{size}B — 预检通过</span>")
        else:
            self._info_lbl.setText(
                f"<span style='color:{ERROR};'>{size}B — {reason}</span>")

    def _on_start(self) -> None:
        if self.is_firmware_ready():
            self.start_requested.emit(self.device(), self._filepath)
