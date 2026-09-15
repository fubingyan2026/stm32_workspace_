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
from ..settings import FirmwareHistory
from ..theme import ACCENT_TEXT, ERROR, OK
from .widgets import Card, LogView


class UpgradePage(QWidget):
    """固件升级操作页（App 内直接下载 / 无 App 时 Boot 兜底，与轮询互斥，由主窗口编排）。"""

    start_requested = pyqtSignal(str, str)   # (目标设备, 固件路径)
    cancel_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._filepath = ""
        self._app_ok = False
        # 每个目标设备各自独立的固件路径历史
        self._histories = {
            DEVICE_MASTER: FirmwareHistory(DEVICE_MASTER),
            DEVICE_SLAVER: FirmwareHistory(DEVICE_SLAVER),
        }

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        heading = QLabel("固件升级 · RS485 寻址分块传输")
        heading.setStyleSheet("font-size: 14px; font-weight: bold;")
        root.addWidget(heading)

        root.addWidget(self._settings_card())
        root.addWidget(self._progress_card())
        root.addWidget(self._log_card(), 1)

        # 界面就绪后再接信号；切换设备即切换到该设备自己的固件历史
        self._device_cb.currentIndexChanged.connect(self._on_device_changed)
        self._reload_history()

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
        self._file_cb = QComboBox()
        self._file_cb.setMinimumWidth(260)
        self._file_cb.setMinimumContentsLength(28)
        self._file_cb.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._file_cb.activated.connect(self._on_history_selected)
        grid.addWidget(self._file_cb, 1, 1)

        browse_btn = QPushButton("选择 .bin")
        browse_btn.setObjectName("neutral")
        browse_btn.clicked.connect(self._on_browse)
        grid.addWidget(browse_btn, 1, 2)

        self._clear_history_btn = QPushButton("清除历史")
        self._clear_history_btn.setObjectName("ghost")
        self._clear_history_btn.clicked.connect(self._on_clear_history)
        grid.addWidget(self._clear_history_btn, 1, 3)

        self._info_lbl = QLabel("")
        grid.addWidget(self._info_lbl, 2, 1, 1, 3)
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
        self._file_cb.setEnabled(not running)
        self._clear_history_btn.setEnabled(not running and bool(self._current_history().items()))
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
    def _current_history(self) -> FirmwareHistory:
        """当前目标设备对应的固件路径历史。"""
        return self._histories[self._device_cb.currentData()]

    def _on_device_changed(self, _index: int) -> None:
        """切换目标设备：载入该设备自己的历史与最近一次固件。"""
        self._reload_history()

    def _reload_history(self) -> None:
        self._populate_history()
        recent = self._current_history().items()
        if recent:
            self._load_firmware(recent[0], record=False)
        else:
            self._clear_selection()

    def _clear_selection(self) -> None:
        self._filepath = ""
        self._app_ok = False
        self._file_cb.setToolTip("")
        self._info_lbl.clear()

    def _on_browse(self) -> None:
        start_dir = os.path.dirname(self._filepath) if self._filepath else ""
        path, _selected = QFileDialog.getOpenFileName(
            self, "选择固件", start_dir, "固件 (*.bin *.hex *.elf);;所有文件 (*)")
        if not path:
            return
        self._load_firmware(path, record=True)

    def _on_history_selected(self, index: int) -> None:
        if index >= 0:
            self._load_firmware(self._file_cb.itemText(index), record=False)

    def _on_clear_history(self) -> None:
        """清空当前设备历史，但保留其当前已选固件，避免下拉框变空。"""
        history = self._current_history()
        history.clear()
        if self._filepath:
            history.add(self._filepath)
        self._populate_history()
        self._select_current(self._filepath)

    def _populate_history(self) -> None:
        """用当前设备的历史记录重建下拉框（阻塞信号，避免误触发加载）。"""
        history = self._current_history().items()
        self._file_cb.blockSignals(True)
        self._file_cb.clear()
        for path in history:
            self._file_cb.addItem(path)
        self._file_cb.blockSignals(False)
        self._clear_history_btn.setEnabled(bool(history))

    def _select_current(self, path: str) -> None:
        index = self._file_cb.findText(path)
        if index >= 0:
            self._file_cb.blockSignals(True)
            self._file_cb.setCurrentIndex(index)
            self._file_cb.blockSignals(False)

    def _load_firmware(self, path: str, record: bool) -> None:
        """载入并预检固件；``record=True`` 时写入当前设备的历史。"""
        self._filepath = path
        self._file_cb.setToolTip(path)
        ok, size, reason = inspect_firmware(path)
        self._app_ok = ok
        if ok:
            self._info_lbl.setText(
                f"<span style='color:{OK};'>{size}B — 预检通过</span>")
        else:
            self._info_lbl.setText(
                f"<span style='color:{ERROR};'>{size}B — {reason}</span>")
        if record:
            self._current_history().add(path)
            self._populate_history()
            self._select_current(path)

    def _on_start(self) -> None:
        if self.is_firmware_ready():
            self.start_requested.emit(self.device(), self._filepath)
