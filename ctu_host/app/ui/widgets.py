# -*- coding: utf-8 -*-
"""可复用界面控件：卡片、键值网格、状态胶囊、连接栏、日志视图。"""

from __future__ import annotations

import time
from collections.abc import Iterable

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLayout,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from .. import theme

BAUD_RATES = (115200, 460800, 921600)


class Card(QFrame):
    """带标题的白色圆角卡片容器。"""

    def __init__(self, title: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("card")
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(12, 8, 12, 10)
        self.body.setSpacing(6)
        if title:
            label = QLabel(title)
            label.setObjectName("cardTitle")
            self.body.addWidget(label)

    def add(self, item: QWidget | QLayout) -> None:
        if isinstance(item, QLayout):
            self.body.addLayout(item)
        else:
            self.body.addWidget(item)

    def add_row(self, *items: QWidget, spacing: int = 8) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(spacing)
        for item in items:
            row.addWidget(item)
        self.body.addLayout(row)
        return row


class KvGrid(QWidget):
    """名称/数值键值网格，支持按 ``columns`` 分栏排布。"""

    def __init__(self, rows: Iterable[tuple[str, str]], columns: int = 2,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(3)
        self._labels: dict[str, QLabel] = {}

        rows = list(rows)
        per_col = (len(rows) + columns - 1) // columns if columns else len(rows)
        for col in range(columns):
            chunk = rows[col * per_col:(col + 1) * per_col]
            if not chunk:
                continue
            for row, (title, key) in enumerate(chunk):
                name = QLabel(title)
                name.setStyleSheet(f"color: {theme.TEXT_DIM};")
                value = QLabel("--")
                value.setObjectName("statValue")
                value.setTextInteractionFlags(
                    Qt.TextInteractionFlag.TextSelectableByMouse)
                layout.addWidget(name, row, col * 2)
                layout.addWidget(value, row, col * 2 + 1)
                layout.setColumnStretch(col * 2 + 1, 1)
                self._labels[key] = value
        layout.setColumnStretch(columns * 2, 1)

    def set_value(self, key: str, text: str, color: str | None = None) -> None:
        label = self._labels.get(key)
        if label is None:
            return
        if color:
            label.setText(f"<span style='color:{color};'>{text}</span>")
        else:
            label.setText(text)

    def set_values(self, values: dict[str, str]) -> None:
        for key, text in values.items():
            self.set_value(key, text)

    @property
    def labels(self) -> dict[str, QLabel]:
        """键 → 数值标签，便于宿主页面统一更新。"""
        return self._labels


class StatusChip(QLabel):
    """状态胶囊：按 state（ok/warn/error/idle）自动着色。"""

    def __init__(self, text: str = "", state: str = "idle",
                 parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self.setObjectName("chip")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_state(text, state)

    def set_state(self, text: str, state: str) -> None:
        self.setText(text)
        self.setProperty("state", state)
        self.style().unpolish(self)
        self.style().polish(self)


class ConnectionBar(QWidget):
    """串口连接栏：端口/波特率选择 + 连接/断开。"""

    connect_requested = pyqtSignal()
    disconnect_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        row.addWidget(QLabel("串口:"))
        self._port_cb = QComboBox()
        self._port_cb.setMinimumWidth(140)
        row.addWidget(self._port_cb)

        refresh_btn = QPushButton("刷新")
        refresh_btn.setObjectName("neutral")
        refresh_btn.clicked.connect(self.refresh_ports)
        row.addWidget(refresh_btn)

        row.addWidget(QLabel("波特率:"))
        self._baud_cb = QComboBox()
        for baud in BAUD_RATES:
            self._baud_cb.addItem(str(baud))
        self._baud_cb.setCurrentText("115200")
        row.addWidget(self._baud_cb)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._on_clicked)
        row.addWidget(self._connect_btn)

        self._connected = False
        self.refresh_ports()

    # -------------------------------------------------------------- 对外接口
    def port(self) -> str:
        return self._port_cb.currentText()

    def baud(self) -> int:
        return int(self._baud_cb.currentText())

    def set_connected(self, connected: bool, text: str = "") -> None:
        self._connected = connected
        self._connect_btn.setText("断开" if connected else "连接")
        self._connect_btn.setObjectName("danger" if connected else "")
        self._connect_btn.style().unpolish(self._connect_btn)
        self._connect_btn.style().polish(self._connect_btn)
        self._port_cb.setEnabled(not connected)
        self._baud_cb.setEnabled(not connected)
        if text:
            self._connect_btn.setToolTip(text)

    def set_enabled(self, enabled: bool) -> None:
        self.setEnabled(enabled)

    def refresh_ports(self) -> None:
        current = self._port_cb.currentText()
        self._port_cb.clear()
        for info in list_ports.comports():
            self._port_cb.addItem(info.device)
        if current:
            self._port_cb.setCurrentText(current)

    # -------------------------------------------------------------- 内部
    def _on_clicked(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit()


class LogView(QPlainTextEdit):
    """带时间戳与级别着色的只读日志视图（块数有上限，避免无限增长）。"""

    def __init__(self, max_blocks: int = 5000,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(max_blocks)

    def append_line(self, text: str, level: str = "info") -> None:
        timestamp = time.strftime("%H:%M:%S")
        color = theme.LEVEL_COLOR.get(level)
        prefix = theme.LEVEL_PREFIX.get(level, "")
        head = f"<span style='color:{theme.TEXT_DIM}'>{timestamp}</span> "
        if color:
            body = f"<span style='color:{color}'>{prefix}{text}</span>"
        else:
            body = text
        self.appendHtml(head + body)
