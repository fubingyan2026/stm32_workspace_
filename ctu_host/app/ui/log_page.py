# -*- coding: utf-8 -*-
"""通信日志页：集中显示所有收发帧、状态与告警。"""

from __future__ import annotations

from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..theme import TEXT_DIM
from .widgets import Card, LogView


class LogPage(QWidget):
    """全局通信日志。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        heading = QLabel("通信日志 · TX / RX / 告警")
        heading.setStyleSheet("font-size: 14px; font-weight: bold;")
        root.addWidget(heading)

        card = Card()
        toolbar = QHBoxLayout()
        hint = QLabel("轮询期间不记录原始收发帧，避免刷屏；错误应答始终记录。")
        hint.setStyleSheet(f"color: {TEXT_DIM};")
        toolbar.addWidget(hint)
        toolbar.addStretch(1)
        clear_btn = QPushButton("清空日志")
        clear_btn.setObjectName("neutral")
        clear_btn.clicked.connect(self.clear)
        toolbar.addWidget(clear_btn)
        card.add(toolbar)

        self._log = LogView()
        card.add(self._log)
        root.addWidget(card, 1)

    def append_log(self, text: str, level: str = "info") -> None:
        self._log.append_line(text, level)

    def clear(self) -> None:
        self._log.clear()
