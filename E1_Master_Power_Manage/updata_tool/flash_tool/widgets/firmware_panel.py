"""右侧面板：固件选择、升级控制、进度条、日志。"""
from __future__ import annotations

import os
import time

from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QProgressBar, QTextEdit, QCheckBox,
    QFileDialog, QMessageBox,
)

from ..protocol import compute_checksum32

# 日志面板最大行数：超出丢弃最旧（防止 QTextDocument 无限增长导致每次 insertHtml 全量重排、升级变慢）
MAX_LOG_BLOCKS = 2000
# 日志批量刷新：消息先进缓冲，定时器统一一次性 insertHtml，避免每条日志都触发文档重排
LOG_FLUSH_INTERVAL_MS = 150
LOG_FLUSH_BATCH = 500


class FirmwarePanel(QWidget):
    start_requested = Signal()
    stop_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._fw_path: str = ""
        self._building = False
        self._log_buffer: list[str] = []
        self._log_at_bottom = True  # 是否停在日志底部（决定是否自动滚动）
        self._build_ui()
        self._wire_signals()
        # 日志批量刷新定时器
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(LOG_FLUSH_INTERVAL_MS)
        self._log_timer.timeout.connect(self._flush_logs)
        self._log_timer.start()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # ── 固件选择 ──
        gb_fw = QGroupBox("固件文件")
        fw_layout = QVBoxLayout(gb_fw)

        fw_row = QHBoxLayout()
        self.fw_path_edit = QLabel("未选择文件")
        self.fw_path_edit.setWordWrap(True)
        fw_row.addWidget(self.fw_path_edit, 1)
        self.browse_btn = QPushButton("浏览...")
        fw_row.addWidget(self.browse_btn)
        fw_layout.addLayout(fw_row)

        fw_info_row = QHBoxLayout()
        self.fw_size_label = QLabel("大小: —")
        fw_info_row.addWidget(self.fw_size_label)
        self.fw_crc_label = QLabel("CHECKSUM: —")
        fw_info_row.addWidget(self.fw_crc_label)
        fw_layout.addLayout(fw_info_row)

        layout.addWidget(gb_fw)

        # ── 升级控制 ──
        gb_ctrl = QGroupBox("升级控制")
        ctrl_layout = QVBoxLayout(gb_ctrl)

        self.start_btn = QPushButton("开始升级")
        self.start_btn.setEnabled(False)
        self.start_btn.setMinimumHeight(36)
        ctrl_layout.addWidget(self.start_btn)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        ctrl_layout.addWidget(self.progress_bar)

        self.status_label = QLabel("就绪")
        ctrl_layout.addWidget(self.status_label)

        layout.addWidget(gb_ctrl)

        # ── 日志 ──
        gb_log = QGroupBox("日志")
        log_layout = QVBoxLayout(gb_log)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFontFamily("Consolas, monospace")
        self.log_text.setLineWrapMode(QTextEdit.NoWrap)
        # 限制文档块数：文档无限增长 → 每次 insertHtml 全量重排 → GUI 线程变慢、拖累升级
        self.log_text.document().setMaximumBlockCount(MAX_LOG_BLOCKS)
        # 跟踪用户是否停留在底部（决定批量刷新后是否自动滚动）
        self.log_text.verticalScrollBar().valueChanged.connect(self._on_log_scroll)
        log_layout.addWidget(self.log_text)

        log_row = QHBoxLayout()
        self.log_enable_chk = QCheckBox("显示日志")
        self.log_enable_chk.setChecked(True)
        self.log_enable_chk.toggled.connect(self._on_log_enable_toggled)
        log_row.addWidget(self.log_enable_chk)
        log_row.addStretch(1)
        clear_btn = QPushButton("清空日志")
        log_row.addWidget(clear_btn)
        clear_btn.clicked.connect(self.log_text.clear)
        log_layout.addLayout(log_row)

        layout.addWidget(gb_log, 1)

    def _wire_signals(self):
        self.browse_btn.clicked.connect(self._browse_fw)
        self.start_btn.clicked.connect(self._on_start_click)

    @Slot()
    def _browse_fw(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "选择固件文件", "",
            "固件文件 (*.bin *.hex);;所有文件 (*)")
        if not path:
            return
        self._fw_path = path

        base = os.path.basename(path)
        self.fw_path_edit.setText(base)

        size = os.path.getsize(path)
        self.fw_size_label.setText(f"大小: {size:,} 字节 ({size/1024:.1f} KB)")

        with open(path, "rb") as f:
            data = f.read()
        checksum = compute_checksum32(data)
        self.fw_crc_label.setText(f"CHECKSUM: 0x{checksum:08X}")

        self.start_btn.setEnabled(True)

    @Slot()
    def _on_start_click(self):
        if self._building:
            # 取消：立即禁用按钮并反馈，避免误触重新开始升级
            self.start_btn.setEnabled(False)
            self.start_btn.setText("取消中…")
            self.status_label.setText("正在取消…")
            self.stop_requested.emit()
        else:
            if not self._fw_path:
                QMessageBox.warning(self, "警告", "请先选择固件文件")
                return
            self.start_requested.emit()

    def get_fw_path(self) -> str:
        return self._fw_path

    def get_fw_size(self) -> int:
        return os.path.getsize(self._fw_path) if self._fw_path else 0

    @Slot(int, int)
    def on_progress(self, current: int, total: int):
        if total > 0:
            pct = int(current * 100 / total)
            self.progress_bar.setValue(pct)
            self.status_label.setText(f"Block {current}/{total} ({pct}%)")

    @Slot(bool)
    def _on_log_enable_toggled(self, enabled: bool):
        """开关日志显示：关闭时置灰并清空日志面板，停止打印。"""
        self.log_text.setEnabled(enabled)
        if not enabled:
            self.log_text.clear()
            self._log_buffer.clear()

    @Slot(int)
    def _on_log_scroll(self, value: int):
        """记录用户是否停留在底部（用于批量刷新后是否自动滚动）。"""
        sb = self.log_text.verticalScrollBar()
        self._log_at_bottom = (value >= sb.maximum() - 8)

    def _format_log_html(self, msg: str) -> str:
        """单条日志 → HTML（含换行），供批量刷新拼接。"""
        ts = time.strftime("%H:%M:%S")
        level = msg[0] if len(msg) > 1 else "I"
        rest = msg[2:] if len(msg) > 2 else msg

        level_colors = {
            "E": "#E53935",
            "W": "#FDD835",
            "D": "#4FC3F7",
            "V": "#888888",
        }
        color = level_colors.get(level, "#A5D6A7")
        ts_color = "#888888"
        # 每条日志独立成 <p> 块：setMaximumBlockCount 按块裁剪才有效（<br> 是段内换行，会被并进同一块）
        return (f'<p style="margin:0;"><span style="color:{color};">{level}</span> '
                f'<span style="color:{ts_color};">({ts})</span> '
                f'<span style="color:{color};">{rest}</span></p>')

    def _flush_logs(self):
        """批量刷新：把缓冲内所有日志一次性 insertHtml，避免逐条触发文档重排。"""
        if not self._log_buffer:
            return
        msgs, self._log_buffer = self._log_buffer, []
        html = "".join(self._format_log_html(m) for m in msgs)

        cursor = self.log_text.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        cursor.insertHtml(html)

        if self._log_at_bottom:
            sb = self.log_text.verticalScrollBar()
            sb.setValue(sb.maximum())

    @Slot(str)
    def on_log(self, msg: str):
        if not self.log_enable_chk.isChecked():
            return  # 日志显示已关闭，不再打印
        self._log_buffer.append(msg)
        # 缓冲过大（GUI 事件循环可能被阻塞）时立即冲刷，避免积压
        if len(self._log_buffer) >= LOG_FLUSH_BATCH:
            self._flush_logs()

    @Slot(bool)
    def on_finished(self, success: bool):
        self._building = False
        self.start_btn.setText("开始升级")
        self.start_btn.setEnabled(True)
        self.browse_btn.setEnabled(True)
        if success:
            self.status_label.setText("✅ 升级完成")
        else:
            self.status_label.setText("❌ 升级失败")

    def set_building(self, building: bool):
        self._building = building
        self.start_btn.setText("取消" if building else "开始升级")
        self.start_btn.setEnabled(True)
        self.browse_btn.setEnabled(not building)

    def set_cancelling(self):
        """取消进行中：禁用按钮并反馈，直到 on_finished 恢复。"""
        self.start_btn.setEnabled(False)
        self.start_btn.setText("取消中…")
        self.status_label.setText("正在取消…")

    def reset(self):
        self.progress_bar.setValue(0)
        self.status_label.setText("就绪")
