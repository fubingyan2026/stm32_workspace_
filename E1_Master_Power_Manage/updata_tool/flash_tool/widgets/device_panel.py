"""左侧面板：CAN 设备扫描、连接、总线参数配置。

适配 E1_Master_Power_Manage：STM32F407 经典 bxCAN，**仅 8 字节帧**，
Boot 硬件兼容 ID 固定 `0x0002`（G474 为 0x0001）。CAN FD 不再开放。
"""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QHBoxLayout,
    QGroupBox, QComboBox, QPushButton, QLabel, QCheckBox,
    QListWidget, QListWidgetItem, QLineEdit, QSpinBox,
)

from canable_sdk import ZDTCanable

BITRATES = [10_000, 20_000, 50_000, 100_000, 125_000, 250_000,
            500_000, 800_000, 1_000_000]


class DevicePanel(QWidget):
    connected = Signal()
    disconnected = Signal()
    device_scan_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._build_ui()
        self._wire_signals()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # ── 设备分组 ──
        gb_dev = QGroupBox("CAN 设备")
        dv = QVBoxLayout(gb_dev)

        self.device_list = QListWidget()
        dv.addWidget(self.device_list)

        scan_btn_row = QHBoxLayout()
        self.scan_btn = QPushButton("扫描设备")
        scan_btn_row.addWidget(self.scan_btn)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setCheckable(True)
        self.connect_btn.setEnabled(False)
        scan_btn_row.addWidget(self.connect_btn)
        dv.addLayout(scan_btn_row)

        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: #888;")
        dv.addWidget(self.status_label)

        layout.addWidget(gb_dev)

        # ── 总线配置分组 ──
        gb_bus = QGroupBox("总线配置")
        bf = QFormLayout(gb_bus)

        self.bitrate_combo = QComboBox()
        for b in BITRATES:
            self.bitrate_combo.addItem(f"{b:,} bps", b)
        self.bitrate_combo.setCurrentText("1,000,000 bps")
        bf.addRow("波特率:", self.bitrate_combo)

        self.can_id_edit = QLineEdit("0x701")
        self.can_id_edit.setToolTip("Host→Node CAN ID，Node→Host 自动设为 ID+1")
        bf.addRow("CAN ID:", self.can_id_edit)

        self.boot_req_can_id_edit = QLineEdit("0x003")
        self.boot_req_can_id_edit.setToolTip("进 boot 触发帧 CAN ID（App 态专用，0x003）")
        bf.addRow("进Boot CAN ID:", self.boot_req_can_id_edit)

        self.hw_id_edit = QLineEdit("0x0002")
        self.hw_id_edit.setToolTip("E1 Boot 硬件兼容 ID=0x0002（G474 为 0x0001）")
        bf.addRow("HW Compat ID:", self.hw_id_edit)

        self.version_spin = QSpinBox()
        self.version_spin.setRange(1, 65535)
        self.version_spin.setValue(1)
        bf.addRow("固件版本:", self.version_spin)

        self.frame_interval_spin = QSpinBox()
        self.frame_interval_spin.setRange(0, 5000)
        self.frame_interval_spin.setValue(100)
        self.frame_interval_spin.setSingleStep(100)
        self.frame_interval_spin.setSuffix(" µs")
        self.frame_interval_spin.setToolTip(
            "每帧 DATA 发送间隔（亚毫秒用忙等保证精度，毫秒级用 sleep）；0=不限速；默认 100µs")
        bf.addRow("帧间隔:", self.frame_interval_spin)

        note = QLabel("协议: E1 经典 CAN · 8 字节帧 · 双 A/B 分区")
        note.setStyleSheet("color: #888;")
        bf.addRow("", note)

        layout.addWidget(gb_bus)
        layout.addStretch(1)

    def _wire_signals(self):
        self.scan_btn.clicked.connect(self._scan_devices)
        self.connect_btn.clicked.connect(self._on_connect_toggle)

    @Slot()
    def _scan_devices(self):
        self.device_list.clear()
        try:
            devs = ZDTCanable.list_devices()
        except Exception as e:
            item = QListWidgetItem(f"扫描失败: {e}")
            item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
            self.device_list.addItem(item)
            return

        if not devs:
            item = QListWidgetItem("未发现 CANable 设备")
            item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
            self.device_list.addItem(item)
            return

        for d in devs:
            label = f"{d.get('manufacturer','') or 'CANable'} {d.get('product','') or ''}\n  S/N: {d.get('serial','?')}"
            li = QListWidgetItem(label.strip())
            li.setData(Qt.UserRole, d)
            self.device_list.addItem(li)

        self.device_list.setCurrentRow(0)
        self.connect_btn.setEnabled(True)
        self.device_scan_requested.emit()

    @Slot()
    def _on_connect_toggle(self):
        if self._connected:
            self.disconnected.emit()
        else:
            self.connected.emit()

    @Slot()
    def set_connected(self):
        self._connected = True
        self.connect_btn.setChecked(True)
        self.connect_btn.setText("断开")
        self.status_label.setText("已连接")
        self.status_label.setStyleSheet("color: #0a0; font-weight: bold;")
        self.scan_btn.setEnabled(False)
        self.device_list.setEnabled(False)

    @Slot()
    def set_disconnected(self):
        self._connected = False
        self.connect_btn.setChecked(False)
        self.connect_btn.setText("连接")
        self.status_label.setText("未连接")
        self.status_label.setStyleSheet("color: #888;")
        self.scan_btn.setEnabled(True)
        self.device_list.setEnabled(True)

    def get_selected_device(self) -> Optional[dict]:
        item = self.device_list.currentItem()
        if item is None:
            return None
        return item.data(Qt.UserRole)

    def get_config_dict(self) -> dict:
        return {
            "bitrate": self.bitrate_combo.currentData(),
            # E1 为经典 bxCAN：固定非 FD、帧长 8（Boot 仅接受经典 CAN）
            "fd_mode": False,
            "data_bitrate": 0,
            "max_frame_size": 8,
            "can_id": int(self.can_id_edit.text(), 16) if self.can_id_edit.text().startswith("0x") else int(self.can_id_edit.text()),
            "boot_request_can_id": int(self.boot_req_can_id_edit.text(), 16) if self.boot_req_can_id_edit.text().startswith("0x") else int(self.boot_req_can_id_edit.text()),
            "hw_compat_id": int(self.hw_id_edit.text(), 16) if self.hw_id_edit.text().startswith("0x") else int(self.hw_id_edit.text()),
            "version": self.version_spin.value(),
            "frame_interval_us": self.frame_interval_spin.value(),
        }
