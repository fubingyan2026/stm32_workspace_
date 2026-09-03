# -*- coding: utf-8 -*-
"""G0 手套串口上位机 — 协议控制与数据显示

依赖: pyserial + PySide6
运行: python g0_host.py
"""

from __future__ import annotations

import queue
import sys
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QDoubleSpinBox, QPlainTextEdit,
    QFrame, QSplitter, QSlider, QCheckBox, QLineEdit,
)

from g0_protocol import (
    CMD_HEARTBEAT, CMD_KEY_EVENT, CMD_MOTOR_FEEDBACK_REPORT,
    MOTOR_STATE_TEXT, KEY_EVENT_TEXT, KEY_NAME,
    FrameParser, build_motor_target, build_req_feedback, unpack_float_le,
    POS_MIN, POS_MAX, VEL_MIN, VEL_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX,
    TOR_MIN, TOR_MAX,
)

# ---------------- 马卡龙浅色主题 ----------------
STYLE = """
QMainWindow, QWidget { background-color: #F5F0E8; color: #3A3A3A; font-size: 13px; }
QGroupBox {
    background-color: #FFFFFF; border: 1px solid #E4DED4; border-radius: 8px;
    margin-top: 12px; padding: 8px;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4C9B73; font-weight: bold; }
QLabel#title { font-size: 20px; font-weight: bold; color: #4C9B73; }
QLabel#value { font-size: 16px; font-weight: bold; color: #3A3A3A; }
QLabel#dim { color: #9E9E9E; }
QPushButton {
    background-color: #7EC8A0; color: white; border: none; border-radius: 6px;
    padding: 8px 16px; font-weight: bold;
}
QPushButton:hover { background-color: #6DB88F; }
QPushButton:disabled { background-color: #D9D4C8; color: #9E9E9E; }
QPushButton#danger { background-color: #F2A999; }
QPushButton#danger:hover { background-color: #ED9481; }
QComboBox, QDoubleSpinBox {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 5px;
    padding: 4px 8px;
}
QPlainTextEdit {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 6px;
    font-family: Consolas, monospace; font-size: 12px;
}
QSplitter::handle { background-color: #E4DED4; }
"""


# ---------------- 串口接收线程 ----------------
@dataclass
class RxEvent:
    """解析出的单帧事件"""
    kind: str  # "heartbeat" / "key" / "feedback" / "unknown"
    text: str
    payload: bytes = field(default_factory=bytes)


class SerialWorker(QThread):
    frame_received = Signal(object)   # RxEvent
    status_changed = Signal(str, bool)  # (文本, 是否错误)
    log_line = Signal(str, str)       # (文本, 级别: info/warn/tx)

    def __init__(self, port: str, baud: int = 115200, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._parser = FrameParser()
        self._running = False
        self._tx_queue: queue.Queue[bytes] = queue.Queue()

    def run(self) -> None:
        try:
            self._serial = serial.Serial(
                self._port, self._baud,
                timeout=0.05, write_timeout=0.5,
            )
        except Exception as exc:  # noqa: BLE001
            self.status_changed.emit(f"连接失败: {exc}", True)
            return

        self.status_changed.emit(f"已连接 {self._port} @ {self._baud}", False)
        self._running = True

        while self._running:
            # 发送队列由本线程统一处理（跨线程直接写串口会与 read 竞争导致死锁）
            self._flush_tx_queue()

            try:
                data = self._serial.read(256)
            except Exception as exc:  # noqa: BLE001
                self.status_changed.emit(f"串口错误: {exc}", True)
                break
            if not data:
                continue
            for frame in self._parser.feed(data):
                ev = self._decode_frame(frame)
                if ev:
                    self.frame_received.emit(ev)

        try:
            if self._serial:
                self._serial.close()
        except Exception:  # noqa: BLE001
            pass
        self.status_changed.emit("已断开", True)

    def _flush_tx_queue(self) -> None:
        """排空发送队列（仅在工作线程调用）"""
        while True:
            try:
                data = self._tx_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._serial.write(data)  # type: ignore[union-attr]
                self.log_line.emit(data.hex(" ").upper(), "tx")
            except Exception as exc:  # noqa: BLE001
                self.status_changed.emit(f"发送失败: {exc}", True)
                return

    def send(self, data: bytes) -> bool:
        """主线程调用：仅入队，由工作线程实际发送（线程安全）"""
        if not self._running:
            return False
        self._tx_queue.put(data)
        return True

    def stop(self) -> None:
        self._running = False
        self.wait(2000)

    def _decode_frame(self, frame: bytes) -> RxEvent | None:
        cmd = frame[1]
        payload = frame[3 : 3 + frame[2]]

        if cmd == CMD_HEARTBEAT and len(payload) >= 2:
            tick = payload[0] | (payload[1] << 8)
            return RxEvent("heartbeat", f"心跳 tick={tick}", payload)

        if cmd == CMD_KEY_EVENT and len(payload) >= 2:
            key = KEY_NAME.get(payload[0], f"KEY{payload[0]}")
            event = KEY_EVENT_TEXT.get(payload[1], f"0x{payload[1]:02X}")
            return RxEvent("key", f"按键 {key} -> {event}", payload)

        if cmd == CMD_MOTOR_FEEDBACK_REPORT and len(payload) >= 21:
            state = payload[0]
            vals = [
                unpack_float_le(payload[1:5]),
                unpack_float_le(payload[5:9]),
                unpack_float_le(payload[9:13]),
                unpack_float_le(payload[13:17]),
                unpack_float_le(payload[17:21]),
            ]
            stext = MOTOR_STATE_TEXT.get(state, f"未知(0x{state:02X})")
            pos, vel, tor, tmos, tcoil = vals
            text = (
                f"state={state}({stext}) pos={pos:.3f} vel={vel:.3f} "
                f"tor={tor:.3f} Tmos={tmos:.1f} Tcoil={tcoil:.1f}"
            )
            return RxEvent("feedback", text, payload)

        return RxEvent("unknown", f"cmd=0x{cmd:02X} data={payload.hex(' ').upper()}", payload)


# ---------------- 主窗口 ----------------
class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("G0 手套串口上位机")
        self.resize(860, 640)

        self._worker: SerialWorker | None = None
        self._auto_fb_timer: QTimer | None = None
        self._build_ui()

    # ---- UI ----
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)

        # 标题
        title = QLabel("G0 手套串口控制台")
        title.setObjectName("title")
        root.addWidget(title)

        # 连接栏
        conn = QHBoxLayout()
        conn.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        self._refresh_ports()
        conn.addWidget(self.port_combo)

        conn.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        for b in (115200,1000000,2000000):
            self.baud_combo.addItem(f"{b}", b)
        self.baud_combo.setCurrentText("115200")
        conn.addWidget(self.baud_combo)

        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn.addWidget(self.connect_btn)

        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("dim")
        conn.addWidget(self.status_label, 1)
        root.addLayout(conn)

        # 主体
        splitter = QSplitter(Qt.Horizontal)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 8, 0)
        left_layout.addWidget(self._build_control_box())
        left_layout.addWidget(self._build_feedback_box())
        left_layout.addStretch(1)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(8, 0, 0, 0)
        right_layout.addWidget(self._build_log_box())

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([420, 420])
        root.addWidget(splitter, 1)

        self.setCentralWidget(central)

    def _build_control_box(self) -> QGroupBox:
        box = QGroupBox("电机控制")
        grid = QGridLayout(box)

        # 位置行：滑动条（0~464 → 0.000~0.464 rad）+ 右侧显示/输入框
        self.pos_slider = QSlider(Qt.Horizontal)
        self.pos_slider.setRange(0, 464)
        self.pos_slider.setValue(0)
        self.pos_slider.setTickInterval(100)
        self.pos_slider.setTickPosition(QSlider.TicksBelow)
        self.pos_edit = QLineEdit("0.000")
        self.pos_edit.setFixedWidth(72)
        self.pos_edit.setAlignment(Qt.AlignRight)
        pos_row = QHBoxLayout()
        pos_row.addWidget(self.pos_slider, 1)
        pos_row.addWidget(self.pos_edit)
        self.pos_slider.valueChanged.connect(self._on_pos_slider)
        self.pos_edit.editingFinished.connect(self._on_pos_edit)

        self.vel_spin = self._mk_spin(VEL_MIN, VEL_MAX, 0.1, 0.0, 2)
        self.kp_spin = self._mk_spin(KP_MIN, KP_MAX, 0.5, 2.5, 2)
        self.kd_spin = self._mk_spin(KD_MIN, KD_MAX, 0.05, 0.5, 2)
        self.tor_spin = self._mk_spin(TOR_MIN, TOR_MAX, 0.005, 0.0, 3)

        rows = [
            ("位置 pos (rad)", pos_row, f"[{POS_MIN}, {POS_MAX}]"),
            ("速度 vel (rad/s)", self.vel_spin, f"[{VEL_MIN}, {VEL_MAX}]"),
            ("刚度 kp", self.kp_spin, f"[{KP_MIN}, {KP_MAX}]"),
            ("阻尼 kd", self.kd_spin, f"[{KD_MIN}, {KD_MAX}]"),
            ("扭矩 tor (N·m)", self.tor_spin, f"[{TOR_MIN}, {TOR_MAX}]"),
        ]
        for i, (label, widget, hint) in enumerate(rows):
            grid.addWidget(QLabel(label), i, 0)
            if isinstance(widget, QHBoxLayout):
                grid.addLayout(widget, i, 1)
            else:
                grid.addWidget(widget, i, 1)
            hint_label = QLabel(hint)
            hint_label.setObjectName("dim")
            grid.addWidget(hint_label, i, 2)

        # 自动周期请求反馈
        self.auto_fb_check = QCheckBox("自动请求反馈")
        self.auto_fb_spin = self._mk_spin(5, 1000, 5, 200, 0)
        self.auto_fb_spin.setSuffix(" ms")
        self.auto_fb_check.toggled.connect(self._on_auto_fb_toggled)
        self.auto_fb_spin.valueChanged.connect(self._on_auto_fb_period)
        fb_row = QHBoxLayout()
        fb_row.addWidget(self.auto_fb_check)
        fb_row.addWidget(self.auto_fb_spin)
        fb_row.addStretch(1)

        btn_row = QHBoxLayout()
        self.send_btn = QPushButton("发送目标")
        self.send_btn.clicked.connect(self._send_target)
        self.req_fb_btn = QPushButton("请求反馈")
        self.req_fb_btn.clicked.connect(self._send_req_feedback)
        btn_row.addWidget(self.send_btn)
        btn_row.addWidget(self.req_fb_btn)

        grid.addLayout(fb_row, len(rows), 0, 1, 3)
        grid.addLayout(btn_row, len(rows) + 1, 0, 1, 3)
        return box

    def _build_feedback_box(self) -> QGroupBox:
        box = QGroupBox("电机反馈（最新）")
        grid = QGridLayout(box)
        self.state_label = QLabel("--")
        self.state_label.setObjectName("value")
        self.pos_val = QLabel("--")
        self.vel_val = QLabel("--")
        self.tor_val = QLabel("--")
        self.temp_val = QLabel("--")

        grid.addWidget(QLabel("状态:"), 0, 0)
        grid.addWidget(self.state_label, 0, 1, 1, 2)
        grid.addWidget(QLabel("位置:"), 1, 0)
        grid.addWidget(self.pos_val, 1, 1)
        grid.addWidget(QLabel("rad"), 1, 2)
        grid.addWidget(QLabel("速度:"), 2, 0)
        grid.addWidget(self.vel_val, 2, 1)
        grid.addWidget(QLabel("rad/s"), 2, 2)
        grid.addWidget(QLabel("扭矩:"), 3, 0)
        grid.addWidget(self.tor_val, 3, 1)
        grid.addWidget(QLabel("N·m"), 3, 2)
        grid.addWidget(QLabel("温度:"), 4, 0)
        grid.addWidget(self.temp_val, 4, 1, 1, 2)
        return box

    def _build_log_box(self) -> QGroupBox:
        box = QGroupBox("通信日志")
        layout = QVBoxLayout(box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        layout.addWidget(self.log_view)
        return box

    @staticmethod
    def _mk_spin(lo: float, hi: float, step: float, val: float, dec: int) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setSingleStep(step)
        spin.setValue(val)
        spin.setDecimals(dec)
        return spin

    # ---- 串口 ----
    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device}  {p.description}")
        if current:
            self.port_combo.setCurrentText(current)
        self.port_combo.blockSignals(False)

    def _toggle_connect(self) -> None:
        if self._worker and self._worker.isRunning():
            if self._auto_fb_timer is not None:
                self._auto_fb_timer.stop()
            self.auto_fb_check.setChecked(False)
            self._worker.stop()
            self._worker = None
            self.connect_btn.setText("连接")
            self.status_label.setText("未连接")
            return

        text = self.port_combo.currentText()
        port = text.split("  ")[0] if text else ""
        if not port:
            self._log("未选择串口", "warn")
            return

        baud = int(self.baud_combo.currentData())

        self._worker = SerialWorker(port, baud)
        self._worker.frame_received.connect(self._on_frame)
        self._worker.status_changed.connect(self._on_status)
        self._worker.log_line.connect(lambda msg, lvl: self._log(msg, lvl))
        self._worker.start()
        self.connect_btn.setText("断开")

    # ---- 发送 ----
    def _on_pos_slider(self, value: int) -> None:
        pos = value / 1000.0
        self.pos_edit.blockSignals(True)
        self.pos_edit.setText(f"{pos:.3f}")
        self.pos_edit.blockSignals(False)
        self._send_pos(pos)

    def _on_pos_edit(self) -> None:
        try:
            pos = float(self.pos_edit.text())
        except ValueError:
            return
        pos = min(max(pos, POS_MIN), POS_MAX)
        self.pos_edit.setText(f"{pos:.3f}")
        self.pos_slider.blockSignals(True)
        self.pos_slider.setValue(round(pos * 1000.0))
        self.pos_slider.blockSignals(False)
        self._send_pos(pos)

    def _send_pos(self, pos: float) -> None:
        if self._ensure_ready():
            self._worker.send(build_motor_target(  # type: ignore[union-attr]
                pos, self.vel_spin.value(),
                self.kp_spin.value(), self.kd_spin.value(), self.tor_spin.value(),
            ))

    def _send_target(self) -> None:
        if not self._ensure_ready():
            return
        frame = build_motor_target(
            self.pos_slider.value() / 1000.0, self.vel_spin.value(),
            self.kp_spin.value(), self.kd_spin.value(), self.tor_spin.value(),
        )
        self._worker.send(frame)  # type: ignore[union-attr]

    def _send_req_feedback(self) -> None:
        if not self._ensure_ready():
            return
        self._worker.send(build_req_feedback())  # type: ignore[union-attr]

    def _on_auto_fb_toggled(self, checked: bool) -> None:
        if checked:
            if self._auto_fb_timer is None:
                self._auto_fb_timer = QTimer(self)
                self._auto_fb_timer.timeout.connect(self._send_req_feedback)
            self._auto_fb_timer.start(int(self.auto_fb_spin.value()))
        else:
            if self._auto_fb_timer is not None:
                self._auto_fb_timer.stop()

    def _on_auto_fb_period(self, value: float) -> None:
        if self.auto_fb_check.isChecked() and self._auto_fb_timer is not None:
            self._auto_fb_timer.start(int(value))

    def _ensure_ready(self) -> bool:
        if not self._worker or not self._worker.isRunning():
            self._log("未连接串口", "warn")
            return False
        return True

    # ---- 接收/状态 ----
    @Slot(object)
    def _on_frame(self, ev: object) -> None:
        event = ev  # SerialWorker.RxEvent
        self._log(event.text, "info")

        if event.kind == "feedback":
            self._update_feedback(event.text)
        elif event.kind == "heartbeat":
            self._log(f"   [心跳] {event.text}", "info")

    @Slot(str, bool)
    def _on_status(self, text: str, is_error: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #D4655C; font-weight: bold;" if is_error
            else "color: #4C9B73; font-weight: bold;"
        )
        if is_error:
            self._log(text, "warn")
            self.connect_btn.setText("连接")

    def _update_feedback(self, text: str) -> None:
        """从反馈文本更新展示区（简单解析首字段 state=xx）"""
        self.state_label.setText(text.split(" ")[0] if text else "--")
        # 完整字段解析
        try:
            pos = float(text.split("pos=")[1].split(" ")[0])
            vel = float(text.split("vel=")[1].split(" ")[0])
            tor = float(text.split("tor=")[1].split(" ")[0])
            tmos = float(text.split("Tmos=")[1].split(" ")[0])
            tcoil = float(text.split("Tcoil=")[1].split(" ")[0])
            self.pos_val.setText(f"{pos:.3f}")
            self.vel_val.setText(f"{vel:.3f}")
            self.tor_val.setText(f"{tor:.3f}")
            self.temp_val.setText(f"MOS {tmos:.1f}°C / 线圈 {tcoil:.1f}°C")
        except (ValueError, IndexError):
            pass

    def _log(self, text: str, level: str = "info") -> None:
        prefix = {"tx": "[TX]", "warn": "[警告]", "info": "[RX]"}.get(level, "[RX]")
        self.log_view.appendPlainText(f"{prefix} {text}")

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._worker:
            self._worker.stop()
        event.accept()


def main() -> None:
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
