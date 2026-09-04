# -*- coding: utf-8 -*-
"""E1_MASTER_POWER_CTU RS485 上位机 — 状态/温度/电压查询与蜂鸣器控制

依赖: pyserial + PySide6
运行: python ctu_host.py
协议见 docs/protocol_master_485.md（与 ctu_protocol.py 对齐）。
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QPlainTextEdit, QSlider,
    QCheckBox, QSpinBox, QFrame,
)

from ctu_protocol import (
    CMD_ERR, CMD_REPLY_FLAG,
    CMD_READ_STATUS, CMD_READ_TEMP, CMD_READ_VOLT, CMD_CTRL,
    ERR_TEXT, FrameParser, build_read_status, build_read_temp,
    build_read_volt, build_ctrl, status_items,
    unpack_i16_le, unpack_u16_le,
    frame_hex, parse_cmd, parse_payload,
)

STYLE = """
QMainWindow, QWidget { background-color: #F5F0E8; color: #3A3A3A; font-size: 13px; }
QGroupBox {
    background-color: #FFFFFF; border: 1px solid #E4DED4; border-radius: 8px;
    margin-top: 12px; padding: 8px;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4C9B73; font-weight: bold; }
QLabel#title { font-size: 20px; font-weight: bold; color: #4C9B73; }
QPushButton {
    background-color: #7EC8A0; color: white; border: none; border-radius: 6px;
    padding: 7px 14px; font-weight: bold;
}
QPushButton:hover { background-color: #6DB88F; }
QPushButton:disabled { background-color: #D9D4C8; color: #9E9E9E; }
QPushButton#danger { background-color: #F2A999; }
QPushButton#danger:hover { background-color: #ED9481; }
QComboBox, QSpinBox {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 5px;
    padding: 4px 8px;
}
QPlainTextEdit {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 6px;
    font-family: Consolas, monospace; font-size: 12px;
}
"""


# ---------------- 串口工作线程 ----------------
class SerialWorker(QThread):
    connected = Signal(bool, str)   # (成功, 描述)
    frame_rx = Signal(bytes)
    log_line = Signal(str, str)     # (文本, 级别: info/warn/error)

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
                    n = self._serial.in_waiting
                except Exception as exc:  # noqa: BLE001
                    self.connected.emit(False, f"串口错误: {exc}")
                    break
                if n > 0:
                    data = self._serial.read(n)
                    for frame in self._parser.feed(data):
                        self.frame_rx.emit(frame)
                time.sleep(0.002)
        finally:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None

    def send(self, data: bytes, log: bool = True) -> bool:
        if self._serial is None:
            return False
        try:
            self._serial.write(data)
            if log:
                self.log_line.emit(f"TX  {frame_hex(data)}", "tx")
            return True
        except Exception as exc:  # noqa: BLE001
            self.connected.emit(False, f"发送失败: {exc}")
            return False


# ---------------- 主窗口 ----------------
class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("E1_MASTER_POWER_CTU — RS485 主控电源板上位机")
        self.resize(900, 640)
        self._worker: SerialWorker | None = None
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        title = QLabel("E1_MASTER_POWER_CTU · 主控电源板调试")
        title.setObjectName("title")
        root.addWidget(title)

        # 连接栏
        conn = QHBoxLayout()
        conn.addWidget(QLabel("串口:"))
        self._port_cb = QComboBox()
        self._refresh_ports()
        conn.addWidget(self._port_cb)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)
        conn.addWidget(btn_refresh)
        conn.addWidget(QLabel("波特率:"))
        self._baud_cb = QComboBox()
        for b in (115200, 460800, 921600):
            self._baud_cb.addItem(str(b))
        self._baud_cb.setCurrentText("115200")
        conn.addWidget(self._baud_cb)
        self._conn_btn = QPushButton("连接")
        self._conn_btn.setObjectName("danger")
        self._conn_btn.clicked.connect(self._on_connect)
        conn.addWidget(self._conn_btn)
        conn.addStretch(1)
        self._auto_poll = QCheckBox("自动轮询")
        self._auto_poll.toggled.connect(self._on_auto_poll_toggled)
        conn.addWidget(self._auto_poll)
        self._poll_ms = QSpinBox()
        self._poll_ms.setRange(100, 2000)
        self._poll_ms.setValue(500)
        self._poll_ms.setSuffix(" ms")
        conn.addWidget(self._poll_ms)
        root.addLayout(conn)

        body = QHBoxLayout()

        # 左：查询/控制
        left = QVBoxLayout()

        qbox = QGroupBox("查询")
        ql = QHBoxLayout()
        for text, fn in (
            ("读系统状态", self._send_read_status),
            ("读温度", self._send_read_temp),
            ("读电压", self._send_read_volt),
        ):
            b = QPushButton(text)
            b.clicked.connect(fn)
            ql.addWidget(b)
        qbox.setLayout(ql)
        left.addWidget(qbox)

        cbox = QGroupBox("控制 — 蜂鸣器 (占空比 0-50%)")
        cl = QVBoxLayout()
        row = QHBoxLayout()
        row.addWidget(QLabel("导通占空比:"))
        self._buzzer_slider = QSlider(Qt.Horizontal)
        self._buzzer_slider.setRange(0, 50)
        self._buzzer_slider.setValue(0)
        row.addWidget(self._buzzer_slider)
        self._buzzer_val = QLabel("0")
        self._buzzer_slider.valueChanged.connect(
            lambda v: self._buzzer_val.setText(str(v)))
        row.addWidget(self._buzzer_val)
        cl.addLayout(row)
        brow = QHBoxLayout()
        btn_buzzer = QPushButton("发送控制帧")
        btn_buzzer.clicked.connect(self._send_ctrl)
        btn_mute = QPushButton("静音")
        btn_mute.clicked.connect(lambda: self._buzzer_slider.setValue(0))
        brow.addWidget(btn_buzzer)
        brow.addWidget(btn_mute)
        brow.addStretch(1)
        cl.addLayout(brow)
        cbox.setLayout(cl)
        left.addWidget(cbox)
        left.addStretch(1)
        body.addLayout(left)

        # 中：数据面板（固定行槽位，未读取到的保持 "--"，不折叠）
        dbox = QGroupBox("数据")
        dd = QGridLayout()
        self._val: dict[str, QLabel] = {}
        rows: list[tuple[str, str, bool]] = [
            ("急停状态", "estop", True),
            ("12V 电源", "r12", False),
            ("24V 电源", "r24", False),
            ("VIN_DC-DC", "rvin", False),
            ("AUX 电源", "raux", False),
            ("MOTOR 电源", "rmotor", False),
            ("风扇 0", "fan0", False),
            ("风扇 1", "fan1", False),
            ("NTC1 连接", "ntc1", False),
            ("NTC2 连接", "ntc2", False),
            ("NTC1 温度", "t1", False),
            ("NTC2 温度", "t2", False),
            ("MCU 温度", "tm", False),
            ("VIN 电压", "v1", False),
            ("VIN_DC-DC 电压", "v2", False),
        ]
        for r, (title, key, _big) in enumerate(rows):
            name_lbl = QLabel(title)
            val_lbl = QLabel("--")
            val_lbl.setMinimumWidth(150)
            val_lbl.setStyleSheet("font-weight: bold;")
            dd.addWidget(name_lbl, r, 0)
            dd.addWidget(val_lbl, r, 1)
            self._val[key] = val_lbl
        dbox.setLayout(dd)
        body.addWidget(dbox, 1)

        # 右：日志
        logbox = QGroupBox("日志")
        lv = QVBoxLayout()
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        logtop = QHBoxLayout()
        logtop.addStretch(1)
        btn_clear = QPushButton("清空")
        btn_clear.clicked.connect(self._log.clear)
        logtop.addWidget(btn_clear)
        lv.addLayout(logtop)
        lv.addWidget(self._log)
        logbox.setLayout(lv)
        body.addWidget(logbox, 1)

        body.setStretch(1, 0)
        body.setStretch(2, 2)
        root.addLayout(body)

        self.setCentralWidget(central)
        self.setStyleSheet(STYLE)
        self._update_conn_ui(False, "")

    # ---------- 串口 ----------
    def _refresh_ports(self) -> None:
        current = self._port_cb.currentText()
        self._port_cb.clear()
        for p in list_ports.comports():
            self._port_cb.addItem(p.device)
        if current:
            self._port_cb.setCurrentText(current)

    def _on_connect(self) -> None:
        if self._worker and self._worker.isRunning():
            self._disconnect()
            return
        port = self._port_cb.currentText()
        if not port:
            self._log_line("请先选择串口", "warn")
            return
        self._worker = SerialWorker(port, int(self._baud_cb.currentText()))
        self._worker.connected.connect(self._on_connected)
        self._worker.frame_rx.connect(self._on_frame)
        self._worker.log_line.connect(self._log_line)
        self._worker.start()

    def _disconnect(self) -> None:
        if self._worker:
            self._worker.connected.disconnect()
            self._worker.frame_rx.disconnect()
            self._worker.log_line.disconnect()
            self._worker.stop()
            self._worker.wait(2000)
            self._worker = None
        self._update_conn_ui(False, "")

    def _on_connected(self, ok: bool, text: str) -> None:
        self._update_conn_ui(ok, text)
        if ok:
            self._log_line(text, "info")
        else:
            self._log_line(text, "error")
            if self._worker:
                self._worker.wait(500)
                self._worker = None
            self._update_conn_ui(False, "")

    def _update_conn_ui(self, ok: bool, text: str) -> None:
        self._conn_btn.setText("断开" if ok else "连接")
        self._port_cb.setEnabled(not ok)
        self._baud_cb.setEnabled(not ok)
        self._conn_btn.setObjectName("danger" if not ok else "")
        self._conn_btn.style().unpolish(self._conn_btn)
        self._conn_btn.style().polish(self._conn_btn)
        if ok:
            self.setWindowTitle(f"E1_MASTER_POWER_CTU — RS485 ({text})")

    # ---------- 发送 ----------
    def _worker_send(self, frame: bytes, tag: str = "") -> None:
        if not self._worker or not self._worker.isRunning():
            self._log_line("未连接", "warn")
            return
        if not self._worker.send(frame):
            self._log_line(f"发送失败: {tag or frame_hex(frame)}", "error")

    def _send_read_status(self) -> None:
        self._worker_send(build_read_status(), "读状态")

    def _send_read_temp(self) -> None:
        self._worker_send(build_read_temp(), "读温度")

    def _send_read_volt(self) -> None:
        self._worker_send(build_read_volt(), "读电压")

    def _send_ctrl(self) -> None:
        self._worker_send(build_ctrl(self._buzzer_slider.value()), "控制")

    # ---------- 自动轮询 ----------
    def _on_auto_poll_toggled(self, checked: bool) -> None:
        if checked:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._poll_once)
            self._timer.start(self._poll_ms.value())
        else:
            timer = getattr(self, "_timer", None)
            if timer:
                timer.stop()
                timer.deleteLater()
                self._timer = None

    def _poll_once(self) -> None:
        if not (self._worker and self._worker.isRunning()):
            return
        self._worker.send(build_read_status(), log=False)
        self._worker.send(build_read_temp(), log=False)
        self._worker.send(build_read_volt(), log=False)

    # ---------- 接收 ----------
    def _on_frame(self, frame: bytes) -> None:
        cmd = parse_cmd(frame)
        payload = parse_payload(frame)
        if cmd == CMD_ERR:
            code = payload[0] if payload else 0xFF
            self._log_line(f"错误应答 cmd=0x{cmd:02X} err=0x{code:02X} "
                           f"({ERR_TEXT.get(code, '未知')})", "warn")
            return
        base = cmd & ~CMD_REPLY_FLAG
        if base == CMD_READ_STATUS:
            self._apply_status(status_items(payload))
        elif base == CMD_READ_TEMP:
            if len(payload) >= 6:
                self._set_kv("t1", f"{unpack_i16_le(payload, 0) / 100.0:.2f} °C")
                self._set_kv("t2", f"{unpack_i16_le(payload, 2) / 100.0:.2f} °C")
                self._set_kv("tm", f"{unpack_i16_le(payload, 4) / 100.0:.2f} °C")
        elif base == CMD_READ_VOLT:
            if len(payload) >= 4:
                self._set_kv("v1", f"{unpack_u16_le(payload, 0)} mV")
                self._set_kv("v2", f"{unpack_u16_le(payload, 2)} mV")
        elif base == CMD_CTRL:
            self._log_line("控制 ACK 已收到", "info")
        else:
            self._log_line(f"收到未知帧 cmd=0x{cmd:02X}: {frame_hex(frame)}", "warn")

    def _apply_status(self, items: dict[str, bool]) -> None:
        """状态帧各槽位：true 用红色标注，false 显示“正常”绿色"""
        if items.get("estop"):
            self._set_kv("estop", "触发", "#E0563F")
        else:
            self._set_kv("estop", "释放", "#4C9B73")
        for key in ("r12", "r24", "rvin", "raux", "rmotor",
                    "fan0", "fan1", "ntc1", "ntc2"):
            if items.get(key):
                color = "#E0563F"
                text = "断开" if key.startswith("ntc") else "异常"
            else:
                color = "#4C9B73"
                text = "正常"
            self._set_kv(key, text, color)

    def _set_kv(self, key: str, text: str, color: str | None = None) -> None:
        lbl = self._val.get(key)
        if lbl is None:
            return
        if color:
            lbl.setText(f"<span style='color:{color};'>{text}</span>")
        else:
            lbl.setText(text)

    # ---------- 工具 ----------
    def _log_line(self, text: str, level: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        prefix = {"info": "", "warn": "[WARN] ", "error": "[ERR]  ", "tx": ""}[level]
        color = {"warn": "#D29A2E", "error": "#E0563F", "tx": "#7A8A9E"}.get(level)
        html = f"<span style='color:#9E9E9E'>{ts}</span> "
        if color:
            html += f"<span style='color:{color}'>{prefix}{text}</span>"
        else:
            html += text
        self._log.appendHtml(html)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._disconnect()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
