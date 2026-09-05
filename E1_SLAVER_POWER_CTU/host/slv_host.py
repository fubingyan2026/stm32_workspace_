# -*- coding: utf-8 -*-
"""E1_SLAVER_POWER_CTU RS485 上位机 — 状态/电压/温度查询 + 输出控制/清锁存

依赖: pyserial + PySide6
运行: python slv_host.py
协议见 docs/protocol_slaver_485.md（与 slv_protocol.py 对齐）。
"""

from __future__ import annotations

import sys
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QPlainTextEdit, QSlider,
    QCheckBox, QSpinBox,
)

from slv_protocol import (
    CMD_ERR, CMD_REPLY_FLAG,
    CMD_READ_STATUS, CMD_READ_VOLT, CMD_READ_TEMP, CMD_CTRL, CMD_RESET_LATCH,
    ERR_TEXT, FrameParser, build_read_status, build_read_volt, build_read_temp,
    build_ctrl, build_reset_latch, status_items, decode_volt, decode_temp,
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
    log_line = Signal(str, str)     # (文本, 级别: info/warn/error/tx)

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
        self.setWindowTitle("E1_SLAVER_POWER_CTU — RS485 副电源模块上位机")
        self.resize(960, 680)
        self._worker: SerialWorker | None = None
        self._timer: QTimer | None = None
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        title = QLabel("E1_SLAVER_POWER_CTU · 副电源模块调试")
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
            ("读电压", self._send_read_volt),
            ("读温度/VDDA", self._send_read_temp),
        ):
            b = QPushButton(text)
            b.clicked.connect(fn)
            ql.addWidget(b)
        qbox.setLayout(ql)
        left.addWidget(qbox)

        cbox = QGroupBox("控制")
        cl = QVBoxLayout()

        mask_row = QHBoxLayout()
        mask_row.addWidget(QLabel("输出:"))
        self._out_checks: dict[str, QCheckBox] = {}
        for key, text in (("24v", "24V"), ("12v", "12V_ISO"),
                          ("lsd1", "LSD1"), ("lsd2", "LSD2")):
            cb = QCheckBox(text)
            mask_row.addWidget(cb)
            self._out_checks[key] = cb
        cl.addLayout(mask_row)

        duty_row = QHBoxLayout()
        duty_row.addWidget(QLabel("补光亮度:"))
        self._duty_slider = QSlider(Qt.Horizontal)
        self._duty_slider.setRange(0, 1000)
        self._duty_slider.setValue(0)
        duty_row.addWidget(self._duty_slider)
        self._duty_val = QLabel("0")
        self._duty_slider.valueChanged.connect(
            lambda v: self._duty_val.setText(f"{v} ({(v / 10):.1f}%)"))
        duty_row.addWidget(self._duty_val)
        cl.addLayout(duty_row)

        act_row = QHBoxLayout()
        btn_ctrl = QPushButton("发送输出控制")
        btn_ctrl.clicked.connect(self._send_ctrl)
        btn_all_on = QPushButton("全开")
        btn_all_on.clicked.connect(self._set_all_outs)
        btn_all_off = QPushButton("全关")
        btn_all_off.clicked.connect(self._clear_all_outs)
        btn_reset = QPushButton("清除锁存 (0x11)")
        btn_reset.setObjectName("danger")
        btn_reset.clicked.connect(self._send_reset_latch)
        act_row.addWidget(btn_ctrl)
        act_row.addWidget(btn_all_on)
        act_row.addWidget(btn_all_off)
        act_row.addWidget(btn_reset)
        act_row.addStretch(1)
        cl.addLayout(act_row)
        cbox.setLayout(cl)
        left.addWidget(cbox)
        left.addStretch(1)
        body.addLayout(left)

        # 中：数据面板（固定行槽位，未读取到的保持 "--"，不折叠）
        dbox = QGroupBox("数据")
        dd = QGridLayout()
        self._val: dict[str, QLabel] = {}

        status_rows = [
            ("24V 输出", "out_24v"),
            ("12V_ISO 输出", "out_12v"),
            ("LSD1 输出", "out_lsd1"),
            ("LSD2 输出", "out_lsd2"),
            ("24V 故障", "err_24v"),
            ("12V_ISO 故障", "err_12v"),
            ("LSD1 故障", "err_lsd1"),
            ("LSD2 故障", "err_lsd2"),
            ("AUX 输入", "err_aux"),
            ("MOTOR 输入", "err_motor"),
            ("故障锁存", "latch_active"),
            ("AUX 电压", "aux_mv"),
            ("MOTOR 电压", "motor_mv"),
            ("LSD1 电压", "lsd1_mv"),
            ("LSD2 电压", "lsd2_mv"),
            ("MCU 温度", "mcu_temp"),
            ("VDDA", "vdda_mv"),
        ]
        for r, (name, key) in enumerate(status_rows):
            name_lbl = QLabel(name)
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

        body.setStretch(0, 0)
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
        if self._auto_poll.isChecked():
            self._auto_poll.setChecked(False)
        if self._worker:
            try:
                self._worker.connected.disconnect()
                self._worker.frame_rx.disconnect()
                self._worker.log_line.disconnect()
            except RuntimeError:
                pass
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
            self.setWindowTitle(f"E1_SLAVER_POWER_CTU — RS485 ({text})")

    # ---------- 发送 ----------
    def _worker_send(self, frame: bytes, tag: str = "") -> None:
        if not self._worker or not self._worker.isRunning():
            self._log_line("未连接", "warn")
            return
        if not self._worker.send(frame):
            self._log_line(f"发送失败: {tag or frame_hex(frame)}", "error")

    def _send_read_status(self) -> None:
        self._worker_send(build_read_status(), "读状态")

    def _send_read_volt(self) -> None:
        self._worker_send(build_read_volt(), "读电压")

    def _send_read_temp(self) -> None:
        self._worker_send(build_read_temp(), "读温度")

    def _out_mask(self) -> int:
        mask = 0
        if self._out_checks["24v"].isChecked():
            mask |= 0x01
        if self._out_checks["12v"].isChecked():
            mask |= 0x02
        if self._out_checks["lsd1"].isChecked():
            mask |= 0x04
        if self._out_checks["lsd2"].isChecked():
            mask |= 0x08
        return mask

    def _send_ctrl(self) -> None:
        self._worker_send(build_ctrl(self._out_mask(), self._duty_slider.value()),
                          "输出控制")

    def _set_all_outs(self) -> None:
        for cb in self._out_checks.values():
            cb.setChecked(True)

    def _clear_all_outs(self) -> None:
        for cb in self._out_checks.values():
            cb.setChecked(False)
        self._duty_slider.setValue(0)

    def _send_reset_latch(self) -> None:
        self._worker_send(build_reset_latch(), "清锁存")

    # ---------- 自动轮询 ----------
    def _on_auto_poll_toggled(self, checked: bool) -> None:
        if checked:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._poll_once)
            self._timer.start(self._poll_ms.value())
            self._poll_once()
        else:
            timer = self._timer
            if timer:
                timer.stop()
                timer.deleteLater()
                self._timer = None

    def _poll_once(self) -> None:
        if not (self._worker and self._worker.isRunning()):
            return
        self._worker.send(build_read_status(), log=False)
        self._worker.send(build_read_volt(), log=False)
        self._worker.send(build_read_temp(), log=False)

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
        elif base == CMD_READ_VOLT:
            self._apply_volt(decode_volt(payload))
        elif base == CMD_READ_TEMP:
            self._apply_temp(decode_temp(payload))
        elif base == CMD_CTRL:
            self._log_line("输出控制 ACK 已收到", "info")
        elif base == CMD_RESET_LATCH:
            self._log_line("清除锁存 ACK 已收到", "info")
        else:
            self._log_line(f"收到未知帧 cmd=0x{cmd:02X}: {frame_hex(frame)}", "warn")

    def _apply_status(self, items: dict[str, bool]) -> None:
        out_map = {"out_24v": "24v", "out_12v": "12v",
                   "out_lsd1": "lsd1", "out_lsd2": "lsd2"}
        for key, cb_key in out_map.items():
            ok = bool(items.get(key))
            self._set_kv(key, "开" if ok else "关",
                         "#4C9B73" if ok else "#9E9E9E")

        for key in ("err_24v", "err_12v", "err_lsd1", "err_lsd2",
                    "err_aux", "err_motor"):
            bad = bool(items.get(key))
            self._set_kv(key, "异常" if bad else "正常",
                         "#E0563F" if bad else "#4C9B73")

        latched = bool(items.get("latch_active"))
        self._set_kv("latch_active", "锁存" if latched else "无",
                     "#E0563F" if latched else "#4C9B73")

    def _apply_volt(self, v: dict[str, int]) -> None:
        for key in ("aux_mv", "motor_mv", "lsd1_mv", "lsd2_mv"):
            self._set_kv(key, f"{v.get(key, 0)} mV")

    def _apply_temp(self, d: dict[str, float | int]) -> None:
        self._set_kv("mcu_temp", f"{d.get('mcu_temp_c', 0.0):.2f} °C")
        self._set_kv("vdda_mv", f"{d.get('vdda_mv', 0)} mV")

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
        prefix = {"info": "", "warn": "[WARN] ", "error": "[ERR]  ",
                  "tx": ""}[level]
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
