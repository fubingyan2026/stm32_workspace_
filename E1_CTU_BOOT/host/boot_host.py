# -*- coding: utf-8 -*-
"""E1_CTU_BOOT RS485 固件升级上位机（GUI）

界面风格参考兄弟工程合并后的 `ctu_host/ctu_host.py`（米色底 + 绿色强调）。

功能:
  1. 连接串口（USB-RS485，115200-8N1）做被动监听
  2. 统一先发一帧 0x06：运行中的 App 会复位进入 Boot；已在 Boot 则等同 SELECT（幂等）
  3. 选择 .bin 固件 → 寻址分块传输（SELECT/START/DATA/END）→ 板端校验提升后复位

依赖: pyserial + PySide6
运行: python host/boot_host.py
协议见 docs/boot_485_ymodem.md；协议层在 host/boot_protocol.py。
"""

from __future__ import annotations

import os
import sys
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QPlainTextEdit,
    QFileDialog, QProgressBar,
)

from boot_protocol import (
    DEVICE_BOOT, DEVICE_MASTER, DEVICE_SLAVER, DEVICE_NAMES,
    MAX_FW_SIZE, BootProtoSender, check_app_image, request_boot,
)

STYLE = """
QMainWindow, QWidget { background-color: #F5F0E8; color: #3A3A3A; font-size: 13px; }
QGroupBox {
    background-color: #FFFFFF; border: 1px solid #E4DED4; border-radius: 8px;
    margin-top: 12px; padding: 8px;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4C9B73; font-weight: bold; }
QLabel#title { font-size: 20px; font-weight: bold; color: #4C9B73; }
QLabel#phase { color: #4C9B73; font-weight: bold; }
QPushButton {
    background-color: #7EC8A0; color: white; border: none; border-radius: 6px;
    padding: 7px 14px; font-weight: bold;
}
QPushButton:hover { background-color: #6DB88F; }
QPushButton:disabled { background-color: #D9D4C8; color: #9E9E9E; }
QPushButton#danger { background-color: #F2A999; }
QPushButton#danger:hover { background-color: #ED9481; }
QPushButton#neutral { background-color: #E4DED4; color: #3A3A3A; }
QPushButton#neutral:hover { background-color: #D9D2C6; }
QComboBox {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 5px;
    padding: 4px 8px;
}
QLineEdit {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 5px;
    padding: 4px 8px;
}
QPlainTextEdit {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 6px;
    font-family: Consolas, monospace; font-size: 12px;
}
QProgressBar {
    border: 1px solid #E4DED4; border-radius: 6px; background: #FDFCFA;
    text-align: center;
}
QProgressBar::chunk { background-color: #7EC8A0; border-radius: 6px; }
"""


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


# ---------------- 被动监听线程（连接/断开用，独占串口） ----------------
class MonitorWorker(QThread):
    connected = Signal(bool, str)   # (成功, 描述)
    log_line = Signal(str, str)     # (文本, 级别)
    rx_hex = Signal(str)            # 收到字节 hex（被动观察）

    def __init__(self, port: str, baud: int, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
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
                    if data:
                        self.rx_hex.emit(_hex(data[:128]))
                time.sleep(0.002)
        finally:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None


# ---------------- 升级工作线程（独占串口，跑完即关） ----------------
class UpgradeWorker(QThread):
    log_line = Signal(str, str)     # (文本, 级别)
    progress = Signal(int)          # 0..100
    phase = Signal(str)
    done = Signal(bool, str)        # (成功, 描述)

    def __init__(self, port: str, baud: int, device: str,
                 filepath: str, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._device = device
        self._filepath = filepath
        self._serial: serial.Serial | None = None
        self._sender: BootProtoSender | None = None

    def cancel(self) -> None:
        if self._sender:
            self._sender.cancel()

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self._port, self._baud,
                                         timeout=0, write_timeout=0.5)
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"连接失败: {exc}", "error")
            self.done.emit(False, f"连接失败: {exc}")
            return
        self.log_line.emit(f"已打开 {self._port} @ {self._baud}", "info")

        ok = False
        try:
            # 统一先发一帧 0x06：App 会复位进入 Boot；已在 Boot 则等同 SELECT（幂等）
            dev = DEVICE_NAMES.get(self._device, self._device)
            self.log_line.emit(f"发送 0x06 邀请给 {dev}...", "info")
            if not request_boot(self._serial, self._device,
                                lambda t, l: self.log_line.emit(t, l)):
                self.done.emit(False, "0x06 邀请发送失败")
                return
            self.phase.emit("等待板端进入/确认 Boot...")
            time.sleep(0.5)

            self._sender = BootProtoSender(
                self._serial,
                log_cb=lambda t, l: self.log_line.emit(t, l),
                phase_cb=lambda t: self.phase.emit(t),
                progress_cb=lambda f: self.progress.emit(int(f * 100)))
            ok = self._sender.transfer(self._device, self._filepath)
        except Exception as exc:  # noqa: BLE001
            self.log_line.emit(f"异常: {exc}", "error")
        finally:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None
            self._sender = None

        self.done.emit(ok, "升级完成，板端复位运行新固件" if ok else "升级失败")


# ---------------- 主窗口 ----------------
class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("E1_CTU_BOOT — RS485 固件升级")
        self.resize(860, 640)
        self._monitor: MonitorWorker | None = None
        self._worker: UpgradeWorker | None = None
        self._filepath: str = ""
        self._app_ok: bool = False
        self._log: QPlainTextEdit | None = None
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        title = QLabel("E1_CTU_BOOT · RS485 固件升级")
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
        for b in (9600, 115200, 460800, 921600):
            self._baud_cb.addItem(str(b))
        self._baud_cb.setCurrentText("115200")
        conn.addWidget(self._baud_cb)
        self._conn_btn = QPushButton("连接")
        self._conn_btn.setObjectName("danger")
        self._conn_btn.clicked.connect(self._on_connect)
        conn.addWidget(self._conn_btn)
        conn.addStretch(1)
        root.addLayout(conn)

        # 升级设置
        box = QGroupBox("升级设置")
        bl = QGridLayout()
        bl.addWidget(QLabel("目标设备:"), 0, 0)
        self._device_cb = QComboBox()
        for dev in (DEVICE_MASTER, DEVICE_SLAVER, DEVICE_BOOT):
            self._device_cb.addItem(DEVICE_NAMES[dev], dev)
        self._device_cb.setCurrentIndex(2)  # 默认直连 Boot
        bl.addWidget(self._device_cb, 0, 1, 1, 2)

        bl.addWidget(QLabel("固件文件:"), 2, 0)
        self._file_lbl = QLabel("--")
        self._file_lbl.setMinimumWidth(320)
        bl.addWidget(self._file_lbl, 2, 1)
        btn_browse = QPushButton("选择 .bin")
        btn_browse.clicked.connect(self._on_browse)
        bl.addWidget(btn_browse, 2, 2)

        self._size_lbl = QLabel("")
        bl.addWidget(self._size_lbl, 3, 1, 1, 2)
        box.setLayout(bl)
        root.addWidget(box)

        # 操作/进度
        action = QHBoxLayout()
        self._start_btn = QPushButton("开始升级")
        self._start_btn.clicked.connect(self._on_start)
        action.addWidget(self._start_btn)
        self._cancel_btn = QPushButton("中止")
        self._cancel_btn.setObjectName("danger")
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.clicked.connect(self._on_cancel)
        action.addWidget(self._cancel_btn)
        action.addStretch(1)
        btn_clear = QPushButton("清空日志")
        btn_clear.setObjectName("neutral")
        btn_clear.clicked.connect(self._on_clear_log)
        action.addWidget(btn_clear)
        root.addLayout(action)

        self._phase_lbl = QLabel("待命")
        self._phase_lbl.setObjectName("phase")
        root.addWidget(self._phase_lbl)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        root.addWidget(self._progress)

        logbox = QGroupBox("日志")
        lv = QVBoxLayout()
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        lv.addWidget(self._log)
        logbox.setLayout(lv)
        root.addWidget(logbox, 1)

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
        if self._monitor and self._monitor.isRunning():
            self._disconnect_monitor()
            return
        port = self._port_cb.currentText()
        if not port:
            self._log_line("请先选择串口", "warn")
            return
        self._monitor = MonitorWorker(port, int(self._baud_cb.currentText()))
        self._monitor.connected.connect(self._on_monitor_connected)
        self._monitor.log_line.connect(self._log_line)
        self._monitor.rx_hex.connect(
            lambda h: self._log_line(f"RX  {h}", "tx"))
        self._monitor.start()

    def _disconnect_monitor(self) -> None:
        m = self._monitor
        if m:
            try:
                m.connected.disconnect()
                m.log_line.disconnect()
                m.rx_hex.disconnect()
            except RuntimeError:
                pass
            m.stop()
            m.wait(2000)
            self._monitor = None
        self._update_conn_ui(False, "")

    def _on_monitor_connected(self, ok: bool, text: str) -> None:
        self._update_conn_ui(ok, text)
        self._log_line(text, "info" if ok else "error")
        if not ok:
            if self._monitor:
                self._monitor.wait(500)
                self._monitor = None
            self._update_conn_ui(False, "")

    def _update_conn_ui(self, ok: bool, text: str) -> None:
        self._conn_btn.setText("断开" if ok else "连接")
        self._port_cb.setEnabled(not ok)
        self._baud_cb.setEnabled(not ok)
        self._device_cb.setEnabled(not ok)
        self._conn_btn.setObjectName("danger" if not ok else "")
        self._conn_btn.style().unpolish(self._conn_btn)
        self._conn_btn.style().polish(self._conn_btn)
        if ok:
            self.setWindowTitle(f"E1_CTU_BOOT — RS485 固件升级 ({text})")

    # ---------- 文件/设备 ----------
    def _on_browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择固件", "",
            "固件 (*.bin *.hex *.elf);;所有文件 (*)")
        if not path:
            return
        self._filepath = path
        self._file_lbl.setText(os.path.basename(path))
        self._file_lbl.setToolTip(path)
        size = os.path.getsize(path)
        ok = size <= MAX_FW_SIZE
        note = "" if ok else "（超出 96KB App 分区容量！）"
        color = "#4C9B73" if ok else "#E0563F"

        # 固件向量表预检：防止误发 Boot 镜像或链接地址不符的 bin
        try:
            with open(path, "rb") as f:
                reason = check_app_image(f.read())
        except OSError as exc:  # noqa: BLE001
            reason = f"读取失败: {exc}"
        self._app_ok = ok and (reason is None)
        if reason is not None:
            color = "#E0563F"
            note = f"（{reason}）"
            self._log_line(f"固件预检失败：{reason}", "error")

        self._size_lbl.setText(
            f"<span style='color:{color};'>大小 {size}B{note}</span>")
        if not ok:
            self._log_line(f"固件 {size}B 超出 App 分区容量 96KB", "error")

    # ---------- 升级控制 ----------
    def _on_start(self) -> None:
        if not self._filepath:
            self._log_line("请先选择固件 .bin", "warn")
            return
        if not self._app_ok:
            self._log_line("固件未通过预检（非 AppA 链接的有效固件），已阻止升级", "error")
            return
        if self._worker and self._worker.isRunning():
            self._log_line("升级进行中，请先中止", "warn")
            return
        port = self._port_cb.currentText()
        if not port:
            self._log_line("请先选择串口", "warn")
            return

        # 释放监听连接，避免双开同一串口
        if self._monitor and self._monitor.isRunning():
            self._disconnect_monitor()

        dev = self._device_cb.currentData()
        if dev == DEVICE_BOOT:
            self._log_line("目标为广播(0x00)：仅适用于已在 Boot/空片；"
                           "升级运行中的 App 请选 Master/Slaver", "info")

        self._log_line(f"开始升级：{os.path.basename(self._filepath)}", "info")
        self._worker = UpgradeWorker(port, int(self._baud_cb.currentText()),
                                     dev, self._filepath)
        self._worker.log_line.connect(self._log_line)
        self._worker.progress.connect(self._progress.setValue)
        self._worker.phase.connect(self._phase_lbl.setText)
        self._worker.done.connect(self._on_done)
        self._start_btn.setEnabled(False)
        self._cancel_btn.setEnabled(True)
        self._port_cb.setEnabled(False)
        self._baud_cb.setEnabled(False)
        self._conn_btn.setEnabled(False)
        self._progress.setValue(0)
        self._phase_lbl.setText("开始...")
        self._worker.start()

    def _on_cancel(self) -> None:
        if self._worker and self._worker.isRunning():
            self._worker.cancel()
            self._log_line("请求中止（发送 CAN）...", "warn")

    def _on_done(self, ok: bool, text: str) -> None:
        self._log_line(text, "info" if ok else "error")
        self._phase_lbl.setText("完成" if ok else "失败")
        self._start_btn.setEnabled(True)
        self._cancel_btn.setEnabled(False)
        self._conn_btn.setEnabled(True)
        self._port_cb.setEnabled(True)
        self._baud_cb.setEnabled(True)
        self._device_cb.setEnabled(True)
        if not ok:
            self._progress.setValue(0)
        self._worker = None

    # ---------- 工具 ----------
    def _on_clear_log(self) -> None:
        if self._log is not None:
            self._log.clear()

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
        if self._worker and self._worker.isRunning():
            self._worker.cancel()
            self._worker.wait(2000)
        self._disconnect_monitor()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
