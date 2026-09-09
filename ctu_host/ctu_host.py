# -*- coding: utf-8 -*-
"""E1 CTU 电源板 RS485 双板调试上位机（E1_MASTER_POWER_CTU + E1_SLAVER_POWER_CTU 合并版）

特性：
  - 高精度定时器（Qt.PreciseTimer + Windows timeBeginPeriod(1) 1ms 分辨率）
  - 严格等间隔步进调度：帧间时间完全由定时器设定保证
  - 严格 5ms 应答超时丢包判定与终端打印输出
  - 终端详细打印协议解析失败原因（脏数据、帧头错、帧尾错、CRC错、未知地址）

依赖: pyserial + PySide6
运行: python ctu_host.py
"""

from __future__ import annotations

import sys
import time
from collections import deque

# 在 Windows 平台启用 1ms 级系统高精度时钟
if sys.platform == "win32":
    import ctypes
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QPlainTextEdit, QSlider,
    QCheckBox, QSpinBox, QSplitter,
)

from ctu_protocol import (
    DEV_ADDR_MASTER, DEV_ADDR_SLAVER, DEV_NAMES,
    CMD_ERR, CMD_REPLY_FLAG, CMD_NAME,
    CMD_READ_STATUS, CMD_READ_VOLT, CMD_READ_TEMP, CMD_CTRL, CMD_RESET_LATCH,
    ERR_TEXT, FrameParser,
    build_read_status, build_read_volt, build_read_temp,
    build_mst_ctrl, build_slv_ctrl, build_reset_latch,
    decode_mst_status, decode_mst_volt, decode_mst_temp,
    decode_slv_status, decode_slv_volt, decode_slv_temp,
    frame_hex, parse_addr, parse_cmd, parse_payload,
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
QPushButton#small { padding: 4px 8px; font-size: 12px; }
QComboBox, QSpinBox {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 5px;
    padding: 4px 8px;
}
QPlainTextEdit {
    background-color: #FDFCFA; border: 1px solid #E4DED4; border-radius: 6px;
    font-family: Consolas, monospace; font-size: 12px;
}
"""

POLL_REPLY_TIMEOUT_MS = 10  # 严格 10ms 超时阈值


# ---------------- 串口工作线程 ----------------
class SerialWorker(QThread):
    connected = Signal(bool, str)
    frame_rx = Signal(bytes, float)  # 信号增加时间戳参数：(frame, rx_timestamp)

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
                    # 关键优化：在从底层串口读到的第一时间打时间戳，消除 Qt 跨线程分发延迟
                    rx_ts = time.perf_counter()
                    for frame in self._parser.feed(data):
                        self.frame_rx.emit(frame, rx_ts)
                    # 读到数据时不休眠，尽快把剩余数据吞完
                    continue
                # 空闲时才轻微让出 CPU
                time.sleep(0.0005)
        finally:
            try:
                if self._serial:
                    self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None

    def send(self, data: bytes) -> bool:
        if self._serial is None:
            return False
        try:
            self._serial.write(data)
            return True
        except Exception as exc:  # noqa: BLE001
            self.connected.emit(False, f"发送失败: {exc}")
            return False


# ---------------- 单板面板基类 ----------------
class BoardPanel(QGroupBox):
    addr: int = 0
    tag_status = "读状态"
    tag_volt = "读电压"
    tag_temp = "读温度"

    def __init__(self, title: str, parent=None) -> None:
        super().__init__(title, parent)
        self._send_impl = None
        self._log_fn = None
        self._log_rx_fn = None
        self._log_warn_fn = None
        self._val: dict[str, QLabel] = {}
        self._poll_cb: QCheckBox | None = None
        self._body()

    def attach(self, send_fn, log_fn, log_rx_fn, log_warn_fn) -> None:
        self._send_impl = send_fn
        self._log_fn = log_fn
        self._log_rx_fn = log_rx_fn
        self._log_warn_fn = log_warn_fn

    def is_poll_enabled(self) -> bool:
        return self._poll_cb.isChecked() if self._poll_cb else False

    def _body(self) -> None:
        raise NotImplementedError

    def _send(self, frame: bytes, tag: str = "") -> None:
        if self._send_impl:
            self._send_impl(frame, tag)

    def apply_status(self, payload: bytes) -> None:
        raise NotImplementedError

    def apply_volt(self, payload: bytes) -> None:
        raise NotImplementedError

    def apply_temp(self, payload: bytes) -> None:
        raise NotImplementedError

    def _set_kv(self, key: str, text: str, color: str | None = None) -> None:
        lbl = self._val.get(key)
        if lbl is None:
            return
        if color:
            lbl.setText(f"<span style='color:{color};'>{text}</span>")
        else:
            lbl.setText(text)

    def handle_frame(self, frame: bytes) -> None:
        addr = parse_addr(frame)
        cmd = parse_cmd(frame)
        payload = parse_payload(frame)
        dev = DEV_NAMES.get(addr, f"addr=0x{addr:02X}")

        if self._log_rx_fn:
            if cmd == CMD_ERR:
                name = "错误应答"
            else:
                base = cmd & ~CMD_REPLY_FLAG
                if base in CMD_NAME:
                    name = CMD_NAME[base] + ("应答" if cmd & CMD_REPLY_FLAG else "命令")
                else:
                    name = f"未知(0x{cmd:02X})"
            self._log_rx_fn(f"RX {dev}: {frame_hex(frame)}  [{name}]")

        if cmd == CMD_ERR:
            content = payload[1:] if payload else b""
            code = content[0] if content else 0xFF
            msg = f"{dev} 错误应答 err=0x{code:02X} ({ERR_TEXT.get(code, '未知')})"
            print(f"[从机应答错误] {msg}")
            if self._log_warn_fn:
                self._log_warn_fn(msg)
            return

        content = payload[1:] if payload else b""
        base = cmd & ~CMD_REPLY_FLAG
        if base == CMD_READ_STATUS:
            self.apply_status(content)
        elif base == CMD_READ_VOLT:
            self.apply_volt(content)
        elif base == CMD_READ_TEMP:
            self.apply_temp(content)
        elif base == CMD_CTRL:
            if self._log_fn:
                self._log_fn(f"{dev} 控制 ACK 已收到", "info")
        elif base == CMD_RESET_LATCH:
            if self._log_fn:
                self._log_fn(f"{dev} 清除锁存 ACK 已收到", "info")
        else:
            msg = f"{dev} 收到未知命令 cmd=0x{cmd:02X}: {frame_hex(frame)}"
            print(f"[协议警告] {msg}")
            if self._log_warn_fn:
                self._log_warn_fn(msg)

    def _query_status(self) -> None:
        self._send(build_read_status(self.addr), self.tag_status)

    def _query_volt(self) -> None:
        self._send(build_read_volt(self.addr), self.tag_volt)

    def _query_temp(self) -> None:
        self._send(build_read_temp(self.addr), self.tag_temp)

    def poll_requests(self) -> list[bytes]:
        addr = self.addr
        return [build_read_status(addr), build_read_volt(addr),
                build_read_temp(addr)]


# ---------------- E1_MASTER 面板 ----------------
class MasterPanel(BoardPanel):
    addr = DEV_ADDR_MASTER

    def __init__(self, parent=None) -> None:
        super().__init__("E1_MASTER (0x01) · 主电源板", parent)

    def _body(self) -> None:
        root = QVBoxLayout(self)

        qbox = QGroupBox("查询")
        ql = QHBoxLayout()
        for text, fn in (("读系统状态", self._query_status),
                         ("读电压", self._query_volt),
                         ("读温度", self._query_temp)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            ql.addWidget(b)
        ql.addStretch(1)
        qbox.setLayout(ql)
        root.addWidget(qbox)

        cbox = QGroupBox("控制 — 蜂鸣器 + 保护锁存")
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
        btn_buzzer.clicked.connect(self._send_buzzer)
        btn_mute = QPushButton("静音")
        btn_mute.clicked.connect(lambda: self._buzzer_slider.setValue(0))
        brow.addWidget(btn_buzzer)
        brow.addWidget(btn_mute)
        brow.addStretch(1)
        cl.addLayout(brow)
        lrow = QHBoxLayout()
        btn_clear = QPushButton("清除保护锁存 (0x05)")
        btn_clear.setObjectName("danger")
        btn_clear.clicked.connect(self._send_clear_latch)
        lrow.addWidget(btn_clear)
        lrow.addStretch(1)
        cl.addLayout(lrow)
        cbox.setLayout(cl)
        root.addWidget(cbox)

        poll_row = QHBoxLayout()
        self._poll_cb = QCheckBox("自动轮询此板")
        self._poll_cb.setChecked(True)
        poll_row.addWidget(self._poll_cb)
        poll_row.addStretch(1)
        root.addLayout(poll_row)

        dbox = QGroupBox("数据")
        dd = QGridLayout()
        rows: list[tuple[str, str]] = [
            ("急停状态", "estop"),
            ("12V 电源", "r12"),
            ("24V 电源", "r24"),
            ("VIN_DC-DC", "rvin"),
            ("AUX 电源", "raux"),
            ("MOTOR 电源", "rmotor"),
            ("风扇 0", "fan0"),
            ("风扇 1", "fan1"),
            ("NTC1 连接", "ntc1"),
            ("NTC2 连接", "ntc2"),
            ("NTC1 温度", "t1"),
            ("NTC2 温度", "t2"),
            ("MCU 温度", "tm"),
            ("VIN 电压", "v1"),
            ("VIN_DC-DC 电压", "v2"),
        ]
        for r, (title, key) in enumerate(rows):
            name_lbl = QLabel(title)
            val_lbl = QLabel("--")
            val_lbl.setMinimumWidth(150)
            val_lbl.setStyleSheet("font-weight: bold;")
            dd.addWidget(name_lbl, r, 0)
            dd.addWidget(val_lbl, r, 1)
            self._val[key] = val_lbl
        dbox.setLayout(dd)
        root.addWidget(dbox)
        root.addStretch(1)

    def _send_buzzer(self) -> None:
        self._send(build_mst_ctrl(self._buzzer_slider.value(), self.addr),
                   "蜂鸣器控制")

    def _send_clear_latch(self) -> None:
        self._send(build_reset_latch(self.addr), "清保护锁存")

    def apply_status(self, payload: bytes) -> None:
        items = decode_mst_status(payload)
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

    def apply_volt(self, payload: bytes) -> None:
        v = decode_mst_volt(payload)
        self._set_kv("v1", f"{v.get('vin_mv', 0)} mV")
        self._set_kv("v2", f"{v.get('vin_dcdc_mv', 0)} mV")

    def apply_temp(self, payload: bytes) -> None:
        t = decode_mst_temp(payload)
        self._set_kv("t1", f"{t.get('ntc1_c', 0.0):.2f} °C")
        self._set_kv("t2", f"{t.get('ntc2_c', 0.0):.2f} °C")
        self._set_kv("tm", f"{t.get('mcu_c', 0.0):.2f} °C")


# ---------------- E1_SLAVER 面板 ----------------
class SlavePanel(BoardPanel):
    addr = DEV_ADDR_SLAVER
    tag_volt = "读电压"
    tag_temp = "读温度/VDDA"

    def __init__(self, parent=None) -> None:
        super().__init__("E1_SLAVER (0x02) · 副电源模块", parent)

    def _body(self) -> None:
        root = QVBoxLayout(self)

        qbox = QGroupBox("查询")
        ql = QHBoxLayout()
        for text, fn in (("读系统状态", self._query_status),
                         ("读电压", self._query_volt),
                         ("读温度/VDDA", self._query_temp)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            ql.addWidget(b)
        ql.addStretch(1)
        qbox.setLayout(ql)
        root.addWidget(qbox)

        cbox = QGroupBox("输出控制")
        cl = QVBoxLayout()

        mask_row = QHBoxLayout()
        mask_row.addWidget(QLabel("输出:"))
        self._out_checks: dict[str, QCheckBox] = {}
        for key, text in (("24v", "24V"), ("12v", "12V_ISO"),
                          ("lsd1", "LSD1"), ("lsd2", "LSD2")):
            cb = QCheckBox(text)
            mask_row.addWidget(cb)
            self._out_checks[key] = cb
        mask_row.addStretch(1)
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
        act_row.addWidget(btn_ctrl)
        act_row.addWidget(btn_all_on)
        act_row.addWidget(btn_all_off)
        act_row.addStretch(1)
        cl.addLayout(act_row)

        lrow = QHBoxLayout()
        btn_reset = QPushButton("清除锁存 (0x05)")
        btn_reset.setObjectName("danger")
        btn_reset.clicked.connect(self._send_clear_latch)
        lrow.addWidget(btn_reset)
        lrow.addStretch(1)
        cl.addLayout(lrow)

        cbox.setLayout(cl)
        root.addWidget(cbox)

        poll_row = QHBoxLayout()
        self._poll_cb = QCheckBox("自动轮询此板")
        self._poll_cb.setChecked(True)
        poll_row.addWidget(self._poll_cb)
        poll_row.addStretch(1)
        root.addLayout(poll_row)

        dbox = QGroupBox("数据")
        dd = QGridLayout()
        rows: list[tuple[str, str]] = [
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
        for r, (name, key) in enumerate(rows):
            name_lbl = QLabel(name)
            val_lbl = QLabel("--")
            val_lbl.setMinimumWidth(150)
            val_lbl.setStyleSheet("font-weight: bold;")
            dd.addWidget(name_lbl, r, 0)
            dd.addWidget(val_lbl, r, 1)
            self._val[key] = val_lbl
        dbox.setLayout(dd)
        root.addWidget(dbox)
        root.addStretch(1)

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
        self._send(build_slv_ctrl(self._out_mask(), self._duty_slider.value(),
                                  self.addr), "输出控制")

    def _set_all_outs(self) -> None:
        for cb in self._out_checks.values():
            cb.setChecked(True)

    def _clear_all_outs(self) -> None:
        for cb in self._out_checks.values():
            cb.setChecked(False)
        self._duty_slider.setValue(0)

    def _send_clear_latch(self) -> None:
        self._send(build_reset_latch(self.addr), "清锁存")

    def apply_status(self, payload: bytes) -> None:
        items = decode_slv_status(payload)
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

    def apply_volt(self, payload: bytes) -> None:
        v = decode_slv_volt(payload)
        for key in ("aux_mv", "motor_mv", "lsd1_mv", "lsd2_mv"):
            self._set_kv(key, f"{v.get(key, 0)} mV")

    def apply_temp(self, payload: bytes) -> None:
        d = decode_slv_temp(payload)
        self._set_kv("mcu_temp", f"{d.get('mcu_c', 0.0):.2f} °C")
        self._set_kv("vdda_mv", f"{d.get('vdda_mv', 0)} mV")


# ---------------- 主窗口 ----------------
class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("E1 CTU 电源板 RS485 调试上位机（MASTER + SLAVER）")
        self.resize(1280, 800)
        self._worker: SerialWorker | None = None

        # 高精度步进轮询定时器
        self._poll_timer = QTimer(self)
        self._poll_timer.setTimerType(Qt.PreciseTimer)
        self._poll_timer.timeout.connect(self._on_poll_step)
        self._poll_seq_idx = 0

        # 5ms 严格应答超时监测定时器
        self._loss_timeout_timer = QTimer(self)
        self._loss_timeout_timer.setTimerType(Qt.PreciseTimer)
        self._loss_timeout_timer.setSingleShot(True)
        self._loss_timeout_timer.timeout.connect(self._on_loss_timeout)

        # 待决应答追踪
        self._pending_poll_id: int | None = None
        self._pending_expected: tuple[int, int] | None = None
        self._pending_tx_ts: float = 0.0
        self._poll_id_counter: int = 0

        # 丢包与帧率统计
        self._stat_poll_tx_count: int = 0
        self._stat_poll_loss_count: int = 0
        self._stat_rx_total_count: int = 0
        self._rx_time_window: deque[float] = deque()

        # UI 周期刷新定时器 (200ms)
        self._ui_stats_timer = QTimer(self)
        self._ui_stats_timer.timeout.connect(self._update_stats_ui)
        self._ui_stats_timer.start(200)

        self._build_ui()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)

        title = QLabel("E1 CTU 电源板 RS485 调试上位机 · 主控 + 副电源同屏")
        title.setObjectName("title")
        root.addWidget(title)

        # 1. 连接栏
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
        self._poll_ms.setRange(5, 10000)
        self._poll_ms.setValue(100)
        self._poll_ms.setSuffix(" ms")
        self._poll_ms.valueChanged.connect(self._on_poll_ms_changed)
        conn.addWidget(self._poll_ms)
        self._show_frames = QCheckBox("显示收发帧")
        self._show_frames.setChecked(True)
        conn.addWidget(self._show_frames)
        root.addLayout(conn)

        # 2. 实时监测条 (5ms 判定)
        stats_box = QGroupBox("实时通信监测 (严格 5ms 判定)")
        stats_layout = QHBoxLayout()

        self._lbl_stat_fps = QLabel("RX 帧率: <b>0.0</b> fps")
        self._lbl_stat_loss = QLabel("丢包率: <b>0.00%</b> (0 / 0)")
        self._lbl_stat_tx = QLabel("轮询发包: <b>0</b>")
        self._lbl_stat_rx = QLabel("接收总数: <b>0</b>")

        btn_reset_stats = QPushButton("清零统计")
        btn_reset_stats.setObjectName("small")
        btn_reset_stats.clicked.connect(self._reset_stats)

        stats_layout.addWidget(self._lbl_stat_fps)
        stats_layout.addSpacing(20)
        stats_layout.addWidget(self._lbl_stat_loss)
        stats_layout.addSpacing(20)
        stats_layout.addWidget(self._lbl_stat_tx)
        stats_layout.addSpacing(20)
        stats_layout.addWidget(self._lbl_stat_rx)
        stats_layout.addStretch(1)
        stats_layout.addWidget(btn_reset_stats)

        stats_box.setLayout(stats_layout)
        root.addWidget(stats_box)

        # 3. 双板面板
        splitter = QSplitter(Qt.Horizontal)
        self._master_panel = MasterPanel()
        self._slave_panel = SlavePanel()
        splitter.addWidget(self._master_panel)
        splitter.addWidget(self._slave_panel)
        splitter.setSizes([600, 640])
        root.addWidget(splitter, 3)

        # 4. 日志
        logbox = QGroupBox("日志")
        lv = QVBoxLayout()
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        logtop = QHBoxLayout()
        logtop.addWidget(QLabel("RX 显示：帧名带 [应答] 后缀；0x7F=错误应答"))
        logtop.addStretch(1)
        btn_clear = QPushButton("清空")
        btn_clear.clicked.connect(self._log.clear)
        logtop.addWidget(btn_clear)
        lv.addLayout(logtop)
        lv.addWidget(self._log)
        logbox.setLayout(lv)
        root.addWidget(logbox, 1)

        self.setCentralWidget(central)
        self.setStyleSheet(STYLE)

        for panel in (self._master_panel, self._slave_panel):
            panel.attach(self._worker_send, self._log_line, self._log_rx,
                         self._log_warn)

        self._update_conn_ui(False, "")

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
        self._worker.start()

    def _disconnect(self) -> None:
        if self._auto_poll.isChecked():
            self._auto_poll.setChecked(False)
        self._poll_timer.stop()
        self._loss_timeout_timer.stop()
        self._pending_poll_id = None
        if self._worker:
            try:
                self._worker.connected.disconnect()
                self._worker.frame_rx.disconnect()
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
            self._reset_stats()
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
            self.setWindowTitle(f"E1 CTU 电源板 RS485 调试上位机 ({text})")

    def _worker_send(self, frame: bytes, tag: str = "") -> None:
        if not self._worker or not self._worker.isRunning():
            self._log_line("未连接", "warn")
            return
        if self.frames_log_enabled():
            text = f"TX  {tag}: {frame_hex(frame)}" if tag else f"TX  {frame_hex(frame)}"
            self._log_line(text, "tx")
        if not self._worker.send(frame):
            self._log_line(f"发送失败: {tag or frame_hex(frame)}", "error")

    # ---------- 统计重置与刷新 ----------
    def _reset_stats(self) -> None:
        self._stat_poll_tx_count = 0
        self._stat_poll_loss_count = 0
        self._stat_rx_total_count = 0
        self._rx_time_window.clear()
        self._pending_poll_id = None
        self._update_stats_ui()

    def _update_stats_ui(self) -> None:
        now = time.perf_counter()
        while self._rx_time_window and (now - self._rx_time_window[0] > 1.0):
            self._rx_time_window.popleft()
        fps = len(self._rx_time_window)

        tx = self._stat_poll_tx_count
        loss = self._stat_poll_loss_count
        rate = (loss / tx * 100.0) if tx > 0 else 0.0

        self._lbl_stat_fps.setText(f"RX 帧率: <b style='color:#3E7BB6;'>{fps:.1f}</b> fps")
        loss_color = "#4C9B73" if rate == 0 else ("#D29A2E" if rate < 5 else "#E0563F")
        self._lbl_stat_loss.setText(
            f"丢包率: <b style='color:{loss_color};'>{rate:.2f}%</b> ({loss} / {tx})")
        self._lbl_stat_tx.setText(f"轮询发包: <b>{tx}</b>")
        self._lbl_stat_rx.setText(f"接收总数: <b>{self._stat_rx_total_count}</b>")

    # ---------- 高精度轮询与 5ms 丢包监测 ----------
    def _on_auto_poll_toggled(self, checked: bool) -> None:
        if checked:
            if not (self._worker and self._worker.isRunning()):
                self._log_line("未连接串口，无法开启轮询", "warn")
                self._auto_poll.setChecked(False)
                return
            self._poll_seq_idx = 0
            self._poll_timer.start(self._poll_ms.value())
            self._on_poll_step()
        else:
            self._poll_timer.stop()
            self._loss_timeout_timer.stop()
            self._pending_poll_id = None

    def _on_poll_ms_changed(self, value: int) -> None:
        if self._auto_poll.isChecked() and self._poll_timer.isActive():
            self._poll_timer.setInterval(value)

    def _get_active_poll_sequence(self) -> list[bytes]:
        m_reqs = self._master_panel.poll_requests() if self._master_panel.is_poll_enabled() else []
        s_reqs = self._slave_panel.poll_requests() if self._slave_panel.is_poll_enabled() else []

        seq: list[bytes] = []
        width = max(len(m_reqs), len(s_reqs))
        for i in range(width):
            if i < len(m_reqs):
                seq.append(m_reqs[i])
            if i < len(s_reqs):
                seq.append(s_reqs[i])
        return seq

    def _on_poll_step(self) -> None:
        if not (self._worker and self._worker.isRunning()):
            return

        seq = self._get_active_poll_sequence()
        if not seq:
            return

        if self._poll_seq_idx >= len(seq):
            self._poll_seq_idx = 0

        frame = seq[self._poll_seq_idx]
        self._poll_seq_idx = (self._poll_seq_idx + 1) % len(seq)

        # 统计发包与待决应答记录
        self._stat_poll_tx_count += 1
        self._poll_id_counter += 1
        this_id = self._poll_id_counter
        self._pending_poll_id = this_id
        self._pending_expected = (parse_addr(frame), parse_cmd(frame) & ~CMD_REPLY_FLAG)
        self._pending_tx_ts = time.perf_counter()

        self._worker.send(frame)

        # 启动严格 5ms 超时判定
        self._loss_timeout_timer.start(POLL_REPLY_TIMEOUT_MS)

    def _on_loss_timeout(self) -> None:
        """5ms 定时器到点：未收到应答，精准记为丢包一次，并清理状态"""
        if self._pending_poll_id is not None:
            self._stat_poll_loss_count += 1
            # 状态重置，防止后续晚到的帧再次重复判定
            self._pending_poll_id = None
            self._pending_expected = None
    # ---------- 接收处理 ----------
    def frames_log_enabled(self) -> bool:
        return self._show_frames.isChecked() and not self._auto_poll.isChecked()

    def _on_frame(self, frame: bytes, rx_ts: float) -> None:
        # 1. 记录总接收与滑动窗口
        self._stat_rx_total_count += 1
        self._rx_time_window.append(rx_ts)

        addr = parse_addr(frame)
        cmd = parse_cmd(frame)

        # 2. 状态机严格防重入匹配
        if self._auto_poll.isChecked() and (self._pending_poll_id is not None):
            exp_addr, exp_cmd = self._pending_expected
            if addr == exp_addr:
                base = cmd & ~CMD_REPLY_FLAG
                if base == exp_cmd or cmd == CMD_ERR:
                    # 计算该帧真实往返耗时
                    elapsed_ms = (rx_ts - self._pending_tx_ts) * 1000.0
                    
                    # 停止 5ms 定时器
                    self._loss_timeout_timer.stop()

                    # 若在 5ms 内正常返回，判定成功；若超过 5ms，则记为超时丢包
                    if elapsed_ms <= float(POLL_REPLY_TIMEOUT_MS):
                        pass # 成功，不计丢包
                    else:
                        self._stat_poll_loss_count += 1

                    # 无论成功还是晚到，立即清空待决标记，绝不让状态跨帧污染
                    self._pending_poll_id = None
                    self._pending_expected = None

        # 3. 面板路由渲染（保持不变）
        if addr == DEV_ADDR_MASTER:
            self._master_panel.handle_frame(frame)
        elif addr == DEV_ADDR_SLAVER:
            self._slave_panel.handle_frame(frame)
        else:
            msg = f"收到未知设备帧 addr=0x{addr:02X}: {frame_hex(frame)}"
            self._log_warn(msg)
    # ---------- 日志与退出 ----------
    def _log_rx(self, text: str) -> None:
        if self.frames_log_enabled():
            self._log_line(text, "rx")

    def _log_warn(self, text: str) -> None:
        self._log_line(text, "warn")

    def _log_line(self, text: str, level: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        prefix = {"info": "", "warn": "[WARN] ", "error": "[ERR]  ",
                  "tx": "", "rx": ""}[level]
        color = {"warn": "#D29A2E", "error": "#E0563F", "tx": "#7A8A9E",
                 "rx": "#3E7BB6"}.get(level)
        html = f"<span style='color:#9E9E9E'>{ts}</span> "
        if color:
            html += f"<span style='color:{color}'>{prefix}{text}</span>"
        else:
            html += text
        self._log.appendHtml(html)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._disconnect()
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeEndPeriod(1)
            except Exception:
                pass
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())