#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
橘虾(juxie) MIT 电机 + Mz 扭矩传感器 — 串口上位机
=================================================
协议依据 docs/juxie_mz_uart_protocol.md（20 字节定长帧，USART1 1M bps）。

依赖：
    pip install pyserial
    tkinter（Python 标准库自带）

功能：
  - MIT 控制：位置用滑块设定（滑动事件即时下发一帧控制数据），速度/Kp/Kd/力矩
    按物理量输入，支持单次或连续下发，Kp/Kd/目标可随时动态修改
  - 电机 使能/失能/抱闸/清错，量程(Pos_Max/Vel_Max/T_Max)在线设置
  - 读取电机反馈/状态、Mz 扭矩；自动刷新或流式显示
  - 扭矩传感器标零（0xE4）

用法：
    python juxie_host.py
"""

import queue
import struct
import threading
import time

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import serial
from serial.tools import list_ports

# =============================================================================
# 协议层（docs/juxie_mz_uart_protocol.md）
# =============================================================================

HEAD = bytes((0x55, 0xAA, 0x00, 0x14))
FRAME_LEN = 20

# 命令 can_id（主机→设备）
CID_MIT_CTRL = 0x000001D0
CID_MIT_CFG = 0x000001D1
CID_MOTOR_ZERO = 0x000001D2
CID_GET_FB = 0x000001E0
CID_GET_ST = 0x000001E1
CID_GET_MZ = 0x000000E2
CID_STREAM = 0x000000E3
CID_ZERO_MZ = 0x000000E4

# 应答 can_id（设备→主机）
CID_RSP_FB = 0x00E00100
CID_RSP_ST = 0x00E10100
CID_RSP_MZ = 0x00E20000
CID_RSP_STREAM = 0x00E30000
CID_ACK = 0x00FF0000

# 电机配置参数索引（srv_juxie_motor.h）
PARAM_ENABLE = 1
PARAM_CLEAR_ERR = 2
PARAM_BRAKE = 3
PARAM_POS_MAX = 4  # 0.1°
PARAM_VEL_MAX = 5  # rpm
PARAM_TQ_MAX = 6  # 0.01Nm

# juxie MIT 参数满量程（§4.1）
KP_FULL = 500.0
KD_FULL = 5.0


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_frame(cid: int, data: bytes = b"") -> bytes:
    """构建 20 字节帧：[55 AA 00 14][can_id 4B LE][data 8B][CRC16 4B LE]"""
    if len(data) > 8:
        raise ValueError("data 超过 8 字节")
    payload = struct.pack("<I", cid) + bytes(data).ljust(8, b"\x00")
    crc = crc16_ccitt_false(payload)
    return HEAD + payload + struct.pack("<I", crc)


def unpack_frame(frame: bytes):
    """校验并解析 20 字节帧，返回 (can_id, data8)；非法帧抛 ValueError"""
    if len(frame) != FRAME_LEN:
        raise ValueError("帧长度错误")
    if frame[:4] != HEAD:
        raise ValueError("帧头不匹配")
    if crc16_ccitt_false(frame[4:16]) != (frame[16] | (frame[17] << 8)):
        raise ValueError("CRC 错误")
    cid = struct.unpack("<I", frame[4:8])[0]
    return cid, bytes(frame[8:16])


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _to_i16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


# ---- 命令构造 ----------------------------------------------------------------

def mit_ctrl(pos_raw: int, vel_raw: int, kp_raw: int, kd_raw: int, tq_raw: int) -> bytes:
    """juxie 载荷 Byte[1..8]：pos16 大端 + vel/kp/kd/tq 12bit（§4.1）"""
    d = bytearray(8)
    d[0] = (pos_raw >> 8) & 0xFF
    d[1] = pos_raw & 0xFF
    d[2] = (vel_raw >> 4) & 0xFF
    d[3] = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)
    d[4] = kp_raw & 0xFF
    d[5] = (kd_raw >> 4) & 0xFF
    d[6] = ((kd_raw & 0x0F) << 4) | ((tq_raw >> 8) & 0x0F)
    d[7] = tq_raw & 0xFF
    return pack_frame(CID_MIT_CTRL, bytes(d))


def motor_cfg(param: int, value: int) -> bytes:
    v = int(value) & 0xFFFF
    return pack_frame(CID_MIT_CFG, bytes((param, v & 0xFF, (v >> 8) & 0xFF, 0, 0, 0, 0, 0)))


def cmd_get_fb() -> bytes:
    return pack_frame(CID_GET_FB, b"\x01\x00\x00\x00\x00\x00\x00\x00")


def cmd_get_st() -> bytes:
    return pack_frame(CID_GET_ST, b"\x01\x00\x00\x00\x00\x00\x00\x00")


def cmd_get_mz() -> bytes:
    return pack_frame(CID_GET_MZ)


def cmd_motor_zero() -> bytes:
    """电机标零：固件下发 SDO 写 CAN 0x601 / 0x2531=1，当前角度置零"""
    return pack_frame(CID_MOTOR_ZERO)


def cmd_stream(on: bool, interval_ms: int) -> bytes:
    return pack_frame(CID_STREAM, bytes((1 if on else 0, interval_ms & 0xFF, 0, 0, 0, 0, 0, 0)))


def cmd_zero_mz() -> bytes:
    return pack_frame(CID_ZERO_MZ)


# ---- 物理量 → 原始值（12/16bit 归一化） --------------------------------------

def to_pos_raw(deg: float, pos_max_deg: float) -> int:
    pm = float(pos_max_deg)
    return int(_clamp((deg / pm + 1.0) / 2.0 * 65535.0, 0, 65535))


def to_vel_raw(rpm: float, vel_max_rpm: float) -> int:
    vm = float(vel_max_rpm)
    return int(_clamp((rpm / vm + 1.0) / 2.0 * 4095.0, 0, 4095))


def to_tq_raw(nm: float, tq_max_nm: float) -> int:
    tm = float(tq_max_nm)
    return int(_clamp((nm / tm + 1.0) / 2.0 * 4095.0, 0, 4095))


def to_kp_raw(kp: float) -> int:
    return int(_clamp(kp / KP_FULL * 4095.0, 0, 4095))


def to_kd_raw(kd: float) -> int:
    return int(_clamp(kd / KD_FULL * 4095.0, 0, 4095))


# ---- 应答解析 ----------------------------------------------------------------

def parse_fb(data: bytes):
    """0x00E00100：pos(2B 0.01°) + speed(2B rpm) + iq(2B mA) + tq(2B 0.01Nm) 小端"""
    pos = _to_i16(data[0] | (data[1] << 8))
    speed = _to_i16(data[2] | (data[3] << 8))
    iq = _to_i16(data[4] | (data[5] << 8))
    tq = _to_i16(data[6] | (data[7] << 8))
    return {"pos_deg": pos / 100.0, "speed_rpm": speed, "iq_ma": iq, "tq_nm": tq / 100.0}


def parse_st(data: bytes):
    """0x00E10100：err(2B) + temp(2B 0.1℃) + mode(1B) + status(1B)"""
    err = data[0] | (data[1] << 8)
    temp = _to_i16(data[2] | (data[3] << 8)) / 10.0
    mode = data[4]
    status = data[5]
    return {
        "err": err,
        "temp_c": temp,
        "mode": mode,
        "status": status,
        "en": bool(status & 0x80),
        "brake": bool(status & 0x40),
        "err_bit": bool(status & 0x20),
        "in_pos": bool(status & 0x10),
    }


def parse_mz(data: bytes):
    """0x00E20000：mz(4B float32 LE) + seq(2B) + flags(1B)"""
    mz = struct.unpack("<f", bytes(data[0:4]))[0]
    seq = data[4] | (data[5] << 8)
    flags = data[6]
    return {"mz_nm": mz, "seq": seq, "online": bool(flags & 0x01), "zero_ok": bool(flags & 0x02)}


def parse_stream(data: bytes):
    """0x00E30000：mz(4B float32 LE) + pos(2B 0.01°) + tq(2B 0.01Nm)"""
    mz = struct.unpack("<f", bytes(data[0:4]))[0]
    pos = _to_i16(data[4] | (data[5] << 8)) / 100.0
    tq = _to_i16(data[6] | (data[7] << 8)) / 100.0
    return {"mz_nm": mz, "pos_deg": pos, "tq_nm": tq}


def parse_ack(data: bytes):
    return {"ok": bool(data[0]), "echo": data[1]}


# =============================================================================
# 串口帧解析器 / 串口链路
# =============================================================================

class FrameParser:
    """把字节流切分为合法 20 字节帧，坏帧自动重同步"""

    def __init__(self):
        self.buf = bytearray()
        self.bad_cnt = 0  # 被丢弃的坏帧计数

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(HEAD)
            if idx < 0:
                self.buf = self.buf[-3:]  # 保留可能跨边界的帧头尾部
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < FRAME_LEN:
                break
            frame = bytes(self.buf[:FRAME_LEN])
            try:
                out.append(unpack_frame(frame))
            except ValueError:
                self.bad_cnt += 1
                del self.buf[:1]  # 坏帧：丢 1 字节继续重同步
                continue
            del self.buf[:FRAME_LEN]
        return out


class SerialLink:
    """串口 + 后台接收线程（帧解析后压入队列，可显示原始字节）"""

    def __init__(self, port: str, baud: int = 1000000):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rx_q = queue.Queue()
        self.tx_count = 0
        self.rx_bytes = 0  # 收到的原始字节数
        self.rx_frames = 0  # 解析出的有效帧数
        self.rx_bad = 0  # 坏帧数
        self.raw_debug = False  # 是否把原始字节也放入队列显示
        self._alive = False
        self._thread = None

    def start(self):
        self._alive = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._alive = False
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _rx_loop(self):
        parser = FrameParser()
        while self._alive:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    self.rx_bytes += len(data)
                    if self.raw_debug:
                        self.rx_q.put(("raw", data.hex(" ").upper()))
                    bad0 = parser.bad_cnt
                    frames = parser.feed(data)
                    self.rx_bad += parser.bad_cnt - bad0
                    for cid, d in frames:
                        self.rx_q.put(("frame", cid, d))
                    self.rx_frames += len(frames)
                else:
                    time.sleep(0.001)
            except Exception as exc:
                self.rx_q.put(("err", str(exc)))
                break

    def send(self, frame: bytes) -> bool:
        if self.ser and self.ser.is_open:
            self.ser.write(frame)
            self.tx_count += 1
            return True
        return False


# =============================================================================
# 上位机 GUI
# =============================================================================

class JuxieHostApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("橘虾 MIT 电机 + Mz 扭矩传感器 上位机")
        root.geometry("920x760")
        root.minsize(760, 600)

        self.link = None
        self.cfg = {"pos_max": 180.0, "vel_max": 3000.0, "tq_max": 50.0}
        self.mit_on = False
        self.auto_refresh = False
        self._last_mit = 0.0
        self._last_refresh = 0.0
        self._mz_rate_cnt = 0   # 收到 Mz 帧计数（用于显示实际更新率）
        self._mz_rate_t0 = time.time()

        self._build_ui()
        self._update_connect_state()
        self._tick()

    # ---------------- UI ----------------

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True)

        self._tab_ctrl = ttk.Frame(nb)
        self._tab_mon = ttk.Frame(nb)
        nb.add(self._tab_ctrl, text="控制")
        nb.add(self._tab_mon, text="监控 / 日志")

        self._build_serial_bar()
        self._build_control_tab()
        self._build_monitor_tab()

    def _build_serial_bar(self):
        bar = ttk.LabelFrame(self.root, text="串口")
        bar.pack(fill="x", padx=6, pady=4)

        ttk.Label(bar, text="端口:").pack(side="left", padx=(6, 2))
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=10, state="readonly")
        self.port_cb.pack(side="left")
        self.port_cb.bind("<<ComboboxSelected>>", lambda e: self._update_port_desc())
        self.port_desc_var = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.port_desc_var, foreground="gray50", width=26).pack(side="left", padx=4)
        ttk.Button(bar, text="刷新", width=5, command=self._refresh_ports).pack(side="left", padx=2)

        ttk.Label(bar, text="波特率:").pack(side="left", padx=(10, 2))
        self.baud_var = tk.StringVar(value="1000000")
        ttk.Entry(bar, textvariable=self.baud_var, width=8).pack(side="left")

        self.connect_btn = ttk.Button(bar, text="连接", width=6, command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=(12, 2))
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(bar, textvariable=self.status_var, foreground="gray20").pack(side="left", padx=8)
        self.cnt_var = tk.StringVar(value="TX:0 RX:0B/0帧 坏:0")
        ttk.Label(bar, textvariable=self.cnt_var).pack(side="right", padx=8)

        self._refresh_ports()

    def _build_control_tab(self):
        pad = {"padx": 6, "pady": 3}

        # ---- MIT 控制 ----
        mit = ttk.LabelFrame(self._tab_ctrl, text="MIT 控制（位置滑块滑动即下发一帧，其余为物理量输入）")
        mit.pack(fill="x", padx=6, pady=6)

        self.mit_vars = {}
        for key, default in (("vel", "0"), ("kp", "0"), ("kd", "0"), ("tq", "0")):
            self.mit_vars[key] = tk.StringVar(value=default)
        self.mit_vars["pos"] = tk.StringVar(value="0.0")

        # 位置滑块：滑动事件 → 更新显示 + 发送一帧 MIT 控制
        pos_row = ttk.Frame(mit)
        pos_row.pack(fill="x", **pad)
        ttk.Label(pos_row, text="位置 (deg):").pack(side="left")
        self.pos_val_var = tk.StringVar(value="0.0°")
        self.pos_scale = tk.Scale(
            pos_row, from_=-180.0, to=180.0, orient="horizontal",
            resolution=0.5, length=420, showvalue=False,
            command=self._on_pos_slide)
        self.pos_scale.pack(side="left", padx=6, fill="x", expand=True)
        ttk.Label(pos_row, textvariable=self.pos_val_var, width=8).pack(side="left")

        row = ttk.Frame(mit)
        row.pack(fill="x", **pad)
        for label, key in (("速度 (rpm)", "vel"), ("Kp (0~500)", "kp"),
                           ("Kd (0~5)", "kd"), ("力矩 (Nm)", "tq")):
            cell = ttk.Frame(row)
            cell.pack(side="left", expand=True, fill="x")
            ttk.Label(cell, text=label).pack(side="top", anchor="w")
            ttk.Entry(cell, textvariable=self.mit_vars[key], width=12).pack(side="top", fill="x")

        btns = ttk.Frame(mit)
        btns.pack(fill="x", **pad)
        ttk.Button(btns, text="发送一次", width=9, command=self._send_mit_once).pack(side="left")
        self.mit_btn = ttk.Button(btns, text="连续发送 ▶", width=11, command=self._toggle_mit)
        self.mit_btn.pack(side="left", padx=6)
        ttk.Label(btns, text="间隔 ms:").pack(side="left", padx=(10, 2))
        self.mit_iv_var = tk.StringVar(value="10")
        ttk.Entry(btns, textvariable=self.mit_iv_var, width=5).pack(side="left")
        ttk.Label(btns, text="（建议 ≥5ms）").pack(side="left")

        # ---- 电机开关 ----
        sw = ttk.LabelFrame(self._tab_ctrl, text="电机状态")
        sw.pack(fill="x", padx=6, pady=6)
        ttk.Button(sw, text="使能", width=8, command=lambda: self._send_cfg(PARAM_ENABLE, 1)).pack(side="left", **pad)
        ttk.Button(sw, text="失能", width=8, command=lambda: self._send_cfg(PARAM_ENABLE, 0)).pack(side="left", **pad)
        ttk.Button(sw, text="抱闸释放", width=8, command=lambda: self._send_cfg(PARAM_BRAKE, 1)).pack(side="left", **pad)
        ttk.Button(sw, text="抱闸吸合", width=8, command=lambda: self._send_cfg(PARAM_BRAKE, 0)).pack(side="left", **pad)
        ttk.Button(sw, text="清错误", width=8, command=lambda: self._send_cfg(PARAM_CLEAR_ERR, 1)).pack(side="left", **pad)
        ttk.Button(sw, text="电机标零", width=8, command=self._motor_zero).pack(side="left", **pad)
        ttk.Label(sw, text="电机 ID = 1").pack(side="right", padx=10)

        # ---- 量程设置 ----
        sc = ttk.LabelFrame(self._tab_ctrl, text="量程设置（影响 MIT 原始值换算，建议与电机实际参数一致）")
        sc.pack(fill="x", padx=6, pady=6)
        rng = [
            ("Pos_Max (deg)", "pos_max", "180"),
            ("Vel_Max (rpm)", "vel_max", "3000"),
            ("T_Max (Nm)", "tq_max", "50"),
        ]
        self.rng_vars = {}
        for label, key, default in rng:
            ttk.Label(sc, text=label).pack(side="left", padx=(10, 2))
            var = tk.StringVar(value=default)
            self.rng_vars[key] = var
            ttk.Entry(sc, textvariable=var, width=8).pack(side="left", padx=2)
        ttk.Button(sc, text="应用量程", width=9, command=self._apply_range).pack(side="left", padx=12)

        # ---- 传感器 ----
        sz = ttk.LabelFrame(self._tab_ctrl, text="扭矩传感器 (Mz)")
        sz.pack(fill="x", padx=6, pady=6)
        ttk.Button(sz, text="读 Mz", width=9, command=lambda: self._send(cmd_get_mz(), "读 Mz")).pack(side="left", **pad)
        ttk.Button(sz, text="标零", width=9, command=self._zero_mz).pack(side="left", **pad)
        ttk.Label(sz, text="流式:").pack(side="left", padx=(14, 2))
        self.stream_btn = ttk.Button(sz, text="开 ▶", width=6, command=self._toggle_stream)
        self.stream_btn.pack(side="left")
        ttk.Label(sz, text="间隔 ms:").pack(side="left", padx=(8, 2))
        self.stream_iv_var = tk.StringVar(value="10")
        ttk.Entry(sz, textvariable=self.stream_iv_var, width=5).pack(side="left")

    def _build_monitor_tab(self):
        pad = {"padx": 6, "pady": 3}

        top = ttk.Frame(self._tab_mon)
        top.pack(fill="x", **pad)
        self.fb_vars = {
            "pos": tk.StringVar(value="-"),
            "speed": tk.StringVar(value="-"),
            "iq": tk.StringVar(value="-"),
            "tq": tk.StringVar(value="-"),
            "temp": tk.StringVar(value="-"),
            "err": tk.StringVar(value="-"),
            "mode": tk.StringVar(value="-"),
            "status": tk.StringVar(value="-"),
        }
        self.mz_vars = {
            "mz": tk.StringVar(value="-"),
            "seq": tk.StringVar(value="-"),
            "rate": tk.StringVar(value="-"),
            "flags": tk.StringVar(value="-"),
        }

        ttk.Button(top, text="读反馈", width=8, command=lambda: self._send(cmd_get_fb(), "读反馈")).pack(side="left", **pad)
        ttk.Button(top, text="读状态", width=8, command=lambda: self._send(cmd_get_st(), "读状态")).pack(side="left", **pad)
        self.auto_refresh_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="自动刷新", variable=self.auto_refresh_var, command=self._toggle_refresh).pack(side="left", **pad)
        ttk.Label(top, text="间隔 ms:").pack(side="left", padx=(8, 2))
        self.refresh_iv_var = tk.StringVar(value="50")
        ttk.Entry(top, textvariable=self.refresh_iv_var, width=5).pack(side="left")

        # 电机反馈
        mf = ttk.LabelFrame(self._tab_mon, text="电机反馈")
        mf.pack(fill="x", padx=6, pady=6)
        labels = [
            ("位置", "pos", "deg"),
            ("速度", "speed", "rpm"),
            ("电流", "iq", "mA"),
            ("力矩", "tq", "Nm"),
            ("温度", "temp", "℃"),
            ("错误码", "err", ""),
            ("模式", "mode", ""),
            ("状态", "status", ""),
        ]
        for name, key, unit in labels:
            f = ttk.Frame(mf)
            f.pack(side="left", expand=True, fill="x")
            ttk.Label(f, text=f"{name}({unit})").pack(side="top", anchor="w")
            ttk.Label(f, textvariable=self.fb_vars[key], font=("Consolas", 11)).pack(side="top", anchor="w")

        # 传感器
        ms = ttk.LabelFrame(self._tab_mon, text="Mz 扭矩传感器")
        ms.pack(fill="x", padx=6, pady=6)
        mz_labels = [
            ("Mz", "mz", "Nm"),
            ("序号", "seq", ""),
            ("查询率", "rate", "/s"),
            ("状态", "flags", ""),
        ]
        for name, key, unit in mz_labels:
            f = ttk.Frame(ms)
            f.pack(side="left", expand=True, fill="x")
            ttk.Label(f, text=f"{name}({unit})").pack(side="top", anchor="w")
            ttk.Label(f, textvariable=self.mz_vars[key], font=("Consolas", 11)).pack(side="top", anchor="w")

        # 日志
        lg = ttk.LabelFrame(self._tab_mon, text="收发日志")
        lg.pack(fill="both", expand=True, padx=6, pady=6)
        self.log_txt = scrolledtext.ScrolledText(lg, height=14, state="disabled", font=("Consolas", 9))
        self.log_txt.pack(fill="both", expand=True, **pad)
        bar = ttk.Frame(lg)
        bar.pack(fill="x", padx=6, pady=2)
        self.raw_debug_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="调试: 显示原始 RX", variable=self.raw_debug_var,
                        command=self._toggle_raw_debug).pack(side="left")
        ttk.Label(bar, text="（勾选后把所有收到的字节以 HEX 打印，便于排查非 20 字节帧数据）").pack(side="left", padx=6)
        ttk.Button(bar, text="清空日志", command=self._clear_log).pack(side="right")

    def _toggle_raw_debug(self):
        if self.link:
            self.link.raw_debug = self.raw_debug_var.get()

    # ---------------- 串口 ----------------

    def _refresh_ports(self):
        self.port_info = {}
        ports = []
        for p in list_ports.comports():
            ports.append(p.device)
            if p.description:
                desc = p.description
                if p.vid:
                    desc += f" VID={p.vid:04X} PID={p.pid:04X}"
                self.port_info[p.device] = desc
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self._update_port_desc()

    def _update_port_desc(self):
        device = self.port_var.get()
        self.port_desc_var.set(self.port_info.get(device, ""))

    def _toggle_connect(self):
        if self.link:
            self.link.stop()
            self.link = None
            self.mit_on = False
            self.mit_btn.config(text="连续发送 ▶")
            self._update_connect_state()
            self._log("已断开串口", "sys")
            return
        port = self.port_var.get()
        if not port:
            self._log("未选择串口端口", "err")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = 1000000
        try:
            self.link = SerialLink(port, baud)
            self.link.start()
        except Exception as exc:
            self.link = None
            msg = str(exc)
            self._log(f"串口打开失败: {msg}", "err")
            hint = "端口被其他程序占用或已拔出/无效，请检查后重试。"
            low = msg.lower()
            if "permission" in low or "access is denied" in low or "拒绝访问" in low or "in use" in low:
                hint = ("端口被其他程序占用：请先关闭占用该端口的工具"
                        "（串口监视器/调试助手/本程序旧实例），再点「连接」。")
            elif "filenotfound" in low or "系统找不到指定的文件" in low or "无法找到" in low:
                hint = ("端口不存在或已拔出：请确认 USB 转串口已插入并装好驱动，"
                        "点「刷新」重新枚举后选择正确的 COM 口。")
            messagebox.showwarning(
                "串口错误",
                f"无法打开 {port} @ {baud}\n\n原因：{msg}\n\n{hint}")
            return
        self._update_connect_state()
        self._log(f"已连接 {port} @ {baud}", "sys")

    def _update_connect_state(self):
        if self.link:
            self.connect_btn.config(text="断开")
            self.status_var.set(f"已连接 {self.port_var.get()}")
        else:
            self.connect_btn.config(text="连接")
            self.status_var.set("未连接")

    # ---------------- 发送 ----------------

    def _send(self, frame: bytes, desc: str):
        if not self.link:
            self._log("未连接串口", "err")
            return False
        ok = self.link.send(frame)
        if ok:
            self._log(f"TX {frame.hex(' ').upper()}  <- {desc}", "tx")
        else:
            self._log("发送失败", "err")
        return ok

    def _send_cfg(self, param, value):
        names = {1: "使能", 2: "清错误", 3: "抱闸", 4: "Pos_Max", 5: "Vel_Max", 6: "T_Max"}
        self._send(motor_cfg(param, value), f"配置 {names.get(param, param)}={value}")

    def _apply_range(self):
        try:
            pm = float(self.rng_vars["pos_max"].get())
            vm = float(self.rng_vars["vel_max"].get())
            tm = float(self.rng_vars["tq_max"].get())
        except ValueError:
            self._log("量程输入非法", "err")
            return
        if pm <= 0 or vm <= 0 or tm <= 0:
            self._log("量程必须为正数", "err")
            return
        self.cfg = {"pos_max": pm, "vel_max": vm, "tq_max": tm}
        self.pos_scale.config(from_=-pm, to=pm)  # 位置滑块范围跟随量程
        self._send_cfg(PARAM_POS_MAX, int(pm * 10))
        self._send_cfg(PARAM_VEL_MAX, int(vm))
        self._send_cfg(PARAM_TQ_MAX, int(tm * 100))
        self._log(f"量程已更新: Pos_Max={pm}°, Vel_Max={vm}rpm, T_Max={tm}Nm", "sys")

    def _on_pos_slide(self, val_str):
        """位置滑块滑动事件：更新显示并立即下发一帧 MIT 控制数据"""
        try:
            val = float(val_str)
        except ValueError:
            return
        self.mit_vars["pos"].set(f"{val:.1f}")
        self.pos_val_var.set(f"{val:.1f}°")
        if not self.link:
            return  # 未连接时不刷错误日志
        frame = self._collect_mit()
        if frame:
            self.link.send(frame)  # 高频滑动不逐帧写日志，位置读数随动即可

    def _collect_mit(self):
        try:
            pos = float(self.mit_vars["pos"].get())
            vel = float(self.mit_vars["vel"].get())
            kp = float(self.mit_vars["kp"].get())
            kd = float(self.mit_vars["kd"].get())
            tq = float(self.mit_vars["tq"].get())
        except ValueError:
            self._log("MIT 参数输入非法", "err")
            return None
        return mit_ctrl(
            to_pos_raw(pos, self.cfg["pos_max"]),
            to_vel_raw(vel, self.cfg["vel_max"]),
            to_kp_raw(kp),
            to_kd_raw(kd),
            to_tq_raw(tq, self.cfg["tq_max"]),
        )

    def _send_mit_once(self):
        frame = self._collect_mit()
        if frame:
            self._send(frame, "MIT 控制(单次)")

    def _toggle_mit(self):
        self.mit_on = not self.mit_on
        self.mit_btn.config(text="停止连续 ■" if self.mit_on else "连续发送 ▶")
        self._log("连续 MIT 控制 " + ("开始" if self.mit_on else "停止"), "sys")
        if self.mit_on:
            self._last_mit = 0.0

    def _toggle_stream(self):
        if not self.link:
            self._log("未连接串口", "err")
            return
        try:
            iv = int(self.stream_iv_var.get())
        except ValueError:
            iv = 20
        iv = _clamp(iv, 1, 1000)
        on = self.stream_btn.cget("text") == "开 ▶"
        self.stream_btn.config(text="关 ■" if on else "开 ▶")
        self._send(cmd_stream(on, iv), f"流式{'开' if on else '关'} {iv}ms")

    def _zero_mz(self):
        self._send(cmd_zero_mz(), "Mz 标零")

    def _motor_zero(self):
        self._send(cmd_motor_zero(), "电机标零 (SDO 0x601/0x2531=1)")

    def _toggle_refresh(self):
        self.auto_refresh = self.auto_refresh_var.get()

    # ---------------- 接收 ----------------

    def _tick(self):
        self._drain_rx()
        now = time.time() * 1000.0
        if self.link:
            if self.mit_on:
                try:
                    iv = max(5, int(self.mit_iv_var.get()))
                except ValueError:
                    iv = 10
                if now - self._last_mit >= iv:
                    self._last_mit = now
                    frame = self._collect_mit()
                    if frame:
                        self.link.send(frame)  # 高频连续发送不逐帧写日志
            if self.auto_refresh:
                try:
                    iv = max(20, int(self.refresh_iv_var.get()))
                except ValueError:
                    iv = 50
                if now - self._last_refresh >= iv:
                    self._last_refresh = now
                    self.link.send(cmd_get_fb())
                    self.link.send(cmd_get_mz())
        if self.link:
            self.cnt_var.set(f"TX:{self.link.tx_count} RX:{self.link.rx_bytes}B/{self.link.rx_frames}帧 坏:{self.link.rx_bad}")
        # Mz 实际更新率统计（每秒刷新）
        now_s = time.time()
        if now_s - self._mz_rate_t0 >= 1.0:
            self.mz_vars["rate"].set(f"{self._mz_rate_cnt}")
            self._mz_rate_cnt = 0
            self._mz_rate_t0 = now_s
        self.root.after(5, self._tick)

    def _drain_rx(self):
        if not self.link:
            return
        while True:
            try:
                item = self.link.rx_q.get_nowait()
            except queue.Empty:
                break
            if item[0] == "frame":
                self._on_frame(item[1], item[2])
            elif item[0] == "raw":
                # 原始字节可能高频，合并相邻批次为一行
                if self.raw_debug_var.get():
                    self._log(f"RX RAW {item[1]}", "raw")
            elif item[0] == "err":
                self._log(f"接收线程错误: {item[1]}", "err")

    def _on_frame(self, cid, data):
        # 自动刷新/流式模式下的高频帧只刷新显示，手动单次读取才写日志
        do_log = not self.auto_refresh
        if cid == CID_RSP_FB:
            fb = parse_fb(data)
            self.fb_vars["pos"].set(f"{fb['pos_deg']:.2f}")
            self.fb_vars["speed"].set(f"{fb['speed_rpm']}")
            self.fb_vars["iq"].set(f"{fb['iq_ma']}")
            self.fb_vars["tq"].set(f"{fb['tq_nm']:.2f}")
            if do_log:
                self._log(f"RX 反馈: pos={fb['pos_deg']:.2f}° speed={fb['speed_rpm']}rpm "
                          f"iq={fb['iq_ma']}mA tq={fb['tq_nm']:.2f}Nm", "rx")
        elif cid == CID_RSP_ST:
            st = parse_st(data)
            mode_txt = {1: "轮廓位置", 2: "轮廓速度", 3: "CSP", 4: "CSV", 5: "电流", 6: "MIT"}.get(st["mode"], f"0x{st['mode']:02X}")
            status_txt = f"使能:{int(st['en'])} 抱闸:{int(st['brake'])} 报错:{int(st['err_bit'])} 到位:{int(st['in_pos'])}"
            self.fb_vars["temp"].set(f"{st['temp_c']:.1f}")
            self.fb_vars["err"].set(f"0x{st['err']:04X}")
            self.fb_vars["mode"].set(mode_txt)
            self.fb_vars["status"].set(status_txt)
            if do_log:
                self._log(f"RX 状态: err=0x{st['err']:04X} temp={st['temp_c']:.1f}℃ mode={mode_txt} [{status_txt}]", "rx")
        elif cid == CID_RSP_MZ:
            self._mz_rate_cnt += 1
            mz = parse_mz(data)
            self.mz_vars["mz"].set(f"{mz['mz_nm']:.4f}")
            self.mz_vars["seq"].set(f"{mz['seq']}")
            self.mz_vars["flags"].set(f"在线:{int(mz['online'])} 标零:{int(mz['zero_ok'])}")
            if do_log:
                self._log(f"RX Mz: {mz['mz_nm']:.4f}Nm seq={mz['seq']} 在线={int(mz['online'])}", "rx")
        elif cid == CID_RSP_STREAM:
            self._mz_rate_cnt += 1
            st = parse_stream(data)
            self.mz_vars["mz"].set(f"{st['mz_nm']:.4f}")
            self.fb_vars["pos"].set(f"{st['pos_deg']:.2f}")
            self.fb_vars["tq"].set(f"{st['tq_nm']:.2f}")
            # 高频流帧只刷新显示，不逐帧写日志（避免刷屏）
        elif cid == CID_ACK:
            ack = parse_ack(data)
            self._log(f"RX ACK: {'OK' if ack['ok'] else 'NAK'} (cmd=0x{ack['echo']:02X})", "rx")

    # ---------------- 日志 ----------------

    def _log(self, msg: str, tag: str = "info"):
        colors = {"tx": "#1a6fb5", "rx": "#1a8a3c", "sys": "#8a6d1a",
                  "err": "#c00000", "raw": "#666666"}
        color = colors.get(tag, "#000000")
        ts = time.strftime("%H:%M:%S")
        self.log_txt.config(state="normal")
        self.log_txt.insert("end", f"[{ts}] ", "ts")
        self.log_txt.insert("end", msg + "\n", ("tag",))
        self.log_txt.tag_config("ts", foreground="#888888")
        self.log_txt.tag_config("tag", foreground=color)
        self.log_txt.see("end")
        # 限制行数防止内存膨胀
        if int(self.log_txt.index("end-1c").split(".")[0]) > 2000:
            self.log_txt.delete("1.0", "500.0")
        self.log_txt.config(state="disabled")

    def _clear_log(self):
        self.log_txt.config(state="normal")
        self.log_txt.delete("1.0", "end")
        self.log_txt.config(state="disabled")


def main():
    try:
        import serial  # noqa: F401
    except ImportError:
        import sys
        sys.exit("缺少 pyserial：请先执行  pip install pyserial")
    root = tk.Tk()
    JuxieHostApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
