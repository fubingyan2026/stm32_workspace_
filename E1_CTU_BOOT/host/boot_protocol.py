# -*- coding: utf-8 -*-
"""E1_CTU_BOOT YMODEM 传输协议层 — 无 GUI 依赖，供 CLI 与上位机(GUI)复用

与 docs/boot_485_ymodem.md 对齐：
  - 标准 YMODEM 接收端邀请流程（板端发 'C'）→ 本端作为发送端
  - SOH(128B)/STX(1024B) 分包、CRC-16/xmodem（poly 0x1021, init 0）
  - block0 携带文件名 + 长度；EOT → NAK → EOT → ACK 结束
  - 另含"请求进入 Boot"的 z 帧构建/应答辅助（带设备地址：
    Master addr=0x01 cmd=0x06 / Slaver addr=0x02 cmd=0x06；命令码两板统一为 0x06）

依赖: pyserial
"""

from __future__ import annotations

import os
import time
from typing import Callable, Optional

import serial

# ---- YMODEM 控制字符 ----
SOH = 0x01
STX = 0x02
EOT = 0x04
ACK = 0x06
NAK = 0x15
CAN = 0x18
C = 0x43

MAX_RETRY = 10
SILENT_TIMEOUT = 5.0        # 逐包应答等待（秒）
C_WAIT_TIMEOUT = 30.0       # 等待板端 'C' 邀请（秒）
MAX_FW_SIZE = 0x18000       # App 分区容量 96KB

# ---- App 侧"进入升级"命令（兄弟工程预留，见 docs/boot_485_ymodem.md §5） ----
DEVICE_MASTER = "master"    # 主电源板 E1_MASTER_POWER_CTU，地址 0x01，升级命令 0x06（两板统一）
DEVICE_SLAVER = "slaver"    # 副电源板 E1_SLAVER_POWER_CTU，地址 0x02，升级命令 0x06
DEVICE_BOOT = "boot"        # 直连 Boot（板端已在升级模式，不需请求）

DEVICE_NAMES = {
    DEVICE_MASTER: "Master 主电源板 (0x01/0x06)",
    DEVICE_SLAVER: "Slaver 副电源板 (0x02/0x06)",
    DEVICE_BOOT: "直连 Boot（已在升级模式）",
}

DEVICE_ADDR = {DEVICE_MASTER: 0x01, DEVICE_SLAVER: 0x02}
UPGRADE_CMD = {DEVICE_MASTER: 0x06, DEVICE_SLAVER: 0x06}
Z_HEADER = 0x7A             # 'z'
Z_REPLY_FLAG = 0x80
Z_ERR = 0x7F
ERR_TEXT = {
    0x00: "无错误",
    0x01: "未知命令",
    0x02: "功能暂不支持",
    0x03: "帧长度与命令不匹配",
}

LogCb = Callable[[str, str], None]          # (文本, 级别: info/warn/error/tx)
PhaseCb = Callable[[str], None]             # 阶段文本
ProgressCb = Callable[[float], None]        # 0..1 进度


def crc16_xmodem(data: bytes, crc: int = 0) -> int:
    """CRC-16/xmodem：多项式 0x1021，初值 0"""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---- z 帧 CRC8（与兄弟固件 m_middlewares crc.c / host *_protocol.py 一致） ----
_CRC8_TABLE: list[int] = []
for _i in range(256):
    _crc = _i
    for _ in range(8):
        if _crc & 0x01:
            _crc = ((_crc >> 1) ^ 0x8C) & 0xFF
        else:
            _crc = (_crc >> 1) & 0xFF
    _CRC8_TABLE.append(_crc)


def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def build_upgrade_frame(device: str) -> Optional[bytes]:
    """构建"请求进入 Boot" z 帧（Master 0x01/0x06、Slaver 0x02/0x06，payload magic=0x01）

    [z][addr][cmd][data_len=1][0x01][crc8][0x0A]
    """
    cmd = UPGRADE_CMD.get(device)
    addr = DEVICE_ADDR.get(device)
    if cmd is None or addr is None:
        return None
    body = bytes([Z_HEADER, addr, cmd, 0x01, 0x01])
    return body + bytes([crc8(body), 0x0A])


def _frame_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


class YmodemSender:
    """YMODEM 发送端（阻塞式同步运行，须在独立线程调用）

    用法:
        ser = serial.Serial(port, baud, timeout=0)
        sender = YmodemSender(ser, log_cb=..., phase_cb=..., progress_cb=...)
        ok = sender.send_file("app.bin")

    中止: 另一线程调用 sender.cancel()（内部在安全点发 CAN(0x18)）。
    """

    def __init__(self,
                 serial_handle: serial.Serial,
                 log_cb: Optional[LogCb] = None,
                 phase_cb: Optional[PhaseCb] = None,
                 progress_cb: Optional[ProgressCb] = None) -> None:
        self._ser = serial_handle
        self._log = log_cb or (lambda _t, _l: None)
        self._phase = phase_cb or (lambda _t: None)
        self._progress = progress_cb or (lambda _f: None)
        self._cancel_flag = False

    # ---- 对外控制 ----
    def cancel(self) -> None:
        """请求中止（线程安全；发送端会在安全点发出 CAN）"""
        self._cancel_flag = True

    # ---- 传输 ----
    def send_file(self, filepath: str, data: Optional[bytes] = None,
                  use_1k: bool = True) -> bool:
        """阻塞发送一个固件文件。成功返回 True。

        use_1k=True 用 STX(1024B) 数据包（默认，YMODEM 常规）；置 False 用
        SOH(128B) 小包，用于定位 1KB 突发相关的链路问题。
        """
        if data is None:
            with open(filepath, "rb") as f:
                data = f.read()
        size = len(data)
        if size == 0 or size > MAX_FW_SIZE:
            self._log(f"固件大小 {size}B 非法（0 < size ≤ {MAX_FW_SIZE}）", "error")
            return False
        fname = os.path.basename(filepath)

        block = 1024 if use_1k else 128
        block_hdr = STX if use_1k else SOH

        # 1) 等待板端 'C' 邀请
        self._phase("等待板端进入升级模式 ('C')...")
        self._log("等待接收端 'C'（板端 Boot 升级模式）...", "info")
        if not self._wait_for_c(C_WAIT_TIMEOUT):
            self._log("超时：未收到接收端邀请（确认板端已进入升级模式、波特率一致）",
                      "error")
            return False
        self._log("收到 'C'，开始传输", "info")

        # 2) block0 头包（文件名 + 长度）
        self._phase("发送头包 block0")
        hdr = bytearray(128)
        name = fname.encode("utf-8")[:90]
        hdr[:len(name)] = name
        hdr[len(name)] = 0
        hdr[124] = (size >> 16) & 0xFF
        hdr[125] = (size >> 8) & 0xFF
        hdr[126] = size & 0xFF
        if not self._send_packet(SOH, 0, bytes(hdr)):
            return False
        self._log(f"block0 已 ACK（{fname}, {size} 字节）", "info")

        # 3) 数据包
        self._phase("传输固件数据")
        seq = 1
        offset = 0
        total_pkts = (size + block - 1) // block
        while offset < size:
            if self._cancel_flag:
                self._abort()
                return False
            chunk = data[offset:offset + block]
            pad = chunk + b"\x1A" * (block - len(chunk))
            if not self._send_packet(block_hdr, seq, pad):
                return False
            offset += len(chunk)
            seq = (seq + 1) & 0xFF
            self._progress(min(offset / size, 1.0))
            self._log(f"块 {(seq - 1 + total_pkts) % 256}/{total_pkts} 已 ACK"
                      f"（{(offset / max(size, 1)) * 100:.0f}%）", "tx")
        self._log("全部数据已 ACK", "info")

        # 4) EOT → NAK → EOT → ACK
        self._phase("结束传输（EOT），板端校验并提交")
        self._send_byte(EOT)
        if self._recv_byte(SILENT_TIMEOUT) != NAK:
            self._log("首个 EOT 未收到 NAK（继续）", "warn")
        self._send_byte(EOT)
        if self._recv_byte(SILENT_TIMEOUT) != ACK:
            self._log("第二个 EOT 未收到 ACK（板端可能已进入提交，稍后自动复位）",
                      "warn")
        self._log("传输结束。板端正在校验并提升 A 分区，随后自动复位运行新固件...",
                  "info")
        return True

    # ---- 内部 ----
    def _send_byte(self, byte: int) -> None:
        try:
            self._ser.write(bytes([byte]))
        except Exception as exc:  # noqa: BLE001
            self._log(f"发送失败: {exc}", "error")

    def _abort(self) -> None:
        try:
            self._ser.write(bytes([CAN]))
        except Exception:  # noqa: BLE001
            pass
        self._log("已中止传输（发送 CAN）", "warn")
        self._phase("已中止")

    def _recv_byte(self, timeout: float, skip_c: bool = True) -> Optional[int]:
        """读取单字节；skip_c=True 时跳过冗余的 'C'（接收端心跳提示/续发邀请）"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel_flag:
                self._abort()
                return None
            n = self._ser.in_waiting
            if n > 0:
                b = self._ser.read(1)
                if not b:
                    continue
                v = b[0]
                if skip_c and v == C:
                    continue
                return v
            time.sleep(0.002)
        return None

    def _wait_for_c(self, timeout: float) -> bool:
        """等待接收端进入传输的 'C' 邀请。

        板端 Boot 升级模式会周期性发 'C'；此阶段必须返回 'C' 本身，
        其他噪声字节忽略后继续等待。返回 True=已收到 'C'。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel_flag:
                self._abort()
                return False
            n = self._ser.in_waiting
            if n > 0:
                b = self._ser.read(1)
                if not b:
                    continue
                if b[0] == C:
                    return True
                # 非 'C' 噪声（如请求跳转后遗留应答/总线杂讯）：忽略继续等
            time.sleep(0.002)
        return False

    def _send_packet(self, header: int, seq: int, data: bytes) -> bool:
        """发送单包并等待 ACK/NAK（含重试）。True=ACK。"""
        payload = bytes([header, seq & 0xFF, (~seq) & 0xFF]) + data
        crc = crc16_xmodem(data)
        frame = payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

        for attempt in range(MAX_RETRY):
            if self._cancel_flag:
                self._abort()
                return False
            try:
                self._ser.write(frame)
            except Exception as exc:  # noqa: BLE001
                self._log(f"发送失败: {exc}", "error")
                return False
            resp = self._recv_byte(SILENT_TIMEOUT)
            if resp == ACK:
                return True
            if resp == NAK:
                self._log(f"NAK seq={seq & 0xFF} attempt={attempt + 1}，重发...",
                          "warn")
                continue
            if resp == CAN:
                self._log("接收端 CAN 中止", "error")
                return False
            if resp is None:
                self._log(f"等待应答超时 seq={seq & 0xFF}，重发...", "warn")
                continue
            # 其他字符忽略后重发
        self._log(f"seq={seq & 0xFF} 重试次数超限", "error")
        return False


def request_boot(serial_handle: serial.Serial,
                 device: str,
                 log_cb: Optional[LogCb] = None) -> bool:
    """向运行中的 App 发送"请求进入 Boot"命令（z 帧），并简短读取应答/状态

    device: DEVICE_MASTER / DEVICE_SLAVER。返回是否已发出（不校验后续复位）。
    """
    log = log_cb or (lambda _t, _l: None)
    frame = build_upgrade_frame(device)
    if frame is None:
        log("未知设备类型，无法构建升级请求", "error")
        return False
    log(f"TX  {_frame_hex(frame)}", "tx")
    try:
        serial_handle.write(frame)
    except Exception as exc:  # noqa: BLE001
        log(f"发送失败: {exc}", "error")
        return False

    # 短暂读取 App 应答（0x7F 错误 / 0x9x ACK），不阻塞等待复位后的 Boot
    deadline = time.time() + 0.5
    buf = bytearray()
    while time.time() < deadline:
        n = serial_handle.in_waiting
        if n > 0:
            buf.extend(serial_handle.read(n))
        time.sleep(0.01)
    if buf:
        log(f"RX  {_frame_hex(bytes(buf))}", "tx")
        if len(buf) >= 6 and buf[2] == Z_ERR:
            code = buf[4] if len(buf) > 4 else 0xFF
            log(f"App 应答不支持升级命令: {ERR_TEXT.get(code, '未知')} (0x{code:02X})",
                "warn")
        else:
            log("App 已应答，将复位进入 Boot...", "info")
    else:
        log("已发送升级请求（App 将复位进入 Boot）", "info")
    return True
