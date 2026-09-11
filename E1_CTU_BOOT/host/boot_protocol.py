# -*- coding: utf-8 -*-
"""E1_CTU_BOOT 上位机协议层（方案 B：z 帧寻址分块升级，无 GUI 依赖）

与 docs/boot_485_ymodem.md 及固件 boot_proto 对齐：
  帧: [ 'z' ][ cmd ][ len ][ payload... ][ CRC8 ][ '\n' ]
      CRC8 = poly 0x8C, init 0xFF，覆盖 z..payload
  payload[0] = 目标设备 ID（0x01=MASTER / 0x02=SLAVER / 0x00=广播）
  数据传输：块内 16 位块号 + CRC16；每帧由设备单独应答 → 多节点 485 安全
  Boot 【不主动发送】，仅被寻址帧校验通过才回帧（无 'C' 心跳）

依赖: pyserial
"""

from __future__ import annotations

import os
import time
from typing import Callable, Optional

import serial

# ---- z 帧信封 ----
Z_HEADER = 0x7A             # 'z'
Z_FOOT = 0x0A               # '\n'
Z_REPLY_FLAG = 0x80
Z_ERR = 0x7F

# ---- 设备 ----
DEVICE_MASTER = "master"    # Id=0x01，升级命令 0x06（App/Boot 通用）
DEVICE_SLAVER = "slaver"    # Id=0x02，升级命令 0x06
DEVICE_BOOT = "boot"        # 广播 Id=0x00（Boot 未定 ID 时仍可选中）

DEVICE_NAMES = {
    DEVICE_MASTER: "Master 主电源板 (0x01)",
    DEVICE_SLAVER: "Slaver 副电源板 (0x02)",
    DEVICE_BOOT: "广播 / 直连 Boot (0x00)",
}
DEVICE_ADDR = {DEVICE_MASTER: 0x01, DEVICE_SLAVER: 0x02, DEVICE_BOOT: 0x00}
# 0x06 对 App = 请求升级并复位；对 Boot = SELECT 选中会话（两者同码，幂等）
UPGRADE_CMD = {DEVICE_MASTER: 0x06, DEVICE_SLAVER: 0x06, DEVICE_BOOT: 0x06}

# ---- 升级分块协议 ----
CMD_SELECT = 0x06
CMD_START = 0x08
CMD_DATA = 0x09
CMD_END = 0x0A
CMD_ABORT = 0x0B

ERR_NONE = 0x00
ERR_TEXT = {
    0x01: "帧长非法",
    0x02: "状态错/未选中",
    0x03: "块号错",
    0x04: "数据 CRC16 错",
    0x05: "Flash 写失败",
    0x06: "长度/容量错",
}

DATA_MAX = 248              # 每块最大数据字节（与固件 BOOT_PROTO_DATA_MAX 一致；4 字节对齐）
MAX_FW_SIZE = 0x18000       # App 分区容量 96KB

# ---- App 镜像预检（链接到 AppA 的向量表 sanity） ----
APP_VECT_ADDR = 0x08008000
APP_END_ADDR = 0x08008000 + 0x18000
RAM_BASE = 0x20000000
RAM_END = 0x20000000 + 48 * 1024

LogCb = Callable[[str, str], None]
PhaseCb = Callable[[str], None]
ProgressCb = Callable[[float], None]


def crc16_xmodem(data: bytes, crc: int = 0) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


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


def check_app_image(data: bytes) -> Optional[str]:
    """预检是否为链接到 AppA 的有效固件（向量表 sanity）。返回 None=通过。"""
    if len(data) < 16:
        return "文件过小，不是有效固件"
    sp = int.from_bytes(data[0:4], "little")
    pc = int.from_bytes(data[4:8], "little")
    if not (RAM_BASE <= sp <= RAM_END):
        return f"向量表栈顶 0x{sp:08X} 不在 RAM 范围（疑似非本 App 固件）"
    if (pc & 1) == 0 or not (APP_VECT_ADDR <= pc < APP_END_ADDR):
        return (f"复位向量 0x{pc:08X} 不在 AppA 区间 "
                f"[0x{APP_VECT_ADDR:08X}, 0x{APP_END_ADDR:08X})")
    return None


def build_frame(cmd: int, payload: bytes) -> bytes:
    """构建 z 帧：[z][cmd][len][payload][crc8(z..payload)][\\n]"""
    body = bytes([Z_HEADER, cmd, len(payload)]) + payload
    return body + bytes([crc8(body), Z_FOOT])


def build_upgrade_frame(device: str) -> Optional[bytes]:
    """构建"请求进入升级"帧（两板 0x06，payload=[目标ID][magic=0x01]）"""
    cmd = UPGRADE_CMD.get(device)
    addr = DEVICE_ADDR.get(device)
    if cmd is None or addr is None:
        return None
    return build_frame(cmd, bytes([addr, 0x01]))


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class BootProtoSender:
    """寻址分块升级发送端（阻塞同步，须在独立线程运行）

    流程：SELECT(0x06) → START(0x08) → DATA(0x09)×N → END(0x0A)
    每步等待设备应答并重试；设备若 ABORT/超时则失败。
    """

    def __init__(self, serial_handle: serial.Serial,
                 log_cb: Optional[LogCb] = None,
                 phase_cb: Optional[PhaseCb] = None,
                 progress_cb: Optional[ProgressCb] = None) -> None:
        self._ser = serial_handle
        self._log = log_cb or (lambda _t, _l: None)
        self._phase = phase_cb or (lambda _t: None)
        self._progress = progress_cb or (lambda _f: None)
        self._cancel = False
        self._rx = bytearray()

    def cancel(self) -> None:
        self._cancel = True

    # ---------- 对外主流程 ----------
    def transfer(self, device: str, filepath: str,
                 data: Optional[bytes] = None) -> bool:
        addr = DEVICE_ADDR.get(device)
        if addr is None:
            self._log("未知设备类型", "error")
            return False
        if data is None:
            with open(filepath, "rb") as f:
                data = f.read()
        size = len(data)
        if size == 0 or size > MAX_FW_SIZE:
            self._log(f"固件大小 {size}B 非法（≤{MAX_FW_SIZE}）", "error")
            return False
        reason = check_app_image(data)
        if reason is not None:
            self._log(f"固件预检失败：{reason}", "error")
            return False
        checksum = sum(data) & 0xFFFFFFFF
        fname = os.path.basename(filepath)

        # 1) SELECT
        self._phase("选中设备并进入升级会话")
        if not self._select(addr):
            return False
        self._log(f"已选中设备 0x{addr:02X}（{fname}, {size}B, sum=0x{checksum:08X}）",
                  "info")

        # 2) START
        self._phase("擦除暂存区并开始下载")
        if not self._cmd_start(addr, size, checksum):
            return False

        # 3) DATA
        self._phase("传输固件数据")
        total_blk = (size + DATA_MAX - 1) // DATA_MAX
        blk = 0
        offset = 0
        while offset < size:
            if self._cancel:
                self._send_abort(addr)
                return False
            chunk = data[offset:offset + DATA_MAX]
            if not self._cmd_data(addr, blk, chunk):
                self._send_abort(addr)
                return False
            offset += len(chunk)
            blk += 1
            self._progress(min(offset / size, 1.0))
            if (blk % 8 == 0) or (blk == total_blk):
                self._log(f"块 {blk}/{total_blk}（{offset * 100 // size}%）", "tx")
        self._log("数据全部写入", "info")

        # 4) END（板端提交并复位）
        self._phase("提交固件（校验→提升→复位）")
        if not self._cmd_end(addr):
            return False
        self._log("升级完成，板端复位运行新固件", "info")
        return True

    # ---------- 各命令 ----------
    def _select(self, addr: int) -> bool:
        frame = build_frame(CMD_SELECT, bytes([addr, 0x01]))
        for attempt in range(8):
            if self._cancel:
                return False
            self._tx(frame)
            rep = self._wait_reply(CMD_SELECT, 2.0)
            if rep is not None:
                err = rep[1] if len(rep) > 1 else 0xFF
                if err == ERR_NONE:
                    return True
                self._log(f"SELECT 失败: {ERR_TEXT.get(err, hex(err))}", "warn")
            time.sleep(0.3)
        self._log("SELECT 超时：未收到 Boot 应答（确认设备在升级模式/ID 正确）",
                  "error")
        return False

    def _cmd_start(self, addr: int, size: int, checksum: int) -> bool:
        pl = bytes([addr]) + size.to_bytes(4, "little") \
            + checksum.to_bytes(4, "little")
        return self._cmd_err_only(CMD_START, pl, 5.0, "START")

    def _cmd_data(self, addr: int, blk: int, chunk: bytes) -> bool:
        for attempt in range(8):
            if self._cancel:
                return False
            crc = crc16_xmodem(chunk)
            pl = (bytes([addr]) + blk.to_bytes(2, "little")
                  + crc.to_bytes(2, "little") + chunk)
            self._tx(build_frame(CMD_DATA, pl))
            rep = self._wait_reply(CMD_DATA, 2.0)
            if rep is None:
                self._log(f"块 {blk} 应答超时，重发...", "warn")
                continue
            err = rep[1] if len(rep) > 1 else 0xFF
            if err == ERR_NONE:
                return True
            self._log(f"块 {blk} 失败: {ERR_TEXT.get(err, hex(err))}，重发...",
                      "warn")
        return False

    def _cmd_end(self, addr: int) -> bool:
        return self._cmd_err_only(CMD_END, bytes([addr]), 30.0, "END")

    def _send_abort(self, addr: int) -> None:
        try:
            self._tx(build_frame(CMD_ABORT, bytes([addr])))
            self._wait_reply(CMD_ABORT, 1.0)
        except Exception:  # noqa: BLE001
            pass
        self._log("已发送 ABORT 中止", "warn")

    def _cmd_err_only(self, cmd: int, payload: bytes, timeout: float,
                      name: str) -> bool:
        for attempt in range(4):
            if self._cancel:
                return False
            self._tx(build_frame(cmd, payload))
            rep = self._wait_reply(cmd, timeout)
            if rep is not None:
                err = rep[1] if len(rep) > 1 else 0xFF
                if err == ERR_NONE:
                    return True
                self._log(f"{name} 失败: {ERR_TEXT.get(err, hex(err))}", "error")
                return False
            self._log(f"{name} 应答超时，重试...", "warn")
        return False

    # ---------- 收发 ----------
    def _tx(self, frame: bytes) -> None:
        try:
            self._ser.write(frame)
        except Exception as exc:  # noqa: BLE001
            self._log(f"发送失败: {exc}", "error")

    def _wait_reply(self, cmd: int, timeout: float) -> Optional[bytes]:
        deadline = time.time() + timeout
        want = cmd | Z_REPLY_FLAG
        while time.time() < deadline:
            if self._cancel:
                return None
            frame = self._read_frame(deadline)
            if frame is None:
                return None
            rcmd, payload = frame
            if rcmd == want:
                return payload
            if rcmd == Z_ERR:
                code = payload[1] if len(payload) > 1 else 0xFF
                self._log(f"设备错误应答: {ERR_TEXT.get(code, hex(code))}", "error")
                return None
        return None

    def _read_frame(self, deadline: float) -> Optional[tuple[int, bytes]]:
        """从串口解析一帧完整 z 帧（CRC8 校验通过）；超时返回 None"""
        while time.time() < deadline:
            if self._cancel:
                return None
            n = self._ser.in_waiting
            if n <= 0:
                time.sleep(0.002)
                continue
            self._rx.extend(self._ser.read(n))
            # 查找完整帧
            while True:
                idx = self._rx.find(bytes([Z_HEADER]))
                if idx < 0:
                    self._rx.clear()
                    break
                if idx > 0:
                    del self._rx[:idx]
                if len(self._rx) < 3:
                    break
                plen = self._rx[2]
                total = plen + 5
                if len(self._rx) < total:
                    break
                frame = bytes(self._rx[:total])
                del self._rx[:total]
                if frame[-1] != Z_FOOT:
                    continue
                if crc8(frame[:-2]) != frame[-2]:
                    continue
                return frame[1], frame[3:3 + plen]
        return None


def request_boot(serial_handle: serial.Serial, device: str,
                 log_cb: Optional[LogCb] = None) -> bool:
    """发送 0x06 邀请：对 App=请求升级并复位；对 Boot=SELECT 选中会话（幂等）"""
    log = log_cb or (lambda _t, _l: None)
    frame = build_upgrade_frame(device)
    if frame is None:
        log("该目标不支持 0x06（检查设备类型）", "error")
        return False
    log(f"TX  {_hex(frame)}", "tx")
    try:
        serial_handle.write(frame)
    except Exception as exc:  # noqa: BLE001
        log(f"发送失败: {exc}", "error")
        return False
    time.sleep(0.4)
    n = serial_handle.in_waiting
    if n > 0:
        log(f"RX  {_hex(serial_handle.read(n))}", "tx")
    return True
