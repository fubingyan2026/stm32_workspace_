# -*- coding: utf-8 -*-
"""虚拟电源板：在 pty 上模拟 E1 CTU 电源板（应用协议 + Boot 升级协议）。

用于在没有真实硬件的情况下（例如 WSL / CI）端到端验证 SDK：它既能应答
0x01/0x02/0x03/0x04/0x05/0x06/0x07，也能完成 SELECT/START/DATA/END 分块升级，
并记录收到的控制指令，供测试断言。

仅用于测试，不随 SDK 发布到生产环境。
"""

from __future__ import annotations

import select
import threading
import time
from typing import BinaryIO

from ..devices import device_addr, normalize_device
from ..firmware import (
    BOOT_ERR_NONE,
    CMD_ABORT,
    CMD_DATA,
    CMD_END,
    CMD_SELECT,
    CMD_START,
    crc16_xmodem,
)
from ..protocol import (
    CMD_CTRL,
    CMD_ERR,
    CMD_READ_INFO,
    CMD_READ_STATUS,
    CMD_READ_TEMP,
    CMD_READ_VOLT,
    CMD_REPLY_FLAG,
    CMD_RESET_LATCH,
    CMD_UPGRADE,
    FrameParser,
    build_frame,
    parse_addr,
    parse_cmd,
    parse_payload,
)

# 错误码（与 boot_protocol.ERR_TEXT 一致）
ERR_BAD_STATE = 0x02
ERR_BAD_BLOCK = 0x03
ERR_BAD_CRC16 = 0x04
ERR_BAD_SIZE = 0x06


def _u16(value: int) -> bytes:
    return int(value & 0xFFFF).to_bytes(2, "little")


def _i16(value: float) -> bytes:
    raw = int(round(value * 100))
    return (raw & 0xFFFF).to_bytes(2, "little")


def _u32(value: int) -> bytes:
    return int(value & 0xFFFFFFFF).to_bytes(4, "little")


class FakeCtuBoard:
    """单个设备的虚拟板卡。

    @param stream 板卡侧的二进制流（通常是 pty 的 master 端），需支持
                  ``fileno()`` / ``read()`` / ``write()``
    @param device ``"master"`` / ``"slaver"``
    @param addr   为 ``None`` 时只应答本设备地址；可显式指定以模拟总线过滤
    """

    def __init__(self, stream: BinaryIO, device: str = "master",
                 respond_all_addrs: bool = False) -> None:
        self._stream = stream
        self.device = normalize_device(device)
        self.addr = device_addr(self.device)
        self._respond_all = respond_all_addrs
        self._parser = FrameParser()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # ---- 可配置的数据段 ----
        self.status_payload = bytes([0x00, 0x00])
        self.voltage_payload = _u16(48000) + _u16(47900)
        self.temperature_payload = _i16(25.0) + _i16(26.0) + _i16(35.0)
        self.info_payload = (_u16(1) + _u16(2) + _u32(0x4000)
                             + _u32(0x00EFCDAB) + _u16(3) + bytes([0x01]))

        # ---- 观测到的控制指令 ----
        self.frames_rx = 0
        self.buzzer_duty: int | None = None
        self.output_mask: int | None = None
        self.fill_duty: int | None = None
        self.latch_clears = 0
        self.upgrade_requests = 0
        self.drop_replies = False   # True 时不回帧，用于模拟丢包/离线
        self.force_error: int | None = None   # 非 None 时统一回 0x7F 错误应答
        self.block_delay = 0.0      # 每个 DATA 块处理前的延时（模拟慢速写入）

        # ---- Boot 升级状态 ----
        self.boot_entered = False
        self.boot_done = False
        self.boot_size = 0
        self.boot_checksum = 0
        self.boot_data = bytearray()
        self._boot_started = False
        self._expected_block = 0
        self.boot_errors: list[int] = []

    # ------------------------------------------------------------ 生命周期
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="fake-ctu-board",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def close(self) -> None:
        self.stop()
        self.join(1.0)
        try:
            self._stream.close()
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------ 主循环
    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self._stream], [], [], 0.02)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                data = self._stream.read(4096)
            except (OSError, ValueError):
                break
            if not data:
                break  # 对端关闭
            for frame in self._parser.feed(data):
                self._handle_frame(frame)

    # ------------------------------------------------------------ 帧处理
    def _reply(self, cmd: int, content: bytes = b"") -> None:
        frame = build_frame(cmd | CMD_REPLY_FLAG, bytes([self.addr]) + content)
        try:
            self._stream.write(frame)
            self._stream.flush()
        except (OSError, ValueError):
            pass

    def _reply_error(self, code: int) -> None:
        """0x7F 错误应答（不带应答标志位）。"""
        frame = build_frame(CMD_ERR, bytes([self.addr, code]))
        try:
            self._stream.write(frame)
            self._stream.flush()
        except (OSError, ValueError):
            pass

    def _handle_frame(self, frame: bytes) -> None:
        addr = parse_addr(frame)
        if not self._respond_all and addr != self.addr:
            return
        if parse_cmd(frame) & CMD_REPLY_FLAG:
            return  # 忽略应答帧

        self.frames_rx += 1
        if self.drop_replies:
            return
        if self.force_error is not None:
            self._reply_error(self.force_error)
            return

        cmd = parse_cmd(frame)
        payload = parse_payload(frame)
        content = payload[1:] if payload else b""

        if cmd == CMD_READ_STATUS:
            self._reply(cmd, self.status_payload)
        elif cmd == CMD_READ_VOLT:
            self._reply(cmd, self.voltage_payload)
        elif cmd == CMD_READ_TEMP:
            self._reply(cmd, self.temperature_payload)
        elif cmd == CMD_READ_INFO:
            self._reply(cmd, self.info_payload)
        elif cmd == CMD_CTRL:
            if self.device == "master":
                self.buzzer_duty = content[0] if content else 0
            else:
                self.output_mask = content[0] if content else 0
                self.fill_duty = (content[2] | (content[3] << 8)) if len(content) >= 4 else 0
            self._reply(cmd, content)
        elif cmd == CMD_RESET_LATCH:
            self.latch_clears += 1
            self._reply(cmd, bytes([BOOT_ERR_NONE]))
        elif cmd == CMD_UPGRADE or cmd == CMD_SELECT:
            # 0x06 对 App = 请求升级并复位；对 Boot = 选中会话（同码，幂等）
            self.upgrade_requests += 1
            self.boot_entered = True
            self._boot_started = False
            self._expected_block = 0
            self._reply(cmd, bytes([BOOT_ERR_NONE]))
        elif cmd == CMD_START:
            self._handle_start(content)
        elif cmd == CMD_DATA:
            self._handle_data(content)
        elif cmd == CMD_END:
            self._handle_end()
        elif cmd == CMD_ABORT:
            self._boot_started = False
            self._reply(cmd, bytes([BOOT_ERR_NONE]))

    # ------------------------------------------------------------ Boot 升级
    def _handle_start(self, content: bytes) -> None:
        if len(content) < 8:
            self._reply(CMD_START, bytes([0x01]))
            return
        self.boot_size = int.from_bytes(content[0:4], "little")
        self.boot_checksum = int.from_bytes(content[4:8], "little")
        self.boot_data = bytearray()
        self._expected_block = 0
        self._boot_started = True
        self.boot_done = False
        self._reply(CMD_START, bytes([BOOT_ERR_NONE]))

    def _handle_data(self, content: bytes) -> None:
        if self.block_delay:
            time.sleep(self.block_delay)
        if not self._boot_started:
            self._record_boot_error(CMD_DATA, ERR_BAD_STATE)
            return
        if len(content) < 4:
            self._record_boot_error(CMD_DATA, 0x01)
            return
        block = content[0] | (content[1] << 8)
        crc = content[2] | (content[3] << 8)
        chunk = content[4:]
        if block != self._expected_block:
            self._record_boot_error(CMD_DATA, ERR_BAD_BLOCK)
            return
        if crc16_xmodem(chunk) != crc:
            self._record_boot_error(CMD_DATA, ERR_BAD_CRC16)
            return
        self.boot_data.extend(chunk)
        self._expected_block += 1
        self._reply(CMD_DATA, bytes([BOOT_ERR_NONE]))

    def _handle_end(self) -> None:
        if not self._boot_started:
            self._record_boot_error(CMD_END, ERR_BAD_STATE)
            return
        if (len(self.boot_data) != self.boot_size
                or (sum(self.boot_data) & 0xFFFFFFFF) != self.boot_checksum):
            self._record_boot_error(CMD_END, ERR_BAD_SIZE)
            return
        self.boot_done = True
        self._reply(CMD_END, bytes([BOOT_ERR_NONE]))

    def _record_boot_error(self, cmd: int, code: int) -> None:
        self.boot_errors.append(code)
        self._reply(cmd, bytes([code]))
