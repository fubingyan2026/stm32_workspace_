# -*- coding: utf-8 -*-
"""高层客户端：覆盖上位机的全部功能（查询 / 控制 / 清锁存 / 升级）。

用法::

    from ctu_sdk import CtuClient, MASTER, SLAVER

    with CtuClient("/dev/ttyUSB0", baud=115200) as ctu:
        print(ctu.read_status(MASTER))
        print(ctu.read_voltage(SLAVER))
        ctu.set_outputs(0x03, fill_duty=500)     # SLAVER: 24V + 12V_ISO 开
        ctu.upgrade(MASTER, "E1_MASTER_POWER_CTU.bin")

所有请求/应答都是**同步阻塞**的；``CtuClient`` 内部用一把可重入锁串行化总线访问，
因此可以安全地与 :class:`~ctu_sdk.poller.CtuPoller` 或多线程一起使用。
"""

from __future__ import annotations

import os
import threading
import time
from typing import Callable

from serial.tools import list_ports

from .devices import (
    ALL_DEVICES,
    MASTER,
    SLAVER,
    device_addr,
    boot_device_id,
    normalize_device,
)
from .errors import (
    CtuDeviceError,
    CtuNotConnected,
    CtuTimeout,
    CtuUnexpectedReply,
    FirmwareImageError,
    UpgradeError,
)
from .firmware import (
    MAX_FW_SIZE,
    BootProtoSender,
    check_app_image,
    inspect_firmware,
    request_boot,
)
from .models import (
    Ack,
    FirmwareInfo,
    MasterStatus,
    MasterTemperature,
    MasterVoltage,
    SlaverStatus,
    SlaverTemperature,
    SlaverVoltage,
)
from .protocol import (
    CMD_CTRL,
    CMD_ERR,
    CMD_READ_INFO,
    CMD_READ_STATUS,
    CMD_READ_TEMP,
    CMD_READ_VOLT,
    CMD_REPLY_FLAG,
    CMD_RESET_LATCH,
    CMD_UPGRADE,
    build_mst_ctrl,
    build_read_info,
    build_read_status,
    build_read_temp,
    build_read_volt,
    build_reset_latch,
    build_slv_ctrl,
    build_upgrade,
    parse_addr,
    parse_cmd,
    parse_payload,
)
from .transport import LogCallback, SerialTransport

#: 默认单次请求超时（秒）
DEFAULT_TIMEOUT = 0.2

#: 升级前 0x06 邀请后等待板端进入 Boot 的时间（秒）
BOOT_INVITE_DELAY = 0.5

ProgressCallback = Callable[[float], None]   # 0.0 ~ 1.0
PhaseCallback = Callable[[str], None]
CancelEvent = threading.Event


def list_serial_ports() -> list[str]:
    """列出本机可用串口设备名（Linux 下形如 ``/dev/ttyUSB0``）。"""
    return [info.device for info in list_ports.comports()]


class CtuClient:
    """E1 CTU 电源板（MASTER + SLAVER 同一 RS485 总线）客户端。"""

    def __init__(self, port: str | None = None, baud: int = 115200,
                 timeout: float = DEFAULT_TIMEOUT,
                 logger: LogCallback | None = None) -> None:
        self._timeout = float(timeout)
        self._transport: SerialTransport | None = None
        if port:
            self._transport = SerialTransport(port, baud, logger=logger)
            self._transport.open()

    # ------------------------------------------------------------------ 生命周期
    def open(self, port: str | None = None, baud: int | None = None) -> None:
        """打开串口（``port``/``baud`` 省略时复用构造时的设置）。"""
        if self._transport is None:
            if not port:
                raise ValueError("未指定串口")
            self._transport = SerialTransport(port, baud or 115200)
        elif baud is not None and baud != self._transport.baud:
            # 改波特率需重开串口
            current_port = port or self._transport.port
            self._transport.close()
            self._transport = SerialTransport(current_port, baud)
        self._transport.open()

    def close(self) -> None:
        if self._transport is not None:
            self._transport.close()

    def __enter__(self) -> "CtuClient":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # ------------------------------------------------------------------ 属性
    @property
    def transport(self) -> SerialTransport:
        if self._transport is None:
            raise CtuNotConnected("串口未打开")
        return self._transport

    @property
    def is_open(self) -> bool:
        return bool(self._transport and self._transport.is_open)

    @property
    def port(self) -> str:
        return self._transport.port if self._transport else ""

    @property
    def baud(self) -> int:
        return self._transport.baud if self._transport else 0

    # ------------------------------------------------------------------ 事务
    def send(self, frame: bytes) -> None:
        """只发送、不等应答（用于需要自行接收的场景）。"""
        self.transport.write(frame)

    def read_frame(self, timeout: float | None = None) -> bytes | None:
        """读取任意一个合法帧（不校验设备/命令），超时返回 ``None``。"""
        return self.transport.read_frame(
            self._timeout if timeout is None else timeout)

    def request(self, frame: bytes, device: object, command: int,
                timeout: float | None = None) -> bytes:
        """发送 ``frame`` 并等待 ``device`` 对该 ``command`` 的应答。

        @return 应答的**内容段**（已去掉 payload 首字节设备 ID）
        @raise CtuTimeout         超时未收到应答
        @raise CtuDeviceError     设备返回 0x7F 错误应答
        @raise CtuUnexpectedReply 收到了不匹配的应答
        @raise CtuNotConnected    串口未打开
        """
        name = normalize_device(device)
        addr = device_addr(name)
        expected = command & ~CMD_REPLY_FLAG
        wait = self._timeout if timeout is None else float(timeout)

        if not self.is_open:
            raise CtuNotConnected("串口未打开")

        with self.transport.lock:
            self.transport.flush_input()
            self.transport.write(frame)
            deadline = time.monotonic() + wait
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CtuTimeout(
                        f"{name} 对 cmd=0x{command:02X} 的应答超时（{wait * 1000:.0f} ms）")
                reply = self.transport.read_frame(remaining)
                if reply is None:
                    raise CtuTimeout(
                        f"{name} 对 cmd=0x{command:02X} 的应答超时（{wait * 1000:.0f} ms）")

                reply_addr = parse_addr(reply)
                reply_cmd = parse_cmd(reply)
                payload = parse_payload(reply)

                if reply_cmd == CMD_ERR:
                    if reply_addr != addr:
                        continue  # 其它设备的错误应答，忽略
                    content = payload[1:] if payload else b""
                    raise CtuDeviceError(name, content[0] if content else 0xFF)
                if reply_addr != addr:
                    continue  # 总线上其它设备的帧
                if (reply_cmd & ~CMD_REPLY_FLAG) != expected:
                    raise CtuUnexpectedReply(expected, reply_cmd)
                return payload[1:] if payload else b""

    def _ack(self, frame: bytes, device: object, command: int,
             timeout: float | None = None) -> Ack:
        name = normalize_device(device)
        started = time.monotonic()
        self.request(frame, name, command, timeout)
        return Ack(device=name, command=command,
                   elapsed_ms=(time.monotonic() - started) * 1000.0)

    # ------------------------------------------------------------------ 查询
    def read_status(self, device: object, timeout: float | None = None):
        """0x01 读系统状态。

        @return MASTER → :class:`MasterStatus`；SLAVER → :class:`SlaverStatus`
        """
        name = normalize_device(device)
        content = self.request(build_read_status(device_addr(name)), name,
                               CMD_READ_STATUS, timeout)
        if name == MASTER:
            return MasterStatus.from_payload(content)
        return SlaverStatus.from_payload(content)

    def read_voltage(self, device: object, timeout: float | None = None):
        """0x02 读电压（mV）。

        @return MASTER → :class:`MasterVoltage`；SLAVER → :class:`SlaverVoltage`
        """
        name = normalize_device(device)
        content = self.request(build_read_volt(device_addr(name)), name,
                               CMD_READ_VOLT, timeout)
        if name == MASTER:
            return MasterVoltage.from_payload(content)
        return SlaverVoltage.from_payload(content)

    def read_temperature(self, device: object, timeout: float | None = None):
        """0x03 读温度（°C）。

        @return MASTER → :class:`MasterTemperature`；SLAVER → :class:`SlaverTemperature`
        """
        name = normalize_device(device)
        content = self.request(build_read_temp(device_addr(name)), name,
                               CMD_READ_TEMP, timeout)
        if name == MASTER:
            return MasterTemperature.from_payload(content)
        return SlaverTemperature.from_payload(content)

    def read_info(self, device: object, timeout: float | None = None) -> FirmwareInfo:
        """0x07 读固件 / Boot 信息。"""
        name = normalize_device(device)
        content = self.request(build_read_info(device_addr(name)), name,
                               CMD_READ_INFO, timeout)
        return FirmwareInfo.from_payload(content)

    # ------------------------------------------------------------------ 控制
    def set_buzzer_duty(self, duty: int,
                        timeout: float | None = None) -> Ack:
        """0x04 主控板蜂鸣器占空比（0-50%，超限截断）。"""
        frame = build_mst_ctrl(duty, device_addr(MASTER))
        return self._ack(frame, MASTER, CMD_CTRL, timeout)

    def set_outputs(self, output_mask: int, fill_duty: int = 0,
                    timeout: float | None = None) -> Ack:
        """0x04 副板输出控制。

        @param output_mask 输出位域：bit0=24V, bit1=12V_ISO, bit2=LSD1, bit3=LSD2
        @param fill_duty   补光亮度 0-1000（0.0%-100.0%，超限截断）
        """
        frame = build_slv_ctrl(output_mask, fill_duty, device_addr(SLAVER))
        return self._ack(frame, SLAVER, CMD_CTRL, timeout)

    def set_output(self, channel: str, on: bool, *,
                   mask: int | None = None, timeout: float | None = None) -> Ack:
        """0x04 按通道开关单个输出（``mask`` 为其它通道的当前位域，可选）。

        @param channel ``"24v"`` / ``"12v"`` / ``"lsd1"`` / ``"lsd2"``
        """
        bits = {"24v": 0, "12v": 1, "lsd1": 2, "lsd2": 3}
        key = channel.strip().lower()
        if key not in bits:
            raise ValueError(f"未知输出通道: {channel!r}（可选 24v/12v/lsd1/lsd2）")
        current = 0 if mask is None else int(mask)
        if on:
            current |= 1 << bits[key]
        else:
            current &= ~(1 << bits[key])
        return self.set_outputs(current, timeout=timeout)

    def clear_fault_latch(self, device: object,
                          timeout: float | None = None) -> Ack:
        """0x05 清除故障锁存（两板通用）。"""
        name = normalize_device(device)
        frame = build_reset_latch(device_addr(name))
        return self._ack(frame, name, CMD_RESET_LATCH, timeout)

    def request_upgrade(self, device: object,
                        timeout: float | None = None) -> Ack:
        """0x06 请求升级：板端应答后复位进入 Bootloader（或已在 Boot 则选中会话）。"""
        name = normalize_device(device)
        frame = build_upgrade(device_addr(name))
        return self._ack(frame, name, CMD_UPGRADE, timeout)

    # ------------------------------------------------------------------ 固件升级
    def upgrade(self, device: object, filepath: str, *,
                progress: ProgressCallback | None = None,
                phase: PhaseCallback | None = None,
                log: LogCallback | None = None,
                cancel: CancelEvent | None = None,
                timeout: float | None = None) -> None:
        """完整固件升级：预检 → 0x06 邀请 → SELECT/START/DATA/END。

        @param device   目标设备（master/slaver）
        @param filepath ``.bin`` 固件路径
        @param progress 进度回调，参数为 0.0~1.0
        @param phase    阶段描述回调
        @param log      日志回调 ``(文本, 级别)``
        @param cancel   传入 ``threading.Event``，置位后中止升级（发送 ABORT）
        @raise FirmwareImageError 固件未通过预检
        @raise UpgradeError       升级过程中失败或被中止
        """
        ok, size, reason = inspect_firmware(filepath)
        if not ok:
            raise FirmwareImageError(f"{os.path.basename(filepath)}: {reason}")
        self._run_upgrade(device, filepath=filepath, name=os.path.basename(filepath),
                          size=size, progress=progress, phase=phase, log=log,
                          cancel=cancel, timeout=timeout)

    def upgrade_image(self, device: object, data: bytes,
                      name: str = "image.bin", *,
                      progress: ProgressCallback | None = None,
                      phase: PhaseCallback | None = None,
                      log: LogCallback | None = None,
                      cancel: CancelEvent | None = None,
                      timeout: float | None = None) -> None:
        """与 :meth:`upgrade` 相同，但固件内容来自内存 ``bytes``。"""
        if len(data) > MAX_FW_SIZE:
            raise FirmwareImageError(
                f"固件 {len(data)}B 超出 App 分区容量 96KB")
        reason = check_app_image(data)
        if reason is not None:
            raise FirmwareImageError(reason)
        self._run_upgrade(device, filepath=name, name=name, size=len(data),
                          data=data, progress=progress, phase=phase, log=log,
                          cancel=cancel, timeout=timeout)

    def _run_upgrade(self, device: object, *, filepath: str, name: str, size: int,
                     data: bytes | None = None,
                     progress: ProgressCallback | None = None,
                     phase: PhaseCallback | None = None,
                     log: LogCallback | None = None,
                     cancel: CancelEvent | None = None,
                     timeout: float | None = None) -> None:
        if not self.is_open:
            raise CtuNotConnected("串口未打开")

        boot_id = boot_device_id(device)
        dev_name = normalize_device(device)
        emit_log = log or (lambda _t, _l: None)
        emit_phase = phase or (lambda _t: None)
        emit_progress = progress or (lambda _f: None)

        with self.transport.lock:
            # 1) 0x06 邀请：App 复位进 Boot / 已在 Boot 则等同 SELECT（幂等）
            emit_phase("发送 0x06 邀请 / 等待 Boot")
            emit_log(f"发送 0x06 邀请给 {dev_name}（{name}, {size}B）", "info")
            if not request_boot(self.transport.raw, boot_id,
                                lambda t, l: emit_log(t, l)):
                raise UpgradeError("0x06 邀请发送失败")
            self.transport.reset_parser()
            time.sleep(BOOT_INVITE_DELAY)

            if cancel is not None and cancel.is_set():
                raise UpgradeError("升级被中止")

            # 2) 分块传输（复用 Boot 工程协议层，单一来源）
            sender = BootProtoSender(
                self.transport.raw,
                log_cb=lambda t, l: emit_log(t, l),
                phase_cb=lambda t: emit_phase(t),
                progress_cb=lambda f: emit_progress(min(max(f, 0.0), 1.0)))
            watcher, stop_watch = self._start_cancel_watcher(sender, cancel)
            try:
                transferred = sender.transfer(boot_id, filepath, data=data)
            finally:
                stop_watch.set()
                if watcher is not None:
                    watcher.join(timeout=1.0)
                self.transport.reset_parser()

        if cancel is not None and cancel.is_set():
            raise UpgradeError("升级被中止")
        if not transferred:
            raise UpgradeError("固件传输失败（SELECT/START/DATA/END 未被确认）")

    @staticmethod
    def _start_cancel_watcher(sender: BootProtoSender,
                              cancel: CancelEvent | None):
        """把 ``cancel`` 事件桥接到 ``BootProtoSender.cancel()``。"""
        if cancel is None:
            return None, threading.Event()
        stop = threading.Event()

        def _watch() -> None:
            while not stop.wait(0.05):
                if cancel.is_set():
                    sender.cancel()
                    return

        thread = threading.Thread(target=_watch, name="ctu-upgrade-cancel",
                                  daemon=True)
        thread.start()
        return thread, stop

    # ------------------------------------------------------------------ 便利方法
    def probe(self, device: object, timeout: float | None = None) -> bool:
        """探测设备是否在线（读一次状态是否成功）。"""
        try:
            self.read_status(device, timeout)
            return True
        except (CtuTimeout, CtuDeviceError, CtuUnexpectedReply):
            return False

    def scan(self, devices=ALL_DEVICES,
             timeout: float | None = None) -> dict[str, bool]:
        """批量探测在线状态，返回 ``{设备名: 是否在线}``。"""
        return {normalize_device(d): self.probe(d, timeout) for d in devices}
