# -*- coding: utf-8 -*-
"""命令行工具：``python -m ctu_sdk <command>``。

覆盖 SDK 的全部能力，便于在 Linux 上脚本化调试与巡检。

示例::

    python -m ctu_sdk ports
    python -m ctu_sdk -p /dev/ttyUSB0 scan
    python -m ctu_sdk -p /dev/ttyUSB0 status --device master
    python -m ctu_sdk -p /dev/ttyUSB0 output --on 24v,12v --duty 500
    python -m ctu_sdk -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
    python -m ctu_sdk -p /dev/ttyUSB0 monitor --interval 0.5
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

from . import __version__
from .client import DEFAULT_TIMEOUT, CtuClient, list_serial_ports
from .devices import ALL_DEVICES, MASTER, SLAVER, normalize_device
from .errors import CtuError
from .poller import CtuPoller
from .protocol import CMD_NAME

DEVICE_CHOICES = (MASTER, SLAVER)


# --------------------------------------------------------------------- 通用
def _add_common(parser: argparse.ArgumentParser, suppress: bool = False) -> None:
    """公共参数；``suppress=True`` 用于子命令副本，避免覆盖顶层已解析值。"""
    default: Any = argparse.SUPPRESS if suppress else None
    parser.add_argument("-p", "--port", default=default,
                        help="串口设备，如 /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=default,
                        help="波特率（默认 115200）")
    parser.add_argument("--timeout", type=float, default=default,
                        help=f"单次请求超时秒数（默认 {DEFAULT_TIMEOUT}）")
    parser.add_argument("--json", action="store_true", default=default,
                        help="以 JSON 输出结果")
    parser.add_argument("-v", "--verbose", action="store_true", default=default,
                        help="打印收发帧日志到 stderr")


def _make_logger(verbose: bool):
    if not verbose:
        return None

    def _log(text: str, level: str) -> None:
        print(f"  [{level}] {text}", file=sys.stderr)

    return _log


def _emit(args, payload: Any, text_lines: list[str]) -> None:
    if args.json:
        if hasattr(payload, "to_dict"):
            payload = payload.to_dict()
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        for line in text_lines:
            print(line)


def _client(args) -> CtuClient:
    if not args.port:
        raise SystemExit("请用 -p/--port 指定串口（可用 `ports` 子命令列出）")
    return CtuClient(args.port, args.baud or 115200,
                     timeout=args.timeout or DEFAULT_TIMEOUT,
                     logger=_make_logger(args.verbose))


# --------------------------------------------------------------------- 命令
def _cmd_ports(args) -> int:
    ports = list_serial_ports()
    _emit(args, ports, ports or ["(未发现串口)"])
    return 0


def _cmd_scan(args) -> int:
    with _client(args) as ctu:
        result = ctu.scan(timeout=args.timeout)
    _emit(args, result,
          [f"{name:<8} {'在线' if online else '离线'}" for name, online in result.items()])
    return 0


def _cmd_status(args) -> int:
    with _client(args) as ctu:
        status = ctu.read_status(args.device)
    if args.json:
        _emit(args, status, [])
        return 0
    lines = [f"{args.device}: {'正常' if status.healthy else '存在异常'}"]
    lines += [f"  {key} = {value}" for key, value in status.to_dict().items()
              if key != "raw"]
    if status.faults:
        lines.append(f"  异常项: {', '.join(status.faults)}")
    _emit(args, status, lines)
    return 0


def _cmd_volt(args) -> int:
    with _client(args) as ctu:
        voltage = ctu.read_voltage(args.device)
    _emit(args, voltage,
          [f"{key} = {value} mV" for key, value in voltage.to_dict().items()])
    return 0


def _cmd_temp(args) -> int:
    with _client(args) as ctu:
        temperature = ctu.read_temperature(args.device)
    _emit(args, temperature,
          [f"{key} = {value}" for key, value in temperature.to_dict().items()])
    return 0


def _cmd_info(args) -> int:
    with _client(args) as ctu:
        info = ctu.read_info(args.device)
    lines = [
        f"App 版本    : v{info.app_version}",
        f"Boot 版本   : v{info.meta_version}",
        f"固件大小    : {info.fw_size} B",
        f"固件校验和  : 0x{info.fw_checksum:08X}",
        f"上电次数    : {info.reboot_counts}",
        f"标志        : {info.flags_text} (0x{info.flags:02X})",
    ]
    _emit(args, info, lines)
    return 0


def _cmd_buzzer(args) -> int:
    with _client(args) as ctu:
        ack = ctu.set_buzzer_duty(args.duty)
    _emit(args, ack, [f"蜂鸣器占空比 {args.duty}% 已发送（{ack.elapsed_ms:.1f} ms）"])
    return 0


def _cmd_output(args) -> int:
    changes: list[tuple[bool, int]] = []
    if args.on:
        changes += [(True, _channel_bit(name))
                    for name in args.on.split(",") if name.strip()]
    if args.off:
        changes += [(False, _channel_bit(name))
                    for name in args.off.split(",") if name.strip()]

    with _client(args) as ctu:
        if args.mask is not None:
            mask = args.mask
        elif args.preserve:
            mask = ctu.read_status(SLAVER).output_mask
        else:
            mask = 0
        for turn_on, bit in changes:
            mask = (mask | bit) if turn_on else (mask & ~bit)
        ack = ctu.set_outputs(mask, args.duty)
    _emit(args, {**ack.to_dict(), "mask": mask},
          [f"输出位域 0x{mask:02X}，补光 {args.duty} 已发送"
           f"（{ack.elapsed_ms:.1f} ms）"])
    return 0


def _cmd_clear_latch(args) -> int:
    with _client(args) as ctu:
        ack = ctu.clear_fault_latch(args.device)
    _emit(args, ack, [f"{args.device} 清除故障锁存已发送（{ack.elapsed_ms:.1f} ms）"])
    return 0


def _cmd_request_upgrade(args) -> int:
    with _client(args) as ctu:
        ack = ctu.request_upgrade(args.device)
    _emit(args, ack,
          [f"{args.device} 升级请求已确认，板端将复位进入 Bootloader"])
    return 0


def _cmd_upgrade(args) -> int:
    def _progress(fraction: float) -> None:
        if not args.json:
            print(f"\r  进度 {fraction * 100:5.1f}%", end="", file=sys.stderr)

    with _client(args) as ctu:
        ctu.upgrade(args.device, args.file, progress=_progress,
                    phase=lambda text: print(f"  · {text}", file=sys.stderr))
    if not args.json:
        print("", file=sys.stderr)
    _emit(args, {"device": args.device, "file": args.file, "ok": True},
          [f"{args.device} 升级完成：{args.file}"])
    return 0


def _cmd_monitor(args) -> int:
    devices = [normalize_device(d) for d in args.device]
    snapshots: dict[str, dict] = {name: {} for name in devices}
    last_stats = 0.0

    def _print_snapshot(device: str) -> None:
        snap = snapshots[device]
        status = snap.get("status")
        voltage = snap.get("voltage")
        temperature = snap.get("temperature")
        parts = [time.strftime("%H:%M:%S"), f"{device:<6}"]
        if status is not None:
            parts.append("正常" if status.healthy else
                         "异常[" + ",".join(status.faults) + "]")
        if voltage is not None:
            parts.append(" ".join(f"{k}={v}mV"
                                  for k, v in voltage.to_dict().items()))
        if temperature is not None:
            parts.append(" ".join(f"{k}={v}"
                                  for k, v in temperature.to_dict().items()))
        print("  ".join(parts))

    def _on_status(device, result):
        snapshots[device]["status"] = result

    def _on_voltage(device, result):
        snapshots[device]["voltage"] = result

    def _on_temperature(device, result):
        snapshots[device]["temperature"] = result
        _print_snapshot(device)

    def _on_error(device, command, exc):
        name = CMD_NAME.get(command, f"0x{command:02X}")
        print(f"{time.strftime('%H:%M:%S')}  {device:<6} {name} 失败: {exc}",
              file=sys.stderr)

    def _on_stats(stats):
        nonlocal last_stats
        now = time.monotonic()
        if now - last_stats < 2.0:
            return
        last_stats = now
        print(f"  -- tx={stats.tx} rx={stats.rx} 丢包={stats.loss} "
              f"({stats.loss_rate:.1f}%) fps={stats.fps:.1f}", file=sys.stderr)

    with _client(args) as ctu:
        poller = CtuPoller(ctu, devices, args.interval,
                           on_status=_on_status, on_voltage=_on_voltage,
                           on_temperature=_on_temperature, on_error=_on_error,
                           on_stats=_on_stats)
        poller.start()
        try:
            if args.duration:
                time.sleep(args.duration)
            else:
                while True:
                    time.sleep(0.3)
        except KeyboardInterrupt:
            print("\n已停止", file=sys.stderr)
        finally:
            poller.stop()
            poller.join(2.0)
        _on_stats(poller.stats)
    return 0


_CHANNELS = {"24v": 0, "12v": 1, "lsd1": 2, "lsd2": 3}


def _channel_bit(name: str) -> int:
    key = name.strip().lower()
    if key not in _CHANNELS:
        raise SystemExit(f"未知输出通道: {name}（可选 24v/12v/lsd1/lsd2）")
    return 1 << _CHANNELS[key]


# --------------------------------------------------------------------- 入口
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ctu_sdk",
        description="E1 CTU 电源板 RS485 SDK 命令行工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("--version", action="version",
                        version=f"ctu_sdk {__version__}")
    _add_common(parser)

    common = argparse.ArgumentParser(add_help=False)
    _add_common(common, suppress=True)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", parents=[common], help="列出可用串口").set_defaults(func=_cmd_ports)
    sub.add_parser("scan", parents=[common], help="探测两板在线状态").set_defaults(func=_cmd_scan)

    p = sub.add_parser("status", parents=[common], help="读系统状态 (0x01)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_status)

    p = sub.add_parser("volt", parents=[common], help="读电压 (0x02)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_volt)

    p = sub.add_parser("temp", parents=[common], help="读温度 (0x03)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_temp)

    p = sub.add_parser("info", parents=[common], help="读固件信息 (0x07)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_info)

    p = sub.add_parser("buzzer", parents=[common], help="主控蜂鸣器 (0x04)")
    p.add_argument("--duty", type=int, default=0, help="占空比 0-50%%（百分号需转义）")
    p.set_defaults(func=_cmd_buzzer)

    p = sub.add_parser("output", parents=[common], help="副板输出控制 (0x04)")
    p.add_argument("--mask", type=lambda v: int(v, 0), default=None,
                   help="输出位域（如 0x03）")
    p.add_argument("--on", default=None, help="打开的通道，逗号分隔：24v,12v,lsd1,lsd2")
    p.add_argument("--off", default=None, help="关闭的通道，逗号分隔")
    p.add_argument("--duty", type=int, default=0, help="补光亮度 0-1000")
    p.add_argument("--preserve", action="store_true",
                   help="先读回当前位域，再叠加 --on/--off")
    p.set_defaults(func=_cmd_output)

    p = sub.add_parser("clear-latch", parents=[common], help="清除故障锁存 (0x05)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_clear_latch)

    p = sub.add_parser("request-upgrade", parents=[common], help="升级请求 (0x06)")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.set_defaults(func=_cmd_request_upgrade)

    p = sub.add_parser("upgrade", parents=[common], help="固件升级（分块传输）")
    p.add_argument("--device", required=True, choices=DEVICE_CHOICES)
    p.add_argument("--file", required=True, help="固件 .bin 路径")
    p.set_defaults(func=_cmd_upgrade)

    p = sub.add_parser("monitor", parents=[common], help="周期轮询监测")
    p.add_argument("--device", action="append", choices=DEVICE_CHOICES,
                   default=None, help="可多次指定，默认两板")
    p.add_argument("--interval", type=float, default=0.5, help="轮询间隔秒（默认 0.5）")
    p.add_argument("--duration", type=float, default=0.0, help="持续秒数，0=直到 Ctrl+C")
    p.set_defaults(func=_cmd_monitor)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "monitor" and not args.device:
        args.device = list(ALL_DEVICES)
    try:
        return int(args.func(args))
    except CtuError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
