#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固件升级示例：预检 → 0x06 邀请 → 分块传输 → 支持 Ctrl+C 中止。

用法::

    python3 examples/upgrade_firmware.py /dev/ttyUSB0 master E1_MASTER_POWER_CTU.bin
"""

from __future__ import annotations

import argparse
import os
import sys
import threading

from ctu_sdk import CtuClient, inspect_firmware


def main() -> int:
    parser = argparse.ArgumentParser(description="E1 CTU 电源板固件升级")
    parser.add_argument("port", help="串口设备，如 /dev/ttyUSB0")
    parser.add_argument("device", choices=("master", "slaver"))
    parser.add_argument("firmware", help="固件 .bin 路径")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args()

    ok, size, reason = inspect_firmware(args.firmware)
    print(f"固件: {os.path.basename(args.firmware)}  {size} B  "
          f"{'预检通过' if ok else '预检失败: ' + str(reason)}")
    if not ok:
        return 2

    # Ctrl+C → 置位 cancel 事件，SDK 会发送 ABORT 中止升级
    cancel = threading.Event()

    def on_progress(fraction: float) -> None:
        print(f"\r进度 {fraction * 100:5.1f}%", end="", flush=True)

    def on_phase(text: str) -> None:
        print(f"\n· {text}")

    try:
        with CtuClient(args.port, args.baud, timeout=args.timeout) as ctu:
            ctu.upgrade(args.device, args.firmware, progress=on_progress,
                        phase=on_phase, log=lambda t, l: None, cancel=cancel)
    except KeyboardInterrupt:
        cancel.set()
        print("\n已请求中止", file=sys.stderr)
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"\n升级失败: {exc}", file=sys.stderr)
        return 1

    print("\n升级完成，板端将复位/提升新固件")
    return 0


if __name__ == "__main__":
    sys.exit(main())
