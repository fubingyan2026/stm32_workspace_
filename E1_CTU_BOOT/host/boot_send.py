#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E1_CTU_BOOT 寻址分块升级 CLI（协议层 host/boot_protocol.py，GUI 见 host/boot_host.py）

用法:
    python host/boot_send.py COMx path/to/app.bin [baud] [master|slaver|boot]

    master/slaver : 目标设备 ID 0x01/0x02（Boot 据此定向应答）
    boot          : 广播 ID 0x00（未定 ID / 空片 Boot 也可选中）
    统一先发一帧 0x06：App 会复位进入 Boot；已在 Boot 则等同 SELECT（幂等）
"""

import sys
import time

import serial

from boot_protocol import (
    DEVICE_BOOT, DEVICE_MASTER, DEVICE_SLAVER, BootProtoSender, request_boot,
)


def _print_log(text: str, level: str = "info") -> None:
    tag = {"warn": "[WARN] ", "error": "[ERR]  "}.get(level, "")
    print(f"{tag}{text}")


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    com, path = sys.argv[1], sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
    device = sys.argv[4].lower() if len(sys.argv) > 4 else DEVICE_BOOT

    with serial.Serial(com, baud, timeout=0) as ser:
        ser.reset_input_buffer()
        request_boot(ser, device, _print_log)
        time.sleep(0.5)
        sender = BootProtoSender(ser, log_cb=_print_log,
                                 phase_cb=lambda t: print(f">> {t}"),
                                 progress_cb=lambda f: print(f">> 进度 {f * 100:.0f}%"))
        ok = sender.transfer(device, path)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
