#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E1_CTU_BOOT YMODEM 发送 CLI（协议层在 boot_protocol.py，UI 见 boot_host.py）

用法:
    python host/ymodem_send.py COMx path/to/app.bin [baud=115200] [master|slaver|boot]

    默认 boot：直接等待板端 'C'（板端须已进入升级模式）；
    传 master/slaver 时先向运行中的 App 发送带地址的升级请求(0x01/0x06、0x02/0x06)
    再等待其复位进入 Boot。
"""

import os
import sys

import serial

from boot_protocol import DEVICE_BOOT, DEVICE_MASTER, DEVICE_SLAVER, YmodemSender, request_boot


def _print_log(text: str, level: str = "info") -> None:
    tag = {"warn": "[WARN] ", "error": "[ERR]  "}.get(level, "")
    print(f"{tag}{text}")


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    com = sys.argv[1]
    path = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
    # 可选第 4 参：master / slaver → 先向运行中的 App 发升级请求进入 Boot
    device = sys.argv[4].lower() if len(sys.argv) > 4 else DEVICE_BOOT

    with serial.Serial(com, baud, timeout=0) as ser:
        ser.reset_input_buffer()
        if device in (DEVICE_MASTER, DEVICE_SLAVER):
            request_boot(ser, device, _print_log)
        sender = YmodemSender(ser, log_cb=_print_log,
                              phase_cb=lambda t: print(f">> {t}"),
                              progress_cb=lambda f: print(f">> 进度 {f * 100:.0f}%"))
        # 调试：BOOT_YM_128=1 时用 128B 小包定位 1KB 突发链路问题
        use_1k = os.environ.get("BOOT_YM_128", "0") != "1"
        if not use_1k:
            print(">> 使用 128B(SOH) 数据包模式")
        ok = sender.send_file(path, use_1k=use_1k)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
