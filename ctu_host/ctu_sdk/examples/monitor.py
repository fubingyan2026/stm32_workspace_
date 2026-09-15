#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""周期轮询示例：打印两板状态与通信统计，异常时高亮提示。

用法::

    python3 examples/monitor.py /dev/ttyUSB0            # 默认间隔 0.5 s
    python3 examples/monitor.py /dev/ttyUSB0 --interval 0.2
"""

from __future__ import annotations

import argparse
import sys
import time

from ctu_sdk import MASTER, SLAVER, CtuClient, CtuPoller


def main() -> int:
    parser = argparse.ArgumentParser(description="E1 CTU 电源板轮询监测")
    parser.add_argument("port", help="串口设备，如 /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args()

    state: dict[str, str] = {}

    def on_status(device: str, status) -> None:
        state[device] = "正常" if status.healthy else "异常: " + ",".join(status.faults)

    def on_temperature(device: str, temperature) -> None:
        stamp = time.strftime("%H:%M:%S")
        print(f"[{stamp}] {device:<6} {state.get(device, '?'):<24}"
              f" mcu={temperature.mcu_c:.2f}°C")

    def on_error(device: str, command: int, exc: Exception) -> None:
        print(f"  ! {device} cmd=0x{command:02X} 失败: {exc}", file=sys.stderr)

    def on_stats(stats) -> None:
        print(f"  -- tx={stats.tx} rx={stats.rx} 丢包={stats.loss} "
              f"({stats.loss_rate:.1f}%) fps={stats.fps:.1f}", file=sys.stderr)

    try:
        with CtuClient(args.port, args.baud, timeout=args.timeout) as ctu:
            if not any(ctu.scan().values()):
                print("两条板均无应答，请检查串口与接线", file=sys.stderr)
                return 1
            with CtuPoller(ctu, [MASTER, SLAVER], args.interval,
                           on_status=on_status, on_temperature=on_temperature,
                           on_error=on_error, on_stats=on_stats):
                while True:
                    time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n已停止")
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
