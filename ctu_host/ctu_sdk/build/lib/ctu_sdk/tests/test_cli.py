# -*- coding: utf-8 -*-
"""CLI 测试：通过 pty 虚拟串口验证 ``python -m ctu_sdk`` 各子命令。"""

from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import unittest
from pathlib import Path

if os.name != "posix":  # pragma: no cover - pty 仅 POSIX
    raise unittest.SkipTest("CLI 回环测试需要 POSIX pty")

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ctu_sdk.cli import main as cli_main  # noqa: E402
from ctu_sdk.tests.virtual import (  # noqa: E402
    make_firmware_image,
    virtual_port,
)


def run_cli(argv: list[str]) -> tuple[int, str, str]:
    """执行 CLI 并捕获 stdout / stderr。"""
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = cli_main(argv)
    return code, out.getvalue(), err.getvalue()


class CliTests(unittest.TestCase):
    def test_ports(self) -> None:
        code, out, _ = run_cli(["ports"])
        self.assertEqual(code, 0)
        self.assertTrue(out.strip())

    def test_missing_port_exits(self) -> None:
        with self.assertRaises(SystemExit):
            run_cli(["status", "--device", "master"])

    def test_status_json(self) -> None:
        with virtual_port("master") as (board, port):
            board.status_payload = bytes([0x01, 0x00])  # 急停
            code, out, _ = run_cli(["-p", port, "status", "--device", "master",
                                    "--json"])
            self.assertEqual(code, 0)
            payload = json.loads(out)
            self.assertTrue(payload["estop"])
            self.assertFalse(payload["rail_12v_fault"])

    def test_volt_and_temp(self) -> None:
        with virtual_port("master") as (board, port):
            code, out, _ = run_cli(["-p", port, "volt", "--device", "master"])
            self.assertEqual(code, 0)
            self.assertIn("vin_mv", out)
            code, out, _ = run_cli(["-p", port, "temp", "--device", "master"])
            self.assertEqual(code, 0)
            self.assertIn("mcu_c", out)

    def test_info(self) -> None:
        with virtual_port("master") as (board, port):
            code, out, _ = run_cli(["-p", port, "info", "--device", "master"])
            self.assertEqual(code, 0)
            self.assertIn("App 版本", out)

    def test_buzzer(self) -> None:
        with virtual_port("master") as (board, port):
            code, _, _ = run_cli(["-p", port, "buzzer", "--duty", "30"])
            self.assertEqual(code, 0)
            self.assertEqual(board.buzzer_duty, 30)

    def test_output_channels_preserve(self) -> None:
        with virtual_port("slaver") as (board, port):
            board.status_payload = bytes([0x00, 0x01])  # 24V 已开
            code, _, _ = run_cli(["-p", port, "output", "--on", "12v",
                                  "--duty", "500", "--preserve"])
            self.assertEqual(code, 0)
            self.assertEqual(board.output_mask, 0x03)
            self.assertEqual(board.fill_duty, 500)

    def test_clear_latch_and_request_upgrade(self) -> None:
        with virtual_port("slaver") as (board, port):
            self.assertEqual(
                run_cli(["-p", port, "clear-latch", "--device", "slaver"])[0], 0)
            self.assertEqual(board.latch_clears, 1)
            self.assertEqual(
                run_cli(["-p", port, "request-upgrade", "--device", "slaver"])[0], 0)
            self.assertTrue(board.boot_entered)

    def test_scan(self) -> None:
        with virtual_port("master") as (board, port):
            code, out, _ = run_cli(["-p", port, "scan"])
            self.assertEqual(code, 0)
            self.assertIn("master", out)

    def test_upgrade(self) -> None:
        image = Path("/tmp") / f"ctu_cli_{os.getpid()}.bin"
        image.write_bytes(make_firmware_image(800))
        self.addCleanup(lambda: image.unlink(missing_ok=True))

        with virtual_port("master") as (board, port):
            code, out, _ = run_cli(["-p", port, "upgrade", "--device", "master",
                                    "--file", str(image), "--json"])
            self.assertEqual(code, 0)
            self.assertTrue(board.boot_done)
            self.assertEqual(bytes(board.boot_data), image.read_bytes())

    def test_upgrade_bad_image_returns_error(self) -> None:
        image = Path("/tmp") / f"ctu_cli_bad_{os.getpid()}.bin"
        image.write_bytes(b"\x00" * 32)
        self.addCleanup(lambda: image.unlink(missing_ok=True))

        with virtual_port("master") as (board, port):
            code, _, err = run_cli(["-p", port, "upgrade", "--device", "master",
                                    "--file", str(image)])
            self.assertEqual(code, 1)
            self.assertIn("错误", err)


if __name__ == "__main__":
    unittest.main(verbosity=2)
