# -*- coding: utf-8 -*-
"""ctu_sdk 端到端测试（通过 pty 虚拟串口 + 虚拟电源板，无需真实硬件）。

运行（Linux / WSL）::

    cd ctu_host
    python3 -m unittest discover -s ctu_sdk/tests -t . -v
"""

from __future__ import annotations

import os
import struct
import sys
import threading
import time
import unittest
from pathlib import Path

if os.name != "posix":  # pragma: no cover - pty 仅 POSIX
    raise unittest.SkipTest("ctu_sdk 回环测试需要 POSIX pty")

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ctu_sdk import (  # noqa: E402
    MASTER,
    SLAVER,
    CtuClient,
    CtuDeviceError,
    CtuNotConnected,
    CtuPoller,
    CtuTimeout,
    FirmwareImageError,
    MasterStatus,
    MasterTemperature,
    MasterVoltage,
    SlaverStatus,
    UpgradeError,
    inspect_firmware,
    list_serial_ports,
)
from ctu_sdk.tests.virtual import (  # noqa: E402
    make_firmware_image,
    virtual_board,
)


class ReadTests(unittest.TestCase):
    def test_list_ports_returns_list(self) -> None:
        self.assertIsInstance(list_serial_ports(), list)

    def test_master_status(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            board.status_payload = bytes([0x01 | 0x02, 0x00])  # 急停 + 12V 异常
            status = ctu.read_status(MASTER)
            self.assertIsInstance(status, MasterStatus)
            self.assertTrue(status.estop)
            self.assertTrue(status.rail_12v_fault)
            self.assertFalse(status.rail_24v_fault)
            self.assertFalse(status.healthy)
            self.assertEqual(status.faults, ["急停", "12V"])

    def test_master_voltage_and_temperature(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            board.voltage_payload = (48000).to_bytes(2, "little") + (47900).to_bytes(2, "little")
            board.temperature_payload = (struct.pack("<h", 2510)
                                         + struct.pack("<h", -500)
                                         + struct.pack("<h", 3600))
            voltage = ctu.read_voltage(MASTER)
            temperature = ctu.read_temperature(MASTER)
            self.assertIsInstance(voltage, MasterVoltage)
            self.assertEqual((voltage.vin_mv, voltage.vin_dcdc_mv), (48000, 47900))
            self.assertIsInstance(temperature, MasterTemperature)
            self.assertAlmostEqual(temperature.ntc1_c, 25.10)
            self.assertAlmostEqual(temperature.ntc2_c, -5.00)
            self.assertAlmostEqual(temperature.mcu_c, 36.00)

    def test_slaver_status_and_voltage(self) -> None:
        with virtual_board(SLAVER) as (board, ctu):
            # byte0 bit5 = err_lsd2；byte1 bit0/1 = 24V / 12V_ISO 输出开
            board.status_payload = bytes([0x20, 0x03])
            board.voltage_payload = b"".join(
                v.to_bytes(2, "little") for v in (12000, 24000, 500, 0))
            status = ctu.read_status(SLAVER)
            voltage = ctu.read_voltage(SLAVER)
            self.assertIsInstance(status, SlaverStatus)
            self.assertTrue(status.out_24v and status.out_12v)
            self.assertFalse(status.out_lsd1)
            self.assertEqual(status.output_mask, 0x03)
            self.assertTrue(status.fault_lsd2)
            self.assertEqual(status.faults, ["LSD2"])
            self.assertEqual(voltage.aux_mv, 12000)
            self.assertEqual(voltage.motor_mv, 24000)
            self.assertEqual(voltage.lsd1_mv, 500)

    def test_firmware_info(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            info = ctu.read_info(MASTER)
            self.assertEqual(info.app_version, 1)
            self.assertEqual(info.meta_version, 2)
            self.assertEqual(info.fw_size, 0x4000)
            self.assertEqual(info.fw_checksum, 0x00EFCDAB)
            self.assertEqual(info.reboot_counts, 3)
            self.assertTrue(info.meta_valid)
            self.assertFalse(info.upgrade_pending)

    def test_scan_reports_online(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            self.assertTrue(ctu.probe(MASTER))
            self.assertTrue(ctu.scan(timeout=0.2)[MASTER])


class ControlTests(unittest.TestCase):
    def test_buzzer_control(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            ack = ctu.set_buzzer_duty(25)
            self.assertEqual(board.buzzer_duty, 25)
            self.assertTrue(ack.elapsed_ms >= 0)

    def test_buzzer_duty_clamped(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            ctu.set_buzzer_duty(200)
            self.assertEqual(board.buzzer_duty, 50)  # 协议层截断到 50%

    def test_output_mask_and_fill_duty(self) -> None:
        with virtual_board(SLAVER) as (board, ctu):
            ctu.set_outputs(0x03, fill_duty=500)
            self.assertEqual(board.output_mask, 0x03)
            self.assertEqual(board.fill_duty, 500)

    def test_set_single_output_channel(self) -> None:
        with virtual_board(SLAVER) as (board, ctu):
            ctu.set_output("lsd1", True, mask=0x01)
            self.assertEqual(board.output_mask, 0x05)
            ctu.set_output("24v", False, mask=board.output_mask)
            self.assertEqual(board.output_mask, 0x04)

    def test_clear_fault_latch(self) -> None:
        with virtual_board(SLAVER) as (board, ctu):
            ctu.clear_fault_latch(SLAVER)
            self.assertEqual(board.latch_clears, 1)

    def test_request_upgrade(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            ctu.request_upgrade(MASTER)
            self.assertEqual(board.upgrade_requests, 1)
            self.assertTrue(board.boot_entered)


class ErrorTests(unittest.TestCase):
    def test_timeout_when_no_reply(self) -> None:
        with virtual_board(MASTER, timeout=0.15) as (board, ctu):
            board.drop_replies = True
            with self.assertRaises(CtuTimeout):
                ctu.read_status(MASTER)

    def test_device_error_reply(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            board.force_error = 0x02  # 功能暂不支持
            with self.assertRaises(CtuDeviceError) as ctx:
                ctu.read_status(MASTER)
            self.assertEqual(ctx.exception.code, 0x02)

    def test_not_connected(self) -> None:
        ctu = CtuClient()
        with self.assertRaises(CtuNotConnected):
            ctu.read_status(MASTER)


class UpgradeTests(unittest.TestCase):
    def _write_image(self, size: int = 1000) -> str:
        path = Path("/tmp") / f"ctu_fw_{os.getpid()}_{size}.bin"
        path.write_bytes(make_firmware_image(size))
        self.addCleanup(lambda: path.unlink(missing_ok=True))
        return str(path)

    def test_inspect_rejects_bad_image(self) -> None:
        path = Path("/tmp") / f"ctu_bad_{os.getpid()}.bin"
        path.write_bytes(b"\x00" * 32)
        self.addCleanup(lambda: path.unlink(missing_ok=True))
        ok, size, reason = inspect_firmware(str(path))
        self.assertFalse(ok)
        self.assertIsNotNone(reason)

    def test_upgrade_rejects_bad_image_before_io(self) -> None:
        path = Path("/tmp") / f"ctu_bad2_{os.getpid()}.bin"
        path.write_bytes(b"\x00" * 32)
        self.addCleanup(lambda: path.unlink(missing_ok=True))
        with virtual_board(MASTER) as (board, ctu):
            with self.assertRaises(FirmwareImageError):
                ctu.upgrade(MASTER, str(path))
            self.assertEqual(board.frames_rx, 0)  # 未碰总线

    def test_upgrade_end_to_end(self) -> None:
        image_path = self._write_image(1000)
        image = Path(image_path).read_bytes()
        progress: list[float] = []
        phases: list[str] = []

        with virtual_board(MASTER) as (board, ctu):
            ctu.upgrade(MASTER, image_path,
                        progress=progress.append, phase=phases.append)
            self.assertTrue(board.boot_done)
            self.assertEqual(bytes(board.boot_data), image)
            self.assertEqual(board.boot_size, len(image))
            self.assertEqual(board.boot_errors, [])
            self.assertEqual(board.upgrade_requests, 2)  # request_boot + SELECT
        self.assertTrue(progress)
        self.assertAlmostEqual(progress[-1], 1.0)
        self.assertTrue(any("SELECT" in p or "选中" in p for p in phases) or phases)

    def test_upgrade_from_memory(self) -> None:
        image = make_firmware_image(600)
        with virtual_board(SLAVER) as (board, ctu):
            ctu.upgrade_image(SLAVER, image, name="mem.bin")
            self.assertTrue(board.boot_done)
            self.assertEqual(bytes(board.boot_data), image)

    def test_upgrade_can_be_cancelled(self) -> None:
        image_path = self._write_image(96 * 1024)  # 大固件，确保来得及中止
        cancel = threading.Event()

        with virtual_board(MASTER) as (board, ctu):
            board.block_delay = 0.004
            threading.Timer(0.15, cancel.set).start()
            started = time.monotonic()
            with self.assertRaises(UpgradeError):
                ctu.upgrade(MASTER, image_path, cancel=cancel)
            self.assertLess(time.monotonic() - started, 10.0)
            self.assertFalse(board.boot_done)


class PollerTests(unittest.TestCase):
    def test_poller_collects_stats(self) -> None:
        with virtual_board(MASTER) as (board, ctu):
            seen: list[tuple[str, object]] = []
            poller = CtuPoller(ctu, [MASTER], interval=0.02,
                               on_status=lambda d, s: seen.append(("status", s)),
                               on_voltage=lambda d, v: seen.append(("volt", v)),
                               on_temperature=lambda d, t: seen.append(("temp", t)))
            poller.start()
            time.sleep(0.4)
            poller.stop()
            poller.join(2.0)
            stats = poller.stats
            self.assertGreater(stats.tx, 0)
            self.assertGreater(stats.rx, 0)
            self.assertEqual(stats.loss, 0)
            self.assertTrue(any(kind == "status" for kind, _ in seen))
            self.assertTrue(any(kind == "temp" for kind, _ in seen))

    def test_poller_counts_losses(self) -> None:
        with virtual_board(MASTER, timeout=0.05) as (board, ctu):
            board.drop_replies = True
            poller = CtuPoller(ctu, [MASTER], interval=0.01)
            poller.start()
            time.sleep(0.4)
            poller.stop()
            poller.join(2.0)
            stats = poller.stats
            self.assertGreater(stats.tx, 0)
            self.assertGreater(stats.loss, 0)
            self.assertEqual(stats.rx, 0)
            self.assertGreater(stats.loss_rate, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
