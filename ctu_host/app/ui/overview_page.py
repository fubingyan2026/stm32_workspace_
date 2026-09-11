# -*- coding: utf-8 -*-
"""概览页：两板关键健康指标一屏速览 + 在线状态。"""

from __future__ import annotations

import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ..protocol import (
    CMD_READ_STATUS,
    CMD_READ_TEMP,
    CMD_READ_VOLT,
    CMD_REPLY_FLAG,
    DEV_ADDR_MASTER,
    DEV_ADDR_SLAVER,
    decode_mst_status,
    decode_mst_temp,
    decode_mst_volt,
    decode_slv_status,
    decode_slv_temp,
    decode_slv_volt,
    parse_addr,
    parse_cmd,
    parse_payload,
)
from ..session import SerialSession
from ..theme import ERROR, OK, TEXT_DIM
from .widgets import Card, KvGrid, StatusChip

MASTER_ROWS = (
    ("急停状态", "m_estop"),
    ("12V 电源", "m_r12"),
    ("24V 电源", "m_r24"),
    ("VIN 电压", "m_vin"),
    ("VIN_DC-DC 电压", "m_vdcdc"),
    ("MCU 温度", "m_mcu"),
)

SLAVER_ROWS = (
    ("故障锁存", "s_latch"),
    ("24V 输出", "s_out24"),
    ("12V_ISO 输出", "s_out12"),
    ("AUX 电压", "s_aux"),
    ("MOTOR 电压", "s_motor"),
    ("MCU 温度", "s_mcu"),
)

ONLINE_WINDOW_S = 2.0


class OverviewPage(QWidget):
    """把两板最关心的状态压缩到一屏，便于快速判断系统健康。"""

    def __init__(self, session: SerialSession,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._session = session
        self._grids: dict[int, KvGrid] = {}
        self._chips: dict[int, StatusChip] = {}
        self._updated: dict[int, QLabel] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        heading = QLabel("系统概览 · MASTER + SLAVER 同总线")
        heading.setStyleSheet("font-size: 14px; font-weight: bold;")
        root.addWidget(heading)

        root.addWidget(self._board_card(
            DEV_ADDR_MASTER, "E1_MASTER (0x01) · 主控电源板", MASTER_ROWS))
        root.addWidget(self._board_card(
            DEV_ADDR_SLAVER, "E1_SLAVER (0x02) · 副电源模块", SLAVER_ROWS))
        root.addStretch(1)

        session.frame_received.connect(self._on_frame)
        self._online_timer = QTimer(self)
        self._online_timer.timeout.connect(self._refresh_online)
        self._online_timer.start(500)

    def _board_card(self, addr: int, title: str, rows) -> Card:
        grid = KvGrid(rows, columns=3)
        chip = StatusChip("等待数据", "idle")
        updated = QLabel("尚未收到数据")
        updated.setStyleSheet(f"color: {TEXT_DIM};")

        header = QHBoxLayout()
        header.addWidget(chip)
        header.addStretch(1)
        header.addWidget(updated)

        card = Card(title)
        card.add(header)
        card.add(grid)

        self._grids[addr] = grid
        self._chips[addr] = chip
        self._updated[addr] = updated
        return card

    # -------------------------------------------------------------- 数据更新
    def _on_frame(self, frame: bytes, _ts: float) -> None:
        addr = parse_addr(frame)
        base = parse_cmd(frame) & ~CMD_REPLY_FLAG
        payload = parse_payload(frame)
        content = payload[1:] if payload else b""

        if addr == DEV_ADDR_MASTER:
            self._apply_master(base, content)
        elif addr == DEV_ADDR_SLAVER:
            self._apply_slaver(base, content)
        else:
            return

        self._chips[addr].set_state("在线", "ok")
        self._updated[addr].setText(f"最后更新 {time.strftime('%H:%M:%S')}")

    def _apply_master(self, base: int, content: bytes) -> None:
        grid = self._grids[DEV_ADDR_MASTER]
        if base == CMD_READ_STATUS:
            items = decode_mst_status(content)
            grid.set_value("m_estop", "触发" if items["estop"] else "释放",
                           ERROR if items["estop"] else OK)
            grid.set_value("m_r12", "异常" if items["r12"] else "正常",
                           ERROR if items["r12"] else OK)
            grid.set_value("m_r24", "异常" if items["r24"] else "正常",
                           ERROR if items["r24"] else OK)
        elif base == CMD_READ_VOLT:
            values = decode_mst_volt(content)
            grid.set_value("m_vin", f"{values['vin_mv']} mV")
            grid.set_value("m_vdcdc", f"{values['vin_dcdc_mv']} mV")
        elif base == CMD_READ_TEMP:
            grid.set_value("m_mcu", f"{decode_mst_temp(content)['mcu_c']:.2f} °C")

    def _apply_slaver(self, base: int, content: bytes) -> None:
        grid = self._grids[DEV_ADDR_SLAVER]
        if base == CMD_READ_STATUS:
            items = decode_slv_status(content)
            grid.set_value("s_latch",
                           "锁存" if items["latch_active"] else "无",
                           ERROR if items["latch_active"] else OK)
            grid.set_value("s_out24", "开" if items["out_24v"] else "关",
                           OK if items["out_24v"] else TEXT_DIM)
            grid.set_value("s_out12", "开" if items["out_12v"] else "关",
                           OK if items["out_12v"] else TEXT_DIM)
        elif base == CMD_READ_VOLT:
            values = decode_slv_volt(content)
            grid.set_value("s_aux", f"{values['aux_mv']} mV")
            grid.set_value("s_motor", f"{values['motor_mv']} mV")
        elif base == CMD_READ_TEMP:
            grid.set_value("s_mcu", f"{decode_slv_temp(content)['mcu_c']:.2f} °C")

    def _refresh_online(self) -> None:
        for addr, chip in self._chips.items():
            age = self._session.seconds_since_rx(addr)
            if age is None:
                chip.set_state("等待数据", "idle")
            elif age <= ONLINE_WINDOW_S:
                chip.set_state("在线", "ok")
            else:
                chip.set_state("离线", "warn")
