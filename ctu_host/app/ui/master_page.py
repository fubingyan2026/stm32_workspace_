# -*- coding: utf-8 -*-
"""E1_MASTER（主控电源板 0x01）调试页。"""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSlider

from ..protocol import (
    DEV_ADDR_MASTER,
    build_mst_ctrl,
    decode_mst_status,
    decode_mst_temp,
    decode_mst_volt,
)
from .board_page import BoardPage
from .widgets import Card

MST_STATUS_ROWS = (
    ("急停状态", "estop"),
    ("12V 电源", "r12"),
    ("24V 电源", "r24"),
    ("VIN_DC-DC", "rvin"),
    ("AUX 电源", "raux"),
    ("MOTOR 电源", "rmotor"),
    ("风扇 0", "fan0"),
    ("风扇 1", "fan1"),
    ("NTC1 连接", "ntc1"),
    ("NTC2 连接", "ntc2"),
)

MST_MEASURE_ROWS = (
    ("VIN 电压", "v1"),
    ("VIN_DC-DC 电压", "v2"),
    ("NTC1 温度", "t1"),
    ("NTC2 温度", "t2"),
    ("MCU 温度", "tm"),
)


class MasterPage(BoardPage):
    addr = DEV_ADDR_MASTER
    page_title = "E1_MASTER (0x01) · 主控电源板"

    def _build(self) -> None:
        self._root.addWidget(self._query_card([
            ("读系统状态", self._query_status),
            ("读电压", self._query_volt),
            ("读温度", self._query_temp),
            ("读固件信息", self._query_info),
        ]))
        self._root.addWidget(self._control_card())
        self._root.addWidget(self._data_card())
        self._root.addWidget(self._info_card())

    def _control_card(self) -> Card:
        card = Card("控制 — 蜂鸣器")
        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(QLabel("导通占空比:"))
        self._buzzer = QSlider(Qt.Orientation.Horizontal)
        self._buzzer.setRange(0, 50)
        self._buzzer.setValue(0)
        self._buzzer_value = QLabel("0")
        self._buzzer.valueChanged.connect(
            lambda v: self._buzzer_value.setText(str(v)))
        row.addWidget(self._buzzer, 1)
        row.addWidget(self._buzzer_value)
        card.add(row)

        # 发送 / 静音 / 清锁存 / 升级请求合并为一行，减少纵向占用
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        send_btn = QPushButton("发送控制帧")
        send_btn.clicked.connect(self._send_buzzer)
        mute_btn = QPushButton("静音")
        mute_btn.setObjectName("neutral")
        mute_btn.clicked.connect(lambda: self._buzzer.setValue(0))
        for button in (send_btn, mute_btn, *self._danger_buttons()):
            buttons.addWidget(button)
        buttons.addStretch(1)
        card.add(buttons)
        return card

    def _data_card(self) -> Card:
        card = Card("实时数据")
        card.add(self._grid(MST_STATUS_ROWS, columns=3))
        card.add(self._grid(MST_MEASURE_ROWS, columns=3))
        return card

    # -------------------------------------------------------------- 控制
    def _send_buzzer(self) -> None:
        self._session.send(build_mst_ctrl(self._buzzer.value(), self.addr),
                           "蜂鸣器控制")

    # -------------------------------------------------------------- 数据渲染
    def apply_status(self, payload: bytes) -> None:
        items = decode_mst_status(payload)
        self._set_bool("estop", items["estop"], "触发", "释放")
        for key in ("r12", "r24", "rvin", "raux", "rmotor",
                    "fan0", "fan1"):
            self._set_bool(key, items[key], "异常", "正常")
        for key in ("ntc1", "ntc2"):
            self._set_bool(key, items[key], "断开", "连接正常")

    def apply_volt(self, payload: bytes) -> None:
        values = decode_mst_volt(payload)
        self._set_kv("v1", f"{values['vin_mv']} mV")
        self._set_kv("v2", f"{values['vin_dcdc_mv']} mV")

    def apply_temp(self, payload: bytes) -> None:
        values = decode_mst_temp(payload)
        self._set_kv("t1", f"{values['ntc1_c']:.2f} °C")
        self._set_kv("t2", f"{values['ntc2_c']:.2f} °C")
        self._set_kv("tm", f"{values['mcu_c']:.2f} °C")
