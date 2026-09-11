# -*- coding: utf-8 -*-
"""E1_SLAVER（副电源模块 0x02）调试页。"""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
)

from ..protocol import (
    DEV_ADDR_SLAVER,
    build_slv_ctrl,
    decode_slv_status,
    decode_slv_temp,
    decode_slv_volt,
)
from ..theme import OK, TEXT_DIM
from .board_page import BoardPage
from .widgets import Card

OUTPUT_KEYS = (("24v", "24V"), ("12v", "12V_ISO"),
               ("lsd1", "LSD1"), ("lsd2", "LSD2"))

SLV_OUTPUT_ROWS = (
    ("24V 输出", "out_24v"),
    ("12V_ISO 输出", "out_12v"),
    ("LSD1 输出", "out_lsd1"),
    ("LSD2 输出", "out_lsd2"),
)

SLV_FAULT_ROWS = (
    ("24V 故障", "err_24v"),
    ("12V_ISO 故障", "err_12v"),
    ("LSD1 故障", "err_lsd1"),
    ("LSD2 故障", "err_lsd2"),
    ("AUX 输入", "err_aux"),
    ("MOTOR 输入", "err_motor"),
    ("故障锁存", "latch_active"),
)

SLV_MEASURE_ROWS = (
    ("AUX 电压", "aux_mv"),
    ("MOTOR 电压", "motor_mv"),
    ("LSD1 电压", "lsd1_mv"),
    ("LSD2 电压", "lsd2_mv"),
    ("MCU 温度", "mcu_temp"),
    ("VDDA", "vdda_mv"),
)


class SlaverPage(BoardPage):
    addr = DEV_ADDR_SLAVER
    page_title = "E1_SLAVER (0x02) · 副电源模块"
    tag_temp = "读温度/VDDA"

    def _build(self) -> None:
        self._root.addWidget(self._query_card([
            ("读系统状态", self._query_status),
            ("读电压", self._query_volt),
            ("读温度/VDDA", self._query_temp),
            ("读固件信息", self._query_info),
        ]))
        self._root.addWidget(self._control_card())
        self._root.addWidget(self._data_card())
        self._root.addWidget(self._info_card())

    def _control_card(self) -> Card:
        card = Card("输出控制")
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(QLabel("输出:"))
        self._out_checks: dict[str, QCheckBox] = {}
        for key, text in OUTPUT_KEYS:
            check = QCheckBox(text)
            row.addWidget(check)
            self._out_checks[key] = check
        row.addStretch(1)
        card.add(row)

        duty_row = QHBoxLayout()
        duty_row.setSpacing(8)
        duty_row.addWidget(QLabel("补光亮度:"))
        self._duty = QSlider(Qt.Orientation.Horizontal)
        self._duty.setRange(0, 1000)
        self._duty.setValue(0)
        self._duty_value = QLabel("0 (0.0%)")
        self._duty.valueChanged.connect(
            lambda v: self._duty_value.setText(f"{v} ({v / 10:.1f}%)"))
        duty_row.addWidget(self._duty, 1)
        duty_row.addWidget(self._duty_value)
        card.add(duty_row)

        # 发送 / 全开 / 全关 / 清锁存 / 升级请求合并为一行
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        send_btn = QPushButton("发送输出控制")
        send_btn.clicked.connect(self._send_ctrl)
        all_on_btn = QPushButton("全开")
        all_on_btn.setObjectName("neutral")
        all_on_btn.clicked.connect(lambda: self._set_all(True))
        all_off_btn = QPushButton("全关")
        all_off_btn.setObjectName("neutral")
        all_off_btn.clicked.connect(lambda: self._set_all(False))
        for button in (send_btn, all_on_btn, all_off_btn,
                       *self._danger_buttons()):
            buttons.addWidget(button)
        buttons.addStretch(1)
        card.add(buttons)
        return card

    def _data_card(self) -> Card:
        card = Card("实时数据")
        card.add(self._grid(SLV_OUTPUT_ROWS, columns=3))
        card.add(self._grid(SLV_FAULT_ROWS, columns=3))
        card.add(self._grid(SLV_MEASURE_ROWS, columns=3))
        return card

    # -------------------------------------------------------------- 控制
    def _out_mask(self) -> int:
        mask = 0
        for bit, (key, _text) in enumerate(OUTPUT_KEYS):
            if self._out_checks[key].isChecked():
                mask |= 1 << bit
        return mask

    def _send_ctrl(self) -> None:
        self._session.send(
            build_slv_ctrl(self._out_mask(), self._duty.value(), self.addr),
            "输出控制")

    def _set_all(self, enabled: bool) -> None:
        for check in self._out_checks.values():
            check.setChecked(enabled)
        if not enabled:
            self._duty.setValue(0)

    # -------------------------------------------------------------- 数据渲染
    def apply_status(self, payload: bytes) -> None:
        items = decode_slv_status(payload)
        for key in ("out_24v", "out_12v", "out_lsd1", "out_lsd2"):
            on = items[key]
            self._set_kv(key, "开" if on else "关", OK if on else TEXT_DIM)
        for key in ("err_24v", "err_12v", "err_lsd1", "err_lsd2",
                    "err_aux", "err_motor"):
            self._set_bool(key, items[key], "异常", "正常")
        self._set_bool("latch_active", items["latch_active"], "锁存", "无")

    def apply_volt(self, payload: bytes) -> None:
        values = decode_slv_volt(payload)
        for key in ("aux_mv", "motor_mv", "lsd1_mv", "lsd2_mv"):
            self._set_kv(key, f"{values[key]} mV")

    def apply_temp(self, payload: bytes) -> None:
        values = decode_slv_temp(payload)
        self._set_kv("mcu_temp", f"{values['mcu_c']:.2f} °C")
        self._set_kv("vdda_mv", f"{values['vdda_mv']} mV")
