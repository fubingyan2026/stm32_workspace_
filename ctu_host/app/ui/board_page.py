# -*- coding: utf-8 -*-
"""板卡页基类：查询/控制卡片、固件信息卡片与帧分发逻辑。

E1_MASTER 与 E1_SLAVER 的差异仅在于数据段与输出控制方式，公共流程（查询、
信息显示、应答路由）均在此实现，子类只负责自己的数据卡片与控制卡片。
"""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..protocol import (
    CMD_CTRL,
    CMD_ERR,
    CMD_READ_INFO,
    CMD_READ_STATUS,
    CMD_READ_TEMP,
    CMD_READ_VOLT,
    CMD_REPLY_FLAG,
    CMD_RESET_LATCH,
    CMD_UPGRADE,
    DEV_NAMES,
    ERR_TEXT,
    build_read_info,
    build_read_status,
    build_read_temp,
    build_read_volt,
    build_reset_latch,
    build_upgrade,
    decode_info,
    info_flags_text,
    parse_cmd,
    parse_payload,
)
from ..session import SerialSession
from ..theme import ERROR, OK, WARN
from .widgets import Card, KvGrid

INFO_ROWS = (
    ("App 版本", "fw_app"),
    ("Boot 版本", "fw_boot"),
    ("固件大小", "fw_size"),
    ("固件校验和", "fw_sum"),
    ("上电次数", "fw_boot_cnt"),
    ("标志", "fw_flags"),
)


class BoardPage(QWidget):
    """单块电源板的调试页（查询 + 控制 + 实时数据 + 固件信息）。"""

    addr: int = 0
    page_title: str = ""
    tag_status = "读状态"
    tag_volt = "读电压"
    tag_temp = "读温度"

    log = pyqtSignal(str, str)  # (文本, 级别)

    def __init__(self, session: SerialSession,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._session = session
        self._val: dict[str, QLabel] = {}
        self._poll_cb: QCheckBox | None = None
        self._root = self._make_scroll_root()
        if self.page_title:
            heading = QLabel(self.page_title)
            heading.setObjectName("appSubtitle")
            heading.setStyleSheet("font-size: 14px; font-weight: bold;")
            self._root.addWidget(heading)
        self._build()
        self._root.addStretch(1)

    # -------------------------------------------------------------- 布局骨架
    def _make_scroll_root(self) -> QVBoxLayout:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        return layout

    def _grid(self, rows, columns: int = 2) -> KvGrid:
        grid = KvGrid(rows, columns)
        self._val.update(grid.labels)
        return grid

    def _build(self) -> None:
        raise NotImplementedError

    # -------------------------------------------------------------- 公共卡片
    def _query_card(self, entries, poll_hint: str = "参与自动轮询") -> Card:
        """查询按钮与轮询开关同排，节省纵向空间。"""
        card = Card("查询")
        row = QHBoxLayout()
        row.setSpacing(8)
        for text, slot in entries:
            button = QPushButton(text)
            button.clicked.connect(slot)
            row.addWidget(button)
        row.addStretch(1)
        self._poll_cb = QCheckBox(poll_hint)
        self._poll_cb.setChecked(True)
        row.addWidget(self._poll_cb)
        card.add(row)
        return card

    def _info_card(self) -> Card:
        card = Card("固件信息 (0x07)")
        card.add(self._grid(INFO_ROWS, columns=3))
        return card

    def _danger_buttons(self) -> list[QPushButton]:
        """清除故障锁存 + 升级请求（两板通用），由子类并入控制按钮行。"""
        clear_btn = QPushButton("清除故障锁存 (0x05)")
        clear_btn.setObjectName("danger")
        clear_btn.clicked.connect(self._query_clear_latch)
        upgrade_btn = QPushButton("升级请求 (0x06)")
        upgrade_btn.setObjectName("danger")
        upgrade_btn.clicked.connect(self._send_upgrade)
        return [clear_btn, upgrade_btn]

    # -------------------------------------------------------------- 轮询接口
    def is_poll_enabled(self) -> bool:
        return bool(self._poll_cb and self._poll_cb.isChecked())

    def poll_requests(self) -> list[bytes]:
        if not self.is_poll_enabled():
            return []
        addr = self.addr
        return [build_read_status(addr), build_read_volt(addr),
                build_read_temp(addr)]

    # -------------------------------------------------------------- 查询动作
    def _query_status(self) -> None:
        self._session.send(build_read_status(self.addr), self.tag_status)

    def _query_volt(self) -> None:
        self._session.send(build_read_volt(self.addr), self.tag_volt)

    def _query_temp(self) -> None:
        self._session.send(build_read_temp(self.addr), self.tag_temp)

    def _query_info(self) -> None:
        self._session.send(build_read_info(self.addr), "读固件信息")

    def _query_clear_latch(self) -> None:
        self._session.send(build_reset_latch(self.addr), "清保护锁存")

    def _send_upgrade(self) -> None:
        dev = DEV_NAMES.get(self.addr, f"0x{self.addr:02X}")
        answer = QMessageBox.warning(
            self, "确认升级",
            f"向 {dev} 发送升级请求 (0x06)？\n\n"
            "板端将写入升级标志并复位进入 Bootloader；"
            "随后请在『固件升级』页选择固件并开始升级。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            self._session.send(build_upgrade(self.addr), "升级请求")

    # -------------------------------------------------------------- 应答处理
    def _set_kv(self, key: str, text: str, color: str | None = None) -> None:
        label = self._val.get(key)
        if label is None:
            return
        if color:
            label.setText(f"<span style='color:{color};'>{text}</span>")
        else:
            label.setText(text)

    def _set_bool(self, key: str, active: bool,
                  active_text: str, inactive_text: str) -> None:
        """active=True 用红色高亮（异常/有效），否则绿色正常。"""
        self._set_kv(key, active_text if active else inactive_text,
                     ERROR if active else OK)

    def handle_frame(self, frame: bytes) -> None:
        cmd = parse_cmd(frame)
        payload = parse_payload(frame)
        dev = DEV_NAMES.get(self.addr, f"addr=0x{self.addr:02X}")

        if cmd == CMD_ERR:
            content = payload[1:] if payload else b""
            code = content[0] if content else 0xFF
            self.log.emit(f"{dev} 错误应答 err=0x{code:02X} "
                          f"({ERR_TEXT.get(code, '未知')})", "warn")
            return

        content = payload[1:] if payload else b""
        base = cmd & ~CMD_REPLY_FLAG
        if base == CMD_READ_STATUS:
            self.apply_status(content)
        elif base == CMD_READ_VOLT:
            self.apply_volt(content)
        elif base == CMD_READ_TEMP:
            self.apply_temp(content)
        elif base == CMD_READ_INFO:
            self.apply_info(content)
            info = decode_info(content)
            self.log.emit(
                f"{dev} 固件: App=v{info['app_version']} "
                f"Boot=v{info['meta_version']} size={info['fw_size']}B "
                f"sum=0x{info['fw_checksum']:08X} boot={info['reboot_counts']} "
                f"[{info_flags_text(info['flags'])}]", "info")
        elif base == CMD_CTRL:
            self.log.emit(f"{dev} 控制 ACK 已收到", "info")
        elif base == CMD_RESET_LATCH:
            self.log.emit(f"{dev} 清除锁存 ACK 已收到", "info")
        elif base == CMD_UPGRADE:
            self.log.emit(f"{dev} 升级请求 ACK 已收到，板将复位进入 Bootloader",
                          "warn")
        else:
            self.log.emit(f"{dev} 收到未知命令 cmd=0x{cmd:02X}", "warn")

    def apply_status(self, payload: bytes) -> None:
        raise NotImplementedError

    def apply_volt(self, payload: bytes) -> None:
        raise NotImplementedError

    def apply_temp(self, payload: bytes) -> None:
        raise NotImplementedError

    def apply_info(self, payload: bytes) -> None:
        info = decode_info(payload)
        self._set_kv("fw_app", f"v{info['app_version']}")
        self._set_kv("fw_boot", f"v{info['meta_version']}")
        self._set_kv("fw_size", f"{info['fw_size']} B")
        self._set_kv("fw_sum", f"0x{info['fw_checksum']:08X}")
        self._set_kv("fw_boot_cnt", f"{info['reboot_counts']}")
        flags = info["flags"]
        self._set_kv("fw_flags", info_flags_text(flags),
                     WARN if (flags & 0x02) else OK)
