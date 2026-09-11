# -*- coding: utf-8 -*-
"""界面主题：马卡龙浅色调色板与全局 QSS。

颜色与兄弟工程上位机保持一致（米色底 + 绿色强调），并通过扁平化选择器
统一到卡片/导航/按钮/输入框等控件。所有尺寸使用设备无关像素，交由 Qt 的
High-DPI 缩放自动处理。
"""

from __future__ import annotations

# ------------------------------------------------------------------ 调色板
BG_MAIN = "#F5F0E8"      # 窗口主背景
BG_CARD = "#FFFFFF"      # 卡片底色
BG_INPUT = "#FDFCFA"     # 输入框底色
BG_HEADER = "#EDE6DA"    # 顶栏 / 表头
BG_SIDEBAR = "#F8F4EC"   # 侧栏
BG_HOVER = "#D2F0E3"     # 悬停（薄荷淡绿）
BG_SELECT = "#BAE6D3"    # 选中
BG_STATUS = "#D8F0E2"    # 状态栏成功底色

ACCENT = "#7EC8A0"       # 强调 / 连接成功
ACCENT_HOVER = "#6DB88F"
ACCENT_TEXT = "#4C9B73"  # 强调文字
CORAL = "#F2A999"        # 珊瑚（危险/断开）
CORAL_HOVER = "#ED9481"
CORAL_TEXT = "#D4745C"

TEXT = "#3A3A3A"
TEXT_DIM = "#9E9E9E"
BORDER = "#E4DED4"

OK = "#4C9B73"
WARN = "#D29A2E"
ERROR = "#E0563F"
INFO = "#3E7BB6"
TX = "#7A8A9E"

# 日志级别 → 颜色
LEVEL_COLOR = {"info": None, "warn": WARN, "error": ERROR, "tx": TX, "rx": INFO}
LEVEL_PREFIX = {"info": "", "warn": "[WARN] ", "error": "[ERR]  ", "tx": "", "rx": ""}


def status_color(state: str) -> str:
    return {"ok": OK, "warn": WARN, "error": ERROR, "idle": TEXT_DIM}.get(state, TEXT)


QSS = f"""
QMainWindow {{ background-color: {BG_MAIN}; }}
QMainWindow, QWidget {{ color: {TEXT}; font-size: 13px; }}
QMainWindow > QWidget {{ background-color: {BG_MAIN}; }}
QScrollArea, QScrollArea > QWidget, QScrollArea > QWidget > QWidget {{
    background: transparent;
    border: none;
}}

/* ---------- 顶栏 ---------- */
QFrame#header {{
    background-color: {BG_HEADER};
    border-bottom: 1px solid {BORDER};
}}
QLabel#appTitle {{ font-size: 19px; font-weight: bold; color: {ACCENT_TEXT}; }}
QLabel#appSubtitle {{ color: {TEXT_DIM}; }}

/* ---------- 侧栏导航 ---------- */
QListWidget#nav {{
    background-color: {BG_SIDEBAR};
    border: none;
    border-right: 1px solid {BORDER};
    outline: 0;
    padding: 8px 6px;
}}
QListWidget#nav::item {{
    height: 33px;
    padding: 0 12px;
    border-radius: 7px;
    color: {TEXT};
}}
QListWidget#nav::item:hover {{ background-color: {BG_HOVER}; }}
QListWidget#nav::item:selected {{
    background-color: {ACCENT};
    color: #FFFFFF;
    font-weight: bold;
}}

/* ---------- 卡片 ---------- */
QFrame#card {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#cardTitle {{
    color: {ACCENT_TEXT};
    font-size: 13px;
    font-weight: bold;
    padding-bottom: 0px;
}}

/* ---------- 按钮 ---------- */
QPushButton {{
    background-color: {ACCENT};
    color: #FFFFFF;
    border: none;
    border-radius: 5px;
    padding: 5px 12px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
QPushButton:pressed {{ background-color: {ACCENT_TEXT}; }}
QPushButton:disabled {{ background-color: #D9D4C8; color: {TEXT_DIM}; }}
QPushButton#danger {{ background-color: {CORAL}; }}
QPushButton#danger:hover {{ background-color: {CORAL_HOVER}; }}
QPushButton#neutral {{ background-color: {BORDER}; color: {TEXT}; }}
QPushButton#neutral:hover {{ background-color: #D9D2C6; }}
QPushButton#ghost {{
    background-color: transparent;
    color: {ACCENT_TEXT};
    border: 1px solid {ACCENT};
}}
QPushButton#ghost:hover {{ background-color: {BG_HOVER}; }}

/* ---------- 输入控件 ---------- */
QComboBox, QSpinBox, QLineEdit {{
    background-color: {BG_INPUT};
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 3px 7px;
    min-height: 18px;
}}
QComboBox:focus, QSpinBox:focus, QLineEdit:focus {{ border-color: {ACCENT}; }}
QComboBox:disabled, QSpinBox:disabled, QLineEdit:disabled {{
    background-color: {BG_HEADER}; color: {TEXT_DIM};
}}
QComboBox::drop-down {{ border: none; width: 20px; }}

QCheckBox {{ spacing: 6px; }}
QCheckBox::indicator {{
    width: 16px; height: 16px;
    border: 1px solid {BORDER};
    border-radius: 4px;
    background-color: {BG_INPUT};
}}
QCheckBox::indicator:checked {{ background-color: {ACCENT}; border-color: {ACCENT}; }}
QCheckBox::indicator:disabled {{ background-color: {BG_HEADER}; }}

/* ---------- 文本 / 日志 ---------- */
QPlainTextEdit {{
    background-color: {BG_INPUT};
    border: 1px solid {BORDER};
    border-radius: 8px;
    font-family: Consolas, "Cascadia Mono", monospace;
    font-size: 12px;
    padding: 4px;
}}

/* ---------- 进度 / 滑块 ---------- */
QProgressBar {{
    border: 1px solid {BORDER};
    border-radius: 7px;
    background: {BG_INPUT};
    text-align: center;
    min-height: 16px;
}}
QProgressBar::chunk {{ background-color: {ACCENT}; border-radius: 6px; }}
QSlider::groove:horizontal {{
    height: 6px; background: {BORDER}; border-radius: 3px;
}}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 3px; }}
QSlider::handle:horizontal {{
    background: {ACCENT_TEXT}; width: 14px; margin: -5px 0; border-radius: 7px;
}}

/* ---------- 状态胶囊 ---------- */
QLabel#chip {{
    padding: 3px 10px;
    border-radius: 9px;
    background-color: {BG_HEADER};
    color: {TEXT};
}}
QLabel#chip[state="ok"] {{ background-color: {BG_STATUS}; color: {OK}; }}
QLabel#chip[state="warn"] {{ background-color: #F6EBD2; color: {WARN}; }}
QLabel#chip[state="error"] {{ background-color: #FBE0DC; color: {ERROR}; }}
QLabel#chip[state="idle"] {{ background-color: {BG_HEADER}; color: {TEXT_DIM}; }}

/* ---------- 状态栏 ---------- */
QStatusBar {{ background-color: {BG_HEADER}; color: {TEXT}; }}
QStatusBar::item {{ border: none; }}
QLabel#statValue {{ font-weight: bold; }}

/* ---------- 滚动条 ---------- */
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{
    background: {BORDER}; border-radius: 5px; min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{ background: {TEXT_DIM}; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle:horizontal {{
    background: {BORDER}; border-radius: 5px; min-width: 24px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
"""


def apply_theme(app) -> None:
    """把全局 QSS 应用到 QApplication。"""
    app.setStyleSheet(QSS)
