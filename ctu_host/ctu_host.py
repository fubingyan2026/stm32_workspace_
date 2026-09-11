# -*- coding: utf-8 -*-
"""E1 CTU 电源板 RS485 双板调试上位机 — 启动入口。

运行: python ctu_host.py

应用代码位于 :mod:`app` 包，按 protocol / transport / session / ui 分层。
"""

from __future__ import annotations

import sys

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication

from app import __app_name__, __version__
from app.theme import apply_theme
from app.ui import MainWindow


def main() -> int:
    # 高分屏：不做整数倍取整，允许 125% / 150% 等非整数缩放下正确布局
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QApplication(sys.argv)
    app.setApplicationName(__app_name__)
    app.setApplicationVersion(__version__)
    app.setOrganizationName("E1")
    apply_theme(app)

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
