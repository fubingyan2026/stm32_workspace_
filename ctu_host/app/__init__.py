# -*- coding: utf-8 -*-
"""E1 CTU 电源板 RS485 双板调试上位机 — 应用包。

分层（严格单向依赖 UI → session → transport → protocol）：
    protocol   纯协议编解码（无 Qt、无串口依赖）
    transport  Qt 工作线程（串口收发 / 固件升级）
    session   连接与轮询调度、丢包统计（无界面依赖）
    theme     调色板与 QSS
    ui         窗口、页面与可复用控件
"""

__version__ = "2.0.0"
__app_name__ = "E1 CTU Host"
