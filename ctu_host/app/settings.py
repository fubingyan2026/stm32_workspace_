# -*- coding: utf-8 -*-
"""持久化设置：按目标设备分别保存最近使用的固件路径。

使用 Qt 的 :class:`QSettings`（Windows 下写入注册表 HKCU），无需关心可执行文件
所在目录的写权限，打包成 exe 后同样有效。
"""

from __future__ import annotations

from PyQt6.QtCore import QSettings

#: 单个设备的固件路径历史最大条数
MAX_FIRMWARE_HISTORY = 10


class FirmwareHistory:
    """某个设备最近使用的固件路径（最近优先、去重、容量上限）。

    master / slaver **各自使用独立的存储键**（``firmware/history/<device>``），
    切换目标设备时互不干扰。
    """

    def __init__(self, device: str, max_items: int = MAX_FIRMWARE_HISTORY,
                 settings: QSettings | None = None) -> None:
        self._max = max_items
        self._settings = settings or QSettings()
        self._key = f"firmware/history/{device}"

    @property
    def key(self) -> str:
        return self._key

    def items(self) -> list[str]:
        raw = self._settings.value(self._key, [])
        if isinstance(raw, str):
            raw = [raw] if raw else []
        if not isinstance(raw, list):
            return []
        return [path for path in raw if isinstance(path, str) and path]

    def add(self, path: str) -> list[str]:
        """把 ``path`` 置于最前，返回更新后的列表。"""
        history = [item for item in self.items() if item != path]
        history.insert(0, path)
        history = history[:self._max]
        self._settings.setValue(self._key, history)
        self._settings.sync()
        return history

    def clear(self) -> None:
        self._settings.remove(self._key)
        self._settings.sync()
