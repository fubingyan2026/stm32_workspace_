# -*- coding: utf-8 -*-
"""周期轮询：把 SDK 当作无 GUI 的长期监测服务使用。

对应上位机的「自动轮询」：按间隔依次查询各设备的 状态 / 电压 / 温度，
统计发送数、应答数与超时丢包率，并通过回调把结果推给使用方。
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable

from .client import CtuClient
from .devices import ALL_DEVICES
from .errors import CtuTimeout
from .models import PollStats
from .protocol import CMD_READ_STATUS, CMD_READ_TEMP, CMD_READ_VOLT

StatusCallback = Callable[[str, object], None]
VoltageCallback = Callable[[str, object], None]
TemperatureCallback = Callable[[str, object], None]
ErrorCallback = Callable[[str, int, Exception], None]
StatsCallback = Callable[[PollStats], None]

_FPS_WINDOW_S = 1.0
_STATS_INTERVAL_S = 0.2


class CtuPoller:
    """后台轮询线程。

    @param client     已连接的 :class:`~ctu_sdk.client.CtuClient`
    @param devices    参与轮询的设备（默认 master + slaver）
    @param interval   相邻两次查询之间的间隔（秒）
    """

    def __init__(self, client: CtuClient, devices=ALL_DEVICES,
                 interval: float = 0.1, *,
                 on_status: StatusCallback | None = None,
                 on_voltage: VoltageCallback | None = None,
                 on_temperature: TemperatureCallback | None = None,
                 on_error: ErrorCallback | None = None,
                 on_stats: StatsCallback | None = None) -> None:
        self._client = client
        self._devices = list(devices)
        self._interval = max(0.0, float(interval))
        self._on_status = on_status
        self._on_voltage = on_voltage
        self._on_temperature = on_temperature
        self._on_error = on_error
        self._on_stats = on_stats

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._tx = 0
        self._rx = 0
        self._loss = 0
        self._rx_window: deque[float] = deque()
        self._last_stats_emit = 0.0

    # ------------------------------------------------------------------ 控制
    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="ctu-poller",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def __enter__(self) -> "CtuPoller":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()
        self.join(2.0)

    # ------------------------------------------------------------------ 统计
    @property
    def stats(self) -> PollStats:
        with self._lock:
            self._trim_window()
            fps = float(len(self._rx_window))
            tx, rx, loss = self._tx, self._rx, self._loss
        return PollStats(tx=tx, rx=rx, loss=loss,
                         loss_rate=(loss / tx * 100.0) if tx else 0.0,
                         fps=fps)

    def reset_stats(self) -> None:
        with self._lock:
            self._tx = self._rx = self._loss = 0
            self._rx_window.clear()

    def _trim_window(self) -> None:
        now = time.monotonic()
        while self._rx_window and (now - self._rx_window[0] > _FPS_WINDOW_S):
            self._rx_window.popleft()

    # ------------------------------------------------------------------ 轮询
    def poll_once(self) -> None:
        """执行一轮查询（主线程手动调用或由后台线程循环调用）。"""
        for device in self._devices:
            self._query(device, CMD_READ_STATUS, "status")
            if self._stop.is_set():
                return
            self._query(device, CMD_READ_VOLT, "voltage")
            if self._stop.is_set():
                return
            self._query(device, CMD_READ_TEMP, "temperature")

    def _query(self, device: str, command: int, kind: str) -> None:
        with self._lock:
            self._tx += 1
        try:
            if kind == "status":
                result = self._client.read_status(device)
                if self._on_status:
                    self._on_status(device, result)
            elif kind == "voltage":
                result = self._client.read_voltage(device)
                if self._on_voltage:
                    self._on_voltage(device, result)
            else:
                result = self._client.read_temperature(device)
                if self._on_temperature:
                    self._on_temperature(device, result)
        except CtuTimeout as exc:
            with self._lock:
                self._loss += 1
            if self._on_error:
                self._on_error(device, command, exc)
        except Exception as exc:  # noqa: BLE001 - 交给回调，轮询不中断
            with self._lock:
                self._loss += 1
            if self._on_error:
                self._on_error(device, command, exc)
        else:
            with self._lock:
                self._rx += 1
                self._rx_window.append(time.monotonic())

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            self.poll_once()
            self._emit_stats_maybe()
            if self._stop.is_set():
                break
            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, self._interval - elapsed))
        self._emit_stats_maybe(force=True)

    def _emit_stats_maybe(self, force: bool = False) -> None:
        if self._on_stats is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_stats_emit) < _STATS_INTERVAL_S:
            return
        self._last_stats_emit = now
        self._on_stats(self.stats)
