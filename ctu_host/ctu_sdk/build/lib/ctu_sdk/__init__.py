# -*- coding: utf-8 -*-
"""E1 CTU 电源板 RS485 上位机 SDK（Linux / 跨平台，纯 Python，无 GUI 依赖）。

覆盖上位机的全部功能：

- 查询：系统状态 ``0x01``、电压 ``0x02``、温度 ``0x03``、固件信息 ``0x07``
- 控制：主控蜂鸣器 ``0x04``、副板输出掩码 + 补光亮度 ``0x04``
- 维护：清除故障锁存 ``0x05``、升级请求 ``0x06``
- 升级：0x06 邀请 + SELECT/START/DATA/END 分块传输（复用 Boot 工程协议层）
- 轮询：:class:`CtuPoller` 周期查询 + 超时丢包统计

最小示例::

    from ctu_sdk import CtuClient, MASTER, SLAVER

    with CtuClient("/dev/ttyUSB0") as ctu:
        if ctu.scan()[MASTER]:
            print(ctu.read_status(MASTER).faults)
        ctu.upgrade(SLAVER, "E1_SLAVER_POWER_CTU.bin",
                    progress=lambda f: print(f"{f * 100:.0f}%"))
"""

from __future__ import annotations

from .client import DEFAULT_TIMEOUT, CtuClient, list_serial_ports
from .devices import (
    ALL_DEVICES,
    DEVICE_ADDR,
    DEVICE_BOOT_ID,
    MASTER,
    SLAVER,
    boot_device_id,
    device_addr,
    normalize_device,
)
from .errors import (
    CtuDeviceError,
    CtuError,
    CtuFramingError,
    CtuNotConnected,
    CtuTimeout,
    CtuUnexpectedReply,
    FirmwareImageError,
    UpgradeError,
)
from .firmware import (
    MAX_FW_SIZE,
    boot_host_dir,
    inspect_firmware,
)
from .models import (
    Ack,
    FirmwareInfo,
    MasterStatus,
    MasterTemperature,
    MasterVoltage,
    PollStats,
    SlaverStatus,
    SlaverTemperature,
    SlaverVoltage,
)
from .poller import CtuPoller
from .transport import SerialTransport

__version__ = "1.0.0"

__all__ = [
    "__version__",
    # 客户端
    "CtuClient", "CtuPoller", "SerialTransport", "list_serial_ports",
    "DEFAULT_TIMEOUT",
    # 设备
    "MASTER", "SLAVER", "ALL_DEVICES", "DEVICE_ADDR", "DEVICE_BOOT_ID",
    "normalize_device", "device_addr", "boot_device_id",
    # 数据模型
    "MasterStatus", "MasterVoltage", "MasterTemperature",
    "SlaverStatus", "SlaverVoltage", "SlaverTemperature",
    "FirmwareInfo", "Ack", "PollStats",
    # 异常
    "CtuError", "CtuNotConnected", "CtuTimeout", "CtuFramingError",
    "CtuDeviceError", "CtuUnexpectedReply", "FirmwareImageError",
    "UpgradeError",
    # 固件
    "inspect_firmware", "MAX_FW_SIZE", "boot_host_dir",
]
