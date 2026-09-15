# -*- coding: utf-8 -*-
"""SDK 返回的数据模型（不可变 dataclass），与各命令的数据段一一对应。

所有 ``*_from_payload`` 工厂接收的是**去掉 payload 首字节（设备 ID）之后**的内容段，
与 :mod:`ctu_sdk.protocol` 中各 ``decode_*`` 函数的入参一致。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

from .protocol import (
    INFO_FLAG_META_VALID,
    INFO_FLAG_UPGRADE_DONE,
    INFO_FLAG_UPGRADE_REQ,
    decode_info,
    decode_mst_status,
    decode_mst_temp,
    decode_mst_volt,
    decode_slv_status,
    decode_slv_temp,
    decode_slv_volt,
    info_flags_text,
)


class _DictMixin:
    """提供 ``to_dict()``，方便 JSON 序列化 / 日志输出。"""

    def to_dict(self) -> dict:
        return asdict(self)  # type: ignore[arg-type]


# =============================================================================
# E1_MASTER
# =============================================================================

@dataclass(frozen=True)
class MasterStatus(_DictMixin):
    """0x01 主控板系统状态。

    注意位域语义：``estop`` / ``*_fault`` / ``ntc*_disconnected`` 为 ``True``
    表示**异常或有效**，而不是“正常”。
    """

    estop: bool = False
    rail_12v_fault: bool = False
    rail_24v_fault: bool = False
    vin_dcdc_fault: bool = False
    aux_fault: bool = False
    motor_fault: bool = False
    fan0_fault: bool = False
    fan1_fault: bool = False
    ntc1_disconnected: bool = False
    ntc2_disconnected: bool = False
    raw: dict[str, bool] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: bytes) -> "MasterStatus":
        raw = decode_mst_status(payload)
        return cls(
            estop=raw["estop"],
            rail_12v_fault=raw["r12"],
            rail_24v_fault=raw["r24"],
            vin_dcdc_fault=raw["rvin"],
            aux_fault=raw["raux"],
            motor_fault=raw["rmotor"],
            fan0_fault=raw["fan0"],
            fan1_fault=raw["fan1"],
            ntc1_disconnected=raw["ntc1"],
            ntc2_disconnected=raw["ntc2"],
            raw=raw,
        )

    @property
    def faults(self) -> list[str]:
        """当前置位的异常项名称列表。"""
        names = [
            ("急停", self.estop),
            ("12V", self.rail_12v_fault),
            ("24V", self.rail_24v_fault),
            ("VIN_DC-DC", self.vin_dcdc_fault),
            ("AUX", self.aux_fault),
            ("MOTOR", self.motor_fault),
            ("风扇0", self.fan0_fault),
            ("风扇1", self.fan1_fault),
            ("NTC1断开", self.ntc1_disconnected),
            ("NTC2断开", self.ntc2_disconnected),
        ]
        return [name for name, active in names if active]

    @property
    def healthy(self) -> bool:
        return not self.faults


@dataclass(frozen=True)
class MasterVoltage(_DictMixin):
    """0x02 主控板电压（mV）。"""

    vin_mv: int = 0
    vin_dcdc_mv: int = 0

    @classmethod
    def from_payload(cls, payload: bytes) -> "MasterVoltage":
        values = decode_mst_volt(payload)
        return cls(vin_mv=values["vin_mv"], vin_dcdc_mv=values["vin_dcdc_mv"])


@dataclass(frozen=True)
class MasterTemperature(_DictMixin):
    """0x03 主控板温度（°C）。"""

    ntc1_c: float = 0.0
    ntc2_c: float = 0.0
    mcu_c: float = 0.0

    @classmethod
    def from_payload(cls, payload: bytes) -> "MasterTemperature":
        values = decode_mst_temp(payload)
        return cls(ntc1_c=values["ntc1_c"], ntc2_c=values["ntc2_c"],
                   mcu_c=values["mcu_c"])


# =============================================================================
# E1_SLAVER
# =============================================================================

@dataclass(frozen=True)
class SlaverStatus(_DictMixin):
    """0x01 副电源模块状态。

    ``out_*`` 为 ``True`` 表示**输出已开启**；``fault_*`` / ``latch_active``
    为 ``True`` 表示**故障或已锁存**。
    """

    out_24v: bool = False
    out_12v: bool = False
    out_lsd1: bool = False
    out_lsd2: bool = False
    fault_24v: bool = False
    fault_12v: bool = False
    fault_lsd1: bool = False
    fault_lsd2: bool = False
    fault_aux: bool = False
    fault_motor: bool = False
    latch_active: bool = False
    raw: dict[str, bool] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: bytes) -> "SlaverStatus":
        raw = decode_slv_status(payload)
        return cls(
            out_24v=raw["out_24v"],
            out_12v=raw["out_12v"],
            out_lsd1=raw["out_lsd1"],
            out_lsd2=raw["out_lsd2"],
            fault_24v=raw["err_24v"],
            fault_12v=raw["err_12v"],
            fault_lsd1=raw["err_lsd1"],
            fault_lsd2=raw["err_lsd2"],
            fault_aux=raw["err_aux"],
            fault_motor=raw["err_motor"],
            latch_active=raw["latch_active"],
            raw=raw,
        )

    @property
    def output_mask(self) -> int:
        """当前输出位域（bit0=24V, bit1=12V_ISO, bit2=LSD1, bit3=LSD2）。"""
        mask = 0
        for bit, on in enumerate((self.out_24v, self.out_12v,
                                  self.out_lsd1, self.out_lsd2)):
            if on:
                mask |= 1 << bit
        return mask

    @property
    def faults(self) -> list[str]:
        names = [
            ("24V", self.fault_24v),
            ("12V_ISO", self.fault_12v),
            ("LSD1", self.fault_lsd1),
            ("LSD2", self.fault_lsd2),
            ("AUX", self.fault_aux),
            ("MOTOR", self.fault_motor),
        ]
        active = [name for name, bad in names if bad]
        if self.latch_active:
            active.append("锁存")
        return active

    @property
    def healthy(self) -> bool:
        return not self.faults


@dataclass(frozen=True)
class SlaverVoltage(_DictMixin):
    """0x02 副电源模块电压（mV）。"""

    aux_mv: int = 0
    motor_mv: int = 0
    lsd1_mv: int = 0
    lsd2_mv: int = 0

    @classmethod
    def from_payload(cls, payload: bytes) -> "SlaverVoltage":
        values = decode_slv_volt(payload)
        return cls(aux_mv=values["aux_mv"], motor_mv=values["motor_mv"],
                   lsd1_mv=values["lsd1_mv"], lsd2_mv=values["lsd2_mv"])


@dataclass(frozen=True)
class SlaverTemperature(_DictMixin):
    """0x03 副电源模块 MCU 温度（°C）与 VDDA（mV）。"""

    mcu_c: float = 0.0
    vdda_mv: int = 0

    @classmethod
    def from_payload(cls, payload: bytes) -> "SlaverTemperature":
        values = decode_slv_temp(payload)
        return cls(mcu_c=values["mcu_c"], vdda_mv=values["vdda_mv"])


# =============================================================================
# 通用
# =============================================================================

@dataclass(frozen=True)
class FirmwareInfo(_DictMixin):
    """0x07 固件 / Boot 信息（两块板格式一致）。"""

    app_version: int = 0
    meta_version: int = 0
    fw_size: int = 0
    fw_checksum: int = 0
    reboot_counts: int = 0
    flags: int = 0

    @classmethod
    def from_payload(cls, payload: bytes) -> "FirmwareInfo":
        values = decode_info(payload)
        return cls(**values)

    @property
    def meta_valid(self) -> bool:
        return bool(self.flags & INFO_FLAG_META_VALID)

    @property
    def upgrade_pending(self) -> bool:
        return bool(self.flags & INFO_FLAG_UPGRADE_REQ)

    @property
    def upgrade_done(self) -> bool:
        return bool(self.flags & INFO_FLAG_UPGRADE_DONE)

    @property
    def flags_text(self) -> str:
        return info_flags_text(self.flags)


@dataclass(frozen=True)
class Ack(_DictMixin):
    """无数据段的命令应答（0x04/0x05/0x06）。"""

    device: str
    command: int
    elapsed_ms: float


@dataclass(frozen=True)
class PollStats(_DictMixin):
    """轮询统计快照。"""

    tx: int = 0
    rx: int = 0
    loss: int = 0
    loss_rate: float = 0.0
    fps: float = 0.0
