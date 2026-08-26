#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""juxie_host.py 协议层自检（无需串口/硬件）

运行：python test_protocol.py
覆盖：CRC16_CCITT_FALSE、20 字节帧打包/解析、MIT 12bit 位打包、
     物理量→原始值换算、应答解析、帧流重同步/分片。
"""
import struct
import sys

import juxie_host as jh


def main():
    # 1) CRC16_CCITT_FALSE 对照 uart_protocol.md 示例：can_id=0xA0 data=02 01.. => 0x773F
    payload = bytes.fromhex("A0 00 00 00 02 01 00 00 00 00 00 00")
    crc = jh.crc16_ccitt_false(payload)
    assert crc == 0x773F, "CRC mismatch: %04X" % crc
    print("1) CRC ok: 0x%04X" % crc)

    # 2) pack/unpack 往返
    for cid in (jh.CID_GET_MZ, jh.CID_GET_FB, jh.CID_MIT_CTRL):
        c, d = jh.unpack_frame(jh.pack_frame(cid))
        assert c == cid and d == b"\x00" * 8
    print("2) pack/unpack round-trip ok")

    # 2b) 电机标零命令帧
    c, d = jh.unpack_frame(jh.cmd_motor_zero())
    assert c == jh.CID_MOTOR_ZERO and d == b"\x00" * 8
    print("2b) motor_zero cmd ok")

    # 3) MIT 载荷位打包对照 juxie §4.1
    f = jh.mit_ctrl(0x4000, 0x123, 0x456, 0x078, 0x9AB)
    d = f[8:16]
    assert d == bytes.fromhex("40 00 12 34 56 07 89 AB"), d.hex()
    print("3) MIT 位打包 ok:", d.hex())

    # 4) 物理量→原始值换算（int() 截断，中点差 1 LSB 可忽略）
    assert jh.to_pos_raw(0.0, 180.0) == 32767
    assert jh.to_pos_raw(180.0, 180.0) == 65535
    assert jh.to_pos_raw(-180.0, 180.0) == 0
    assert jh.to_kp_raw(250.0) == 2047
    assert jh.to_vel_raw(3000.0, 3000.0) == 4095
    assert jh.to_tq_raw(0.0, 50.0) == 2047
    print("4) 物理量换算 ok")

    # 5) 应答解析
    fb = jh.parse_fb(bytes.fromhex("E8 03 2C 01 90 01 20 00"))
    assert abs(fb["pos_deg"] - 10.00) < 1e-6 and fb["speed_rpm"] == 300
    assert fb["iq_ma"] == 400 and abs(fb["tq_nm"] - 0.32) < 1e-6, fb
    st = jh.parse_st(bytes.fromhex("00 00 64 00 06 F0"))
    assert st["err"] == 0 and abs(st["temp_c"] - 10.0) < 1e-6
    assert st["mode"] == 6 and st["en"] and st["brake"] and st["in_pos"], st
    mz = jh.parse_mz(struct.pack("<f", -0.79) + b"\x2a\x00" + b"\x01")
    assert abs(mz["mz_nm"] - (-0.79)) < 1e-6 and mz["seq"] == 42 and mz["online"], mz
    print("5) 应答解析 ok (fb/st/mz)")

    # 6) FrameParser：垃圾 + 两帧粘连 + 坏帧重同步
    raw = (b"\x00\x11\x22" + jh.pack_frame(jh.CID_GET_MZ)
           + b"\xde\xad\xbe\xef" + jh.pack_frame(jh.CID_RSP_MZ))
    ids = [c for c, _ in jh.FrameParser().feed(raw)]
    assert jh.CID_GET_MZ in ids and jh.CID_RSP_MZ in ids, ids

    # 分片输入：同一解析器实例累积验证
    f = jh.pack_frame(jh.CID_GET_FB)
    p = jh.FrameParser()
    got = []
    for i in range(0, len(f), 7):
        got += p.feed(f[i:i + 7])
    assert len(got) == 1 and got[0][0] == jh.CID_GET_FB, got
    print("6) FrameParser 重同步/分片 ok")

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
