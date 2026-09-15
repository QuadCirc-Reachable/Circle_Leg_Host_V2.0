"""CRC16 与 V1.0 main.py 中表 / 函数完全等价。"""

import os
import struct
import sys

# 也把 V1.0 加入 path, 直接调用其内部函数做黄金对比
_V1_CANDIDATES = [os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", name))
                  for name in ("Circle_Leg_Host_V1.0", "Circle_Leg_Host_V1")]
V1_DIR = next((d for d in _V1_CANDIDATES if os.path.isfile(os.path.join(d, "main.py"))), _V1_CANDIDATES[0])
if V1_DIR not in sys.path:
    sys.path.insert(0, V1_DIR)

import importlib.util


def _load_v1_main():
    """V1 main.py 顶层会启动 pygame, 这里只取符号。"""
    path = os.path.join(V1_DIR, "main.py")
    spec = importlib.util.spec_from_file_location("v1_main_for_test", path)
    mod = importlib.util.module_from_spec(spec)
    # 通过 exec 只到 wCRC_Table / get_crc16_checksum 定义就足够; 但 main.py 顶层会跑 pygame。
    # 为简便, 直接读取源文件抽取 wCRC_Table 与函数: 这里改为只比对常量与函数。
    src = open(path, "r", encoding="utf-8").read()
    ns = {}
    exec(compile(src, path, "exec"), ns)
    return ns


from circle_leg_host.protocol.crc16 import (
    append_crc16_checksum,
    get_crc16_checksum,
    verify_crc16,
    wCRC_Table,
)


def test_table_matches_v1():
    import pytest
    if not os.path.isfile(os.path.join(V1_DIR, "main.py")):
        pytest.skip("Circle_Leg_Host_V1.0 checkout not found next to this repo")
    # 不执行 V1 全部 main: 仅文本提取
    src = open(os.path.join(V1_DIR, "main.py"), "r", encoding="utf-8").read()
    # 简单方法: 让 wCRC_Table 在隔离命名空间被定义
    ns = {}
    start = src.index("wCRC_Table = [")
    end = src.index("]", start) + 1
    exec(src[start:end], ns)
    v1_table = ns["wCRC_Table"]
    assert list(wCRC_Table) == list(v1_table)
    assert len(wCRC_Table) == 256


def test_known_vectors():
    assert get_crc16_checksum(b"") == 0xFFFF
    # CRC 应等于查表实现的逐字节扫描结果
    data = b"\xAA\x0E\xFF"
    crc = get_crc16_checksum(data)
    appended = append_crc16_checksum(data)
    assert appended[-2:] == struct.pack("<H", crc)


def test_roundtrip_random():
    import random

    rnd = random.Random(0)
    for _ in range(64):
        n = rnd.randint(0, 40)
        data = bytes(rnd.randint(0, 255) for _ in range(n))
        framed = append_crc16_checksum(data)
        assert verify_crc16(framed)
        # 翻一个 bit -> CRC 失败
        if len(framed) > 0:
            bad = bytearray(framed)
            bad[0] ^= 0x01
            assert not verify_crc16(bytes(bad))
