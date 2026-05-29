"""帧封装与解析: byte-level 等价 V1.0, 解析器对噪声/拆包鲁棒。"""

import struct

from circle_leg_host.protocol.crc16 import append_crc16_checksum
from circle_leg_host.protocol.frame import FrameParser, build_frame
from circle_leg_host.protocol.ids import PROTO_PC_MSG, PROTO_VISION_CTRL
from circle_leg_host.protocol.messages import (
    pack_pc_msg,
    pack_vision_ctrl,
    unpack_vision_ctrl,
)


def _v1_build(payload: bytes, proto: int = 0xFF) -> bytes:
    sof = 0xAA
    header = struct.pack("<BBB", sof, len(payload), proto)
    return append_crc16_checksum(append_crc16_checksum(header) + payload)


def test_build_frame_matches_v1():
    payload = pack_pc_msg(1234, 500, -1, 0, 100, 200, 0b00000101, 0b00000010)
    assert len(payload) == 14
    a = build_frame(PROTO_PC_MSG, payload)
    b = _v1_build(payload, 0xFF)
    assert a == b


def test_parser_roundtrip_and_split_feed():
    payload = pack_vision_ctrl(enable=1, min_height_mm=25, flags=0)
    frame = build_frame(PROTO_VISION_CTRL, payload)

    received = []

    def cb(pid, pl):
        received.append((pid, pl))

    parser = FrameParser(cb)
    # 拆成两段 + 前后噪声
    parser.feed(b"\x00\x11" + frame[:5])
    parser.feed(frame[5:] + b"\xAA\x00")

    assert len(received) == 1
    pid, pl = received[0]
    assert pid == PROTO_VISION_CTRL
    msg = unpack_vision_ctrl(pl)
    assert msg.enable == 1 and msg.min_height_mm == 25 and msg.flags == 0


def test_parser_resync_on_bad_crc():
    good = build_frame(PROTO_PC_MSG, pack_pc_msg(0, 0, -1, 0, 0, 0, 0, 0))
    # 故意把头 CRC 破坏的一段 + 一个完好帧
    broken = bytearray(good)
    broken[3] ^= 0xFF  # 破坏 header CRC

    received = []
    parser = FrameParser(lambda pid, pl: received.append((pid, pl)))
    parser.feed(bytes(broken) + good)
    # 至少最后一个有效帧能被解出
    assert len(received) >= 1
    assert received[-1][0] == PROTO_PC_MSG


def test_parser_buffer_capped():
    received = []
    parser = FrameParser(lambda pid, pl: received.append((pid, pl)))
    parser.feed(b"\x00" * 10000)
    # 喂完后再喂一个完整帧, 仍能解
    parser.feed(build_frame(PROTO_PC_MSG, pack_pc_msg(0, 0, -1, 0, 0, 0, 0, 0)))
    assert len(received) == 1
