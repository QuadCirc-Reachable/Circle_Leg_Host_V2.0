"""帧编解码: [SOF(1) LEN(1) ID(1) HDR_CRC16(2) PAYLOAD(N) PKT_CRC16(2)]

与 Circle_Leg_Host_V1.0 / RosComm 一致。
"""

import struct
from typing import Callable, Optional

from .crc16 import append_crc16_checksum, get_crc16_checksum
from .ids import SOF


HEADER_SIZE = 3
HEADER_CRC_SIZE = 2
PKT_CRC_SIZE = 2


def build_frame(protocol_id: int, payload: bytes) -> bytes:
    """组装一个完整帧, 与 V1.0 main.py 中 frame 构造方式字节级一致。"""
    if not (0 <= protocol_id <= 0xFF):
        raise ValueError("protocol_id out of range")
    if len(payload) > 0xFF:
        raise ValueError("payload too long for u8 length field")

    header = struct.pack("<BBB", SOF, len(payload), protocol_id)
    header_with_crc = append_crc16_checksum(header)
    frame_partial = header_with_crc + payload
    return append_crc16_checksum(frame_partial)


class FrameParser:
    """字节流解析器: 容忍噪声, SOF 重同步, 双 CRC 校验, 通过 on_frame 回调输出。"""

    MAX_BUFFER = 4096

    def __init__(self, on_frame: Callable[[int, bytes], None]):
        self._on_frame = on_frame
        self._buf = bytearray()

    def feed(self, data: bytes) -> None:
        if not data:
            return
        self._buf.extend(data)
        if len(self._buf) > self.MAX_BUFFER:
            # 缓冲过长说明长时间未同步, 丢弃旧数据
            del self._buf[: len(self._buf) - self.MAX_BUFFER]
        self._try_parse()

    def _try_parse(self) -> None:
        while True:
            # 1. 找 SOF
            if not self._buf:
                return
            if self._buf[0] != SOF:
                # 跳到下一个可能的 SOF
                idx = self._buf.find(bytes([SOF]))
                if idx < 0:
                    self._buf.clear()
                    return
                del self._buf[:idx]

            # 2. 至少要有 header + header_crc
            if len(self._buf) < HEADER_SIZE + HEADER_CRC_SIZE:
                return

            data_len = self._buf[1]
            protocol_id = self._buf[2]
            header = bytes(self._buf[:HEADER_SIZE])
            hdr_crc = struct.unpack("<H", bytes(self._buf[HEADER_SIZE:HEADER_SIZE + HEADER_CRC_SIZE]))[0]

            if get_crc16_checksum(header) != hdr_crc:
                # 头 CRC 错, 丢一个字节重新同步
                del self._buf[0]
                continue

            total = HEADER_SIZE + HEADER_CRC_SIZE + data_len + PKT_CRC_SIZE
            if len(self._buf) < total:
                return

            payload = bytes(self._buf[HEADER_SIZE + HEADER_CRC_SIZE: HEADER_SIZE + HEADER_CRC_SIZE + data_len])
            pkt_body = bytes(self._buf[: total - PKT_CRC_SIZE])
            pkt_crc = struct.unpack("<H", bytes(self._buf[total - PKT_CRC_SIZE: total]))[0]

            if get_crc16_checksum(pkt_body) != pkt_crc:
                # 包 CRC 错, 丢一个字节重新同步
                del self._buf[0]
                continue

            del self._buf[:total]
            try:
                self._on_frame(protocol_id, payload)
            except Exception:  # 回调异常不能影响解析器
                pass
