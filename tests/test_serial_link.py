"""SerialLink: 打开/关闭/重连行为 (使用 Linux pty 模拟)."""

import os
import time

import pytest

pty = pytest.importorskip("pty", reason="pty (Linux/macOS) required")

from circle_leg_host.comm.router import FrameRouter
from circle_leg_host.comm.serial_link import SerialLink
from circle_leg_host.protocol.frame import build_frame
from circle_leg_host.protocol.ids import PROTO_PC_MSG, PROTO_REACHABLE
from circle_leg_host.protocol.messages import pack_pc_msg


pytestmark = pytest.mark.skipif(not hasattr(os, "openpty"), reason="pty required (Linux/macOS)")


def _open_pty():
    """返回 (master_fd, slave_device_path)."""
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    # 关掉 slave fd; pyserial 自己会重新 open slave_path
    os.close(slave)
    return master, slave_path


def test_open_send_receive():
    master, slave_path = _open_pty()
    try:
        router = FrameRouter()
        received = []
        router.register(PROTO_REACHABLE, lambda pid, pl: received.append((pid, pl)))

        link = SerialLink(router=router, preferred_port=slave_path,
                          baudrate=115200, reconnect_interval_ms=300,
                          mcu_idle_timeout_ms=200, mcu_dead_timeout_ms=10000)
        link.start()

        # 等串口打开
        deadline = time.monotonic() + 2
        while not link.is_open() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert link.is_open(), "SerialLink failed to open pty"

        # 模拟 MCU -> PC: 发一个 0xFE 假帧
        fake_payload = b"\x00" * 40
        frame = build_frame(PROTO_REACHABLE, fake_payload)
        os.write(master, frame)

        # 模拟 PC -> MCU: SerialLink 发一帧, master 端应收到
        tx = build_frame(PROTO_PC_MSG, pack_pc_msg(900, 500, -1, 0, 0, 0, 0, 0))
        ok = link.send_frame(tx)
        assert ok

        # 主端读回
        deadline = time.monotonic() + 1
        buf = b""
        while time.monotonic() < deadline and len(buf) < len(tx):
            try:
                buf += os.read(master, 128)
            except BlockingIOError:
                time.sleep(0.01)
        assert tx in buf, f"TX mismatch: got {buf!r}"

        # 路由器应收到 0xFE
        deadline = time.monotonic() + 1
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        assert received, "0xFE not routed"
        assert received[0][0] == PROTO_REACHABLE
        assert received[0][1] == fake_payload

        link.stop()
    finally:
        try:
            os.close(master)
        except Exception:
            pass


def test_reconnect_after_disconnect():
    """关掉 pty 后, SerialLink 应在 reconnect_interval_ms 节流内尝试重连."""
    master, slave_path = _open_pty()
    router = FrameRouter()
    link = SerialLink(router=router, preferred_port=slave_path,
                      baudrate=115200,
                      reconnect_interval_ms=200, mcu_idle_timeout_ms=200,
                      mcu_dead_timeout_ms=10000)
    link.start()

    # 等首次连通
    deadline = time.monotonic() + 2
    while not link.is_open() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert link.is_open()

    # 关掉 master 端 -> pyserial 下一次 read 会失败
    os.close(master)

    # 等 SerialLink 检测到并关闭
    deadline = time.monotonic() + 2
    while link.is_open() and time.monotonic() < deadline:
        time.sleep(0.05)
    # 注意: pty 关闭后 slave 也消失, port_exists 会返回 False, 重连会失败 -> 状态应为 closed
    assert not link.is_open(), "Link should be closed after master fd gone"

    # 现在 send_frame 应返回 False, 不抛
    assert link.send_frame(b"\xAA\x00\xFF") is False

    link.stop()
