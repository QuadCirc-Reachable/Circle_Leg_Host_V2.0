"""端到端: pty 一端跑 SerialLink + Router + VisionOrchestrator,
另一端模拟 MCU: 发 0xFD + 0xFE, 读 0xFC + 0xFF."""

import os
import time

import pytest

pty = pytest.importorskip("pty", reason="pty (Linux/macOS) required")

from circle_leg_host.app.vision_orchestrator import VisionOrchestrator
from circle_leg_host.comm.router import FrameRouter
from circle_leg_host.comm.serial_link import SerialLink
from circle_leg_host.protocol.frame import FrameParser, build_frame
from circle_leg_host.protocol.ids import (
    PROTO_PC_MSG,
    PROTO_REACHABLE,
    PROTO_VISION_CTRL,
    PROTO_VISION_RESULT,
)
from circle_leg_host.protocol.messages import (
    pack_pc_msg,
    pack_vision_ctrl,
    unpack_vision_result,
)
from circle_leg_host.vision.state import CurbResult, VisionState


pytestmark = pytest.mark.skipif(not hasattr(os, "openpty"), reason="pty required")


def _open_pty():
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    os.close(slave)
    return master, slave_path


def test_e2e_vision_control_and_result_flow():
    master, slave_path = _open_pty()

    # --- host side ---
    router = FrameRouter()
    link = SerialLink(router=router, preferred_port=slave_path,
                      baudrate=115200,
                      reconnect_interval_ms=200, mcu_idle_timeout_ms=300,
                      mcu_dead_timeout_ms=10000)
    link.start()

    state = VisionState(default_enabled=False, default_min_height_mm=20)
    orch = VisionOrchestrator(link=link, router=router, state=state,
                              publish_period_ms=50, ctrl_timeout_ms=2000,
                              control_required=True, default_enabled=False,
                              result_stale_ms=10000)
    orch.start()

    # --- mock MCU side ---
    mcu_received = []
    mcu_parser = FrameParser(lambda pid, pl: mcu_received.append((pid, pl)))

    try:
        # 等串口连通
        deadline = time.monotonic() + 2
        while not link.is_open() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert link.is_open()

        # 1) MCU 发 0xFD enable=1, min_height=35
        os.write(master, build_frame(PROTO_VISION_CTRL,
                                     pack_vision_ctrl(1, 35, 0)))

        # 等 host 处理
        end = time.monotonic() + 1
        while time.monotonic() < end and not state.get_enabled():
            time.sleep(0.02)
        assert state.get_enabled() is True
        assert state.get_min_height_m() == 0.035

        # 2) 注入一个"识别到"的结果, host 应通过 0xFC 上报回 MCU
        state.set_result(CurbResult(
            timestamp_ms=int(time.monotonic() * 1000),
            detected=True, reliable=True,
            curb_height_m=0.08, distance_m=0.50, theta_deg=-5.5,
            confidence=92,
        ))

        # 3) MCU 同时也发 0xFE 假数据, host 应能路由
        host_seen_fe = []
        router.register(PROTO_REACHABLE, lambda pid, pl: host_seen_fe.append((pid, pl)))
        os.write(master, build_frame(PROTO_REACHABLE, b"\x00" * 40))

        # 持续读 master, 直到看到 curb_height_mm=80 的 0xFC (跳过早期 stale 帧)
        seen_fc = None
        end = time.monotonic() + 3
        while time.monotonic() < end:
            try:
                chunk = os.read(master, 1024)
            except BlockingIOError:
                chunk = b""
            if chunk:
                mcu_parser.feed(chunk)
            for pid, pl in mcu_received:
                if pid == PROTO_VISION_RESULT:
                    msg = unpack_vision_result(pl)
                    if msg.curb_height_mm == 80:
                        seen_fc = pl
                        break
            if seen_fc is not None:
                break
            time.sleep(0.02)

        assert seen_fc is not None, "MCU never received expected 0xFC"
        msg = unpack_vision_result(seen_fc)
        assert msg.curb_height_mm == 80
        assert msg.distance_mm == 500
        assert msg.theta_cdeg == -550
        assert msg.confidence == 92

        # 4) host 应该已经路由了 0xFE
        end = time.monotonic() + 0.5
        while time.monotonic() < end and not host_seen_fe:
            time.sleep(0.02)
        assert host_seen_fe, "Host failed to route 0xFE"

        # 5) MCU 发 0xFD enable=0 -> host 关掉视觉
        os.write(master, build_frame(PROTO_VISION_CTRL,
                                     pack_vision_ctrl(0, 35, 0)))
        end = time.monotonic() + 1
        while time.monotonic() < end and state.get_enabled():
            time.sleep(0.02)
        assert state.get_enabled() is False

    finally:
        orch.stop()
        link.stop()
        try:
            os.close(master)
        except Exception:
            pass


def test_e2e_pc_msg_byte_exact_v1():
    """0xFF 帧通过 SerialLink 发出后, MCU 端字节流应与 V1.0 一致."""
    master, slave_path = _open_pty()
    router = FrameRouter()
    link = SerialLink(router=router, preferred_port=slave_path,
                      baudrate=115200,
                      reconnect_interval_ms=200, mcu_idle_timeout_ms=300)
    link.start()
    try:
        deadline = time.monotonic() + 2
        while not link.is_open() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert link.is_open()

        payload = pack_pc_msg(1234, 500, -1, 0, 100, 200, 0b00000101, 0b00000010)
        frame = build_frame(PROTO_PC_MSG, payload)
        link.send_frame(frame)

        # 读回, 应字节完全一致
        end = time.monotonic() + 1
        buf = b""
        while time.monotonic() < end and len(buf) < len(frame):
            try:
                buf += os.read(master, 256)
            except BlockingIOError:
                time.sleep(0.01)
        assert frame in buf
        # 期望长度: 1+1+1 + 2 + 14 + 2 = 21
        assert len(frame) == 21
    finally:
        link.stop()
        try:
            os.close(master)
        except Exception:
            pass
