"""VisionOrchestrator: 0xFD ctrl 路由, 周期 0xFC 上报, 看门狗超时."""

import struct
import time
from typing import List, Tuple

from circle_leg_host.app.vision_orchestrator import VisionOrchestrator
from circle_leg_host.comm.router import FrameRouter
from circle_leg_host.protocol.frame import FrameParser, build_frame
from circle_leg_host.protocol.ids import PROTO_VISION_CTRL, PROTO_VISION_RESULT
from circle_leg_host.protocol.messages import (
    VSTATUS_DETECTED,
    VSTATUS_DISABLED,
    VSTATUS_RELIABLE,
    pack_vision_ctrl,
    unpack_vision_result,
)
from circle_leg_host.vision.state import CurbResult, VisionState


class FakeLink:
    """伪装 SerialLink: 永远 is_open=True, send_frame 收进 list + 解析回调."""

    def __init__(self):
        self.frames: List[bytes] = []
        self.parsed: List[Tuple[int, bytes]] = []
        self._parser = FrameParser(lambda pid, pl: self.parsed.append((pid, pl)))

    def is_open(self):
        return True

    def is_mcu_alive(self):
        return True

    def send_frame(self, frame: bytes) -> bool:
        self.frames.append(frame)
        self._parser.feed(frame)
        return True


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_publishes_periodic_0xFC():
    link = FakeLink()
    router = FrameRouter()
    state = VisionState(default_enabled=True, default_min_height_mm=20)
    orch = VisionOrchestrator(link=link, router=router, state=state,
                              publish_period_ms=30, ctrl_timeout_ms=2000,
                              control_required=False, default_enabled=True,
                              result_stale_ms=10000)
    orch.start()
    try:
        assert _wait(lambda: len(link.parsed) >= 3, timeout=2.0)
        # 全部应为 0xFC, payload 10 字节
        for pid, pl in link.parsed[:3]:
            assert pid == PROTO_VISION_RESULT
            assert len(pl) == 10
        # seq 应连续递增
        seqs = [pl[0] for _, pl in link.parsed[:3]]
        assert seqs[1] == (seqs[0] + 1) & 0xFF
        assert seqs[2] == (seqs[1] + 1) & 0xFF
    finally:
        orch.stop()


def test_ctrl_0xFD_updates_state():
    link = FakeLink()
    router = FrameRouter()
    state = VisionState(default_enabled=False, default_min_height_mm=20)
    orch = VisionOrchestrator(link=link, router=router, state=state,
                              publish_period_ms=30, ctrl_timeout_ms=2000,
                              control_required=True, default_enabled=False,
                              result_stale_ms=10000)
    orch.start()
    try:
        # 灌入 0xFD: enable=1, min_height_mm=42
        payload = pack_vision_ctrl(enable=1, min_height_mm=42, flags=0)
        router.dispatch(PROTO_VISION_CTRL, payload)
        assert _wait(lambda: state.get_enabled() is True)
        assert state.get_min_height_m() == 0.042

        # 再灌一次关掉
        payload = pack_vision_ctrl(enable=0, min_height_mm=42, flags=0)
        router.dispatch(PROTO_VISION_CTRL, payload)
        assert _wait(lambda: state.get_enabled() is False)
    finally:
        orch.stop()


def test_watchdog_disables_when_ctrl_silent():
    link = FakeLink()
    router = FrameRouter()
    state = VisionState(default_enabled=True, default_min_height_mm=20)
    orch = VisionOrchestrator(link=link, router=router, state=state,
                              publish_period_ms=30, ctrl_timeout_ms=120,
                              control_required=True, default_enabled=True)
    orch.start()
    try:
        # control_required=True 时 启动初始 enable=False (没收到 ctrl)
        time.sleep(0.05)
        assert state.get_enabled() is False

        # 灌一次 enable=1 -> 短暂 enable
        router.dispatch(PROTO_VISION_CTRL, pack_vision_ctrl(1, 20, 0))
        assert _wait(lambda: state.get_enabled() is True, timeout=0.5)

        # 静默超过 ctrl_timeout_ms -> 自动关闭
        assert _wait(lambda: state.get_enabled() is False, timeout=1.5)
    finally:
        orch.stop()


def test_result_status_encoding():
    """喂入一个 detected+reliable 的 CurbResult, 0xFC payload status 应正确编码."""
    link = FakeLink()
    router = FrameRouter()
    state = VisionState(default_enabled=True, default_min_height_mm=20)
    state.set_result(CurbResult(
        timestamp_ms=int(time.monotonic() * 1000),
        detected=True, reliable=True,
        curb_height_m=0.123, distance_m=0.456, theta_deg=12.34,
        confidence=88,
    ))
    orch = VisionOrchestrator(link=link, router=router, state=state,
                              publish_period_ms=30, ctrl_timeout_ms=2000,
                              control_required=False, default_enabled=True,
                              result_stale_ms=10000)
    orch.start()
    try:
        assert _wait(lambda: len(link.parsed) >= 1)
        pid, pl = link.parsed[-1]
        assert pid == PROTO_VISION_RESULT
        msg = unpack_vision_result(pl)
        assert msg.status & VSTATUS_DETECTED
        assert msg.status & VSTATUS_RELIABLE
        assert not (msg.status & VSTATUS_DISABLED)
        assert msg.curb_height_mm == 123
        assert msg.distance_mm == 456
        assert msg.theta_cdeg == 1234
        assert msg.confidence == 88
    finally:
        orch.stop()
