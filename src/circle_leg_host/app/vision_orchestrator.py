"""Vision 编排器: 把 0xFD ctrl 灌进 VisionState; 周期发 0xFC result。

线程: 1 个 TX 节拍线程 (publish_period_ms, 默认 50ms = 20Hz)。
0xFD 回调由 SerialLink RX 线程同步触发, 仅做轻量赋值。
"""

import logging
import threading
import time
from typing import Optional

from ..comm.router import FrameRouter
from ..comm.serial_link import SerialLink
from ..protocol.frame import build_frame
from ..protocol.ids import PROTO_VISION_CTRL, PROTO_VISION_RESULT
from ..protocol.messages import (
    VFLAG_DIST_BAD,
    VFLAG_NO_PLANES,
    VFLAG_STALE,
    VFLAG_THETA_BAD,
    VSTATUS_DETECTED,
    VSTATUS_DISABLED,
    VSTATUS_FAULT,
    VSTATUS_RELIABLE,
    VisionResultMsg,
    unpack_vision_ctrl,
)
from ..vision.state import VisionState


log = logging.getLogger(__name__)


def _ms() -> int:
    return int(time.monotonic() * 1000)


class VisionOrchestrator:
    def __init__(
        self,
        link: SerialLink,
        router: FrameRouter,
        state: VisionState,
        publish_period_ms: int = 50,
        ctrl_timeout_ms: int = 1500,
        control_required: bool = False,
        default_enabled: bool = True,
        result_stale_ms: int = 500,
    ):
        self.link = link
        self.router = router
        self.state = state
        self.publish_period_ms = max(10, int(publish_period_ms))
        self.ctrl_timeout_ms = max(0, int(ctrl_timeout_ms))
        self.control_required = bool(control_required)
        self.default_enabled = bool(default_enabled)
        self.result_stale_ms = max(0, int(result_stale_ms))

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0

    # ---------------- public ----------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.router.register(PROTO_VISION_CTRL, self._on_ctrl)
        self._stop.clear()
        self._thread = threading.Thread(target=self._tx_loop, name="vision-orch", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.router.unregister(PROTO_VISION_CTRL)

    # ---------------- handlers ----------------

    def _on_ctrl(self, _protocol_id: int, payload: bytes) -> None:
        try:
            msg = unpack_vision_ctrl(payload)
        except Exception as exc:
            log.warning("Bad 0xFD payload: %s", exc)
            return
        self.state.mark_ctrl_received()
        self.state.set_enable(bool(msg.enable))
        self.state.set_min_height_mm(int(msg.min_height_mm))

    def _apply_watchdog(self) -> None:
        if not self.control_required:
            return
        last = self.state.last_ctrl_ms()
        if last == 0:
            # 从未收到 ctrl: 强制保持关闭 (control_required 的语义)
            self.state.set_enable(False)
            return
        if (_ms() - last) > self.ctrl_timeout_ms:
            self.state.set_enable(False)

    def _tx_loop(self) -> None:
        # 启动时根据默认设置 enable
        self.state.set_enable(self.default_enabled if not self.control_required else False)
        period = self.publish_period_ms / 1000.0

        while not self._stop.is_set():
            t0 = time.monotonic()
            self._apply_watchdog()
            self._publish()
            elapsed = time.monotonic() - t0
            sleep_for = period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _publish(self) -> None:
        if not self.link.is_open():
            return

        r = self.state.get_result()
        enabled = self.state.get_enabled()
        now = _ms()
        stale = (now - r.timestamp_ms) > self.result_stale_ms if r.timestamp_ms else True

        status = 0
        flags = 0
        if not enabled:
            status |= VSTATUS_DISABLED
        if r.fault:
            status |= VSTATUS_FAULT
        if r.detected:
            status |= VSTATUS_DETECTED
        if r.reliable:
            status |= VSTATUS_RELIABLE
        if r.no_planes:
            flags |= VFLAG_NO_PLANES
        if r.theta_bad:
            flags |= VFLAG_THETA_BAD
        if r.dist_bad:
            flags |= VFLAG_DIST_BAD
        if stale:
            flags |= VFLAG_STALE

        # 量化
        curb_h_mm = max(0, min(0xFFFF, int(round(r.curb_height_m * 1000.0))))
        dist_mm = max(0, min(0xFFFF, int(round(r.distance_m * 1000.0))))
        theta_cdeg = max(-32768, min(32767, int(round(r.theta_deg * 100.0))))
        confidence = max(0, min(255, int(r.confidence)))

        self._seq = (self._seq + 1) & 0xFF
        msg = VisionResultMsg(
            seq=self._seq,
            status=status,
            curb_height_mm=curb_h_mm,
            distance_mm=dist_mm,
            theta_cdeg=theta_cdeg,
            confidence=confidence,
            flags=flags,
        )
        try:
            frame = build_frame(PROTO_VISION_RESULT, msg.to_bytes())
        except Exception as exc:
            log.exception("Build 0xFC frame failed: %s", exc)
            return
        self.link.send_frame(frame)
