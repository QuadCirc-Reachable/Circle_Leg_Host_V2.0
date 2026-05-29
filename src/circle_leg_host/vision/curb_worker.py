"""视觉线程: 启停控制 + 帧去抖 + 一阶低通 + 共享状态写入。

行为:
- enabled=False: 线程仅 sleep, 不构造/不调用 pipeline (CPU 接近 0)。
- enabled=True : 懒构造 pipeline, 循环 step(); 异常时关闭 pipeline, 标记 fault, 之后按节流重试。
- K 帧连续 reliable 才把 detected 置 True。
- 高度做 alpha 低通。
"""

import logging
import threading
import time
from typing import Optional

from .state import CurbResult, VisionState


log = logging.getLogger(__name__)


def _ms() -> int:
    return int(time.monotonic() * 1000)


class CurbWorker:
    RETRY_INTERVAL_MS = 1500

    def __init__(
        self,
        state: VisionState,
        realsense_yaml: Optional[str] = None,
        curbsvm1_path: Optional[str] = None,
        camera_to_front_edge_m: float = 0.30,
        k_confirm: int = 3,
        alpha: float = 0.3,
        show_window: bool = False,
    ):
        self.state = state
        self.realsense_yaml = realsense_yaml
        self.curbsvm1_path = curbsvm1_path
        self.camera_to_front_edge_m = float(camera_to_front_edge_m)
        self.k_confirm = max(1, int(k_confirm))
        self.alpha = float(alpha)
        self.show_window = bool(show_window)

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pipeline = None
        self._last_open_attempt_ms = 0

        self._reliable_streak = 0
        self._h_filt = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close_pipeline()

    # ---------------- internal ----------------

    def _open_pipeline(self) -> bool:
        from .curb_pipeline import CurbPipeline  # 延迟 import

        now = _ms()
        if (now - self._last_open_attempt_ms) < self.RETRY_INTERVAL_MS:
            return False
        self._last_open_attempt_ms = now

        try:
            self._pipeline = CurbPipeline(
                realsense_yaml=self.realsense_yaml,
                curbsvm1_path=self.curbsvm1_path,
                camera_to_front_edge_m=self.camera_to_front_edge_m,
                show_window=self.show_window,
            )
            log.info("Vision pipeline opened")
            return True
        except Exception as exc:
            log.warning("Open vision pipeline failed: %s", exc)
            self._publish_fault()
            return False

    def _close_pipeline(self) -> None:
        if self._pipeline is None:
            return
        try:
            self._pipeline.close()
        except Exception:
            pass
        self._pipeline = None

    def _publish_fault(self) -> None:
        r = CurbResult(timestamp_ms=_ms(), fault=True)
        self.state.set_result(r)

    def _publish_disabled(self) -> None:
        r = CurbResult(timestamp_ms=_ms())
        self.state.set_result(r)
        self._reliable_streak = 0
        self._h_filt = 0.0

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.state.get_enabled():
                # 关闭算力: 释放 pipeline, 进入低功耗
                if self._pipeline is not None:
                    self._close_pipeline()
                self._publish_disabled()
                time.sleep(0.1)
                continue

            if self._pipeline is None:
                if not self._open_pipeline():
                    time.sleep(0.2)
                    continue

            try:
                pf = self._pipeline.step()
            except Exception as exc:
                log.warning("Vision step failed: %s", exc)
                self._close_pipeline()
                self._publish_fault()
                continue

            if pf is None:
                # 无新帧, 短睡一会
                time.sleep(0.005)
                continue

            self._publish_normal(pf)

    def _publish_normal(self, pf) -> None:
        min_h = self.state.get_min_height_m()
        result = CurbResult(timestamp_ms=_ms())

        if pf.curb_height_m <= 0.0 or not pf.found_planes:
            # 没找到双平面
            self._reliable_streak = 0
            self._h_filt = 0.0
            result.no_planes = True
            result.confidence = 0
            self.state.set_result(result)
            return

        # 一阶低通
        if self._h_filt <= 0.0:
            self._h_filt = pf.curb_height_m
        else:
            self._h_filt = self.alpha * pf.curb_height_m + (1.0 - self.alpha) * self._h_filt

        # 可靠性 (与 curbsvm1 现有规则一致)
        theta = pf.theta_deg or 0.0
        dist = pf.distance_m or 0.0
        theta_bad = not ((-65 <= theta <= 65) or (-180 <= theta <= -120))
        dist_bad = dist < 0.15
        height_ok = self._h_filt >= min_h

        is_reliable = height_ok and (not theta_bad) and (not dist_bad)
        if is_reliable:
            self._reliable_streak = min(self.k_confirm, self._reliable_streak + 1)
        else:
            self._reliable_streak = 0

        detected = self._reliable_streak >= self.k_confirm

        result.curb_height_m = self._h_filt
        result.distance_m = self._pipeline.ground_distance_from_front(dist)
        result.theta_deg = theta
        result.reliable = is_reliable and height_ok
        result.detected = detected
        result.theta_bad = theta_bad
        result.dist_bad = dist_bad
        result.confidence = 90 if (detected and not theta_bad and not dist_bad) else (45 if height_ok else 0)
        self.state.set_result(result)
