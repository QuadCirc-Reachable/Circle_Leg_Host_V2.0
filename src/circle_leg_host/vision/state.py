"""共享数据结构: 视觉最近一次结果 + 启停开关。

线程安全, 由 vision worker 写入, orchestrator 读取。
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class CurbResult:
    timestamp_ms: int = 0           # 写入时间戳
    detected: bool = False
    reliable: bool = False
    curb_height_m: float = 0.0      # 滤波后高度
    distance_m: float = 0.0         # 车前缘到台阶
    theta_deg: float = 0.0
    confidence: int = 0
    no_planes: bool = False
    theta_bad: bool = False
    dist_bad: bool = False
    fault: bool = False             # 相机/算法异常


class VisionState:
    def __init__(self, default_enabled: bool = True, default_min_height_mm: int = 20):
        self._lock = threading.Lock()
        self._enabled = bool(default_enabled)
        self._min_height_mm = int(default_min_height_mm)
        self._result = CurbResult()
        self._last_ctrl_ms: int = 0

    # ---- control (orchestrator -> worker) ----
    def set_enable(self, enable: bool) -> None:
        with self._lock:
            self._enabled = bool(enable)

    def set_min_height_mm(self, mm: int) -> None:
        with self._lock:
            self._min_height_mm = max(0, int(mm))

    def mark_ctrl_received(self) -> None:
        with self._lock:
            self._last_ctrl_ms = int(time.monotonic() * 1000)

    def get_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def get_min_height_m(self) -> float:
        with self._lock:
            return self._min_height_mm / 1000.0

    def last_ctrl_ms(self) -> int:
        with self._lock:
            return self._last_ctrl_ms

    # ---- result (worker -> orchestrator) ----
    def set_result(self, result: CurbResult) -> None:
        with self._lock:
            self._result = result

    def get_result(self) -> CurbResult:
        with self._lock:
            # 浅拷贝够用 (字段都是不可变标量)
            r = self._result
            return CurbResult(
                timestamp_ms=r.timestamp_ms,
                detected=r.detected,
                reliable=r.reliable,
                curb_height_m=r.curb_height_m,
                distance_m=r.distance_m,
                theta_deg=r.theta_deg,
                confidence=r.confidence,
                no_planes=r.no_planes,
                theta_bad=r.theta_bad,
                dist_bad=r.dist_bad,
                fault=r.fault,
            )
