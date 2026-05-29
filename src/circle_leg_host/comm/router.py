"""ID -> callback 路由。线程安全。"""

import threading
from typing import Callable, Dict


FrameCallback = Callable[[int, bytes], None]


class FrameRouter:
    def __init__(self):
        self._lock = threading.Lock()
        self._cbs: Dict[int, FrameCallback] = {}

    def register(self, protocol_id: int, cb: FrameCallback) -> None:
        with self._lock:
            self._cbs[protocol_id & 0xFF] = cb

    def unregister(self, protocol_id: int) -> None:
        with self._lock:
            self._cbs.pop(protocol_id & 0xFF, None)

    def dispatch(self, protocol_id: int, payload: bytes) -> None:
        with self._lock:
            cb = self._cbs.get(protocol_id & 0xFF)
        if cb is None:
            return
        try:
            cb(protocol_id, payload)
        except Exception:
            # 回调异常隔离, 不影响其它链路
            pass
