"""共享单串口收发链路。

设计:
- 1 个 RX 线程: 持续读字节流, 喂入 FrameParser, 通过 FrameRouter 分发。
- TX: 主调用方 (gamepad / orchestrator) 通过 send_frame() 同步写, 内部加锁,
  失败立刻关闭串口并触发后台重连。
- 自动重连: 复用 V1.0 节流策略 (reconnect_interval_ms).
- MCU 活跃检测: 任何 RX 字节均刷新 last_rx_ms; 调用方可读取 is_mcu_alive / age 做发送门控。
"""

import logging
import os
import threading
import time
from typing import Optional

import serial

from ..protocol.frame import FrameParser
from ..utils.port_select import port_exists, select_serial_port
from .router import FrameRouter


log = logging.getLogger(__name__)


def _ms() -> int:
    return int(time.monotonic() * 1000)


class SerialLink:
    def __init__(
        self,
        router: FrameRouter,
        preferred_port: Optional[str] = "/dev/ttyACM0",
        baudrate: int = 2000000,
        read_timeout: float = 0.1,
        write_timeout: float = 0.1,
        reconnect_interval_ms: int = 2000,
        mcu_idle_timeout_ms: int = 1000,
        mcu_dead_timeout_ms: int = 8000,
    ):
        self.router = router
        self.preferred_port = preferred_port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self.reconnect_interval_ms = reconnect_interval_ms
        self.mcu_idle_timeout_ms = mcu_idle_timeout_ms
        self.mcu_dead_timeout_ms = mcu_dead_timeout_ms

        self._ser: Optional[serial.Serial] = None
        self._current_port: Optional[str] = preferred_port
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._last_rx_ms = 0
        self._last_reconnect_ms = 0
        self._parser = FrameParser(self._on_frame)
        self._last_state_log: Optional[str] = None

    # ---------------- public API ----------------

    def start(self) -> None:
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._stop.clear()
        self._try_open()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="serial-rx", daemon=True)
        self._rx_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        self._close()

    def is_open(self) -> bool:
        with self._state_lock:
            return self._ser is not None and self._ser.is_open

    def is_mcu_alive(self) -> bool:
        if not self.is_open():
            return False
        return (_ms() - self._last_rx_ms) <= self.mcu_idle_timeout_ms

    def send_frame(self, frame: bytes) -> bool:
        """发送一个完整帧。串口异常会立即关闭, 由 RX 线程重连。"""
        if self._stop.is_set():
            return False
        with self._tx_lock:
            ser = self._ser
            if ser is None or not ser.is_open:
                return False
            try:
                ser.write(frame)
                return True
            except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
                log.warning("UART write error: %s", exc)
                self._close()
                return False

    @property
    def current_port(self) -> Optional[str]:
        return self._current_port

    # ---------------- internal ----------------

    def _try_open(self) -> bool:
        port = self._current_port
        if not port or not port_exists(port):
            # 优先用 list_ports; 若失败 (pty/某些 USB 不在 comports 中) 回退到原始路径
            picked = select_serial_port(self.preferred_port)
            if picked:
                port = picked
            elif port and os.path.exists(port):
                # 路径存在 (例如 pty), 直接尝试
                pass
            else:
                self._log_state("disconnected")
                return False

        try:
            ser = serial.Serial(
                port,
                self.baudrate,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            with self._state_lock:
                self._ser = ser
                self._current_port = port
            self._last_rx_ms = _ms()
            self._log_state("connected")
            return True
        except serial.SerialException as exc:
            log.warning("Open %s failed: %s", port, exc)
            with self._state_lock:
                self._current_port = None
            return False

    def _close(self) -> None:
        with self._state_lock:
            ser = self._ser
            self._ser = None
        if ser is None:
            return
        try:
            ser.close()
        except Exception:
            pass
        self._log_state("disconnected")

    def _maybe_reconnect(self) -> None:
        now = _ms()
        if now - self._last_reconnect_ms < self.reconnect_interval_ms:
            return
        self._last_reconnect_ms = now
        self._try_open()

    def _on_frame(self, protocol_id: int, payload: bytes) -> None:
        self.router.dispatch(protocol_id, payload)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            if not self.is_open():
                self._maybe_reconnect()
                time.sleep(0.05)
                continue

            try:
                ser = self._ser
                if ser is None:
                    continue
                waiting = ser.in_waiting
                if waiting > 0:
                    data = ser.read(waiting)
                else:
                    # 短阻塞读, 让出 CPU
                    data = ser.read(1)
                if data:
                    self._last_rx_ms = _ms()
                    self._parser.feed(data)
            except (serial.SerialException, OSError) as exc:
                log.warning("UART read error: %s", exc)
                self._close()
                continue
            except Exception as exc:
                log.exception("RX loop error: %s", exc)
                time.sleep(0.05)
                continue

            # MCU 长时间无回 -> 主动重连
            if self.is_open() and (_ms() - self._last_rx_ms) > self.mcu_dead_timeout_ms:
                log.warning("MCU silent > %d ms, forcing reconnect", self.mcu_dead_timeout_ms)
                self._close()

            # MCU 空闲 -> 状态提示 (不刷屏)
            if self.is_open():
                if (_ms() - self._last_rx_ms) > self.mcu_idle_timeout_ms:
                    self._log_state("waiting")
                else:
                    self._log_state("connected")

    def _log_state(self, state: str) -> None:
        if state == self._last_state_log:
            return
        self._last_state_log = state
        if state == "connected":
            log.info("[serial] connected (%s)", self._current_port)
        elif state == "waiting":
            log.warning("[serial] MCU silent (waiting)")
        elif state == "disconnected":
            log.error("[serial] disconnected")
