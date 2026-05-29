"""Xbox 手柄 -> PC_Msg (14B) -> 0xFF 帧。

字节级兼容 Circle_Leg_Host_V1.0/main.py:
- 摇杆: angle*10 (0..3600, -1=居中), magnitude*1000 (0..1000)
- 扳机: 0..1000
- button_status bit: LB=0 RB=1 X=2 A=3 B=4 Y=5 ML=6 MR=7
- dpad_status bit:   UP=0 DOWN=1 LEFT=2 RIGHT=3
- 帧: build_frame(0xFF, payload14)
"""

import logging
import math
import platform
import threading
import time
from typing import Optional

import pygame

from ..comm.serial_link import SerialLink
from ..protocol.frame import build_frame
from ..protocol.ids import PROTO_PC_MSG
from ..protocol.messages import pack_pc_msg


log = logging.getLogger(__name__)


def _build_mapping():
    """复用 V1.0 中按 OS 区分的映射表。"""
    os_name = platform.system()
    if os_name == "Windows":
        logic_map = {0: 0, 1: 1, 2: 2, 3: 3, 6: 4, 7: 5, 11: 6}
        btn_ids = {"A": 0, "B": 1, "X": 2, "Y": 3,
                   "LB": 4, "RB": 5, "ML": 6, "MR": 7, "EXIT": 8}
        axis_ids = {"LT": 4, "RT": 5, "LX": 0, "LY": 1, "RX": 2, "RY": 3}
    else:
        # Linux (xpadneo / SDL 标准布局)
        logic_map = {0: 0, 1: 1, 2: 2, 3: 3, 6: 4, 7: 5}
        btn_ids = {"A": 0, "B": 1, "X": 2, "Y": 3,
                   "LB": 4, "RB": 5, "ML": 6, "MR": 7, "EXIT": -1}
        axis_ids = {"LT": 2, "RT": 5, "LX": 0, "LY": 1, "RX": 3, "RY": 4}
    return logic_map, btn_ids, axis_ids


# D-pad 在 SDL/Linux 上的常见暴露方式 (xpadneo / 内核 xpad 都见过)
#   1) hat(0)              ← 经典
#   2) axis 6 (X), 7 (Y)   ← xpadneo 默认
#   3) button 11 UP/12 DOWN/13 LEFT/14 RIGHT  ← 某些固件
# 同时探测三种来源, 按位 OR, 保证任何一种都能用。
DPAD_AXIS_X_DEFAULT = 6
DPAD_AXIS_Y_DEFAULT = 7
DPAD_BTN_DEFAULTS = {"UP": 11, "DOWN": 12, "LEFT": 13, "RIGHT": 14}
DPAD_AXIS_THRESHOLD = 0.5
# 若某 axis 空闲就 > 此值 (例: 扳机被错配成 dpad), 判定该 axis 不是真 dpad, 自动忽略
DPAD_AXIS_IDLE_GUARD = 0.4


def read_dpad_bits(js, dpad_axis_x: int = DPAD_AXIS_X_DEFAULT,
                   dpad_axis_y: int = DPAD_AXIS_Y_DEFAULT,
                   dpad_btn_ids: dict = None,
                   axis_baseline: dict = None,
                   sources: tuple = ("hat", "axis", "button")) -> int:
    """从 hat / axis / button 三种来源合并 D-pad 状态。

    axis_baseline: {axis_idx: idle_value}, 若 |当前-基线| 未越过阈值则不计;
                   None 时按"原始值靠近 0"判定。
    sources: 可选关闭某些来源, 用于调试隔离脏源。
    """
    if dpad_btn_ids is None:
        dpad_btn_ids = DPAD_BTN_DEFAULTS

    bits = 0
    # (1) hat
    if "hat" in sources:
        try:
            if js.get_numhats() > 0:
                hx, hy = js.get_hat(0)
                if hy > 0: bits |= (1 << 0)  # UP
                if hy < 0: bits |= (1 << 1)  # DOWN
                if hx < 0: bits |= (1 << 2)  # LEFT
                if hx > 0: bits |= (1 << 3)  # RIGHT
        except Exception:
            pass
    # (2) axis (xpadneo 风格)
    if "axis" in sources:
        try:
            n_ax = js.get_numaxes()

            def _axis_signed(idx):
                """返回相对基线的偏移; 若该 axis 空闲就严重偏置则视作非 dpad 轴, 返回 0。"""
                if idx is None or not (0 <= idx < n_ax):
                    return 0.0
                raw = js.get_axis(idx)
                base = (axis_baseline or {}).get(idx)
                if base is None:
                    # 无校准: 若疑似扳机(常驻 -1) 则忽略
                    if abs(raw) > DPAD_AXIS_IDLE_GUARD and not (-0.9 < raw < 0.9):
                        # 极端值 (-1 / +1) 仍可能是真的方向按下, 这里宽松一点: 仅当 ~ -1 时疑似扳机
                        if raw < -0.9:
                            return 0.0
                    return raw
                # 有校准: 基线归零
                if abs(base) > 0.5:
                    # 校准时就偏离很远 -> 不是 dpad 轴
                    return 0.0
                return raw - base

            ay = _axis_signed(dpad_axis_y)
            if ay < -DPAD_AXIS_THRESHOLD: bits |= (1 << 0)  # UP
            if ay >  DPAD_AXIS_THRESHOLD: bits |= (1 << 1)  # DOWN
            ax = _axis_signed(dpad_axis_x)
            if ax < -DPAD_AXIS_THRESHOLD: bits |= (1 << 2)  # LEFT
            if ax >  DPAD_AXIS_THRESHOLD: bits |= (1 << 3)  # RIGHT
        except Exception:
            pass
    # (3) buttons
    if "button" in sources:
        try:
            n_btn = js.get_numbuttons()
            for name, bit in (("UP", 0), ("DOWN", 1), ("LEFT", 2), ("RIGHT", 3)):
                bid = dpad_btn_ids.get(name, -1)
                if bid is not None and 0 <= bid < n_btn and js.get_button(bid):
                    bits |= (1 << bit)
        except Exception:
            pass
    return bits


def calibrate_axis_baseline(js, samples: int = 10) -> dict:
    """采集若干次 axis 值取均值, 作为空闲基线。调用前确保手柄静止。"""
    import time as _t
    n = js.get_numaxes()
    acc = [0.0] * n
    for _ in range(max(1, samples)):
        try:
            pygame.event.pump()
        except Exception:
            pass
        for i in range(n):
            try:
                acc[i] += js.get_axis(i)
            except Exception:
                pass
        _t.sleep(0.02)
    return {i: acc[i] / samples for i in range(n)}


class GamepadWorker:
    """以固定频率读取手柄并通过 SerialLink 发送 0xFF 帧。后台线程。"""

    def __init__(
        self,
        link: SerialLink,
        send_rate_hz: int = 30,
        deadzone: float = 0.15,
        output_mode: int = 1,
        dpad_axis_x: int = DPAD_AXIS_X_DEFAULT,
        dpad_axis_y: int = DPAD_AXIS_Y_DEFAULT,
        dpad_btn_ids: dict = None,
    ):
        self.link = link
        self.send_rate_hz = max(1, int(send_rate_hz))
        self.deadzone = float(deadzone)
        self.output_mode = int(output_mode)
        self.dpad_axis_x = dpad_axis_x
        self.dpad_axis_y = dpad_axis_y
        self.dpad_btn_ids = dpad_btn_ids if dpad_btn_ids is not None else DPAD_BTN_DEFAULTS

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._js: Optional[pygame.joystick.Joystick] = None
        self._axis_baseline: dict = {}

    # ---------------- public ----------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        pygame.init()
        pygame.joystick.init()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gamepad", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            pygame.quit()
        except Exception:
            pass

    # ---------------- internal ----------------

    def _wait_joystick(self) -> bool:
        log.info("Scanning for gamepad...")
        while not self._stop.is_set():
            try:
                pygame.event.pump()
            except Exception:
                pass
            if pygame.joystick.get_count() > 0:
                self._js = pygame.joystick.Joystick(0)
                self._js.init()
                log.info("Gamepad connected: %s", self._js.get_name())
                # 静止基线校准 (假设玩家此刻未触碰摇杆/扳机/方向键)
                self._axis_baseline = calibrate_axis_baseline(self._js, samples=10)
                log.info("Axis idle baseline: %s",
                         {i: round(v, 3) for i, v in self._axis_baseline.items()})
                return True
            time.sleep(0.5)
        return False

    def _run(self) -> None:
        logic_map, btn_ids, axis_ids = _build_mapping()
        period = 1.0 / self.send_rate_hz

        while not self._stop.is_set():
            if not self._wait_joystick():
                return

            js = self._js
            assert js is not None
            try:
                while not self._stop.is_set():
                    t0 = time.monotonic()
                    pygame.event.pump()

                    # 检测手柄断开
                    if pygame.joystick.get_count() == 0:
                        log.warning("Gamepad removed, rescanning...")
                        break

                    payload = self._build_payload(js, logic_map, btn_ids, axis_ids)
                    if payload is not None:
                        frame = build_frame(PROTO_PC_MSG, payload)
                        # 仅当 MCU 活跃时才发送 (与 V1.0 mcu_timeout 行为一致)
                        if self.link.is_mcu_alive():
                            self.link.send_frame(frame)

                    elapsed = time.monotonic() - t0
                    sleep_for = period - elapsed
                    if sleep_for > 0:
                        time.sleep(sleep_for)
            except Exception as exc:
                log.exception("Gamepad loop error: %s", exc)
                time.sleep(0.2)

    def _build_payload(self, js, logic_map, btn_ids, axis_ids) -> Optional[bytes]:
        try:
            # 左摇杆
            x = js.get_axis(axis_ids.get("LX", 0))
            y = js.get_axis(axis_ids.get("LY", 1))
            if abs(x) < self.deadzone:
                x = 0.0
            if abs(y) < self.deadzone:
                y = 0.0
            if x == 0.0 and y == 0.0:
                LA = -1
                LM = 0
            else:
                mag = min(1.0, math.sqrt(x * x + y * y))
                ang = math.degrees(math.atan2(-y, x))
                if ang < 0:
                    ang += 360
                LA = int(round(ang * 10))
                LM = int(round(mag * 100))

            # 右摇杆
            rx = js.get_axis(axis_ids.get("RX", 2))
            ry = js.get_axis(axis_ids.get("RY", 3))
            if abs(rx) < self.deadzone:
                rx = 0.0
            if abs(ry) < self.deadzone:
                ry = 0.0
            if rx == 0.0 and ry == 0.0:
                RA = -1
                RM = 0
            else:
                mag = min(1.0, math.sqrt(rx * rx + ry * ry))
                ang = math.degrees(math.atan2(-ry, rx))
                if ang < 0:
                    ang += 360
                RA = int(round(ang * 10))
                RM = int(round(mag * 100))

            # 扳机 (Pygame 初始 -1.0, 按下 +1.0 -> 归一到 0..1)
            lt_raw = js.get_axis(axis_ids["LT"])
            rt_raw = js.get_axis(axis_ids["RT"])
            lt_norm = max(0.0, min(1.0, (lt_raw + 1) / 2.0))
            rt_norm = max(0.0, min(1.0, (rt_raw + 1) / 2.0))
            LT = int(round(lt_norm * 1000))
            RT = int(round(rt_norm * 1000))

            # 半径换算 (与 V1.0 一致: LM*10)
            left_r = max(0, min(1000, LM * 10))
            right_r = max(0, min(1000, RM * 10))
            LT_val = max(0, min(1000, LT))
            RT_val = max(0, min(1000, RT))

            # 按键 bitmask
            button_status = 0
            mapping = [
                ("LB", 0), ("RB", 1), ("X", 2), ("A", 3),
                ("B", 4), ("Y", 5), ("ML", 6), ("MR", 7),
            ]
            for name, bit in mapping:
                bid = btn_ids.get(name)
                if bid is None:
                    continue
                if bid < js.get_numbuttons() and js.get_button(bid):
                    button_status |= (1 << bit)

            # dpad: 同时探测 hat / axis / button (Linux xpadneo 兼容)
            dpad_status = read_dpad_bits(
                js,
                dpad_axis_x=self.dpad_axis_x,
                dpad_axis_y=self.dpad_axis_y,
                dpad_btn_ids=self.dpad_btn_ids,
                axis_baseline=self._axis_baseline,
            )

            return pack_pc_msg(LA, left_r, RA, right_r, LT_val, RT_val, button_status, dpad_status)
        except Exception as exc:
            log.warning("Build payload error: %s", exc)
            return None
