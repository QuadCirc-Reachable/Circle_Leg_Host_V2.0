"""程序入口。

用法:
    python -m circle_leg_host.app.main [--config path/to/config.yaml]

视觉子系统仅在 config 中 vision.enabled=true 时才被 import 和启动。
"""

import argparse
import logging
import os
import signal
import sys
import time
from typing import Any, Dict

import yaml

from ..comm.router import FrameRouter
from ..comm.serial_link import SerialLink
from ..controller.gamepad import GamepadWorker


def _default_config_path() -> str:
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "config", "default.yaml")
    )


def _load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=_default_config_path(), help="YAML config path")
    parser.add_argument("--vision", action="store_true",
                        help="Override: force-enable vision (equivalent to vision.enabled=true)")
    parser.add_argument("--no-vision", action="store_true",
                        help="Override: force-disable vision regardless of config")
    parser.add_argument("--vision-window", action="store_true",
                        help="Show vision debug window (overrides vision.show_window=true)")
    parser.add_argument("--no-vision-window", action="store_true",
                        help="Hide vision debug window (overrides vision.show_window=false)")
    args = parser.parse_args(argv)

    _setup_logging()
    log = logging.getLogger("main")

    cfg = _load_config(args.config)
    serial_cfg = cfg.get("serial", {})
    gamepad_cfg = cfg.get("gamepad", {})
    vision_cfg = cfg.get("vision", {})

    vision_enabled = bool(vision_cfg.get("enabled", False))
    if args.vision:
        vision_enabled = True
    if args.no_vision:
        vision_enabled = False

    vision_show_window = bool(vision_cfg.get("show_window", False))
    if args.vision_window:
        vision_show_window = True
    if args.no_vision_window:
        vision_show_window = False

    router = FrameRouter()
    link = SerialLink(
        router=router,
        preferred_port=serial_cfg.get("preferred_port", "/dev/ttyACM0"),
        baudrate=int(serial_cfg.get("baudrate", 2000000)),
        reconnect_interval_ms=int(serial_cfg.get("reconnect_interval_ms", 2000)),
        mcu_idle_timeout_ms=int(serial_cfg.get("mcu_idle_timeout_ms", 1000)),
        mcu_dead_timeout_ms=int(serial_cfg.get("mcu_dead_timeout_ms", 8000)),
    )
    link.start()

    gamepad: GamepadWorker = None  # type: ignore[assignment]
    if gamepad_cfg.get("enabled", True):
        if gamepad_cfg.get("headless", True):
            # 让 pygame 不依赖显示器
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        gamepad = GamepadWorker(
            link=link,
            send_rate_hz=int(gamepad_cfg.get("send_rate_hz", 30)),
            deadzone=float(gamepad_cfg.get("deadzone", 0.15)),
            output_mode=int(gamepad_cfg.get("output_mode", 1)),
            dpad_axis_x=gamepad_cfg.get("dpad_axis_x", 6),
            dpad_axis_y=gamepad_cfg.get("dpad_axis_y", 7),
            dpad_btn_ids=gamepad_cfg.get("dpad_btn_ids",
                                         {"UP": 11, "DOWN": 12, "LEFT": 13, "RIGHT": 14}),
        )
        gamepad.start()
        log.info("Gamepad subsystem started")
    else:
        log.info("Gamepad disabled by config")

    vision_worker = None
    vision_orch = None
    if vision_enabled:
        # ★ 严格懒导入: 这两个模块在关闭视觉时不会被加载 ★
        from ..vision.state import VisionState
        from ..vision.curb_worker import CurbWorker
        from .vision_orchestrator import VisionOrchestrator

        vstate = VisionState(
            default_enabled=bool(vision_cfg.get("default_enabled", True)),
            default_min_height_mm=int(vision_cfg.get("min_height_mm", 20)),
        )
        vision_worker = CurbWorker(
            state=vstate,
            realsense_yaml=vision_cfg.get("realsense_yaml"),
            curbsvm1_path=vision_cfg.get("curbsvm1_path"),
            camera_to_front_edge_m=float(vision_cfg.get("camera_to_front_edge_m", 0.30)),
            k_confirm=int(vision_cfg.get("k_confirm", 3)),
            alpha=float(vision_cfg.get("alpha", 0.3)),
            show_window=vision_show_window,
        )
        vision_orch = VisionOrchestrator(
            link=link,
            router=router,
            state=vstate,
            publish_period_ms=int(vision_cfg.get("publish_period_ms", 50)),
            ctrl_timeout_ms=int(vision_cfg.get("ctrl_timeout_ms", 1500)),
            control_required=bool(vision_cfg.get("control_required", False)),
            default_enabled=bool(vision_cfg.get("default_enabled", True)),
        )
        vision_worker.start()
        vision_orch.start()
        log.info("Vision subsystem started (enabled=%s, control_required=%s, show_window=%s)",
                 vision_cfg.get("default_enabled", True),
                 vision_cfg.get("control_required", False),
                 vision_show_window)
    else:
        log.info("Vision subsystem disabled (no vision modules imported)")

    stop = False

    def _on_sig(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    try:
        if vision_worker and vision_show_window:
            import cv2  # 在主线程 lazy import; 不开窗口时不涉及
            cv2.namedWindow("RealSense Color/Depth (Aligned)", cv2.WINDOW_AUTOSIZE)
            log.info("Vision window opened (press q to quit)")
            while not stop:
                img = vision_worker.pop_debug_image()
                if img is not None:
                    cv2.imshow("RealSense Color/Depth (Aligned)", img)
                key = cv2.waitKey(15) & 0xFF
                if key == ord("q"):
                    stop = True
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        else:
            while not stop:
                time.sleep(0.2)
    finally:
        log.info("Shutting down...")
        if vision_orch:
            vision_orch.stop()
        if vision_worker:
            vision_worker.stop()
        if gamepad:
            gamepad.stop()
        link.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
