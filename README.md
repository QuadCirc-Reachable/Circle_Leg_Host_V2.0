# Circle_Leg_Host_V2 — REACHABLE Host Software (Gamepad Link + Curb Vision)

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/pygame-2.5%2B-green" alt="pygame">
  <img src="https://img.shields.io/badge/pyserial-3.5-lightgrey" alt="pyserial">
  <img src="https://img.shields.io/badge/Vision-RealSense%20D435%20(optional)-0071C5?logo=intel&logoColor=white" alt="RealSense">
  <img src="https://img.shields.io/badge/tests-pytest-0A9EDC?logo=pytest&logoColor=white" alt="pytest">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License">
</p>

<p align="center">
  <b>English</b> | <a href="README_zh.md">中文</a>
</p>

Host-side software for the **REACHABLE (QuadCirc)** CircLeg wheel-leg wheelchair, running on the
vehicle's Jetson Orin Nano. It streams the operator's Xbox gamepad to the STM32 chassis controller
and can optionally run the **RealSense curb-detection pipeline**, sending curb height, distance and
angle to the MCU. HKUST Final Year Design Project SL05a-25.

---

## Overview

Version 2.0 restructures the single-script [Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0)
into a tested Python package, and adds a vision link.

- **Drop-in gamepad link**: byte-for-byte compatible with V1.0. Same `0xFF` frame, 14-byte
  `PC_Msg` and double CRC16, sent at 30 Hz. With vision off, the behaviour is identical to V1.0.
- **Shared serial link**: one UART, one RX thread, and a thread-safe router that dispatches frames by
  protocol ID. It auto-selects the port, reconnects every 2 s, pauses TX after 1 s without MCU
  traffic and forces a reconnect after 8 s.
- **Optional curb vision**: RealSense D435 → `curbsvm1` plane extraction + SVM boundary → curb
  height / distance / angle / confidence. Detection needs K consecutive reliable frames, height is
  low-pass filtered, and the result goes to the MCU as `0xFC` at 20 Hz. The MCU can enable, disable
  or tune it with a `0xFD` heartbeat. Vision dependencies are imported only when vision is enabled.
- **Robust gamepad input**: per-OS mapping, hot-plug, and on Linux D-pad detection that merges hat,
  axis and button sources with automatic baseline calibration.
- **Configurable**: one YAML file plus CLI overrides; Ctrl+C shuts down cleanly.

| Repository | Content |
|------------|---------|
| [Circle_Leg_V2.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0) | Full-size firmware (consumes the `0xFF` gamepad link) |
| [Circle_Leg_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V1.0) | Half-size firmware |
| [Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0) | Previous single-script host (gamepad only) |
| **Circle_Leg_Host_V2.0** (this repo) | Host package: gamepad link + optional curb vision |

---

## Architecture

```
                    config/default.yaml  +  CLI flags  →  app/main.py
                                              │
 ┌──────────────┐ pygame  ┌───────────────────▼──────────────────────────────────────────┐
 │ Xbox gamepad │ ──────► │ GamepadWorker ── PC_Msg (14 B) ── frame 0xFF ── 30 Hz ──┐      │
 └──────────────┘         │                                                           │      │
 ┌──────────────┐ USB 3   │ CurbWorker ─► VisionState ─► VisionOrchestrator           │      │
 │ RealSense    │ ──────► │ (curbsvm1:      ▲             ├─ frame 0xFC ── 20 Hz ─────┤      │
 │ D435         │         │  planes → SVM)  └─ 0xFD ctrl ─┘   (vision optional)       ▼      │
 └──────────────┘         │                        FrameRouter ◄─ RX thread ◄─ SerialLink ─ TX│
                          └──────────────────────────────────────────────────────┬───────┘
                                                                                  │ UART 2 Mbit/s
                                                                                  ▼
                                                             STM32G473 · Circle_Leg_V2 firmware
```

---

## Getting Started

```bash
git clone https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V2.0.git
cd Circle_Leg_Host_V2.0
pip install -r requirements.txt       # pygame, pyserial, pyyaml, numpy

# ① Gamepad only (same behaviour as Host V1.0, the usual mode)
PYTHONPATH=src python -m circle_leg_host.app.main

# ② Gamepad + vision, headless (deployment)
PYTHONPATH=src python -m circle_leg_host.app.main --vision

# ③ Gamepad + vision + OpenCV debug window (check the camera / algorithm)
PYTHONPATH=src python -m circle_leg_host.app.main --vision --vision-window

# ④ Gamepad mapping debugger (local only, no serial port)
PYTHONPATH=src python tools/gamepad_debug.py
```

Alternatively, `pip install -e .` makes the package importable without `PYTHONPATH`. On Windows
PowerShell, set it with `$env:PYTHONPATH = "src"`. On Linux, add your user to the `dialout` group
for serial access.

| Flag | Meaning |
|------|---------|
| `--config PATH` | YAML config (default `config/default.yaml`) |
| `--vision` / `--no-vision` | Force vision on / off (overrides `vision.enabled`) |
| `--vision-window` / `--no-vision-window` | Show / hide the vision debug window |

### Vision setup

Vision wraps the team's `curbsvm1` perception package (RealSense point cloud → Polylidar3D plane
extraction → SVM curb boundary), which is **not part of this repository**:

1. Place `curbsvm1_package/` next to this repository (`<workspace>/curbsvm1_package`), or point
   `vision.curbsvm1_path` at it.
2. Install the extra dependencies: `pip install -e .[vision]`, plus `polylidar` and `fastgac`.
3. Mount the D435 and set `vision.camera_to_front_edge_m`, the distance from the camera to the
   front edge of the chair.

---

## Configuration

`config/default.yaml`:

| Key | Default | Meaning |
|-----|---------|---------|
| `serial.preferred_port` | `/dev/ttyACM0` | Preferred port; falls back to auto-detection |
| `serial.baudrate` | `2000000` | Must match the firmware's USART2 |
| `serial.reconnect_interval_ms` | `2000` | Reconnect throttle |
| `serial.mcu_idle_timeout_ms` / `mcu_dead_timeout_ms` | `1000` / `8000` | Pause TX / force reconnect without MCU traffic |
| `gamepad.headless` | `true` | No pygame window |
| `gamepad.send_rate_hz` | `30` | Gamepad frame rate |
| `gamepad.deadzone` | `0.15` | Stick deadzone |
| `gamepad.dpad_axis_x/y`, `dpad_btn_ids` | `6/7`, `11–14` | Extra Linux D-pad sources (besides the hat) |
| `vision.enabled` | `false` | Master switch; `false` means no vision imports at all |
| `vision.show_window` | `false` | OpenCV debug window |
| `vision.control_required` | `false` | `true`: run only while the MCU sends `0xFD` heartbeats |
| `vision.default_enabled` | `true` | Start in detecting state |
| `vision.min_height_mm` | `20` | Minimum curb height to report |
| `vision.publish_period_ms` | `50` | `0xFC` period (20 Hz) |
| `vision.ctrl_timeout_ms` | `1500` | `0xFD` heartbeat timeout → stop detecting |
| `vision.k_confirm` / `alpha` | `3` / `0.3` | Frames to confirm a detection / height low-pass factor |
| `vision.camera_to_front_edge_m` | `0.30` | Camera → chair front edge offset |

Optional keys read by the code: `vision.curbsvm1_path` and `vision.realsense_yaml`. By default
`realsense_yaml` uses `curbsvm1_package/surfacedetector/config/default.yaml`.

---

## Communication Protocol

Every frame: `| SOF 0xAA | LEN | ID | CRC16(header) | payload | CRC16(frame) |`. It is the
firmware's RosComm framing, identical to V1.0.

| ID | Direction | Payload | Content |
|----|-----------|---------|---------|
| `0xFF` | PC → MCU | 14 B `PC_Msg` | Sticks (angle × 10, radius × 1000), triggers, button and D-pad bits |
| `0xFE` | MCU → PC | 40 B `Reachable_Msg` | Motor feedback (decoded for debugging) |
| `0xFD` | MCU → PC | 4 B `VisionCtrl` | `enable` (u8), `min_height_mm` (u16), `flags` (u8: bit0 reset, bit1 verbose) |
| `0xFC` | PC → MCU | 10 B `VisionResult` | `seq`, `status`, `curb_height_mm`, `distance_mm`, `theta_cdeg` (deg × 100), `confidence`, `flags` |

`VisionResult.status` has these bits: 0 detected, 1 reliable, 2 disabled, 3 fault. `flags` has:
0 theta invalid, 1 distance invalid, 2 no planes, 3 stale (> 500 ms).

> **Firmware status:** the archived Circle_Leg_V2 firmware consumes the `0xFF` gamepad link only.
> Firmware without vision support is unaffected by the vision link. The minimal MCU-side changes to
> consume `0xFC` and send `0xFD` (`ENABLE_VISION_COMM`, `Vision_Comm.hpp/.cpp`) are written up in
> [docs/MCU_Integration_Guide.md](docs/MCU_Integration_Guide.md).

---

## Testing

```bash
python -m pytest tests/ -q
```

| Test | Covers |
|------|--------|
| `test_crc16.py` | CRC16 table and function, golden comparison against Host V1.0 (skipped if no V1.0 checkout sits next to this repo) |
| `test_frame.py` | Frame build / parse, byte-level equivalence with V1.0, robustness to noise and split packets |
| `test_vision_orchestrator.py` | `0xFD` routing, periodic `0xFC` publishing, heartbeat watchdog |
| `test_serial_link.py`, `test_e2e_loopback.py` | Serial open / reconnect and full end-to-end loop over a pseudo-terminal (Linux / macOS only) |

---

## Repository Structure

```
Circle_Leg_Host_V2.0/
├── config/default.yaml             # Serial, gamepad and vision settings
├── src/circle_leg_host/
│   ├── app/
│   │   ├── main.py                 # Entry point, CLI, thread start-up / shutdown
│   │   └── vision_orchestrator.py  # 0xFD → VisionState, periodic 0xFC publisher
│   ├── comm/
│   │   ├── serial_link.py          # Shared UART: RX thread, locked TX, reconnect, MCU liveness
│   │   └── router.py               # Protocol-ID → callback dispatch
│   ├── controller/gamepad.py       # Xbox gamepad → PC_Msg → 0xFF frames
│   ├── protocol/                   # crc16.py, frame.py, ids.py, messages.py
│   ├── utils/port_select.py        # Serial port auto-selection
│   └── vision/                     # curb_pipeline.py, curb_worker.py, state.py (lazy-loaded)
├── tools/gamepad_debug.py          # Visual gamepad mapping debugger
├── tests/                          # pytest suite
├── docs/MCU_Integration_Guide.md   # MCU-side changes for the vision link
├── requirements.txt, setup.py
└── LICENSE
```

---

## Team

REACHABLE (QuadCirc), HKUST FYP SL05a-25: LIU Hualin (embedded control lead), FANG Ruoyun
(perception & HMI), WU Ziyao (mechanical architecture & communication), XU Jusen (mechanical lead).

## License

[MIT](LICENSE). The external `curbsvm1_package` and its dependencies keep their own licenses.
