# Circle_Leg_Host_V2 — REACHABLE 上位机软件（手柄链路 + 台阶视觉）

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/pygame-2.5%2B-green" alt="pygame">
  <img src="https://img.shields.io/badge/pyserial-3.5-lightgrey" alt="pyserial">
  <img src="https://img.shields.io/badge/Vision-RealSense%20D435%20(optional)-0071C5?logo=intel&logoColor=white" alt="RealSense">
  <img src="https://img.shields.io/badge/tests-pytest-0A9EDC?logo=pytest&logoColor=white" alt="pytest">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License">
</p>

<p align="center">
  <a href="README.md">English</a> | <b>中文</b>
</p>

**REACHABLE（QuadCirc）** CircLeg 轮腿轮椅的上位机软件，运行在车载 Jetson Orin Nano 上：把操作者的
Xbox 手柄实时发送给 STM32 底盘控制器，并可选运行 **RealSense 台阶识别**，把台阶高度、距离和角度发给
MCU。香港科技大学毕业设计项目 SL05a-25。

---

## 项目概述

2.0 版把单文件的 [Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0)
重构为带测试的 Python 包，并新增了视觉链路。

- **手柄链路即插即用**：与 V1.0 字节级兼容——同样的 `0xFF` 帧、14 字节 `PC_Msg`、双 CRC16、30 Hz 发送。
  关闭视觉时，行为与 V1.0 完全一致。
- **共享串口链路**：单串口、单 RX 线程，加一个线程安全的路由，按协议 ID 分发帧。自动选口、每 2 s 重连；
  MCU 超过 1 s 无数据则暂停发送，超过 8 s 强制重连。
- **可选台阶视觉**：RealSense D435 → `curbsvm1` 平面提取 + SVM 边界 → 台阶高度 / 距离 / 角度 / 置信度。
  需连续 K 帧可靠才判定为检测到，高度经低通滤波，结果以 `0xFC` 20 Hz 发给 MCU；MCU 可用 `0xFD` 心跳
  启停或调阈值。只有开启视觉时才会导入视觉依赖。
- **手柄输入稳健**：按系统映射、支持热插拔；Linux 下 D-pad 同时合并 hat、axis、button 三路来源，
  并自动基线校准。
- **可配置**：一个 YAML 配置文件加命令行覆盖；Ctrl+C 可优雅退出。

| 仓库 | 内容 |
|------|------|
| [Circle_Leg_V2.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V2.0) | 全尺寸固件（使用 `0xFF` 手柄链路） |
| [Circle_Leg_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_V1.0) | 半尺寸固件 |
| [Circle_Leg_Host_V1.0](https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V1.0) | 旧版单文件上位机（仅手柄） |
| **Circle_Leg_Host_V2.0**（本仓库） | 上位机包：手柄链路 + 可选台阶视觉 |

---

## 架构

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

## 快速开始

```bash
git clone https://github.com/QuadCirc-Reachable/Circle_Leg_Host_V2.0.git
cd Circle_Leg_Host_V2.0
pip install -r requirements.txt       # pygame, pyserial, pyyaml, numpy

# ① 只跑手柄（等价 Host V1.0 的行为，最常用）
PYTHONPATH=src python -m circle_leg_host.app.main

# ② 手柄 + 视觉，无窗口（部署模式）
PYTHONPATH=src python -m circle_leg_host.app.main --vision

# ③ 手柄 + 视觉 + OpenCV 调试窗口（确认相机 / 算法是否正常）
PYTHONPATH=src python -m circle_leg_host.app.main --vision --vision-window

# ④ 手柄按键映射调试（纯本地，不开串口）
PYTHONPATH=src python tools/gamepad_debug.py
```

也可以执行 `pip install -e .`，之后无需设置 `PYTHONPATH`。Windows PowerShell 下用
`$env:PYTHONPATH = "src"`。Linux 上需要把用户加入 `dialout` 组才能访问串口。

| 命令行参数 | 含义 |
|------------|------|
| `--config PATH` | 指定 YAML 配置（默认 `config/default.yaml`） |
| `--vision` / `--no-vision` | 强制开启 / 关闭视觉（覆盖 `vision.enabled`） |
| `--vision-window` / `--no-vision-window` | 显示 / 隐藏视觉调试窗口 |

### 视觉环境

视觉部分是对团队 `curbsvm1` 感知包的封装（RealSense 点云 → Polylidar3D 平面提取 → SVM 台阶边界），
该包 **不包含在本仓库中**：

1. 把 `curbsvm1_package/` 放在本仓库旁边（`<workspace>/curbsvm1_package`），或用
   `vision.curbsvm1_path` 指向它。
2. 安装额外依赖：`pip install -e .[vision]`，另外还需要 `polylidar` 和 `fastgac`。
3. 装好 D435，并设置 `vision.camera_to_front_edge_m`，即相机到车身前缘的距离。

---

## 配置

`config/default.yaml`：

| 配置项 | 默认值 | 含义 |
|--------|--------|------|
| `serial.preferred_port` | `/dev/ttyACM0` | 优先使用的端口；不存在时回退到自动扫描 |
| `serial.baudrate` | `2000000` | 必须与固件的 USART2 一致 |
| `serial.reconnect_interval_ms` | `2000` | 重连间隔 |
| `serial.mcu_idle_timeout_ms` / `mcu_dead_timeout_ms` | `1000` / `8000` | MCU 无数据时暂停发送 / 强制重连 |
| `gamepad.headless` | `true` | 不开 pygame 窗口 |
| `gamepad.send_rate_hz` | `30` | 手柄帧率 |
| `gamepad.deadzone` | `0.15` | 摇杆死区 |
| `gamepad.dpad_axis_x/y`、`dpad_btn_ids` | `6/7`、`11–14` | Linux 下 D-pad 的额外来源（除 hat 外） |
| `vision.enabled` | `false` | 总开关；`false` 时完全不导入视觉依赖 |
| `vision.show_window` | `false` | OpenCV 调试窗口 |
| `vision.control_required` | `false` | `true`：仅在收到 MCU 的 `0xFD` 心跳时运行 |
| `vision.default_enabled` | `true` | 启动后默认处于识别状态 |
| `vision.min_height_mm` | `20` | 上报的最小台阶高度 |
| `vision.publish_period_ms` | `50` | `0xFC` 上报周期（20 Hz） |
| `vision.ctrl_timeout_ms` | `1500` | `0xFD` 心跳超时 → 停止识别 |
| `vision.k_confirm` / `alpha` | `3` / `0.3` | 确认检测所需帧数 / 高度低通系数 |
| `vision.camera_to_front_edge_m` | `0.30` | 相机到车身前缘的距离 |

代码还会读取两个可选项：`vision.curbsvm1_path` 和 `vision.realsense_yaml`。默认情况下
`realsense_yaml` 使用 `curbsvm1_package/surfacedetector/config/default.yaml`。

---

## 通信协议

每一帧的格式为 `| SOF 0xAA | LEN | ID | CRC16(header) | payload | CRC16(frame) |`，即固件的 RosComm
帧格式，与 V1.0 完全一致。

| ID | 方向 | 载荷 | 内容 |
|----|------|------|------|
| `0xFF` | PC → MCU | 14 B `PC_Msg` | 摇杆（角度 × 10、幅度 × 1000）、扳机、按键与 D-pad 位 |
| `0xFE` | MCU → PC | 40 B `Reachable_Msg` | 电机反馈（仅用于调试解包） |
| `0xFD` | MCU → PC | 4 B `VisionCtrl` | `enable`（u8）、`min_height_mm`（u16）、`flags`（u8：bit0 复位，bit1 详细日志） |
| `0xFC` | PC → MCU | 10 B `VisionResult` | `seq`、`status`、`curb_height_mm`、`distance_mm`、`theta_cdeg`（角度 × 100）、`confidence`、`flags` |

`VisionResult.status` 的位含义：0 检测到，1 可靠，2 已停用，3 故障。`flags` 的位含义：
0 角度无效，1 距离无效，2 未找到平面，3 数据过期（> 500 ms）。

> **固件现状**：本项目归档的 Circle_Leg_V2 固件只处理 `0xFF` 手柄链路。视觉链路不会影响不支持视觉的
> 固件。要让 MCU 接收 `0xFC` 并发送 `0xFD`（`ENABLE_VISION_COMM`、`Vision_Comm.hpp/.cpp`），
> 最小改动见 [docs/MCU_Integration_Guide.md](docs/MCU_Integration_Guide.md)。

---

## 测试

```bash
python -m pytest tests/ -q
```

| 测试 | 覆盖内容 |
|------|----------|
| `test_crc16.py` | CRC16 表与函数，并与 Host V1.0 做逐字节对照（旁边没有 V1.0 目录时自动跳过） |
| `test_frame.py` | 组帧 / 解析、与 V1.0 的字节级一致性，以及对噪声和分包的鲁棒性 |
| `test_vision_orchestrator.py` | `0xFD` 路由、`0xFC` 周期上报、心跳看门狗 |
| `test_serial_link.py`、`test_e2e_loopback.py` | 串口打开 / 重连，以及基于伪终端的端到端回环（仅 Linux / macOS） |

---

## 目录结构

```
Circle_Leg_Host_V2.0/
├── config/default.yaml             # 串口、手柄、视觉配置
├── src/circle_leg_host/
│   ├── app/
│   │   ├── main.py                 # 入口、命令行、线程启动与退出
│   │   └── vision_orchestrator.py  # 0xFD → VisionState，周期发送 0xFC
│   ├── comm/
│   │   ├── serial_link.py          # 共享串口：RX 线程、加锁 TX、重连、MCU 活跃检测
│   │   └── router.py               # 协议 ID → 回调分发
│   ├── controller/gamepad.py       # Xbox 手柄 → PC_Msg → 0xFF 帧
│   ├── protocol/                   # crc16.py、frame.py、ids.py、messages.py
│   ├── utils/port_select.py        # 串口自动选择
│   └── vision/                     # curb_pipeline.py、curb_worker.py、state.py（懒加载）
├── tools/gamepad_debug.py          # 手柄映射可视化调试工具
├── tests/                          # pytest 测试
├── docs/MCU_Integration_Guide.md   # 视觉链路的 MCU 侧改动指南
├── requirements.txt, setup.py
└── LICENSE
```

---

## 团队

REACHABLE（QuadCirc），香港科技大学毕业设计项目 SL05a-25：LIU Hualin（嵌入式控制负责人）、
FANG Ruoyun（感知与人机交互）、WU Ziyao（机械架构与通信）、XU Jusen（机械负责人）。

## 许可证

[MIT](LICENSE)。外部的 `curbsvm1_package` 及其依赖保留各自的许可证。
