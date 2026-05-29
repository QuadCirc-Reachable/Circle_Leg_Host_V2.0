# Circle_Leg_Host_V2.0

> 上位机：手柄遥控 (V1.0 协议) + 可选台阶视觉识别 (curbsvm1)。

---

## 🚀 启动命令（最常用，复制即用）

```bash
cd /home/rklb/workspace/Circle_Leg_Host_V2.0

# ① 只跑手柄 (= V1.0 行为，最常用)
PYTHONPATH=src python -m circle_leg_host.app.main

# ② 手柄 + 视觉 (无窗口，部署模式)
PYTHONPATH=src python -m circle_leg_host.app.main --vision

# ③ 手柄 + 视觉 + 弹出调试窗口 (确认相机/算法是否在跑)
PYTHONPATH=src python -m circle_leg_host.app.main --vision --vision-window

# ④ 手柄按键 mapping 调试 (不开串口，纯本地)
PYTHONPATH=src python tools/gamepad_debug.py
```

CLI 速查：

| flag                  | 含义                                       |
| --------------------- | ------------------------------------------ |
| `--config PATH`       | 指定 YAML 配置（默认 `config/default.yaml`）|
| `--vision`            | 强制开启视觉（覆盖 yaml）                  |
| `--no-vision`         | 强制关闭视觉                                |
| `--vision-window`     | 弹出视觉调试窗口                           |
| `--no-vision-window`  | 关闭视觉调试窗口                           |

Ctrl+C 优雅退出。

---

## 特性
- 协议字节级兼容 `Circle_Leg_Host_V1.0`（0xFF/PC_Msg/14B，双 CRC16，21B 帧）。
- 视觉功能通过 `vision.enabled` 开关整体启停；关闭时不 import 任何视觉依赖，行为等价 V1.0。
- 新增协议（仅启用视觉时生效，不影响老 MCU）：
  - `0xFD` MCU→PC：视觉控制（enable + 阈值）
  - `0xFC` PC→MCU：视觉结果（高度/距离/角度/置信度）
- D-pad 在 Linux 上同时探测 hat / axis / button 三路，自动基线校准。

---

## 目录
- `src/circle_leg_host/app/main.py` 入口
- `config/default.yaml` 配置
- `src/circle_leg_host/protocol/` 帧/CRC/消息
- `src/circle_leg_host/comm/` 串口收发与路由
- `src/circle_leg_host/controller/` 手柄
- `src/circle_leg_host/vision/` 视觉（懒加载）
- `src/circle_leg_host/app/` 编排与状态
- `tools/gamepad_debug.py` 手柄按键 mapping 可视化调试
- `docs/MCU_Integration_Guide.md` MCU 侧最小修改指南
- `tests/` 协议测试 (`python -m pytest tests/ -q`)

---

## 安装
```bash
pip install -r requirements.txt
# 视觉额外依赖 (仅当需要 --vision):
pip install pyrealsense2 polylidar open3d opencv-python shapely scikit-learn matplotlib scipy
```
