"""手柄调试 UI — Circle_Leg_Host_V2.0

用途:
  - 实时查看所有 axes / buttons / hats 的原始值, 用来确认 button mapping
  - 同时显示打包后的 PC_Msg 字段 (角度/半径/扳机/按钮位/dpad 位)
  - 同时显示 D-pad 三种来源 (hat / axis / button) 各自的探测结果
  - 不发送串口数据, 纯本地, 安全

用法:
  cd Circle_Leg_Host_V2.0
  PYTHONPATH=src python tools/gamepad_debug.py
"""

import math
import os
import sys
from pathlib import Path

# 允许 tools 目录直接运行
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import pygame  # noqa: E402

from circle_leg_host.controller.gamepad import (  # noqa: E402
    DPAD_AXIS_THRESHOLD,
    DPAD_AXIS_X_DEFAULT,
    DPAD_AXIS_Y_DEFAULT,
    DPAD_BTN_DEFAULTS,
    calibrate_axis_baseline,
    read_dpad_bits,
)


WIDTH, HEIGHT = 920, 720
BG = (18, 18, 22)
FG = (220, 220, 220)
DIM = (120, 120, 130)
HI = (90, 200, 120)
WARN = (240, 180, 80)
BAD = (220, 90, 90)


def _compute_stick(x: float, y: float, deadzone: float):
    if abs(x) < deadzone:
        x = 0.0
    if abs(y) < deadzone:
        y = 0.0
    if x == 0.0 and y == 0.0:
        return -1, 0
    mag = min(1.0, math.sqrt(x * x + y * y))
    ang = math.degrees(math.atan2(-y, x))
    if ang < 0:
        ang += 360
    return int(round(ang * 10)), int(round(mag * 100))


def main():
    # 确保有显示器才弹窗
    if not os.environ.get("DISPLAY") and sys.platform != "win32":
        print("WARN: no $DISPLAY detected. If you are on a headless box, run this on the desktop.")
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Gamepad Debug — Circle_Leg_Host_V2.0")
    font = pygame.font.SysFont("monospace", 16)
    big = pygame.font.SysFont("monospace", 20, bold=True)
    clock = pygame.time.Clock()

    def render_text(text, pos, color=FG, f=font):
        screen.blit(f.render(text, True, color), pos)

    deadzone = 0.15
    dpad_axis_x = DPAD_AXIS_X_DEFAULT
    dpad_axis_y = DPAD_AXIS_Y_DEFAULT
    dpad_btn_ids = dict(DPAD_BTN_DEFAULTS)

    js = None
    last_pressed_btn = -1
    last_moved_axis = (-1, 0.0)
    axis_baseline: dict = {}
    src_hat = True
    src_axis = True
    src_btn = True

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_1:
                src_hat = not src_hat
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_2:
                src_axis = not src_axis
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_3:
                src_btn = not src_btn
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_c and js is not None:
                axis_baseline = calibrate_axis_baseline(js, samples=10)
            elif ev.type == pygame.JOYBUTTONDOWN:
                last_pressed_btn = ev.button
            elif ev.type == pygame.JOYAXISMOTION:
                if abs(ev.value) > 0.5:
                    last_moved_axis = (ev.axis, ev.value)

        if js is None and pygame.joystick.get_count() > 0:
            js = pygame.joystick.Joystick(0)
            js.init()
            # 自动校准 (假设此刻手柄静止)
            axis_baseline = calibrate_axis_baseline(js, samples=10)

        screen.fill(BG)

        if js is None:
            render_text("No joystick detected. Plug in your Xbox controller.", (20, 20), WARN, big)
            pygame.display.flip()
            clock.tick(30)
            continue

        # ---------------- header ----------------
        render_text(f"Name : {js.get_name()}", (20, 12), FG, big)
        render_text(f"Axes : {js.get_numaxes()}    Buttons: {js.get_numbuttons()}    Hats: {js.get_numhats()}",
                    (20, 40), DIM)

        # ---------------- axes ----------------
        render_text("[ AXES ]", (20, 72), HI, big)
        for i in range(js.get_numaxes()):
            v = js.get_axis(i)
            bar = int((v + 1) / 2 * 200)
            color = HI if abs(v) > deadzone else DIM
            render_text(f"axis {i:2d}: {v:+.3f}", (20, 100 + i * 22), color)
            pygame.draw.rect(screen, DIM, (200, 100 + i * 22 + 4, 200, 10), 1)
            pygame.draw.rect(screen, color, (200, 100 + i * 22 + 4, bar, 10))

        # ---------------- buttons ----------------
        render_text("[ BUTTONS ]", (440, 72), HI, big)
        for i in range(js.get_numbuttons()):
            pressed = js.get_button(i)
            col = HI if pressed else DIM
            x = 440 + (i // 10) * 110
            y = 100 + (i % 10) * 22
            render_text(f"btn {i:2d}: {pressed}", (x, y), col)

        # ---------------- hats ----------------
        y_hat = 100 + max(js.get_numbuttons() % 10, js.get_numaxes()) * 22 + 30
        render_text("[ HATS ]", (440, y_hat), HI, big)
        for i in range(js.get_numhats()):
            hx, hy = js.get_hat(i)
            render_text(f"hat {i}: ({hx:+d}, {hy:+d})", (440, y_hat + 28 + i * 22), HI if (hx or hy) else DIM)

        # ---------------- D-pad sources ----------------
        y_dp = 340
        render_text("[ D-PAD source diagnosis ]", (20, y_dp), WARN, big)
        render_text("hotkeys: [1]=hat  [2]=axis  [3]=button  [C]=recalibrate baseline",
                    (20, y_dp - 22), DIM)

        # 用统一函数算每个单源
        hat_bits  = read_dpad_bits(js, dpad_axis_x, dpad_axis_y, dpad_btn_ids,
                                   axis_baseline=axis_baseline, sources=("hat",))
        axis_bits = read_dpad_bits(js, dpad_axis_x, dpad_axis_y, dpad_btn_ids,
                                   axis_baseline=axis_baseline, sources=("axis",))
        btn_bits  = read_dpad_bits(js, dpad_axis_x, dpad_axis_y, dpad_btn_ids,
                                   axis_baseline=axis_baseline, sources=("button",))

        active_srcs = []
        if src_hat:    active_srcs.append("hat")
        if src_axis:   active_srcs.append("axis")
        if src_btn:    active_srcs.append("button")
        merged = read_dpad_bits(js, dpad_axis_x, dpad_axis_y, dpad_btn_ids,
                                axis_baseline=axis_baseline, sources=tuple(active_srcs))

        def fmt(bits):
            return "U" * bool(bits & 1) + "D" * bool(bits & 2) + "L" * bool(bits & 4) + "R" * bool(bits & 8) or "-"

        # 显示每路当前 raw + 基线 + 是否启用
        ax_raw_x = js.get_axis(dpad_axis_x) if 0 <= dpad_axis_x < js.get_numaxes() else None
        ax_raw_y = js.get_axis(dpad_axis_y) if 0 <= dpad_axis_y < js.get_numaxes() else None
        base_x = axis_baseline.get(dpad_axis_x)
        base_y = axis_baseline.get(dpad_axis_y)

        def opt(v): return "n/a" if v is None else f"{v:+.2f}"

        render_text(f"[{'ON ' if src_hat else 'off'}] hat(hat0)                          -> {bin(hat_bits)[2:].zfill(4)} [{fmt(hat_bits)}]",
                    (20, y_dp + 28), HI if hat_bits else (FG if src_hat else DIM))
        render_text(f"[{'ON ' if src_axis else 'off'}] axis(X=axis{dpad_axis_x} raw={opt(ax_raw_x)} base={opt(base_x)} "
                    f"| Y=axis{dpad_axis_y} raw={opt(ax_raw_y)} base={opt(base_y)}) -> {bin(axis_bits)[2:].zfill(4)} [{fmt(axis_bits)}]",
                    (20, y_dp + 50), HI if axis_bits else (FG if src_axis else DIM))
        render_text(f"[{'ON ' if src_btn else 'off'}] button(UP={dpad_btn_ids['UP']} DOWN={dpad_btn_ids['DOWN']} "
                    f"LEFT={dpad_btn_ids['LEFT']} RIGHT={dpad_btn_ids['RIGHT']}) -> "
                    f"{bin(btn_bits)[2:].zfill(4)} [{fmt(btn_bits)}]",
                    (20, y_dp + 72), HI if btn_bits else (FG if src_btn else DIM))
        render_text(f"MERGED (final dpad_status)             -> {bin(merged)[2:].zfill(4)} [{fmt(merged)}]   "
                    f"= {merged}",
                    (20, y_dp + 100), WARN if merged else DIM, big)

        merged_via_lib = read_dpad_bits(js, dpad_axis_x, dpad_axis_y, dpad_btn_ids,
                                        axis_baseline=axis_baseline,
                                        sources=tuple(active_srcs))
        consistent = merged == merged_via_lib
        render_text(f"(sanity: read_dpad_bits()={merged_via_lib}  consistent={consistent})",
                    (20, y_dp + 128), HI if consistent else BAD)

        # ---------------- PC_Msg preview ----------------
        y_pm = 510
        render_text("[ PC_Msg preview (not sent) ]", (20, y_pm), WARN, big)

        def axval(idx, default=0.0):
            return js.get_axis(idx) if 0 <= idx < js.get_numaxes() else default

        LA, LM = _compute_stick(axval(0), axval(1), deadzone)
        RA, RM = _compute_stick(axval(2), axval(3), deadzone)
        # 默认按 Linux mapping 取扳机 (axis 2 / 5) — 如不准请按 axis 一栏自行调整
        lt_raw = axval(2, -1.0)
        rt_raw = axval(5, -1.0)
        LT = int(round(max(0.0, min(1.0, (lt_raw + 1) / 2.0)) * 1000))
        RT = int(round(max(0.0, min(1.0, (rt_raw + 1) / 2.0)) * 1000))

        render_text(f"left_stick : angle_x10={LA:5d}  r_x1000={LM*10:5d}", (20, y_pm + 28))
        render_text(f"right_stick: angle_x10={RA:5d}  r_x1000={RM*10:5d}", (20, y_pm + 50))
        render_text(f"triggers   : LT={LT:4d}  RT={RT:4d}", (20, y_pm + 72))
        render_text(f"dpad bits  : {bin(merged)[2:].zfill(4)} ({fmt(merged)})", (20, y_pm + 94),
                    HI if merged else DIM)

        # ---------------- footer ----------------
        render_text(f"last pressed button: {last_pressed_btn}    "
                    f"last moved axis: {last_moved_axis[0]} ({last_moved_axis[1]:+.3f})",
                    (20, HEIGHT - 56), DIM)
        render_text("Press D-pad UP/DOWN/LEFT/RIGHT now. If 'MERGED' lights up, your mapping works.",
                    (20, HEIGHT - 34), WARN)
        render_text("Esc to quit.", (20, HEIGHT - 16), DIM)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
