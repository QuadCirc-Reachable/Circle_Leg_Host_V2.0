"""协议消息定义 (struct 二进制布局)。

与 MCU 端 (Circle_Leg_V2.0/Applications/Comm_Msg.hpp) 字段顺序严格一致。
"""

import struct
from dataclasses import dataclass


# ============================================================================
# 0xFF  PC -> MCU  Protocol::PC_Msg (14 bytes)  —— V1.0 现有协议, 不可修改
# ============================================================================
PC_MSG_FMT = "<hHhHHHBB"
PC_MSG_SIZE = struct.calcsize(PC_MSG_FMT)
assert PC_MSG_SIZE == 14, "PC_Msg must be 14 bytes"


def pack_pc_msg(
    la_x10: int,
    lm_x1000: int,
    ra_x10: int,
    rm_x1000: int,
    lt_x1000: int,
    rt_x1000: int,
    button_status: int,
    dpad_status: int,
) -> bytes:
    """打包手柄输入帧 payload (14 bytes)。"""
    return struct.pack(
        PC_MSG_FMT,
        la_x10, lm_x1000, ra_x10, rm_x1000,
        lt_x1000, rt_x1000,
        button_status & 0xFF, dpad_status & 0xFF,
    )


# ============================================================================
# 0xFE  MCU -> PC  Protocol::Reachable_Msg (40 bytes) —— 仅解包用 (调试可选)
# ============================================================================
REACHABLE_MSG_FMT = "<4h4h4b4h4h4b"
REACHABLE_MSG_SIZE = struct.calcsize(REACHABLE_MSG_FMT)
assert REACHABLE_MSG_SIZE == 40, "Reachable_Msg must be 40 bytes"


@dataclass
class ReachableMsg:
    gm6020_cur_pos_x10: tuple
    gm6020_tgt_pos_x10: tuple
    gm6020_temp: tuple
    m3508_cur_rpm: tuple
    m3508_tgt_rpm: tuple
    m3508_temp: tuple


def unpack_reachable_msg(data: bytes) -> ReachableMsg:
    vals = struct.unpack(REACHABLE_MSG_FMT, data)
    return ReachableMsg(
        gm6020_cur_pos_x10=vals[0:4],
        gm6020_tgt_pos_x10=vals[4:8],
        gm6020_temp=vals[8:12],
        m3508_cur_rpm=vals[12:16],
        m3508_tgt_rpm=vals[16:20],
        m3508_temp=vals[20:24],
    )


# ============================================================================
# 0xFD  MCU -> PC  VisionCtrl_Msg (4 bytes) —— 新增, 视觉启停 + 阈值
# ----------------------------------------------------------------------------
#   uint8_t  enable;          // 0=stop, 1=run
#   uint16_t min_height_mm;   // 最小台阶阈值 (mm)
#   uint8_t  flags;           // bit0=reset_state, bit1=verbose_log
# ============================================================================
VISION_CTRL_FMT = "<BHB"
VISION_CTRL_SIZE = struct.calcsize(VISION_CTRL_FMT)
assert VISION_CTRL_SIZE == 4, "VisionCtrl_Msg must be 4 bytes"

VCTRL_FLAG_RESET = 1 << 0
VCTRL_FLAG_VERBOSE = 1 << 1


@dataclass
class VisionCtrlMsg:
    enable: bool
    min_height_mm: int
    flags: int

    @property
    def reset(self) -> bool:
        return bool(self.flags & VCTRL_FLAG_RESET)

    @property
    def verbose(self) -> bool:
        return bool(self.flags & VCTRL_FLAG_VERBOSE)


def pack_vision_ctrl(enable: bool, min_height_mm: int, flags: int = 0) -> bytes:
    return struct.pack(
        VISION_CTRL_FMT,
        1 if enable else 0,
        max(0, min(0xFFFF, int(min_height_mm))),
        flags & 0xFF,
    )


def unpack_vision_ctrl(data: bytes) -> VisionCtrlMsg:
    enable, h, flags = struct.unpack(VISION_CTRL_FMT, data)
    return VisionCtrlMsg(enable=bool(enable), min_height_mm=h, flags=flags)


# ============================================================================
# 0xFC  PC -> MCU  VisionResult_Msg (10 bytes) —— 新增, 视觉识别结果
# ----------------------------------------------------------------------------
#   uint8_t  seq;             // 包序号 (溢出自然回绕)
#   uint8_t  status;          // bit0=detected, bit1=reliable,
#                             // bit2=disabled (MCU 或本地停了),
#                             // bit3=fault    (相机/算法异常)
#   uint16_t curb_height_mm;  // 台阶高度 (0..65535 mm)
#   uint16_t distance_mm;     // 车前缘到台阶 (0..65535 mm)
#   int16_t  theta_cdeg;      // 相对台阶角度 ×100 (deg*100)
#   uint8_t  confidence;      // 0..100
#   uint8_t  flags;           // bit0=theta_bad, bit1=dist_bad,
#                             // bit2=no_planes, bit3=stale (>500ms 未更新)
# ============================================================================
VISION_RESULT_FMT = "<BBHHhBB"
VISION_RESULT_SIZE = struct.calcsize(VISION_RESULT_FMT)
assert VISION_RESULT_SIZE == 10, "VisionResult_Msg must be 10 bytes"

VSTATUS_DETECTED = 1 << 0
VSTATUS_RELIABLE = 1 << 1
VSTATUS_DISABLED = 1 << 2
VSTATUS_FAULT = 1 << 3

VFLAG_THETA_BAD = 1 << 0
VFLAG_DIST_BAD = 1 << 1
VFLAG_NO_PLANES = 1 << 2
VFLAG_STALE = 1 << 3


@dataclass
class VisionResultMsg:
    seq: int = 0
    status: int = 0
    curb_height_mm: int = 0
    distance_mm: int = 0
    theta_cdeg: int = 0
    confidence: int = 0
    flags: int = 0

    def to_bytes(self) -> bytes:
        return struct.pack(
            VISION_RESULT_FMT,
            self.seq & 0xFF,
            self.status & 0xFF,
            max(0, min(0xFFFF, int(self.curb_height_mm))),
            max(0, min(0xFFFF, int(self.distance_mm))),
            max(-32768, min(32767, int(self.theta_cdeg))),
            max(0, min(0xFF, int(self.confidence))),
            self.flags & 0xFF,
        )


def unpack_vision_result(data: bytes) -> VisionResultMsg:
    seq, status, h, d, th, conf, fl = struct.unpack(VISION_RESULT_FMT, data)
    return VisionResultMsg(
        seq=seq, status=status,
        curb_height_mm=h, distance_mm=d, theta_cdeg=th,
        confidence=conf, flags=fl,
    )
