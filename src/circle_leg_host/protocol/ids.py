"""Protocol ID & frame constants. 与 Circle_Leg_Host_V1.0 / MCU RosComm 一致。"""

SOF = 0xAA

# 已有链路 (V1.0 保持不变)
PROTO_PC_MSG = 0xFF        # PC -> MCU  手柄输入 (14 bytes)
PROTO_REACHABLE = 0xFE     # MCU -> PC  电机反馈 (40 bytes)

# 新增视觉链路 (仅在 vision.enabled=true 时生效)
PROTO_VISION_CTRL = 0xFD   # MCU -> PC  视觉启停控制 (4 bytes)
PROTO_VISION_RESULT = 0xFC # PC -> MCU  视觉识别结果 (10 bytes)
