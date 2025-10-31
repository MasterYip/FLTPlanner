from abc import ABC, abstractmethod
from enum import IntEnum
import pyads

# 运动模式枚举类
class CtrlCmd(IntEnum):
    IDLE = 0
    SET_PTPOSE = 1
    MODAL_MOV = 2
    FORCE_MOV = 3
    BACK_MOV = 4
    FOLLOW_MOV = 6
    WHEEL2LEG = 7
    LEG2WHEEL = 8
    WAIT_TRIG = 9
    EXAMPLE_MOV = 10
    POSE_MOV = 11
    STEP_MOV = 12
    TRACK_MOV = 13
    ONLINE_MOV = 14
    LEGS_MOV = 15
    FORCE_STEP_MOV = 16
    FORCE_ONLINE_MOV = 17
    STOP_MOV = 18
    PARK_MOV = 19
    DITCH_MOV = 20
    OBSTC_MOV = 21
    SLOPE1_MOV = 22
    SLOPE2_MOV = 23
    REMOTE_MOV = 30


# 状态切换枚举类
class State(IntEnum):
    INIT = 0
    ENABLE = 1
    FEEDMOV = 2
    DISENABLE = 3
    SETPOSITION = 4
    RESET = 5
    ERROR = 6
    IDLE = 7
    
    
# 运动参数-时间结构定义
stTime_def = (
    ("TA", pyads.PLCTYPE_REAL, 1),
    ("TM", pyads.PLCTYPE_REAL, 1),
    ("TD", pyads.PLCTYPE_REAL, 1),
    ("TZ", pyads.PLCTYPE_REAL, 1),
)

# 运动参数-步态结构定义
stGait_def = (
    ("GaitMode", pyads.PLCTYPE_UDINT, 1),
    ("GaitDF", pyads.PLCTYPE_REAL, 1),
    ("SwapHigh", pyads.PLCTYPE_REAL, 1),
    ("LegNum", pyads.PLCTYPE_UDINT, 1),
    ("ForceMode", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),
)

# 运动参数-步长/姿态/足端结构定义
stPose_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("Roll", pyads.PLCTYPE_REAL, 1),
    ("Pitch", pyads.PLCTYPE_REAL, 1),
    ("Yaw", pyads.PLCTYPE_REAL, 1),
    ("FG", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),

    ("X1", pyads.PLCTYPE_REAL, 1),
    ("Y1", pyads.PLCTYPE_REAL, 1),
    ("Z1", pyads.PLCTYPE_REAL, 1),
    ("SF1", pyads.PLCTYPE_DINT, 1),

    ("X2", pyads.PLCTYPE_REAL, 1),
    ("Y2", pyads.PLCTYPE_REAL, 1),
    ("Z2", pyads.PLCTYPE_REAL, 1),
    ("SF2", pyads.PLCTYPE_DINT, 1),

    ("X3", pyads.PLCTYPE_REAL, 1),
    ("Y3", pyads.PLCTYPE_REAL, 1),
    ("Z3", pyads.PLCTYPE_REAL, 1),
    ("SF3", pyads.PLCTYPE_DINT, 1),

    ("X4", pyads.PLCTYPE_REAL, 1),
    ("Y4", pyads.PLCTYPE_REAL, 1),
    ("Z4", pyads.PLCTYPE_REAL, 1),
    ("SF4", pyads.PLCTYPE_DINT, 1),

    ("X5", pyads.PLCTYPE_REAL, 1),
    ("Y5", pyads.PLCTYPE_REAL, 1),
    ("Z5", pyads.PLCTYPE_REAL, 1),
    ("SF5", pyads.PLCTYPE_DINT, 1),

    ("X6", pyads.PLCTYPE_REAL, 1),
    ("Y6", pyads.PLCTYPE_REAL, 1),
    ("Z6", pyads.PLCTYPE_REAL, 1),
    ("SF6", pyads.PLCTYPE_DINT, 1),
)

# Additional structure for free gait foothold definition
stXYZ_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("SF", pyads.PLCTYPE_DINT, 1),
)