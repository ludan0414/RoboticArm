"""Controller FSM IDs from the vendored CPS SDK; never compare their order."""

ESTOP = 5
RESETTING = 19
COLLISION_STOP = 21
ERROR = 22
ENABLING = 23
DISABLED = 24
MOVING = 25
JOGGING = 26
STOPPING = 27
DISABLING = 28
HOLDING = 32
STANDBY = 33

# Script states 34..38 are a different lifecycle, not ServoJ active states.
MOTION_READY = frozenset((MOVING, JOGGING, HOLDING, STANDBY))
SERVO_STATES = frozenset((MOVING, STANDBY))
FAULTS = frozenset((ESTOP, COLLISION_STOP, ERROR))
STOPPABLE = frozenset((MOVING, JOGGING, STOPPING, HOLDING, STANDBY))

LABELS = {
    ESTOP: "急停 ESTOP", RESETTING: "复位中 Resetting",
    COLLISION_STOP: "碰撞停止 CollisionStop", ERROR: "控制器错误 Error",
    ENABLING: "使能中 Enabling", DISABLED: "未使能 Disable",
    MOVING: "运动中 Moving", JOGGING: "点动中 JogMoving",
    STOPPING: "停止中 Stopping", DISABLING: "下使能中 Disabling",
    29: "开启自由驱动", 30: "关闭自由驱动", 31: "自由驱动 FreeDriver",
    HOLDING: "保持中 Holding", STANDBY: "待机 StandBy",
    34: "脚本运行 ScriptRunning", 35: "脚本暂停处理中",
    36: "脚本已暂停", 37: "脚本停止中", 38: "脚本已停止",
}
