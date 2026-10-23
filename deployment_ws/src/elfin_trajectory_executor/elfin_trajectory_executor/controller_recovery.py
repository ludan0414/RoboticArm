"""Bounded, explicit controller recovery. No ROS, vacuum or trajectory effects.

The caller owns the hardware exclusively. Commands must return raw CPS codes;
read_state must read the controller, not an application READY flag.
"""
from dataclasses import dataclass
import time

from . import controller_state as fsm


@dataclass(frozen=True)
class RecoveryResult:
    ok: bool
    stage: str
    fsm_id: int | None
    message: str


def recover_standby(read_state, command, timeout=8.0, *,
                    monotonic=time.monotonic, sleep=time.sleep):
    """Stop once; reset real faults once; enable only if disabled; verify 33.

    Emergency stop, unknown states and script/free-drive modes require local
    intervention. A return code 0 is not proof of the requested state.
    """
    deadline = monotonic() + timeout
    stopped = reset = enabled = False
    stable_since = None
    stage = "READ_STATE"
    while monotonic() < deadline:
        state, description = read_state()
        if state == fsm.STANDBY and stopped:
            if stable_since is None:
                stable_since = monotonic()
            elif monotonic() - stable_since >= .1:
                return RecoveryResult(True, "STANDBY", state, "confirmed StandBy 33")
        else:
            stable_since = None
            name = None
            if state in fsm.STOPPABLE and not stopped:
                name, stage, stopped = "GrpStop", "STOP", True
            elif state in (fsm.COLLISION_STOP, fsm.ERROR) and not reset:
                name, stage, reset = "GrpReset", "RESET_FAULT", True
                stopped = False
            elif state == fsm.DISABLED and not enabled:
                name, stage, enabled = "GrpEnable", "ENABLE", True
            elif state not in (fsm.RESETTING, fsm.ENABLING, fsm.DISABLING,
                               fsm.DISABLED, fsm.STOPPING, fsm.MOVING,
                               fsm.JOGGING, fsm.HOLDING):
                return RecoveryResult(False, stage, state,
                    "recovery refused: FSM %s %s" % (state, description))
            if name is not None:
                code = command(name)
                if code != 0:
                    return RecoveryResult(False, stage, state,
                        "%s failed: code %s" % (name, code))
        sleep(.05)
    state, description = read_state()
    return RecoveryResult(False, stage, state,
        "recovery timeout: FSM %s %s" % (state, description))
