"""Bounded drop-release recovery, preserving XY/orientation and full planning.

The operator permits releasing above the configured support. A failed final IK
may be caused by tiny modeled support contact. Try complete straight descents
2.5/5/7.5mm higher; never execute a partial prefix or continue a hardware error.
The independent sealed-grasp release check still requires <=10mm position error.
"""
from copy import deepcopy

NO_IK_SOLUTION = -31
RELEASE_RAISES_M = (.0025, .005, .0075)


def recover_release_height(executor, segment, feedback, timeout, original):
    if getattr(original, 'moveit_error_code', 0) != NO_IK_SOLUTION or segment.waypoints:
        return original
    attempts = executor._release_height_recovery = []
    for height in RELEASE_RAISES_M:
        cancel = getattr(executor, '_cancel_check', None)
        if cancel is not None and cancel():
            return executor._result(False, 'release recovery cancelled')
        goal = deepcopy(segment)
        goal.target_pose.position.z += height
        executor._notify(feedback, 'planning', segment.name, 0.,
                         'retry full release descent %.1fmm higher' % (1000*height))
        fraction, plan = executor._plan_cartesian(goal, feedback)
        attempt = dict(raise_world_z_m=height, fraction=fraction, executed=False)
        attempts.append(attempt)
        if (plan is None or fraction != 1. or getattr(plan, 'occupancy_blocked', False)
                or getattr(plan, 'wrist_singular', False)):
            continue
        if executor._occupancy is not None:
            ok, reason = executor._occupancy_ok_plan(plan, goal)
            if not ok:
                attempt['rejection'] = reason
                continue
        attempt['executed'] = True
        result = executor._execute_trajectory(plan, goal, fraction, feedback, timeout)
        result.message += '; release target raised %.1fmm (XY/orientation unchanged)' % (1000*height)
        return result  # Never retry an execution/hardware failure.
    original.message += '; no complete descent within +7.5mm release allowance'
    return original
