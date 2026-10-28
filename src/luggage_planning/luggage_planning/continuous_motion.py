"""Combine samples within a carry phase; preserve physical stop boundaries.

One Cartesian action carries the ordered poses through GetCartesianPath and
the normal attached-payload collision checks. No spline is fit to pose corners,
no waypoint is discarded and no independent action is sent per sample on the
normal complete path. Incomplete geometry uses cartesian_recovery to reach
every original sample with separately checked full-goal plans.
"""
from copy import deepcopy
from dataclasses import replace


def _phase(segment):
    if segment.type != 'cartesian':
        return None
    for phase in ('outer_lift', 'outside_retract', 'high_lift'):
        prefix = 'ground_entry_' + phase
        if segment.name == prefix or segment.name.startswith(prefix + '_'):
            return prefix
    if segment.name.startswith(('place_pickup_connect_', 'place_via_')) or segment.name == 'pre_place':
        return 'carry'
    return None


def continuous_carry(segments):
    result = []
    group = []
    phase = None

    def flush():
        if not group:
            return
        poses = [deepcopy(p) for item in group for p in [*item.waypoints, item.target_pose]]
        result.append(replace(group[-1], waypoints=poses[:-1], target_pose=poses[-1],
                              required_cartesian_fraction=1.0, allow_ompl_fallback=False))
        group.clear()

    for segment in segments:
        next_phase = _phase(segment)
        if next_phase is None or next_phase != phase:
            flush()
        if next_phase is None:
            result.append(deepcopy(segment))
        else:
            group.append(segment)
        phase = next_phase
    flush()
    return result
