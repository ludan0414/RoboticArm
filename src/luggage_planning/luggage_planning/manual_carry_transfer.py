"""Photographed, site-approved joint targets after a sealed pickup.

The controller pendant displays J1..J6 in degrees. The photographed Base/TCP
coordinates are intentionally not used: downstream placement starts from a
fresh world-frame TF after P2 has actually settled.
"""
import math

from .pose import MotionSegment

TRANSFER_JOINTS_DEG = (
    (212.133, -104.860, -89.370, -71.821, 91.730, 38.169),  # P1
    (276.291, -68.652, -122.978, -75.087, 87.203, 102.188),  # P2
)
TRANSFER_JOINTS_RAD = tuple(
    tuple(math.radians(angle) for angle in joints) for joints in TRANSFER_JOINTS_DEG
)
TRANSFER_TYPES = ('commissioning_carry_p1', 'commissioning_carry_p2')
TRANSFER_SETTLE_TOLERANCE_RAD = 0.02


def transfer_segments():
    return [MotionSegment(name='carry_p%d' % (i + 1), type=kind)
            for i, kind in enumerate(TRANSFER_TYPES)]


def target_joint_error(segment_type, actual_joints):
    """Require actual six-axis arrival; wrapping would hide a full turn."""
    index = TRANSFER_TYPES.index(segment_type)
    if actual_joints is None or len(actual_joints) != 6:
        return None
    return max(abs(float(a) - desired)
               for a, desired in zip(actual_joints, TRANSFER_JOINTS_RAD[index]))


def execute_transfer(ports, segments, start=0, checkpoint=None):
    """Execute P1 then P2 with held-box gates; never release on failure."""
    if not 0 <= start <= len(segments):
        raise ValueError('invalid carry transfer checkpoint')
    for index in range(start, len(segments)):
        segment = segments[index]
        healthy, reason = ports.graph_health()
        if not healthy or ports.di0() != 1:
            return False, 'TRANSFER_CARRY_FAULT: %s / DI0' % reason
        if checkpoint is not None:
            checkpoint(index)
        outcome = ports.execute_segment(segment, 'carry_transfer')
        ports.log_event(dict(stage='carry_transfer', name=segment.name,
                             ok=outcome.ok, message=outcome.message))
        if not outcome.ok:
            return False, 'TRANSFER_CARRY_FAULT [%s]: %s' % (segment.name, outcome.message)
        if checkpoint is not None:
            checkpoint(index + 1)
    return True, 'P2 reached; plan from fresh actual TF'
