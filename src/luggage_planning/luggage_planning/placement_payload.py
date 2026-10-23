"""Configured payload geometry for the operator's frozen commissioning route."""
from dataclasses import replace

from .pose import Point, Pose
from .suction_candidate_waypoints import quat_rotate

# The STL floor has millimetre-scale relief. The operator permits dropping the
# box; release 5 mm above support instead of driving the mesh into contact.
RELEASE_SUPPORT_GAP_M = .005


def release_above_support(target):
    """Keep configured dimensions and add a small gap along the box's up axis.

    Reference dimensions and target came from the same frozen policy input.
    Vision must not change that input's height in this commissioning mode.
    Every shifted target still needs complete collision-checked planning.
    """
    def shifted(pose):
        delta = quat_rotate(pose.orientation, (0., 0., RELEASE_SUPPORT_GAP_M))
        return Pose(Point(*(getattr(pose.position, k) + d
                            for k, d in zip(('x', 'y', 'z'), delta))), pose.orientation)

    return replace(target, box_pose=shifted(target.box_pose),
                   via=tuple(shifted(pose) for pose in target.via))
