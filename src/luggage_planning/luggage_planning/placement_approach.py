"""Lower the final carry approach before the arm reaches full extension."""
from dataclasses import replace
import math

from .pose import Point, Pose

FINAL_APPROACH_RADIUS_M = .30


def lower_final_approach(target, radius=FINAL_APPROACH_RADIUS_M):
    """Blend only the final near-target suffix toward pre-place box height.

    XY and box orientation remain the reviewed route. Far-field/pedestal carry
    heights remain unchanged. Every modified pose must still pass live complete
    Cartesian/MoveIt collision checks with the measured attached payload.
    """
    if not math.isfinite(radius) or radius < .05:
        raise ValueError("final approach radius must be finite and >= 0.05m")
    via = list(target.via)
    end = target.box_pose.position
    pre_z = end.z + target.clearance_m
    for i in range(len(via)-1, -1, -1):
        pose = via[i]
        distance = math.hypot(pose.position.x-end.x, pose.position.y-end.y)
        if distance >= radius:
            break
        ratio = distance/radius
        blend = ratio*ratio*(3-2*ratio)
        z = min(pose.position.z, pre_z + blend*max(0., pose.position.z-pre_z))
        via[i] = Pose(Point(pose.position.x, pose.position.y, z), pose.orientation)
    return replace(target, via=tuple(via))
