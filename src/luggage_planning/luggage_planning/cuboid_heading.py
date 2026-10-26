"""Choose a nearby cuboid heading without distinguishing its front/back.

Only a half-turn about BOX Z is equivalent for a non-square cuboid. A quarter
turn swaps its long/short axes and must still come from the placement policy.
Keep one symmetry choice for the entire route so it cannot jump branches.
"""
from dataclasses import replace
import math

from .pose import Quaternion
from .policy_place import _quaternion
from .suction_candidate_waypoints import quat_conjugate, quat_mul


def _unit(q):
    return _quaternion([q.x,q.y,q.z,q.w])


def yaw_twist(reference, target):
    """Signed twist about the picked box's up axis, independent of q/-q."""
    relative=quat_mul(quat_conjugate(_unit(reference)),_unit(target))
    if math.hypot(relative.z,relative.w)<1e-8:
        raise ValueError('CUBOID_HEADING: box up axis cannot be reversed')
    return (2*math.atan2(relative.z,relative.w)+math.pi)%(2*math.pi)-math.pi


def nearest_cuboid_heading(target, picked_box_pose):
    """Preserve positions, sizes and occupied volumes; limit yaw to <=90deg.

    The measured rigid grasp is unchanged. Tool poses must be regenerated
    from the chosen box poses, including any off-centre contact translation.
    No IK joint values are patched after planning.
    """
    original=[*target.via,target.box_pose]
    candidates=[]
    for half_turn in (False,True):
        poses=[replace(p,orientation=quat_mul(_unit(p.orientation),Quaternion(0,0,1,0)))
               if half_turn else p for p in original]
        turns=[yaw_twist(picked_box_pose.orientation,p.orientation) for p in poses]
        candidates.append((max(abs(v) for v in turns),abs(turns[0]),half_turn,poses,turns))
    maximum,_,half_turn,poses,turns=min(candidates,key=lambda c:c[:3])
    if maximum>math.pi/2+1e-8:
        raise ValueError('CUBOID_HEADING: equivalent route still exceeds 90deg from pickup')
    previous=picked_box_pose.orientation
    for pose in poses:
        if abs(yaw_twist(previous,pose.orientation))>math.pi/2+1e-8:
            raise ValueError('CUBOID_HEADING: route contains an unnecessary half-turn')
        previous=pose.orientation
    report=dict(half_turn_equivalent=half_turn,
                original_max_yaw_deg=math.degrees(candidates[0][0]),
                selected_max_yaw_deg=math.degrees(maximum),
                first_yaw_deg=math.degrees(turns[0]),
                final_yaw_deg=math.degrees(turns[-1]))
    return replace(target,via=tuple(poses[:-1]),box_pose=poses[-1]),report
