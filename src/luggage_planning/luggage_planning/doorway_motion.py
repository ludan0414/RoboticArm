"""Commissioning adapter for the shared doorway envelope, without ROS imports."""
from dataclasses import replace
import math

from luggage_description.doorway_clearance import DoorwayGeometry
from .pose import Point


def pose_values(pose):
    return ([pose.position.x,pose.position.y,pose.position.z],
            [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])


class DoorwayMotionGuard:
    def __init__(self, profile):
        settings=profile.get('doorway_clearance')
        if settings is None:
            raise ValueError('profile lacks doorway_clearance; regenerate the >=5cm checked route')
        self.geometry=DoorwayGeometry.from_settings(settings,require_site_margin=True)

    def collision_boxes(self, container_pose):
        xyz,quat=pose_values(container_pose)
        return self.geometry.world_guard_boxes(xyz,quat)

    def align_target(self, target, box_size, container_pose):
        origin,container_quat=pose_values(container_pose)

        def aligned(pose):
            xyz,quat=pose_values(pose)
            result=self.geometry.align_box(xyz,quat,box_size,origin,container_quat)
            return replace(pose,position=Point(*result))

        final=aligned(target.box_pose)
        if math.dist(pose_values(final)[0],pose_values(target.box_pose)[0])>1e-8:
            raise ValueError('DOORWAY_PLACE_TARGET_UNSAFE: final slot overlaps the protected frame')
        via=tuple(aligned(pose) for pose in target.via)
        changes=sum(math.dist(pose_values(a)[0],pose_values(b)[0])>1e-9 for a,b in zip(target.via,via))
        return replace(target,via=via),dict(clearance_m=self.geometry.clearance,adjusted_via_poses=changes)
