"""Measured successful carry waypoints for empty-tool return and recovery."""
from copy import deepcopy

from .pose import MotionSegment


class PickReturnPath:
    def __init__(self):
        self.poses = []

    def record(self, name, pose, waypoints=()):
        if name == 'pick_retreat':
            self.poses = [deepcopy(pose)]
        elif self.poses and (name.startswith(('ground_entry_', 'place_pickup_connect_', 'place_via_'))
                            or name == 'pre_place'):
            self.poses.extend(deepcopy(list(waypoints)))
            self.poses.append(deepcopy(pose))
        # Descend/retreat are deliberately excluded: reversing them would
        # lower the empty tool back into the box that was just released.

    def next_segment(self):
        return MotionSegment(name='return_pick_stop_%d' % (len(self.poses)-1),
                             type='cartesian', target_pose=self.poses[-1],
                             required_cartesian_fraction=1.0, allow_ompl_fallback=False)

    def settled(self):
        # Consume only after success so a failed return resumes at that waypoint.
        self.poses.pop()
