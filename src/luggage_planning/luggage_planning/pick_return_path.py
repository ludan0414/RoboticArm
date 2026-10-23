"""Measured successful carry waypoints for empty-tool return and recovery."""
from copy import deepcopy

from .pose import MotionSegment
from .motion_batches import GROUND_TURN_STOPS, stop_indices


class PickReturnPath:
    def __init__(self):
        self.poses = []
        self._chunk_sizes = None
        self._required_indices = set()

    def record(self, name, pose, waypoints=()):
        self._chunk_sizes = None
        if name == 'pick_retreat':
            self.poses = [deepcopy(pose)]
            self._required_indices = {0}
        elif self.poses and (name.startswith(('ground_entry_', 'place_pickup_connect_', 'place_via_'))
                            or name == 'pre_place'):
            self.poses.extend(deepcopy(list(waypoints)))
            self.poses.append(deepcopy(pose))
            if name in GROUND_TURN_STOPS:
                self._required_indices.add(len(self.poses)-1)
        # Descend/retreat are deliberately excluded: reversing them would
        # lower the empty tool back into the box that was just released.

    def next_segment(self):
        if self._chunk_sizes is None:
            count = len(self.poses)
            ends = stop_indices(count,7,(count-1-i for i in self._required_indices if i<count))
            self._chunk_sizes = [end-start for start,end in zip([-1,*ends],ends)]
        poses = list(reversed(self.poses[-self._chunk_sizes[0]:]))
        return MotionSegment(name='return_pick_stop_%d' % (len(self._chunk_sizes)-1),
                             type='cartesian', target_pose=poses[-1], waypoints=poses[:-1],
                             required_cartesian_fraction=1.0, allow_ompl_fallback=False)

    def settled(self):
        # Consume only after success so a failed return resumes at that waypoint.
        count = self._chunk_sizes.pop(0)
        del self.poses[-count:]
