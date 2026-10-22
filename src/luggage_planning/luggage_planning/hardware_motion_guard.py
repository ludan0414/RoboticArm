"""Commissioning frame identity and short motions after vision-owned pickup."""
import math

from luggage_planning.pose import Point, Pose, Quaternion, MotionSegment
from luggage_planning.policy_place import _pose
from luggage_planning.suction_candidate_waypoints import quat_rotate


def quaternion_angle(first, second):
    a = (first.x, first.y, first.z, first.w)
    b = (second.x, second.y, second.z, second.w)
    if (not all(math.isfinite(v) for v in a + b)
            or abs(sum(v * v for v in a) - 1) > .01
            or abs(sum(v * v for v in b) - 1) > .01):
        return math.inf
    return 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a, b)))))


class HardwareMotionGuard:
    def __init__(self, profile):
        settings = profile.get("motion_safety", {})
        if settings.get("schema_version") != 1:
            raise ValueError("profile lacks motion_safety v1; old scene profiles must be replaced")
        self.base_world = _pose(settings["expected_base_world_pose"])
        self.container_world = _pose(settings["expected_container_world_pose"])
        self.feeder = tuple(float(v) for v in settings["feeder_base_xy_m"])
        if len(self.feeder) != 2 or not all(math.isfinite(v) for v in self.feeder):
            raise ValueError("finite base-frame feeder XY required")
        if list(self.feeder) != profile.get("assumptions", {}).get("feeder_xy_m"):
            raise ValueError("feeder assumption and safety gate disagree")
        self.step = self._limit(settings, "max_segment_translation_m", .50, .60)
        self.rotation = self._limit(settings, "max_segment_rotation_rad", math.pi / 2, math.pi / 2)
        self.enforce_segments = bool(settings.get("enforce_segment_heuristics", False))

    @staticmethod
    def _limit(settings, name, default, maximum):
        value = float(settings.get(name, default))
        if not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError("unsafe limit: " + name)
        return value

    def check_base(self, actual):
        distance = math.dist((actual.position.x, actual.position.y, actual.position.z),
                            (self.base_world.position.x, self.base_world.position.y, self.base_world.position.z))
        if not math.isfinite(distance) or distance > .03 or quaternion_angle(actual.orientation, self.base_world.orientation) > math.radians(5):
            return False, "BASE_FRAME_MISMATCH: expected site base pose, possible 180-degree yaw reversal"
        return True, "site base TF matches"

    def check_container(self, actual):
        distance = math.dist((actual.position.x, actual.position.y, actual.position.z),
            (self.container_world.position.x, self.container_world.position.y, self.container_world.position.z))
        if not math.isfinite(distance) or distance > .03 or quaternion_angle(
                actual.orientation, self.container_world.orientation) > math.radians(5):
            return False, "CONTAINER_FRAME_MISMATCH: runtime scene differs from approved YAML"
        return True, "site container TF matches"

    def check_pickup(self, detection, base_from_world=None):
        if (detection.frame != "world" or len(detection.box_xyz) != 3
                or len(detection.box_size) != 3
                or not all(math.isfinite(v) for v in detection.box_xyz)
                or not all(math.isfinite(v) and v > 0 for v in detection.box_size)):
            return False, "INVALID_PICKUP_GEOMETRY"
        # The feeder is nominal, not a measured target. Original vision Pick
        # owns candidate authorization, reachability, approach and seal checks.
        # Ignore legacy pickup_radius_m / ground_base_z_m in installed profiles.
        return True, "measured pickup delegated to vision Pick"

    def check_segment(self, actual, target, name):
        start = (actual.position.x, actual.position.y, actual.position.z)
        end = (target.position.x, target.position.y, target.position.z)
        if not all(math.isfinite(v) for v in start + end):
            return False, "NONFINITE_MOTION_TARGET"
        if name in ("initialize_observe", "return_observe"):
            # Free-space pose goals already have collision-aware MoveIt plans.
            # A TCP chord cap was blocking valid initialization/return paths.
            if not math.isfinite(quaternion_angle(actual.orientation, target.orientation)):
                return False, "INVALID_MOTION_ORIENTATION"
            return True, "MoveIt owns free-space observe motion"
        if name in ("pre_grasp", "approach", "attach", "pick_retreat", "retry_reverse"):
            if not math.isfinite(quaternion_angle(actual.orientation, target.orientation)):
                return False, "INVALID_PICK_ORIENTATION"
            return True, "existing vision Pick owns this segment"
        limit = self.step
        angle = quaternion_angle(actual.orientation, target.orientation)
        if not math.isfinite(angle):
            return False, "INVALID_MOTION_ORIENTATION"
        if math.dist(start, end) > limit + 1e-5:
            return not self.enforce_segments, "MOTION_STEP_WARNING: %s %.3fm > %.3fm" % (name, math.dist(start, end), limit)
        if angle > self.rotation:
            return not self.enforce_segments, "MOTION_ROTATION_WARNING: " + name
        return True, "bounded segment"


def planned_joint_excursion(robot_trajectory, maximum_rad):
    """Reject large raw joint swings before sending a planned trajectory.

    Do not wrap +/-pi: a controller receives these raw positions and would
    physically execute a long turn. Applies to pose-target and Cartesian plans.
    """
    trajectory = getattr(robot_trajectory, "joint_trajectory", robot_trajectory)
    points = list(getattr(trajectory, "points", []))
    if not points or not points[0].positions:
        return False, "JOINT_EXCURSION_GUARD: empty trajectory"
    start = list(points[0].positions)
    for point in points:
        positions = list(point.positions)
        if len(positions) != len(start) or not all(math.isfinite(v) for v in positions):
            return False, "JOINT_EXCURSION_GUARD: malformed trajectory"
        worst = max(range(len(start)), key=lambda i: abs(positions[i]-start[i]))
        excursion = abs(positions[worst]-start[worst])
        if excursion > maximum_rad:
            names = getattr(trajectory, "joint_names", ())
            name = names[worst] if worst < len(names) else "joint%d" % (worst+1)
            return False, ("JOINT_EXCURSION_GUARD: %s moves %.1f degrees relative to segment start; limit %.1f degrees"
                           % (name, math.degrees(excursion), math.degrees(maximum_rad)))
    return True, "bounded planned joint motion"


def axis_test_segments(origin, base_world, step_m=.02):
    """Six base-axis jogs, each followed by a return to the same saved pose."""
    if not math.isfinite(step_m) or not 0 < step_m <= .03:
        raise ValueError("axis test step must be >0 and <=3cm")
    segments = []
    for axis, label in enumerate(("x", "y", "z")):
        for sign, direction in ((1, "plus"), (-1, "minus")):
            delta = [0.0, 0.0, 0.0]
            delta[axis] = sign * step_m
            world = quat_rotate(base_world.orientation, delta)
            target = Pose(Point(origin.position.x + world[0], origin.position.y + world[1],
                                origin.position.z + world[2]), origin.orientation)
            for name, pose in (("axis_test_%s_%s" % (direction, label), target),
                               ("axis_return_%s_%s" % (direction, label), origin)):
                segments.append(MotionSegment(name=name, type="cartesian", target_pose=pose,
                    allow_ompl_fallback=False, required_cartesian_fraction=1.0))
    return segments
