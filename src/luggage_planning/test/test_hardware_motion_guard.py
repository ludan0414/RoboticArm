import math
from types import SimpleNamespace
import unittest

from luggage_planning.hardware_motion_guard import (
    HardwareMotionGuard, axis_test_segments, planned_joint_excursion,
)
from luggage_planning.pose import Point, Pose, Quaternion


def yaw_quat(yaw):
    return Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))


def profile():
    return dict(assumptions=dict(feeder_xy_m=[-1.2, 0]), motion_safety=dict(
        schema_version=1, feeder_base_xy_m=[-1.2, 0], ground_base_z_m=-.86,
        expected_container_world_pose=dict(position_m=[-1.65, -.04, 0], orientation_xyzw=[0, 0, 1, 0]),
        expected_base_world_pose=dict(position_m=[0, 0, .86],
            orientation_xyzw=[0, 0, math.sin(-math.pi / 4), math.cos(-math.pi / 4)])))


class GuardTests(unittest.TestCase):
    def test_missing_old_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            HardwareMotionGuard({})

    def test_180_degree_base_reversal_rejected(self):
        guard = HardwareMotionGuard(profile())
        self.assertTrue(guard.check_base(Pose(Point(0, 0, .86), yaw_quat(-math.pi / 2)))[0])
        self.assertFalse(guard.check_base(Pose(Point(0, 0, .86), yaw_quat(math.pi / 2)))[0])
        self.assertFalse(guard.check_base(Pose(Point(math.nan, 0, .86), yaw_quat(-math.pi / 2)))[0])

    def test_right_feeder_passes_and_mirrored_box_stops(self):
        guard = HardwareMotionGuard(profile())
        base_from_world = Pose(Point(0, 0, -.86), yaw_quat(math.pi / 2))
        detected = SimpleNamespace(frame="world", box_xyz=[0, 1.2, .1], box_size=[.55, .35, .2])
        self.assertTrue(guard.check_pickup(detected, base_from_world)[0])
        detected.box_xyz = [0, -1.2, .1]
        self.assertFalse(guard.check_pickup(detected, base_from_world)[0])
        detected.box_xyz = [math.nan, 1.2, .1]
        self.assertFalse(guard.check_pickup(detected, base_from_world)[0])

    def test_cross_cell_jump_and_half_turn_rejected_but_vertical_pick_allowed(self):
        guard = HardwareMotionGuard(profile())
        origin = Pose(Point(0, 1.2, .86), Quaternion())
        self.assertTrue(guard.check_segment(origin, Pose(Point(0, 1.2, .30), Quaternion()), "pre_grasp")[0])
        self.assertFalse(guard.check_segment(origin, Pose(Point(0, -1.2, .86), Quaternion()), "return_observe")[0])
        self.assertFalse(guard.check_segment(origin, Pose(origin.position, yaw_quat(math.pi)), "pre_grasp")[0])

    def test_six_axis_jogs_are_base_relative_and_return_after_every_direction(self):
        origin = Pose(Point(.1, 1.2, .86), Quaternion())
        segments = axis_test_segments(origin, Pose(Point(), yaw_quat(-math.pi / 2)))
        self.assertEqual(len(segments), 12)
        self.assertAlmostEqual(segments[0].target_pose.position.y, 1.18)  # +X left
        self.assertAlmostEqual(segments[4].target_pose.position.x, .12)  # +Y behind
        self.assertAlmostEqual(segments[8].target_pose.position.z, .88)
        for move, back in zip(segments[::2], segments[1::2]):
            self.assertEqual(back.target_pose, origin)
            self.assertEqual(move.target_pose.orientation, origin.orientation)
            self.assertEqual(move.required_cartesian_fraction, 1.0)
            self.assertFalse(move.allow_ompl_fallback)
        with self.assertRaises(ValueError):
            axis_test_segments(origin, Pose(), .1)

    def test_planned_half_turn_and_period_wrap_blocked_before_execution(self):
        def trajectory(values):
            return SimpleNamespace(points=[SimpleNamespace(positions=[v, 0]) for v in values])
        self.assertTrue(planned_joint_excursion(trajectory([0, .1, .2]), math.pi / 4)[0])
        self.assertFalse(planned_joint_excursion(trajectory([0, 1, math.pi]), math.pi / 4)[0])
        self.assertFalse(planned_joint_excursion(trajectory([3.13, -3.13]), math.pi / 4)[0])


if __name__ == "__main__":
    unittest.main()
