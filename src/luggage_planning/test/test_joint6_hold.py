"""Hold elfin_joint6 during pick (no ROS graph)."""

import unittest

from luggage_planning.joint6_hold import (
    JOINT6_NAME,
    PICK_HOLD_JOINT6_NAMES,
    joint6_value,
    pick_holds_joint6,
    pin_joint6_positions,
    pin_joint6_trajectory,
)


class _Point(object):
    def __init__(self, positions, velocities=None):
        self.positions = list(positions)
        self.velocities = None if velocities is None else list(velocities)


class _Traj(object):
    def __init__(self, names, points):
        self.joint_names = list(names)
        self.points = list(points)


class Joint6HoldTest(unittest.TestCase):
    def test_pick_segments_hold_joint6_place_does_not(self):
        for name in ("pre_grasp", "approach", "attach",
                     "pick_retreat", "retry_reverse"):
            self.assertTrue(pick_holds_joint6(name), name)
        self.assertTrue(pick_holds_joint6("place_pickup_connect_1"))
        self.assertTrue(pick_holds_joint6("place_pickup_connect_12"))
        for name in ("transit", "traverse", "insert", "descend", "retreat",
                     "place_via_0", "pre_place", "place_descend"):
            self.assertFalse(pick_holds_joint6(name), name)
        self.assertEqual(
            PICK_HOLD_JOINT6_NAMES,
            frozenset((
                "pre_grasp", "approach", "attach",
                "pick_retreat", "retry_reverse")))

    def test_joint6_value_reads_index_five(self):
        self.assertIsNone(joint6_value(None))
        self.assertIsNone(joint6_value([0.0] * 5))
        self.assertAlmostEqual(joint6_value([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]), 0.6)

    def test_pin_positions_keeps_j1_j5_and_holds_j6(self):
        start = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        ik = [1.0, 1.1, 1.2, 1.3, 1.4, -2.5]
        names = [
            "elfin_joint1", "elfin_joint2", "elfin_joint3",
            "elfin_joint4", "elfin_joint5", JOINT6_NAME,
        ]
        pinned = pin_joint6_positions(ik, start[5], names)
        self.assertEqual(pinned[:5], [1.0, 1.1, 1.2, 1.3, 1.4])
        self.assertAlmostEqual(pinned[5], 0.6)
        self.assertEqual(ik[5], -2.5)

    def test_pin_trajectory_holds_j6_on_every_knot(self):
        names = [
            "elfin_joint1", "elfin_joint2", "elfin_joint3",
            "elfin_joint4", "elfin_joint5", JOINT6_NAME,
        ]
        traj = _Traj(names, [
            _Point([0.0, 0.0, 0.0, 0.0, 0.0, 0.1]),
            _Point([0.2, 0.0, 0.0, 0.3, 0.4, 1.7]),
            _Point([0.4, 0.0, 0.0, 0.3, 0.4, -0.8]),
        ])
        pin_joint6_trajectory(traj, 0.1)
        for point in traj.points:
            self.assertAlmostEqual(point.positions[5], 0.1)
        self.assertAlmostEqual(traj.points[1].positions[0], 0.2)
        self.assertAlmostEqual(traj.points[2].positions[3], 0.3)

    def test_pin_trajectory_zeros_j6_velocity(self):
        names = [
            "elfin_joint1", "elfin_joint2", "elfin_joint3",
            "elfin_joint4", "elfin_joint5", JOINT6_NAME,
        ]
        traj = _Traj(names, [
            _Point([0.0] * 5 + [0.4], [0.1, 0.0, 0.0, 0.0, 0.2, 1.5]),
        ])
        pin_joint6_trajectory(traj, -0.2)
        self.assertAlmostEqual(traj.points[0].positions[5], -0.2)
        self.assertAlmostEqual(traj.points[0].velocities[5], 0.0)
        self.assertAlmostEqual(traj.points[0].velocities[0], 0.1)
        self.assertAlmostEqual(traj.points[0].velocities[4], 0.2)


if __name__ == "__main__":
    unittest.main()
