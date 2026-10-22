import unittest
import numpy as np
from joint_approach_path import nearest_revolute_solution, plan_joint_approach


class JointApproachTests(unittest.TestCase):
    def test_equivalent_winding_avoids_extra_revolution(self):
        result = nearest_revolute_solution([5.5], [0.0], [-6.3], [6.3])
        self.assertAlmostEqual(result[0], 5.5 - 2 * np.pi)
        np.testing.assert_allclose(np.cos(result), np.cos([5.5]))

    def test_detours_around_obstacle(self):
        def valid(q):
            return bool(np.all(np.abs(q) <= 1) and not (abs(q[0]) < 0.2 and abs(q[1]) < 0.5))

        path = plan_joint_approach([-.8, 0], [.8, 0], [-1, -1], [1, 1], valid)
        np.testing.assert_allclose(path[0], [-.8, 0])
        np.testing.assert_allclose(path[-1], [.8, 0])
        self.assertGreater(len(path), 2)
        for first, last in zip(path, path[1:]):
            for alpha in np.linspace(0, 1, 101):
                self.assertTrue(valid(first + (last - first) * alpha))

    def test_invalid_start_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "start or goal"):
            plan_joint_approach([0, 0], [.8, 0], [-1, -1], [1, 1], lambda q: q[0] > .1)

    def test_disconnected_space_reports_exhaustion(self):
        with self.assertRaisesRegex(RuntimeError, "exhausted"):
            plan_joint_approach([-.8, 0], [.8, 0], [-1, -1], [1, 1],
                                lambda q: abs(q[0]) > .2, max_nodes=40)


if __name__ == "__main__":
    unittest.main()
