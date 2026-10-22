import unittest
import numpy as np
from demo_collision import surface_gaps


class ContactGapTests(unittest.TestCase):
    def test_effective_thickness_is_subtracted(self):
        q = np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=float)
        gap = surface_gaps(q, np.array([-1]), np.array([-1]),
                           np.array([[0, 0, 0]]), np.array([[.01, 0, 0]]),
                           np.array([[1, 0, 0]]), np.array([.01]), np.array([.01]))
        self.assertAlmostEqual(gap[0], -.01)

    def test_body_frame_points_transform_to_world(self):
        q = np.array([[1, 0, 0, 0, 0, np.sqrt(.5), np.sqrt(.5)]])
        gap = surface_gaps(q, np.array([0]), np.array([-1]),
                           np.array([[1, 0, 0]]), np.array([[1, 1.2, 0]]),
                           np.array([[0, 1, 0]]), np.array([0]), np.array([0]))
        self.assertAlmostEqual(gap[0], .2)


if __name__ == "__main__":
    unittest.main()
