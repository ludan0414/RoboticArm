import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from ground_carry_clearance import pedestal_geometry, box_bottom, box_pedestal_separated, outside_staging_xy


class ClearanceTests(unittest.TestCase):
    def setUp(self):
        self.pedestal = pedestal_geometry(dict(pedestal=dict(
            translation=[0, 0, 0], size=[1.05, .74, .86], rotation_rpy=[0, 0, -np.pi/2])))
        self.size = [.55, .35, .2]

    def test_bottom_must_clear_inflated_top(self):
        self.assertFalse(box_pedestal_separated([0, 0, 1.0, 0, 0, 0, 1], self.size, self.pedestal))
        self.assertTrue(box_pedestal_separated([0, 0, 1.02, 0, 0, 0, 1], self.size, self.pedestal))

    def test_ground_pickup_outside_pedestal_is_allowed(self):
        self.assertTrue(box_pedestal_separated([0, 1.6, .1, 0, 0, 0, 1], self.size, self.pedestal))

    def test_side_near_miss_rejected_before_contact(self):
        # Rotated pedestal half-X=.37, box half-X=.275, gap=.035 < .05.
        self.assertFalse(box_pedestal_separated([.68, 0, .5, 0, 0, 0, 1], self.size, self.pedestal))
        self.assertTrue(box_pedestal_separated([.71, 0, .5, 0, 0, 0, 1], self.size, self.pedestal))

    def test_bottom_accounts_for_tilted_long_edge(self):
        q = Rotation.from_euler('y', 45, degrees=True).as_quat()
        expected = 1.2 - (.55 + .2) / 2 / np.sqrt(2)
        self.assertAlmostEqual(box_bottom([0, 0, 1.2, *q], self.size), expected)

    def test_extended_pickup_retracts_while_staying_outside(self):
        staging = outside_staging_xy([0, 1.6], self.pedestal, self.size)
        np.testing.assert_allclose(staging, [0, 1.25])
        for y in np.linspace(1.6, staging[1], 20):
            self.assertTrue(box_pedestal_separated([0, y, .6, 0, 0, 0, 1], self.size, self.pedestal))
        with self.assertRaises(ValueError):
            outside_staging_xy([0, .6], self.pedestal, self.size)
