"""Door approach staging preserves height and follows the configured opening."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from ground_feeder_bridge import portal_staging_point


class PortalStagingTests(unittest.TestCase):
    def test_outward_for_rotated_container_without_lowering_payload(self):
        point = np.array([.4, -.2, 1.3])
        for yaw in (0., .8, -3.13):
            container = dict(rotation_rpy=[.02, -.03, yaw])
            result = portal_staging_point(point, container)
            outward = Rotation.from_euler('xyz', container['rotation_rpy']).apply([-1., 0., 0.])[:2]
            np.testing.assert_allclose(result[:2] - point[:2], outward / np.linalg.norm(outward) * .2, atol=1e-12)
            self.assertEqual(result[2], point[2])
            np.testing.assert_array_equal(point, [.4, -.2, 1.3])

    def test_rejects_invalid_standoff_or_vertical_opening(self):
        for distance in (0., -.2, float('nan')):
            with self.assertRaises(ValueError):
                portal_staging_point([0., 0., 1.], dict(rotation_rpy=[0., 0., 0.]), distance)
        with self.assertRaises(ValueError):
            portal_staging_point([0., 0., 1.], dict(rotation_rpy=[0., np.pi / 2, 0.]))


if __name__ == '__main__':
    unittest.main()
