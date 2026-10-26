"""Routing clearance must not change the physical hull or legacy playback."""
import copy
from pathlib import Path
import unittest

import yaml

from doorway_scene import clearance_hull
from online_packing_real import Hull


class DoorwaySceneTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'deployment_ws/config/scene_tf.loading_site.yaml'
        self.scene = yaml.safe_load(path.read_text(encoding='utf-8'))

    def test_clearance_only_narrows_route_aperture(self):
        physical = Hull.from_config(self.scene)
        before = copy.deepcopy(vars(physical))
        route = clearance_hull(physical, self.scene)
        self.assertIsNot(route, physical)
        self.assertEqual(vars(physical), before)
        for key, value in before.items():
            if key not in ('door_y0', 'door_y1', 'door_z0', 'door_z1'):
                self.assertEqual(getattr(route, key), value)
        self.assertAlmostEqual(route.door_y0 - physical.door_y0, .056)
        self.assertAlmostEqual(physical.door_y1 - route.door_y1, .056)
        self.assertAlmostEqual(route.door_z0 - physical.door_z0, .056)
        self.assertAlmostEqual(physical.door_z1 - route.door_z1, .056)

    def test_old_scene_preserves_the_original_model(self):
        del self.scene['container']['opening']['clearance_m']
        physical = Hull.from_config(self.scene)
        self.assertIs(clearance_hull(physical, self.scene), physical)


if __name__ == '__main__':
    unittest.main()
