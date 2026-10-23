"""Check the default frozen demo and deployment profile in robot-base coordinates."""
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from demo_contract import check_bundle
from export_hardware_loading_profile import export_profile, post_place_exit_indices, sample_poses


class LoadingObservationTests(unittest.TestCase):
    def assert_profile_close(self, actual, expected):
        # Normalizing saved quaternions can differ by one float ULP across
        # NumPy/platform versions (observed maximum 1.11e-16). Keep structure,
        # strings and list order exact; permit only 1e-12 numeric roundoff.
        if isinstance(expected, dict):
            self.assertEqual(set(actual),set(expected))
            for key in expected:
                self.assert_profile_close(actual[key],expected[key])
        elif isinstance(expected, list):
            self.assertEqual(len(actual),len(expected))
            for left,right in zip(actual,expected):
                self.assert_profile_close(left,right)
        elif isinstance(expected,float):
            self.assertAlmostEqual(actual,expected,delta=1e-12)
        else:
            self.assertEqual(actual,expected)

    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).parent
        cls.bundle = cls.root / "demo_data/site_four_colors_right160_observe120"
        cls.plan = check_bundle(cls.bundle)
        cls.profile = json.loads((cls.root.parent / "deployment_ws/config/loading_cycle.site.template.json").read_text(encoding="utf-8"))
        cls.data = np.load(cls.bundle / "trajectory.npz")
        cls.addClassCleanup(cls.data.close)
        base = cls.profile["motion_safety"]["expected_base_world_pose"]
        cls.base_position = np.array(base["position_m"])
        cls.inverse_base = Rotation.from_quat(base["orientation_xyzw"]).inv()

    def base_xyz(self, world_points):
        return self.inverse_base.apply(np.asarray(world_points) - self.base_position)

    def test_boxes_still_spawn_at_minus160_on_ground(self):
        self.assertEqual(self.plan["feeder"]["xy_m"], [-1.6, 0.0])
        for row in self.plan["sequence"]:
            mask = (self.data["box_id"] == row["id"]) & (self.data["stage"] == "SPAWN")
            poses = self.data["box_pose"][mask]
            np.testing.assert_allclose(self.base_xyz(poses[:, :3])[:, :2],
                                       np.tile([-1.6, 0], (len(poses), 1)), atol=1e-4)
            np.testing.assert_allclose(poses[:, 2] - row["input_dims_m"][2] / 2, 0, atol=1e-5)

    def test_observe_and_final_return_wait_at_minus120(self):
        self.assertEqual(self.plan["observation"]["base_xy_m"], [-1.2, 0.0])
        for row in self.plan["sequence"]:
            this_box = self.data["box_id"] == row["id"]
            observing = self.data["tool_pose"][this_box & (self.data["stage"] == "OBSERVE"), :3]
            returning = self.data["tool_pose"][this_box & (self.data["stage"] == "RETURN"), :3][-1]
            np.testing.assert_allclose(self.base_xyz(observing), np.tile([-1.2, 0, 0], (len(observing), 1)), atol=.01)
            np.testing.assert_allclose(self.base_xyz(returning), [-1.2, 0, 0], atol=.01)

    def test_deployment_profile_uses_same_observe_and_post_place_exit(self):
        self.assert_profile_close(self.profile, export_profile(self.bundle))
        np.testing.assert_allclose(self.base_xyz(self.profile["observe_tool_pose"]["position_m"]), [-1.2, 0, 0], atol=.01)
        for row, cycle in zip(self.plan["sequence"], self.profile["cycles"]):
            indices = post_place_exit_indices(self.data, row["id"])
            self.assertEqual(cycle["exit_tool_poses"], sample_poses(self.data["tool_pose"][indices]))
            np.testing.assert_allclose(self.base_xyz(cycle["exit_tool_poses"][-1]["position_m"]), [-1.2, 0, 0], atol=.01)

    def test_luggage_order_and_place_targets_are_preserved(self):
        old = json.loads((self.root / "demo_data/site_four_colors_right160/plan.json").read_text(encoding="utf-8"))
        keys = ["id", "label", "input_dims_m", "position_world_m", "orientation_world_xyzw"]
        self.assertEqual([[row[k] for k in keys] for row in self.plan["sequence"]],
                         [[row[k] for k in keys] for row in old["sequence"]])


if __name__ == "__main__":
    unittest.main()
