import unittest
import numpy as np
from demo_recording import validate_recording


class RecordingTests(unittest.TestCase):
    def fixture(self):
        sequence = [dict(id="B1", position_world_m=[0, 0, 0], orientation_world_xyzw=[0, 0, 0, 1])]
        body = np.zeros((4, 1, 7))
        body[:, :, 6] = 1
        body[0, 0, :3] = 100
        return sequence, dict(stage=np.asarray(["RESET_HOME", "SPAWN", "RELEASE", "COMPLETE"]),
                              box_id=np.asarray(["B1"] * 4), body_q=body, body_labels=np.asarray(["B1"]))

    def test_ordered_visible_and_fixed_box_passes(self):
        sequence, data = self.fixture()
        self.assertTrue(validate_recording(sequence, data)["success"])

    def test_early_spawn_is_rejected(self):
        sequence, data = self.fixture()
        data["body_q"][0, 0, :3] = 0
        with self.assertRaisesRegex(ValueError, "before its spawn"):
            validate_recording(sequence, data)

    def test_displaced_feeder_spawn_is_rejected(self):
        sequence, data = self.fixture()
        sequence[0]["spawn_position_world_m"] = [0, 0, 0]
        self.assertTrue(validate_recording(sequence, data)["success"])
        data["body_q"][1, 0, 0] = .02
        with self.assertRaisesRegex(ValueError, "fixed feeder"):
            validate_recording(sequence, data)

    def test_ground_feeder_uses_box_bottom(self):
        sequence, data = self.fixture()
        sequence[0].update(spawn_position_world_m=[0, 0, .15],
                           spawn_bottom_world_z_m=0, placed_dims_m=[.4, .3, .3])
        data["body_q"][1, 0, 2] = .15
        self.assertTrue(validate_recording(sequence, data)["success"])
        sequence[0]["spawn_bottom_world_z_m"] = .01
        with self.assertRaisesRegex(ValueError, "specified ground"):
            validate_recording(sequence, data)

    def test_released_box_motion_is_rejected(self):
        sequence, data = self.fixture()
        data["body_q"][-1, 0, 0] = .1
        with self.assertRaisesRegex(ValueError, "after release"):
            validate_recording(sequence, data)


if __name__ == "__main__":
    unittest.main()
