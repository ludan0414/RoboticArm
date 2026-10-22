"""Ensure the offline gate distinguishes complete cycles from broken ones."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from verify_pick_place_run import BOX_STAGES, verify


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        events = [dict(stage=s, box_id="B1") for s in BOX_STAGES]
        events.extend([dict(stage="VERIFY_RESULT", box_id="B1", position_error_m=0,
                            rotation_error_rad=0), dict(stage="COMPLETE", box_id="B1")])
        self.events = events
        (self.path / "result.json").write_text(json.dumps(dict(success=True, placed=1)))
        (self.path / "plan.json").write_text(json.dumps([dict(id="B1")]))
        tool = np.zeros((10, 7))
        tool[:, 0] = np.arange(10) * 0.1
        tool[:, 6] = 1
        box = tool.copy()
        box[:, 2] = -0.2
        box[7:] = box[6]
        self.data = dict(joint_q=np.zeros((10, 6)), tool_pose=tool, box_pose=box,
                         box_id=np.asarray(["B1"] * 10), stage=np.asarray(BOX_STAGES),
                         attached=np.asarray([s in ("ATTACH", "LIFT", "CARRY", "PLACE")
                                              for s in BOX_STAGES]))
        self.save()

    def save(self):
        np.savez(self.path / "trajectory.npz", **self.data)
        (self.path / "stages.jsonl").write_text("\n".join(json.dumps(x) for x in self.events))

    def test_complete_cycle_passes(self):
        self.assertTrue(verify(self.path)["success"])

    def test_float32_quaternion_roundoff_is_not_grasp_slip(self):
        q = np.asarray([.3, .4, .5, .7], dtype=np.float32)
        q /= np.linalg.norm(q)
        self.data["tool_pose"] = self.data["tool_pose"].astype(np.float32)
        self.data["tool_pose"][:, 3:] = q
        self.data["box_pose"][:, 3:] = q
        self.save()
        self.assertTrue(verify(self.path)["success"])

    def test_actual_grasp_rotation_is_rejected(self):
        self.data["box_pose"][5, 3:] = [0, 0, np.sin(.1), np.cos(.1)]
        self.save()
        with self.assertRaisesRegex(ValueError, "grasp slipped"):
            verify(self.path)

    def test_pre_spawn_absent_payload_is_not_a_nonfinite_box(self):
        for key in self.data:
            prefix = self.data[key][:1].copy()
            if key == "stage":
                # NumPy must widen the string dtype for RESET_HOME.
                self.data[key] = np.r_[np.asarray(["RESET_HOME"]), self.data[key]]
                continue
            if key == "box_pose":
                prefix[:] = np.nan
            if key == "attached":
                prefix[:] = False
            self.data[key] = np.concatenate((prefix, self.data[key]))
        self.save()
        self.assertTrue(verify(self.path)["success"])

    def test_nonfinite_spawned_payload_is_rejected(self):
        self.data["box_pose"][0, 0] = np.nan
        self.save()
        with self.assertRaisesRegex(ValueError, "nonfinite payload"):
            verify(self.path)

    def test_missing_release_fails(self):
        self.events = [x for x in self.events if x["stage"] != "RELEASE"]
        self.save()
        with self.assertRaisesRegex(ValueError, "stage order"):
            verify(self.path)

    def test_slipping_payload_fails(self):
        self.data["box_pose"][5, 0] += 0.1
        self.save()
        with self.assertRaisesRegex(ValueError, "grasp slipped"):
            verify(self.path)

    def test_payload_following_arm_after_release_fails(self):
        self.data["box_pose"][8, 0] += 0.1
        self.save()
        with self.assertRaisesRegex(ValueError, "moved after release"):
            verify(self.path)


if __name__ == "__main__":
    unittest.main()
