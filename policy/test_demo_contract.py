import json
from pathlib import Path
import tempfile
import unittest

from demo_contract import (check_bundle, freeze_bundle, generate_boxes, write_json,
                           stack_contact_heights)


class DemoContractTests(unittest.TestCase):
    def test_supported_heights_use_actual_boxes_and_bounded_counts(self):
        heights = stack_contact_heights([[.55, .36, .24], [.63, .37, .28]], 1.3)
        self.assertEqual(heights, [.24, .28, .52])
        self.assertNotIn(.48, heights)  # There is only one 24cm case.
        self.assertEqual(stack_contact_heights([[.75, .51, .32]] * 5, 1.3),
                         [.32, .64, .96, 1.28])

    def test_random_fixture_is_reproducible_and_has_ten_varied_boxes(self):
        first = generate_boxes(20261022)
        self.assertEqual(first, generate_boxes(20261022))
        self.assertNotEqual(first, generate_boxes(20261023))
        self.assertEqual(len(first["boxes"]), 10)
        self.assertGreater(len({tuple(d) for d in first["boxes"]}), 1)
        self.assertTrue(all(min(d) > 0 for d in first["boxes"]))

    def fixture(self, folder):
        write_json(folder / "input.json", generate_boxes(20261022))
        write_json(folder / "plan.json", dict(sequence=[dict(input_index=i) for i in range(10)]))
        for name in ("domain.json", "scene_tf.yaml", "validation.json", "model_manifest.json"):
            (folder / name).write_text("{}")
        (folder / "trajectory.npz").write_bytes(b"test hash payload")

    def test_changed_plan_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            self.fixture(folder)
            freeze_bundle(folder)
            self.assertEqual(len(check_bundle(folder)["sequence"]), 10)
            (folder / "plan.json").write_text("{}")
            with self.assertRaisesRegex(ValueError, "integrity mismatch"):
                check_bundle(folder)

    def test_partial_or_duplicate_plan_is_rejected_even_if_rehashed(self):
        for indices in (list(range(9)), [0] * 10):
            with self.subTest(indices=indices), tempfile.TemporaryDirectory() as name:
                folder = Path(name)
                self.fixture(folder)
                write_json(folder / "plan.json", dict(sequence=[dict(input_index=i) for i in indices]))
                freeze_bundle(folder)
                with self.assertRaises(ValueError):
                    check_bundle(folder)

    def test_unprepared_folder_cannot_replay(self):
        with tempfile.TemporaryDirectory() as name:
            with self.assertRaises(FileNotFoundError):
                check_bundle(name)


if __name__ == "__main__":
    unittest.main()
