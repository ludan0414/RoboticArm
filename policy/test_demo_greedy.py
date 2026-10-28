import math
from types import SimpleNamespace
import unittest
from demo_contract import generate_boxes, stack_contact_heights, domain_y_positions
from generate_suitcase_inputs import generate_identical_boxes
from demo_greedy import greedy_sequence
from demo_feeder import FEEDER_CENTER_M
from online_packing_real import Hull


class Workspace:
    config = dict(static_transforms=[dict(child="container_link", rotation_rpy=[0, 0, 0])])

    def box_yaw(self, source, dims):
        return 0 if tuple(source) == tuple(dims) else -math.pi / 2

    def route_is_feasible(self, route, dims, yaw, hull):
        return True


class GreedyTests(unittest.TestCase):
    def test_fixed55_boxes_use_explicit_outside_staging_point(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        boxes = generate_identical_boxes([55, 35, 20])["boxes"]
        poses = [dict(contact_local_m=[x, y, z], contact_world_m=[x, y, z],
                      box_yaw_rad=yaw, joint_q=[0] * 6)
                 for x in (.3, .45, .6) for y in domain_y_positions(boxes)
                 for z in stack_contact_heights(boxes)
                 for yaw in (0, -math.pi / 2)]
        feeder = (-.345, .5, .55)
        rows = greedy_sequence(boxes, hull, dict(poses=poses), Workspace(),
                               progress=lambda _: None, feeder_center=feeder)
        self.assertEqual(len(rows), 10)
        self.assertEqual(sorted(row["input_index"] for row in rows), list(range(10)))
        for row in rows:
            lower = row["route"][0]
            self.assertLessEqual(lower[0] + row["placed_dims_m"][0], -.05)
            self.assertEqual(row["spawn_center_container_door_floor_m"], list(feeder))

    def test_ten_unique_inputs_have_certified_destinations(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        poses = [dict(contact_local_m=[x, y, z], contact_world_m=[x, y, z],
                      box_yaw_rad=yaw, joint_q=[0] * 6)
                 for x in (.25, .75) for y in (.3, .9, 1.3)
                 for z in (.25, .3, .5, .55, .6, .75, .8, .85, .9, 1.0)
                 for yaw in (0, -math.pi / 2)]
        rows = greedy_sequence(generate_boxes(20261022)["boxes"], hull, dict(poses=poses),
                               Workspace(), progress=lambda _: None)
        self.assertEqual(sorted(r["input_index"] for r in rows), list(range(10)))
        depths = [r["center_container_door_floor_m"][0] for r in rows]
        self.assertTrue(all(a + 1e-6 >= b for a, b in zip(depths, depths[1:])))
        for row in rows:
            self.assertIn(row["domain_certificate"], range(len(poses)))
            self.assertEqual(len(row["orientation_world_xyzw"]), 4)
            self.assertTrue(row["route"])
            centre = [row["route"][0][k] + row["placed_dims_m"][k] / 2 for k in range(3)]
            self.assertTrue(all(abs(a - b) < 1e-8 for a, b in zip(centre, FEEDER_CENTER_M)))
            # Outside lateral alignment must use the original insertion rail,
            # keeping the complete payload 10 cm outside the door.
            for a, b in zip(row["route"], row["route"][1:]):
                if abs(a[1] - b[1]) > 1e-6 and a[0] < 0 and b[0] < 0:
                    self.assertLessEqual(a[0] + row["placed_dims_m"][0], -.1 + 1e-6)

    def test_deep_small_box_precedes_shallow_large_box(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        # Only the smaller second input fits the deep point (large one extends
        # past the back wall); input/footprint ordering must not win over depth.
        poses = [dict(contact_local_m=[x, .5, .3], contact_world_m=[x, .5, .3],
                      box_yaw_rad=0, joint_q=[0] * 6) for x in (.4, 1.3)]
        rows = greedy_sequence([[.5, .4, .3], [.3, .3, .3]], hull,
                               dict(poses=poses), Workspace(), progress=lambda _: None)
        self.assertEqual([r["input_index"] for r in rows], [1, 0])

    def test_supported_deep_stack_precedes_shallow_floor(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        poses = [dict(contact_local_m=[x, .5, z], contact_world_m=[x, .5, z],
                      box_yaw_rad=0, joint_q=[0] * 6)
                 for x, z in ((.75, .3), (.75, .6), (.25, .3))]
        rows = greedy_sequence([[.4, .3, .3]] * 2, hull, dict(poses=poses),
                               Workspace(), progress=lambda _: None)
        self.assertEqual([r["center_container_door_floor_m"][0] for r in rows], [.75, .75])
        self.assertAlmostEqual(rows[1]["route"][-1][2], .3)

    def test_unknown_domain_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "GREEDY_BLOCKED"):
            greedy_sequence([[.5, .4, .3]], None, dict(poses=[]), Workspace(), progress=lambda _: None)

    def test_partial_plan_requires_opt_in_and_keeps_selected_boxes(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        point = dict(contact_local_m=[.25, .5, .3], contact_world_m=[.25, .5, .3],
                     box_yaw_rad=0, joint_q=[0] * 6)
        boxes = [[.4, .3, .3]] * 2
        with self.assertRaisesRegex(RuntimeError, "GREEDY_BLOCKED"):
            greedy_sequence(boxes, hull, dict(poses=[point]), Workspace(), progress=lambda _: None)
        rows = greedy_sequence(boxes, hull, dict(poses=[point]), Workspace(),
                               progress=lambda _: None, allow_partial=True)
        self.assertEqual([r['input_index'] for r in rows], [0])


if __name__ == "__main__":
    unittest.main()
