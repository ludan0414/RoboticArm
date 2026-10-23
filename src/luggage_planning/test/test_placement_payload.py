import unittest

from luggage_planning.placement_payload import release_above_support
from luggage_planning.policy_place import parse_place_target
from luggage_planning.pose import Point


class SupportGapTest(unittest.TestCase):
    def test_release_gap_follows_tilted_support_normal_without_changing_dimensions(self):
        target = parse_place_target(dict(frame_id='world', pose_reference='box_center',
            position_m=[1,2,3], orientation_xyzw=[0,2**-.5,0,2**-.5],
            via_box_poses=[dict(position_m=[1,2,4], orientation_xyzw=[0,0,0,1])]))
        result = release_above_support(target)
        self.assertAlmostEqual(result.box_pose.position.x,1.005)
        self.assertAlmostEqual(result.box_pose.position.z,3)
        self.assertAlmostEqual(result.via[0].position.z,4.005)
        self.assertEqual(result.box_pose.orientation,target.box_pose.orientation)
        self.assertEqual(result.reference_box_size_m,target.reference_box_size_m)
        self.assertEqual(target.box_pose.position,Point(1,2,3))
