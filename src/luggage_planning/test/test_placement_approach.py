import unittest

from luggage_planning.placement_approach import lower_final_approach
from luggage_planning.policy_place import parse_place_target


def pose(x,y,z):
    return dict(position_m=[x,y,z], orientation_xyzw=[0,0,0,1])


class ApproachTest(unittest.TestCase):
    def test_lowers_only_near_target_tail_and_preserves_box_orientation_and_xy(self):
        data = dict(frame_id='world', pose_reference='box_center', clearance_m=.15,
                    **pose(1,0,.6), via_box_poses=[pose(0,0,1.2),pose(.8,0,1.2),pose(1,0,1.2)])
        original = parse_place_target(data)
        result = lower_final_approach(original)
        self.assertIs(result.via[0], original.via[0])
        self.assertGreater(result.via[1].position.z,.75)
        self.assertLess(result.via[1].position.z,1.2)
        self.assertAlmostEqual(result.via[-1].position.z,.75)
        self.assertIs(result.box_pose,original.box_pose)
        for old,new in zip(original.via,result.via):
            self.assertEqual(old.position.x,new.position.x)
            self.assertEqual(old.position.y,new.position.y)
            self.assertEqual(old.orientation,new.orientation)
        self.assertEqual(original.via[-1].position.z,1.2)

    def test_earlier_pass_near_target_is_unchanged_and_low_route_is_not_raised(self):
        original = parse_place_target(dict(frame_id='world', pose_reference='box_center',
            **pose(1,0,.6), via_box_poses=[pose(1,0,1.2),pose(0,0,1.2),pose(1,0,.7)]))
        result = lower_final_approach(original)
        self.assertIs(result.via[0],original.via[0])
        self.assertEqual(result.via[-1].position.z,.7)
