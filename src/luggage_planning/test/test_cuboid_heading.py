import itertools
import math
from types import SimpleNamespace
import unittest

from luggage_planning.cuboid_heading import nearest_cuboid_heading, yaw_twist
from luggage_planning.policy_place import PlaceTarget, build_place_segments, box_pose_from_grasp
from luggage_planning.pose import Point, Pose, Quaternion
from luggage_planning.suction_candidate_waypoints import quat_mul, quat_rotate


def yaw(degrees):
    angle=math.radians(degrees)/2
    return Quaternion(0,0,math.sin(angle),math.cos(angle))


def corners(pose,size):
    return {tuple(round(getattr(pose.position,k)+value,9) for k,value in zip('xyz',
                quat_rotate(pose.orientation,tuple(sign*s/2 for sign,s in zip(signs,size)))))
            for signs in itertools.product((-1,1),repeat=3)}


class CuboidHeadingTests(unittest.TestCase):
    def test_half_turn_is_removed_from_the_entire_route(self):
        via=tuple(Pose(Point(i,1,1),yaw(180)) for i in range(4))
        target=PlaceTarget(Pose(Point(4,1,.6),yaw(180)),.15,via,(.59,.37,.24))
        result,report=nearest_cuboid_heading(target,Pose())
        self.assertTrue(report['half_turn_equivalent'])
        self.assertAlmostEqual(report['original_max_yaw_deg'],180)
        self.assertAlmostEqual(report['selected_max_yaw_deg'],0)
        for before,after in zip([*via,target.box_pose],[*result.via,result.box_pose]):
            self.assertEqual(before.position,after.position)
            self.assertEqual(corners(before,(.59,.37,.24)),corners(after,(.59,.37,.24)))
        self.assertEqual(result.reference_box_size_m,target.reference_box_size_m)
        self.assertEqual(target.box_pose.orientation,yaw(180))

    def test_quarter_turn_keeps_long_and_short_axes_in_the_policy_slot(self):
        target=PlaceTarget(Pose(Point(),yaw(90)),.15)
        result,report=nearest_cuboid_heading(target,Pose())
        self.assertAlmostEqual(report['selected_max_yaw_deg'],90)
        self.assertEqual(corners(result.box_pose,(.59,.37,.24)),corners(target.box_pose,(.59,.37,.24)))
        self.assertNotEqual(corners(result.box_pose,(.59,.37,.24)),corners(Pose(),(.59,.37,.24)))

    def test_closest_equivalent_heading_handles_wrap_and_quaternion_sign(self):
        for sign in (-1,1):
            q=yaw(-170);q=Quaternion(*(sign*getattr(q,k) for k in ('x','y','z','w')))
            result,report=nearest_cuboid_heading(PlaceTarget(Pose(Point(),q),.15),Pose(Point(),yaw(170)))
            self.assertAlmostEqual(report['selected_max_yaw_deg'],20)

    def test_tilted_cuboid_preserves_up_axis_and_full_occupied_volume(self):
        tilt=Quaternion(math.sin(.015),0,0,math.cos(.015))
        q=quat_mul(yaw(180),tilt)
        target=PlaceTarget(Pose(Point(1,2,3),q),.15)
        result,_=nearest_cuboid_heading(target,Pose())
        self.assertEqual(corners(target.box_pose,(.59,.37,.24)),corners(result.box_pose,(.59,.37,.24)))
        for a,b in zip(quat_rotate(q,(0,0,1)),quat_rotate(result.box_pose.orientation,(0,0,1))):
            self.assertAlmostEqual(a,b)

    def test_offcentre_grasp_is_not_relabelled_or_pinned_after_ik(self):
        grasp=((.04,-.03,.12),Quaternion(0,1,0,0))
        target=PlaceTarget(Pose(Point(1,2,.65),yaw(180)),.15)
        selected,_=nearest_cuboid_heading(target,Pose())
        segments=build_place_segments(selected,SimpleNamespace(frame='world',yaw_valid=True),
            SimpleNamespace(frame='world'),measured_grasp=grasp)
        recovered=box_pose_from_grasp(segments[-2].target_pose,grasp)
        for axis in 'xyz':
            self.assertAlmostEqual(getattr(recovered.position,axis),getattr(selected.box_pose.position,axis))
        self.assertLess(abs(yaw_twist(recovered.orientation,selected.box_pose.orientation)),1e-8)
        self.assertEqual(grasp[0],(.04,-.03,.12))

    def test_route_that_cannot_stay_within_quarter_turn_is_rejected(self):
        target=PlaceTarget(Pose(Point(),yaw(110)),.15,(Pose(Point(),yaw(0)),))
        with self.assertRaisesRegex(ValueError,'exceeds 90deg'):
            nearest_cuboid_heading(target,Pose())
