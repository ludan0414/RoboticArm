import math
from types import SimpleNamespace
import unittest

from luggage_planning.commissioning_home import HOME_JOINTS_DEG, HOME_JOINTS_RAD, HOME_SEGMENT_TYPE
from luggage_planning.pick_return_path import PickReturnPath
from luggage_planning.pose import Point, Pose
from test_place_cartesian_recovery import executor_type


class CommissioningHomeTests(unittest.TestCase):
    def test_joint_home_uses_photographed_angles_without_ik_or_turn_wrapping(self):
        executor=executor_type()()
        captured=[]
        current=[-3,-2,-1,0,1,0]
        def run(positions,feedback,timeout,note,**kwargs):
            captured.append((positions,kwargs['seed_joints']))
            return SimpleNamespace(success=True)
        executor._run_joint_goal=run
        executor._ik_joints=lambda *args: self.fail('fixed home must bypass IK')
        result=executor._execute_segment_inner(
            SimpleNamespace(type=HOME_SEGMENT_TYPE),None,45,current)
        self.assertTrue(result.success)
        self.assertEqual(HOME_JOINTS_DEG,(188.299,-135.510,-95.754,-36.132,91.924,12.500))
        self.assertEqual(captured[0][1],current)
        for actual,expected in zip(captured[0][0],HOME_JOINTS_DEG):
            self.assertAlmostEqual(math.degrees(actual),expected)
        self.assertGreater(captured[0][0][0],math.pi)

    def test_failed_joint_plan_is_returned_without_a_pose_fallback(self):
        executor=executor_type()()
        failed=SimpleNamespace(success=False,message='collision')
        executor._run_joint_goal=lambda *args,**kwargs: failed
        executor._run_pose_target=lambda *args: self.fail('no TCP fallback')
        self.assertIs(executor._execute_segment_inner(
            SimpleNamespace(type=HOME_SEGMENT_TYPE),None,45,[0]*6),failed)

    def test_reverse_restores_each_original_point_and_failure_consumes_nothing(self):
        path=PickReturnPath()
        path.record('pick_retreat',Pose(Point(0,0,1)))
        for i in range(1,54):
            path.record('place_via_%d'%i,Pose(Point(i,0,1)))
        points=[]
        while path.poses:
            segment=path.next_segment()
            self.assertEqual(path.next_segment(),segment)
            self.assertEqual(segment.waypoints,[])
            self.assertEqual(segment.required_cartesian_fraction,1)
            points.append(segment.target_pose.position.x)
            path.settled()
        self.assertEqual(points,list(range(53,-1,-1)))
