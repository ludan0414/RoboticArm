from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from luggage_planning.release_height_recovery import recover_release_height
from luggage_planning.pose import MotionSegment,Pose,Point,Quaternion


class ReleaseRecoveryTests(unittest.TestCase):
    def test_only_complete_raised_descent_executes_with_same_xy_and_orientation(self):
        requested=MotionSegment(name='place_descend',type='cartesian',target_pose=Pose(Point(1,2,3),Quaternion()))
        attempts,executed=[],[]
        def plan(segment,feedback):
            attempts.append(segment.target_pose.position.z)
            return (.99 if len(attempts)==1 else 1.),NS()
        executor=NS(_occupancy=None,_notify=lambda *args:None,_plan_cartesian=plan,
            _execute_trajectory=lambda plan,segment,*args:(executed.append(deepcopy(segment)) or
                                                          NS(success=True,message='full path')))
        original=NS(moveit_error_code=-31,message='IK failed')
        result=recover_release_height(executor,requested,None,60.,original)
        self.assertTrue(result.success)
        self.assertEqual(attempts,[3.0025,3.005])
        self.assertEqual(len(executed),1)
        self.assertEqual((executed[0].target_pose.position.x,executed[0].target_pose.position.y),(1,2))
        self.assertEqual(executed[0].target_pose.orientation,requested.target_pose.orientation)
        self.assertEqual(requested.target_pose.position.z,3)

    def test_partial_prefix_or_hardware_failure_never_causes_another_motion(self):
        requested=MotionSegment(name='place_descend',target_pose=Pose(Point(0,0,1)))
        calls=[]
        executor=NS(_occupancy=None,_notify=lambda *args:None,
            _plan_cartesian=lambda *args:(.99,NS()),
            _execute_trajectory=lambda *args:self.fail('partial path executed'))
        original=NS(moveit_error_code=-31,message='IK failed')
        self.assertIs(recover_release_height(executor,requested,None,60.,original),original)
        self.assertEqual(len(executor._release_height_recovery),3)
        hardware=NS(moveit_error_code=-4,message='controller stopped')
        executor._plan_cartesian=lambda *args:self.fail('hardware failure retried')
        self.assertIs(recover_release_height(executor,requested,None,60.,hardware),hardware)
