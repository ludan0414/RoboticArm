"""Exercise the executor's reach fallback, contact exclusions and stop paths."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from luggage_planning.pose import Pose, Point, MotionSegment
from luggage_planning.reach_limit import closest_transit_pose


class ReachLimitTests(unittest.TestCase):
    def executor(self, error=-31):
        path=Path(__file__).parents[1]/'luggage_planning/motion_executor.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='MotionExecutor')
        cls.body=[node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='_run_pose_target']
        scope={}
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
        executor=scope['MotionExecutor']()
        calls=[]
        def ik(pose,seed):
            executor._last_ik_code=1 if pose.position.x <= 2 else error
            return [pose.position.x]*6 if pose.position.x <= 2 else None
        executor._ik_joints=ik
        executor._fk_pose=lambda joints:Pose(Point())
        executor._notify=lambda *a:None
        executor._wrist_blocked=lambda *a:False
        executor._result=lambda ok,message,**kw:NS(success=ok,message=message)
        executor._run_joint_goal=lambda joints,*a,**kw:(calls.append(joints) or NS(success=True))
        return executor,calls

    def test_transit_extends_to_last_reachable_point_without_mutating_request(self):
        executor,calls=self.executor()
        segment=MotionSegment(name='pre_place',type='pose_target',target_pose=Pose(Point(3,0,0)))
        self.assertTrue(executor._run_pose_target(segment,None,45,[0]*6).success)
        self.assertAlmostEqual(calls[0][0],2.,delta=.005)
        self.assertEqual(segment.target_pose.position.x,3)
        self.assertAlmostEqual(executor._reach_projections[0]['shortfall_m'],1.,delta=.005)

    def test_contacts_returns_and_non_ik_errors_cannot_be_shortened(self):
        for name,error in [('pick_attach',-31),('place_descend',-31),('return_pick_stop_0',-31),('pre_place',-10)]:
            executor,calls=self.executor(error)
            segment=MotionSegment(name=name,target_pose=Pose(Point(3,0,0)))
            self.assertFalse(executor._run_pose_target(segment,None,45,[0]*6).success)
            self.assertEqual(calls,[])

    def test_cancellation_never_executes_the_best_previous_probe(self):
        executor,calls=self.executor()
        executor._cancel_check=lambda:True
        segment=MotionSegment(name='pre_place',target_pose=Pose(Point(3,0,0)))
        self.assertFalse(executor._run_pose_target(segment,None,45,[0]*6).success)
        self.assertEqual(calls,[])

    def test_same_point_with_unreachable_attitude_does_not_claim_extension(self):
        self.assertIsNone(closest_transit_pose(Pose(),Pose(),lambda p:[0]*6))

    def test_recovery_records_achieved_hops_for_reverse_route(self):
        from luggage_planning.cartesian_recovery import replan_complete_route
        executor,calls=self.executor()
        executor._reach_projections=[]
        executor._cartesian_recovery=[]
        def execute(joints,*args,**kw):
            executor._planned_traj=NS(joint_trajectory=NS(joint_names=['j1'],
                points=[NS(positions=[joints[0]], velocities=[], accelerations=[],
                    time_from_start=NS(sec=1,nanosec=0))]))
            return NS(success=True,message='done')
        executor._run_joint_goal=execute
        executor._fk_pose=lambda joints:Pose(Point(joints[0],0,0))
        segment=MotionSegment(name='place_via_continuous',target_pose=Pose(Point(1.8,0,0)),
                              waypoints=[Pose(Point(1,0,0)), Pose(Point(3,0,0)),
                                         Pose(Point(3.2,0,0))])
        result=replan_complete_route(executor,segment,None,45,[0]*6,.5)
        self.assertTrue(result.success)
        achieved=[item['achieved_pose']['position']['x'] for item in executor._cartesian_recovery
                  if item.get('success')]
        self.assertEqual(achieved[0],1.)
        self.assertAlmostEqual(achieved[1],2.,delta=.005)
        self.assertTrue(executor._cartesian_recovery[2]['skipped_unreachable_after_boundary'])
        self.assertAlmostEqual(achieved[-1],1.8)
        self.assertEqual(segment.target_pose.position.x,1.8)

    def test_final_target_is_never_shortened_after_boundary(self):
        from luggage_planning.cartesian_recovery import replan_complete_route
        executor,_=self.executor()
        executor._reach_projections=[]
        def execute(joints,*args,**kw):
            executor._planned_traj=NS(joint_trajectory=NS(joint_names=['j1'],
                points=[NS(positions=[joints[0]], velocities=[], accelerations=[],
                    time_from_start=NS(sec=1,nanosec=0))]))
            return NS(success=True,message='done')
        executor._run_joint_goal=execute
        executor._fk_pose=lambda joints:Pose(Point(joints[0],0,0))
        segment=MotionSegment(name='pre_place',target_pose=Pose(Point(3.2,0,0)),
                              waypoints=[Pose(Point(3,0,0))])
        result=replan_complete_route(executor,segment,None,45,[0]*6,.5)
        self.assertFalse(result.success)
        self.assertIn('IK failed',result.message)
        self.assertEqual(len(executor._reach_projections),1)

    def test_reach_boundary_lowers_in_place_before_retrying_final_target(self):
        from luggage_planning.cartesian_recovery import replan_complete_route
        from luggage_planning.reach_limit import pose_dict
        executor,_=self.executor()
        executor._reach_projections=[]
        executor._fk_pose=lambda joints:Pose(Point(joints[0],0,joints[1]))
        calls=[]
        def move(hop, feedback, timeout, seed, allow_reach_projection=True):
            p=hop.target_pose.position
            calls.append((p.x,p.z,allow_reach_projection))
            if p.x > 2 and p.z > .5 and allow_reach_projection:
                actual=Pose(Point(2,0,p.z))
                executor._reach_projections.append(dict(achieved=pose_dict(actual)))
            elif p.x > 2 and p.z > .5:
                executor._last_ik_code=-31
                return NS(success=False,message='IK failed error_code=-31')
            else:
                actual=hop.target_pose
            executor._last_ik_code=1
            executor._planned_traj=NS(joint_trajectory=NS(joint_names=['x','z'],
                points=[NS(positions=[actual.position.x,actual.position.z], velocities=[],
                    accelerations=[],time_from_start=NS(sec=1,nanosec=0))]))
            return NS(success=True,message='done')
        executor._run_pose_target=move
        segment=MotionSegment(name='pre_place',target_pose=Pose(Point(3,0,.4)),
            waypoints=[Pose(Point(3,0,1)),Pose(Point(3,0,.7))])
        result=replan_complete_route(executor,segment,None,45,[1,1],.5)
        self.assertTrue(result.success)
        self.assertEqual(calls,[(3,1,True),(3,.7,False),(2,.7,False),(3,.4,False)])
        self.assertTrue(executor._cartesian_recovery[1]['boundary_lowering'])
        self.assertEqual(executor._cartesian_recovery[1]['achieved_pose']['position']['x'],2)

    def test_action_adapter_replaces_requested_waypoints_before_return_recording(self):
        import json
        from luggage_planning.reach_limit import pose_dict
        path=Path(__file__).parents[1]/'scripts/hardware_pick_driver.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='RosPickSessionPorts')
        cls.bases=[]
        cls.body=[node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='execute_segment']
        scope=dict(json=json,PlanMotion=NS(Goal=lambda:NS()),
            adapters=NS(segment_to_msg=lambda segment:segment),
            SegmentOutcome=lambda **kw:NS(**kw))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
        ports=scope['RosPickSessionPorts']()
        observed=[]
        payload=json.dumps(dict(reach_projections=[dict(shortfall_m=1.)],
            executed_transit_poses=[pose_dict(Pose(Point(1,0,0))),pose_dict(Pose(Point(2,0,0)))]))
        ports._driver=NS(segment_guard=None,report_stage=lambda *a:None,_plan=None,
            _args=NS(plan_timeout=45),record_completed_motion=observed.append,
            _send_action=lambda *a:(True,'done',NS(fraction=1.,settle_json=payload)))
        ports.log_event=lambda *a:None
        original=MotionSegment(name='pre_place',type='cartesian',target_pose=Pose(Point(3,0,0)),
                               waypoints=[Pose(Point(2.5,0,0))])
        self.assertTrue(ports.execute_segment(original,'box').ok)
        self.assertEqual([pose.position.x for pose in observed[0].waypoints],[1.])
        self.assertEqual(original.waypoints[0].position.x,2.5)


if __name__=='__main__':unittest.main()
