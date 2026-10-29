"""Exercise the actual service retry method using a deterministic fake solver."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from luggage_planning.ik_search import nearby_ik_seeds


class IKSearchTests(unittest.TestCase):
    def executor(self, codes):
        path = Path(__file__).parents[1]/'luggage_planning/motion_executor.py'
        tree = ast.parse(path.read_text())
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='MotionExecutor')
        owner.body = [n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name=='_ik_joints']
        names = ['j%d'%i for i in range(6)]
        def request():
            return NS(ik_request=NS(robot_state=NS(joint_state=NS())))
        scope = dict(GetPositionIK=NS(Request=request), JOINTS=names,
                     DurationMsg=lambda **kw:NS(**kw),
                     PoseStamped=lambda:NS(header=NS()), MoveItErrorCodes=NS(SUCCESS=1),
                     _wrap_near=lambda current, target:target)
        exec(compile(ast.Module(body=[owner], type_ignores=[]), str(path), 'exec'), scope)
        executor = scope['MotionExecutor']()
        calls = []
        results = iter(codes)
        def solve(req):
            calls.append(deepcopy(req))
            return NS(error_code=NS(val=next(results)), solution=NS(joint_state=NS(
                name=names,position=[.1]*6)))
        executor._ik = NS(wait_for_service=lambda **kw:True, call_async=solve)
        executor._wait_future = lambda result, timeout:result
        executor._group,executor._link,executor._frame = 'arm','tcp','world'
        executor._avoid_collisions = True
        return executor,calls

    def test_solver_exhaustion_retries_nearby_seeds_with_same_pose_and_collision_checks(self):
        executor,calls = self.executor([-31,-31,1])
        target = NS(marker='unchanged target')
        self.assertEqual(executor._ik_joints(target,[0.]*6),[.1]*6)
        self.assertEqual(len(calls),3)
        for request in calls:
            self.assertTrue(request.ik_request.avoid_collisions)
            self.assertEqual(request.ik_request.pose_stamped.pose.marker,target.marker)
            self.assertEqual(request.ik_request.timeout.sec,2)
        self.assertEqual([c.ik_request.robot_state.joint_state.position for c in calls],
                         nearby_ik_seeds([0.]*6)[:len(calls)])
        self.assertEqual([a['error_code'] for a in executor._last_ik_attempts],[-31,-31,1])

    def test_other_errors_and_cancel_never_retry(self):
        executor,calls = self.executor([-10])
        self.assertIsNone(executor._ik_joints(NS(),[0.]*6))
        self.assertEqual(len(calls),1)

    def test_full_extension_seed_recovers_after_folded_seeds_fail(self):
        executor,calls=self.executor([-31,-31,-31,1])
        current=[4.4,-1.2,-1.5,-1.7,1.55,1.4]
        self.assertIsNotNone(executor._ik_joints(NS(),current))
        straight=calls[-1].ik_request.robot_state.joint_state.position
        self.assertEqual(straight[2],0.)
        self.assertAlmostEqual(sum(straight[1:4]),sum(current[1:4]))

    def test_cancellation_stops_before_another_seed(self):
        executor,calls = self.executor([-31,1])
        checks = iter([False,True])
        executor._cancel_check = lambda:next(checks)
        self.assertIsNone(executor._ik_joints(NS(),[0.]*6))
        self.assertEqual(len(calls),1)

    def test_seeds_leave_base_and_wrist_winding_unchanged(self):
        current = [4.4,-2.5,-.5,-1.5,1.55,1.4]
        for seed in nearby_ik_seeds(current):
            self.assertEqual([seed[i] for i in (0,4,5)],[current[i] for i in (0,4,5)])
            self.assertAlmostEqual(sum(seed[1:4]),sum(current[1:4]))
        self.assertEqual(nearby_ik_seeds(None),[None])
        self.assertEqual(nearby_ik_seeds([float('nan')]*6),[None])
