"""Exercise the real executor control flow without requiring ROS on the host."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


def executor_type():
    source = Path(__file__).parents[1] / "luggage_planning/motion_executor.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    methods = {"_execute_segment_inner", "_plan_cartesian"}
    owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MotionExecutor")
    owner.body = [n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    def request():
        return NS(header=NS(), start_state=NS(is_diff=False, joint_state=NS()))
    scope = {"GetCartesianPath": NS(Request=request), 'math': math}
    exec(compile(ast.Module(body=[owner], type_ignores=[]), str(source), "exec"), scope)
    return scope["MotionExecutor"]


class PlaceRecoveryTests(unittest.TestCase):
    def test_frozen_paths_never_project_replan_or_execute_partial_prefix(self):
        for name in ('pre_place','place_descend','place_retreat'):
            for fraction,plan in ((.98,NS()),(1.,NS(occupancy_blocked=True)),
                                  (1.,NS(wrist_singular=True))):
                executor=self.setup_executor()
                executor._plan_cartesian=lambda *a,f=fraction,p=plan:(f,p)
                executor._run_pose_target=lambda *a:self.fail('frozen route was replanned')
                executor._execute_trajectory=lambda *a:self.fail('rejected frozen route moved')
                segment=NS(name=name,type='offline_frozen_cartesian',target_pose=NS(),
                           waypoints=[NS()],allow_ompl_fallback=True)
                result=executor._execute_segment_inner(segment,None,45,[0.]*6)
                self.assertFalse(result.success)
                self.assertIn('OFFLINE_ROUTE_RECHECK_REQUIRED',result.message)

    def test_complete_frozen_path_executes_the_original_request(self):
        executor=self.setup_executor();plan=NS();seen=[]
        executor._plan_cartesian=lambda *a:(1.,plan)
        executor._execute_trajectory=lambda *a:(seen.append(a) or NS(success=True))
        segment=NS(name='pre_place',type='offline_frozen_cartesian',target_pose=NS(),waypoints=[NS()])
        self.assertTrue(executor._execute_segment_inner(segment,None,45,[0.]*6).success)
        self.assertIs(seen[0][0],plan)
        self.assertIs(seen[0][1],segment)

    def setup_executor(self):
        executor = executor_type()()
        executor._min_fraction = .95
        executor._occupancy = None
        executor._notify = lambda *a: None
        executor._result = lambda ok, message, fraction=0: NS(success=ok, message=message, fraction=fraction)
        return executor

    def test_complete_requirement_allows_complete_pose_replan(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.867, NS())
        calls = []
        executor._run_pose_target = lambda *a: (calls.append(a[0].name) or executor._result(True, "full goal reached", 1))
        segment = NS(name="pre_place", type="cartesian", target_pose=NS(), allow_ompl_fallback=True, required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 45, [0]*6)
        self.assertTrue(result.success)
        self.assertTrue(result.used_ompl_fallback)
        self.assertEqual(calls, ["pre_place"])

    def test_incomplete_descent_cannot_execute_or_release(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.867, NS())
        executor._run_pose_target = lambda *a: executor._result(False, 'full target IK failed', 0.)
        segment = NS(name="place_descend", type="cartesian", target_pose=NS(), allow_ompl_fallback=False, required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 45, [0]*6)
        self.assertFalse(result.success)
        self.assertIn("place_descend", result.message)

    def test_fraction_failure_recovers_every_waypoint_even_with_legacy_flag_off(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.792, NS())
        calls = []
        def move(segment, feedback, timeout, seed):
            calls.append((segment.target_pose, list(seed), segment.name))
            self.assertEqual(segment.waypoints, [])
            self.assertEqual(segment.type, 'pose_target')
            executor._planned_traj = NS(joint_trajectory=NS(joint_names=[], points=[
                NS(positions=[float(len(calls))]*6, time_from_start=NS(sec=1, nanosec=0))]))
            return executor._result(True, 'complete goal')
        executor._run_pose_target = move
        segment = NS(name='pre_place', type='cartesian', target_pose='finish',
                     waypoints=['door_outer', 'door_inner'], allow_ompl_fallback=False,
                     required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
        self.assertTrue(result.success)
        self.assertEqual([r[0] for r in calls], ['door_outer', 'door_inner', 'finish'])
        self.assertEqual([r[1][0] for r in calls], [0., 1., 2.])
        self.assertTrue(result.used_ompl_fallback)
        self.assertEqual(result.fraction, .792)
        self.assertEqual(len(executor._cartesian_recovery), 3)
        self.assertEqual(segment.type, 'cartesian')
        self.assertEqual(segment.waypoints, ['door_outer', 'door_inner'])

    def test_replan_stops_before_later_waypoints_when_a_full_goal_fails(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.792, NS())
        calls = []
        executor._run_pose_target = lambda segment, *args: (
            calls.append(segment.target_pose) or executor._result(False, 'collision blocked'))
        segment = NS(name='pre_place', type='cartesian', target_pose='home',
                     waypoints=['door'], allow_ompl_fallback=False)
        result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
        self.assertFalse(result.success)
        self.assertEqual(calls, ['door'])
        self.assertIn('collision blocked', result.message)

    def test_incomplete_recorded_return_never_replans_or_executes_prefix(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.792, NS())
        executor._run_pose_target = lambda *a: self.fail('return must not replan')
        segment = NS(name='return_pick_stop_0', type='cartesian', target_pose='home',
                     waypoints=['door'], allow_ompl_fallback=False)
        result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
        self.assertFalse(result.success)
        self.assertEqual(result.fraction, .792)
        self.assertIn('recorded return path incomplete', result.message)

    def test_blocked_recorded_return_never_uses_fallback_even_if_enabled(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (1., NS(occupancy_blocked=True))
        executor._run_pose_target = lambda *a: self.fail('blocked return must not replan')
        segment = NS(name='return_pick_stop_0', type='cartesian', target_pose='home',
                     waypoints=['door'], allow_ompl_fallback=True)
        result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
        self.assertFalse(result.success)
        self.assertIn('recorded return path blocked', result.message)

    def test_cancelled_route_never_starts_a_replan(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.792, NS())
        executor._cancel_check = lambda: True
        executor._run_pose_target = lambda *args: self.fail('cancelled goal moved')
        segment = NS(name='pre_place', type='cartesian', target_pose=NS())
        result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
        self.assertFalse(result.success)
        self.assertIn('cancelled', result.message)

    def test_explicit_collision_or_wrist_rejection_is_not_ignored(self):
        for plan in (NS(occupancy_blocked=True), NS(wrist_singular=True, reason='wrist branch')):
            executor = self.setup_executor()
            executor._plan_cartesian = lambda *args: (.792, plan)
            executor._run_pose_target = lambda *args: self.fail('explicit rejection ignored')
            segment = NS(name='pre_place', type='cartesian', target_pose=NS(), allow_ompl_fallback=False)
            result = executor._execute_segment_inner(segment, None, 90., [0.]*6)
            self.assertFalse(result.success)

    def test_finer_descent_plans_do_not_execute_a_partial_prefix(self):
        for fractions, expected in [([.867, .94, 1.], 1.), ([.867, .86, .85], .867)]:
            executor = self.setup_executor()
            steps = []
            executor._frame = "world"
            executor._group = "arm"
            executor._link = "tool"
            executor._max_step = .01
            executor._avoid_collisions = True
            responses = iter(NS(fraction=f) for f in fractions)
            executor._cartesian = NS(call_async=lambda req: (steps.append(req.max_step) or next(responses)))
            executor._wait_future = lambda future, timeout: future
            executor._hold_pick_joint6 = lambda *a: None
            executor._record_cartesian_excursion = lambda *a: None
            executor._reject_for_wrist = lambda *a: None
            fraction, _ = executor._plan_cartesian(NS(name="place_descend", target_pose=NS()), None)
            self.assertEqual(fraction, expected)
            self.assertEqual(steps, [.01, .005, .0025])


if __name__ == "__main__":
    unittest.main()
