"""Exercise the real executor control flow without requiring ROS on the host."""
import ast
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
    scope = {"GetCartesianPath": NS(Request=request)}
    exec(compile(ast.Module(body=[owner], type_ignores=[]), str(source), "exec"), scope)
    return scope["MotionExecutor"]


class PlaceRecoveryTests(unittest.TestCase):
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
        segment = NS(name="pre_place", type="cartesian", allow_ompl_fallback=True, required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 45, [0]*6)
        self.assertTrue(result.success)
        self.assertTrue(result.used_ompl_fallback)
        self.assertEqual(calls, ["pre_place"])

    def test_incomplete_descent_cannot_execute_or_release(self):
        executor = self.setup_executor()
        executor._plan_cartesian = lambda *a: (.867, NS())
        segment = NS(name="place_descend", type="cartesian", allow_ompl_fallback=False, required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 45, [0]*6)
        self.assertFalse(result.success)
        self.assertIn("place_descend", result.message)

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
