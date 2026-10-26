"""Actual planner timing -> actual ServoJ grid, without ROS or hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace
import math
import unittest

from luggage_planning.trajectory_timing import time_parameterize_cartesian
from luggage_planning.trajectory_smoothing import smooth_timing, hermite_peaks, execution_timeout
from elfin_trajectory_executor.servo_j import fjt_to_servo_j_deg, limit_servo_braking


def q(x, y=0.):
    return [x, y, 0., 0., 0., 0.]


class SmoothTimingTests(unittest.TestCase):
    def test_executor_tempo_setter_validates_before_mutating(self):
        path = Path(__file__).resolve().parents[1] / 'luggage_planning/motion_executor.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MotionExecutor')
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == 'set_speed_scale')
        scope = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        executor = SimpleNamespace()
        scope[method.name](executor, .35)
        with self.assertRaises(ValueError):
            scope[method.name](executor, 2.)
        self.assertEqual(executor._speed_scale, .35)

    def test_corners_have_bounded_hermite_acceleration_and_continuous_velocity(self):
        positions = [q(0), q(.1), q(.2), q(.2, .1), q(.2, .2)]
        base = time_parameterize_cartesian(positions, .3, .3)
        timed = smooth_timing(positions, base)
        velocity, acceleration = hermite_peaks(positions, timed)
        self.assertLessEqual(acceleration, math.radians(20))
        self.assertLessEqual(velocity, base.v_max)
        self.assertTrue(any(abs(v) > 0 for v in timed.velocities_rad[2]))
        grid, reason = fjt_to_servo_j_deg(positions, timed.times_s, dt_s=.02,
                                         velocities_rad=timed.velocities_rad)
        self.assertIsNone(reason)
        # No executor-side time extension is needed for this bounded curve.
        self.assertEqual(limit_servo_braking(grid, .02), grid)

    def test_speed_override_scales_full_time_law_without_changing_curve(self):
        positions = [q(0), q(.1), q(.2)]
        base = time_parameterize_cartesian(positions, .3, .3)
        normal = smooth_timing(positions, base)
        slow = smooth_timing(positions, base, .5)
        self.assertAlmostEqual(slow.duration, 2*normal.duration)
        self.assertEqual(slow.times_s, [2*t for t in normal.times_s])
        for left, right in zip(normal.velocities_rad, slow.velocities_rad):
            self.assertEqual(right, [v/2 for v in left])
        for value in (0., 1.1, float('nan'), True, '0.5'):
            with self.assertRaises(ValueError):
                smooth_timing(positions, base, value)

    def test_production_retimer_removes_duplicate_knots_before_interpolation(self):
        path = Path(__file__).resolve().parents[1] / 'luggage_planning/motion_executor.py'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == 'apply_rest_to_rest_to_joint_trajectory')
        duration = lambda t: SimpleNamespace(sec=int(t), nanosec=int((t-int(t))*1e9))
        scope = dict(time_parameterize_cartesian=time_parameterize_cartesian, _sec_to_duration=duration, math=math)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), scope)
        trajectory = SimpleNamespace(points=[SimpleNamespace(positions=row) for row in [q(0), q(.1), q(.1), q(.2)]])
        result = scope[function.name](trajectory, .3, .3, .5)
        self.assertEqual([p.positions for p in trajectory.points], [q(0), q(.1), q(.2)])
        self.assertLess(result.duration, 20.)
        self.assertGreater(execution_timeout(SimpleNamespace(joint_trajectory=trajectory), 1.), result.duration)
        for positions in ([], q(float('nan')), q(float('inf'))):
            invalid = SimpleNamespace(points=[SimpleNamespace(positions=q(0)), SimpleNamespace(positions=positions)])
            with self.assertRaises(ValueError):
                scope[function.name](invalid, .3, .3)

    def test_wait_budget_covers_deliberately_slow_action(self):
        traj = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[SimpleNamespace(
            time_from_start=SimpleNamespace(sec=200, nanosec=0))]))
        self.assertEqual(execution_timeout(traj, 90.), 310.)

    def test_hard_joint_corner_is_limited_in_all_signed_directions(self):
        path = [q(i*.02) for i in range(20)]
        path += [q(.38, i*.02) for i in range(1, 20)]
        path += [q(.38-i*.02, .38) for i in range(1, 20)]
        result = limit_servo_braking(path, .02)
        velocities = [[0.]*6] + [[(b[j]-a[j])/.02 for j in range(6)]
                                 for a,b in zip(result, result[1:])] + [[0.]*6]
        peak = max(abs(b[j]-a[j])/.02 for a,b in zip(velocities, velocities[1:]) for j in range(6))
        self.assertLessEqual(peak, 20.0001)
        self.assertEqual(result[0], path[0])
        self.assertEqual(result[-1], path[-1])


if __name__ == '__main__':
    unittest.main()
