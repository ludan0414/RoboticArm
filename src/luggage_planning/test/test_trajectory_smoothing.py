"""Actual planner timing -> actual ServoJ grid, without ROS or hardware."""
import ast
import json
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
        self.assertTrue(any(abs(v) > 0 for v in timed.velocities_rad[1]))
        # A right-angle corner has no common tangent. Stop there without
        # slowing all the collinear portions to its speed.
        self.assertEqual(timed.velocities_rad[2], [0.]*6)
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

    def test_recorded_carry_is_twice_as_fast_without_executor_dilation(self):
        fixture = Path(__file__).parent / 'data/carry_speed_replay.json'
        cases = json.loads(fixture.read_text())['cases']
        previous_total = new_total = 0.
        for case in cases:
            with self.subTest(case=case['name']):
                positions = case['positions']
                timed = smooth_timing(positions, time_parameterize_cartesian(positions, .3, .3))
                velocity, acceleration = hermite_peaks(positions, timed)
                self.assertLessEqual(velocity, 1.57*.3)
                self.assertLessEqual(acceleration, math.radians(20))
                grid, reason = fjt_to_servo_j_deg(positions, timed.times_s, dt_s=.02,
                                                velocities_rad=timed.velocities_rad)
                self.assertIsNone(reason)
                self.assertEqual(limit_servo_braking(grid, .02), grid)
                self.assertEqual(len(timed.times_s), len(positions))
                if case['name'] == 'pre_place':
                    self.assertLessEqual(timed.duration, case['previous_duration_s']/2)
                previous_total += case['previous_duration_s']
                new_total += timed.duration
        self.assertLessEqual(new_total, previous_total/2)

    def test_local_corner_interpolation_has_no_joint_overshoot(self):
        positions = [q(0), q(.1), q(.2), q(.2, .1), q(.2, .2)]
        timed = smooth_timing(positions, time_parameterize_cartesian(positions, .3, .3))
        for i, (before, after) in enumerate(zip(positions, positions[1:])):
            duration = timed.times_s[i+1]-timed.times_s[i]
            for j, (a, b) in enumerate(zip(before, after)):
                for sample in range(101):
                    u = sample/100
                    value = ((2*u**3-3*u**2+1)*a + (-2*u**3+3*u**2)*b
                             + (u**3-2*u**2+u)*duration*timed.velocities_rad[i][j]
                             + (u**3-u**2)*duration*timed.velocities_rad[i+1][j])
                    self.assertGreaterEqual(value, min(a,b)-1e-12)
                    self.assertLessEqual(value, max(a,b)+1e-12)

    def test_local_iteration_failure_uses_bounded_uniform_fallback(self):
        from unittest.mock import patch
        positions = [q(0), q(.1), q(.2, .1)]
        base = time_parameterize_cartesian(positions, .3, .3)
        with patch('luggage_planning.trajectory_smoothing._local_timing', return_value=None):
            timed = smooth_timing(positions, base)
        velocity, acceleration = hermite_peaks(positions, timed)
        self.assertLessEqual(velocity, base.v_max)
        self.assertLessEqual(acceleration, math.radians(20))

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
