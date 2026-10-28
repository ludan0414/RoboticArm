"""Site P1/P2 transfer contract, checked without moving hardware."""
import math
from types import SimpleNamespace as NS
import unittest

from luggage_planning.manual_carry_transfer import (
    TRANSFER_JOINTS_DEG, TRANSFER_JOINTS_RAD, TRANSFER_TYPES,
    execute_transfer, joint_line, on_p1_p2_line, target_joint_error, transfer_segments,
)
from test_place_cartesian_recovery import executor_type


class ManualTransferTests(unittest.TestCase):
    def test_exact_photographed_joints_use_taught_targets_without_ik(self):
        executor = executor_type()()
        calls = []
        executor._run_joint_goal = lambda joints, *args, **kwargs: (
            calls.append((joints, kwargs['seed_joints'])) or NS(success=True))
        executor._run_taught_joint_line = lambda joints, *args: (
            calls.append((joints, args[2])) or NS(success=True))
        executor._ik_joints = lambda *args: self.fail('P1/P2 must not invoke IK')
        self.assertEqual(TRANSFER_JOINTS_DEG, (
            (212.133, -104.860, -89.370, -71.821, 91.730, 38.169),
            (276.291, -68.652, -122.978, -75.087, 87.203, 102.188)))
        for index, segment in enumerate(transfer_segments()):
            seed = [index] * 6
            result = executor._execute_segment_inner(NS(type=segment.type), None, 45, seed)
            self.assertTrue(result.success)
            self.assertEqual(calls[-1][1], seed)
            for actual, expected in zip(calls[-1][0], TRANSFER_JOINTS_DEG[index]):
                self.assertAlmostEqual(math.degrees(actual), expected)

    def test_order_checkpoints_and_failure_preserves_held_box(self):
        calls, checkpoints = [], []
        class Ports:
            def graph_health(self): return True, 'ready'
            def di0(self): return 1
            def execute_segment(self, segment, _):
                calls.append(segment.type)
                return NS(ok=segment.type != TRANSFER_TYPES[0], message='collision')
            def log_event(self, event): pass
            def vacuum_release(self): self.fail('must never release')
        ok, reason = execute_transfer(Ports(), transfer_segments(), checkpoint=checkpoints.append)
        self.assertFalse(ok)
        self.assertIn('collision', reason)
        self.assertEqual(calls, [TRANSFER_TYPES[0]])
        self.assertEqual(checkpoints, [0])

    def test_resumes_at_unfinished_p2_and_checks_di0_before_moving(self):
        calls = []
        class Ports:
            def graph_health(self): return True, 'ready'
            def di0(self): return 1
            def execute_segment(self, segment, _):
                calls.append(segment.type)
                return NS(ok=True, message='done')
            def log_event(self, event): pass
        ok, _ = execute_transfer(Ports(), transfer_segments(), start=1)
        self.assertTrue(ok)
        self.assertEqual(calls, [TRANSFER_TYPES[1]])
        Ports.di0 = lambda self: 0
        ok, _ = execute_transfer(Ports(), transfer_segments())
        self.assertFalse(ok)
        self.assertEqual(calls, [TRANSFER_TYPES[1]])

    def test_actual_joint_arrival_never_accepts_a_wrapped_full_turn(self):
        target = list(TRANSFER_JOINTS_RAD[1])
        self.assertEqual(target_joint_error(TRANSFER_TYPES[1], target), 0.)
        target[0] -= 2 * math.pi
        self.assertGreater(target_joint_error(TRANSFER_TYPES[1], target), 6.)

    def test_p1_to_p2_joint_line_is_monotone_with_one_degree_samples(self):
        line = joint_line(TRANSFER_JOINTS_RAD[0], TRANSFER_JOINTS_RAD[1])
        self.assertEqual(line[0], TRANSFER_JOINTS_RAD[0])
        self.assertEqual(line[-1], TRANSFER_JOINTS_RAD[1])
        for joint in range(6):
            changes = [b[joint]-a[joint] for a,b in zip(line,line[1:])]
            self.assertTrue(all(abs(v) <= math.radians(1.000001) for v in changes))
            self.assertTrue(all(v >= 0 for v in changes) or all(v <= 0 for v in changes))
        self.assertTrue(on_p1_p2_line(TRANSFER_JOINTS_RAD[0]))
        self.assertTrue(on_p1_p2_line(line[len(line)//2]))
        off=list(line[len(line)//2]);off[3]+=.2
        self.assertFalse(on_p1_p2_line(off))


if __name__ == '__main__':
    unittest.main()
