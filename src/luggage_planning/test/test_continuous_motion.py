import copy
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

from luggage_planning.continuous_motion import continuous_carry
from luggage_planning.loading_speed import LoadingSpeed
from luggage_planning.pick_return_path import PickReturnPath
from luggage_planning.pose import MotionSegment, Pose, Point


def segment(name, x, **kwargs):
    return MotionSegment(name=name, type='cartesian', target_pose=Pose(Point(x, 0, 1)), **kwargs)


def poses(segments):
    return [p for s in segments for p in [*s.waypoints, s.target_pose]]


class ContinuousMotionTests(unittest.TestCase):
    def test_real_four_box_profile_preserves_every_carry_pose(self):
        from luggage_planning.carry_entry import CarryConfig, CarryEntry
        from luggage_planning.hardware_loading_cycle import parse_profile
        from luggage_planning.policy_place import build_place_segments
        from luggage_planning.pose import Quaternion
        root = Path(__file__).resolve().parents[3]
        profile = json.loads((root/'deployment_ws/config/loading_cycle.site.template.json').read_text(encoding='utf-8'))
        _, cycles = parse_profile(profile)
        counts = []
        for row, (target, _) in zip(profile['cycles'], cycles):
            size = row['expected_demo_input_dims_m']
            detection = SimpleNamespace(frame='world', yaw_valid=True, box_size=size)
            candidate = SimpleNamespace(frame='world')
            grasp = ((0., 0., size[2]/2), Quaternion(1., 0., 0., 0.))
            route = build_place_segments(target, detection, candidate, measured_grasp=grasp)
            entry = CarryEntry(CarryConfig(profile['carry_entry']), detection, candidate, measured_grasp=grasp)
            route = entry.connect(Pose(Point(0., 1.6, size[2]+.12), Quaternion(1., 0., 0., 0.)), route)
            result = continuous_carry(route)
            self.assertEqual(poses(route), poses(result))
            self.assertLess(len(result), len(route))
            self.assertEqual([s.name for s in result[-2:]], ['place_descend', 'place_retreat'])
            counts.append(dict(box=row['demo_box_id'], original_actions=len(route), continuous_actions=len(result)))
        print('four-box pose preservation: '+json.dumps(counts), flush=True)

    def test_all_poses_preserved_in_order_without_mutating_source(self):
        route = [segment('place_via_%d' % i, i) for i in range(40)]
        route[4].waypoints = [Pose(Point(3.5, 0, 1))]
        route += [segment('pre_place', 41), segment('place_descend', 42), segment('place_retreat', 41)]
        before = copy.deepcopy(route)
        result = continuous_carry(route)
        self.assertEqual(route, before)
        self.assertEqual(poses(result), poses(before))
        self.assertEqual([s.name for s in result], ['pre_place', 'place_descend', 'place_retreat'])
        self.assertEqual(result[0].required_cartesian_fraction, 1.)
        self.assertFalse(result[0].allow_ompl_fallback)
        result[0].waypoints[0].position.x = -99
        self.assertEqual(route, before)

    def test_lift_retract_high_lift_and_noncartesian_boundaries_stay(self):
        route = [segment('ground_entry_'+name+suffix, i)
                 for i, (name, suffix) in enumerate([
                     ('outer_lift', '_0'), ('outer_lift', ''), ('outside_retract', ''),
                     ('high_lift', '_0'), ('high_lift', '')])]
        result = continuous_carry(route)
        self.assertEqual(len(result), 3)
        self.assertEqual(poses(result), poses(route))
        route[1].type = 'pose_target'
        self.assertEqual(continuous_carry(route)[1].type, 'pose_target')

    def test_successful_return_reverses_all_carry_points_not_descent(self):
        path = PickReturnPath()
        path.record('pick_retreat', Pose(Point(0, 0, 1)))
        path.record('pre_place', Pose(Point(3, 0, 1)), [Pose(Point(i, 0, 1)) for i in (1, 2)])
        path.record('place_descend', Pose(Point(3, 0, .8)))
        reverse = path.continuous_segment()
        self.assertEqual([p.position.x for p in poses([reverse])], [3, 2, 1, 0])
        self.assertEqual(len(path.poses), 4)
        path.settled(4)
        self.assertEqual(path.poses, [])


class LoadingSpeedTests(unittest.TestCase):
    def setUp(self):
        self.events, self.calls = [], []
        self.cycle = SimpleNamespace(lock=threading.RLock(), state='WAIT_START', request_id='current',
                                     ports=SimpleNamespace(record=self.events.append))
        self.control = LoadingSpeed(self.cycle, self.apply)

    def apply(self, value):
        self.calls.append(value)
        return value

    def test_confirmed_speed_does_not_start_motion_or_change_cycle(self):
        self.assertTrue(self.control.update(35, 'current', 1))
        self.assertEqual(self.calls, [.35])
        self.assertEqual(self.control.snapshot()['percent'], 35)
        self.assertEqual(self.cycle.state, 'WAIT_START')
        self.assertEqual(self.cycle.request_id, 'current')

    def test_busy_and_stale_updates_send_nothing(self):
        self.assertFalse(self.control.update(50, 'old', 1))
        self.cycle.state = 'BUSY'
        self.assertFalse(self.control.update(50, 'current', 1))
        self.assertEqual(self.calls, [])

    def test_invalid_scale_is_rejected_before_ros_call(self):
        for value in (True, '50', 0, 9, 101, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.control.update(value, 'current', 1)
        self.assertEqual(self.calls, [])

    def test_failed_readback_is_never_reported_as_applied(self):
        self.control.update(50, 'current', 1)
        self.control.apply = lambda _: .8
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            self.control.update(40, 'current', 1)
        self.assertIsNone(self.control.snapshot()['percent'])


if __name__ == '__main__':
    unittest.main()
