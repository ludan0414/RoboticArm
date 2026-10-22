"""Behavioural checks for policy pose conversion and place failure boundaries."""
import math
from types import SimpleNamespace
import unittest

from luggage_planning.policy_place import (
    build_place_segments, connect_place_from_pose, parse_place_target, run_place,
)
from luggage_planning.pose import Point, Pose, Quaternion, MotionSegment


class PickupConnectionTests(unittest.TestCase):
    def test_actual_pickup_connects_without_changing_remaining_route(self):
        actual = Pose(Point(0, 1.443, .3), Quaternion())
        first = MotionSegment(name="place_via_0", type="cartesian",
            target_pose=Pose(Point(.3, 1.0, .86), Quaternion(0, 0, 1, 0)),
            required_cartesian_fraction=1.0)
        tail = MotionSegment(name="place_descend")
        result = connect_place_from_pose(actual, [first, tail])
        self.assertGreater(len(result), 2)
        self.assertIs(result[-2], first)
        self.assertIs(result[-1], tail)
        previous = actual
        for segment in result[:-1]:
            pose = segment.target_pose
            self.assertLessEqual(math.dist(vars(previous.position).values(), vars(pose.position).values()), .150001)
            dot = abs(sum(a*b for a, b in zip(vars(previous.orientation).values(), vars(pose.orientation).values())))
            self.assertLessEqual(2 * math.acos(min(1, dot)), math.radians(20) + 1e-6)
            self.assertFalse(segment.allow_ompl_fallback)
            self.assertEqual(segment.required_cartesian_fraction, 1.0)
            previous = pose


def target(**overrides):
    data = dict(frame_id="world", pose_reference="box_center",
                position_m=[1, 2, 0.3], orientation_xyzw=[0, 0, 0, 1])
    data.update(overrides)
    return parse_place_target(data)


def fixture():
    detection = SimpleNamespace(
        frame="world", box_xyz=(0, 0, 0.2), box_quat=(0, 0, 0, 1),
        detection_yaw=0, yaw_valid=True)
    candidate = SimpleNamespace(
        frame="world", contact=Pose(Point(0.1, 0, 0.4), Quaternion()))
    return detection, candidate


class Ports:
    def __init__(self, fail="", di0_after_release=0):
        self.calls, self.events = [], []
        self.fail = fail
        self.released = False
        self.di0_after_release = di0_after_release
        self.time = 0.0

    def now(self):
        return self.time

    def sleep(self, dt):
        self.time += dt

    def log_event(self, event):
        self.events.append(event)

    def graph_health(self):
        return self.fail != "graph", "graph"

    def di0(self):
        return self.di0_after_release if self.released else 1

    def execute_segment(self, segment, candidate_id):
        self.calls.append(segment.name)
        return SimpleNamespace(ok=segment.name != self.fail, message=self.fail)

    def vacuum_release(self, candidate_id):
        self.calls.append("release")
        if self.fail == "release":
            return False, "release failed"
        self.released = True
        return True, "ok"

    def scene_remove(self):
        self.calls.append("detach")
        return self.fail != "detach", "detach"


class PolicyPlaceTests(unittest.TestCase):
    def test_rejects_ambiguous_or_invalid_input(self):
        for invalid in (
            dict(frame_id="container"), dict(pose_reference="tcp"),
            dict(position_m=[0, 1]), dict(position_m=[0, 0, float("nan")]),
            dict(orientation_xyzw=[0, 0, 0, 0]), dict(clearance_m=-1),
        ):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                target(**invalid)

    def test_off_center_grasp_rotates_with_box(self):
        detection, candidate = fixture()
        half = math.sqrt(0.5)
        segments = build_place_segments(target(orientation_xyzw=[0, 0, half, half]),
                                        detection, candidate)
        contact = segments[-2].target_pose
        self.assertAlmostEqual(contact.position.x, 1.0)
        self.assertAlmostEqual(contact.position.y, 2.1)
        self.assertAlmostEqual(contact.position.z, 0.5)
        self.assertAlmostEqual(segments[0].target_pose.position.z, 0.65)
        self.assertAlmostEqual(abs(contact.orientation.x), half)
        self.assertAlmostEqual(abs(contact.orientation.y), half)

    def test_full_rotation_and_nonidentity_pick(self):
        detection, candidate = fixture()
        half = math.sqrt(0.5)
        detection.box_quat = (0, 0, half, half)
        # World +X contact offset becomes box-local -Y. A 90 deg rotation
        # about X at the destination turns it into world -Z.
        segments = build_place_segments(target(orientation_xyzw=[half, 0, 0, half]),
                                        detection, candidate)
        p = segments[-2].target_pose.position
        self.assertAlmostEqual(p.x, 1.0)
        self.assertAlmostEqual(p.y, 1.8)
        self.assertAlmostEqual(p.z, 0.2)

    def test_policy_via_poses_precede_final_approach(self):
        via = dict(position_m=[0.5, 1, 0.8], orientation_xyzw=[0, 0, 0, 1])
        segments = build_place_segments(target(via_box_poses=[via]), *fixture())
        self.assertEqual(segments[0].name, "place_via_0")
        self.assertAlmostEqual(segments[0].target_pose.position.x, 0.6)
        self.assertTrue(all(s.required_cartesian_fraction == 1.0 for s in segments))
        self.assertTrue(all(not s.allow_ompl_fallback for s in segments))

    def test_unknown_pick_orientation_and_wrong_frames_fail_closed(self):
        for attribute, value in (("yaw_valid", False), ("frame", "base_link")):
            detection, candidate = fixture()
            setattr(detection, attribute, value)
            with self.assertRaises(ValueError):
                build_place_segments(target(), detection, candidate)

    def test_success_waits_for_release_then_detaches_and_retreats(self):
        ports = Ports()
        self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 0)
        self.assertEqual(ports.calls, ["pre_place", "place_descend", "release",
                                      "detach", "place_retreat"])
        self.assertGreaterEqual(ports.time, 0.5)

    def test_carry_failures_never_release(self):
        for stage in ("graph", "pre_place", "place_descend"):
            ports = Ports(fail=stage)
            self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 10)
            self.assertNotIn("release", ports.calls)
            self.assertNotIn("place_retreat", ports.calls)

    def test_release_failure_or_unknown_di0_never_retreats(self):
        for ports in (Ports(fail="release"), Ports(di0_after_release=1),
                      Ports(di0_after_release=None), Ports(fail="detach")):
            self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 11)
            self.assertNotIn("place_retreat", ports.calls)

    def test_retreat_failure_is_reported_after_release(self):
        ports = Ports(fail="place_retreat")
        self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 11)
        self.assertTrue(ports.released)

    def test_seal_loss_prevents_release(self):
        ports = Ports()
        ports.di0 = lambda: 0
        self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 10)
        self.assertEqual(ports.calls, [])

    def test_port_exception_preserves_vacuum_and_stops(self):
        ports = Ports()

        def crash(*args):
            raise RuntimeError("transport failure")

        ports.execute_segment = crash
        self.assertEqual(run_place(ports, build_place_segments(target(), *fixture()))[0], 10)
        self.assertNotIn("release", ports.calls)


if __name__ == "__main__":
    unittest.main()
