#!/usr/bin/env python3
"""Gates C0/C1: candidate-driven waypoints and bounded planning selection.

Also the planning-side identity gate (B5 mirror), the contact-model
gate, session config fail-closed validation, and the ROS-free
architecture isolation scan for every new suction module.
"""

import math
import os
import unittest

from harness import suction_fakes as fakes

from luggage_planning.pose import Point, Pose, Quaternion
from luggage_planning.suction_candidate_selection import (
    MAX_CANDIDATE_ATTEMPTS,
    REQUIRED_CARTESIAN_FRACTION,
    SUCTION_CANDIDATE_IDENTITY_MISMATCH,
    SUCTION_CONTACT_MODEL_MISMATCH,
    ProbeRecord,
    SelectionState,
    contact_model_matches,
    judge_probe,
    next_candidate,
    observation_identity_consistent,
    rank_candidates,
    record_attempt,
    record_rejection,
)
from luggage_planning.suction_candidate_waypoints import (
    REVERSE_MIN_M,
    attach_pose_error,
    build_candidate_pick_segments,
    candidate_normal,
    pose_along_normal,
    quaternion_aligning_z_to,
    reverse_distance_m,
    retry_reverse_segment,
    tool_orientation_for_candidate,
)
from luggage_planning.suction_pick_session import (
    DETECT_NO_SEALABLE_PATCH,
    EXIT_DETECT,
    EXIT_IDENTITY,
    EXIT_OK,
    PickSession,
    SessionConfig,
    SessionConfigError,
)
from luggage_planning.suction_candidate_waypoints import (
    quat_rotate,
)


def flat_candidate(cid="C001_001", rank=1, xy=(0.65, 0.05), z=0.85):
    return fakes.make_candidate(cid, rank, xy=xy, z=z)


def tilted_candidate(cid="C010_001", rank=1, xy=(0.65, 0.05), z=0.85):
    normal = (math.sin(math.radians(7.0)), 0.0,
              math.cos(math.radians(7.0)))
    return fakes.make_candidate(cid, rank, xy=xy, z=z, normal=normal)


class TestCandidateWaypoints(unittest.TestCase):
    """Gate C0: attach pose equals the selected contact pose."""

    def _segments(self, candidate, yaw=0.3, yaw_valid=True):
        return build_candidate_pick_segments(
            candidate, detection_yaw=yaw, yaw_valid=yaw_valid,
            fallback_yaw=0.0)

    def test_attach_pose_equals_contact_pose_within_1mm_half_degree(self):
        for candidate in (flat_candidate(), tilted_candidate()):
            segments = self._segments(candidate)
            attach = next(s for s in segments if s.name == "attach")
            mm, normal_deg, _residual = attach_pose_error(
                attach.target_pose, candidate)
            self.assertLessEqual(mm, 1.0)
            self.assertLessEqual(normal_deg, 0.5)
            self.assertAlmostEqual(mm, 0.0, places=9)      # by construction
            self.assertLess(normal_deg, 1e-4)

    def test_attach_orientation_opposes_candidate_normal(self):
        candidate = tilted_candidate()
        attach = next(s for s in self._segments(candidate)
                      if s.name == "attach")
        normal = candidate_normal(candidate.contact.orientation)
        tool_plus_z = quat_rotate(attach.target_pose.orientation,
                                  (0.0, 0.0, 1.0))
        for axis, component in zip(normal, tool_plus_z):
            self.assertAlmostEqual(axis, -component, places=9)

    def test_flat_top_reduces_to_legacy_tool_down_quaternion(self):
        q = tool_orientation_for_candidate((0.0, 0.0, 1.0), 0.7)
        self.assertAlmostEqual(q.x, math.cos(0.35), places=12)
        self.assertAlmostEqual(q.y, math.sin(0.35), places=12)
        self.assertAlmostEqual(q.z, 0.0, places=12)
        self.assertAlmostEqual(q.w, 0.0, places=12)

    def test_altering_box_pose_changes_no_pick_waypoint(self):
        # structural: the builder takes no box argument at all. Prove it
        # end-to-end: two detections whose box pose differs by +-0.3 m
        # and 30 deg produce byte-identical segments for fixed candidates.
        candidate = flat_candidate()
        segments = self._segments(candidate)
        detection_a = fakes.make_detection([candidate], yaw_valid=True)
        detection_b = fakes.make_detection(
            [candidate], yaw_valid=True)
        detection_b = type(detection_b)(
            **{**detection_b.__dict__, "box_xyz": (0.95, -0.3, 0.7)})
        rebuilt = build_candidate_pick_segments(
            detection_b.candidates[0], detection_yaw=detection_a.detection_yaw,
            yaw_valid=True)
        self.assertEqual(
            [s.target_pose.position.x for s in segments],
            [s.target_pose.position.x for s in rebuilt])
        self.assertEqual(
            [s.target_pose.position.y for s in segments],
            [s.target_pose.position.y for s in rebuilt])
        self.assertEqual(
            [s.target_pose.position.z for s in segments],
            [s.target_pose.position.z for s in rebuilt])

    def test_motion_directions_parallel_to_candidate_normal(self):
        candidate = tilted_candidate()
        segments = {s.name: s for s in self._segments(candidate)}
        normal = candidate_normal(candidate.contact.orientation)
        attach_pos = segments["attach"].target_pose.position
        for name in ("pre_grasp", "approach", "pick_retreat"):
            pos = segments[name].target_pose.position
            delta = (pos.x - attach_pos.x, pos.y - attach_pos.y,
                     pos.z - attach_pos.z)
            length = math.sqrt(sum(d * d for d in delta))
            along = sum(d * n for d, n in zip(delta, normal)) / length
            self.assertGreater(along, 0.999, name)

    def test_approach_attach_forbid_ompl_and_require_fraction_one(self):
        segments = {s.name: s for s in self._segments(flat_candidate())}
        for name in ("approach", "attach"):
            self.assertFalse(segments[name].allow_ompl_fallback, name)
            self.assertEqual(
                segments[name].required_cartesian_fraction, 1.0, name)
        # retreat keeps the legacy free-space fallback
        self.assertTrue(segments["pick_retreat"].allow_ompl_fallback)

    def test_top_surface_valid_zero_candidates_cannot_build(self):
        ports = fakes.make_ports([])
        session = PickSession(
            ports, SessionConfig(contact_model_version=1,
                                 contact_model_hash="abc123"))
        result = session.run_request(
            fakes.make_detection([], top_surface_valid=True))
        self.assertEqual(result.reason_code, DETECT_NO_SEALABLE_PATCH)
        self.assertEqual(result.exit_code, EXIT_DETECT)
        self.assertEqual(ports.probe_fake.calls, [])
        self.assertEqual(ports.motion.executed, [])
        self.assertEqual(ports.vacuum.enable_calls, 0)

    def test_retry_reverse_reaches_recorded_approach_at_least_80mm(self):
        candidate = flat_candidate()
        segments = {s.name: s for s in self._segments(candidate)}
        approach_pose = segments["approach"].target_pose
        segment = retry_reverse_segment(candidate, approach_pose)
        distance = reverse_distance_m(
            candidate, segments["attach"].target_pose, approach_pose)
        self.assertGreaterEqual(distance, REVERSE_MIN_M)
        self.assertGreaterEqual(distance, 0.249)   # default clearance 0.25
        self.assertEqual(segment.required_cartesian_fraction, 1.0)
        self.assertFalse(segment.allow_ompl_fallback)

    def test_retry_reverse_rejects_too_short_retreat(self):
        candidate = flat_candidate()
        contact = candidate.contact
        near = Pose(position=Point(x=contact.position.x + 0.01,
                                   y=contact.position.y,
                                   z=contact.position.z + 0.01))
        with self.assertRaises(ValueError):
            retry_reverse_segment(candidate, near)

    def test_quaternion_helper_agrees_with_perception(self):
        import numpy as np
        from luggage_perception.suction_patch_evaluator import (
            _quaternion_aligning_z_to as perception_ref,
        )
        for vector in ((0, 0, 1), (0, 0, -1), (0.3, 0, 0.95),
                       (0.1, 0.2, 0.97), (0.0, -0.5, 0.87)):
            norm = math.sqrt(sum(c * c for c in vector))
            unit = tuple(c / norm for c in vector)
            mine = quaternion_aligning_z_to(unit)
            ref = perception_ref(np.array(unit))
            for got, want in zip((mine.x, mine.y, mine.z, mine.w), ref):
                self.assertAlmostEqual(got, want, places=12)


class TestCandidateSelection(unittest.TestCase):
    """Gate C1: rank-order selection skipping failed probes."""

    RANKED = ("C001_001", "C002_001", "C003_001", "C004_001", "C005_001")

    def _records(self, outcomes):
        return [
            ProbeRecord(candidate_id=cid, segment_name="approach",
                        ik_ok=ik, fraction=fraction, moveit_error_code=code)
            for cid, ik, fraction, code in outcomes
        ]

    def test_five_ranked_outcomes_select_fourth(self):
        candidates = [flat_candidate(cid, rank=i + 1)
                      for i, cid in enumerate(self.RANKED)]
        probes = {
            "C001_001": ([ProbeRecord("C001_001", "approach", ik_ok=False,
                                      fraction=-1.0,
                                      moveit_error_code=-31)],
                         "SUCTION_REJECT_IK"),
            "C002_001": ([ProbeRecord("C002_001", "approach", ik_ok=False,
                                      fraction=-1.0,
                                      moveit_error_code=-22)],
                         "SUCTION_REJECT_COLLISION"),
            "C003_001": ([ProbeRecord("C003_001", "approach", ik_ok=True,
                                      fraction=0.94, moveit_error_code=1)],
                         "SUCTION_REJECT_CARTESIAN_FRACTION"),
            "C004_001": ([ProbeRecord("C004_001", "approach", ik_ok=True,
                                      fraction=1.0, moveit_error_code=1)],
                         None),
        }
        state = SelectionState(ranked_ids=self.RANKED)
        selected = None
        rejections = []
        for cid in self.RANKED:
            if cid not in probes:
                continue
            records, reason = probes[cid]
            judged = judge_probe(
                next(c for c in candidates if c.candidate_id == cid),
                records)
            self.assertEqual(judged, reason, cid)
            if judged is None:
                selected = cid
                state = record_attempt(state, cid)
                break
            state = record_rejection(state, cid, judged, "")
            rejections.append((cid, judged))
        self.assertEqual(selected, "C004_001")
        self.assertEqual(
            rejections,
            [("C001_001", "SUCTION_REJECT_IK"),
             ("C002_001", "SUCTION_REJECT_COLLISION"),
             ("C003_001", "SUCTION_REJECT_CARTESIAN_FRACTION")])
        self.assertEqual(state.rejections[0][0], "C001_001")
        self.assertEqual(state.rejections[2][1],
                         "SUCTION_REJECT_CARTESIAN_FRACTION")

    def test_probe_rejections_do_not_consume_attempt_cap(self):
        # three probe rejections must not stop a fourth ranked candidate
        state = SelectionState(ranked_ids=self.RANKED)
        for cid in self.RANKED[:3]:
            state = record_rejection(state, cid, "SUCTION_REJECT_IK", "")
        self.assertEqual(next_candidate(state), "C004_001")
        state = record_attempt(state, "C004_001")
        state = record_attempt(state, "C005_001")
        self.assertEqual(next_candidate(state),
                         "SUCTION_CANDIDATES_EXHAUSTED")

    def test_fraction_0p999_is_rejected_against_exact_one(self):
        candidate = flat_candidate()
        records = [ProbeRecord("C001_001", "attach", ik_ok=True,
                               fraction=0.999, moveit_error_code=1)]
        self.assertEqual(judge_probe(candidate, records),
                         "SUCTION_REJECT_CARTESIAN_FRACTION")
        records[0] = ProbeRecord("C001_001", "attach", ik_ok=True,
                                 fraction=1.0, moveit_error_code=1)
        self.assertIsNone(judge_probe(candidate, records))

    def test_frame_mismatch_is_tf_rejection(self):
        candidate = fakes.make_candidate("C001_001", 1, frame="odom")
        records = [ProbeRecord("C001_001", "approach", ik_ok=True,
                               fraction=1.0, moveit_error_code=1)]
        self.assertEqual(judge_probe(candidate, records,
                                     planning_frame="world"),
                         "SUCTION_REJECT_TF")

    def test_rank_order_and_deterministic_tiebreak(self):
        unordered = [flat_candidate("C009_001", 3),
                     flat_candidate("C004_001", 1),
                     flat_candidate("C004_000", 1),
                     flat_candidate("C001_001", 2)]
        ranked = rank_candidates(unordered)
        self.assertEqual([c.candidate_id for c in ranked],
                         ["C004_000", "C004_001", "C001_001", "C009_001"])

    def test_default_caps_match_plan(self):
        self.assertEqual(MAX_CANDIDATE_ATTEMPTS, 3)
        self.assertEqual(REQUIRED_CARTESIAN_FRACTION, 1.0)


class TestIdentityGate(unittest.TestCase):
    """Planning-side mirror of the B5 identity contract."""

    def test_each_mismatch_returns_identity_reason(self):
        base = dict(stamp=1234.5, frame="world", instance_id="box-1",
                    generation=7)
        cases = [
            ("stamp", dict(stamp=1240.0)),
            ("frame", dict(frame="odom")),
            ("instance_id", dict(instance_id="box-2")),
            ("generation", dict(generation=8)),
        ]
        for field, override in cases:
            candidate_kwargs = dict(base)
            candidate_kwargs.update(override)
            candidate = fakes.make_candidate(
                "C001_001", 1, stamp=candidate_kwargs["stamp"],
                frame=candidate_kwargs["frame"],
                instance_id=candidate_kwargs["instance_id"],
                generation=candidate_kwargs["generation"])
            mismatch = observation_identity_consistent(
                [candidate], base["stamp"], base["frame"],
                base["instance_id"], base["generation"])
            self.assertIsNotNone(mismatch, field)
            self.assertEqual(mismatch[0], SUCTION_CANDIDATE_IDENTITY_MISMATCH)
            self.assertIn(field, mismatch[1])

    def test_stale_stamp_is_rejected(self):
        candidate = fakes.make_candidate("C001_001", 1, stamp=1234.5,
                                         instance_id="box-1", generation=7)
        mismatch = observation_identity_consistent(
            [candidate], 1234.5 + 1.1, "world", "box-1", 7)
        self.assertIsNotNone(mismatch)
        self.assertEqual(mismatch[0], SUCTION_CANDIDATE_IDENTITY_MISMATCH)

    def test_consistent_observation_passes(self):
        candidate = fakes.make_candidate("C001_001", 1, stamp=1234.5,
                                         frame="world", instance_id="box-1",
                                         generation=7)
        self.assertIsNone(observation_identity_consistent(
            [candidate], 1234.5, "world", "box-1", 7))

    def test_agreement_with_perception_authority(self):
        from luggage_perception.suction_patch_evaluator import (
            suction_identity_mismatch,
        )
        candidate = fakes.make_candidate("C001_001", 1, stamp=1234.5,
                                         frame="world", instance_id="box-1",
                                         generation=7)
        grid = [
            (1234.5, "world", "box-1", 7),
            (1240.0, "world", "box-1", 7),
            (1234.5, "odom", "box-1", 7),
            (1234.5, "world", "box-9", 7),
            (1234.5, "world", "box-1", 9),
        ]
        for stamp, frame, instance_id, generation in grid:
            mine = observation_identity_consistent(
                [candidate], stamp, frame, instance_id, generation)
            theirs = suction_identity_mismatch(
                candidate, stamp, frame, instance_id, generation)
            self.assertEqual(mine is None, theirs is None)
            if mine is not None:
                self.assertEqual(mine[0], theirs[0])

    def test_identity_gate_blocks_before_any_waypoint_or_motion(self):
        candidate = fakes.make_candidate("C001_001", 1, stamp=1234.5)
        ports = fakes.make_ports([candidate])
        session = PickSession(
            ports, SessionConfig(contact_model_version=1,
                                 contact_model_hash="abc123"))
        stale_detection = fakes.make_detection([candidate], stamp=9999.9)
        result = session.run_request(stale_detection)
        self.assertEqual(result.exit_code, EXIT_IDENTITY)
        self.assertEqual(result.reason_code,
                         SUCTION_CANDIDATE_IDENTITY_MISMATCH)
        # nothing may have been built, probed, moved or vacuumed
        self.assertEqual(ports.probe_fake.calls, [])
        self.assertEqual(ports.motion.executed, [])
        self.assertEqual(ports.vacuum.enable_calls, 0)
        self.assertEqual(ports.scene.calls, [])


class TestContactModelGate(unittest.TestCase):
    def test_contact_model_matches(self):
        candidate = fakes.make_candidate("C001_001", 1, model_version=1,
                                         model_hash="abc123")
        self.assertTrue(contact_model_matches(candidate, 1, "abc123"))
        self.assertFalse(contact_model_matches(candidate, 2, "abc123"))
        self.assertFalse(contact_model_matches(candidate, 1, "dead00"))

    def test_session_refuses_model_mismatch(self):
        candidate = fakes.make_candidate("C001_001", 1, model_version=1,
                                         model_hash="abc123")
        ports = fakes.make_ports([candidate])
        session = PickSession(
            ports, SessionConfig(contact_model_version=1,
                                 contact_model_hash="dead00"))
        result = session.run_request(fakes.make_detection([candidate]))
        self.assertEqual(result.exit_code, EXIT_IDENTITY)
        self.assertEqual(result.reason_code, SUCTION_CONTACT_MODEL_MISMATCH)
        self.assertEqual(ports.probe_fake.calls, [])
        self.assertEqual(ports.motion.executed, [])


class TestSessionConfigValidation(unittest.TestCase):
    def _check(self, **overrides):
        fields = dict(contact_model_version=1, contact_model_hash="abc123")
        fields.update(overrides)
        with self.assertRaises(SessionConfigError):
            SessionConfig(**fields).validate()

    def test_non_positive_budgets_fail_closed(self):
        self._check(release_low_hold_sec=0.0)
        self._check(release_deadline_sec=-1.0)
        self._check(reverse_min_m=0.0)
        self._check(request_deadline_sec=0.0)
        self._check(di0_poll_sec=0.0)

    def test_hold_must_fit_inside_deadline(self):
        self._check(release_low_hold_sec=2.5, release_deadline_sec=2.0)

    def test_fraction_bounds(self):
        self._check(required_cartesian_fraction=0.95)
        self._check(required_cartesian_fraction=1.01)

    def test_reverse_floor(self):
        self._check(reverse_min_m=0.04)

    def test_candidate_bounds(self):
        self._check(max_candidates=0)
        self._check(max_candidates=6)

    def test_vacuumless_pick_is_refused(self):
        self._check(use_vacuum=False)

    def test_defaults_are_plan_values(self):
        config = SessionConfig()
        config.validate()
        self.assertEqual(config.max_candidates, 3)
        self.assertEqual(config.release_low_hold_sec, 0.5)
        self.assertEqual(config.release_deadline_sec, 2.0)
        self.assertEqual(config.reverse_min_m, 0.08)
        self.assertEqual(config.required_cartesian_fraction, 1.0)
        self.assertEqual(config.request_deadline_sec, 180.0)


class TestSessionSelection(unittest.TestCase):
    """Gate C1 at session level: probe failures skip, fourth is executed."""

    def test_session_selects_fourth_of_five(self):
        candidates = [fakes.make_candidate(cid, rank=i + 1) for i, cid in
                      enumerate(("C001_001", "C002_001", "C003_001",
                                 "C004_001", "C005_001"))]
        overrides = {
            ("C001_001", "approach"): {
                "ik_ok": False, "fraction": -1.0,
                "moveit_error_code": -31},
            ("C002_001", "approach"): {
                "ik_ok": False, "fraction": -1.0,
                "moveit_error_code": -22},
            ("C003_001", "approach"): {
                "ik_ok": True, "fraction": 0.94,
                "moveit_error_code": 1},
        }
        ports = fakes.make_ports(
            candidates, seal_delays={"C004_001": 0.0},
            probe_overrides=overrides)
        session = PickSession(
            ports, SessionConfig(contact_model_version=1,
                                 contact_model_hash="abc123"))
        result = session.run_request(fakes.make_detection(candidates))
        self.assertEqual(result.exit_code, EXIT_OK)
        self.assertEqual(result.selected_candidate_id, "C004_001")
        # only C4's motion segments executed
        self.assertEqual(
            ports.motion.executed,
            [("C004_001", "pre_grasp"), ("C004_001", "approach"),
             ("C004_001", "attach"), ("C004_001", "pick_retreat")])
        self.assertEqual(ports.vacuum.enable_candidates, ["C004_001"])
        # every rejection reason and id preserved
        self.assertEqual(
            [(row[0], row[1]) for row in result.rejections],
            [("C001_001", "SUCTION_REJECT_IK"),
             ("C002_001", "SUCTION_REJECT_COLLISION"),
             ("C003_001", "SUCTION_REJECT_CARTESIAN_FRACTION")])


class TestMessageAdapters(unittest.TestCase):
    """ROS msg <-> pure view adapters (requires built luggage_msgs)."""

    def setUp(self):
        try:
            from luggage_msgs.msg import (  # noqa: F401
                DetectedLuggage,
                MotionSegment,
                SuctionCandidate,
            )
            from luggage_msgs.srv import BuildMotionSequence
        except ImportError:
            self.skipTest("luggage_msgs not built")
        self.DetectedLuggage = DetectedLuggage
        self.MotionSegmentMsg = MotionSegment
        self.BuildMotionSequence = BuildMotionSequence
        self.SuctionCandidate = SuctionCandidate
        from luggage_planning import ros_message_adapters as adapters
        self.adapters = adapters

    def _detected(self):
        msg = self.DetectedLuggage()
        msg.id = "box-1"
        msg.header.stamp.sec = 1234
        msg.header.stamp.nanosec = 500000000
        msg.header.frame_id = "world"
        msg.width = msg.depth = 0.4
        msg.height = 0.3
        msg.top_surface_valid = True
        candidate = self.SuctionCandidate()
        candidate.header.stamp.sec = 1234
        candidate.header.stamp.nanosec = 500000000
        candidate.header.frame_id = "world"
        candidate.instance_id = "box-1"
        candidate.generation = 7
        candidate.candidate_id = "C001_001"
        candidate.rank = 1
        candidate.model_version = 1
        candidate.model_hash = "abc123"
        candidate.contact_pose.position.x = 0.65
        candidate.contact_pose.position.y = 0.05
        candidate.contact_pose.position.z = 0.85
        msg.suction_candidates = [candidate]
        return msg, candidate

    def test_suction_candidates_roundtrip(self):
        msg, _candidate = self._detected()
        views = self.adapters.suction_candidates_from_detected(msg)
        self.assertEqual(len(views), 1)
        view = views[0]
        self.assertEqual(view.candidate_id, "C001_001")
        self.assertEqual(view.stamp, 1234.5)
        self.assertEqual(view.frame, "world")
        self.assertEqual(view.instance_id, "box-1")
        self.assertEqual(view.generation, 7)
        self.assertAlmostEqual(view.contact.position.x, 0.65, places=9)
        self.assertEqual(view.model_version, 1)
        self.assertEqual(view.model_hash, "abc123")
        stamp, frame = self.adapters.detected_observation_identity(msg)
        self.assertEqual((stamp, frame), (1234.5, "world"))
        # observation identity gate passes against the header
        self.assertIsNone(observation_identity_consistent(
            views, stamp, frame, view.instance_id, view.generation))

    def test_candidate_view_roundtrips_back_to_msg(self):
        msg, _candidate = self._detected()
        view = self.adapters.suction_candidates_from_detected(msg)[0]
        out = self.adapters.suction_candidate_to_msg(view, 1234.5, "world")
        self.assertEqual(out.candidate_id, view.candidate_id)
        self.assertAlmostEqual(
            out.contact_pose.position.x, view.contact.position.x, places=9)

    def test_segment_fraction_roundtrip(self):
        segment = build_candidate_pick_segments(flat_candidate())[1]
        msg = self.adapters.segment_to_msg(segment)
        self.assertEqual(msg.required_cartesian_fraction, 1.0)
        back = self.adapters.segment_from_msg(msg)
        self.assertEqual(back.required_cartesian_fraction, 1.0)
        self.assertFalse(back.allow_ompl_fallback)
        legacy = self.adapters.segment_from_msg(self.MotionSegmentMsg())
        self.assertEqual(legacy.required_cartesian_fraction, 0.0)

    def test_build_request_carries_candidate_id_default_empty(self):
        request = self.BuildMotionSequence.Request()
        self.assertEqual(request.suction_candidate_id, "")


class TestWiring(unittest.TestCase):
    """Source-scan: the ROS layer actually wires the candidate contract."""

    ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

    def _source(self, relative):
        with open(os.path.join(self.ROOT, relative)) as handle:
            return handle.read()

    def test_driver_wires_session_ports_and_cli(self):
        source = self._source(os.path.join(
            "scripts", "hardware_pick_driver.py"))
        self.assertIn("RosPickSessionPorts", source)
        self.assertIn("PickSession", source)
        for flag in ("--max-candidates", "--release-low-hold-sec",
                     "--release-deadline-sec", "--reverse-min-m",
                     "--required-cartesian-fraction",
                     "--request-deadline-sec", "--contact-model",
                     "--trace-out", "--dry-run", "--dry-run-count"):
            self.assertIn(flag, source)
        # contact model provenance is logged (B0-style fail-closed)
        self.assertIn("identity_hash", source)
        self.assertIn("load_suction_contact_model", source)

    def test_motion_planner_serves_probe_service(self):
        source = self._source(os.path.join(
            "scripts", "motion_planner_node.py"))
        self.assertIn("ProbeMotionSegment", source)
        self.assertIn("probe_motion_segment", source)
        self.assertIn("probe_segment", source)

    def test_waypoint_node_handles_candidate_requests(self):
        source = self._source(os.path.join(
            "scripts", "waypoint_generator_node.py"))
        self.assertIn("suction_candidate_id", source)
        self.assertIn("DETECT_NO_SEALABLE_PATCH", source)
        self.assertIn("observation_identity_consistent", source)

    def test_executor_honours_required_fraction(self):
        # A partial Cartesian prefix must never be sent as a completed pick.
        from types import SimpleNamespace as NS
        from test_place_cartesian_recovery import executor_type
        executor = executor_type()()
        executor._plan_cartesian = lambda *args: (.8, NS())
        executor._notify = lambda *args: None
        executor._result = lambda ok, message, fraction=0: NS(success=ok, message=message, fraction=fraction)
        executor._run_pose_target = lambda *args: NS(success=False, message='full goal blocked')
        executor._execute_trajectory = lambda *args: self.fail('partial path executed')
        segment = NS(name='pick_approach', type='cartesian', target_pose=NS(),
                     waypoints=[], allow_ompl_fallback=False,
                     required_cartesian_fraction=1.)
        result = executor._execute_segment_inner(segment, None, 45., [0.]*6)
        self.assertFalse(result.success)


class TestArchitectureIsolation(unittest.TestCase):
    """New suction modules are ROS-free, clock-free and IO-free."""

    MODULES = (
        "suction_candidate_waypoints.py",
        "suction_candidate_selection.py",
        "suction_retry_contracts.py",
        "suction_pick_session.py",
    )

    def test_no_ros_imports_no_io_no_time(self):
        here = os.path.dirname(os.path.abspath(__file__))
        package = os.path.join(here, "..", "luggage_planning")
        forbidden = (
            "import rclpy", "import rospy", "tf2_ros", "luggage_msgs",
            "moveit_msgs", "geometry_msgs", "import time", "time.sleep",
            "time.monotonic", "open(", "import yaml", "import numpy",
            "subprocess", "os.system",
        )
        for name in self.MODULES:
            with open(os.path.join(package, name)) as handle:
                source = handle.read()
            for token in forbidden:
                self.assertNotIn(token, source, "%s contains %s"
                                 % (name, token))


if __name__ == "__main__":
    unittest.main()
