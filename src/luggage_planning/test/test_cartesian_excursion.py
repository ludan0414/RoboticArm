#!/usr/bin/env python3
"""Replay the 2026-09-21_1838 cartesian segments through the excursion gate.

The gate exists because `place_exit` in that run planned a smooth but
near-singular unwind: 6.58 rad of elfin_joint1 over a 1.39 m hop, 19.3 s
long, which the open-loop controller could not track (GOAL_TOLERANCE_VIOLATED
on joints 1 and 6). Every other cartesian segment in the same trial is a
legitimate path, so the gate has to separate one from the other six on
measured data rather than on a chosen constant.
"""

import json
import os
import unittest

from luggage_planning.motion_excursion import (
    _CARTESIAN_EXCURSION_MIN_PATH_M,
    _CARTESIAN_EXCURSION_RAD_PER_M,
    cartesian_excursion,
    excursion_diagnostic,
)

_FIXTURE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data", "cartesian_excursion_1838.json")

REJECTED = ("place_exit",)
ACCEPTED = ("stage_mid", "stage_late", "traverse", "insert", "descend",
            "retreat")


class _Point(object):
    def __init__(self, positions):
        self.positions = positions


def _measure(segment, **kwargs):
    return cartesian_excursion(
        segment["joint_names"],
        [_Point(row) for row in segment["positions"]],
        segment["path_len_m"], **kwargs)


class TestCartesianExcursion(unittest.TestCase):
    def test_advisory_mode_reports_3104_without_rejecting(self):
        points=[_Point([0]*6),_Point([0,0,.3104,0,0,0])]
        advisory=excursion_diagnostic([],points,.1,0)
        self.assertEqual(advisory['rad_per_m'],3.104)
        self.assertFalse(advisory['reject'])
        self.assertFalse(advisory['enforced'])
        self.assertTrue(excursion_diagnostic([],points,.1,3)['reject'])

    @classmethod
    def setUpClass(cls):
        with open(_FIXTURE, encoding="utf-8") as handle:
            cls.segments = {
                row["name"]: row for row in json.load(handle)["segments"]}

    def test_fixture_covers_the_measured_trial(self):
        self.assertEqual(
            set(self.segments), set(ACCEPTED) | set(REJECTED))

    def test_legitimate_segments_are_accepted(self):
        for name in ACCEPTED:
            measured = _measure(self.segments[name])
            self.assertTrue(
                measured["ok"],
                "%s rejected at %.3f rad/m" % (name, measured["rad_per_m"]))

    def test_place_exit_is_rejected(self):
        measured = _measure(self.segments["place_exit"])
        self.assertFalse(measured["ok"])
        self.assertEqual(measured["joint"], "elfin_joint1")
        self.assertAlmostEqual(measured["rad_per_m"], 4.720, places=2)

    def test_bound_keeps_margin_on_both_sides(self):
        worst_accept = max(
            _measure(self.segments[name])["rad_per_m"] for name in ACCEPTED)
        rejected = _measure(self.segments["place_exit"])["rad_per_m"]
        self.assertLess(worst_accept, _CARTESIAN_EXCURSION_RAD_PER_M / 1.5)
        self.assertGreater(rejected, _CARTESIAN_EXCURSION_RAD_PER_M * 1.25)

    def test_short_hop_is_judged_against_the_path_floor(self):
        """A 9 mm traverse must not be rejected for its denominator."""
        traverse = self.segments["traverse"]
        self.assertLess(traverse["path_len_m"], _CARTESIAN_EXCURSION_MIN_PATH_M)
        self.assertTrue(_measure(traverse)["ok"])

    def test_short_hop_still_rejects_a_wrist_flip(self):
        """The floor must not become a hole a 180 deg hop fits through."""
        flip = {
            "joint_names": self.segments["traverse"]["joint_names"],
            "path_len_m": 0.005,
            "positions": [[0.0] * 6, [0.0, 0.0, 0.0, 0.0, 3.14159, 0.0]],
        }
        measured = _measure(flip)
        self.assertFalse(measured["ok"])
        self.assertEqual(measured["joint"], "elfin_joint5")

    def test_nothing_to_judge_returns_none(self):
        self.assertIsNone(cartesian_excursion([], [], 1.0))
        self.assertIsNone(
            cartesian_excursion([], [_Point([0.0] * 6)], 1.0))
        self.assertIsNone(
            cartesian_excursion([], [_Point([0.0] * 6)] * 2, None))

    def test_limit_is_configurable(self):
        """The ROS parameter has to actually move the decision."""
        retreat = self.segments["retreat"]
        self.assertTrue(_measure(retreat)["ok"])
        self.assertFalse(_measure(retreat, limit_rad_per_m=0.5)["ok"])


if __name__ == "__main__":
    unittest.main()
