"""Fail-closed tests for CPS polling and controller-limit readiness."""

import threading
import unittest
from unittest.mock import patch

from elfin_trajectory_executor.huayan_interface import (
    ConnectionState,
    HuayanInterface,
    RESULT_ERROR,
    RESULT_INVALID_GOAL,
    RESULT_PREEMPTED,
    RESULT_SUCCESSFUL,
)


class _Logger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def info(self, _message):
        pass

    def error(self, message):
        self.errors.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))

    def warning(self, message):
        self.warn(message)


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def _polling_interface(cps):
    iface = HuayanInterface.__new__(HuayanInterface)
    iface._node = _Node()
    iface._cps = cps
    iface._current_positions_deg = [0.0] * 6
    iface._refresh_positions = lambda: None
    iface._get_error_str = lambda code: "decoded-%s" % code
    iface.stop_calls = 0
    iface.states = []

    def stop(*_args, **_kwargs):
        iface.stop_calls += 1

    iface._safe_stop = stop
    iface._set_state = iface.states.append
    return iface


class HardwareSafetyTest(unittest.TestCase):
    def test_blending_error_stops_without_advancing(self):
        class _Cps:
            def __init__(self):
                self.calls = 0

            def HRIF_IsBlendingDone(self, _box, _robot, _result):
                self.calls += 1
                return 40083

        cps = _Cps()
        iface = _polling_interface(cps)
        result = iface._wait_for_waypoint(
            is_last=False,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(cps.calls, 1)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn("HRIF_IsBlendingDone", iface._node.logger.errors[0])
        self.assertIn("40083", iface._node.logger.errors[0])

    def test_motion_done_malformed_success_stops_and_enters_error(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, _result):
                return 0

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn(
            "malformed success payload", iface._node.logger.errors[0])

    def test_motion_done_nonzero_return_stops_and_enters_error(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, _result):
                return 20018

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn("20018", iface._node.logger.errors[0])

    def test_valid_motion_done_response_succeeds_without_stop(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, result):
                result.append(True)
                return 0

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface.stop_calls, 0)
        self.assertEqual(iface.states, [])

    def test_required_limits_fail_closed_but_jerk_is_diagnostic(self):
        class _Cps:
            def HRIF_ReadJointMaxVel(self, _box, _robot, result):
                result.extend([100.0] * 6)
                return 0

            def HRIF_ReadJointMaxAcc(self, _box, _robot, result):
                result.extend([80.0] * 6)
                return 0

        iface = HuayanInterface.__new__(HuayanInterface)
        iface._node = _Node()
        iface._cps = _Cps()
        iface._controller_limit_fraction = 0.8

        self.assertTrue(iface._read_controller_motion_limits())
        self.assertIsNone(iface._controller_max_jerk_deg)
        self.assertTrue(iface._node.logger.warnings)

        delattr(_Cps, "HRIF_ReadJointMaxAcc")
        self.assertFalse(iface._read_controller_motion_limits())

    def test_limit_fraction_cannot_exceed_safety_maximum(self):
        with self.assertRaisesRegex(ValueError, "controller_limit_fraction"):
            HuayanInterface(_Node(), controller_limit_fraction=0.81)

    def test_servo_j_uses_start_servo_and_push_servo_j_only(self):
        class _Cps:
            def __init__(self):
                self.calls = []

            def HRIF_InitServoEsJ(self, box, robot):
                self.calls.append(("InitServoEsJ", box, robot))
                return 0

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append(
                    ("StartServo", box, robot, servo_time, lookahead))
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self.calls.append(("PushServoJ", box, robot, list(joints)))
                return 0

            def HRIF_PushServoEsJ(self, *args):
                self.calls.append(("PushServoEsJ",) + args)
                return 0

            def HRIF_StartServoEsJ(self, *args):
                self.calls.append(("StartServoEsJ",) + args)
                return 0

        iface = _polling_interface(_Cps())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._refresh_positions = lambda: None
        iface._current_positions_deg = [0.0] * 6
        iface._max_vel = 0.0
        iface._controller_limit_fraction = 0.8
        iface._read_fsm = lambda: (33, "RobotStandBy")

        result = iface.execute_servo_j_path(
            [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(
            [call[0] for call in iface._cps.calls[:2]],
            ["StartServo", "PushServoJ"])
        self.assertNotIn(
            "PushServoEsJ", [call[0] for call in iface._cps.calls])
        self.assertNotIn(
            "StartServoEsJ", [call[0] for call in iface._cps.calls])
        push = [call for call in iface._cps.calls if call[0] == "PushServoJ"]
        self.assertGreaterEqual(len(push), 2)
        self.assertEqual(push[0][3], [0.0] * 6)
        self.assertEqual(push[-1][3], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(iface.states, [
            ConnectionState.EXECUTING, ConnectionState.READY])
        self.assertGreaterEqual(len(push), 12)

    def test_wait_fsm_standby_requires_exact_standby(self):
        reads = [(34, "RobotServo"), (34, "RobotServo"), (33, "RobotStandBy")]
        iface = _polling_interface(object())

        def _read():
            return reads.pop(0) if reads else (33, "RobotStandBy")

        iface._read_fsm = _read
        ok, fsm_id, fsm_desc = iface._wait_fsm_standby(timeout_s=1.0)
        self.assertTrue(ok)
        self.assertEqual(fsm_id, 33)
        self.assertEqual(fsm_desc, "RobotStandBy")

    def servo_controller(self, requires_init=False, clears_init_on_start=False):
        """Model real preconditions, rather than return success on retry N."""
        class Controller:
            def __init__(self):
                self.fsm = 33
                self.initialized = False
                self.calls = []
                self.fail_init = 0
                self.pushes = 0
                self.lose_on_push = None

            def HRIF_StartServo(self, *args):
                self.calls.append("StartServo")
                self.fsm = 34  # Synthetic active state, not a firmware enum.
                if clears_init_on_start:
                    self.initialized = False
                return 0

            def HRIF_InitServoEsJ(self, *args):
                self.calls.append("InitServoEsJ")
                if self.fsm != 33:
                    return 40025
                if self.fail_init:
                    return self.fail_init
                self.initialized = True
                return 0

            def HRIF_PushServoJ(self, *args):
                self.calls.append("PushServoJ")
                self.pushes += 1
                if self.lose_on_push == self.pushes:
                    return 40071
                return 40071 if requires_init and not self.initialized else 0

            def HRIF_GrpStop(self, *args):
                self.calls.append("GrpStop")
                self.fsm = 33
                return 0

        cps = Controller()
        iface = _polling_interface(cps)
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._max_vel = 0.0
        iface._read_fsm = lambda: (cps.fsm, "synthetic")
        def stop(**kwargs):
            iface._servo_j_timing = None
            iface.stop_calls += 1
            cps.HRIF_GrpStop()
        iface._safe_stop = stop
        return iface, cps

    def run_servo(self, iface):
        return iface.execute_servo_j_path(
            [[0.0] * 6], lambda positions: None, threading.Event(),
            servo_time=.001, lookahead_time=.001)

    def test_standard_suite_never_initializes_esj_and_reuses_owned_session(self):
        iface, cps = self.servo_controller()
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertEqual(cps.calls.count("StartServo"), 1)
        self.assertNotIn("InitServoEsJ", cps.calls)
        self.assertNotIn("GrpStop", cps.calls)

    def test_first_40071_stops_and_initializes_only_in_standby(self):
        iface, cps = self.servo_controller(requires_init=True)
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertEqual(cps.calls[:6], ["StartServo", "PushServoJ",
            "GrpStop", "InitServoEsJ", "StartServo", "PushServoJ"])
        self.assertEqual(cps.calls.count("InitServoEsJ"), 1)

    def test_firmware_clearing_init_on_start_fails_without_retry_loop(self):
        iface, cps = self.servo_controller(True, True)
        self.assertEqual(self.run_servo(iface), RESULT_ERROR)
        self.assertEqual(cps.calls.count("InitServoEsJ"), 1)
        self.assertEqual(cps.calls.count("StartServo"), 2)
        self.assertEqual(cps.calls.count("PushServoJ"), 2)

    def test_init_40025_is_not_retried_or_followed_by_start(self):
        iface, cps = self.servo_controller(requires_init=True)
        cps.fail_init = 40025
        self.assertEqual(self.run_servo(iface), RESULT_ERROR)
        self.assertEqual(cps.calls.count("InitServoEsJ"), 1)
        self.assertEqual(cps.calls[-1], "InitServoEsJ")
        self.assertIsNone(iface._servo_j_timing)

    def test_unknown_fsm_never_starts_or_initializes(self):
        iface, cps = self.servo_controller()
        cps.fsm = None
        self.assertEqual(self.run_servo(iface), RESULT_ERROR)
        self.assertEqual(cps.calls, [])

    def test_unowned_active_controller_is_stopped_before_start(self):
        iface, cps = self.servo_controller()
        cps.fsm = 34
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertEqual(cps.calls[:2], ["GrpStop", "StartServo"])
        self.assertNotIn("InitServoEsJ", cps.calls)

    def test_midstream_40071_fails_without_restarting_trajectory(self):
        iface, cps = self.servo_controller()
        cps.lose_on_push = 2
        self.assertEqual(self.run_servo(iface), RESULT_ERROR)
        self.assertEqual(cps.calls.count("StartServo"), 1)
        self.assertNotIn("InitServoEsJ", cps.calls)
        self.assertIsNone(iface._servo_j_timing)

    def test_fault_during_recovery_never_initializes_or_restarts(self):
        iface, cps = self.servo_controller(True)
        def fault_stop(*args):
            cps.calls.append("GrpStop")
            cps.fsm = 21
            return 0
        cps.HRIF_GrpStop = fault_stop
        self.assertEqual(self.run_servo(iface), RESULT_ERROR)
        self.assertNotIn("InitServoEsJ", cps.calls)
        self.assertEqual(cps.calls.count("StartServo"), 1)

    def test_timing_change_stops_owned_session_before_restarting(self):
        iface, cps = self.servo_controller()
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertTrue(iface._open_servo_j(.002, .001))
        self.assertEqual(cps.calls[-2:], ["GrpStop", "StartServo"])

    def test_controller_standby_requires_start_despite_cached_session(self):
        iface, cps = self.servo_controller()
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        cps.fsm = 33
        self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertEqual(cps.calls.count("StartServo"), 2)

    def test_cancel_during_recovery_never_initializes_or_restarts(self):
        iface, cps = self.servo_controller(True)
        cancel = threading.Event()
        original_stop = cps.HRIF_GrpStop
        def stop(*args):
            cancel.set()
            return original_stop(*args)
        cps.HRIF_GrpStop = stop
        result = iface.execute_servo_j_path(
            [[0.0]*6], lambda positions: None, cancel, .001, .001)
        self.assertEqual(result, RESULT_PREEMPTED)
        self.assertNotIn("InitServoEsJ", cps.calls)
        self.assertEqual(cps.calls.count("StartServo"), 1)

    def test_recovery_resets_schedule_without_catchup_burst(self):
        iface, cps = self.servo_controller(True)
        clock = [10.0]
        push_times = []
        original_push = cps.HRIF_PushServoJ
        def push(*args):
            push_times.append(clock[0])
            return original_push(*args)
        cps.HRIF_PushServoJ = push
        def sleep(seconds):
            clock[0] += seconds
        with patch("elfin_trajectory_executor.huayan_interface.time.monotonic",
                   side_effect=lambda: clock[0]), patch(
                   "elfin_trajectory_executor.huayan_interface.time.sleep",
                   side_effect=sleep):
            self.assertEqual(self.run_servo(iface), RESULT_SUCCESSFUL)
        self.assertGreater(push_times[1]-push_times[0], .09)
        for a, b in zip(push_times[1:], push_times[2:]):
            self.assertGreaterEqual(b-a, .001-1e-9)

    def test_servo_j_refuses_start_when_collision_stopped(self):
        class _Cps:
            def __init__(self):
                self.calls = []

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append(
                    ("StartServo", box, robot, servo_time, lookahead))
                return 0

        iface = _polling_interface(_Cps())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._max_vel = 0.0
        iface._read_fsm = lambda: (21, "RobotCollisionStop")

        result = iface.execute_servo_j_path(
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.02,
            lookahead_time=0.1,
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(iface._cps.calls, [])
        self.assertEqual(iface.stop_calls, 0)
        self.assertEqual(iface.states, [])
        self.assertIn("RobotCollisionStop", iface._node.logger.errors[0])

    def test_servo_j_cancel_calls_grp_stop_path(self):
        iface = _polling_interface(object())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._read_fsm = lambda: (33, "RobotStandBy")
        cancel = threading.Event()
        cancel.set()

        result = iface.execute_servo_j_path(
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=cancel,
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_PREEMPTED)
        self.assertEqual(iface.stop_calls, 1)

    def test_servo_j_rejects_bad_path_before_motion(self):
        iface = _polling_interface(object())
        iface._monitor_only = False

        result = iface.execute_servo_j_path(
            [[999.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_INVALID_GOAL)
        self.assertEqual(iface.stop_calls, 0)


if __name__ == "__main__":
    unittest.main()
