"""Fail-closed tests for CPS polling and controller-limit readiness."""

import threading
import unittest

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
            ["StartServo", "InitServoEsJ"])
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

    def test_servo_j_retries_init_after_40025_not_standby(self):
        class _Cps:
            def __init__(self):
                self.calls = []
                self._inits = 0

            def HRIF_InitServoEsJ(self, box, robot):
                self._inits += 1
                self.calls.append("InitServoEsJ")
                return 40025 if self._inits == 1 else 0

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append("StartServo")
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self.calls.append("PushServoJ")
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
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface._cps.calls[:4], [
            "StartServo", "InitServoEsJ", "InitServoEsJ", "PushServoJ",
        ])
        self.assertEqual(iface.stop_calls, 0)
        self.assertTrue(any("40025" in item for item in iface._node.logger.warnings))

    def test_servo_j_reopens_session_when_first_push_is_uninitialized(self):
        class _Cps:
            def __init__(self):
                self.calls = []
                self._pushes = 0

            def HRIF_InitServoEsJ(self, box, robot):
                self.calls.append("InitServoEsJ")
                return 0

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append("StartServo")
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self._pushes += 1
                self.calls.append("PushServoJ")
                if self._pushes == 1:
                    return 40071
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
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface._cps.calls[:4], [
            "StartServo", "InitServoEsJ", "PushServoJ", "InitServoEsJ",
        ])
        self.assertGreaterEqual(iface._cps.calls.count("PushServoJ"), 2)
        self.assertEqual(iface._cps.calls.count("StartServo"), 1)
        self.assertIn("40071", iface._node.logger.warnings[0])

    def test_servo_j_inits_only_when_already_in_servo(self):
        class _Cps:
            def __init__(self):
                self.calls = []

            def HRIF_InitServoEsJ(self, box, robot):
                self.calls.append("InitServoEsJ")
                return 0

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append("StartServo")
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self.calls.append("PushServoJ")
                return 0

        iface = _polling_interface(_Cps())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._refresh_positions = lambda: None
        iface._current_positions_deg = [0.0] * 6
        iface._max_vel = 0.0
        iface._controller_limit_fraction = 0.8
        iface._read_fsm = lambda: (34, "RobotServo")

        result = iface.execute_servo_j_path(
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface._cps.calls[0], "InitServoEsJ")
        self.assertNotIn("StartServo", iface._cps.calls)
        self.assertGreaterEqual(iface._cps.calls.count("PushServoJ"), 1)
        self.assertEqual(iface.stop_calls, 0)

    def test_servo_j_40071_reopens_from_standby_when_init_does_not_clear(self):
        class _Cps:
            def __init__(self):
                self.calls = []
                self.started = False
                self._pushes = 0

            def HRIF_InitServoEsJ(self, box, robot):
                self.calls.append("InitServoEsJ")
                return 0

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.started = True
                self.calls.append("StartServo")
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self._pushes += 1
                self.calls.append("PushServoJ")
                return 0 if self._pushes >= 3 else 40071

        cps = _Cps()
        iface = _polling_interface(cps)
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._refresh_positions = lambda: None
        iface._current_positions_deg = [0.0] * 6
        iface._max_vel = 0.0
        iface._controller_limit_fraction = 0.8

        def _read():
            if iface.stop_calls:
                return (33, "RobotStandBy")
            if cps.started:
                return (34, "RobotServo")
            return (33, "RobotStandBy")

        iface._read_fsm = _read

        result = iface.execute_servo_j_path(
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertGreaterEqual(iface.stop_calls, 1)
        self.assertGreaterEqual(cps.calls.count("StartServo"), 2)
        self.assertGreaterEqual(cps.calls.count("PushServoJ"), 3)
        self.assertTrue(any("StandBy" in item for item in iface._node.logger.warnings))

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
