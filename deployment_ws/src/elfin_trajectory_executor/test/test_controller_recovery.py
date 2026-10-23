"""Offline controller state transitions; no TCP or physical hardware."""
import unittest

from elfin_trajectory_executor.controller_recovery import recover_standby
from elfin_trajectory_executor.huayan_interface import already_motion_ready


class Model:
    def __init__(self, state):
        self.state, self.clock, self.calls = state, 0.0, []
        self.error, self.sticky = 0, False

    def command(self, name):
        self.calls.append(name)
        if not self.error and not self.sticky:
            self.state = 24 if name == "GrpReset" else 33
        return self.error

    def sleep(self, seconds):
        self.clock += seconds

    def recover(self):
        return recover_standby(lambda: (self.state, "model"), self.command,
            timeout=.5, monotonic=lambda: self.clock, sleep=self.sleep)


class ControllerRecoveryTest(unittest.TestCase):
    def test_moving_is_enabled_but_script_running_is_not(self):
        for state in (25, 26, 32, 33):
            self.assertTrue(already_motion_ready(state))
        for state in (None, 5, 21, 22, 24, 27, 34, 38, 100):
            self.assertFalse(already_motion_ready(state))

    def test_stop_moving_jogging_holding_or_standby_without_reset(self):
        for state in (25, 26, 27, 32, 33):
            model = Model(state)
            self.assertTrue(model.recover().ok)
            self.assertEqual(model.calls, ["GrpStop"])

    def test_real_fault_reset_then_enable_then_confirm_stop(self):
        for state in (21, 22):
            model = Model(state)
            self.assertTrue(model.recover().ok)
            self.assertEqual(model.calls, ["GrpReset", "GrpEnable", "GrpStop"])

    def test_disabled_enable_once(self):
        model = Model(24)
        self.assertTrue(model.recover().ok)
        self.assertEqual(model.calls, ["GrpEnable", "GrpStop"])

    def test_estop_unknown_free_drive_and_script_states_need_intervention(self):
        for state in (None, 5, 20, 29, 30, 31, 34, 35, 36, 37, 38, 99):
            model = Model(state)
            self.assertFalse(model.recover().ok)
            self.assertEqual(model.calls, [])

    def test_success_return_without_state_transition_times_out(self):
        model = Model(25)
        model.sticky = True
        self.assertFalse(model.recover().ok)
        self.assertEqual(model.calls, ["GrpStop"])

    def test_command_failure_stops_sequence(self):
        model = Model(25)
        model.error = 40025
        result = model.recover()
        self.assertFalse(result.ok)
        self.assertIn("40025", result.message)
        self.assertEqual(model.calls, ["GrpStop"])

    def test_fault_recurring_after_reset_is_not_repeated(self):
        model = Model(21)
        model.sticky = True
        self.assertFalse(model.recover().ok)
        self.assertEqual(model.calls, ["GrpReset"])


if __name__ == "__main__":
    unittest.main()
