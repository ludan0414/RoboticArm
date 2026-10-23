import unittest

from luggage_planning.release_confirmation import wait_for_release


class ReleaseConfirmationTest(unittest.TestCase):
    def confirm(self, received, state, di0=0):
        clock = [10.0]
        def sleep(seconds):
            clock[0] += seconds
        return wait_for_release(lambda: (received, state), lambda: di0,
            10.0, .2, 2.0, monotonic=lambda: clock[0], sleep=sleep)

    def test_new_empty_pump_off_feedback_succeeds(self):
        self.confirm(10.0, dict(attached=False, vacuum_on=False))

    def test_precommand_unknown_attached_or_enabled_feedback_times_out(self):
        for received, state, di0 in (
            (9.9, dict(attached=False, vacuum_on=False), 0),
            (None, {}, None),
            (10.0, dict(attached=True, vacuum_on=False), 0),
            (10.0, dict(attached=False, vacuum_on=True), 0),
            (10.0, dict(attached=False, vacuum_on=False), 1),
            (10.0, dict(attached=False, vacuum_on=False), None),
            (10.0, {}, 0),
        ):
            with self.assertRaisesRegex(RuntimeError, "UNCONFIRMED"):
                self.confirm(received, state, di0)
