"""Long PlanMotion results, silent servers, and bounded server heartbeats."""
import ast
from concurrent.futures import Future
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

from luggage_planning.action_feedback_wait import FeedbackDeadline


class ScheduledEvent:
    """Deterministic monotonic clock and callback queue; no ROS or wall wait."""
    def __init__(self):
        self.now, self.ready, self.queue = 0., False, []

    def set(self):
        self.ready = True

    def clear(self):
        self.ready = False

    def is_set(self):
        return self.ready

    def wait(self, duration):
        end = self.now + duration
        while not self.ready and self.queue and self.queue[0][0] <= end:
            self.now, callback = self.queue.pop(0)
            callback()
        if not self.ready:
            self.now = end
        return self.ready


def load_method(path, owner_name, method_name, scope):
    tree = ast.parse(path.read_text())
    owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == owner_name)
    method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == method_name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
    return scope[method_name]


class ActionWaitTests(unittest.TestCase):
    def test_real_client_waits_148_seconds_for_result_with_phase_feedback(self):
        self.run_client(148., True, True)

    def test_real_client_cancels_when_server_goes_silent(self):
        self.run_client(148., False, False)

    def test_server_failure_is_not_changed_to_success_by_heartbeats(self):
        self.run_client(148., True, False, result_success=False)

    def run_client(self, result_at, feedback, expected_ok, result_success=True):
        from unittest.mock import patch
        event = ScheduledEvent()
        cancelled, sent = [], []
        result = Future()
        wrapped = SimpleNamespace(status=4, result=SimpleNamespace(success=result_success,
                                                                  message='controller result'))
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                 cancel_goal_async=lambda: cancelled.append(event.now))

        def send(goal, feedback_callback):
            sent.append(goal)
            if feedback:
                event.queue = [(stamp, lambda: feedback_callback(SimpleNamespace(
                    feedback=SimpleNamespace(stage='executing'))))
                               for stamp in (30., 60., 90., 120.)]
            event.queue.append((result_at, lambda: result.set_result(wrapped)))
            accepted = Future()
            accepted.set_result(handle)
            return accepted

        path = Path(__file__).parents[1]/'scripts/hardware_pick_driver.py'
        method = load_method(path, 'HardwarePickDriver', '_send_action', dict(
            threading=SimpleNamespace(Event=lambda: event), GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4)))
        client = SimpleNamespace(wait_for_server=lambda **kwargs: True, send_goal_async=send)
        factory = lambda timeout: FeedbackDeadline(timeout, clock=lambda: event.now)
        with patch('luggage_planning.action_feedback_wait.FeedbackDeadline', side_effect=factory):
            ok, message, returned = method(None, client, 'goal', 90., 'PlanMotion:pre_place')
        self.assertEqual(ok, expected_ok)
        self.assertEqual(sent, ['goal'])
        if feedback:
            self.assertEqual(event.now, result_at)
            self.assertEqual(cancelled, [])
            self.assertEqual(message, 'controller result')
            self.assertIs(returned, wrapped.result)
        else:
            self.assertEqual(event.now, 90.)
            self.assertEqual(cancelled, [90.])
            self.assertIn('feedback/result timeout', message)
            self.assertIsNone(returned)

    def test_unknown_feedback_does_not_extend_deadline_or_signal_success(self):
        event = ScheduledEvent()
        deadline = FeedbackDeadline(90., clock=lambda: event.now)
        event.queue = [(80., lambda: deadline.feedback(SimpleNamespace(stage='unknown')))]
        self.assertFalse(deadline.wait(event))
        self.assertEqual(event.now, 90.)

    def test_real_server_wait_expires_and_stops_heartbeats(self):
        path = Path(__file__).parents[1]/'luggage_planning/motion_executor.py'
        method = load_method(path, 'MotionExecutor', '_wait_goal', dict(
            threading=threading, wait_event=lambda event, timeout, **kwargs:
            (event.wait(timeout), 'timeout')))
        pending = Future()
        pulses, cancelled = [], []
        handle = SimpleNamespace(get_result_async=lambda: pending,
                                 cancel_goal=lambda: cancelled.append(True))
        executor = SimpleNamespace(_cancel_check=None,
                                   _node=SimpleNamespace(get_clock=lambda: None))
        self.assertIsNone(method(executor, handle, .55, heartbeat=lambda: pulses.append(True)))
        self.assertGreater(len(pulses), 0)
        self.assertEqual(cancelled, [True])
        count = len(pulses)
        time.sleep(.25)
        self.assertEqual(len(pulses), count)


if __name__ == '__main__':
    unittest.main()
