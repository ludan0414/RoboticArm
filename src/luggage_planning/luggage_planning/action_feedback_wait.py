"""Client liveness deadline for a server with its own bounded motion timeout.

Continuous routes can exceed the old per-hop result deadline. Valid phase
feedback confirms that the server is still waiting within its execution budget;
only the action result confirms success. Missing feedback still times out.
"""
import threading
import time


class FeedbackDeadline:
    def __init__(self, timeout, clock=time.monotonic):
        self.timeout, self.clock = timeout, clock
        self.lock = threading.Lock()
        self.refresh()

    def refresh(self):
        with self.lock:
            self.deadline = self.clock() + self.timeout

    def feedback(self, message):
        phase = getattr(getattr(message, 'feedback', message), 'stage', '')
        if phase in ('planning', 'executing', 'settling'):
            self.refresh()

    def wait(self, event):
        while True:
            with self.lock:
                remaining = self.deadline - self.clock()
            if event.wait(max(0., remaining)):
                return True
            with self.lock:
                if self.clock() >= self.deadline:
                    return event.is_set()
