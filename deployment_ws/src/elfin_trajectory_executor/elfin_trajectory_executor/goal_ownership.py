"""Thread-safe ownership for one in-flight trajectory goal.

The ROS action server is reentrant so cancellation can run while execution is
blocked in the hardware backend.  Admission must still be exclusive: accepting
two goals would let both callbacks drive the same CPS connection and share
cancellation state.
"""

from __future__ import annotations

import threading
from typing import Optional


class SingleGoalOwner:
    """Own one pending or active goal and its private cancellation event."""

    def __init__(self) -> None:
        """Initialize an idle owner."""
        self._lock = threading.Lock()
        self._reserved = False
        self._goal_id: Optional[str] = None
        self._cancel_event: Optional[threading.Event] = None

    def try_reserve(self) -> bool:
        """Reserve admission before reconnect or action-handle creation."""
        with self._lock:
            if self._reserved:
                return False
            self._reserved = True
            self._goal_id = None
            self._cancel_event = threading.Event()
            return True

    def bind(self, goal_id: str) -> bool:
        """Bind the sole pending reservation to an accepted ROS goal UUID."""
        normalized = str(goal_id or '')
        if not normalized:
            return False
        with self._lock:
            if not self._reserved:
                return False
            if self._goal_id == normalized:
                return True
            if self._goal_id is not None:
                return False
            self._goal_id = normalized
            return True

    def try_reserve_operation(self, name: str) -> bool:
        """Atomically reserve a non-action operation; stale cancels cannot bind it."""
        if not name:
            return False
        with self._lock:
            if self._reserved:
                return False
            self._reserved = True
            self._goal_id = name
            self._cancel_event = None
            return True

    def cancel(self, goal_id: str) -> bool:
        """Set cancellation only when ``goal_id`` owns the executor."""
        normalized = str(goal_id or '')
        if not normalized:
            return False
        with self._lock:
            if not self._reserved or self._cancel_event is None:
                return False
            # The cancel service can run after the accepted response is sent
            # but just before handle_accepted_callback binds the UUID.
            if self._goal_id is None:
                self._goal_id = normalized
            elif self._goal_id != normalized:
                return False
            self._cancel_event.set()
            return True

    def cancel_event(self, goal_id: str) -> Optional[threading.Event]:
        """Return the owning goal's event; callers must not clear it."""
        normalized = str(goal_id or '')
        with self._lock:
            if not self._reserved or self._goal_id != normalized:
                return None
            return self._cancel_event

    def release(self, goal_id: str) -> bool:
        """Release an active owner without affecting a later reservation."""
        normalized = str(goal_id or '')
        with self._lock:
            if not self._reserved or self._goal_id != normalized:
                return False
            self._clear_locked()
            return True

    def release_pending(self) -> bool:
        """Release admission when a goal is rejected before UUID binding."""
        with self._lock:
            if not self._reserved or self._goal_id is not None:
                return False
            self._clear_locked()
            return True

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._reserved

    def _clear_locked(self) -> None:
        """Clear state while the caller holds ``_lock``."""
        self._reserved = False
        self._goal_id = None
        self._cancel_event = None
