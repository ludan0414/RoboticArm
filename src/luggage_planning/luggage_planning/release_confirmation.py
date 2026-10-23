"""Confirm new empty-tool feedback after an explicit release command."""
import time


def wait_for_release(read_state, read_di0, commanded_at, timeout, max_age, *,
                     monotonic=time.monotonic, sleep=time.sleep):
    """read_state returns (local receipt monotonic time, decoded vacuum state).

    Timeout never authorizes motion. Pre-command or stale feedback, missing
    fields, attached payload or enabled pump cannot confirm release.
    """
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        received, state = read_state()
        now = monotonic()
        if (received is not None and commanded_at <= received <= now
                and now - received <= max_age
                and state.get("attached") is False
                and state.get("vacuum_on") is False
                and read_di0() == 0):
            return
        sleep(.05)
    raise RuntimeError("RESET_RELEASE_UNCONFIRMED: no fresh empty-tool / pump-off feedback")
