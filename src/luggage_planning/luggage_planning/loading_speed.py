"""Stationary, serialized operator speed changes with confirmed readback."""
import math


class LoadingSpeed:
    def __init__(self, cycle, apply):
        self.cycle, self.apply = cycle, apply
        self.percent = None

    def snapshot(self):
        with self.cycle.lock:
            return dict(percent=self.percent, minimum=10, maximum=100,
                        meaning='percent of current planned speed; next action only')

    def update(self, percent, request_id, schema_version):
        with self.cycle.lock:
            if (self.cycle.state not in ('WAIT_START', 'WAIT_PICKUP_READY', 'FAULT')
                    or request_id != self.cycle.request_id
                    or type(schema_version) is not int or schema_version != 1):
                return False
            if (type(percent) not in (float, int) or not math.isfinite(percent)
                    or not 10 <= percent <= 100):
                raise ValueError('speed percent must be within 10..100')
            self.cycle.ports.record(dict(event='speed_requested', percent=percent))
            self.percent = None  # An ambiguous service result is never confirmed.
            actual = self.apply(float(percent) / 100)
            if not math.isfinite(actual) or abs(actual - percent / 100) > 1e-9:
                raise ValueError('motion planner speed readback mismatch')
            self.percent = percent
            self.cycle.ports.record(dict(event='speed_applied', percent=percent))
            return True
