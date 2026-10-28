"""Append-only session telemetry; acquisition times are never interpolated."""
import json
import os
from pathlib import Path
import threading
import time


class TelemetryJournal:
    def __init__(self, journal, session):
        self.path = Path(str(journal) + '.telemetry.' + session + '.jsonl')
        self.file = self.path.open('x', encoding='utf-8')
        self.session, self.lock = session, threading.RLock()
        self.counts, self.error, self.closed = {}, '', False
        self.expected = ('/joint_states', '/elfin/cps_telemetry', '/vacuum/state',
                         '/vacuum/io', '/elfin/hardware_status')
        self.last_flush = time.monotonic()
        self.write('recorder', dict(schema='loading_telemetry/v1', session=session,
            units=dict(joint_position='rad', joint_velocity='rad/s', joint_states_effort='A (not Nm)'),
            note='Receipt time and original source stamps retained; missing data is not interpolated.'))

    def write(self, topic, payload):
        with self.lock:
            if self.closed or self.error:
                return False
            now, mono = time.time(), time.monotonic()
            row = dict(time=now, monotonic_sec=mono, session=self.session, topic=topic, data=payload)
            try:
                self.file.write(json.dumps(row, allow_nan=False) + '\n')
                stat = self.counts.setdefault(topic, dict(count=0, first_time=now, last_time=now, max_gap_sec=0.))
                stat['max_gap_sec'] = max(stat['max_gap_sec'], now-stat['last_time'])
                stat['count'] += 1
                stat['last_time'] = now
                fields = payload if isinstance(payload, dict) else {}
                critical = (topic in ('console', '/trajectory_executor/events') or
                            topic == '/rosout' and fields.get('level', 0) >= 40 or
                            topic == '/elfin/hardware_status' and fields.get('current_fault'))
                if mono-self.last_flush >= .5 or critical:
                    self.file.flush()
                    os.fsync(self.file.fileno())
                    self.last_flush = mono
                return True
            except (OSError, ValueError, TypeError) as exc:
                self.error = str(exc)
                return False

    def status(self):
        with self.lock:
            return dict(path=str(self.path), active=not self.closed and not self.error,
                        error=self.error, missing=[key for key in self.expected if key not in self.counts],
                        stale=[key for key in self.expected if key in self.counts and
                               time.time()-self.counts[key]['last_time'] > 2.],
                        topics={key: dict(value) for key, value in self.counts.items()})

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.write('recorder', dict(event='closed'))
            try:
                self.file.flush()
                os.fsync(self.file.fileno())
            finally:
                self.file.close()
                self.closed = True
            self.path.with_suffix('.summary.json').write_text(json.dumps(self.status(), indent=2)+'\n')
