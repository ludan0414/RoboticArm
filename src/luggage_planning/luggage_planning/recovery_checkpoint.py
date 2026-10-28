"""Durable policy progress and typed motion context; no ROS or effects."""
from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile
import threading

from .pose import Point, Pose, Quaternion, MotionSegment
from .policy_place import PlaceTarget

TERMINAL = ('IDLE', 'DONE', 'ABORTED')


def pose_from_record(data):
    return Pose(Point(**data['position']), Quaternion(**data['orientation']))


def segment_from_record(data):
    data = dict(data)
    data['target_pose'] = pose_from_record(data['target_pose'])
    data['waypoints'] = [pose_from_record(p) for p in data['waypoints']]
    return MotionSegment(**data)


def target_from_record(data):
    return PlaceTarget(pose_from_record(data['box_pose']), data['clearance_m'],
                       tuple(pose_from_record(p) for p in data['via']),
                       tuple(data['reference_box_size_m']))


class RecoveryCheckpoint:
    def __init__(self, path, profile_sha256):
        self.path, self.lock = Path(path), threading.RLock()
        self.data = dict(schema='loading_recovery/v1', profile_sha256=profile_sha256,
                         phase='IDLE', box_index=0, placed=[], context={}, segments=[], next_segment=0)
        if self.path.exists():
            saved = json.loads(self.path.read_text())
            if saved.get('schema') != self.data['schema']:
                raise ValueError('unsupported recovery checkpoint: '+str(self.path))
            if saved.get('profile_sha256') != profile_sha256:
                raise ValueError('checkpoint/profile mismatch; use the original profile or a new journal')
            self.data = saved

    def snapshot(self):
        with self.lock:
            return deepcopy(self.data)

    @property
    def pending(self):
        return self.snapshot()['phase'] not in TERMINAL

    def update(self, **fields):
        with self.lock:
            data = {**self.data, **deepcopy(fields)}
            payload = json.dumps(data, allow_nan=False, indent=2)+'\n'
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                    prefix='.'+self.path.name, delete=False) as stream:
                temporary = stream.name
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            try:
                os.replace(temporary, self.path)
                directory = os.open(self.path.parent, os.O_RDONLY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
            self.data = data  # Authorize subsequent effects only after commit.

    def begin_pick(self, index):
        if self.pending:
            raise RuntimeError('unfinished checkpoint; resume or explicitly reset before another pick')
        self.update(phase='PICKING', box_index=index, next_segment=0,
                    context={}, segments=[], error='')
