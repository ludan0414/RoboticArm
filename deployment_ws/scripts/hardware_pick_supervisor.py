#!/usr/bin/env python3
"""Linux-only lifetime owner for the real-cell pick graph (no ROS imports).

One graph per host/user, across checkouts and ROS domains: CPS is a physical
resource. Existing unmanaged nodes are reported, never automatically killed.
"""
import ctypes
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    group: int
    started: int
    state: str
    argv: tuple


def processes(root=Path('/proc')):
    """Bounded snapshot; start ticks protect signals against PID reuse."""
    result = {}
    for entry in root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            argv = tuple(x.decode(errors='replace') for x in
                         (entry / 'cmdline').read_bytes().split(b'\0') if x)
            pid = int(entry.name)
            result[pid] = Process(pid, int(fields[1]), int(fields[2]),
                                  int(fields[19]), fields[0], argv)
        except (OSError, ValueError, IndexError):
            continue  # process exited during snapshot
    return result


def enabled(arguments, name):
    values = [a.split(':=', 1)[1] for a in arguments if a.startswith(name + ':=')]
    return not values or values[-1].lower() in ('true', '1', 'yes', 'on')


def conflicts(snapshot, arguments):
    nodes = set()
    if enabled(arguments, 'start_planning'):
        nodes.update(('scene_manager_node.py', 'waypoint_generator_node.py',
                      'motion_planner_node.py', 'vacuum_controller_node.py'))
        if enabled(arguments, 'use_moveit'):
            nodes.add('move_group')
    if enabled(arguments, 'start_executor'):
        nodes.add('trajectory_executor')
    if enabled(arguments, 'start_scene'):
        nodes.update(('container_tf_publisher', 'scene_assets_node', 'robot_state_publisher'))
    if enabled(arguments, 'start_d555'):
        nodes.update(('realsense2_camera_node', 'd555_host_stamp'))
    if enabled(arguments, 'start_perception'):
        nodes.update(('sensor_preprocessor_node.py', 'semantic_segmenter_node.py',
                      'semantic_point_filter_node.py', 'luggage_detector_node.py'))
    found = []
    for proc in snapshot.values():
        if proc.pid == os.getpid() or proc.state == 'Z':
            continue
        # Examine executable/script arguments only, not arbitrary shell command
        # strings or ROS parameter values (which can mention these names).
        argv = proc.argv
        is_launch = any(Path(a).name == 'ros2' for a in argv[:2]) and (
            'launch' in argv and 'hardware_pick.launch.py' in argv)
        is_node = any(Path(a).name in nodes for a in argv[:2])
        if is_launch or is_node:
            found.append(proc)
    return found


def descendants(snapshot, owner, group):
    owned = {owner}
    while True:
        extra = {p.pid for p in snapshot.values()
                 if p.parent in owned or p.group == group}
        updated = owned | extra
        if updated == owned:
            break
        owned = updated
    return [snapshot[pid] for pid in owned if pid != owner and pid in snapshot]


def signal_owned(items, signum):
    current = processes()
    for proc in items:
        live = current.get(proc.pid)
        if live and live.started == proc.started and live.state != 'Z':
            try:
                os.kill(proc.pid, signum)
            except ProcessLookupError:
                pass


def reap_adopted(launch):
    # Popen owns waitpid for launch; only reap other (subreaper-adopted) children.
    for proc in processes().values():
        if proc.parent == os.getpid() and proc.pid != launch.pid:
            try:
                os.waitpid(proc.pid, os.WNOHANG)
            except ChildProcessError:
                pass


def supervise(arguments, command=None, grace=(15.0, 5.0, 2.0)):
    """Hold the lock until every owned child is stopped and reaped."""
    # A private UID directory avoids symlink attacks in shared /tmp. Do not
    # unlink lock files: unlinking permits two different locked inodes.
    lock_dir = Path('/tmp') / ('roboticarm-hardware-pick-' + str(os.getuid()))
    lock_dir.mkdir(mode=0o700, exist_ok=True)
    info = lock_dir.lstat()
    if lock_dir.is_symlink() or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise RuntimeError('unsafe lock directory: ' + str(lock_dir))
    fd = os.open(lock_dir / 'lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'r+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('REFUSED: hardware pick supervisor is already running. Stop it in its terminal.', file=sys.stderr)
            return 2
        existing = conflicts(processes(), arguments)
        if existing:
            print('REFUSED: existing launch/nodes conflict with this graph; no processes were killed.', file=sys.stderr)
            for proc in existing[:30]:
                print(f'  PID={proc.pid} PPID={proc.parent} ' + ' '.join(proc.argv)[:400], file=sys.stderr)
            return 2
        # Orphaned grandchildren are adopted here instead of by systemd/PID 1.
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
            raise OSError(ctypes.get_errno(), 'cannot enable child subreaper')
        requested = []

        def stopping(signum, _frame):
            if not requested:
                requested.append(signum)

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, stopping)
        lock.seek(0)
        lock.truncate()
        lock.write(f'supervisor_pid={os.getpid()} domain={os.environ.get("ROS_DOMAIN_ID", "0")}\n')
        lock.flush()
        launch = subprocess.Popen(command or ['ros2', 'launch', 'luggage_planning',
                                              'hardware_pick.launch.py', *arguments],
                                  start_new_session=True)
        print(f'hardware pick supervisor PID={os.getpid()} launch PID={launch.pid}', flush=True)
        try:
            while launch.poll() is None and not requested:
                reap_adopted(launch)
                time.sleep(0.2)
        finally:
            # Also runs when launch crashes or exits normally with surviving nodes.
            # Snapshot the whole tree, including reparented and detached children.
            for sig, seconds in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace):
                deadline = time.monotonic() + seconds
                sent = set()
                while True:
                    launch.poll()
                    reap_adopted(launch)
                    owned = descendants(processes(), os.getpid(), launch.pid)
                    if not owned:
                        break
                    fresh = [p for p in owned if (p.pid, p.started) not in sent]
                    signal_owned(fresh, sig)
                    sent.update((p.pid, p.started) for p in fresh)
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(0.1)
                if not owned:
                    break
            launch.poll()
            reap_adopted(launch)
            remaining = descendants(processes(), os.getpid(), launch.pid)
            if remaining:
                print('ERROR: cleanup incomplete; remaining PIDs: ' +
                      ','.join(str(p.pid) for p in remaining), file=sys.stderr)
            else:
                print('hardware pick cleanup complete: no owned child processes remain.', flush=True)
        if remaining:
            return 3
        if requested:
            return 128 + requested[0]
        code = launch.returncode or 0
        return 128 - code if code < 0 else code


if __name__ == '__main__':
    try:
        sys.exit(supervise(sys.argv[1:]))
    except (OSError, RuntimeError) as error:
        print(f'hardware pick supervisor: {error}', file=sys.stderr)
        sys.exit(2)
