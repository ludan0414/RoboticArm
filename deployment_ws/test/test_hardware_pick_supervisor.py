"""No ROS or hardware required; lifetime cases require Linux."""
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/hardware_pick_supervisor.py'
spec = importlib.util.spec_from_file_location('hardware_pick_supervisor', SCRIPT)
s = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = s
if sys.platform == 'linux':
    spec.loader.exec_module(s)
else:
    with patch.dict(sys.modules, {'fcntl': types.ModuleType('fcntl')}):
        spec.loader.exec_module(s)


class SelectionTests(unittest.TestCase):
    def proc(self, pid, parent, group, argv):
        return s.Process(pid, parent, group, 10, 'S', tuple(argv))

    def test_disabled_remote_perception(self):
        p = self.proc(901, 1, 901, ['python3', '/lib/luggage_perception/luggage_detector_node.py'])
        self.assertEqual(s.conflicts({901: p}, ['start_perception:=false']), [])
        self.assertEqual(s.conflicts({901: p}, []), [p])

    def test_orphan_executor(self):
        p = self.proc(902, 1, 902, ['/lib/elfin_trajectory_executor/trajectory_executor'])
        self.assertEqual(s.conflicts({902: p}, []), [p])

    def test_shell_text_is_not_executable(self):
        p = self.proc(903, 1, 903, ['bash', '-c', 'echo hardware_pick.launch.py move_group'])
        self.assertEqual(s.conflicts({903: p}, []), [])

    def test_direct_launch(self):
        p = self.proc(904, 1, 904, ['python3', '/opt/ros/jazzy/bin/ros2', 'launch',
                                  'luggage_planning', 'hardware_pick.launch.py'])
        self.assertEqual(s.conflicts({904: p}, []), [p])

    def test_detached_and_adopted_children(self):
        items = [self.proc(10, 1, 10, []), self.proc(11, 10, 11, []),
                 self.proc(12, 11, 12, []), self.proc(13, 10, 13, []),
                 self.proc(99, 1, 99, [])]
        self.assertEqual({p.pid for p in s.descendants({p.pid: p for p in items}, 10, 11)}, {11, 12, 13})

    def test_proc_stat_parentheses(self):
        with tempfile.TemporaryDirectory() as temp:
            entry = Path(temp) / '123'
            entry.mkdir()
            fields = ['S', '1', '123'] + ['0'] * 16 + ['456']
            (entry / 'stat').write_text('123 (worker (child)) ' + ' '.join(fields))
            (entry / 'cmdline').write_bytes(b'python3\0worker.py\0')
            p = s.processes(Path(temp))[123]
            self.assertEqual((p.parent, p.group, p.started), (1, 123, 456))


@unittest.skipUnless(sys.platform == 'linux', 'requires procfs, flock and subreaper')
class LifetimeTests(unittest.TestCase):
    def run_case(self, code, interrupt=False, duplicate=False):
        with tempfile.TemporaryDirectory() as temp:
            pidfile = Path(temp) / 'child.pid'
            code = code.replace('PIDFILE', repr(str(pidfile)))
            wrapper = (f'import sys;sys.path.insert(0,{str(SCRIPT.parent)!r});'
                       'import hardware_pick_supervisor as s;'
                       f'sys.exit(s.supervise([],command=[sys.executable,"-c",{code!r}],grace=(.5,.5,1)))')
            owner = subprocess.Popen([sys.executable, '-c', wrapper], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)
            try:
                deadline = time.monotonic() + 5
                while not pidfile.exists() and time.monotonic() < deadline:
                    time.sleep(.05)
                self.assertTrue(pidfile.exists())
                pid = int(pidfile.read_text())
                if duplicate:
                    result = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True,
                                            text=True, timeout=5)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertIn('already running', result.stderr)
                    self.assertIsNone(owner.poll())
                if interrupt:
                    owner.send_signal(signal.SIGTERM)
                output = owner.communicate(timeout=8)[0]
                self.assertIn('no owned child processes remain', output)
                self.assertFalse(Path(f'/proc/{pid}').exists(), output)
            finally:
                if owner.poll() is None:
                    owner.send_signal(signal.SIGTERM)
                    owner.communicate(timeout=8)

    def test_crash_cleans_detached_orphan(self):
        self.run_case('import subprocess,sys,pathlib,time;'
                      'p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"],start_new_session=True);'
                      'pathlib.Path(PIDFILE).write_text(str(p.pid));time.sleep(.2);sys.exit(7)')

    def test_signal_and_duplicate(self):
        self.run_case('import os,pathlib,time;pathlib.Path(PIDFILE).write_text(str(os.getpid()));time.sleep(60)',
                      interrupt=True, duplicate=True)

    def test_stubborn_child(self):
        self.run_case('import os,pathlib,signal,time;'
                      'signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);'
                      'pathlib.Path(PIDFILE).write_text(str(os.getpid()));time.sleep(60)', interrupt=True)


if __name__ == '__main__':
    unittest.main()
