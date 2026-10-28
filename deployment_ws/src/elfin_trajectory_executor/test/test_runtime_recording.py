from types import SimpleNamespace as NS
import ast
import json
from pathlib import Path
import unittest

from elfin_trajectory_executor.runtime_recording import hardware_sample, command_record


class RuntimeRecordingTests(unittest.TestCase):
    def sample(self):
        return NS(last_acs_ok=True, last_tcp_ok=True, last_cur_ok=True, last_cmd_ok=True,
            last_io_ok=True, last_sample_mono=123., last_refresh_dt_s=.008, _refresh_n=42,
            current_positions=[.1]*6, current_velocities=[.2]*6, vel_source='cps',
            current_currents=[2.3]*6, command_positions=[.11]*6, current_tcp_mm_deg=[10.]*6,
            vacuum_di0=1, vacuum_do0=1, vacuum_do1=0)

    def test_actual_command_current_and_vacuum_are_separate_with_source_time(self):
        row = hardware_sample(self.sample(), 456)
        self.assertEqual(row['q_rad'], [.1]*6)
        self.assertEqual(row['command_q_rad'], [.11]*6)
        self.assertEqual(row['current_a'], [2.3]*6)
        self.assertEqual(row['vacuum'], dict(DI0=1, DO0=1, DO1=0))
        self.assertEqual(row['sample_monotonic_sec'], 123.)

    def test_invalid_reads_never_reuse_previous_current_command_or_io(self):
        iface = self.sample()
        iface.last_cur_ok = iface.last_cmd_ok = iface.last_io_ok = False
        iface.vel_source = 'none'
        row = hardware_sample(iface, 456)
        self.assertEqual(row['current_a'], [])
        self.assertEqual(row['command_q_rad'], [])
        self.assertEqual(row['qd_rad_s'], [])
        self.assertIsNone(row['vacuum'])
        self.assertFalse(row['sample_ok']['current'])

    def test_command_retains_full_precision_velocities_and_final_servo_grid(self):
        point = NS(time_from_start=NS(sec=1,nanosec=123456789),
            positions=[.123456789]*6, velocities=[.2]*6, accelerations=[.3]*6)
        row = command_record(NS(joint_names=['j1'],points=[point]), 'goal', 123,
            servo_grid_deg=[[0.]*6,[1.]*6],servo_time=.02,hold_count=5)
        self.assertEqual(row['points'][0]['positions_rad'], [.123456789]*6)
        self.assertEqual(row['points'][0]['velocities_rad_s'], [.2]*6)
        self.assertEqual(row['servo_grid_deg'][-1], [1.]*6)

    def test_executor_publisher_records_current_with_only_existing_refresh(self):
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String, Header
        path = Path(__file__).parents[1]/'elfin_trajectory_executor/trajectory_executor_node.py'
        tree = ast.parse(path.read_text())
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='TrajectoryExecutorNode')
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name=='_publish_joint_states')
        scope = dict(__package__='elfin_trajectory_executor', json=json, String=String,
                     Header=Header, JointState=JointState)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        iface = self.sample()
        reads, samples, joints = [], [], []
        iface.refresh = lambda: reads.append(True)
        node = NS(_iface=iface, _joint_names=['j%d'%i for i in range(6)], _vacuum_pub=None,
            _telemetry_pub=NS(publish=samples.append), _js_pub=NS(publish=joints.append),
            get_clock=lambda: NS(now=lambda: NS(nanoseconds=123,to_msg=lambda: Time(nanosec=123))))
        scope[method.name](node)
        self.assertEqual(len(reads), 1)
        self.assertEqual(json.loads(samples[0].data)['current_a'], [2.3]*6)
        self.assertEqual(list(joints[0].effort), [2.3]*6)
