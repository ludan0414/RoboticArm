"""The operator-taught P1->P2 line is executed without scene collision queries."""
import math
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('moveit_msgs')

from luggage_planning.manual_carry_transfer import TRANSFER_JOINTS_RAD
from luggage_planning.motion_executor import MotionExecutor


def fake_executor():
    executor = MotionExecutor.__new__(MotionExecutor)
    executed = []
    executor._wrist_blocked = lambda *args: False
    executor._wrist_check = lambda *args: None
    executor._notify = lambda *args: None
    executor._result = lambda ok, message: NS(success=ok, message=message)
    executor._execute_robot_trajectory = lambda traj, *args: (
        executed.append(traj) or NS(success=True, message='executed'))
    return executor, executed


def test_taught_line_executes_monotone_states_without_scene_client():
    executor, executed = fake_executor()
    start, goal = TRANSFER_JOINTS_RAD
    result = executor._run_taught_joint_line(goal, None, 45., start, NS(name='carry_p2'))
    assert result.success
    assert len(executed) == 1
    path = [list(p.positions) for p in executed[0].joint_trajectory.points]
    assert path[0] == list(start) and path[-1] == list(goal)
    for joint in range(6):
        steps = [b[joint]-a[joint] for a,b in zip(path,path[1:])]
        assert all(abs(step) <= math.radians(1.000001) for step in steps)
        assert all(step >= 0 for step in steps) or all(step <= 0 for step in steps)


def test_off_line_resume_blocks_before_execution():
    executor, executed = fake_executor()
    start, goal = TRANSFER_JOINTS_RAD
    off = list(start)
    off[3] += .25
    result = executor._run_taught_joint_line(goal, None, 45., off, NS(name='carry_p2'))
    assert not result.success
    assert 'off taught' in result.message
    assert not executed
