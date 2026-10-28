"""Behavioral gate for taught P1->P2 execution; no robot is moved."""
import math
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('moveit_msgs')

from luggage_planning.manual_carry_transfer import TRANSFER_JOINTS_RAD
from luggage_planning.motion_executor import MotionExecutor


def fake_executor(validity):
    executor = MotionExecutor.__new__(MotionExecutor)
    requests, executed = [], []
    class ValidityClient:
        def wait_for_service(self, timeout_sec): return True
        def call_async(self, request):
            requests.append(request)
            return NS(response=NS(valid=validity(len(requests)-1), contacts=[]))
    executor._state_validity = ValidityClient()
    executor._wait_future = lambda future, timeout: future.response
    executor._group = 'elfin_arm'
    executor._occupancy = None
    executor._wrist_blocked = lambda *args: False
    executor._wrist_check = lambda *args: None
    executor._notify = lambda *args: None
    executor._result = lambda ok, message: NS(success=ok, message=message)
    executor._execute_robot_trajectory = lambda traj, *args: (
        executed.append(traj) or NS(success=True, message='executed'))
    return executor, requests, executed


def test_valid_line_executes_exactly_checked_monotone_states():
    executor, requests, executed = fake_executor(lambda _: True)
    start, goal = TRANSFER_JOINTS_RAD
    result = executor._run_taught_joint_line(goal, None, 45., start, NS(name='carry_p2'))
    assert result.success
    assert len(executed) == 1
    path = [list(p.positions) for p in executed[0].joint_trajectory.points]
    assert path[0] == list(start) and path[-1] == list(goal)
    assert len(requests) == len(path)
    for request, joints in zip(requests, path):
        assert list(request.robot_state.joint_state.position) == joints
        assert request.robot_state.is_diff
    for joint in range(6):
        steps = [b[joint]-a[joint] for a,b in zip(path,path[1:])]
        assert all(abs(step) <= math.radians(1.000001) for step in steps)
        assert all(step >= 0 for step in steps) or all(step <= 0 for step in steps)


def test_invalid_intermediate_state_blocks_execution():
    executor, requests, executed = fake_executor(lambda index: index != 8)
    start, goal = TRANSFER_JOINTS_RAD
    result = executor._run_taught_joint_line(goal, None, 45., start, NS(name='carry_p2'))
    assert not result.success
    assert 'state 8/' in result.message
    assert len(requests) == 9
    assert not executed


def test_off_line_resume_blocks_before_any_validity_or_execution():
    executor, requests, executed = fake_executor(lambda _: True)
    start, goal = TRANSFER_JOINTS_RAD
    off = list(start);off[3] += .25
    result = executor._run_taught_joint_line(goal, None, 45., off, NS(name='carry_p2'))
    assert not result.success
    assert 'off taught' in result.message
    assert not requests and not executed
