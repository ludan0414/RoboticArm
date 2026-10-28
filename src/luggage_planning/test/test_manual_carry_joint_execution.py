"""P1->P2 uses a complete TCP line with scene collision checking disabled."""
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('moveit_msgs')

from luggage_planning.manual_carry_transfer import TRANSFER_JOINTS_RAD
from luggage_planning.motion_executor import MotionExecutor


def pose(x):
    return NS(position=NS(x=x, y=0.0, z=0.7))


def fake_executor(fraction=1.0):
    executor = MotionExecutor.__new__(MotionExecutor)
    requests, executed = [], []
    p1, p2 = TRANSFER_JOINTS_RAD
    executor._fk_pose = lambda joints: pose(-1.0 if list(joints) == list(p1) else -0.1)
    executor._frame = 'world'
    executor._group = 'elfin_arm'
    executor._link = 'suction_contact_frame'
    executor._max_step = 0.01
    executor._wrist_blocked = lambda *args: False
    executor._wrist_check = lambda *args: None
    executor._notify = lambda *args: None
    executor._result = lambda ok, message: NS(success=ok, message=message)
    executor._execute_robot_trajectory = lambda traj, *args: (
        executed.append(traj) or NS(success=True, message='executed'))
    class CartesianClient:
        def wait_for_service(self, timeout_sec): return True
        def call_async(self, request):
            requests.append(request)
            trajectory = NS(joint_trajectory=NS(
                joint_names=['elfin_joint%d' % i for i in range(1, 7)],
                points=[NS(positions=list(p1)), NS(positions=list(p2))]))
            return NS(response=NS(fraction=fraction, solution=trajectory))
    executor._cartesian = CartesianClient()
    executor._wait_future = lambda future, timeout: future.response
    return executor, requests, executed


def test_complete_tcp_line_uses_no_collision_check_and_reaches_p2():
    executor, requests, executed = fake_executor()
    start, goal = TRANSFER_JOINTS_RAD
    result = executor._run_taught_tcp_line(goal, None, 45., start, NS(name='carry_p2'))
    assert result.success and len(requests) == len(executed) == 1
    assert requests[0].avoid_collisions is False
    assert requests[0].waypoints[0].position.x == -0.1
    assert executed[0].joint_trajectory.points[-1].positions == list(goal)


def test_partial_tcp_line_is_never_executed():
    executor, requests, executed = fake_executor(fraction=0.9)
    start, goal = TRANSFER_JOINTS_RAD
    result = executor._run_taught_tcp_line(goal, None, 45., start, NS(name='carry_p2'))
    assert not result.success
    assert 'incomplete' in result.message
    assert len(requests) == 1 and not executed


def test_p1_settling_error_can_start_actual_tcp_line():
    executor, requests, executed = fake_executor()
    start, goal = TRANSFER_JOINTS_RAD
    measured = [q + 0.018 for q in start]
    executor._fk_pose = lambda joints: (
        NS(position=NS(x=-1.0, y=0.0, z=0.7)) if list(joints) == list(start)
        else NS(position=NS(x=-0.1, y=0.0, z=0.7)) if list(joints) == list(goal)
        else NS(position=NS(x=-1.01, y=0.04, z=0.7)))
    result = executor._run_taught_tcp_line(goal, None, 45., measured, NS(name='carry_p2'))
    assert result.success and len(requests) == len(executed) == 1
    assert list(requests[0].start_state.joint_state.position) == measured
