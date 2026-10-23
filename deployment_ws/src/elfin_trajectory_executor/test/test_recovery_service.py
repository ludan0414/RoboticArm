"""Exercise ROS service handlers with fake hardware and the real owner gate."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from elfin_trajectory_executor.goal_ownership import SingleGoalOwner
from elfin_trajectory_executor.trajectory_executor_node import TrajectoryExecutorNode


class RecoveryServiceTest(unittest.TestCase):
    def test_registered_service_round_trip_uses_existing_backend(self):
        calls = []
        class FakeHardware:
            is_ready = False
            current_positions = [0.0] * 6
            current_velocities = []
            current_currents = []
            def __init__(self, **kwargs):
                calls.append("constructed")
            def connect(self):
                return True
            def refresh(self):
                pass
            def hardware_status(self, **kwargs):
                return {}
            def recover_standby(self):
                calls.append("recover")
                return True, "confirmed StandBy 33"
            def disconnect(self):
                pass
        rclpy.init(args=["--ros-args", "-p", "mode:=real"])
        executor = MultiThreadedExecutor(num_threads=2)
        node = client_node = None
        spinner = None
        try:
            module = "elfin_trajectory_executor.trajectory_executor_node"
            with patch(module + ".HuayanInterface", FakeHardware), patch(
                    module + ".apply_vacuum_io_params"), patch(
                    module + ".VacuumIoPublisher"):
                node = TrajectoryExecutorNode()
                client_node = rclpy.create_node("test_standby_recovery_client")
                client = client_node.create_client(Trigger, "/elfin/recover_standby")
                executor.add_node(node)
                executor.add_node(client_node)
                spinner = threading.Thread(target=executor.spin)
                spinner.start()
                self.assertTrue(client.wait_for_service(timeout_sec=3))
                done = threading.Event()
                future = client.call_async(Trigger.Request())
                future.add_done_callback(lambda _: done.set())
                self.assertTrue(done.wait(3))
                self.assertTrue(future.result().success)
                self.assertIn("33", future.result().message)
                self.assertEqual(calls, ["constructed", "recover"])
        finally:
            executor.shutdown()
            if spinner:
                spinner.join(3)
            if node:
                node.destroy_node()
            if client_node:
                client_node.destroy_node()
            rclpy.shutdown()

    def node(self, recover):
        node = SimpleNamespace(_goal_owner=SingleGoalOwner(),
            _iface=SimpleNamespace(recover_standby=recover),
            _publish_status=lambda *args, **kw: None,
            _publish_event=lambda *args, **kw: None)
        return node

    def call(self, node):
        return TrajectoryExecutorNode._handle_recover_standby(
            node, object(), SimpleNamespace())

    def test_active_goal_rejects_recovery_without_hardware_commands(self):
        calls = []
        node = self.node(lambda: calls.append("recover"))
        node._goal_owner.try_reserve()
        node._goal_owner.bind("active")
        self.assertFalse(self.call(node).success)
        self.assertEqual(calls, [])
        self.assertTrue(node._goal_owner.busy)

    def test_recovery_blocks_goals_do_and_second_recovery(self):
        started, release = threading.Event(), threading.Event()
        def recover():
            started.set()
            if not release.wait(2):
                raise RuntimeError("test synchronization timeout")
            return True, "StandBy 33"
        node = self.node(recover)
        result = []
        worker = threading.Thread(target=lambda: result.append(self.call(node)))
        worker.start()
        try:
            self.assertTrue(started.wait(1))
            self.assertFalse(node._goal_owner.try_reserve())
            self.assertFalse(node._goal_owner.cancel("stale-goal"))
            self.assertFalse(self.call(node).success)
            # If the gate failed, this would access a missing hardware method.
            response = TrajectoryExecutorNode._handle_set_do(
                node, 0, SimpleNamespace(data=False), SimpleNamespace())
            self.assertFalse(response.success)
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertTrue(result[0].success)
        self.assertFalse(node._goal_owner.busy)

    def test_exception_and_failed_recovery_release_reservation(self):
        def fail():
            raise RuntimeError("transport error")
        for recover in (fail, lambda: (False, "timeout")):
            node = self.node(recover)
            self.assertFalse(self.call(node).success)
            self.assertFalse(node._goal_owner.busy)

    def test_held_do_reservation_blocks_recovery(self):
        node = self.node(lambda: (True, "ok"))
        def apply(*args):
            self.assertFalse(node._goal_owner.cancel("stale-goal"))
            self.assertFalse(self.call(node).success)
            return SimpleNamespace(success=True)
        node._apply_set_do = apply
        response = TrajectoryExecutorNode._handle_set_do(
            node, 0, SimpleNamespace(data=False), SimpleNamespace())
        self.assertTrue(response.success)
        self.assertFalse(node._goal_owner.busy)
