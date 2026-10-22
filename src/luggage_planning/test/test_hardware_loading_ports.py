"""Execute the real console adapter with fake transport, without a ROS install."""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
import time
from unittest.mock import patch

from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion


def load_console():
    driver_module = ModuleType("hardware_pick_driver")
    class BaseDriver:
        def remove_scene_box(self):
            self.events.append("detach")
            self._scene_box = None
            return self.detach_ok, "detach result"
    class SessionPorts:
        def __init__(self, _node, driver):
            self.driver = driver
        def di0(self):
            return self.driver.di0
        def graph_health(self):
            return self.driver.healthy, "graph"
        def execute_segment(self, segment, _candidate):
            self.driver.moves.append(segment.name)
            return SimpleNamespace(ok=self.driver.motion_ok, message="motion result")
        def close(self):
            self.driver.closes += 1
    driver_module.HardwarePickDriver = BaseDriver
    driver_module.RosPickSessionPorts = SessionPorts
    driver_module.build_argument_parser = lambda: None
    rclpy = ModuleType("rclpy")
    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    path = Path(__file__).resolve().parents[1] / "scripts/hardware_loading_console.py"
    spec = importlib.util.spec_from_file_location("console_under_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", {"hardware_pick_driver": driver_module,
                                    "rclpy": rclpy, "rclpy.executors": executors}):
        spec.loader.exec_module(module)
    return module


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.module = load_console()
        self.driver = SimpleNamespace(di0=0, healthy=True, motion_ok=True,
            _vacuum_received_at=time.monotonic(),
            validate_scene_frame=lambda: None,
            _vacuum_state={"vacuum_on": False}, moves=[], closes=0,
            report_stage=lambda _stage: None)
        self.ports = self.module.HardwarePorts(self.driver, None, [], None, "test")

    def test_return_executes_all_segments_and_closes_subscription_ports(self):
        self.ports.execute([MotionSegment(name="exit"), MotionSegment(name="observe")])
        self.assertEqual(self.driver.moves, ["exit", "observe"])
        self.assertEqual(self.driver.closes, 1)

    def test_skip_initialization_checks_graph_scene_and_empty_tool_without_motion(self):
        self.driver.wait_graph = lambda: (True, "ok")
        self.driver._scene_client = lambda: SimpleNamespace(wait_ready=lambda **kw: True)
        self.ports.prepare_current()
        self.assertEqual(self.driver.moves, [])
        self.assertEqual(self.driver.closes, 1)

    def test_held_unknown_or_enabled_vacuum_never_starts_return(self):
        for di0, state in [(1, {"vacuum_on": False}), (None, {}),
                           (0, {"vacuum_on": True}), (0, {})]:
            self.driver.di0, self.driver._vacuum_state = di0, state
            with self.assertRaises(RuntimeError):
                self.ports.execute([MotionSegment(name="observe")])
        self.assertEqual(self.driver.moves, [])
        self.assertEqual(self.driver.closes, 4)

    def test_graph_failure_starts_no_return_motion(self):
        self.driver.healthy = False
        with self.assertRaises(RuntimeError):
            self.ports.execute([MotionSegment(name="exit")])
        self.assertEqual(self.driver.moves, [])

    def test_stale_or_missing_vacuum_state_never_starts_return(self):
        for received in (None, time.monotonic() - 10):
            self.driver._vacuum_received_at = received
            with self.assertRaises(RuntimeError):
                self.ports.execute([MotionSegment(name="exit")])
        self.assertEqual(self.driver.moves, [])

    def test_return_failure_never_dispatches_observe(self):
        self.driver.motion_ok = False
        with self.assertRaises(RuntimeError):
            self.ports.execute([MotionSegment(name="exit"), MotionSegment(name="observe")])
        self.assertEqual(self.driver.moves, ["exit"])
        self.assertEqual(self.driver.closes, 1)

    def test_release_proxy_uses_measured_dimensions_and_expected_target(self):
        driver = self.module.CommissioningDriver()
        driver.events, driver.detach_ok = [], True
        driver._scene_box = ([9, 9, 9], [0, 0, 0, 1], [0.3, 0.4, 0.5])
        driver._args = SimpleNamespace(place_target_data=SimpleNamespace(
            box_pose=Pose(Point(1, 2, 3), Quaternion())))
        driver.placed_proxy_id = "expected_test_1"
        def permissions(pairs, allowed):
            driver.events.append(("touch", pairs, allowed))
            return True, "ok"
        def add(*args):
            driver.events.append(("proxy", args))
            return True, "ok"
        scene = SimpleNamespace(set_acm_pairs=permissions, add_collision_box=add)
        driver._scene_client = lambda: scene
        self.assertTrue(driver.remove_scene_box()[0])
        self.assertEqual(driver.events[0], "detach")
        self.assertEqual(driver.events[-1], ("proxy", (
            "expected_test_1", [1, 2, 3], [0, 0, 0, 1], [0.3, 0.4, 0.5])))
        self.assertEqual(len(driver.events[1][1]), 2)

    def test_failed_detach_never_creates_expected_proxy(self):
        driver = self.module.CommissioningDriver()
        driver.events, driver.detach_ok = [], False
        driver._scene_box = ([0, 0, 0], [0, 0, 0, 1], [1, 1, 1])
        self.assertFalse(driver.remove_scene_box()[0])
        self.assertEqual(driver.events, ["detach"])

    def test_carry_connector_reads_fresh_actual_tcp(self):
        driver = self.module.CommissioningDriver()
        calls = []
        def lookup(target, source, fresh=False):
            calls.append((target, source, fresh))
            return Pose(Point(0, 1.443, .3), Quaternion())
        driver.lookup_pose = lookup
        first = MotionSegment(name="place_via_0", type="cartesian",
                              target_pose=Pose(Point(0, 1.0, .7), Quaternion()),
                              required_cartesian_fraction=1.0)
        result = driver.prepare_place_segments([first])
        self.assertEqual(calls, [("world", "suction_contact_frame", True)])
        self.assertGreater(len(result), 1)
        self.assertIs(result[-1], first)


if __name__ == "__main__":
    unittest.main()
