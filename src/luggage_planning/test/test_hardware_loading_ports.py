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
        def _detection_view(self, box):
            return box.detection
        def attach_scene_box(self):
            self.events.append("attach")
            return True, "attached"
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
    def test_seal_records_actual_tool_tf_instead_of_candidate_orientation(self):
        from luggage_planning.policy_place import box_pose_from_grasp
        module = load_console()
        driver = module.CommissioningDriver()
        driver.events = []
        driver._scene_box = ([.1,.2,.3],[0,0,0,1],[.3,.4,.5])
        sealed = Pose(Point(.12,.18,.55),Quaternion(1,0,0,0))
        seen = []
        def lookup(target, source, fresh=False):
            seen.append((target,source,fresh))
            return sealed
        driver.lookup_pose = lookup
        driver.get_logger = lambda: SimpleNamespace(info=lambda message: None)
        self.assertTrue(driver.attach_scene_box()[0])
        box = box_pose_from_grasp(sealed,driver._measured_grasp)
        self.assertAlmostEqual(box.position.x,.1)
        self.assertAlmostEqual(box.position.z,.3)
        self.assertEqual(seen,[("world","suction_contact_frame",True)])

    def test_repeated_return_resolves_from_nominal_and_keeps_exit_route(self):
        self.ports.nominal_observe = Pose(Point(0,1.6,.86),Quaternion())
        root2 = 2**-.5
        base = Pose(Point(0,0,.86),Quaternion(0,0,-root2,root2))
        camera = Pose(Point(),Quaternion(0,root2,0,root2))
        self.driver.lookup_pose = lambda target,source: base if source=="elfin_base_link" else camera
        self.ports.record = lambda event: None
        exit_segment=MotionSegment(name="exit")
        self.ports.cycles=[(None,[exit_segment,MotionSegment(name="return_observe")])]
        captured=[]
        self.ports.execute=lambda segments: captured.append(segments)
        self.ports.return_observe(0)
        self.ports.return_observe(0)
        self.assertEqual([s[0].name for s in captured],
                         ['exit','return_pick_wait','return_observe']*2)
        for segments in (captured[2], captured[5]):
            self.assertAlmostEqual(segments[0].target_pose.position.y,.8)
            self.assertAlmostEqual(segments[0].target_pose.position.z,.66)

    def test_configured_height_replaces_visual_height_and_preserves_contact_top(self):
        from dataclasses import make_dataclass
        DetectionView = make_dataclass('DetectionView',
            ['stamp','frame','yaw_valid','box_xyz','box_size'])
        driver = self.module.CommissioningDriver()
        driver._configured_payload_size = (.55,.35,.20)
        driver.get_logger = lambda: SimpleNamespace(info=lambda message: None)
        observed = DetectionView(stamp=10, frame='world', yaw_valid=True,
            box_xyz=(.1,1.6,.1), box_size=(.47,.294,.24))
        box = SimpleNamespace(detection=observed,
            top_surface_pose=Pose(Point(.1,1.6,.22),Quaternion()))
        result = driver._detection_view(box)
        self.assertEqual(result.box_size,(.55,.35,.20))
        self.assertAlmostEqual(result.box_xyz[2],.12)
        self.assertAlmostEqual(result.box_xyz[2]+result.box_size[2]/2,.22)
        self.assertEqual(observed.box_size,(.47,.294,.24))

    def test_place_and_release_check_use_same_configured_target_with_support_gap(self):
        from luggage_planning.policy_place import PlaceTarget, measured_grasp_transform
        module = self.module
        driver = module.CommissioningDriver()
        box = Pose(Point(0,1.6,.1),Quaternion())
        tool = Pose(Point(0,1.6,.2),Quaternion(1,0,0,0))
        driver._measured_grasp = measured_grasp_transform(box,tool)
        target = PlaceTarget(Pose(Point(-1,.5,.63),Quaternion()),.15,
                             reference_box_size_m=(.55,.35,.20))
        driver._args = SimpleNamespace(place_target_data=target)
        driver.lookup_pose = lambda *args,**kwargs: tool
        driver.get_logger = lambda: SimpleNamespace(info=lambda message: None)
        detection=SimpleNamespace(frame='world',yaw_valid=True,box_size=(.55,.35,.20))
        segments=driver.prepare_place_segments([],detection,SimpleNamespace(frame='world'))
        descend=next(s for s in segments if s.name=='place_descend')
        self.assertAlmostEqual(descend.target_pose.position.z,.735)
        self.assertAlmostEqual(driver._effective_place_target.box_pose.position.z,.635)
        self.assertAlmostEqual(target.box_pose.position.z,.63)
        driver.lookup_pose=lambda *args,**kwargs: descend.target_pose
        self.assertTrue(driver.validate_place_pose()[0])

    def test_breadcrumb_records_actual_pick_stop_and_excludes_descent(self):
        driver=self.module.CommissioningDriver()
        actual=Pose(Point(.1,1.6,.6),Quaternion(1,0,0,0))
        calls=[]
        def lookup(*args,**kwargs):
            calls.append(kwargs)
            return actual
        driver.lookup_pose=lookup
        driver.record_completed_motion(MotionSegment(name='pick_retreat'))
        driver.record_completed_motion(MotionSegment(name='place_descend'))
        self.assertEqual(calls,[dict(fresh=True)])
        self.assertEqual(driver._pick_return_path.poses,[actual])

    def test_prepare_reverses_successful_carry_to_pick_stop_before_observe(self):
        from luggage_planning.pick_return_path import PickReturnPath
        path = self.driver._pick_return_path = PickReturnPath()
        for name,x in [('pick_retreat',1),('ground_entry_high_lift',2),('pre_place',3),
                       ('place_descend',4),('place_retreat',3)]:
            path.record(name,Pose(Point(x,0,1),Quaternion()))
        self.ports.prepare_current = lambda: None
        self.ports.resolve_return_pose = lambda: None
        captured=[]
        self.ports.execute=lambda segments: captured.extend(segments)
        self.ports.prepare()
        self.assertEqual([s.target_pose.position.x for s in captured[:-1]],[3,2,1])
        self.assertEqual(captured[-1].name,'initialize_observe')
        self.assertEqual(path.poses,[])
        self.assertTrue(all(s.required_cartesian_fraction==1 for s in captured[:-1]))

    def test_failed_reverse_preserves_waypoint_and_blocks_observe(self):
        from luggage_planning.pick_return_path import PickReturnPath
        path=self.driver._pick_return_path=PickReturnPath()
        path.record('pick_retreat',Pose(Point(1,0,1),Quaternion()))
        path.record('pre_place',Pose(Point(3,0,1),Quaternion()))
        self.ports.prepare_current=lambda: None
        self.ports.resolve_return_pose=lambda: None
        names=[]
        def execute(segments):
            names.extend(s.name for s in segments)
            if segments[0].name=='return_pick_stop_0':
                raise RuntimeError('blocked')
        self.ports.execute=execute
        with self.assertRaisesRegex(RuntimeError,'blocked'):
            self.ports.prepare()
        self.assertEqual(names,['return_pick_stop_1','return_pick_stop_0'])
        self.assertEqual(len(path.poses),1)

    def test_final_pose_check_refuses_yaw_error_before_release(self):
        from luggage_planning.policy_place import measured_grasp_transform
        module = load_console()
        driver=module.CommissioningDriver()
        box=Pose(Point(0,0,.2),Quaternion())
        tool=Pose(Point(0,0,.4),Quaternion(1,0,0,0))
        driver._measured_grasp=measured_grasp_transform(box,tool)
        driver._args=SimpleNamespace(place_target_data=SimpleNamespace(box_pose=box))
        driver.get_logger=lambda: SimpleNamespace(info=lambda message: None)
        driver.lookup_pose=lambda *args,**kw: tool
        self.assertTrue(driver.validate_place_pose()[0])
        driver._args.place_target_data.box_pose=Pose(box.position,Quaternion(0,0,.1,(.99)**.5))
        self.assertFalse(driver.validate_place_pose()[0])

    def test_hardware_feedback_expiry_never_reports_current_state_available(self):
        module = load_console()
        driver = SimpleNamespace(_console_hardware={"available": True, "fsm_id": 33},
            _console_hardware_received_at=time.monotonic()-10)
        self.assertFalse(module.hardware_view(driver)["available"])
        driver._console_hardware_received_at = time.monotonic()
        self.assertTrue(module.hardware_view(driver)["available"])

    def setUp(self):
        self.module = load_console()
        self.driver = SimpleNamespace(di0=0, healthy=True, motion_ok=True,
            _vacuum_received_at=time.monotonic(),
            _args=SimpleNamespace(vacuum_state_max_age_sec=2.0),
            validate_scene_frame=lambda: None,
            _vacuum_state={"vacuum_on": False, "attached": False}, moves=[], closes=0,
            report_stage=lambda _stage: None)
        self.ports = self.module.HardwarePorts(self.driver, None, [], None, "test")

    def configure_reset(self):
        self.actions = []
        self.driver.recover_controller = lambda: (self.actions.append("recover") or True, "ok")
        self.ports.record = lambda event: None
        self.ports.observe = Pose(Point(0, 1.2, .86), Quaternion())
        self.driver._args.release_deadline_sec = .01
        self.driver._scene_box = ([0, 0, 1], [0, 0, 0, 1], [.3, .4, .5])
        def release(enable):
            self.actions.append("release")
            self.assertFalse(enable)
            self.driver._vacuum_state = dict(attached=False, vacuum_on=False)
            self.driver._vacuum_received_at = time.monotonic()
            self.driver.di0 = 0
            return True, "ok"
        self.driver.vacuum = release
        def detach():
            self.actions.append("detach")
            return True, "ok"
        self.driver._scene_client = lambda: SimpleNamespace(detach_and_remove=detach)
        self.driver.set_pickup_touch = lambda allowed: (True, "ok")
        self.ports.prepare = lambda: self.actions.append("return")

    def test_reset_explicitly_releases_held_unknown_and_stale_then_returns(self):
        for di0, state, age in [(1, dict(attached=True, vacuum_on=True), 0),
                                 (None, {}, 0), (0, dict(attached=False), 10)]:
            self.configure_reset()
            self.driver.di0, self.driver._vacuum_state = di0, state
            self.driver._vacuum_received_at = time.monotonic() - age
            self.ports.reset_observe()
            self.assertEqual(self.actions, ["recover", "release", "detach", "return"])
            self.assertIsNone(self.driver._scene_box)

    def test_controller_failure_preserves_suction_and_starts_no_return(self):
        self.configure_reset()
        self.driver.recover_controller = lambda: (False, "40025")
        with self.assertRaisesRegex(RuntimeError, "RESET_CONTROLLER"):
            self.ports.reset_observe()
        self.assertEqual(self.actions, [])

    def test_release_failure_never_detaches_or_returns(self):
        self.configure_reset()
        self.driver.vacuum = lambda enable: (False, "release timeout")
        with self.assertRaisesRegex(RuntimeError, "RESET_PUMP"):
            self.ports.reset_observe()
        self.assertEqual(self.actions, ["recover"])
        self.assertIsNotNone(self.driver._scene_box)

    def test_successful_rpc_without_new_feedback_never_detaches_or_returns(self):
        self.configure_reset()
        self.driver._vacuum_state = dict(attached=False, vacuum_on=False)
        self.driver.vacuum = lambda enable: (True, "ok")
        with self.assertRaisesRegex(RuntimeError, "UNCONFIRMED"):
            self.ports.reset_observe()
        self.assertEqual(self.actions, ["recover"])

    def test_failed_detach_preserves_geometry_and_blocks_return(self):
        self.configure_reset()
        self.driver._scene_client = lambda: SimpleNamespace(
            detach_and_remove=lambda: (False, "scene unavailable"))
        with self.assertRaisesRegex(RuntimeError, "RESET_DETACH"):
            self.ports.reset_observe()
        self.assertEqual(self.actions, ["recover", "release"])
        self.assertIsNotNone(self.driver._scene_box)

    def test_return_failure_does_not_report_reset_complete(self):
        self.configure_reset()
        events = []
        self.ports.record = events.append
        def fail_return():
            raise RuntimeError("MoveIt failed")
        self.ports.prepare = fail_return
        with self.assertRaisesRegex(RuntimeError, "MoveIt failed"):
            self.ports.reset_observe()
        self.assertNotIn("reset_complete", [event["event"] for event in events])

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

    def test_status_payload_shows_hrif_not_site_profile(self):
        snapshot = {"state": "FAULT", "stage": "APPROACH", "error": "exit 5",
                    "completed": 0, "total": 4}
        payload = self.module.merge_console_status(
            snapshot, "下一箱 1/4",
            {"attached": False, "vacuum_on": False, "age_sec": 0.2},
            {"available": True, "fsm_id": 33, "last_hrif": {"code": 40071}},
            {"event": "aborted", "error_string": "Hardware execution error"},
        )
        self.assertEqual(payload["hardware"]["last_hrif"]["code"], 40071)
        self.assertEqual(payload["vacuum"]["age_sec"], 0.2)
        self.assertNotIn("profile_assumptions", payload)
        self.assertIn("robot-fsm", self.module.PAGE)
        self.assertIn("hrif-code", self.module.PAGE)
        self.assertIn("event-log", self.module.PAGE)
        self.assertIn("/elfin/hardware_status", self.module.PAGE)


if __name__ == "__main__":
    unittest.main()
