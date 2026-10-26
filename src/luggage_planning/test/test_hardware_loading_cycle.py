"""Behavioral commissioning authorization and failure-boundary checks."""
import unittest

from luggage_planning.hardware_loading_cycle import LoadingCycle, parse_profile


class Ports:
    def __init__(self, fail=""):
        self.calls, self.events, self.fail = [], [], fail

    def record(self, event):
        self.events.append(event)
        if self.fail == "journal":
            raise OSError("disk full")

    def effect(self, name, index=None):
        self.calls.append((name, index))
        if self.fail == name:
            raise RuntimeError(name + " failed")

    def reset_observe(self):
        self.effect("reset")

    def recover_controller(self):
        self.effect("recover")

    def prepare(self):
        self.effect("prepare")

    def prepare_current(self):
        self.effect("check_current")

    def axis_test(self):
        self.effect("axis_test")

    def pick_place(self, index):
        self.effect("pick_place", index)

    def return_observe(self, index):
        self.effect("return", index)


def submit(cycle, command):
    accepted = cycle.submit(command, cycle.snapshot()["request_id"], 1, True)
    if cycle.worker:
        cycle.worker.join(2)
        assert not cycle.worker.is_alive()
    return accepted


class LoadingCycleTest(unittest.TestCase):
    def test_start_box_runs_remaining_absolute_indices_and_counts_only_this_run(self):
        ports=Ports()
        cycle=LoadingCycle(ports,4,start_box=3)
        self.assertEqual(ports.calls,[])
        self.assertEqual(cycle.snapshot()['next_box'],3)
        self.assertEqual(cycle.snapshot()['completed'],0)
        self.assertEqual(cycle.snapshot()['skipped'],2)
        self.assertEqual(cycle.snapshot()['run_total'],2)
        self.assertTrue(submit(cycle,'skip_initialize'))
        self.assertTrue(submit(cycle,'pick'))
        self.assertEqual(cycle.snapshot()['next_box'],4)
        self.assertEqual(cycle.snapshot()['completed'],1)
        self.assertTrue(submit(cycle,'pick'))
        self.assertEqual(cycle.state,'COMPLETE')
        self.assertEqual(cycle.snapshot()['completed'],2)
        self.assertEqual([c for c in ports.calls if c[0]=='pick_place'],
                         [('pick_place',2),('pick_place',3)])
        self.assertEqual([c for c in ports.calls if c[0]=='return'],
                         [('return',2),('return',3)])

    def test_invalid_start_box_cannot_start_effects(self):
        for start in (0,-1,5,True,1.5):
            ports=Ports()
            with self.assertRaises(ValueError):
                LoadingCycle(ports,4,start_box=start)
            self.assertEqual(ports.calls,[])

    def test_starting_last_box_preserves_index_through_fault_and_recovery(self):
        ports=Ports('pick_place')
        cycle=LoadingCycle(ports,4,start_box=4)
        submit(cycle,'skip_initialize')
        submit(cycle,'pick')
        self.assertEqual(cycle.snapshot()['completed'],0)
        self.assertEqual(cycle.snapshot()['next_box'],4)
        ports.fail=''
        submit(cycle,'recover')
        self.assertEqual(cycle.snapshot()['next_box'],4)
        submit(cycle,'skip_initialize')
        submit(cycle,'pick')
        self.assertEqual(cycle.state,'COMPLETE')
        self.assertEqual(cycle.snapshot()['completed'],1)

    def test_controller_recovery_clears_fault_without_release_or_count_change(self):
        ports = Ports("pick_place")
        cycle = LoadingCycle(ports, 2)
        submit(cycle, "skip_initialize")
        submit(cycle, "pick")
        ports.fail = ""
        self.assertTrue(submit(cycle, "recover"))
        self.assertEqual(cycle.state, "WAIT_START")
        self.assertEqual(cycle.error, "")
        self.assertEqual(cycle.index, 0)
        self.assertEqual(ports.calls[-1], ("recover", None))

    def test_controller_recovery_does_not_account_pending_return_or_allow_repick(self):
        ports = Ports("return")
        cycle = LoadingCycle(ports, 2)
        submit(cycle, "initialize")
        submit(cycle, "pick")
        ports.fail = ""
        self.assertTrue(submit(cycle, "recover"))
        self.assertTrue(cycle.return_pending)
        self.assertEqual(cycle.index, 0)
        self.assertFalse(submit(cycle, "skip_initialize"))
        self.assertFalse(submit(cycle, "pick"))
        self.assertTrue(submit(cycle, "reset"))
        self.assertEqual(cycle.index, 1)

    def test_controller_recovery_needs_current_request_and_scene_confirmation(self):
        cycle = LoadingCycle(Ports(), 1)
        self.assertFalse(cycle.submit("recover", "stale", 1, True))
        self.assertFalse(cycle.submit("recover", cycle.request_id, 1, False))
        self.assertEqual(cycle.ports.calls, [])

    def test_reset_after_pick_failure_preserves_index(self):
        ports = Ports("pick_place")
        cycle = LoadingCycle(ports, 2)
        submit(cycle, "initialize")
        submit(cycle, "pick")
        ports.fail = ""
        self.assertTrue(submit(cycle, "reset"))
        self.assertEqual(cycle.index, 0)
        self.assertEqual(cycle.state, "WAIT_PICKUP_READY")
        self.assertEqual(ports.calls[-1], ("reset", None))

    def test_reset_after_return_failure_accounts_placed_box_once(self):
        ports = Ports("return")
        cycle = LoadingCycle(ports, 1)
        submit(cycle, "initialize")
        submit(cycle, "pick")
        self.assertTrue(cycle.return_pending)
        ports.fail = "reset"
        submit(cycle, "reset")
        self.assertEqual(cycle.index, 0)
        self.assertTrue(cycle.return_pending)
        ports.fail = ""
        self.assertTrue(submit(cycle, "reset"))
        self.assertEqual(cycle.index, 1)
        self.assertEqual(cycle.state, "COMPLETE")
        self.assertEqual(ports.calls.count(("pick_place", 0)), 1)
        self.assertFalse(submit(cycle, "reset"))

    def test_optional_axis_test_restores_wait_state_and_never_picks(self):
        ports = Ports()
        cycle = LoadingCycle(ports, 1)
        self.assertTrue(submit(cycle, "axis_test"))
        self.assertEqual(cycle.state, "WAIT_START")
        self.assertTrue(cycle.axis_test_completed)
        self.assertEqual(ports.calls, [("axis_test", None)])
        self.assertTrue(submit(cycle, "skip_initialize"))
        self.assertTrue(submit(cycle, "axis_test"))
        self.assertEqual(cycle.state, "WAIT_PICKUP_READY")
        self.assertEqual(cycle.index, 0)

    def test_failed_axis_test_blocks_initialization_and_pick(self):
        cycle = LoadingCycle(Ports("axis_test"), 1)
        self.assertTrue(submit(cycle, "axis_test"))
        self.assertEqual(cycle.state, "FAULT")
        self.assertFalse(cycle.axis_test_completed)
        self.assertFalse(submit(cycle, "skip_initialize"))
        self.assertFalse(submit(cycle, "pick"))
    def test_skip_initialization_checks_current_then_waits_for_separate_pick(self):
        ports = Ports()
        cycle = LoadingCycle(ports, 1)
        old = cycle.request_id
        self.assertTrue(submit(cycle, "skip_initialize"))
        self.assertEqual(ports.calls, [("check_current", None)])
        self.assertEqual(cycle.index, 0)
        self.assertEqual(cycle.state, "WAIT_PICKUP_READY")
        self.assertFalse(cycle.submit("pick", old, 1, True))
        self.assertFalse(submit(cycle, "skip_initialize"))
        self.assertTrue(submit(cycle, "pick"))
        self.assertEqual(ports.calls[-2:], [("pick_place", 0), ("return", 0)])

    def test_skip_initialization_readiness_failure_latches_fault(self):
        cycle = LoadingCycle(Ports("check_current"), 1)
        self.assertTrue(submit(cycle, "skip_initialize"))
        self.assertEqual(cycle.state, "FAULT")
        self.assertFalse(submit(cycle, "pick"))

    def test_startup_has_no_effects_and_pick_cannot_initialize(self):
        ports = Ports()
        cycle = LoadingCycle(ports, 2)
        self.assertEqual(ports.calls, [])
        self.assertFalse(submit(cycle, "pick"))
        self.assertEqual(ports.calls, [])

    def test_ten_boxes_each_require_new_confirmation_and_return(self):
        ports = Ports()
        cycle = LoadingCycle(ports, 10)
        self.assertTrue(submit(cycle, "initialize"))
        for index in range(10):
            old = cycle.snapshot()["request_id"]
            self.assertTrue(submit(cycle, "pick"))
            self.assertEqual(ports.calls[-2:], [("pick_place", index), ("return", index)])
            self.assertFalse(cycle.submit("pick", old, 1, True))
        self.assertEqual(cycle.snapshot()["state"], "COMPLETE")
        self.assertEqual(cycle.snapshot()["completed"], 10)
        self.assertFalse(submit(cycle, "pick"))

    def test_malformed_or_unconfirmed_requests_emit_nothing(self):
        cycle = LoadingCycle(Ports(), 1)
        for request, schema, ack in [("", 1, True), ("old", 1, True),
                                     (cycle.request_id, True, True),
                                     (cycle.request_id, 2, True),
                                     (cycle.request_id, 1, False)]:
            self.assertFalse(cycle.submit("initialize", request, schema, ack))
        self.assertEqual(cycle.ports.calls, [])

    def test_faults_block_future_requests_and_never_skip_failed_stage(self):
        for failed in ("prepare", "pick_place", "return"):
            with self.subTest(failed=failed):
                ports = Ports(failed)
                cycle = LoadingCycle(ports, 2)
                submit(cycle, "initialize")
                if failed != "prepare":
                    submit(cycle, "pick")
                self.assertEqual(cycle.snapshot()["state"], "FAULT")
                self.assertEqual(cycle.snapshot()["completed"], 0)
                before = list(ports.calls)
                self.assertFalse(submit(cycle, "initialize"))
                self.assertFalse(submit(cycle, "pick"))
                self.assertEqual(before, ports.calls)
                if failed == "pick_place":
                    self.assertNotIn(("return", 0), ports.calls)

    def test_journal_failure_starts_no_effects(self):
        ports = Ports("journal")
        cycle = LoadingCycle(ports, 1)
        with self.assertRaises(OSError):
            submit(cycle, "initialize")
        self.assertEqual(ports.calls, [])
        self.assertEqual(cycle.state, "WAIT_START")

    def test_busy_and_duplicate_click_not_queued(self):
        import threading
        release, entered = threading.Event(), threading.Event()
        ports = Ports()
        def prepare():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test timeout")
        ports.prepare = prepare
        cycle = LoadingCycle(ports, 1)
        token = cycle.request_id
        try:
            self.assertTrue(cycle.submit("initialize", token, 1, True))
            self.assertTrue(entered.wait(1))
            self.assertFalse(cycle.submit("initialize", token, 1, True))
            self.assertFalse(cycle.submit("pick", token, 1, True))
        finally:
            release.set()
            cycle.worker.join(2)
        self.assertEqual(cycle.state, "WAIT_PICKUP_READY")
        self.assertEqual(cycle.index, 0)

    def test_route_requires_explicit_world_tcp_and_complete_cartesian_exit(self):
        pose = dict(position_m=[0.1, 0.2, 0.8], orientation_xyzw=[0, 0, 0, 1])
        profile = dict(profile="site_commissioning_v1", frame_id="world",
                       tool_frame="suction_contact_frame", observe_tool_pose=pose,
                       cycles=[dict(place_target=dict(frame_id="world",
                           pose_reference="box_center", **pose), exit_tool_poses=[pose])])
        observe, cycles = parse_profile(profile)
        self.assertEqual(observe.position.z, 0.8)
        self.assertEqual(cycles[0][1][0].required_cartesian_fraction, 1.0)
        self.assertFalse(cycles[0][1][0].allow_ompl_fallback)
        self.assertEqual(cycles[0][1][-1].name, "return_observe")
        profile["cycles"][0]["exit_tool_poses"] = []
        with self.assertRaises(ValueError):
            parse_profile(profile)
        profile["frame_id"] = "robot_base"
        with self.assertRaises(ValueError):
            parse_profile(profile)


if __name__ == "__main__":
    unittest.main()
