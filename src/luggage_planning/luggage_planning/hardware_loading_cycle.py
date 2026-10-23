"""Site commissioning cycle, independent of ROS and HTTP.

This is not the production cargo-map orchestrator. Each operator confirmation
also confirms the reviewed scene and previous placement. Recovery is an explicit
empty-tool reset; no automatic retry or motion on construction is allowed.
"""
import threading
import uuid

from luggage_planning.policy_place import _pose, parse_place_target
from luggage_planning.pose import MotionSegment


def loading_box_prompt(profile, snapshot):
    """Operator label from the same frozen row that owns the place target."""
    number = snapshot["next_box"]
    if number is None:
        return "全部完成"
    row = profile["cycles"][number - 1]
    dimensions = " × ".join("%g" % (v * 100) for v in row.get("expected_demo_input_dims_m", []))
    return "下一箱 %d/%d：%s（%s cm）" % (number, len(profile["cycles"]),
        row.get("box_label", row.get("demo_box_id", str(number))), dimensions)


def parse_profile(data):
    if (data.get("profile") != "site_commissioning_v1"
            or data.get("frame_id") != "world"
            or data.get("tool_frame") != "suction_contact_frame"):
        raise ValueError("requires site_commissioning_v1, world, suction_contact_frame")
    observe = _pose(data["observe_tool_pose"])
    entries = data["cycles"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("cycles must be a nonempty list")
    cycles = []
    for entry in entries:
        target = parse_place_target(entry["place_target"])
        route = entry["exit_tool_poses"]
        if not isinstance(route, list) or not route:
            raise ValueError("each cycle needs explicit exit_tool_poses")
        segments = [MotionSegment(
            name="exit_%d" % i, type="cartesian", target_pose=_pose(p),
            required_cartesian_fraction=1.0, allow_ompl_fallback=False)
            for i, p in enumerate(route)]
        segments.append(MotionSegment(name="return_observe", type="pose_target",
                                      target_pose=observe))
        cycles.append((target, tuple(segments)))
    return observe, tuple(cycles)


class LoadingCycle:
    """Single worker; reserve one request atomically before starting effects.

    ports: prepare(), prepare_current(), pick_place(index), return_observe(index), record(event).
    Methods raise on failure. The adapter must only return after settled motion.
    A failed or interrupted session requires explicit on-site recovery.
    """
    def __init__(self, ports, count):
        self.ports, self.count = ports, count
        self.lock = threading.RLock()
        self.state, self.stage, self.error = "WAIT_START", "WAIT_START", ""
        self.index = 0
        self.request_id = uuid.uuid4().hex
        self.worker = None
        self.axis_test_completed = False
        self.return_pending = False

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, stage=self.stage, error=self.error,
                        next_box=self.index + 1 if self.index < self.count else None,
                        completed=self.index, total=self.count,
                        return_pending=self.return_pending,
                        axis_test_motion_completed=self.axis_test_completed,
                        request_id=self.request_id, schema_version=1)

    def stage_changed(self, stage):
        with self.lock:
            self.ports.record(dict(event="stage", stage=stage, index=self.index))
            self.stage = stage

    def submit(self, command, request_id, schema_version, scene_confirmed):
        with self.lock:
            expected = (("reset",) if self.state == "FAULT" else
                        ("initialize", "skip_initialize", "axis_test", "reset") if self.state == "WAIT_START"
                        else ("pick", "axis_test", "reset"))
            if (self.state not in ("WAIT_START", "WAIT_PICKUP_READY", "FAULT")
                    or command not in expected or request_id != self.request_id
                    or type(schema_version) is not int or schema_version != 1
                    or scene_confirmed is not True):
                return False
            # Persist acceptance BEFORE authorizing effects. A journal failure
            # consumes no request and starts no worker.
            self.ports.record(dict(event="accepted", command=command,
                                   request_id=request_id, index=self.index))
            previous_state = self.state
            self.state = "BUSY"
            self.stage = {"initialize": "INITIALIZING", "skip_initialize": "CHECK_CURRENT",
                          "pick": "OBSERVE", "axis_test": "AXIS_TEST", "reset": "RESETTING"}[command]
            self.request_id = ""
            self.worker = threading.Thread(target=self._run, args=(command, previous_state), daemon=True)
            self.worker.start()
            return True

    def _run(self, command, previous_state):
        try:
            if command == "initialize":
                self.ports.prepare()
            elif command == "skip_initialize":
                self.ports.prepare_current()
            elif command == "axis_test":
                self.ports.axis_test()
            elif command == "reset":
                self.ports.reset_observe()
            else:
                self.ports.pick_place(self.index)
                self.return_pending = True
                self.stage_changed("EXIT_AND_RETURN")
                self.ports.return_observe(self.index)
            with self.lock:
                self.ports.record(dict(event="completed", command=command,
                                       index=self.index))
                self.error = ""
                if command == "pick" or (command == "reset" and self.return_pending):
                    self.index += 1
                    self.return_pending = False
                if command == "axis_test":
                    self.axis_test_completed = True
                    self.state = previous_state
                    self.stage = "AXIS_TEST_COMPLETE_CHECK_DIRECTIONS"
                else:
                    self.state = ("COMPLETE" if self.index == self.count
                                  else "WAIT_PICKUP_READY")
                    self.stage = self.state
                self.request_id = uuid.uuid4().hex if self.state != "COMPLETE" else ""
        except Exception as exc:
            with self.lock:
                self.state, self.stage, self.error = "FAULT", "FAULT", str(exc)
                self.request_id = uuid.uuid4().hex
                try:
                    self.ports.record(dict(event="fault", index=self.index, error=str(exc)))
                except Exception:
                    pass  # Remain blocked even if evidence storage also failed.
