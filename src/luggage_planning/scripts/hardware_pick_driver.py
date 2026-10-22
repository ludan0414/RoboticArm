#!/usr/bin/env python3
"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Hardware pick: observe -> DetectLuggage -> candidate selection ->
PlanMotion per candidate, with bounded vacuum retry (dynamic-suction
plan section D).

The pick flow lives in ``luggage_planning.suction_pick_session.PickSession``
(pure, port-injected); this script only wires ROS clients into the
session ports. Candidate contract: at most three candidates per request,
one vacuum attempt per candidate; on VACUUM_SEAL_TIMEOUT the backend
self-releases, DI0 must be observed low 0.5 s within 2.0 s, the arm
reverses >= 80 mm along the candidate normal, and only then may it move
laterally to the next candidate. ``top_surface_valid`` alone never
authorizes motion.

Exit codes: 1 graph, 2 observe, 3 detect/no-sealable-patch, 4 build/scene,
5 segment, 6 legacy vacuum (unused by the candidate path), 7
SUCTION_RETRY_RECOVERY_FAILED, 8 SUCTION_CANDIDATES_EXHAUSTED, 9 identity
or contact-model mismatch, 10 carry fault (vacuum PRESERVED; explicit
recovery required).

Vacuum attach after the attach segment (box DO0/DO1, wait DI0). Default
holds suction after retreat so you can see the box; Ctrl-C or --release
drops it.

With --place-target JSON, continue after successful pick to the policy's
world-frame box-centre pose, release, confirm DI0 low, then retreat. Place
faults preserve vacuum until explicit recovery; no automatic exit release.
Exit 11 is a place release/scene/retreat fault.

Person on e-stop. Does not start Gazebo. Does not spawn a simulated box.

Typical order after hardware_pick.launch.py is up:

  ros2 run luggage_planning hardware_pick_driver.py --detect-only
  ros2 run luggage_planning hardware_pick_driver.py --plan-only
  ros2 run luggage_planning hardware_pick_driver.py --dry-run   # C7 protocol
  ros2 run luggage_planning hardware_pick_driver.py
      # default --observe-pose current: stay at live joints
  ros2 run luggage_planning hardware_pick_driver.py --observe-pose pickup_observe
      # only after filling robot_poses.site.yaml; sim example joints are forbidden
"""

from __future__ import division

import argparse
import json
import signal
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from luggage_msgs.action import GoToRobotPose, PlanMotion
from luggage_msgs.msg import VacuumState
from luggage_msgs.srv import (BuildMotionSequence, DetectLuggage,
                              ProbeMotionSegment, VacuumCommand)
from std_msgs.msg import Bool, String

from luggage_planning import ros_message_adapters as adapters
from luggage_planning.pick_authorization import (
    AuthorizationConfig,
    PickAuthorizationPolicy,
    run_authorization_loop,
)
from luggage_planning.pickup_collision import pickup_collision_aabb
from luggage_planning.suction_candidate_selection import rank_candidates
from luggage_planning.suction_candidate_waypoints import (
    build_candidate_pick_segments,
)
from luggage_planning.suction_pick_session import (
    PickSession,
    SegmentOutcome,
    SessionConfig,
)
from luggage_planning.suction_pick_session import DetectionView
from luggage_planning.policy_place import (
    build_place_segments, parse_place_target, run_place,
)


class RosPickSessionPorts:
    """PickSession ports over the driver's ROS clients."""

    def __init__(self, node, driver, log_path=None):
        self._node = node
        self._driver = driver
        self._di0_raw = None
        self._current_box = None
        trace_mode = "x" if getattr(driver, "exclusive_trace", False) else "w"
        self._trace_file = open(log_path, trace_mode) if log_path else None
        self._di0_subscription = node.create_subscription(
            Bool, "/vacuum/di0", self._on_di0, 10,
            callback_group=driver._group)
        self._box_subscription = node.create_subscription(
            String, "/luggage/current_box", self._on_current_box,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
            callback_group=driver._group)

    def _on_di0(self, msg):
        self._di0_raw = 1 if msg.data else 0

    def _on_current_box(self, msg):
        try:
            from luggage_perception.cargo_instance_tracker import (
                parse_current_box_payload,
            )
            box_id, generation = parse_current_box_payload(msg.data)
            self._current_box = (str(box_id), int(generation))
        except Exception as exc:  # noqa: BLE001 - passive subscription
            self._node.get_logger().warn(
                "current_box payload unreadable: %s" % exc)

    # -- ports interface ----------------------------------------------------

    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(max(0.0, dt))

    def probe(self, segment, candidate_id):
        request = ProbeMotionSegment.Request()
        request.segment = adapters.segment_to_msg(segment)
        response = self._driver._call_srv(
            self._driver._probe, request, 15.0)
        if response is None:
            return {"ik_ok": False, "fraction": -1.0,
                    "moveit_error_code": 0}
        return {"ik_ok": bool(response.ik_ok),
                "fraction": float(response.fraction),
                "moveit_error_code": int(response.moveit_error_code)}

    def execute_segment(self, segment, candidate_id):
        if self._driver.segment_guard is not None:
            ok, message = self._driver.segment_guard(segment)
            if not ok:
                return SegmentOutcome(ok=False, message=message)
        self._driver.report_stage(segment.name.upper())
        goal = PlanMotion.Goal()
        goal.segment = adapters.segment_to_msg(segment)
        ok, message, result = self._driver._send_action(
            self._driver._plan, goal, self._driver._args.plan_timeout,
            "PlanMotion:%s" % segment.name)
        return SegmentOutcome(
            ok=ok, message=message,
            fraction=float(getattr(result, "fraction", -1.0) or 0.0),
            settle_json=str(getattr(result, "settle_json", "") or ""))

    def vacuum_enable(self, candidate_id):
        self._node.get_logger().info(
            "vacuum attach: candidate=%s" % candidate_id)
        ok, message = self._driver.vacuum(True, timeout=20.0)
        return ok, message

    def vacuum_release(self, candidate_id=""):
        return self._driver.vacuum(False)

    def di0(self):
        if self._di0_raw is not None:
            return self._di0_raw
        state = self._driver._vacuum_state
        if state.get("attached"):
            return 1
        if "attached" in state:
            return 0
        return None

    def scene_add_box(self, detection):
        ok, message = self._driver.add_scene_box_detection(detection)
        return ok, message

    def scene_attach(self):
        return self._driver.attach_scene_box()

    def scene_remove(self):
        return self._driver.remove_scene_box()

    def set_pickup_touch(self, allowed):
        return self._driver.set_pickup_touch(allowed)

    def identity_latest(self):
        return self._current_box

    def graph_health(self):
        checks = (
            (self._driver._plan.server_is_ready(),
             "PlanMotion action missing"),
            (self._driver._probe.service_is_ready(),
             "ProbeMotionSegment missing"),
            (self._driver._build.service_is_ready(),
             "BuildMotionSequence missing"),
        )
        for ok, reason in checks:
            if not ok:
                return False, reason
        return True, "ok"

    def log_event(self, record):
        line = json.dumps(record, default=str)
        if self._trace_file is None:
            self._node.get_logger().info("retry: %s" % line)
            return
        self._trace_file.write(line + "\n")
        self._trace_file.flush()

    def close(self):
        self._node.destroy_subscription(self._di0_subscription)
        self._node.destroy_subscription(self._box_subscription)
        if self._trace_file is not None:
            self._trace_file.close()
            self._trace_file = None


class HardwarePickDriver(Node):

    def __init__(self, args):
        super().__init__("hardware_pick_driver")
        self._args = args
        self._group = ReentrantCallbackGroup()
        self._vacuum_state = {}
        self._vacuum_received_at = None
        self._scene_box = None
        self.create_subscription(
            VacuumState, "/vacuum/state", self._on_vacuum, 10,
            callback_group=self._group)
        self._detect = self.create_client(
            DetectLuggage, "/luggage_detector/detect_luggage",
            callback_group=self._group)
        self._build = self.create_client(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            callback_group=self._group)
        self._probe = self.create_client(
            ProbeMotionSegment, "/motion_planner/probe_motion_segment",
            callback_group=self._group)
        self._vacuum = self.create_client(
            VacuumCommand, "/vacuum/command", callback_group=self._group)
        self._goto = ActionClient(
            self, GoToRobotPose, "/motion_planner/go_to_robot_pose",
            callback_group=self._group)
        self._plan = ActionClient(
            self, PlanMotion, "/motion_planner/plan_motion",
            callback_group=self._group)
        self._scene = None
        self._release_on_exit = False
        self.stage_callback = None
        self.segment_guard = None
        self.detection_guard = None

    def report_stage(self, stage):
        if self.stage_callback is not None:
            self.stage_callback(stage)

    def _scene_client(self):
        if self._scene is None:
            from luggage_planning.planning_scene_client import PlanningSceneClient
            self._scene = PlanningSceneClient(self, callback_group=self._group)
        return self._scene

    def _on_vacuum(self, msg):
        self._vacuum_state = {
            "attached": bool(msg.attached),
            "vacuum_on": bool(msg.vacuum_on),
            "fail_reason": msg.fail_reason,
        }
        self._vacuum_received_at = time.monotonic()

    def _call_srv(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def _send_action(self, client, goal, timeout, name):
        if not client.wait_for_server(timeout_sec=5.0):
            return False, "%s server missing" % name, None
        event = threading.Event()
        send = client.send_goal_async(goal)
        send.add_done_callback(lambda _f: event.set())
        if not event.wait(10.0):
            return False, "%s goal send timeout" % name, None
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, "%s rejected" % name, None
        event.clear()
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            handle.cancel_goal_async()
            return False, "%s result timeout" % name, None
        wrapped = result_fut.result()
        result = wrapped.result
        status = wrapped.status
        ok = status == GoalStatus.STATUS_SUCCEEDED and bool(result.success)
        return ok, str(result.message), result

    def vacuum(self, enable, timeout=20.0):
        req = VacuumCommand.Request()
        req.enable = bool(enable)
        response = self._call_srv(self._vacuum, req, timeout)
        if response is None:
            return False, "vacuum timeout"
        return bool(response.success), str(response.message)

    def goto_observe(self):
        goal = GoToRobotPose.Goal()
        goal.pose_name = self._args.observe_pose
        return self._send_action(
            self._goto, goal, self._args.goto_timeout, "GoToRobotPose")

    def detect_once(self):
        return self._call_srv(
            self._detect, DetectLuggage.Request(), self._args.detect_timeout)

    def _auth_policy(self):
        return PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=int(self._args.auth_max_attempts),
            max_elapsed_sec=float(self._args.auth_max_elapsed_sec),
            wait_period_sec=float(self._args.auth_wait_period_sec),
        ))

    def detect_until_authorized(self):
        def _once():
            detect = self.detect_once()
            if detect is None:
                return False, None, "DETECT_TIMEOUT"
            if not detect.success or not detect.luggage:
                return False, None, detect.message or "MEASURED_NONE"
            return True, detect.luggage[0], detect.message or ""

        return run_authorization_loop(
            _once, time.sleep, time.monotonic, self._auth_policy())

    def detect(self):
        last = None
        attempts = max(1, int(self._args.detect_retries))
        for attempt in range(attempts):
            last = self.detect_once()
            if last is not None and last.success and last.luggage:
                return last
            msg = last.message if last else "timeout"
            self.get_logger().warn(
                "detect attempt %d/%d: %s" % (attempt + 1, attempts, msg))
            if attempt + 1 < attempts:
                time.sleep(2.0)
        return last

    def wait_graph(self):
        timeout = float(self._args.ready_timeout)
        deadline = time.monotonic() + timeout
        needed = [
            (self._detect, "DetectLuggage", "service"),
            (self._build, "BuildMotionSequence", "service"),
        ]
        if (self._args.use_vacuum
                and not self._args.detect_only
                and not self._args.plan_only
                and not self._args.dry_run):
            needed.append((self._vacuum, "VacuumCommand", "service"))
        for client, name, kind in needed:
            if not self._wait_ready(client, name, kind, deadline, timeout):
                return False, "%s missing after %.0fs" % (name, timeout)
        if not self._args.skip_observe:
            if not self._wait_ready(
                    self._goto, "GoToRobotPose", "action", deadline, timeout):
                return False, "GoToRobotPose missing after %.0fs" % timeout
        if self._args.detect_only or self._args.plan_only or self._args.dry_run:
            return True, "observe/detect graph ready"
        if not self._wait_ready(
                self._plan, "PlanMotion", "action", deadline, timeout):
            return False, "PlanMotion missing after %.0fs" % timeout
        if not self._wait_ready(
                self._probe, "ProbeMotionSegment", "service", deadline, timeout):
            return False, "ProbeMotionSegment missing"
        return True, "pick graph ready"

    def _wait_ready(self, client, name, kind, deadline, timeout):
        """Poll readiness. Do not call wait_for_service while this node spins.

        wait_for_service/wait_for_server on the same node as
        MultiThreadedExecutor.spin can block past timeout with no log.
        """
        self.get_logger().info(
            "waiting for %s (%s, timeout %.0fs)" % (name, kind, timeout))
        while time.monotonic() < deadline:
            if kind == "action":
                ready = bool(client.server_is_ready())
            else:
                ready = bool(client.service_is_ready())
            if ready:
                self.get_logger().info("%s ready" % name)
                return True
            time.sleep(0.25)
        return False

    @staticmethod
    def _box_geom(box):
        return pickup_collision_aabb(box)

    def add_scene_box(self, box):
        scene = self._scene_client()
        if not scene.wait_ready(timeout_sec=5.0):
            return False, "apply_planning_scene unavailable"
        try:
            xyz, quat, size = pickup_collision_aabb(box)
        except ValueError as exc:
            return False, str(exc)
        ok, message = scene.add_pickup_box(xyz, quat, size)
        if ok:
            self._scene_box = (xyz, quat, size)
        return ok, message

    def add_scene_box_detection(self, detection):
        """Scene add from a session DetectionView (world-frame geometry)."""
        scene = self._scene_client()
        if not scene.wait_ready(timeout_sec=5.0):
            return False, "apply_planning_scene unavailable"
        ok, message = scene.add_pickup_box(
            list(detection.box_xyz), list(detection.box_quat),
            list(detection.box_size))
        if ok:
            self._scene_box = (list(detection.box_xyz),
                               list(detection.box_quat),
                               list(detection.box_size))
        return ok, message

    def set_pickup_touch(self, allowed):
        from luggage_planning.planning_scene_client import (
            BOX_OBJECT_ID,
            DEFAULT_TOUCH_LINKS,
        )
        pairs = [(BOX_OBJECT_ID, link) for link in DEFAULT_TOUCH_LINKS]
        return self._scene_client().set_acm_pairs(
            pairs, bool(allowed), verify=False)

    def attach_scene_box(self):
        if self._scene_box is None:
            return False, "no scene box"
        xyz, quat, size = self._scene_box
        return self._scene_client().attach_pickup_box(
            "pickup_box", xyz, quat, size)

    def remove_scene_box(self):
        if self._scene_box is None:
            return True, "no scene box"
        ok, message = self._scene_client().detach_and_remove()
        self._scene_box = None
        return ok, message

    # ------------------------------------------------------------------
    # candidate pick flow

    def _load_contact_model(self):
        from luggage_description._share import description_config_path
        from luggage_description.suction_contact_model import (
            load_suction_contact_model,
        )
        path = self._args.contact_model or description_config_path(
            "suction_contact_model.yaml")
        model = load_suction_contact_model(path)
        self.get_logger().info(
            "suction contact model %s sha256=%s version=%s footprint=%.3fx%.3f"
            % (path, model.identity_hash, model.model_version,
               model.footprint_size_xy_m[0], model.footprint_size_xy_m[1]))
        return model

    def _detection_view(self, box):
        import math
        orientation = box.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                          + orientation.z * orientation.z))
        stamp, frame = adapters.detected_observation_identity(box)
        xyz, quat, size = pickup_collision_aabb(box)
        return DetectionView(
            stamp=stamp,
            frame=frame,
            candidates=adapters.suction_candidates_from_detected(box),
            top_surface_valid=bool(box.top_surface_valid),
            detection_yaw=yaw,
            yaw_valid=bool(getattr(box, "yaw_valid", True)),
            box_xyz=xyz,
            box_quat=quat,
            box_size=size)

    def _session_config(self, model):
        return SessionConfig(
            max_candidates=int(self._args.max_candidates),
            release_low_hold_sec=float(self._args.release_low_hold_sec),
            release_deadline_sec=float(self._args.release_deadline_sec),
            reverse_min_m=float(self._args.reverse_min_m),
            required_cartesian_fraction=float(
                self._args.required_cartesian_fraction),
            request_deadline_sec=float(self._args.request_deadline_sec),
            use_scene=(not self._args.no_scene),
            use_vacuum=bool(self._args.use_vacuum),
            planning_frame="world",
            contact_model_version=model.model_version,
            contact_model_hash=model.identity_hash,
            dry_run=bool(self._args.dry_run),
            dry_run_count=int(self._args.dry_run_count),
        )

    def _log_detection(self, box, detection):
        pos = box.pose.position
        self.get_logger().info(
            "detect id=%s xyz=(%.3f,%.3f,%.3f) size=%.3fx%.3fx%.3f "
            "top_valid=%s candidates=%d"
            % (box.id, pos.x, pos.y, pos.z, box.width, box.depth,
               box.height, box.top_surface_valid,
               len(detection.candidates)))
        for view in rank_candidates(detection.candidates):
            contact = view.contact.position
            self.get_logger().info(
                "  candidate %s rank=%d contact=(%.3f,%.3f,%.3f) "
                "score=%.3f coverage=%.3f p95=%.4f model=%s/%s"
                % (view.candidate_id, view.rank, contact.x, contact.y,
                   contact.z, view.score, view.valid_coverage,
                   view.p95_residual, view.model_version,
                   view.model_hash[:12]))

    def run(self):
        self.report_stage("WAIT_GRAPH")
        self.get_logger().warn(
            "hardware pick: e-stop person required; vacuum uses box DO0/DO1")
        ready, ready_msg = self.wait_graph()
        if not ready:
            self.get_logger().error("graph not ready: %s" % ready_msg)
            return 1
        self.get_logger().info(ready_msg)

        if self._args.skip_observe:
            self.get_logger().warn(
                "skip-observe: detect/pick from current joints, no %s"
                % self._args.observe_pose)
        else:
            ok, message, _ = self.goto_observe()
            if not ok:
                self.get_logger().error("observe failed: %s" % message)
                return 2
            self.get_logger().info(
                "at %s: %s" % (self._args.observe_pose, message))
        self.report_stage("OBSERVE")
        time.sleep(self._args.settle_sec)

        if self._args.detect_only:
            detect = self.detect()
            if detect is None or not detect.success or not detect.luggage:
                msg = detect.message if detect else "timeout"
                self.get_logger().error("detect failed: %s" % msg)
                return 3
            box = detect.luggage[0]
            self.get_logger().info(
                "detect-only id=%s height_valid=%s height_source=%s "
                "top_valid=%s xyz=(%.3f,%.3f,%.3f) size=%.3fx%.3fx%.3f"
                % (box.id, box.height_valid, box.height_source,
                   box.top_surface_valid, box.pose.position.x,
                   box.pose.position.y, box.pose.position.z,
                   box.width, box.depth, box.height))
            return 0

        self.report_stage("DETECT")
        auth = self.detect_until_authorized()
        self.get_logger().info(
            "pick_authorization action=%s reason=%s attempts=%s"
            % (auth.decision.action, auth.decision.reason,
               auth.decision.attempts))
        if not auth.authorized:
            self.get_logger().error(
                "pick not authorized: %s (%s)"
                % (auth.decision.reason, auth.detect_reason))
            return 3
        box = auth.box
        detection = self._detection_view(box)
        if self.detection_guard is not None:
            ok, message = self.detection_guard(detection)
            if not ok:
                self.get_logger().error("MOTION_GUARD: %s" % message)
                return 12
        self._log_detection(box, detection)

        # Validate the policy/grasp conversion before any pick motion. The
        # successful candidate is only known after PickSession's retry loop.
        place_sequences = {}
        if self._args.place_target_data is not None and not self._args.dry_run:
            try:
                for candidate in detection.candidates:
                    place_sequences[candidate.candidate_id] = build_place_segments(
                        self._args.place_target_data, detection, candidate)
            except (ValueError, TypeError) as exc:
                self.get_logger().error("place target unusable: %s" % exc)
                return 4

        try:
            model = self._load_contact_model()
        except Exception as exc:  # noqa: BLE001 - fail closed at startup
            self.get_logger().error("contact model unusable: %s" % exc)
            return 9

        if self._args.dry_run:
            ports = RosPickSessionPorts(self, self)
            try:
                session = PickSession(
                    ports, self._session_config(model))
                result = session.run_dry_run()
                self.get_logger().info(
                    "dry run complete: %d detections, no motion/vacuum"
                    % self._args.dry_run_count)
                return result.exit_code
            finally:
                ports.close()

        if self._args.plan_only:
            if not detection.candidates:
                self.get_logger().error(
                    "no sealable candidates: refusing to plan box-centre")
                return 3
            for view in rank_candidates(detection.candidates):
                segments = build_candidate_pick_segments(
                    view, detection_yaw=detection.detection_yaw,
                    yaw_valid=detection.yaw_valid)
                self.get_logger().info(
                    "plan %s: %s" % (
                        view.candidate_id,
                        [(s.name, s.type, round(s.target_pose.position.z, 3))
                         for s in segments]))
                for segment in place_sequences.get(view.candidate_id, []):
                    self.get_logger().info("place preview %s: %s" % (
                        view.candidate_id, segment))
            return 0

        ports = RosPickSessionPorts(self, self,
                                    log_path=self._args.trace_out or None)
        try:
            session = PickSession(ports, self._session_config(model))
            result = session.run_request(detection)
            self.get_logger().info(
                "pick session: exit=%d reason=%s detail=%s candidate=%s"
                % (result.exit_code, result.reason_code, result.detail,
                   result.selected_candidate_id))
            if result.completed and self._args.place_target_data is not None:
                # Do not enable the pick-only shutdown release handler: every
                # place fault needs explicit recovery, including Ctrl-C.
                segments = place_sequences.get(result.selected_candidate_id)
                if not segments:
                    self.get_logger().error("PLACE_CARRY_FAULT: selected grasp missing")
                    return 10
                selected = next((c for c in detection.candidates
                                 if c.candidate_id == result.selected_candidate_id), None)
                segments = self.prepare_place_segments(segments, detection, selected)
                for segment in segments:
                    ports.log_event(dict(stage="place_target", name=segment.name,
                                         pose=vars(segment.target_pose.position),
                                         orientation=vars(segment.target_pose.orientation)))
                code, reason = run_place(
                    ports, segments, use_scene=not self._args.no_scene,
                    low_hold_sec=self._args.release_low_hold_sec,
                    release_deadline_sec=self._args.release_deadline_sec)
                self.get_logger().info("place session: exit=%d reason=%s" % (code, reason))
                return code
            if result.completed and self._args.use_vacuum and self._args.release:
                vac_ok, vac_msg = self.vacuum(False)
                self.get_logger().info(
                    "vacuum release: %s (%s)" % (vac_ok, vac_msg))
                self.remove_scene_box()
            elif (result.completed and self._args.use_vacuum):
                self._release_on_exit = True
                self.get_logger().warn(
                    "holding suction (DI0). rerun with --release or Ctrl-C "
                    "to drop")
            elif result.exit_code == 10:
                self.get_logger().error(
                    "CARRY FAULT: vacuum preserved; explicit recovery "
                    "required (DI0 still monitored)")
            return result.exit_code
        finally:
            ports.close()

    def prepare_place_segments(self, segments, detection=None, candidate=None):
        """Optional commissioning connector, called only after successful Pick."""
        return segments

    def shutdown_vacuum(self):
        if self._release_on_exit:
            self.get_logger().warn("releasing vacuum on exit")
            self.vacuum(False)
            self.remove_scene_box()
            self._release_on_exit = False


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--observe-pose", default="current",
        help="GoToRobotPose name. current/here = stay at live joints. "
             "Do not send simulation pickup_observe on this cell.")
    parser.add_argument(
        "--skip-observe", action="store_true",
        help="Do not GoToRobotPose; detect and pick in place.")
    parser.add_argument("--detect-only", action="store_true")
    parser.add_argument(
        "--plan-only", action="store_true",
        help="Detect and print per-candidate pick waypoints; no motion.")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="C7 no-motion protocol: N detections printing ranked "
             "candidates; zero probe/motion/vacuum/scene calls.")
    parser.add_argument("--dry-run-count", type=int, default=10)
    parser.add_argument("--no-vacuum", dest="use_vacuum", action="store_false")
    parser.add_argument(
        "--no-scene", action="store_true",
        help="Do not add the detected box to MoveIt (debug).")
    parser.add_argument(
        "--release", action="store_true",
        help="Drop the box after pick_retreat (default holds suction).")
    parser.add_argument(
        "--place-target", default="", metavar="JSON",
        help="After pick, place at the policy's world-frame box-centre pose. "
             "See deployment_ws/docs/policy_place.md for the JSON contract.")
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument("--goto-timeout", type=float, default=90.0)
    parser.add_argument("--detect-timeout", type=float, default=40.0)
    parser.add_argument("--detect-retries", type=int, default=5)
    parser.add_argument("--auth-max-attempts", type=int, default=5)
    parser.add_argument("--auth-max-elapsed-sec", type=float, default=5.0)
    parser.add_argument("--auth-wait-period-sec", type=float, default=0.5)
    parser.add_argument("--plan-timeout", type=float, default=90.0)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--max-candidates", type=int, default=3)
    parser.add_argument("--release-low-hold-sec", type=float, default=0.5)
    parser.add_argument("--release-deadline-sec", type=float, default=2.0)
    parser.add_argument("--reverse-min-m", type=float, default=0.08)
    parser.add_argument("--required-cartesian-fraction", type=float,
                        default=1.0)
    parser.add_argument("--request-deadline-sec", type=float, default=180.0)
    parser.add_argument(
        "--contact-model", default="",
        help="Suction contact model YAML (default: luggage_description "
             "share suction_contact_model.yaml).")
    parser.add_argument(
        "--trace-out", default="",
        help="Write the session JSONL trace to this path (evidence).")
    parser.set_defaults(use_vacuum=True)
    return parser


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(args=argv)
    args.place_target_data = None
    if args.place_target:
        if args.release or args.no_scene or not args.use_vacuum:
            parser.error("--place-target requires scene/vacuum and cannot combine with --release")
        try:
            with open(args.place_target, encoding="utf-8") as stream:
                args.place_target_data = parse_place_target(json.load(stream))
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            parser.error("invalid --place-target: %s" % exc)

    rclpy.init()
    node = HardwarePickDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def _stop(*_unused):
        node.shutdown_vacuum()
        executor.shutdown()

    signal.signal(signal.SIGINT, _stop)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        code = node.run()
    finally:
        node.shutdown_vacuum()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
