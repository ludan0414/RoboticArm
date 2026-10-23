#!/usr/bin/env python3
"""Vacuum controller node: the one VacuumCommand contract for sim + hardware.

Default backend is sim. hardware_pick.launch.py sets backend:=hardware.
HardwareVacuumBackend is 实机代码 (site / real-cell).

/vacuum/command (luggage_msgs/VacuumCommand) is the same signal:
  sim      -> kinematic follow in gz + PlanningScene attach
  stub     -> state only
  hardware -> box DO0/DO1 via the CPS executor; DI0 is sealed

Publishes /vacuum/state (luggage_msgs/VacuumState, transient-local).
Hardware attach does not use Gazebo current_box or VacuumGate; DI0 is
the hold signal (measured ~6.3 s to rise).
"""

from __future__ import division

import json
import math
import threading
import time

import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from luggage_msgs.msg import VacuumState
from luggage_msgs.srv import GetCurrentBox, VacuumCommand

from luggage_planning.current_box_payload import (
    identity_from_current_box_payload,
    measured_from_current_box_payload,
)
from luggage_planning.vacuum_backend import (
    HardwareVacuumBackend,
    SimVacuumBackend,
    StubVacuumBackend,
)
from luggage_planning.vacuum_gate import VacuumGate

WORLD = "world"


def _pose_to_lists(pose):
    return ([pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])


class GzSetPoseClient(object):
    """Adapts the bridged /world/<w>/set_pose service to the backend's
    minimal gz_client interface."""

    def __init__(self, node, group, world):
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        self._Entity = Entity
        self._SetEntityPose = SetEntityPose
        self._client = node.create_client(
            SetEntityPose, "/world/%s/set_pose" % world,
            callback_group=group)
        self._ready = False

    def set_model_pose(self, model_name, xyz, quat, timeout_sec=0.05):
        if not self._ready:
            if not self._client.wait_for_service(timeout_sec=5.0):
                return False, "set_pose service unavailable"
            self._ready = True
        elif not self._client.service_is_ready():
            return False, "set_pose service unavailable"
        event = threading.Event()
        request = self._SetEntityPose.Request()
        request.entity.name = model_name
        request.entity.type = self._Entity.MODEL
        request.pose.position.x = float(xyz[0])
        request.pose.position.y = float(xyz[1])
        request.pose.position.z = float(xyz[2])
        request.pose.orientation.x = float(quat[0])
        request.pose.orientation.y = float(quat[1])
        request.pose.orientation.z = float(quat[2])
        request.pose.orientation.w = float(quat[3])
        future = self._client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(float(timeout_sec)):
            return False, "set_pose timeout"
        response = future.result()
        if response is None or not response.success:
            return False, "set_pose rejected"
        return True, ""


class ExecutorVacuumIo(object):
    """实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

    Box DO writes through the CPS-owning executor; DI0 from /vacuum/di0.
    """

    def __init__(self, node, group, do0_service, do1_service, di0_topic):
        self._node = node
        from luggage_planning.vacuum_io_health import FreshDigitalInput, VacuumIoTransport
        self._input = FreshDigitalInput(time.monotonic, float(node.get_parameter("di0_max_age_sec").value))
        self._do0 = node.create_client(
            SetBool, do0_service, callback_group=group)
        self._do1 = node.create_client(
            SetBool, do1_service, callback_group=group)
        def request(value):
            req = SetBool.Request()
            req.data = value
            return req
        self._transport = VacuumIoTransport(
            {0: self._do0, 1: self._do1}, request, time.monotonic, time.sleep,
            lambda record: node.get_logger().info("vacuum_io: " + json.dumps(record)),
            service_timeout=node.get_parameter("io_service_timeout_sec").value,
            call_timeout=node.get_parameter("io_call_timeout_sec").value,
            connect_attempts=node.get_parameter("io_connect_attempts").value,
            retry_period=node.get_parameter("io_retry_period_sec").value)
        node.create_subscription(
            Bool, di0_topic, self._on_di0, 10, callback_group=group)

    def _on_di0(self, msg):
        self._input.update(msg.data)

    def di0(self):
        return self._input.value()

    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(max(0.0, float(dt)))

    def set_do(self, bit, value):
        return self._transport.set_do(bit, value)


class VacuumControllerNode(Node):

    def __init__(self):
        super().__init__("vacuum_controller")
        group = ReentrantCallbackGroup()

        self.declare_parameter("backend", "sim")  # sim | stub | hardware
        self.declare_parameter("world", "airport_loading")
        self.declare_parameter("panel_frame", "suction_contact_frame")
        self.declare_parameter("follow_rate_hz", 30.0)
        self.declare_parameter("do0_service", "/elfin/vacuum/set_do0")
        self.declare_parameter("do1_service", "/elfin/vacuum/set_do1")
        self.declare_parameter("di0_topic", "/vacuum/di0")
        self.declare_parameter("seal_timeout_sec", 8.0)
        self.declare_parameter("seal_poll_sec", 0.1)
        self.declare_parameter("release_timeout_sec", 1.0)
        self.declare_parameter("blowoff_hold_sec", 0.1)
        self.declare_parameter("di0_max_age_sec", 2.0)
        self.declare_parameter("io_service_timeout_sec", 2.0)
        self.declare_parameter("io_call_timeout_sec", 2.0)
        self.declare_parameter("io_connect_attempts", 2)
        self.declare_parameter("io_retry_period_sec", 0.2)
        self._command_lock = threading.Lock()
        # Gate parameters (defaults mirror the ROS 1 simulator).
        self.declare_parameter("pressure_kpa", 70.0)
        self.declare_parameter("effective_area_m2", 0.012)
        self.declare_parameter("seal_efficiency", 0.80)
        self.declare_parameter("friction_coefficient", 0.60)
        self.declare_parameter("minimum_retention_margin", 2.0)
        self.declare_parameter("max_suction_tilt_deg", 5.0)
        self.declare_parameter("contact_margin", 0.05)
        self.declare_parameter("contact_xy_margin", 0.05)
        # Spawn AABB vs visible lid: attach often sits ~1 cm into the cube.
        self.declare_parameter("contact_gap_min", -0.02)

        self._panel_frame = str(self.get_parameter("panel_frame").value)
        self._gate = VacuumGate(
            pressure_kpa=float(self.get_parameter("pressure_kpa").value),
            effective_area_m2=float(
                self.get_parameter("effective_area_m2").value),
            seal_efficiency=float(
                self.get_parameter("seal_efficiency").value),
            friction_coefficient=float(
                self.get_parameter("friction_coefficient").value),
            minimum_retention_margin=float(
                self.get_parameter("minimum_retention_margin").value),
            max_suction_tilt_deg=float(
                self.get_parameter("max_suction_tilt_deg").value),
            contact_margin=float(
                self.get_parameter("contact_margin").value),
            contact_xy_margin=float(
                self.get_parameter("contact_xy_margin").value),
            contact_gap_min=float(
                self.get_parameter("contact_gap_min").value))

        self._tf_buffer = tf2_ros.Buffer()
        # Humble TransformListener(spin_thread=True) add_node()s this node.
        # A sidecar keeps /tf off the command/follow executor.
        # use_global_arguments=False: launch -r __node:=... must not rename
        # this sidecar, or ros2 node list reports a duplicate name.
        self._tf_node = Node(
            "vacuum_controller_tf", use_global_arguments=False)
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)
        self._follow_lock = threading.Lock()
        self._follow_skipped = 0

        # Box instance from the spawner (transient-local JSON, schema 2):
        # identity + MEASURED geometry only. GT for the sim physics path is
        # pulled from get_current_box at attach time.
        self._identity = ("", 0)
        self._measured = None
        box_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/current_box", self._on_current_box, box_qos,
            callback_group=group)
        self._get_box_cli = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box",
            callback_group=group)

        state_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            VacuumState, "/vacuum/state", state_qos)
        self._diag_pub = self.create_publisher(
            String, "/vacuum/events_json", state_qos)

        backend_name = str(self.get_parameter("backend").value)
        if backend_name == "stub":
            self._backend = StubVacuumBackend()
        elif backend_name == "hardware":
            io = ExecutorVacuumIo(
                self, group,
                str(self.get_parameter("do0_service").value),
                str(self.get_parameter("do1_service").value),
                str(self.get_parameter("di0_topic").value))
            self._backend = HardwareVacuumBackend(
                io,
                seal_timeout_sec=float(
                    self.get_parameter("seal_timeout_sec").value),
                seal_poll_sec=float(
                    self.get_parameter("seal_poll_sec").value),
                release_timeout_sec=float(
                    self.get_parameter("release_timeout_sec").value),
                blowoff_hold_sec=float(
                    self.get_parameter("blowoff_hold_sec").value))
        else:
            from luggage_planning.planning_scene_client import PlanningSceneClient
            scene = PlanningSceneClient(self, callback_group=group)
            gz_client = GzSetPoseClient(
                self, group, str(self.get_parameter("world").value))
            self._backend = SimVacuumBackend(
                gz_client, scene,
                follow_rate_hz=float(
                    self.get_parameter("follow_rate_hz").value))

        self._vacuum_on = False
        self._last_fail_reason = ""
        self._last_gate = None

        self.create_service(
            VacuumCommand, "/vacuum/command", self._handle_command,
            callback_group=group)

        follow_period = 1.0 / max(1.0, float(
            self.get_parameter("follow_rate_hz").value))
        self.create_timer(follow_period, self._follow_tick,
                          callback_group=group)

        self.get_logger().info(
            "vacuum_controller ready (backend=%s, panel=%s)"
            % (backend_name, self._panel_frame))

    # ------------------------------------------------------------------

    def _on_current_box(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self._identity = identity_from_current_box_payload(data)
        self._measured = measured_from_current_box_payload(data)
        if self._identity[0]:
            if self._measured is not None:
                self.get_logger().info(
                    "current_box id=%s measured=%.3fx%.3fx%.3f gen=%d"
                    % (self._identity[0], self._measured["width"],
                       self._measured["depth"], self._measured["height"],
                       self._identity[1]))
            else:
                self.get_logger().info(
                    "current_box id=%s gen=%d (no measured geometry yet)"
                    % (self._identity[0], self._identity[1]))
        else:
            self.get_logger().info("current_box cleared")

    def _pull_gt_box(self, timeout=5.0):
        """Pull the GT spawn record for the CURRENT instance epoch.

        Sim-physics context only (gate contact, retention mass, gz follow).
        The MoveIt attach geometry comes from the topic's measured record.
        Returns None unless the pulled generation matches the latched
        identity, so a spawn swap between topic and pull cannot mix epochs.
        """
        if not self._get_box_cli.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = self._get_box_cli.call_async(GetCurrentBox.Request())
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        resp = future.result()
        if resp is None or not resp.success:
            return None
        if int(resp.generation) != int(self._identity[1]):
            return None
        box = resp.box
        position = box.pose.position
        orientation = box.pose.orientation
        return {
            "model_name": str(resp.model_name or ""),
            "mass_kg": float(resp.mass_kg or 0.0),
            "size": [float(box.width), float(box.depth), float(box.height)],
            "xyz": [float(position.x), float(position.y), float(position.z)],
            "quat": [float(orientation.x), float(orientation.y),
                     float(orientation.z), float(orientation.w)],
            "generation": int(resp.generation),
        }

    def _panel_pose(self):
        try:
            stamp = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform(
                WORLD, self._panel_frame, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = transform.transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    # ------------------------------------------------------------------

    def _handle_command(self, request, response):
        if not self._command_lock.acquire(blocking=False):
            response.success, response.message = False, "VACUUM_BUSY: command in progress"
            return response
        start = time.monotonic()
        try:
            self.get_logger().info("vacuum command start: enable=%s" % bool(request.enable))
            return self._perform_command(request, response)
        finally:
            self.get_logger().info("vacuum command end: enable=%s ok=%s elapsed=%.3fs message=%s" %
                (bool(request.enable), response.success, time.monotonic()-start, response.message))
            self._command_lock.release()

    def _perform_command(self, request, response):
        enable = bool(request.enable)
        if not enable:
            ok, message = self._backend.detach({})
            # A failed write/release must not advertise a confirmed pump-off.
            self._vacuum_on = not ok
            self._last_fail_reason = "" if ok else message
            self._publish_state()
            self._publish_event("detach", ok, message)
            response.success = ok
            response.message = message
            return response

        if isinstance(self._backend, HardwareVacuumBackend):
            ok, message = self._backend.attach({})
            # A failed seal/RPC can leave the physical pump enabled. Keep a
            # conservative on/unknown state until an explicit release succeeds.
            self._vacuum_on = True
            self._last_fail_reason = "" if ok else message
            self._publish_state()
            self._publish_event("attach", ok, message)
            response.success = ok
            response.message = message
            return response

        panel = self._panel_pose()
        if panel is None:
            response.success = False
            response.message = "VACUUM_BACKEND_ERROR: panel TF unavailable"
            self._last_fail_reason = response.message
            self._publish_state()
            return response

        # Planning artifacts consume the MEASURED geometry only. Refuse to
        # attach without it: falling back to the GT spawn size would put
        # privileged geometry into the MoveIt planning scene.
        measured = self._measured
        if measured is None:
            response.success = False
            response.message = (
                "ATTACH_REFUSED: no measured geometry synced "
                "(call /pickup_box_spawner/sync_detected_pickup_box first)")
            self._last_fail_reason = response.message
            self._publish_state()
            self._publish_event("attach", False, response.message)
            return response

        # Sim-physics context (contact gate, retention, gz follow) may use
        # the GT spawn record, pulled for the current instance epoch.
        gt = self._pull_gt_box()
        if gt is None:
            response.success = False
            response.message = (
                "VACUUM_BACKEND_ERROR: GT pull unavailable for generation %d"
                % self._identity[1])
            self._last_fail_reason = response.message
            self._publish_state()
            self._publish_event("attach", False, response.message)
            return response

        size = list(gt["size"])
        mass_kg = gt["mass_kg"]
        radius = 0.5 * math.sqrt(sum(float(v) ** 2 for v in size))

        gate = self._gate.evaluate(
            panel[0], panel[1], gt["xyz"], size, mass_kg, radius)
        self._last_gate = gate
        if not gate.ok:
            self._vacuum_on = False
            self._last_fail_reason = gate.reason
            self._publish_state()
            self._publish_event("attach_gate", False, gate.reason)
            response.success = False
            response.message = gate.reason
            return response

        context = {
            "model_name": gt["model_name"],
            "panel_xyz": panel[0], "panel_quat": panel[1],
            "box_xyz": gt["xyz"], "box_quat": gt["quat"],
            # MoveIt attach geometry: perception-measured, not GT.
            "scene_box_size": [
                measured["width"], measured["depth"], measured["height"]],
        }
        ok, message = self._backend.attach(context)
        self._vacuum_on = ok
        self._last_fail_reason = "" if ok else message
        self._publish_state()
        attach_note = "%s; scene=%.3fx%.3fx%.3f@gen%d(measured)" % (
            message, measured["width"], measured["depth"],
            measured["height"], measured["generation"])
        self._publish_event("attach", ok, attach_note)
        response.success = ok
        response.message = message
        return response

    # ------------------------------------------------------------------

    def _follow_tick(self):
        if isinstance(self._backend, HardwareVacuumBackend):
            previous_reason = self._last_fail_reason
            ok, message = self._backend.follow_step(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            if not ok:
                self._last_fail_reason = message
            elif self._vacuum_on and not self._backend.is_attached():
                self._last_fail_reason = "VACUUM_SEAL_LOST"
            elif self._last_fail_reason.startswith("VACUUM_IO_STALE"):
                self._last_fail_reason = ""
            if previous_reason != self._last_fail_reason:
                self.get_logger().warning("vacuum health changed: %s -> %s" %
                                          (previous_reason, self._last_fail_reason or "ok"))
            self._publish_state()
            return
        if not self._backend.is_attached():
            self._publish_state()
            return
        if not self._follow_lock.acquire(blocking=False):
            self._follow_skipped += 1
            self._publish_event("follow_skip", True, "busy")
            return
        try:
            panel = self._panel_pose()
            if panel is None:
                return
            ok, message = self._backend.follow_step(panel[0], panel[1])
            if not ok:
                self._last_fail_reason = message
            self._publish_state()
        finally:
            self._follow_lock.release()

    def _publish_state(self):
        state = VacuumState()
        state.attached = bool(self._backend.is_attached())
        state.vacuum_on = bool(self._vacuum_on)
        state.fail_reason = self._last_fail_reason or ""
        if self._last_gate is not None:
            state.contact_distance = float(self._last_gate.contact_distance)
            state.retention_margin = float(self._last_gate.retention_margin)
            state.tilt_deg = float(self._last_gate.tilt_deg)
        self._state_pub.publish(state)

    def _publish_event(self, event, ok, message):
        record = {
            "event": event,
            "ok": bool(ok),
            "message": message,
            "follow_skipped": int(self._follow_skipped),
        }
        if self._last_gate is not None:
            record["contact_distance"] = self._last_gate.contact_distance
            record["retention_margin"] = self._last_gate.retention_margin
            record["tilt_deg"] = self._last_gate.tilt_deg
        self._diag_pub.publish(String(data=json.dumps(
            record, sort_keys=True)))


def main(argv=None):
    rclpy.init(args=argv)
    node = VacuumControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        listener = getattr(node, "_tf_listener", None)
        tf_exec = getattr(listener, "executor", None) if listener else None
        if tf_exec is not None:
            tf_exec.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        tf_node = getattr(node, "_tf_node", None)
        node.destroy_node()
        if tf_node is not None:
            tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
