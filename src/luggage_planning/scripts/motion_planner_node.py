#!/usr/bin/env python3
"""PlanMotion action server + GoToRobotPose (ROS 2 Humble port).

One segment per PlanMotion goal; the eval driver loops the four pick
segments itself. Feedback walks planning -> executing -> settling
(``settle_criterion.SettleTracker`` on /joint_states) so the caller can
wait for geometry_ok to recover before the next segment.

GoToRobotPose resolves a named pose from robot_poses YAML to joint angles
and sends them via plain FJT (no MoveIt; that is what observe_pose_hold
does, and named poses are joint-space anyway). Pose name ``current`` /
``here`` stays at live /joint_states (no FJT). Site launch uses
``robot_poses.site.yaml``; do not send simulation ``pickup_observe``.

Keeps the original node name per docs/plans/closed_loop_pick_retreat_nodes.md.
"""

from __future__ import division

import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import PlanningSceneComponents
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import SetParametersResult
from luggage_planning.trajectory_smoothing import validate_speed

from luggage_description.scene_tf_config_utils import (
    _point_in_container_link,
    load_scene_tf_config,
    origin_in_world,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
)
from luggage_msgs.action import GoToRobotPose, PlanMotion
from luggage_msgs.srv import ProbeMotionSegment
from luggage_planning.debug_capture import write_failure_capture
from luggage_planning.motion_boundary import robot_traj_to_dict, write_boundary_dump
from luggage_planning.motion_executor import MotionExecutor, _wrap_near
from luggage_planning.named_robot_poses import (
    JOINTS,
    load_poses_config,
    plan_goto_joints,
    resolve_pose_name,
)
from luggage_planning.occupancy_place_paths import (
    OccupancyMismatch,
    OccupancySnapshot,
    exempt_footprint_locals,
    occupancy_collision_boxes,
    load_selector_weights,
    resolve_payload_wdh,
)
from luggage_planning.waypoint_generator import tool_down_yaw
from luggage_planning.current_box_payload import (
    measured_from_current_box_json,
)
from luggage_planning.planning_scene_client import (
    PlanningSceneClient,
    summarize_planning_scene,
)
from luggage_planning.ros_clock_wait import ClockTimeout, wait_event
from luggage_planning.settle_criterion import SettleTracker


def occupancy_scope(names_csv, segment_name):
    """True when ``segment_name`` is inside the occupancy-injection scope.

    ``names_csv`` is a comma-separated segment list; ``*`` widens it to
    every segment (the placement.md contract: every place-motion planner
    queries current cargo occupancy). Narrow it to ``"transit,traverse"``
    to restore the carry-only scope.
    """
    names = {part.strip() for part in str(names_csv or "").split(",")
             if part.strip()}
    return "*" in names or str(segment_name) in names


class MotionPlannerNode(Node):

    def __init__(self):
        super().__init__("motion_planner")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("robot_poses_config", "")
        self.declare_parameter("default_observe_pose", "current")
        self.declare_parameter("execute_timeout", 90.0)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("planner_id", "RRTConnect")
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_min_fraction", 0.95)
        self.declare_parameter("software_collision_checks", True)
        # Legacy parameter: diagnostic reference only. It cannot re-enable
        # rad/m rejection; collisions, wrist checks and limits are independent.
        self.declare_parameter("cartesian_excursion_rad_per_m", 3.0)
        self.declare_parameter("velocity_scaling", 0.3)
        self.declare_parameter("acceleration_scaling", 0.3)
        self.declare_parameter('speed_scale', 1.0)
        validate_speed(self.get_parameter('speed_scale').value)
        self.add_on_set_parameters_callback(self._validate_speed_parameters)
        self.declare_parameter("max_segment_joint_excursion_rad", 0.0)
        # Settle gate between segments.
        self.declare_parameter("settle_vel_tol", 0.02)
        self.declare_parameter("settle_hold_time", 0.5)
        self.declare_parameter("settle_timeout", 8.0)
        # Cap on GoToRobotPose FJT time_from_start. Actual duration is
        # min(cap, max_joint_delta / named_pose_max_vel), at least 1 s.
        self.declare_parameter("named_pose_duration", 4.0)
        self.declare_parameter("named_pose_max_vel", 1.0)
        # Must match elfin_trajectory_executor action_name (Jazzy real launch).
        self.declare_parameter(
            "fjt_action", "/elfin_arm_controller/follow_joint_trajectory")
        self.declare_parameter("scene_tf_config", "")
        # Conservative fallback envelope ONLY: the occupancy sweep prefers
        # the perception-measured geometry synced onto /luggage/current_box
        # (privilege boundary — no GT spawn size on the chain).
        self.declare_parameter("payload_width", 0.55)
        self.declare_parameter("payload_depth", 0.40)
        self.declare_parameter("payload_height", 0.25)
        weights = load_selector_weights()
        self.declare_parameter("inflate_m", float(weights["inflate_m"]))
        self.declare_parameter("arm_radius_m", float(weights["arm_radius_m"]))
        self.declare_parameter("max_occ_objects", int(weights["max_occ_objects"]))
        self.declare_parameter(
            "occupancy_freshness_s",
            float(weights.get("occupancy_freshness_s", 0.0)))
        # Which segments get cargo occupancy injected: "*" (default, the
        # placement.md "place motion through occupancy" contract) or a
        # comma list ("transit,traverse" restores the carry-only scope).
        self.declare_parameter("occupancy_segments", "*")
        # Extra relief on top of arm_radius + inflate for the landing
        # footprint exemption; tune in sim without code changes.
        self.declare_parameter("occupancy_exempt_margin_m", 0.0)
        # Failure-time occ/cloud/depth capture (never blocks motion).
        self.declare_parameter("debug_capture", True)

        scene_path = str(self.get_parameter("scene_tf_config").value or "")
        self._scene = load_scene_tf_config(
            resolve_scene_tf_config_path(scene_path or None))
        self._surface_2d = None
        self._surface_recv = None
        self._payload_measured = None
        self._payload_recv = None
        self._payload_source = "static_default"
        self._payload_wdh_current = []
        self._injected_ids = []
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_surface, map_qos, callback_group=self._group)
        self.create_subscription(
            String, "/luggage/current_box", self._on_current_box, map_qos,
            callback_group=self._group)
        # Failure-capture caches: raw messages only, decoded at dump time.
        # The semantic cloud feeds the cargo map (occupancy provenance);
        # the preprocessed depth is the sensor frame underneath it.
        self._cap_cloud = None
        self._cap_cloud_recv = None
        self._cap_depth = None
        self._cap_depth_recv = None
        if bool(self.get_parameter("debug_capture").value):
            cloud_qos = QoSProfile(
                depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(
                PointCloud2, "/luggage/semantic/cargo_points_untracked",
                self._on_cap_cloud, cloud_qos, callback_group=self._group)
            self.create_subscription(
                Image, "/luggage/preprocessed/camera/depth/image",
                self._on_cap_depth, 1, callback_group=self._group)
        self._scene_client = PlanningSceneClient(
            self, callback_group=self._group)

        self._joint_state = None
        self._joint_lock = threading.Lock()
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self._group)

        self._executor_client = MotionExecutor(
            self,
            allowed_planning_time=float(
                self.get_parameter("planning_time").value),
            num_planning_attempts=int(
                self.get_parameter("num_planning_attempts").value),
            planner_id=str(self.get_parameter("planner_id").value),
            cartesian_max_step=float(
                self.get_parameter("cartesian_max_step").value),
            cartesian_avoid_collisions=bool(
                self.get_parameter("software_collision_checks").value),
            velocity_scaling=float(
                self.get_parameter("velocity_scaling").value),
            acceleration_scaling=float(
                self.get_parameter("acceleration_scaling").value),
            max_segment_joint_excursion_rad=float(
                self.get_parameter("max_segment_joint_excursion_rad").value),
            cartesian_min_fraction=float(
                self.get_parameter("cartesian_min_fraction").value),
            cartesian_excursion_rad_per_m=float(
                self.get_parameter("cartesian_excursion_rad_per_m").value),
        )
        self._fjt = rclpy.action.ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter("fjt_action").value),
            callback_group=self._group)

        self._plan_action = ActionServer(
            self, PlanMotion, "/motion_planner/plan_motion",
            execute_callback=self._execute_plan_motion,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._group)
        self._pose_action = ActionServer(
            self, GoToRobotPose, "/motion_planner/go_to_robot_pose",
            execute_callback=self._execute_goto_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._group)
        # Plan-only reachability probe (candidate selection, gate C1):
        # IK + raw Cartesian fraction; never moves the arm.
        self._probe_service = self.create_service(
            ProbeMotionSegment, "/motion_planner/probe_motion_segment",
            self._handle_probe_segment, callback_group=self._group)
        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._boundary_pub = self.create_publisher(
            String, "/motion_planner/last_boundary", latch)

        # Do not wait_for_server here: __init__ runs before the executor
        # spins, so discovery of /move_action always times out and the
        # node sits 60 s before advertising PlanMotion. execute_callback
        # already calls wait_ready(15s) per goal.
        self.get_logger().info(
            "motion_planner up (move_group probed on first PlanMotion)")

    # ------------------------------------------------------------------
    # shared plumbing

    def _validate_speed_parameters(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name == 'speed_scale':
                    validate_speed(parameter.value)
            return SetParametersResult(successful=True)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _accept_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _on_surface(self, msg):
        try:
            surface = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(surface, dict) and "height" in surface:
            self._surface_2d = surface
            self._surface_recv = time.time()

    def _on_current_box(self, msg):
        measured = measured_from_current_box_json(msg.data)
        if measured is not None:
            self._payload_measured = measured
            self._payload_recv = time.time()

    def _on_cap_cloud(self, msg):
        self._cap_cloud = msg
        self._cap_cloud_recv = time.time()

    def _on_cap_depth(self, msg):
        self._cap_depth = msg
        self._cap_depth_recv = time.time()

    def _world_to_map(self, xyz):
        base = xyz_world_to_base_link(self._scene, xyz)
        return _point_in_container_link(base, self._scene)

    def _map_to_world(self, xyz):
        origin, rpy = origin_in_world(self._scene)
        yaw = float(rpy[2])
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = [float(v) for v in xyz]
        return [
            origin[0] + c * x - s * y,
            origin[1] + s * x + c * y,
            origin[2] + z,
        ]

    def _payload_wdh(self):
        """Occupancy-sweep payload box: measured geometry when synced."""
        wdh, source = resolve_payload_wdh(self._payload_measured, [
            float(self.get_parameter("payload_width").value),
            float(self.get_parameter("payload_depth").value),
            float(self.get_parameter("payload_height").value),
        ])
        self._payload_source = source
        self._payload_wdh_current = wdh
        return wdh

    def _exempt_for(self, segment, snapshot):
        """Landing-footprint exemption for the segment's in-hull waypoints.

        Relief, not a gate: any failure to derive it returns no exemption
        (the sweep then behaves like the old carry-only scope would).
        """
        waypoints = [(wp.position.x, wp.position.y, wp.position.z)
                     for wp in (getattr(segment, "waypoints", None) or [])]
        target = getattr(segment, "target_pose", None)
        yaw = 0.0
        if target is not None and hasattr(target, "position"):
            waypoints.append((target.position.x, target.position.y,
                              target.position.z))
            try:
                quat = target.orientation
                yaw = tool_down_yaw(quat.x, quat.y)
            except (AttributeError, TypeError, ValueError):
                yaw = 0.0
        try:
            mapped = [self._world_to_map(wp) for wp in waypoints]
            return exempt_footprint_locals(
                snapshot, mapped, self._payload_wdh(), yaw=yaw,
                arm_radius=float(self.get_parameter("arm_radius_m").value),
                inflate_m=float(self.get_parameter("inflate_m").value),
                margin_m=float(
                    self.get_parameter("occupancy_exempt_margin_m").value))
        except Exception:  # noqa: BLE001 - exemption must not block motion
            return []

    def _prepare_occupancy(self, segment):
        """Inject occupancy boxes and pin the executor sweep. Empty on success.

        Scope is ``occupancy_segments`` ("*" by default: insert, descend,
        retreat, place_exit and the staging segments are checked too, per
        placement.md "place motion through occupancy"). Waypoints inside
        the hull anchor a landing-footprint exemption so the slot's own
        support and inflated neighbours cannot block the insertion they
        exist for.
        """
        if not self.get_parameter("software_collision_checks").value:
            self._executor_client.set_occupancy(None)
            return ""
        name = str(getattr(segment, "name", ""))
        if not occupancy_scope(
                self.get_parameter("occupancy_segments").value, name):
            self._executor_client.set_occupancy(None)
            return ""
        surface = self._surface_2d
        if surface is None:
            return ""
        freshness = float(self.get_parameter("occupancy_freshness_s").value)
        if (freshness > 0.0 and self._surface_recv is not None
                and (time.time() - self._surface_recv) > freshness):
            snap_try = OccupancySnapshot.from_surface_2d(
                surface, inflate_m=float(self.get_parameter("inflate_m").value))
            if snap_try.occupied_count() > 0:
                return "PLACE_PATH_INFEASIBLE occupancy stale"
        try:
            snapshot = OccupancySnapshot.from_surface_2d(
                surface,
                inflate_m=float(self.get_parameter("inflate_m").value))
        except OccupancyMismatch as exc:
            return str(exc)
        exempt = self._exempt_for(segment, snapshot)
        boxes = occupancy_collision_boxes(
            snapshot,
            max_objects=int(self.get_parameter("max_occ_objects").value),
            exempt=exempt)
        world_boxes = []
        for box in boxes:
            world_boxes.append({
                "id": box["id"],
                "xyz": self._map_to_world(box["xyz"]),
                "size": box["size"],
                "quat": box["quat"],
            })
        if world_boxes:
            ok, message = self._scene_client.add_collision_boxes(
                world_boxes, frame_id="world")
            if not ok:
                return "PLACE_PATH_INFEASIBLE occupancy inject: %s" % message
            self._injected_ids = [box["id"] for box in world_boxes]
        self._executor_client.set_occupancy(
            snapshot, self._payload_wdh(),
            world_to_map=self._world_to_map,
            arm_radius=float(self.get_parameter("arm_radius_m").value),
            inflate_m=float(self.get_parameter("inflate_m").value),
            object_count=len(world_boxes),
            exempt_locals=exempt)
        return ""

    def _clear_occupancy(self):
        self._executor_client.set_occupancy(None)
        ids = list(self._injected_ids)
        self._injected_ids = []
        if ids:
            self._scene_client.remove_objects(ids)

    def _on_joint_state(self, msg):
        if set(JOINTS) <= set(msg.name):
            with self._joint_lock:
                self._joint_state = msg

    def _scene_summary(self):
        scene = self._scene_client.get_scene(
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
            timeout=3.0)
        return summarize_planning_scene(scene)

    def _occ_summary(self):
        surface = self._surface_2d
        if surface is None:
            return {"present": False}
        return {
            "present": True,
            "map_revision": surface.get("map_revision"),
            "geometry_hash": surface.get("geometry_hash"),
            "age_s": (None if self._surface_recv is None
                      else round(time.time() - self._surface_recv, 3)),
        }

    def _capture_failure(self, t_wall, name):
        """Freeze occ + source cloud + depth at failure time. Never raises.

        Writes only when MOTION_BOUNDARY_DUMP is set; the record always
        carries the occ summary and what was or was not captured.
        """
        try:
            if not bool(self.get_parameter("debug_capture").value):
                return {"enabled": False}
            dump_root = os.environ.get("MOTION_BOUNDARY_DUMP", "")
            if not dump_root:
                return {"enabled": True, "reason": "no_dump_root"}
            tag = "%d_%s" % (int(float(t_wall) * 1000.0), name)
            return write_failure_capture(
                dump_root, tag,
                surface=self._surface_2d,
                surface_age_s=(None if self._surface_recv is None
                               else round(time.time() - self._surface_recv,
                                          3)),
                cloud_msg=self._cap_cloud,
                depth_msg=self._cap_depth)
        except Exception as exc:  # noqa: BLE001 - dump path must not fail
            return {"error": str(exc)}

    def _emit_boundary(self, name, extra=None, success=True):
        record = {"name": str(name or ""), "t_wall": time.time()}
        record.update(self._executor_client.last_boundary() or {})
        record["payload_source"] = self._payload_source
        record["payload_wdh"] = list(self._payload_wdh_current)
        if extra:
            record.update(extra)
        if not success:
            record["planning_scene"] = self._scene_summary()
            record["occ"] = self._occ_summary()
            record["capture"] = self._capture_failure(record["t_wall"], name)
        path = write_boundary_dump(
            os.environ.get("MOTION_BOUNDARY_DUMP", ""), record)
        if path:
            record["dump"] = path
        try:
            self._boundary_pub.publish(
                String(data=json.dumps(record, default=str)))
        except Exception:  # noqa: BLE001 - dump path must not fail the motion
            pass
        return record

    def _joint_positions(self):
        with self._joint_lock:
            if self._joint_state is None:
                return None, None
            by_name = dict(zip(self._joint_state.name,
                               self._joint_state.position))
            by_vel = dict(zip(self._joint_state.name,
                              self._joint_state.velocity
                              if len(self._joint_state.velocity) else []))
            return ([float(by_name.get(j, 0.0)) for j in JOINTS],
                    [float(by_vel.get(j, 0.0)) for j in JOINTS])

    def _poses_config(self):
        path = str(self.get_parameter("robot_poses_config").value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(
                get_package_share_directory("luggage_description"),
                "config", "robot_poses.site.yaml")
        return load_poses_config(path), path

    # ------------------------------------------------------------------
    # ProbeMotionSegment (plan-only; no motion, no vacuum)

    def _handle_probe_segment(self, request, response):
        segment = request.segment
        response.fraction = -1.0
        response.moveit_error_code = 0
        if not self._executor_client.wait_ready(15.0):
            response.success = False
            response.message = "move_group not ready"
            return response
        if not self.get_parameter("software_collision_checks").value:
            ok, message = self._scene_client.allow_all_collisions()
            if not ok:
                response.success = False
                response.message = message
                return response
        # Probes see the same occupancy the executed segment would: scope,
        # landing exemption, injected boxes (plan-only, nothing moves).
        occ_err = self._prepare_occupancy(segment)
        if occ_err:
            self._clear_occupancy()
            response.success = False
            response.ik_ok = False
            response.message = occ_err
            return response
        try:
            start = self._joint_positions()[0]
            record = self._executor_client.probe_segment(segment, start)
        finally:
            self._clear_occupancy()
        response.ik_ok = bool(record.get("ik_ok"))
        fraction = record.get("fraction")
        response.fraction = -1.0 if fraction is None else float(fraction)
        response.success = bool(response.ik_ok)
        response.message = "ik_ok=%s fraction=%.3f" % (
            response.ik_ok, response.fraction)
        return response

    # ------------------------------------------------------------------
    # PlanMotion

    def _execute_plan_motion(self, goal_handle):
        # Capture once. UI changes while idle; no time law changes mid-action.
        self._executor_client.set_speed_scale(self.get_parameter('speed_scale').value)
        segment = goal_handle.request.segment
        result = PlanMotion.Result()
        name = str(segment.name)
        result.fraction = 0.0
        result.used_ompl_fallback = False
        result.moveit_error_code = 0
        result.settle_json = ""

        def feedback(stage, segment_name, fraction, note=""):
            fb = PlanMotion.Feedback()
            fb.stage = str(stage)
            fb.segment_name = str(segment_name)
            fb.fraction = float(fraction)
            goal_handle.publish_feedback(fb)
            if note:
                self.get_logger().info("segment %s: %s" % (segment_name, note))

        # Generous: first graph discovery inside this process can take
        # seconds even when move_group has been up the whole time.
        if not self._executor_client.wait_ready(timeout_sec=15.0):
            goal_handle.abort()
            result.success = False
            result.message = "move_group unavailable"
            self._emit_boundary(name, extra={"kind": "PlanMotion"}, success=False)
            return result

        if not self.get_parameter("software_collision_checks").value:
            ok, message = self._scene_client.allow_all_collisions()
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = message
                self._emit_boundary(name, extra={"kind": "PlanMotion"}, success=False)
                return result

        try:
            occ_err = self._prepare_occupancy(segment)
            if occ_err:
                goal_handle.abort()
                result.success = False
                result.message = occ_err
                self._emit_boundary(
                    name, extra={"kind": "PlanMotion"}, success=False)
                return result
            exec_result = self._executor_client.execute_segment(
                segment, feedback_cb=feedback,
                execute_timeout=float(
                    self.get_parameter("execute_timeout").value),
                current_joints=self._joint_positions()[0],
                cancel_check=lambda: goal_handle.is_cancel_requested)
            result.fraction = float(exec_result.fraction)
            result.used_ompl_fallback = bool(exec_result.used_ompl_fallback)
            result.moveit_error_code = int(exec_result.moveit_error_code)
            ok = bool(exec_result.success)
            message = exec_result.message
            fraction = exec_result.fraction
            target = segment.target_pose.position
            boundary = {
                "kind": "PlanMotion",
                "segment_type": str(segment.type),
                "target": [float(target.x), float(target.y), float(target.z)],
            }
            if segment.type == 'commissioning_home':
                from luggage_planning.commissioning_home import HOME_JOINTS_DEG, HOME_JOINTS_RAD
                boundary.pop('target')
                boundary['target_joints_deg'] = list(HOME_JOINTS_DEG)
                boundary['target_joints_rad'] = list(HOME_JOINTS_RAD)
            from luggage_planning.manual_carry_transfer import TRANSFER_TYPES, TRANSFER_JOINTS_DEG, TRANSFER_JOINTS_RAD
            if segment.type in TRANSFER_TYPES:
                index = TRANSFER_TYPES.index(segment.type)
                boundary.pop('target')
                boundary['target_joints_deg'] = list(TRANSFER_JOINTS_DEG[index])
                boundary['target_joints_rad'] = list(TRANSFER_JOINTS_RAD[index])
            self._emit_boundary(name, extra=boundary, success=ok)
        except Exception as exc:  # noqa: BLE001 - action boundary
            self.get_logger().error("segment %s raised: %s" % (name, exc))
            goal_handle.abort()
            result.success = False
            result.message = "executor raised: %s" % exc
            self._emit_boundary(name, extra={"kind": "PlanMotion"}, success=False)
            return result
        finally:
            self._clear_occupancy()

        if not ok:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            result.success = False
            result.message = message
            return result

        settled, settle_diag = self._wait_settled(
            lambda: feedback("settling", name, fraction))
        boundary = self._executor_client.last_boundary() or {}
        if boundary.get('reach_projections'):
            settle_diag['reach_projections'] = boundary['reach_projections']
            recovered = boundary.get('cartesian_recovery') or []
            settle_diag['executed_transit_poses'] = [
                item['achieved_pose'] for item in recovered if item.get('success') and item.get('achieved_pose')]
        result.settle_json = json.dumps(settle_diag, sort_keys=True)
        if not settled:
            goal_handle.abort()
            result.success = False
            result.message = "%s; settle timeout" % message
            return result

        from luggage_planning.manual_carry_transfer import (
            TRANSFER_TYPES, TRANSFER_SETTLE_TOLERANCE_RAD, target_joint_error,
        )
        if segment.type in TRANSFER_TYPES:
            error = target_joint_error(segment.type, self._joint_positions()[0])
            if error is None or error > TRANSFER_SETTLE_TOLERANCE_RAD:
                goal_handle.abort()
                result.success = False
                result.message = ('%s; P1/P2 actual joint target not reached: %s rad'
                                  % (message, 'missing' if error is None else '%.4f' % error))
                return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "%s; canceled after settle" % message
            return result

        goal_handle.succeed()
        result.success = True
        result.message = message
        return result

    def _named_pose_duration(self, current, target):
        cap = max(0.5, float(self.get_parameter("named_pose_duration").value))
        max_vel = max(0.1, float(self.get_parameter("named_pose_max_vel").value))
        if current is None:
            return cap
        max_delta = max(
            abs(float(a) - float(b)) for a, b in zip(current, target))
        return max(1.0, min(cap, max_delta / max_vel))

    def _wait_settled(self, pulse, timeout=None):
        timeout = timeout or float(self.get_parameter("settle_timeout").value)
        tracker = SettleTracker(
            float(self.get_parameter("settle_vel_tol").value),
            float(self.get_parameter("settle_hold_time").value))
        timer = ClockTimeout(self.get_clock(), timeout)
        while not timer.done():
            time.sleep(0.05)
            positions, velocities = self._joint_positions()
            if positions is None:
                continue
            pos_d = dict(zip(JOINTS, positions))
            vel_d = dict(zip(JOINTS, velocities or [0.0] * len(JOINTS)))
            tracker.update(timer.elapsed(), vel_d, pos_d)
            if tracker.settled_at is not None:
                return True, tracker.diagnostics()
            pulse()
        return tracker.settled_at is not None, tracker.diagnostics()

    # ------------------------------------------------------------------
    # GoToRobotPose (named joint pose via FJT)

    def _execute_goto_pose(self, goal_handle):
        result = GoToRobotPose.Result()
        requested = str(goal_handle.request.pose_name)

        def feedback(stage, remaining_error=0.0):
            fb = GoToRobotPose.Feedback()
            fb.stage = str(stage)
            fb.remaining_error = float(remaining_error)
            goal_handle.publish_feedback(fb)

        positions, _ = self._joint_positions()
        path = ""
        pose_name = requested
        try:
            config, path = self._poses_config()
            pose_name = resolve_pose_name(
                requested, config,
                param_override=str(
                    self.get_parameter("default_observe_pose").value))
            target = plan_goto_joints(pose_name, config, positions)
        except Exception as exc:  # noqa: BLE001 - action boundary
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result
        if target is None:
            goal_handle.succeed()
            result.success = True
            result.already_there = True
            result.message = "already at %s (current joints)" % pose_name
            return result
        target = list(target)

        positions, _ = self._joint_positions()
        if positions is not None:
            target = [
                _wrap_near(cur, tgt) for cur, tgt in zip(positions, target)]
        if positions is not None and max(
                abs(a - b) for a, b in zip(positions, target)) < 0.02:
            goal_handle.succeed()
            result.success = True
            result.already_there = True
            result.message = "already at %s" % pose_name
            return result

        if not self._fjt.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = "FJT action server unavailable"
            return result

        feedback("executing")
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        start = list(positions) if positions is not None else list(target)
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0] * len(JOINTS)
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0] * len(JOINTS)
        duration = self._named_pose_duration(start, target)
        sec = int(duration)
        p1.time_from_start = Duration(
            sec=sec, nanosec=int((duration - sec) * 1e9))
        traj.points = [p0, p1]
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        event = threading.Event()
        future = self._fjt.send_goal_async(goal)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(10.0):
            goal_handle.abort()
            result.success = False
            result.message = "FJT goal send timeout"
            return result
        handle = future.result()
        if not handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = "FJT goal rejected"
            return result

        result_event = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: result_event.set())
        reached, wait_reason = wait_event(
            result_event, duration + 60.0, clock=self.get_clock())
        if not reached:
            handle.cancel_goal()
            goal_handle.abort()
            result.success = False
            extra = " (%s)" % wait_reason if wait_reason else ""
            result.message = "FJT execution timeout%s" % extra
            return result
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                "FJT named pose failed (%s %s); MoveIt joint fallback"
                % (wrapped.status, wrapped.result.error_code))
            moveit = self._executor_client.execute_joints(
                target, execute_timeout=float(
                    self.get_parameter("execute_timeout").value),
                current_joints=positions)
            extra = {
                "kind": "GoToRobotPose",
                "pose_name": pose_name,
                "method": "fjt_then_moveit",
                "fjt_status": int(wrapped.status),
                "fjt_error_code": int(wrapped.result.error_code),
                "fjt_trajectory": robot_traj_to_dict(traj),
                "start_joints": list(start),
                "target_joints": list(target),
            }
            if not moveit.success:
                goal_handle.abort()
                result.success = False
                result.message = "FJT status=%s error_code=%s; %s" % (
                    wrapped.status, wrapped.result.error_code, moveit.message)
                self._emit_boundary(
                    "goto_%s" % pose_name, extra=extra, success=False)
                return result
            settled, _diag = self._wait_settled(
                lambda: feedback("settling"))
            if not settled:
                goal_handle.abort()
                result.success = False
                result.message = "reached %s via MoveIt but settle timeout" % pose_name
                self._emit_boundary(
                    "goto_%s" % pose_name, extra=extra, success=False)
                return result
            goal_handle.succeed()
            result.success = True
            result.already_there = False
            result.message = "reached %s (MoveIt joint fallback)" % pose_name
            self._emit_boundary(
                "goto_%s" % pose_name, extra=extra, success=True)
            return result

        extra = {
            "kind": "GoToRobotPose",
            "pose_name": pose_name,
            "method": "fjt",
            "fjt_trajectory": robot_traj_to_dict(traj),
            "start_joints": list(start),
            "target_joints": list(target),
        }
        settled, _diag = self._wait_settled(
            lambda: feedback("settling"))
        if not settled:
            goal_handle.abort()
            result.success = False
            result.message = "reached %s but settle timeout" % pose_name
            self._emit_boundary(
                "goto_%s" % pose_name, extra=extra, success=False)
            return result

        goal_handle.succeed()
        result.success = True
        result.already_there = False
        result.message = "reached %s" % pose_name
        self._emit_boundary(
            "goto_%s" % pose_name, extra=extra, success=True)
        return result


def main(argv=None):
    rclpy.init(args=argv)
    node = MotionPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
