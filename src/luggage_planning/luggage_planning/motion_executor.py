#!/usr/bin/env python3
"""MoveIt 2 client wrapper for one motion segment at a time.

Node-layer code (per docs/architecture/perception_architecture.md): it
imports moveit_msgs at the top level by design; algorithm modules must not
import it. Not a Node subclass - construct with an already-initialized node
(so the caller owns executors/callback groups) and call ``execute_segment``.

Segment routing (docs/plans/closed_loop_pick_retreat_nodes.md):

    pose_target  -> MoveGroup plan_only, then rest-to-rest ExecuteTrajectory
                    (MoveIt TOTG execute stays off; Huayan CollisionStop)
    cartesian    -> GetCartesianPath geometry, wrist gate, excursion diagnostics,
                    occupancy sweep, then rest-to-rest ExecuteTrajectory
    named pose   -> handled by the node via plain FJT (see motion_planner_node)

``keep_tool_down`` adds an orientation constraint (tool Z down).
``keep_camera_down`` / ``lock_wrist`` are NOT implemented on purpose; the
result message says so instead of silently ignoring the flags.
"""

from __future__ import division

import math
import threading
from dataclasses import dataclass

from luggage_planning.joint6_hold import (
    joint6_value,
    pick_holds_joint6,
    pin_joint6_trajectory,
)
from luggage_planning.motion_boundary import robot_traj_to_dict
from luggage_planning.motion_excursion import (
    cartesian_excursion, excursion_diagnostic,
    _CARTESIAN_EXCURSION_RAD_PER_M, _CARTESIAN_EXCURSION_MIN_PATH_M,
)
from luggage_planning.ros_clock_wait import wait_event
from luggage_planning.trajectory_timing import time_parameterize_cartesian

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point as PointMsg, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PositionIKRequest,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from shape_msgs.msg import SolidPrimitive

# Tool-Z (suction normal) pointing at world -Z: quaternion (x=1, w=0).
TOOL_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)
JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]
# Match S20 URDF revolute limits (±2π). Used only to pick the nearest
# equivalent wrap of an IK solution so OMPL does not command a 4 rad spin.
_JOINT_LIMIT = 6.28
_VEL_SCALE = 0.3
_ACC_SCALE = 0.3
# elfin joint limits are uniform (URDF velocity 1.57 rad/s, elfin_moveit_config
# joint_limits.yaml acceleration 3.14 rad/s^2).
_JOINT_VEL_LIMIT = 1.57
_JOINT_ACC_LIMIT = 3.14
# A cartesian solution may be smooth in joint space and still unwind the arm
# through a near-singular reconfiguration: motion_occ/2026-09-21_1838
# place_exit spent 6.58 rad of elfin_joint1 on a 1.39 m hop (4.72 rad/m,
# 19.3 s) and the open-loop controller missed the goal by 0.91 rad. Every
# legitimate cartesian segment in that run stayed at 0.93-1.50 rad/m, so the
# bound sits above the accepted set with 2x margin. jump_threshold does not
# catch this: the rejected path's largest adjacent step was 0.157 rad.
# Below this Cartesian length the ratio is dominated by its denominator (a
# 9 mm traverse reads 2.07 rad/m for 0.02 rad of motion), so short hops are
# judged against the floor instead.
# The wrist is degenerate at elfin_joint5 = k*pi, where joint4 and joint6
# become collinear and a small end-effector change can demand a ~pi sweep of
# both. Across the 45 recorded segments in motion_occ/2026-09-21_* the two
# sets separate exactly: every segment whose joint5 track crossed a k*pi
# boundary was a wrist flip (|dj5| 3.12-6.28 rad, closest approach <= 0.069
# rad), and every segment that did not cross stayed >= 0.454 rad away. So the
# crossing test alone classifies the recorded set with no threshold. The
# margin below is the backstop for a sparse waypoint chain that could step
# over a boundary without landing near it; it sits between the two clusters
# (3.6x above the rejected set, 1.8x below the accepted set). Note 0.454 rad
# is an accepted segment, so a margin at 0.50 would refuse a path that
# provably did not flip.
_WRIST_JOINT_INDEX = 4
_WRIST_SINGULARITY_MARGIN_RAD = 0.25


@dataclass
class SegmentExecResult:
    success: bool
    message: str
    fraction: float = 0.0
    used_ompl_fallback: bool = False
    moveit_error_code: int = 0
    occupancy_checked: bool = False
    occupancy_object_count: int = 0
    trajectory: object = None
    start_joints: object = None


class _OccupancyBlockedPlan(object):
    occupancy_blocked = True
    solution = None


class _WristSingularPlan(object):
    """A solution that reconfigures or operates inside the wrist singularity.

    The excursion diagnostic can look plausible while joint5 still steps
    across a branch boundary; wrist checks remain independent.
    """

    wrist_singular = True
    solution = None

    def __init__(self, reason, measurement):
        self.reason = str(reason)
        self.measurement = dict(measurement)


def _wrist_cell(value):
    """Which [k*pi, (k+1)*pi) band joint5 sits in.

    Band boundaries are the degenerate configurations, so a change of cell
    between two consecutive waypoints means the solution reconfigured the
    wrist through a singularity rather than moving within one branch.
    """
    return math.floor(float(value) / math.pi)


def wrist_singularity(points, margin_rad=_WRIST_SINGULARITY_MARGIN_RAD,
                      joint_index=_WRIST_JOINT_INDEX):
    """Wrist degeneracy check for one solution's joint5 track.

    ``points`` are objects with ``positions`` (a JointTrajectory) or plain
    joint vectors. Returns ``None`` when there is nothing to judge, otherwise
    a dict with the closest approach to k*pi, whether the track crosses a
    branch boundary, and ``ok``.
    """
    rows = [list(getattr(p, "positions", p)) for p in (points or [])]
    track = [float(r[joint_index]) for r in rows if len(r) > joint_index]
    if len(track) < 2:
        return None
    margin = min(abs(v - round(v / math.pi) * math.pi) for v in track)
    crosses = any(
        _wrist_cell(a) != _wrist_cell(b) for a, b in zip(track, track[1:]))
    return {
        "joint": (JOINTS[joint_index] if joint_index < len(JOINTS)
                  else "joint%d" % joint_index),
        "margin_rad": round(margin, 4),
        "margin_limit_rad": float(margin_rad),
        "delta_rad": round(abs(track[-1] - track[0]), 4),
        "crosses_branch": bool(crosses),
        "ok": (not crosses) and margin >= float(margin_rad),
    }


def wrist_singularity_reason(measurement):
    """Human-readable refusal text for a failed ``wrist_singularity``."""
    if measurement["crosses_branch"]:
        return (
            "wrist branch change: %s crosses a multiple of pi (%.3f rad of "
            "travel, closest approach %.4f rad)" % (
                measurement["joint"], measurement["delta_rad"],
                measurement["margin_rad"]))
    return (
        "wrist near singularity: %s within %.4f rad of a multiple of pi "
        "(limit %.3f)" % (
            measurement["joint"], measurement["margin_rad"],
            measurement["margin_limit_rad"]))


def _sec_to_duration(sec):
    nsec_total = int(round(float(sec) * 1e9))
    if nsec_total < 0:
        nsec_total = 0
    return DurationMsg(
        sec=nsec_total // 1000000000,
        nanosec=nsec_total % 1000000000,
    )


def apply_rest_to_rest_to_joint_trajectory(joint_trajectory, vel_scale, acc_scale):
    """Replace TOTG or linear times with the site rest-to-rest trapezoid.

    Keeps the geometric knots. Writes time_from_start, velocities, and
    accelerations capped to 60 deg/s and 20 deg/s^2 on both ramps.
    """
    points = list(joint_trajectory.points)
    if len(points) < 2:
        return None
    timed = time_parameterize_cartesian(
        [list(pt.positions) for pt in points], vel_scale, acc_scale)
    for point, t_s, vel, acc in zip(
            points, timed.times_s, timed.velocities_rad,
            timed.accelerations_rad):
        point.time_from_start = _sec_to_duration(t_s)
        point.velocities = list(vel)
        point.accelerations = list(acc)
    return timed


def _wrap_near(current, target, lower=-_JOINT_LIMIT, upper=_JOINT_LIMIT):
    """Choose target + k·2π inside limits that is closest to current."""
    best = target
    best_err = abs(target - current)
    for k in range(-3, 4):
        cand = target + k * 2.0 * math.pi
        if cand < lower - 1e-6 or cand > upper + 1e-6:
            continue
        err = abs(cand - current)
        if err < best_err:
            best, best_err = cand, err
    return best


class MotionExecutor:

    def __init__(self, node, group_name="elfin_arm",
                 link_name="suction_contact_frame",
                 planner_id="RRTConnect",
                 allowed_planning_time=5.0,
                 num_planning_attempts=10,
                 planning_frame="world",
                 cartesian_max_step=0.01,
                 cartesian_min_fraction=0.95,
                 cartesian_avoid_collisions=True,
                 cartesian_excursion_rad_per_m=_CARTESIAN_EXCURSION_RAD_PER_M,
                 wrist_singularity_margin_rad=_WRIST_SINGULARITY_MARGIN_RAD,
                 tool_down_abs_tol=0.05,
                 velocity_scaling=_VEL_SCALE,
                 acceleration_scaling=_ACC_SCALE,
                 max_segment_joint_excursion_rad=0.0):
        self._node = node
        self._max_segment_joint_excursion_rad = float(max_segment_joint_excursion_rad)
        self._group = str(group_name)
        self._link = str(link_name)
        self._planner_id = str(planner_id)
        self._planning_time = float(allowed_planning_time)
        self._attempts = int(num_planning_attempts)
        self._frame = str(planning_frame)
        self._max_step = float(cartesian_max_step)
        self._min_fraction = float(cartesian_min_fraction)
        self._avoid_collisions = bool(cartesian_avoid_collisions)
        self._excursion_limit = float(cartesian_excursion_rad_per_m)
        self._wrist_margin = float(wrist_singularity_margin_rad)
        self._vel_scale = float(velocity_scaling)
        self._acc_scale = float(acceleration_scaling)
        self._tool_down_abs_tol = float(tool_down_abs_tol)
        # Set per execute_segment call (goals are serialized: the stack
        # never runs two motion segments at once, move_group would refuse
        # the second). When the outer action goal is cancelled, inner
        # action handles are cancelled so the controller stops instead of
        #         grinding on past the caller's timeout.
        self._cancel_check = None
        self._last_ik_code = 0

        import rclpy
        self._rclpy = rclpy
        # MoveIt 2 Humble serves the MoveGroup action at /move_action
        # (node info on the move_group node); /move_group/action/move_group
        # has no server in this distro.
        self._move_group = rclpy.action.ActionClient(
            node, MoveGroup, "/move_action")
        self._execute = rclpy.action.ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self._ik = node.create_client(GetPositionIK, "/compute_ik")
        self._fk = node.create_client(GetPositionFK, "/compute_fk")
        self._occupancy = None
        self._payload_wdh = None
        self._world_to_map = None
        self._occupancy_exempt = []
        self._arm_radius = 0.08
        self._inflate_m = 0.05
        self._occupancy_object_count = 0
        self._planned_traj = None
        self._start_joints = []
        self._last_boundary = {}
        self._cartesian_excursion = None
        self._wrist_measurement = None

    # ------------------------------------------------------------------

    def wait_ready(self, timeout_sec=60.0):
        """True when the MoveGroup/ExecuteTrajectory servers and the
        cartesian service are all available."""
        ok = self._move_group.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            return False
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return False
        if not self._cartesian.wait_for_service(timeout_sec=5.0):
            return False
        return self._ik.wait_for_service(timeout_sec=5.0)

    def set_occupancy(self, snapshot, payload_wdh=None, world_to_map=None,
                      arm_radius=None, inflate_m=None, object_count=0,
                      exempt_locals=None):
        """Pin cargo occupancy for the next segment. ``snapshot=None`` clears.

        ``exempt_locals`` are grid-local landing-footprint AABBs
        (``exempt_footprint_locals``): cargo cells inside them do not
        collide, so approach/insertion segments aimed at their own slot are
        not blocked by the slot's support and inflated neighbours.
        """
        self._occupancy = snapshot
        self._payload_wdh = None if payload_wdh is None else [
            float(v) for v in payload_wdh]
        self._world_to_map = world_to_map
        if arm_radius is not None:
            self._arm_radius = float(arm_radius)
        if inflate_m is not None:
            self._inflate_m = float(inflate_m)
        self._occupancy_object_count = int(object_count)
        self._occupancy_exempt = list(exempt_locals or [])

    def _result(self, success, message, fraction=0.0, used_ompl=False,
                error_code=0):
        return SegmentExecResult(
            bool(success), str(message), float(fraction), bool(used_ompl),
            int(error_code),
            occupancy_checked=self._occupancy is not None,
            occupancy_object_count=self._occupancy_object_count,
        )

    # ------------------------------------------------------------------

    def last_boundary(self):
        return dict(self._last_boundary or {})

    def _stamp_boundary(self, result, name):
        traj = robot_traj_to_dict(self._planned_traj)
        result.trajectory = traj
        result.start_joints = list(self._start_joints or [])
        self._last_boundary = {
            "name": str(name or ""),
            "success": bool(result.success),
            "message": str(result.message),
            "fraction": float(result.fraction),
            "used_ompl_fallback": bool(result.used_ompl_fallback),
            "moveit_error_code": int(result.moveit_error_code),
            "occupancy_checked": bool(result.occupancy_checked),
            "occupancy_object_count": int(result.occupancy_object_count),
            "start_joints": list(self._start_joints or []),
            "trajectory": traj,
        }
        if self._cartesian_excursion is not None:
            self._last_boundary["cartesian_excursion"] = dict(
                self._cartesian_excursion)
        if self._occupancy is not None:
            from luggage_planning.occupancy_place_paths import (
                occupancy_collision_boxes)
            boxes = occupancy_collision_boxes(
                self._occupancy, max_objects=64,
                exempt=self._occupancy_exempt)
            self._last_boundary["occupancy_boxes"] = [
                {"id": box["id"], "xyz": list(box["xyz"]),
                 "size": list(box["size"])}
                for box in boxes]
        return result

    def execute_segment(self, segment_msg, feedback_cb=None,
                        execute_timeout=45.0, current_joints=None,
                        cancel_check=None):
        """Execute one luggage_msgs/MotionSegment.

        Returns ``SegmentExecResult``. ``fraction`` is 1.0 for pose-target
        paths and the computed Cartesian fraction otherwise.
        ``cancel_check`` is polled while inner action goals run; a True
        return cancels the inner goal so the controller stops.
        """
        self._cancel_check = cancel_check
        self._planned_traj = None
        self._cartesian_excursion = None
        self._start_joints = list(current_joints or [])
        try:
            result = self._execute_segment_inner(
                segment_msg, feedback_cb, execute_timeout, current_joints)
            return self._stamp_boundary(
                result, getattr(segment_msg, "name", ""))
        finally:
            self._cancel_check = None

    def _execute_segment_inner(self, segment_msg, feedback_cb,
                               execute_timeout, current_joints):
        seg_type = str(segment_msg.type)
        if seg_type == "pose_target":
            return self._run_pose_target(segment_msg, feedback_cb,
                                         execute_timeout, current_joints)
        if seg_type == "cartesian":
            fraction, plan = self._plan_cartesian(segment_msg, feedback_cb)
            if plan is None:
                return self._result(False, "compute_cartesian_path timeout", 0.0)
            occupancy_blocked = bool(getattr(plan, "occupancy_blocked", False))
            wrist_singular = bool(getattr(plan, "wrist_singular", False))
            required = max(
                self._min_fraction,
                float(getattr(segment_msg,
                              "required_cartesian_fraction", 0.0) or 0.0))
            need_fallback = (
                occupancy_blocked or wrist_singular
                or fraction < required)
            if need_fallback:
                if occupancy_blocked:
                    why = "occupancy blocked"
                elif wrist_singular:
                    why = plan.reason
                else:
                    why = "cartesian %.3f < %.3f" % (
                        fraction, required)
                # A complete Cartesian requirement rejects partial execution;
                # it does not prohibit a separately planned complete pose goal.
                if segment_msg.allow_ompl_fallback:
                    self._notify(feedback_cb, "planning", segment_msg.name,
                                 fraction, "%s; OMPL fallback" % why)
                    fallback = self._run_pose_target(
                        segment_msg, feedback_cb, execute_timeout,
                        current_joints)
                    fallback.message = "%s (fallback from %s)" % (
                        fallback.message, why)
                    fallback.fraction = fraction
                    fallback.used_ompl_fallback = True
                    return fallback
                if occupancy_blocked:
                    return self._result(
                        False, "PLACE_PATH_INFEASIBLE occupancy blocked cartesian",
                        fraction)
                if wrist_singular:
                    return self._result(
                        False, "PLACE_PATH_WRIST_SINGULAR %s" % why, fraction)
                return self._result(
                    False,
                    "%s: cartesian fraction %.3f below %.3f and no OMPL fallback"
                    % (segment_msg.name, fraction, required),
                    fraction)
            if self._occupancy is not None:
                ok, sweep_msg = self._occupancy_ok_plan(plan, segment_msg)
                if not ok:
                    return self._result(
                        False, "PLACE_PATH_INFEASIBLE %s" % sweep_msg, fraction)
            return self._execute_trajectory(plan, segment_msg, fraction,
                                            feedback_cb, execute_timeout)
        return SegmentExecResult(
            False, "unknown segment type %r" % seg_type, 0.0)

    def probe_segment(self, segment_msg, start_joints=None):
        """IK + optional cartesian fraction, no execution (dry-run)."""
        ik = self._ik_joints(segment_msg.target_pose, start_joints)
        record = {
            "name": str(segment_msg.name),
            "type": str(segment_msg.type),
            "ik_ok": ik is not None,
            "ik_joints": ik,
            "fraction": None,
            "cartesian_ok": None,
        }
        if str(segment_msg.type) == "cartesian":
            fraction, plan = self._plan_cartesian(
                segment_msg, None, start_joints=start_joints)
            record["fraction"] = fraction if plan is not None else 0.0
            record["cartesian_ok"] = (
                plan is not None
                and fraction >= self._min_fraction)
            if self._cartesian_excursion is not None:
                record["cartesian_excursion"] = dict(self._cartesian_excursion)
        return record

    # ------------------------------------------------------------------
    # pose_target

    def _build_goal_constraints(self, segment_msg):
        target = segment_msg.target_pose
        constraints = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = self._frame
        pos.link_name = self._link
        box = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005]  # 5 mm position tolerance
        box.primitives = [primitive]
        center = PointMsg()
        center.x = target.position.x
        center.y = target.position.y
        center.z = target.position.z
        box.primitive_poses = [target]
        box.primitive_poses[0].orientation.x = 0.0
        box.primitive_poses[0].orientation.y = 0.0
        box.primitive_poses[0].orientation.z = 0.0
        box.primitive_poses[0].orientation.w = 1.0
        pos.constraint_region = box
        pos.weight = 1.0
        constraints.position_constraints = [pos]

        orient = OrientationConstraint()
        orient.header.frame_id = self._frame
        orient.link_name = self._link
        orient.orientation = target.orientation
        orient.absolute_x_axis_tolerance = 0.05
        orient.absolute_y_axis_tolerance = 0.05
        orient.absolute_z_axis_tolerance = 0.05
        orient.weight = 1.0
        constraints.orientation_constraints = [orient]

        if segment_msg.keep_tool_down:
            down = OrientationConstraint()
            down.header.frame_id = self._frame
            down.link_name = self._link
            down.orientation.x, down.orientation.y = TOOL_DOWN_QUAT[:2]
            down.orientation.z, down.orientation.w = TOOL_DOWN_QUAT[2:]
            down.absolute_x_axis_tolerance = self._tool_down_abs_tol
            down.absolute_y_axis_tolerance = self._tool_down_abs_tol
            down.absolute_z_axis_tolerance = 3.14  # yaw free
            down.weight = 1.0
            constraints.orientation_constraints.append(down)

        return constraints

    def _run_pose_target(self, segment_msg, feedback_cb, execute_timeout,
                         current_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        ik_joints = self._ik_joints(segment_msg.target_pose, current_joints)
        if ik_joints is not None and self._wrist_blocked(
                current_joints, ik_joints):
            return self._result(
                False, "PLACE_PATH_WRIST_SINGULAR %s (no pose-constraint fallback)"
                % wrist_singularity_reason(self._wrist_measurement))
        if ik_joints is not None:
            return self._run_joint_goal(
                ik_joints, feedback_cb, execute_timeout, "IK joint goal",
                seed_joints=current_joints, segment_msg=segment_msg)
        # Pose-constraint OMPL cannot sample a pose KDL already rejected
        # and can wrap a joint by ±2π.
        return self._result(
            False,
            "IK failed error_code=%s (no pose-constraint fallback)"
            % self._last_ik_code,
            error_code=int(self._last_ik_code or 0))

    def execute_joints(self, positions, feedback_cb=None, execute_timeout=45.0,
                       current_joints=None):
        """MoveIt joint-space plan/execute to ``positions`` (rad, JOINTS order)."""
        wrapped = list(positions)
        self._planned_traj = None
        self._start_joints = list(current_joints or [])
        if current_joints and len(current_joints) == len(JOINTS):
            wrapped = [
                _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, wrapped)]
        result = self._run_joint_goal(
            wrapped, feedback_cb, execute_timeout, "named joint goal")
        return self._stamp_boundary(result, "named_joint")

    def _run_joint_goal(self, positions, feedback_cb, execute_timeout, note,
                        seed_joints=None, enforce_wrist=True, segment_msg=None):
        """Plan/execute a joint goal, refusing a wrist-branch change.

        The seed-to-goal pair is checked before anything is sent: a joint
        goal on the far side of joint5 = k*pi cannot be reached without
        reconfiguring through the singularity, whatever path the planner
        picks. This is the pose_target counterpart of the cartesian gate;
        motion_occ/2026-09-21_* transit segments planned and executed a
        3.12-3.16 rad joint5 flip here with no check at all.
        """
        if enforce_wrist and self._wrist_blocked(seed_joints, positions):
            return self._result(
                False, "PLACE_PATH_WRIST_SINGULAR %s (%s)" % (
                    wrist_singularity_reason(self._wrist_measurement), note))
        self._notify(feedback_cb, "planning", note, 0.0)
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.num_planning_attempts = self._attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.planner_id = self._planner_id
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [self._joint_constraints(positions)]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        if not self._move_group.wait_for_server(timeout_sec=5.0):
            return self._result(False, "move_group action server unavailable")
        future = self._move_group.send_goal_async(
            goal, feedback_callback=self._movegroup_feedback(feedback_cb, note))
        handle = self._wait_future(future, 5.0 + self._planning_time)
        if handle is None or not handle.accepted:
            return self._result(False, "MoveGroup goal rejected/timeout")
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "MoveGroup execution timeout")
        result = getattr(wrapped, "result", wrapped)
        return self._finish_move_group(
            result, note, segment_msg, feedback_cb, execute_timeout, "",
            enforce_wrist=enforce_wrist)

    def _ik_joints(self, pose, current_joints):
        """IK seeded at the current arm, then wrap each joint nearest current.

        OMPL pose-constraint sampling otherwise returns a ±2π equivalent of
        joint1 (limits are ±6.28) and the controller times out 4 rad away.
        """
        self._last_ik_code = 0
        if not self._ik.wait_for_service(timeout_sec=2.0):
            self._last_ik_code = -1
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group
        request.ik_request.ik_link_name = self._link
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.is_diff = True
        if current_joints and len(current_joints) == len(JOINTS):
            request.ik_request.robot_state.joint_state.name = list(JOINTS)
            request.ik_request.robot_state.joint_state.position = [
                float(v) for v in current_joints]
        request.ik_request.timeout = DurationMsg(sec=1, nanosec=0)
        stamped = PoseStamped()
        stamped.header.frame_id = self._frame
        stamped.pose = pose
        request.ik_request.pose_stamped = stamped
        future = self._ik.call_async(request)
        response = self._wait_future(future, 3.0)
        if response is None:
            self._last_ik_code = -1
            return None
        self._last_ik_code = int(response.error_code.val)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position))
        if not all(name in by_name for name in JOINTS):
            self._last_ik_code = -2
            return None
        raw = [float(by_name[name]) for name in JOINTS]
        if not current_joints or len(current_joints) != len(JOINTS):
            return raw
        return [
            _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, raw)]

    @staticmethod
    def _joint_constraints(positions):
        constraints = Constraints()
        for name, value in zip(JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _finish_move_group(self, result, note, segment_msg, feedback_cb,
                           execute_timeout, unimplemented, enforce_wrist=True):
        code = int(result.error_code.val)
        traj = getattr(result, "planned_trajectory", None)
        if traj is not None:
            self._planned_traj = traj
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                error_code=code)
        # Backstop for every MoveGroup route: the pose-constraint fallback and
        # the cartesian OMPL fallback both reach here, and neither is bound by
        # the pre-send seed check.
        joint_traj = getattr(traj, "joint_trajectory", None)
        if enforce_wrist and joint_traj is not None:
            measurement = self._wrist_check(
                list(getattr(joint_traj, "points", None) or []),
                getattr(joint_traj, "joint_names", None))
            if measurement is not None and not measurement["ok"]:
                return self._result(
                    False, "PLACE_PATH_WRIST_SINGULAR %s (%s)" % (
                        wrist_singularity_reason(measurement), note),
                    fraction=1.0, error_code=code)
        if self._occupancy is not None:
            ok, sweep_msg = self._occupancy_ok_robot_traj(traj)
            if not ok:
                return self._result(
                    False, "PLACE_PATH_INFEASIBLE %s" % sweep_msg,
                    fraction=1.0, error_code=code)
        self._hold_pick_joint6(traj, segment_msg)
        exec_result = self._execute_robot_trajectory(
            traj, note, 1.0, feedback_cb, execute_timeout)
        if exec_result.success:
            exec_result.message = "pose_target ok (%s; rest-to-rest)" % note
            if unimplemented:
                exec_result.message += "; " + unimplemented
        return exec_result

    def _occupancy_ok_points(self, points):
        if self._occupancy is None:
            return True, ""
        if not self._payload_wdh:
            return False, "payload size missing"
        mapped = []
        convert = self._world_to_map
        for point in points:
            xyz = (float(point[0]), float(point[1]), float(point[2]))
            if convert is not None:
                xyz = tuple(float(v) for v in convert(xyz))
            mapped.append(xyz)
        from luggage_planning.occupancy_place_paths import sweep_polyline
        sweep = sweep_polyline(
            self._occupancy, mapped, self._payload_wdh,
            arm_radius=self._arm_radius,
            exempt=self._occupancy_exempt)
        if sweep.collides:
            return False, sweep.reason or "occupancy_collision"
        return True, ""

    def _occupancy_ok_plan(self, plan_response, segment_msg):
        traj = getattr(getattr(plan_response, "solution", None),
                       "joint_trajectory", None)
        if traj is not None and traj.points:
            return self._occupancy_ok_robot_traj(plan_response.solution)
        points = []
        for wp in list(getattr(segment_msg, "waypoints", None) or []):
            points.append((wp.position.x, wp.position.y, wp.position.z))
        tgt = segment_msg.target_pose
        points.append((tgt.position.x, tgt.position.y, tgt.position.z))
        return self._occupancy_ok_points(points)

    def _occupancy_ok_robot_traj(self, robot_traj):
        if robot_traj is None:
            return False, "empty planned trajectory"
        points = self._fk_polyline(robot_traj)
        if not points:
            return False, "FK polyline empty"
        return self._occupancy_ok_points(points)

    def _fk_polyline(self, robot_traj):
        joint_traj = getattr(robot_traj, "joint_trajectory", robot_traj)
        names = list(getattr(joint_traj, "joint_names", None) or JOINTS)
        pts = list(getattr(joint_traj, "points", None) or [])
        if not pts:
            return []
        step = max(1, len(pts) // 40)
        polyline = []
        if not self._fk.wait_for_service(timeout_sec=2.0):
            return []
        from moveit_msgs.msg import RobotState
        for point in pts[::step] + ([pts[-1]] if pts else []):
            request = GetPositionFK.Request()
            request.header.frame_id = self._frame
            request.fk_link_names = [self._link]
            state = RobotState()
            state.joint_state.name = list(names)
            state.joint_state.position = [float(v) for v in point.positions]
            request.robot_state = state
            future = self._fk.call_async(request)
            response = self._wait_future(future, 2.0)
            if response is None or int(response.error_code.val) != int(
                    MoveItErrorCodes.SUCCESS):
                continue
            if not response.pose_stamped:
                continue
            pos = response.pose_stamped[0].pose.position
            polyline.append((pos.x, pos.y, pos.z))
        return polyline

    def _fk_xyz(self, joint_names, positions):
        """End-effector position for one joint vector, or None."""
        if not self._fk.wait_for_service(timeout_sec=2.0):
            return None
        from moveit_msgs.msg import RobotState
        request = GetPositionFK.Request()
        request.header.frame_id = self._frame
        request.fk_link_names = [self._link]
        state = RobotState()
        state.joint_state.name = list(joint_names or JOINTS)
        state.joint_state.position = [float(v) for v in positions]
        request.robot_state = state
        response = self._wait_future(self._fk.call_async(request), 2.0)
        if response is None or int(response.error_code.val) != int(
                MoveItErrorCodes.SUCCESS):
            return None
        if not response.pose_stamped:
            return None
        pos = response.pose_stamped[0].pose.position
        return (pos.x, pos.y, pos.z)

    def _requested_cartesian_length(self, waypoints, solution):
        """Straight-line length of the requested waypoint chain, or None.

        ``compute_cartesian_path`` interpolates linearly between waypoints, so
        the chain length is the Cartesian distance the solution covers. The
        start pose is not in the request; take it from the first point of the
        returned solution.
        """
        joint_traj = getattr(solution, "joint_trajectory", solution)
        points = list(getattr(joint_traj, "points", None) or [])
        if not points or not waypoints:
            return None
        start = self._fk_xyz(
            getattr(joint_traj, "joint_names", None) or JOINTS,
            points[0].positions)
        if start is None:
            return None
        chain = [start] + [
            (wp.position.x, wp.position.y, wp.position.z) for wp in waypoints]
        return sum(
            math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
            for a, b in zip(chain, chain[1:]))

    def _hold_pick_joint6(self, robot_traj, segment_msg):
        """Keep the suction-cup roll at the segment-start J6 during pick.

        J6 is coaxial with the panel. Pick segments pin every knot to the
        joints captured at ``execute_segment``; place segments are unchanged.
        """
        name = str(getattr(segment_msg, "name", "") or "")
        if robot_traj is None or not pick_holds_joint6(name):
            return
        held = joint6_value(self._start_joints)
        if held is None:
            self._node.get_logger().warning(
                "pick %s: J6 hold skipped, current joints missing" % name)
            return
        joint_traj = getattr(robot_traj, "joint_trajectory", robot_traj)
        pin_joint6_trajectory(joint_traj, held)
        self._node.get_logger().info(
            "pick %s: holding elfin_joint6 at %.4f rad" % (name, held))

    def _retime_robot_trajectory(self, robot_trajectory):
        """Drop MoveIt TOTG or linear times; write the site rest-to-rest profile."""
        joint_trajectory = getattr(robot_trajectory, "joint_trajectory", None)
        if joint_trajectory is None:
            return None
        timed = apply_rest_to_rest_to_joint_trajectory(
            joint_trajectory, self._vel_scale, self._acc_scale)
        if timed is not None:
            self._node.get_logger().info(
                "rest-to-rest duration=%.3fs path=%.4f rad "
                "v_peak=%.3f rad/s a_acc=%.3f a_decel=%.3f rad/s^2 "
                "(site accel 60 deg/s^2, decel 20 deg/s^2)"
                % (timed.duration, timed.path_length, timed.v_peak,
                   timed.a_max, timed.a_decel)
            )
        return timed

    def _guard_planned_trajectory(self, trajectory):
        maximum = getattr(self, "_max_segment_joint_excursion_rad", 0.0)
        if maximum <= 0:
            return True, "joint excursion guard not configured"
        from luggage_planning.hardware_motion_guard import planned_joint_excursion
        return planned_joint_excursion(trajectory, maximum)

    def _execute_robot_trajectory(self, robot_traj, name, fraction,
                                  feedback_cb, execute_timeout):
        if robot_traj is not None:
            self._retime_robot_trajectory(robot_traj)
            self._planned_traj = robot_traj
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj
        ok, message = self._guard_planned_trajectory(goal.trajectory)
        if not ok:
            return self._result(False, message, fraction)
        self._notify(feedback_cb, "executing", name, fraction)
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return self._result(
                False, "execute_trajectory server unavailable", fraction)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return self._result(
                False, "ExecuteTrajectory goal rejected", fraction)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "ExecuteTrajectory timeout", fraction)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(
                False, "ExecuteTrajectory error_code=%s" % code,
                fraction, error_code=code)
        return self._result(
            True, "ompl ok (%s)" % name, fraction, error_code=code)

    # ------------------------------------------------------------------
    # cartesian

    def _plan_cartesian(self, segment_msg, feedback_cb, start_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        waypoints = list(getattr(segment_msg, "waypoints", None) or [])
        waypoints.append(segment_msg.target_pose)
        if self._occupancy is not None:
            ee = [(wp.position.x, wp.position.y, wp.position.z)
                  for wp in waypoints]
            ok, _msg = self._occupancy_ok_points(ee)
            if not ok:
                return 0.0, _OccupancyBlockedPlan()
        request = GetCartesianPath.Request()
        request.header.frame_id = self._frame
        request.group_name = self._group
        request.link_name = self._link
        request.start_state.is_diff = True
        if start_joints is not None:
            request.start_state.joint_state.name = list(JOINTS)
            request.start_state.joint_state.position = [
                float(v) for v in start_joints]
        request.waypoints = waypoints
        request.max_step = self._max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = self._avoid_collisions
        future = self._cartesian.call_async(request)
        response = self._wait_future(future, 15.0)
        if response is None:
            return 0.0, None
        # Retry the final straight descent with finer interpolation, without
        # executing any prefix or accepting a shortened path. A geometric
        # collision will still fail; an IK discretization miss may recover.
        if segment_msg.name == "place_descend" and float(response.fraction) < 1.0:
            for scale in (.5, .25):
                request.max_step = self._max_step * scale
                self._notify(feedback_cb, "planning", segment_msg.name,
                             float(response.fraction),
                             "retry complete descent max_step=%.4f" % request.max_step)
                retry = self._wait_future(self._cartesian.call_async(request), 15.0)
                if retry is not None and float(retry.fraction) > float(response.fraction):
                    response = retry
                if float(response.fraction) >= 1.0:
                    break
        fraction = float(response.fraction)
        # Pick locks J6 before the excursion gate. A Cartesian solution
        # may spin the coaxial cup almost half a turn over a few
        # centimetres; that ratio is not the path that will be sent.
        self._hold_pick_joint6(
            getattr(response, "solution", None), segment_msg)
        self._record_cartesian_excursion(waypoints, response, fraction)
        rejected = self._reject_for_wrist(response)
        if rejected is not None:
            return fraction, rejected
        return fraction, response

    def _record_cartesian_excursion(self, waypoints, response, fraction):
        """Record the metric; no value of this ratio rejects a trajectory."""
        self._cartesian_excursion = None
        solution = getattr(response, "solution", None)
        if solution is None:
            return None
        path_len = self._requested_cartesian_length(waypoints, solution)
        if path_len is None:
            self._cartesian_excursion = {"skipped": "fk_unavailable"}
            return None
        joint_traj = getattr(solution, "joint_trajectory", solution)
        measurement = excursion_diagnostic(
            getattr(joint_traj, "joint_names", None) or JOINTS,
            list(getattr(joint_traj, "points", None) or []),
            path_len * max(0.0, min(1.0, fraction)),
            configured_limit=self._excursion_limit)
        if measurement is None:
            return None
        self._cartesian_excursion = measurement
        if measurement["ok"]:
            return None
        reason = (
            "cartesian joint excursion %.3f rad/m > %.3f (%s, %.3f rad over "
            "%.3f m)" % (
                measurement["rad_per_m"], measurement["limit_rad_per_m"],
                measurement["joint"], measurement["excursion_rad"],
                measurement["path_len_m"]))
        self._node.get_logger().warning("DIAGNOSTIC ONLY: " + reason)

    def _reject_for_wrist(self, response):
        """``_WristSingularPlan`` when the solution reconfigures the wrist.

        Records the measurement either way so a boundary dump shows the
        closest approach of an accepted path, not only of a refused one.
        """
        self._wrist_measurement = None
        solution = getattr(response, "solution", None)
        if solution is None:
            return None
        joint_traj = getattr(solution, "joint_trajectory", solution)
        measurement = self._wrist_check(
            list(getattr(joint_traj, "points", None) or []),
            getattr(joint_traj, "joint_names", None))
        if measurement is None or measurement["ok"]:
            return None
        return _WristSingularPlan(
            wrist_singularity_reason(measurement), measurement)

    def _wrist_check(self, points, joint_names=None):
        """``wrist_singularity`` over ``points``, resolving the joint index.

        A trajectory that does not carry joint5 (a partial-group solution)
        is not judged rather than judged against the wrong column.
        """
        names = list(joint_names or [])
        index = _WRIST_JOINT_INDEX
        if names:
            wrist = JOINTS[_WRIST_JOINT_INDEX]
            if wrist not in names:
                return None
            index = names.index(wrist)
        measurement = wrist_singularity(
            points, margin_rad=self._wrist_margin, joint_index=index)
        if measurement is not None:
            self._wrist_measurement = measurement
        return measurement

    def _wrist_blocked(self, seed_joints, goal_joints):
        """True when seed -> goal cannot avoid a wrist reconfiguration."""
        if not seed_joints or len(seed_joints) != len(goal_joints):
            return False
        measurement = self._wrist_check(
            [list(seed_joints), list(goal_joints)], JOINTS)
        return measurement is not None and not measurement["ok"]

    def _retime_cartesian_solution(self, plan_response):
        """Time a GetCartesianPath solution with the site rest-to-rest cap.

        GetCartesianPath returns geometry without a Huayan-safe time law.
        MoveIt TOTG (joint_limits 3.14 rad/s^2 scaled) trips CollisionStop.
        """
        self._retime_robot_trajectory(plan_response.solution)
        return plan_response

    def _execute_trajectory(self, plan_response, segment_msg, fraction,
                            feedback_cb, execute_timeout):
        self._hold_pick_joint6(
            getattr(plan_response, "solution", None), segment_msg)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._retime_cartesian_solution(
            plan_response).solution
        ok, message = self._guard_planned_trajectory(goal.trajectory)
        if not ok:
            return SegmentExecResult(False, message, fraction)
        self._notify(feedback_cb, "executing", segment_msg.name, fraction)
        self._planned_traj = goal.trajectory
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return SegmentExecResult(
                False, "execute_trajectory server unavailable", fraction)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return SegmentExecResult(
                False, "ExecuteTrajectory goal rejected", fraction)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "ExecuteTrajectory timeout", fraction)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "ExecuteTrajectory error_code=%s" % code,
                fraction, moveit_error_code=code)
        message = "cartesian ok (fraction %.3f)" % fraction
        unimplemented = self._unimplemented_flags_note(segment_msg)
        if unimplemented:
            message += "; " + unimplemented
        return SegmentExecResult(
            True, message, fraction, moveit_error_code=code)

    # ------------------------------------------------------------------
    # helpers

    @staticmethod
    def _unimplemented_flags_note(segment_msg):
        missing = []
        if segment_msg.keep_camera_down:
            missing.append("keep_camera_down")
        if segment_msg.lock_wrist:
            missing.append("lock_wrist")
        if not missing:
            return ""
        return "NOT_IMPLEMENTED: %s" % ", ".join(missing)

    def _notify(self, feedback_cb, stage, segment_name, fraction, note=""):
        if feedback_cb is None:
            return
        try:
            feedback_cb(stage, segment_name, fraction, note)
        except Exception:  # noqa: BLE001 - feedback must never break execution
            pass

    def _movegroup_feedback(self, feedback_cb, segment_name):
        # MoveGroup feedback carries only a state string (MONITORING_PLANNING,
        # PLANNING, ...), no numeric fraction; map planning states to the
        # "planning" stage and everything else to "executing".
        def _cb(_feedback_msg):
            # rclpy hands the callback a wrapper whose .feedback is the
            # MoveGroup.Feedback carrying the state string.
            raw = getattr(_feedback_msg, "feedback", _feedback_msg)
            state = str(getattr(raw, "state", ""))
            stage = "planning" if "PLAN" in state.upper() else "executing"
            self._notify(feedback_cb, stage, segment_name, 0.0)
        return _cb

    def _wait_future(self, future, timeout_sec):
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_sec):
            return None
        return future.result()

    def _wait_goal(self, handle, timeout_sec):
        """Wait for an accepted goal. Timeout is sim seconds when use_sim_time."""
        event = threading.Event()
        future = handle.get_result_async()
        future.add_done_callback(lambda _f: event.set())
        watcher = None
        if self._cancel_check is not None:
            def _watch():
                while not event.wait(0.2):
                    try:
                        if self._cancel_check():
                            handle.cancel_goal()
                            return
                    except Exception:  # noqa: BLE001 - watcher must not leak
                        return
            watcher = threading.Thread(target=_watch, daemon=True)
            watcher.start()
        reached, _reason = wait_event(
            event, timeout_sec, clock=self._node.get_clock())
        if not reached:
            if watcher is not None and watcher.is_alive():
                try:
                    handle.cancel_goal()
                except Exception:  # noqa: BLE001 - best-effort cancel
                    pass
            return None
        return future.result()
