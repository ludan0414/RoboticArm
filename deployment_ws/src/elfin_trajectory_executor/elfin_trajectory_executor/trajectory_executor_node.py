"""
实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

trajectory_executor_node.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 trajectory execution node for the Elfin S20 (Humble or Jazzy).

FollowJointTrajectory is the sequencing contract: the client may start the
next step only after the action result is SUCCEEDED. A JSON events topic
mirrors that terminal state for observers that are not holding the handle.

Do not start Gazebo or a zero-joint publisher alongside this node: both
would fight /joint_states.
"""

from __future__ import annotations

import json
import math
from typing import List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .execution_contract import (
    DEFAULT_ACTION_NAME,
    EVENT_ABORTED,
    EVENT_ACCEPTED,
    EVENT_CANCELED,
    EVENT_EXECUTING,
    EVENT_IDLE,
    EVENT_REJECTED,
    EVENT_SUCCEEDED,
    EVENTS_TOPIC,
    STATUS_TOPIC,
    event_to_json,
    make_event,
)
from .hardware_status import HARDWARE_STATUS_TOPIC
from .goal_ownership import SingleGoalOwner
from .huayan_interface import HuayanInterface, RESULT_ERROR
from .servo_j import fjt_to_servo_j_deg
from .sim_interface import (
    RESULT_INVALID_GOAL,
    RESULT_PREEMPTED,
    RESULT_SUCCESSFUL,
    SimInterface,
)
from .vacuum_io import VacuumIoPublisher, apply_vacuum_io_params, declare_vacuum_io_params

_LATCHED = QoSProfile(
    depth=16,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class TrajectoryExecutorNode(Node):
    """
    ROS2 node that exposes a FollowJointTrajectory action server for the
    Elfin S20 robot arm.
    """

    JOINT_NAMES: List[str] = [
        'elfin_joint1', 'elfin_joint2', 'elfin_joint3',
        'elfin_joint4', 'elfin_joint5', 'elfin_joint6',
    ]

    def __init__(self) -> None:
        super().__init__('trajectory_executor')

        # ----------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('robot_ip', '192.168.0.10')
        self.declare_parameter('robot_port', 10003)
        self.declare_parameter('default_velocity_deg', 30.0)
        self.declare_parameter('max_velocity_deg', 60.0)
        self.declare_parameter('command_acceleration_deg', 60.0)
        self.declare_parameter('controller_limit_fraction', 0.8)
        self.declare_parameter('execution_backend', 'waypoint')
        self.declare_parameter('servo_j_servo_time', 0.02)
        self.declare_parameter('servo_j_lookahead_time', 0.1)
        self.declare_parameter('power_off_on_disconnect', False)
        self.declare_parameter('joint_names', self.JOINT_NAMES)
        self.declare_parameter('action_name', DEFAULT_ACTION_NAME)
        declare_vacuum_io_params(self)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        default_vel = self.get_parameter('default_velocity_deg').get_parameter_value().double_value
        max_vel = self.get_parameter('max_velocity_deg').get_parameter_value().double_value
        command_accel = self.get_parameter(
            'command_acceleration_deg').get_parameter_value().double_value
        controller_limit_fraction = self.get_parameter(
            'controller_limit_fraction').get_parameter_value().double_value
        power_off = self.get_parameter(
            'power_off_on_disconnect').get_parameter_value().bool_value
        self._joint_names: List[str] = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
            or self.JOINT_NAMES
        )
        self._action_name = (
            self.get_parameter('action_name').get_parameter_value().string_value
            or DEFAULT_ACTION_NAME
        )
        self._execution_backend = (
            self.get_parameter(
                'execution_backend').get_parameter_value().string_value
            or 'waypoint'
        )
        self._servo_j_servo_time = self.get_parameter(
            'servo_j_servo_time').get_parameter_value().double_value
        self._servo_j_lookahead_time = self.get_parameter(
            'servo_j_lookahead_time').get_parameter_value().double_value
        if self._execution_backend == 'servo_esj':
            self.get_logger().error(
                "[executor] execution_backend=servo_esj rejected on this S20; "
                "using waypoint"
            )
            self._execution_backend = 'waypoint'
        if self._execution_backend not in ('waypoint', 'servo_j'):
            raise ValueError(
                "execution_backend must be 'waypoint' or 'servo_j'")

        self.get_logger().info(
            f'[executor] Starting in {mode.upper()} mode '
            f'(robot_ip={robot_ip}:{robot_port} action={self._action_name} '
            f'backend={self._execution_backend})'
        )

        # ----------------------------------------------------------------
        # Backend
        # ----------------------------------------------------------------
        self._vacuum_pub = None
        if mode == 'real':
            self._iface = HuayanInterface(
                node=self,
                robot_ip=robot_ip,
                robot_port=robot_port,
                default_velocity_deg=default_vel,
                max_velocity_deg=max_vel,
                command_acceleration_deg=command_accel,
                controller_limit_fraction=controller_limit_fraction,
                power_off_on_disconnect=power_off,
                execution_backend=self._execution_backend,
            )
            apply_vacuum_io_params(self, self._iface)
            if not self._iface.connect():
                self.get_logger().error(
                    '[executor] Failed to connect to robot. '
                    'Will retry on first goal.'
                )
        else:
            self._iface = SimInterface(node=self)

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------
        # Reentrant callback group so action + timer can run concurrently.
        self._cb_group = ReentrantCallbackGroup()
        # Reentrant callbacks share the server, but never hardware ownership.
        self._goal_owner = SingleGoalOwner()

        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, _LATCHED)
        self._events_pub = self.create_publisher(String, EVENTS_TOPIC, _LATCHED)
        self._hw_pub = self.create_publisher(String, HARDWARE_STATUS_TOPIC, _LATCHED)
        self._last_status = "idle"
        if mode == 'real':
            self._vacuum_pub = VacuumIoPublisher(self)

        # 100 Hz joint-state publisher timer.
        self._js_timer = self.create_timer(
            0.01, self._publish_joint_states,
            callback_group=self._cb_group,
        )
        self._hw_timer = self.create_timer(
            0.2, self._publish_hardware_status,
            callback_group=self._cb_group,
        )

        # ----------------------------------------------------------------
        # Action server
        # ----------------------------------------------------------------
        # Use a separate callback group so the action execute callback can
        # block without starving the joint-state timer.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            callback_group=self._cb_group,
        )
        if isinstance(self._iface, HuayanInterface):
            self.create_service(
                Trigger, "/elfin/recover_standby", self._handle_recover_standby,
                callback_group=self._cb_group)
            self.create_service(
                SetBool, "/elfin/vacuum/set_do0",
                lambda req, res: self._handle_set_do(0, req, res),
                callback_group=self._cb_group)
            self.create_service(
                SetBool, "/elfin/vacuum/set_do1",
                lambda req, res: self._handle_set_do(1, req, res),
                callback_group=self._cb_group)

        self._publish_status('idle')
        self._publish_event(EVENT_IDLE)
        self.get_logger().info('[executor] Ready.')

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject incoming goals."""
        traj = goal_request.trajectory

        # Validate joint names — reorder if planner uses a different order.
        error = self._validate_joint_names(traj.joint_names)
        if error:
            self.get_logger().warn(f'[executor] Goal rejected: {error}')
            self._publish_event(EVENT_REJECTED, error_string=error)
            return GoalResponse.REJECT

        # Reserve before checking/reconnecting hardware. EXECUTING is not READY;
        # without this gate an overlapping request could reconnect the live CPS
        # client while the owning callback is still moving the arm.
        if not self._goal_owner.try_reserve():
            error = 'executor busy'
            self.get_logger().warn(f'[executor] Goal rejected: {error}.')
            self._publish_event(EVENT_REJECTED, error_string=error)
            return GoalResponse.REJECT

        accepted = False
        try:
            if (
                isinstance(self._iface, HuayanInterface)
                and not self._iface.is_ready
            ):
                self.get_logger().warn(
                    '[executor] Robot not ready; retrying connect before reject')
                if not self._iface.connect():
                    self.get_logger().warn(
                        '[executor] Goal rejected: robot not ready.')
                    self._publish_event(
                        EVENT_REJECTED, error_string='robot not ready')
                    return GoalResponse.REJECT

            accepted = True
            self.get_logger().info(
                f'[executor] Goal accepted: {len(traj.points)} waypoints.'
            )
            return GoalResponse.ACCEPT
        except Exception as exc:
            error = f'goal admission error: {exc}'
            self.get_logger().error(f'[executor] Goal rejected: {error}')
            self._publish_event(EVENT_REJECTED, error_string=error)
            return GoalResponse.REJECT
        finally:
            if not accepted:
                self._goal_owner.release_pending()

    def _handle_accepted_callback(self, goal_handle) -> None:
        """Bind the admission reservation before scheduling execution."""
        goal_id = _uuid_hex(goal_handle.goal_id)
        if not self._goal_owner.bind(goal_id):
            # This is an internal invariant failure. Execute the handle so the
            # client receives an aborted result instead of waiting forever.
            self.get_logger().error(
                '[executor] Accepted goal has no ownership reservation: %s'
                % goal_id
            )
        goal_handle.execute()

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept cancellation only for the goal that owns the executor."""
        goal_id = _uuid_hex(goal_handle.goal_id)
        if not self._goal_owner.cancel(goal_id):
            self.get_logger().warn(
                '[executor] Cancel rejected for non-owning goal %s' % goal_id)
            return CancelResponse.REJECT
        self.get_logger().info(
            '[executor] Cancel request received for goal %s.' % goal_id)
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        """
        Main execution callback.  Runs in a separate thread (MultiThreadedExecutor).

        1. Reorder waypoints to match our joint name order.
        2. Delegate to the backend.
        3. Map return code → action result.
        """
        goal_id = _uuid_hex(goal_handle.goal_id)
        cancel_flag = self._goal_owner.cancel_event(goal_id)
        if cancel_flag is None:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Internal executor ownership error.'
            goal_handle.abort()
            self._publish_status('error')
            self._publish_event(
                EVENT_ABORTED,
                goal_id=goal_id,
                error_code=result.error_code,
                error_string=result.error_string,
            )
            self._goal_owner.release_pending()
            return result

        self._publish_status('executing')
        self._publish_event(EVENT_ACCEPTED, goal_id=goal_id)
        self._publish_event(EVENT_EXECUTING, goal_id=goal_id)

        traj = goal_handle.request.trajectory
        ordered_traj = self._reorder_trajectory(traj)

        def feedback_fn(positions: List[float]) -> None:
            """Called by the backend at ~100 Hz with current positions."""
            if goal_handle.is_active:
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = self._joint_names
                fb.actual.positions = list(positions)
                fb.actual.time_from_start = self._ros_time_offset()
                goal_handle.publish_feedback(fb)

        try:
            if (
                self._execution_backend == 'servo_j'
                and isinstance(self._iface, HuayanInterface)
            ):
                servo_path = self._servo_j_path_from_trajectory(
                    ordered_traj, self._servo_j_servo_time)
                if isinstance(servo_path, str):
                    self.get_logger().error(
                        '[executor] ServoJ trajectory rejected: %s'
                        % servo_path)
                    ret = RESULT_INVALID_GOAL
                else:
                    ret = self._iface.execute_servo_j_path(
                        servo_path,
                        feedback_fn,
                        cancel_flag,
                        servo_time=self._servo_j_servo_time,
                        lookahead_time=self._servo_j_lookahead_time,
                    )
            else:
                ret = self._iface.execute(ordered_traj, feedback_fn, cancel_flag)
        except Exception as exc:
            self.get_logger().error(
                '[executor] Backend execution exception: %s' % exc)
            ret = RESULT_ERROR

        result = FollowJointTrajectory.Result()
        try:
            if ret == RESULT_SUCCESSFUL:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ''
                goal_handle.succeed()
                self._publish_status('idle')
                self._publish_event(EVENT_SUCCEEDED, goal_id=goal_id)
                self.get_logger().info(
                    '[executor] Goal succeeded. ready_for_next=true'
                )

            elif ret == RESULT_PREEMPTED:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Preempted by cancel request.'
                goal_handle.canceled()
                self._publish_status('idle')
                self._publish_event(
                    EVENT_CANCELED, goal_id=goal_id,
                    error_string=result.error_string,
                )
                self.get_logger().info('[executor] Goal cancelled.')

            elif ret == RESULT_INVALID_GOAL:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = getattr(
                    self._iface, "last_error_string", ""
                ) or 'Trajectory validation failed (check joint limits).'
                goal_handle.abort()
                self._publish_status('error')
                self._publish_event(
                    EVENT_ABORTED, goal_id=goal_id,
                    error_code=result.error_code,
                    error_string=result.error_string,
                )
                self.get_logger().error('[executor] Goal aborted: invalid goal.')

            else:  # RESULT_ERROR
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = getattr(
                    self._iface, "last_error_string", ""
                ) or 'Hardware execution error. Check robot logs.'
                goal_handle.abort()
                self._publish_status('error')
                self._publish_event(
                    EVENT_ABORTED, goal_id=goal_id,
                    error_code=result.error_code,
                    error_string=result.error_string,
                )
                self.get_logger().error('[executor] Goal aborted: hardware error.')

            self._publish_hardware_status()
            return result
        finally:
            if not self._goal_owner.release(goal_id):
                self.get_logger().error(
                    '[executor] Failed to release goal ownership for %s' % goal_id)

    # ------------------------------------------------------------------
    # Joint state publisher
    # ------------------------------------------------------------------

    def _publish_joint_states(self) -> None:
        """Publish this tick's CPS actual joints. Skip if the ACS read failed."""
        if callable(getattr(self._iface, "refresh", None)):
            self._iface.refresh()
        if hasattr(self._iface, "last_acs_ok") and not self._iface.last_acs_ok:
            return
        positions = self._iface.current_positions
        velocities = list(getattr(self._iface, "current_velocities", []))
        effort = list(getattr(self._iface, "current_currents", []))

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = list(positions)
        if len(velocities) == 6:
            msg.velocity = list(velocities)
        if len(effort) == 6:
            msg.effort = list(effort)
        self._js_pub.publish(msg)
        if self._vacuum_pub is not None:
            self._vacuum_pub.publish(self._iface)

    def _handle_recover_standby(self, request, response):
        # Same reservation as trajectories: reject overlap, including admission.
        if not self._goal_owner.try_reserve_operation("controller_recovery"):
            response.success, response.message = False, "executor busy; cancel active goal first"
            return response
        try:
            self._publish_status("recovering")
            self._publish_event("controller_recovery_started")
            response.success, response.message = self._iface.recover_standby()
            self._publish_status("idle" if response.success else "error")
            self._publish_event("controller_recovered" if response.success
                                else "controller_recovery_failed",
                                error_string="" if response.success else response.message)
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self._publish_status("error")
        finally:
            self._goal_owner.release("controller_recovery")
        return response

    def _handle_set_do(self, which: int, request, response):
        if not self._goal_owner.try_reserve_operation("vacuum_io"):
            response.success, response.message = False, "executor busy"
            return response
        try:
            return self._apply_set_do(which, request, response)
        finally:
            self._goal_owner.release("vacuum_io")

    def _apply_set_do(self, which: int, request, response):
        bit = (
            self._iface.vacuum_do0_bit if which == 0
            else self._iface.vacuum_do1_bit
        )
        ok, message = self._iface.set_do_bit(bit, 1 if request.data else 0)
        response.success = bool(ok)
        response.message = message or ""
        if ok:
            self.get_logger().info(
                "[executor] set DO%s=%s" % (which, int(bool(request.data)))
            )
        else:
            self.get_logger().error(
                "[executor] set DO%s failed: %s" % (which, message)
            )
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _validate_joint_names(self, incoming: List[str]) -> Optional[str]:
        """
        Check that the incoming joint list is a permutation of our list.
        Returns error string or None.
        """
        if not incoming:
            return 'Trajectory has no joint_names.'
        missing = set(self._joint_names) - set(incoming)
        if missing:
            return f'Missing joints: {missing}'
        extra = set(incoming) - set(self._joint_names)
        if extra:
            return f'Unknown joints: {extra}'
        return None

    def _reorder_trajectory(self, traj: JointTrajectory) -> JointTrajectory:
        """
        Return a new JointTrajectory whose joint order matches self._joint_names.

        If the incoming order already matches, the original object is returned.
        """
        if list(traj.joint_names) == self._joint_names:
            return traj

        # Build an index map: our_idx → incoming_idx
        incoming_idx = {name: i for i, name in enumerate(traj.joint_names)}
        order = [incoming_idx[name] for name in self._joint_names]

        new_traj = JointTrajectory()
        new_traj.header = traj.header
        new_traj.joint_names = self._joint_names

        for pt in traj.points:
            new_pt = JointTrajectoryPoint()
            new_pt.time_from_start = pt.time_from_start
            new_pt.positions = [pt.positions[i] for i in order]
            if pt.velocities:
                new_pt.velocities = [pt.velocities[i] for i in order]
            if pt.accelerations:
                new_pt.accelerations = [pt.accelerations[i] for i in order]
            new_traj.points.append(new_pt)

        return new_traj

    def _servo_j_path_from_trajectory(
        self, traj: JointTrajectory, servo_time: float,
    ):
        """Resample a MoveIt FJT onto the PushServoJ time grid (joint deg)."""
        servo_time = float(servo_time)
        if not math.isfinite(servo_time) or servo_time <= 0.0:
            return "servo_j_servo_time must be positive"
        if not traj.points:
            return []
        positions_rad = []
        times_s = []
        velocities_rad = []
        have_vel = True
        previous_t = None
        n_joints = len(self._joint_names)
        for index, pt in enumerate(traj.points):
            if len(pt.positions) != n_joints:
                return "point %d has %d joints; expected %d" % (
                    index, len(pt.positions), n_joints)
            t = (
                float(pt.time_from_start.sec)
                + float(pt.time_from_start.nanosec) * 1e-9
            )
            if index == 0:
                if t < -1e-9:
                    return "first point time is negative"
            elif previous_t is not None and t + 1e-9 < float(previous_t):
                return "point %d time goes backwards" % index
            previous_t = t
            positions_rad.append(list(pt.positions))
            times_s.append(t)
            if pt.velocities and len(pt.velocities) == n_joints:
                velocities_rad.append(list(pt.velocities))
            else:
                have_vel = False
        path, reason = fjt_to_servo_j_deg(
            positions_rad, times_s, dt_s=servo_time,
            velocities_rad=velocities_rad if have_vel else None)
        if reason:
            return reason
        if path is None:
            return "ServoJ resample failed"
        dt0 = None
        if len(times_s) >= 2:
            dt0 = times_s[1] - times_s[0]
        self.get_logger().info(
            "[executor] ServoJ resampled %d FJT knots%s -> %d points at %.3fs"
            % (
                len(times_s),
                "" if dt0 is None else " (dt=%.3f)" % dt0,
                len(path),
                servo_time,
            )
        )
        return path

    def _publish_status(self, status: str) -> None:
        self._last_status = str(status)
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_hardware_status(self) -> None:
        getter = getattr(self._iface, "hardware_status", None)
        if not callable(getter):
            return
        payload = getter(refresh_fsm=True)
        payload["executor"] = getattr(self, "_last_status", "")
        payload["stamp_ns"] = int(self.get_clock().now().nanoseconds)
        msg = String()
        msg.data = json.dumps(payload, default=str)
        self._hw_pub.publish(msg)

    def _publish_event(
        self,
        event: str,
        *,
        goal_id: str = "",
        error_code: int = 0,
        error_string: str = "",
    ) -> None:
        payload = make_event(
            event,
            goal_id=goal_id,
            error_code=error_code,
            error_string=error_string,
            stamp_ns=int(self.get_clock().now().nanoseconds),
        )
        msg = String()
        msg.data = event_to_json(payload)
        self._events_pub.publish(msg)

    def _ros_time_offset(self) -> Duration:
        """Return current ROS time as a Duration (used for feedback stamp)."""
        t = self.get_clock().now()
        secs = int(t.nanoseconds // 1_000_000_000)
        nsecs = int(t.nanoseconds % 1_000_000_000)
        d = Duration()
        d.sec = secs
        d.nanosec = nsecs
        return d

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        if isinstance(self._iface, HuayanInterface):
            self._iface.disconnect()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = TrajectoryExecutorNode()

    # MultiThreadedExecutor allows the action execute callback to block while
    # the joint-state timer and other callbacks keep running.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _uuid_hex(goal_id) -> str:
    try:
        return bytes(goal_id.uuid).hex()
    except Exception:
        return ""


if __name__ == '__main__':
    main()
