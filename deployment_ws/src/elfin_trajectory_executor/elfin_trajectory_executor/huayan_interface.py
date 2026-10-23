"""
实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

huayan_interface.py
~~~~~~~~~~~~~~~~~~~
Real-hardware backend for the Elfin S20 trajectory executor.

Wraps the HuayanRobot CPSClient Python SDK (CPS.so / CPS.pyd, V1.0.11.0).
Handles the full connection lifecycle, trajectory execution, and error
recovery so the action server can treat hardware interaction as a simple
blocking call.

Connection lifecycle
--------------------
connect()
    TCP + FSM-aware enable. StandBy (FSM 33) treats Connect2Box /
    Electrify / Connect2Controller 20018 as already-done. Does not
    GrpReset a standing arm.

connect(monitor_only=True)
    HRIF_Connect → HRIF_Connect2Box only. Used by cps_telemetry for bags.
    Does not electrify or enable the servo.

execute(trajectory, feedback_fn, cancel_flag)
    For each waypoint:
        HRIF_WayPoint (joint move, blending on intermediates)
        poll HRIF_IsBlendingDone / HRIF_IsMotionDone
        call feedback_fn with live joint positions

disconnect()
    HRIF_GrpDisable → HRIF_DisConnect. BlackOut only if
    power_off_on_disconnect=True (Ctrl+C must not cut 48 V).

Error / reconnect strategy
--------------------------
* HRIF_ calls that return nRet > 0 are treated as hard errors: execution
  stops, HRIF_GrpStop is called, and RESULT_ERROR is returned.
* If IsConnected returns False mid-execution a reconnect is attempted up
  to MAX_RECONNECT_RETRIES times with exponential backoff.
* Incoming goals are rejected while the connection is not READY.
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple

from .cps_parse import (
    acs_from_read_act_pos,
    as_bit,
    as_float_list,
    choose_joint_vel_deg,
    finite_diff_deg_s,
    joints_moved_deg,
    tcp_from_read_act_pos,
)
from .hardware_status import (
    append_event,
    build_snapshot,
    make_hrif_event,
    should_keep_event,
)
from .servo_j import (
    SERVO_MAX_ACCEL_DEG,
    SERVO_MAX_DECEL_DEG,
    densify_servo_j_path,
    limit_servo_braking,
    servo_j_hold_count,
)

# ---------------------------------------------------------------------------
# Result codes (mirrors control_msgs/action/FollowJointTrajectory constants)
# ---------------------------------------------------------------------------
RESULT_SUCCESSFUL = 0
RESULT_PREEMPTED = -5
RESULT_INVALID_GOAL = -2
RESULT_ERROR = -6

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
BOX_ID = 0
RBT_ID = 0
TCP_NAME = 'TCP'
UCS_NAME = 'Base'

MAX_RECONNECT_RETRIES = 3
CONTROLLER_START_TIMEOUT_S = 30.0
POLL_INTERVAL_S = 0.05          # 20 Hz feedback polling
BLEND_RADIUS_MM = 5.0           # blending on intermediate waypoints
FINAL_BLEND_RADIUS_MM = 0.0     # exact stop at last waypoint
DEFAULT_VELOCITY_DEG = 30.0
DEFAULT_ACCEL_DEG = 60.0        # accepted on site; must exceed velocity
MAX_VELOCITY_DEG = 60.0
DEFAULT_CONTROLLER_LIMIT_FRACTION = 0.8
MAX_CONTROLLER_LIMIT_FRACTION = 0.8
HUAYAN_MIN_VELOCITY_DEG = 1.0   # CPS 20070 below ~1 deg/s
BACKEND_WAYPOINT = "waypoint"
BACKEND_SERVO_J = "servo_j"
BACKEND_SERVO_ESJ = "servo_esj"  # rejected on this S20 (Push 20006/20007)
PREFLIGHT_FAILED_LOG = "Trajectory profile preflight failed before motion"
# MoveIt TOTG densifies OMPL paths to sub-degree samples. Each sample is a
# full HRIF_WayPoint MoveJ; blasting those at default_vel looks like the
# arm is jumping back and forth. Keep only real via points.
WAYPOINT_MIN_DELTA_DEG = 2.0
BLEND_START_TIMEOUT_S = 0.4     # IsBlendingDone is true while idle
STATE_REFUSE = 20018  # command prohibited in current FSM
FSM_BOX_DISCONNECT = 2
FSM_BOX_CONNECTING = 3
FSM_ESTOP = 5
FSM_BLACKOUTING = 6
FSM_BLACKOUT = 7
FSM_ELECTRIFYING = 8
FSM_COLLISION_STOP = 21
FSM_ERROR = 22
FSM_ENABLING = 23
FSM_DISABLE = 24
FSM_STANDBY = 33
# This S20's PushServoJ returns 40071 until InitServoEsJ has cleared the
# EsJ buffer. StartServo leaves StandBy, so Init after Start can 40025.
# Retry Init without Stop; Stop only after those retries lose.
SERVO_ESJ_INIT_RETRIES = 2
SERVO_ESJ_INIT_RETRY_S = 0.02
SERVO_STANDBY_STABLE_S = 0.1


def cps_step_ok(n_ret: int, allow_refuse: bool) -> bool:
    if int(n_ret) == 0:
        return True
    return bool(allow_refuse) and int(n_ret) == STATE_REFUSE


def connect2box_allow_refuse(fsm_id: Optional[int]) -> bool:
    return fsm_id is None or int(fsm_id) not in (
        FSM_BOX_DISCONNECT, FSM_BOX_CONNECTING)


def electrify_allow_refuse(fsm_id: Optional[int]) -> bool:
    return fsm_id is not None and int(fsm_id) >= FSM_ENABLING


def already_motion_ready(fsm_id: Optional[int]) -> bool:
    """StandBy (33) or later: servos enabled. Skip GrpReset / GrpEnable."""
    return fsm_id is not None and int(fsm_id) >= FSM_STANDBY


def hrif_waypoint_joint(
    cps,
    joints_deg: List[float],
    vel_deg: float,
    accel_deg: float,
    radius: float,
    cmd_id: str,
    box_id: int = BOX_ID,
    rbt_id: int = RBT_ID,
    tcp: str = TCP_NAME,
    ucs: str = UCS_NAME,
):
    """Call Python ``CPSClient.HRIF_WayPoint`` for a joint-space MoveJ.

    The C API takes 12 separate doubles. The Python SDK takes two length-6
    lists: unused PCS, then ACS. Passing the C layout raises
    ``TypeError: takes 16 positional arguments but 26 were given``.
    """
    joints = [float(v) for v in joints_deg]
    if len(joints) != 6:
        raise ValueError("HRIF_WayPoint ACS needs 6 joints, got %s" % len(joints))
    return cps.HRIF_WayPoint(
        box_id,
        rbt_id,
        0,
        [0.0] * 6,
        joints,
        tcp,
        ucs,
        float(vel_deg),
        float(accel_deg),
        float(radius),
        1,
        0,
        0,
        0,
        str(cmd_id),
    )


def hrif_init_servo_esj(
    cps,
    box_id: int = BOX_ID,
    rbt_id: int = RBT_ID,
):
    """Clear the servo point buffer. This CPS rejects PushServoJ with 40071
    until InitServoEsJ has run. PushServoEsJ itself stays unused."""
    return cps.HRIF_InitServoEsJ(box_id, rbt_id)


def hrif_start_servo(
    cps,
    servo_time: float,
    lookahead_time: float,
    box_id: int = BOX_ID,
    rbt_id: int = RBT_ID,
):
    return cps.HRIF_StartServo(
        box_id, rbt_id, float(servo_time), float(lookahead_time))


def hrif_push_servo_j(
    cps,
    joints_deg: List[float],
    box_id: int = BOX_ID,
    rbt_id: int = RBT_ID,
):
    joints = [float(v) for v in joints_deg]
    if len(joints) != 6:
        raise ValueError("HRIF_PushServoJ ACS needs 6 joints, got %s" % len(joints))
    return cps.HRIF_PushServoJ(box_id, rbt_id, joints)


def _max_joint_delta_deg(a: List[float], b: List[float]) -> float:
    n = min(6, len(a), len(b))
    if n <= 0:
        return 0.0
    return max(abs(float(a[i]) - float(b[i])) for i in range(n))


def read_positive_joint_limits(cps, method_name: str) -> Optional[List[float]]:
    """Read one six-axis CPS limit vector, returning ``None`` if unavailable."""
    method = getattr(cps, method_name, None)
    if not callable(method):
        return None
    result = []
    try:
        n_ret = method(BOX_ID, RBT_ID, result)
    except Exception:
        return None
    values = as_float_list(result, 6) if n_ret == 0 else None
    if values is None or not all(math.isfinite(v) and v > 0.0 for v in values):
        return None
    return values


def _positive_joint_limit_minimum(
    values: Optional[List[float]], field_name: str,
) -> float:
    parsed = as_float_list(values, 6)
    if parsed is None or not all(
        math.isfinite(value) and value > 0.0 for value in parsed
    ):
        raise ValueError("controller %s limits are unavailable" % field_name)
    return min(parsed)


def safe_waypoint_profile(
    estimated_velocity_deg: float,
    command_acceleration_deg: float,
    configured_max_velocity_deg: float,
    controller_max_velocity_deg: Optional[List[float]],
    controller_max_acceleration_deg: Optional[List[float]],
    controller_limit_fraction: float = DEFAULT_CONTROLLER_LIMIT_FRACTION,
) -> Tuple[float, float]:
    """Clamp one scalar MoveJ profile to configured and reported CPS limits.

    Huayan rejects a waypoint unless acceleration is greater than velocity.
    This helper is deliberately pure so the complete command list can be
    validated before the first motion command is sent.
    """
    requested = [
        float(estimated_velocity_deg),
        float(command_acceleration_deg),
        float(configured_max_velocity_deg),
        float(controller_limit_fraction),
    ]
    if not all(math.isfinite(v) and v > 0.0 for v in requested):
        raise ValueError(
            "velocity, acceleration, and limit fraction must be finite and "
            "positive"
        )
    limit_fraction = requested[3]
    if limit_fraction > MAX_CONTROLLER_LIMIT_FRACTION:
        raise ValueError(
            "controller limit fraction %.3f exceeds safety maximum %.3f"
            % (limit_fraction, MAX_CONTROLLER_LIMIT_FRACTION)
        )
    controller_velocity_min = _positive_joint_limit_minimum(
        controller_max_velocity_deg, "velocity")
    controller_acceleration_min = _positive_joint_limit_minimum(
        controller_max_acceleration_deg, "acceleration")

    velocity_cap = min(
        requested[2], limit_fraction * controller_velocity_min)

    acceleration = min(
        requested[1], limit_fraction * controller_acceleration_min)

    if acceleration <= HUAYAN_MIN_VELOCITY_DEG:
        raise ValueError(
            "controller acceleration limit %.3f leaves no valid Huayan velocity"
            % acceleration
        )
    velocity = min(requested[0], velocity_cap, acceleration - 1.0)
    if velocity < HUAYAN_MIN_VELOCITY_DEG:
        raise ValueError(
            "controller velocity limit %.3f is below Huayan minimum %.3f"
            % (velocity, HUAYAN_MIN_VELOCITY_DEG)
        )
    return velocity, acceleration


def decimate_joint_waypoints(
    joints_deg: List[List[float]],
    *,
    min_delta_deg: float = WAYPOINT_MIN_DELTA_DEG,
    start_deg: Optional[List[float]] = None,
) -> List[int]:
    """Keep last pose plus via points that actually move the arm.

    TOTG samples closer than ``min_delta_deg`` to the last kept pose (or to
    ``start_deg``) are dropped so Huayan does not MoveJ-chatter.
    """
    n = len(joints_deg)
    if n == 0:
        return []
    kept: List[int] = []
    ref = list(start_deg) if start_deg is not None and len(start_deg) >= 6 else None
    for i in range(n - 1):
        q = joints_deg[i]
        if ref is None:
            kept.append(i)
            ref = list(q)
            continue
        if _max_joint_delta_deg(q, ref) >= min_delta_deg:
            kept.append(i)
            ref = list(q)
    last_q = joints_deg[-1]
    if not kept:
        kept.append(n - 1)
    elif _max_joint_delta_deg(last_q, joints_deg[kept[-1]]) > 1e-6:
        kept.append(n - 1)
    else:
        kept[-1] = n - 1
    return kept


class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    READY = auto()
    EXECUTING = auto()
    ERROR = auto()


class HuayanInterface:
    """
    Real-hardware backend for the Elfin S20 via HuayanRobot CPSClient SDK.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent node for logging.
    robot_ip : str
        Controller IP address (default '192.168.0.10').
    robot_port : int
        Controller port (default 10003).
    default_velocity_deg : float
        Fallback joint velocity in °/s when trajectory time hints are absent.
    max_velocity_deg : float
        Upper clamp on computed velocity.
    command_acceleration_deg : float
        Conservative scalar MoveJ acceleration. The site-proven default is
        60 deg/s^2; reported controller limits may reduce it further.
    controller_limit_fraction : float
        Fraction of reported controller velocity/acceleration limits available
        to generated commands. Values above 0.8 are rejected.
    power_off_on_disconnect : bool
        If True, HRIF_BlackOut on teardown (cuts 48 V). Default False.
    """

    JOINT_NAMES: List[str] = [
        'elfin_joint1', 'elfin_joint2', 'elfin_joint3',
        'elfin_joint4', 'elfin_joint5', 'elfin_joint6',
    ]

    # Elfin S20 joint limits in degrees (matches URDF ±360°)
    JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
    ]

    def __init__(
        self,
        node,
        robot_ip: str = '192.168.0.10',
        robot_port: int = 10003,
        default_velocity_deg: float = DEFAULT_VELOCITY_DEG,
        max_velocity_deg: float = MAX_VELOCITY_DEG,
        command_acceleration_deg: float = DEFAULT_ACCEL_DEG,
        controller_limit_fraction: float = DEFAULT_CONTROLLER_LIMIT_FRACTION,
        power_off_on_disconnect: bool = False,
        execution_backend: str = BACKEND_WAYPOINT,
        poll_interval_s: float = POLL_INTERVAL_S,
        blend_start_timeout_s: float = BLEND_START_TIMEOUT_S,
        stop_settle_s: float = 0.1,
    ) -> None:
        self._node = node
        self._ip = robot_ip
        self._port = robot_port
        self._default_vel = default_velocity_deg
        self._max_vel = max_velocity_deg
        self._command_accel = command_acceleration_deg
        self._controller_limit_fraction = float(controller_limit_fraction)
        if (
            not math.isfinite(self._controller_limit_fraction)
            or self._controller_limit_fraction <= 0.0
            or self._controller_limit_fraction > MAX_CONTROLLER_LIMIT_FRACTION
        ):
            raise ValueError(
                "controller_limit_fraction must be in (0, %.3f]"
                % MAX_CONTROLLER_LIMIT_FRACTION
            )
        self._power_off_on_disconnect = bool(power_off_on_disconnect)
        backend = str(execution_backend or BACKEND_WAYPOINT).strip().lower()
        if backend == BACKEND_SERVO_ESJ:
            # PushServoEsJ: 6n → 20007; 7th=0 and 7th=0.02 → 20006.
            node.get_logger().error(
                "[huayan] execution_backend=%s rejected; this cell is waypoint-only"
                % backend
            )
            backend = BACKEND_WAYPOINT
        if backend not in (BACKEND_WAYPOINT, BACKEND_SERVO_J):
            raise ValueError(
                "execution_backend must be 'waypoint' or 'servo_j'"
            )
        self._execution_backend = backend
        self._poll_interval_s = float(poll_interval_s)
        self._blend_start_timeout_s = float(blend_start_timeout_s)
        self._stop_settle_s = float(stop_settle_s)
        self.last_error_string = ""
        self.last_preflight_reason = ""
        self.command_log: List[dict] = []

        self._cps = None          # CPSClient instance (imported lazily)
        self._state = ConnectionState.DISCONNECTED
        self._state_lock = threading.Lock()
        self._monitor_only = False
        self._current_positions_deg: List[float] = [0.0] * 6
        self._current_velocities_deg: List[float] = [0.0] * 6
        self._current_accelerations_deg: List[float] = [0.0] * 6
        self._current_currents_a: List[float] = []
        self._command_positions_deg: List[float] = []
        self._cps_velocities_deg: Optional[List[float]] = None
        self._current_tcp_mm_deg: List[float] = [0.0] * 6
        self._vel_source: str = "none"
        self._acc_source: str = "none"
        self._last_refresh_mono: float = 0.0
        self.last_refresh_dt_s: float = 0.0
        self.last_sample_mono: float = 0.0
        self.last_acs_ok: bool = False
        self.last_tcp_ok: bool = False
        self.last_vel_cps_ok: bool = False
        self.last_cur_ok: bool = False
        self.last_cmd_ok: bool = False
        self.last_io_ok: bool = False
        self.vacuum_io_kind: str = "box"
        self.vacuum_di_bit: int = 0
        self.vacuum_do0_bit: int = 0
        self.vacuum_do1_bit: int = 1
        self._vacuum_di0: Optional[int] = None
        self._vacuum_do0: Optional[int] = None
        self._vacuum_do1: Optional[int] = None
        self._refresh_n: int = 0
        self._controller_max_velocity_deg: Optional[List[float]] = None
        self._controller_max_acceleration_deg: Optional[List[float]] = None
        self._controller_max_jerk_deg: Optional[List[float]] = None

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    @property
    def is_ready(self) -> bool:
        with self._state_lock:
            return self._state == ConnectionState.READY

    def _set_state(self, state: ConnectionState) -> None:
        with self._state_lock:
            self._state = state
        self._node.get_logger().info(f'[huayan] State → {state.name}')

    def _ensure_hw(self) -> threading.Lock:
        if not hasattr(self, "_hw_lock"):
            self._hw_lock = threading.Lock()
            self._last_hrif = None
            self._last_fault = None
            self._last_fsm_id = None
            self._last_fsm_desc = None
            if not hasattr(self, "command_log") or self.command_log is None:
                self.command_log = []
        return self._hw_lock

    def _note_hrif(self, name, code, message="", extra=None):
        lock = self._ensure_hw()
        code = int(code)
        if not message and code != 0 and hasattr(self, "_get_error_str"):
            try:
                message = self._get_error_str(code)
            except Exception:
                message = ""
        event = make_hrif_event(
            name, code, message,
            monotonic_sec=time.monotonic(), extra=extra)
        with lock:
            self._last_hrif = event
            if code != 0:
                self._last_fault = event
                self.last_error_string = (
                    "%s failed (code %s): %s" % (name, code, message or event["hint"])
                )
            if should_keep_event(name, code):
                self.command_log = append_event(self.command_log, event)
        return event

    def hardware_status(self) -> dict:
        lock = self._ensure_hw()
        with lock:
            fsm_id = self._last_fsm_id
            fsm_desc = self._last_fsm_desc or ""
            last_hrif = self._last_hrif
            last_fault = self._last_fault
            events = list(self.command_log)
        connection = ""
        if hasattr(self, "_state"):
            with self._state_lock:
                connection = self._state.name
        joints = list(getattr(self, "_current_positions_deg", []) or [])
        return build_snapshot(
            connection=connection,
            fsm_id=fsm_id,
            fsm_desc=fsm_desc,
            last_hrif=last_hrif,
            last_fault=last_fault,
            events=events,
            joints_deg=joints,
            last_error_string=getattr(self, "last_error_string", "") or "",
            monotonic_sec=time.monotonic(),
        )

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self, monitor_only: bool = False) -> bool:
        """
        Establish connection.

        monitor_only=True only opens the CPS TCP session and reads state.
        It does not electrify, reset, or enable the servo — use that for
        bag recording. Full connect() is required before FollowJointTrajectory.
        """
        self._monitor_only = monitor_only
        self._set_state(ConnectionState.CONNECTING)
        if self._cps is None:
            try:
                self._cps = self._import_cps()
            except ImportError as exc:
                self._node.get_logger().error(
                    f'[huayan] Cannot import CPS SDK: {exc}\n'
                    'Ensure CPS_python3_Linux.so is on PYTHONPATH.'
                )
                self._set_state(ConnectionState.ERROR)
                return False

        ok = self._do_connect_monitor() if monitor_only else self._do_connect()
        if not ok:
            self._set_state(ConnectionState.ERROR)
            return False

        self._set_state(ConnectionState.READY)
        return True

    def disconnect(self) -> None:
        """Drop the TCP session. Default leaves 48 V on (no BlackOut)."""
        if self._cps is None:
            return
        try:
            self._node.get_logger().info('[huayan] Disconnecting...')
            if not self._monitor_only:
                try:
                    if self._is_enabled():
                        self._cps.HRIF_GrpDisable(BOX_ID, RBT_ID)
                        time.sleep(0.2)
                except Exception as exc:
                    self._node.get_logger().warn(
                        '[huayan] GrpDisable on disconnect: %s' % exc)
                if self._power_off_on_disconnect:
                    self._node.get_logger().warn(
                        '[huayan] power_off_on_disconnect: HRIF_BlackOut')
                    self._cps.HRIF_BlackOut(BOX_ID)
                    time.sleep(0.3)
            self._cps.HRIF_DisConnect(BOX_ID)
        except Exception as exc:
            self._node.get_logger().warn(f'[huayan] Disconnect error: {exc}')
        finally:
            self._set_state(ConnectionState.DISCONNECTED)

    # ------------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------------

    @property
    def current_positions(self) -> List[float]:
        """Current joint positions in radians (read from hardware)."""
        return [math.radians(d) for d in self._current_positions_deg]

    @property
    def current_velocities(self) -> List[float]:
        """Current joint velocities in rad/s (Huayan reports deg/s)."""
        return [math.radians(d) for d in self._current_velocities_deg]

    @property
    def current_positions_deg(self) -> List[float]:
        return list(self._current_positions_deg)

    @property
    def current_velocities_deg(self) -> List[float]:
        return list(self._current_velocities_deg)

    @property
    def current_accelerations(self) -> List[float]:
        """Joint acceleration in rad/s^2 (finite-diff of deg/s)."""
        return [math.radians(d) for d in self._current_accelerations_deg]

    @property
    def current_accelerations_deg(self) -> List[float]:
        return list(self._current_accelerations_deg)

    @property
    def current_currents(self) -> List[float]:
        """This tick's ``HRIF_ReadActJointCur`` (ampere). Empty if that call failed."""
        return list(self._current_currents_a)

    @property
    def command_positions(self) -> List[float]:
        """This tick's ``HRIF_ReadCmdJointPos`` in radians. Empty if that call failed."""
        return [math.radians(d) for d in self._command_positions_deg]

    @property
    def command_positions_deg(self) -> List[float]:
        return list(self._command_positions_deg)

    @property
    def cps_velocities_deg(self) -> Optional[List[float]]:
        """Raw ``HRIF_ReadActJointVel`` this tick, or None if the call failed."""
        if self._cps_velocities_deg is None:
            return None
        return list(self._cps_velocities_deg)

    @property
    def vel_source(self) -> str:
        """``cps``, ``finite_diff``, or ``none`` for the last velocity sample."""
        return self._vel_source

    @property
    def acc_source(self) -> str:
        """``finite_diff`` or ``none``. CPS has no joint-acceleration read."""
        return self._acc_source

    @property
    def current_tcp_mm_deg(self) -> List[float]:
        """Actual TCP pose: x,y,z mm then Rx,Ry,Rz deg (Huayan Base)."""
        return list(self._current_tcp_mm_deg)

    @property
    def vacuum_di0(self) -> Optional[int]:
        """Box/end DI0 this tick: 1 = suction confirmed, 0 = not holding."""
        return self._vacuum_di0

    @property
    def vacuum_do0(self) -> Optional[int]:
        """DO0 this tick: 1 = vacuum pump on."""
        return self._vacuum_do0

    @property
    def vacuum_do1(self) -> Optional[int]:
        """DO1 this tick: 1 = de-vacuum / release."""
        return self._vacuum_do1

    def set_do_bit(self, bit: int, state: int) -> Tuple[bool, str]:
        """Write one box/end DO bit. Refuses monitor-only sessions."""
        if self._monitor_only:
            return False, "monitor_only refuses SetDO"
        if self._cps is None:
            return False, "CPS not connected"
        value = 1 if int(state) else 0
        # Never command 11: measured 2026-09-09, both-on never seals.
        if value == 1:
            if (int(bit) == int(self.vacuum_do0_bit)
                    and self._vacuum_do1 == 1):
                return False, "refusing DO0=1 while DO1=1"
            if (int(bit) == int(self.vacuum_do1_bit)
                    and self._vacuum_do0 == 1):
                return False, "refusing DO1=1 while DO0=1"
        try:
            if self.vacuum_io_kind == "end":
                n_ret = self._cps.HRIF_SetEndDO(
                    BOX_ID, RBT_ID, int(bit), value)
            else:
                n_ret = self._cps.HRIF_SetBoxDO(BOX_ID, int(bit), value)
        except Exception as exc:
            return False, "SetDO exception: %s" % exc
        if n_ret != 0:
            return False, self._get_error_str(n_ret)
        if int(bit) == int(self.vacuum_do0_bit):
            self._vacuum_do0 = value
        if int(bit) == int(self.vacuum_do1_bit):
            self._vacuum_do1 = value
        return True, ""

    def refresh(self) -> None:
        """Pull a complete CPS snapshot (ACS+TCP, vel, current, cmd ACS)."""
        if self._cps is None:
            return
        try:
            self._refresh_positions()
        except (OSError, ConnectionError):
            self.last_acs_ok = False
            return

    def validate_trajectory(self, trajectory) -> Optional[str]:
        """Return error string or None if trajectory is valid."""
        previous_time = -1.0
        for point_index, pt in enumerate(trajectory.points):
            if len(pt.positions) != len(self.JOINT_NAMES):
                return (
                    "Waypoint %s has %s positions; expected %s"
                    % (point_index, len(pt.positions), len(self.JOINT_NAMES))
                )
            point_time = _duration_to_sec(pt.time_from_start)
            if not math.isfinite(point_time) or point_time < 0.0:
                return "Waypoint %s has invalid time_from_start" % point_index
            if point_index > 0 and point_time <= previous_time:
                return "Waypoint times must be strictly increasing"
            previous_time = point_time
            for field_name in ("velocities", "accelerations"):
                values = getattr(pt, field_name, None) or []
                if values and len(values) != len(self.JOINT_NAMES):
                    return (
                        "Waypoint %s has %s %s; expected 0 or %s"
                        % (
                            point_index,
                            len(values),
                            field_name,
                            len(self.JOINT_NAMES),
                        )
                    )
                if any(not math.isfinite(float(v)) for v in values):
                    return "Waypoint %s has non-finite %s" % (
                        point_index, field_name,
                    )
            for idx, pos in enumerate(pt.positions):
                pos_deg = math.degrees(pos)
                if not math.isfinite(pos_deg):
                    return "Waypoint %s has a non-finite position" % point_index
                lo, hi = self.JOINT_LIMITS_DEG[idx]
                if not (lo <= pos_deg <= hi):
                    return (
                        f"Joint {self.JOINT_NAMES[idx]} at {pos_deg:.1f}° "
                        f"out of limits [{lo:.0f}°, {hi:.0f}°]"
                    )
        return None

    def validate_servo_j_path(self, joint_path_deg) -> Optional[str]:
        """Return error string or None for absolute ServoJ ACS degrees."""
        if not joint_path_deg:
            return "ServoJ path has no points"
        for point_index, joints_deg in enumerate(joint_path_deg):
            if len(joints_deg) != len(self.JOINT_NAMES):
                return (
                    "ServoJ point %s has %s joints; expected %s"
                    % (point_index, len(joints_deg), len(self.JOINT_NAMES))
                )
            for idx, pos_deg in enumerate(joints_deg):
                pos_deg = float(pos_deg)
                if not math.isfinite(pos_deg):
                    return "ServoJ point %s has a non-finite joint" % point_index
                lo, hi = self.JOINT_LIMITS_DEG[idx]
                if not (lo <= pos_deg <= hi):
                    return (
                        f"Joint {self.JOINT_NAMES[idx]} at {pos_deg:.1f}° "
                        f"out of limits [{lo:.0f}°, {hi:.0f}°]"
                    )
        return None

    def execute_servo_j_path(
        self,
        joint_path_deg,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
        servo_time: float = 0.02,
        lookahead_time: float = 0.1,
    ) -> int:
        """Execute absolute joint-degree targets with Start/Init/PushServoJ.

        Official J/P suite is StartServo then PushServoJ. This CPS still
        returns 40071 until InitServoEsJ has cleared the EsJ buffer, so
        the site order is StartServo, then InitServoEsJ. Init after Start
        can 40025; retry Init without Stop. A finished segment, or a DO
        command between segments, drops the buffer: first Push of the
        next goal is 40071. Init-only first; reopen from StandBy after
        that. PushServoEsJ stays unused (20006/20007 on this S20).
        """
        if self._monitor_only:
            self._node.get_logger().error(
                '[huayan] monitor_only is set; refusing ServoJ')
            return RESULT_ERROR
        servo_time = float(servo_time)
        lookahead_time = float(lookahead_time)
        if (
            not math.isfinite(servo_time)
            or not math.isfinite(lookahead_time)
            or servo_time <= 0.0
            or lookahead_time <= 0.0
        ):
            self._node.get_logger().error(
                '[huayan] invalid ServoJ timing: servo_time=%s lookahead=%s'
                % (servo_time, lookahead_time)
            )
            return RESULT_INVALID_GOAL
        error = self.validate_servo_j_path(joint_path_deg)
        if error:
            self._node.get_logger().error(f'[huayan] {error}')
            return RESULT_INVALID_GOAL
        max_vel = float(getattr(self, "_max_vel", 0.0) or 0.0)
        frac = float(getattr(self, "_controller_limit_fraction", 1.0) or 1.0)
        cap = max_vel * frac if max_vel > 0.0 and frac > 0.0 else 0.0
        path = densify_servo_j_path(list(joint_path_deg), servo_time, cap)
        if len(path) != len(joint_path_deg):
            self._node.get_logger().info(
                "[huayan] ServoJ densified %d -> %d points (cap %.1f deg/s)"
                % (len(joint_path_deg), len(path), cap)
            )
        limited = limit_servo_braking(path, servo_time, SERVO_MAX_DECEL_DEG)
        if len(limited) != len(path):
            self._node.get_logger().info(
                "[huayan] ServoJ accel limited %d -> %d points "
                "(accel/decel cap %.0f/%.0f deg/s^2)"
                % (len(path), len(limited), SERVO_MAX_ACCEL_DEG, SERVO_MAX_DECEL_DEG)
            )
        path = limited
        error = self.validate_servo_j_path(path)
        if error:
            self._node.get_logger().error(f'[huayan] {error}')
            return RESULT_INVALID_GOAL
        hold_n = servo_j_hold_count(lookahead_time, servo_time)
        stream = list(path) + [list(path[-1])] * hold_n
        if not self._ensure_connected():
            return RESULT_ERROR
        fsm_id, fsm_desc = self._read_fsm()
        if fsm_id == FSM_COLLISION_STOP:
            self._node.get_logger().error(
                "[huayan] refusing StartServo: FSM 21 RobotCollisionStop. "
                "Clear the collision on the pendant (GrpReset) to StandBy 33."
            )
            return RESULT_ERROR
        if fsm_id is not None and fsm_id < FSM_STANDBY:
            self._node.get_logger().error(
                "[huayan] refusing StartServo: FSM %s (%s); need StandBy 33"
                % (fsm_id, fsm_desc)
            )
            return RESULT_ERROR

        self._set_state(ConnectionState.EXECUTING)
        try:
            if cancel_flag.is_set():
                self._node.get_logger().info('[huayan] ServoJ preempted.')
                self._safe_stop()
                return RESULT_PREEMPTED
            if not self._open_servo_j(servo_time, lookahead_time):
                return RESULT_ERROR

            next_push = time.monotonic()
            reopen_stage = 0
            for index, joints_deg in enumerate(stream):
                if cancel_flag.is_set():
                    self._node.get_logger().info('[huayan] ServoJ preempted.')
                    self._safe_stop()
                    return RESULT_PREEMPTED
                wait_s = next_push - time.monotonic()
                if wait_s > 0.0:
                    time.sleep(wait_s)
                n_ret = hrif_push_servo_j(self._cps, list(joints_deg))
                if n_ret != 0:
                    self._note_hrif(
                        "PushServoJ", n_ret, extra={"index": index})
                # A finished ServoJ segment, or a DO command between
                # segments, drops the EsJ buffer. The next segment's
                # first push then returns 40071 even after StartServo.
                # Init-only first (loading_057 attach won that retry).
                # Full reopen from StandBy only if Init does not take.
                if n_ret == 40071 and reopen_stage == 0:
                    self._node.get_logger().warning(
                        "[huayan] PushServoJ %d returned 40071; "
                        "InitServoEsJ without restart" % index)
                    n_init = self._init_servo_esj(
                        retries=SERVO_ESJ_INIT_RETRIES, log_error=False)
                    reopen_stage = 1
                    if n_init == 0:
                        n_ret = hrif_push_servo_j(self._cps, list(joints_deg))
                        if n_ret != 0:
                            self._note_hrif(
                                "PushServoJ", n_ret, extra={"index": index})
                if n_ret == 40071 and reopen_stage < 2:
                    self._node.get_logger().warning(
                        "[huayan] PushServoJ %d returned 40071; "
                        "reopening ServoJ from StandBy" % index)
                    if not self._open_servo_j(
                            servo_time, lookahead_time, from_standby=True):
                        return RESULT_ERROR
                    reopen_stage = 2
                    n_ret = hrif_push_servo_j(self._cps, list(joints_deg))
                    if n_ret != 0:
                        self._note_hrif(
                            "PushServoJ", n_ret, extra={"index": index})
                if n_ret != 0:
                    msg = self._get_error_str(n_ret)
                    self._node.get_logger().error(
                        f'[huayan] HRIF_PushServoJ {index} failed '
                        f'(code {n_ret}): {msg}')
                    self._safe_stop()
                    self._set_state(ConnectionState.ERROR)
                    return RESULT_ERROR
                self._refresh_positions()
                feedback_fn(self.current_positions)
                next_push += servo_time
        except Exception as exc:
            self._node.get_logger().error(f'[huayan] ServoJ exception: {exc}')
            self._safe_stop()
            self._set_state(ConnectionState.ERROR)
            return RESULT_ERROR

        fsm_id, fsm_desc = self._read_fsm()
        if fsm_id == FSM_COLLISION_STOP:
            self._node.get_logger().error(
                "[huayan] ServoJ ended in FSM 21 RobotCollisionStop (%s)"
                % fsm_desc
            )
            self._set_state(ConnectionState.ERROR)
            return RESULT_ERROR
        self._node.get_logger().info(
            "[huayan] ServoJ execution complete (hold %d at goal, FSM=%s %s)."
            % (hold_n, fsm_id, fsm_desc)
        )
        self._set_state(ConnectionState.READY)
        return RESULT_SUCCESSFUL

    def _wait_fsm_standby(self, timeout_s=3.0, stable_s=SERVO_STANDBY_STABLE_S):
        """Wait until CPS is exactly StandBy 33.

        InitServoEsJ requires StandBy. FSM >= 33 (servo / moving) is
        already_motion_ready for enable, but it is not StandBy. A single
        33 read right after GrpStop can be transient; require it twice.
        """
        deadline = time.monotonic() + float(timeout_s)
        fsm_id, fsm_desc = None, None
        logged = False
        stable_s = max(0.0, float(stable_s))
        while time.monotonic() < deadline:
            fsm_id, fsm_desc = self._read_fsm()
            if fsm_id in (FSM_ESTOP, FSM_ERROR, FSM_COLLISION_STOP):
                return False, fsm_id, fsm_desc
            if fsm_id == FSM_STANDBY:
                if stable_s <= 0.0:
                    return True, fsm_id, fsm_desc
                time.sleep(stable_s)
                fsm_id, fsm_desc = self._read_fsm()
                if fsm_id == FSM_STANDBY:
                    return True, fsm_id, fsm_desc
                continue
            if not logged:
                self._node.get_logger().info(
                    "[huayan] waiting for StandBy 33 (FSM=%s %s)"
                    % (fsm_id, fsm_desc))
                logged = True
            time.sleep(0.1)
        return False, fsm_id, fsm_desc

    def _init_servo_esj(self, retries=0, log_error=True):
        """Clear the EsJ buffer. Retry 40025 without Stop."""
        n_ret = hrif_init_servo_esj(self._cps)
        if n_ret != 0:
            self._note_hrif("InitServoEsJ", n_ret)
        attempt = 0
        retries = max(0, int(retries))
        while n_ret == 40025 and attempt < retries:
            attempt += 1
            self._node.get_logger().warning(
                "[huayan] InitServoEsJ 40025 (not StandBy); retry %d/%d"
                % (attempt, retries))
            time.sleep(SERVO_ESJ_INIT_RETRY_S)
            n_ret = hrif_init_servo_esj(self._cps)
            self._note_hrif("InitServoEsJ", n_ret)
        if n_ret == 0 and attempt == 0:
            self._note_hrif("InitServoEsJ", 0)
        if n_ret != 0 and log_error:
            msg = self._get_error_str(n_ret)
            self._node.get_logger().error(
                f'[huayan] HRIF_InitServoEsJ failed (code {n_ret}): {msg}')
        return n_ret

    def _start_then_init_servo(self, servo_time, lookahead_time):
        """StartServo, then InitServoEsJ.

        Start leaves StandBy. Init after Start can 40025; retry Init
        without Stop. Init-then-Start left the buffer empty on the
        approach after pre_grasp (40071, then 40025 on the Stop reopen).
        """
        n_ret = hrif_start_servo(self._cps, servo_time, lookahead_time)
        self._note_hrif("StartServo", n_ret)
        if n_ret != 0:
            msg = self._get_error_str(n_ret)
            self._node.get_logger().error(
                f'[huayan] HRIF_StartServo failed (code {n_ret}): {msg}')
            return n_ret
        return self._init_servo_esj(retries=SERVO_ESJ_INIT_RETRIES)

    def _fail_servo_open(self):
        fsm_after, _ = self._read_fsm()
        if fsm_after != FSM_COLLISION_STOP:
            self._safe_stop()
        self._set_state(ConnectionState.ERROR)
        return False

    def _open_servo_j(self, servo_time, lookahead_time, from_standby=False):
        """Open ServoJ: Start then Init. Init-only if already in servo."""
        fsm_id, fsm_desc = self._read_fsm()
        if fsm_id in (FSM_ESTOP, FSM_ERROR, FSM_COLLISION_STOP):
            self._node.get_logger().error(
                "[huayan] refusing ServoJ open: FSM %s (%s)"
                % (fsm_id, fsm_desc))
            self._set_state(ConnectionState.ERROR)
            return False
        if fsm_id is not None and fsm_id < FSM_STANDBY:
            self._node.get_logger().error(
                "[huayan] refusing ServoJ open: FSM %s (%s); need StandBy 33"
                % (fsm_id, fsm_desc))
            self._set_state(ConnectionState.ERROR)
            return False
        if from_standby and fsm_id != FSM_STANDBY:
            self._node.get_logger().info(
                "[huayan] reopening ServoJ from StandBy (FSM=%s %s)"
                % (fsm_id, fsm_desc))
            self._safe_stop(resume_state=ConnectionState.EXECUTING)
            ok, fsm_id, fsm_desc = self._wait_fsm_standby()
            if not ok:
                self._node.get_logger().error(
                    "[huayan] still not StandBy for reopen (FSM=%s %s)"
                    % (fsm_id, fsm_desc))
                self._set_state(ConnectionState.ERROR)
                return False
        elif fsm_id is not None and fsm_id != FSM_STANDBY:
            self._node.get_logger().info(
                "[huayan] ServoJ already moving (FSM=%s %s); Init only"
                % (fsm_id, fsm_desc))
            n_ret = self._init_servo_esj(
                retries=SERVO_ESJ_INIT_RETRIES, log_error=False)
            if n_ret == 0:
                return True
            self._node.get_logger().warning(
                "[huayan] Init-only failed (code %s); stop and Start+Init"
                % n_ret)
            self._safe_stop(resume_state=ConnectionState.EXECUTING)
            ok, fsm_id, fsm_desc = self._wait_fsm_standby()
            if not ok:
                self._node.get_logger().error(
                    "[huayan] still not StandBy after Init-only (FSM=%s %s)"
                    % (fsm_id, fsm_desc))
                self._set_state(ConnectionState.ERROR)
                return False
        n_ret = self._start_then_init_servo(servo_time, lookahead_time)
        if n_ret == 0:
            return True
        if n_ret != 40025:
            return self._fail_servo_open()
        self._node.get_logger().warning(
            "[huayan] InitServoEsJ 40025 after Start; stop and retry")
        self._safe_stop(resume_state=ConnectionState.EXECUTING)
        ok, fsm_id, fsm_desc = self._wait_fsm_standby()
        if not ok:
            self._node.get_logger().error(
                "[huayan] still not StandBy after 40025 (FSM=%s %s)"
                % (fsm_id, fsm_desc))
            self._set_state(ConnectionState.ERROR)
            return False
        n_ret = self._start_then_init_servo(servo_time, lookahead_time)
        if n_ret == 0:
            return True
        return self._fail_servo_open()

    def execute(
        self,
        trajectory,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
    ) -> int:
        """
        Execute a JointTrajectory on the real robot.

        Sends each trajectory point as a HRIF_WayPoint (joint-space move)
        with blending on intermediate points and an exact stop at the final
        waypoint.  Polls HRIF_IsBlendingDone (intermediate) and
        HRIF_IsMotionDone (final) for completion.

        Parameters
        ----------
        trajectory : trajectory_msgs.msg.JointTrajectory
        feedback_fn : callable
            Called with current joint positions (radians) during execution.
        cancel_flag : threading.Event
            Set by the action server on preemption.

        Returns
        -------
        int  RESULT_SUCCESSFUL | RESULT_PREEMPTED | RESULT_ERROR | RESULT_INVALID_GOAL
        """
        if self._monitor_only:
            self._node.get_logger().error(
                '[huayan] monitor_only is set; refusing FollowJointTrajectory'
            )
            return RESULT_ERROR

        error = self.validate_trajectory(trajectory)
        if error:
            self.last_error_string = error
            self._node.get_logger().error(f'[huayan] {error}')
            return RESULT_INVALID_GOAL

        if not self._ensure_connected():
            return RESULT_ERROR

        self._set_state(ConnectionState.EXECUTING)

        points = trajectory.points
        if not points:
            self._set_state(ConnectionState.READY)
            return RESULT_SUCCESSFUL

        # Convert trajectory times to seconds for velocity estimation.
        times_s = [_duration_to_sec(pt.time_from_start) for pt in points]
        all_deg = [
            [math.degrees(a) for a in pt.positions] for pt in points
        ]
        self._refresh_positions()
        start_deg = list(self._current_positions_deg)
        kept = decimate_joint_waypoints(
            all_deg, min_delta_deg=WAYPOINT_MIN_DELTA_DEG, start_deg=start_deg,
        )
        self._node.get_logger().info(
            '[huayan] MoveJ %s/%s TOTG points (min Δ=%.1f°)'
            % (len(kept), len(points), WAYPOINT_MIN_DELTA_DEG)
        )

        # Build and validate the complete command list before sending the
        # first waypoint. This prevents a later point from producing 40083
        # after the robot has already moved partway through the goal.
        commands = []
        try:
            for k, i in enumerate(kept):
                is_last = (k == len(kept) - 1)
                pt = points[i]
                joints_deg = all_deg[i]
                next_i = kept[k + 1] if not is_last else None
                next_deg = all_deg[next_i] if next_i is not None else None
                dt = (
                    times_s[next_i] - times_s[i]
                    if next_i is not None else None
                )
                estimated_velocity = self._estimate_velocity(
                    pt, joints_deg, next_deg, dt,
                )
                velocity, acceleration = safe_waypoint_profile(
                    estimated_velocity,
                    self._command_accel,
                    self._max_vel,
                    self._controller_max_velocity_deg,
                    self._controller_max_acceleration_deg,
                    self._controller_limit_fraction,
                )
                radius = FINAL_BLEND_RADIUS_MM if is_last else BLEND_RADIUS_MM
                commands.append(
                    (i, is_last, joints_deg, velocity, acceleration, radius)
                )
        except (TypeError, ValueError) as exc:
            self.last_preflight_reason = str(exc)
            self.last_error_string = "%s: %s" % (PREFLIGHT_FAILED_LOG, exc)
            self._node.get_logger().error(
                '[huayan] %s' % self.last_error_string
            )
            self._set_state(ConnectionState.READY)
            return RESULT_INVALID_GOAL

        try:
            for k, command in enumerate(commands):
                if cancel_flag.is_set():
                    self._node.get_logger().info('[huayan] Trajectory preempted.')
                    self._safe_stop()
                    return RESULT_PREEMPTED

                i, is_last, joints_deg, vel_deg, accel_deg, radius = command

                self._node.get_logger().info(
                    f'[huayan] WP {k+1}/{len(kept)} (totg {i})  '
                    f'J={[f"{d:.1f}" for d in joints_deg]}°  '
                    f'vel={vel_deg:.1f}°/s  acc={accel_deg:.1f}°/s²  '
                    f'r={radius}mm'
                )

                nRet = hrif_waypoint_joint(
                    self._cps, joints_deg, vel_deg, accel_deg, radius, str(i),
                )

                if nRet != 0:
                    msg = self._get_error_str(nRet)
                    self.last_error_string = (
                        'HRIF_WayPoint failed (code %s): %s' % (nRet, msg)
                    )
                    self._node.get_logger().error(
                        f'[huayan] {self.last_error_string}'
                    )
                    self._safe_stop()
                    self._set_state(ConnectionState.ERROR)
                    return RESULT_ERROR

                # Poll for completion and publish feedback.
                result = self._wait_for_waypoint(
                    is_last=is_last,
                    feedback_fn=feedback_fn,
                    cancel_flag=cancel_flag,
                )
                if result != RESULT_SUCCESSFUL:
                    return result

        except Exception as exc:
            self._node.get_logger().error(f'[huayan] Execution exception: {exc}')
            self._safe_stop()
            self._set_state(ConnectionState.ERROR)
            return RESULT_ERROR

        self._node.get_logger().info('[huayan] Trajectory execution complete.')
        self._set_state(ConnectionState.READY)
        return RESULT_SUCCESSFUL

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _do_connect_monitor(self) -> bool:
        """TCP + box session only. Does not enable the servo."""
        log = self._node.get_logger()
        log.info(
            f'[huayan] Monitor connect {self._ip}:{self._port} '
            '(no electrify / enable)'
        )
        nRet = self._cps.HRIF_Connect(BOX_ID, self._ip, self._port)
        if nRet != 0:
            log.error(f'[huayan] HRIF_Connect failed: {self._get_error_str(nRet)}')
            return False
        nRet = self._cps.HRIF_Connect2Box(BOX_ID)
        if nRet != 0:
            log.warn(
                f'[huayan] HRIF_Connect2Box failed: {self._get_error_str(nRet)}; '
                'continuing with TCP session only'
            )
        self._refresh_positions()
        self._read_controller_motion_limits()
        log.info(
            '[huayan] Monitor ready. q_deg=%s'
            % [round(v, 2) for v in self._current_positions_deg]
        )
        return True

    def _do_connect(self) -> bool:
        """Low-level connection sequence. Returns True on success.

        The cell is often already in StandBy (FSM 33): ConnectToBox /
        Electrify then refuse with 20018. Treat those as already-done,
        matching the Noetic executor. Do not GrpReset a standing arm.
        """
        log = self._node.get_logger()

        if not self._cps.HRIF_IsConnected(BOX_ID):
            log.info(f'[huayan] Connecting to {self._ip}:{self._port}...')
            nRet = self._cps.HRIF_Connect(BOX_ID, self._ip, self._port)
            if nRet != 0:
                log.error(
                    f'[huayan] HRIF_Connect failed: {self._get_error_str(nRet)}')
                return False
        else:
            log.info('[huayan] TCP already up; continuing enable sequence')

        fsm_id, fsm_desc = self._read_fsm()
        if fsm_id is not None:
            log.info('[huayan] Current robot FSM: %s (%s)' % (fsm_id, fsm_desc))
        if fsm_id in (FSM_ESTOP, FSM_ERROR):
            log.error(
                '[huayan] Robot is in fault/e-stop (%s). Clear it on the pendant.'
                % fsm_id)
            return False

        if not self._cps_step(
                'HRIF_Connect2Box',
                lambda: self._cps.HRIF_Connect2Box(BOX_ID),
                connect2box_allow_refuse(fsm_id)):
            return False

        if not self._cps_step(
                'HRIF_Electrify',
                lambda: self._cps.HRIF_Electrify(BOX_ID),
                electrify_allow_refuse(fsm_id)):
            return False

        if fsm_id in (FSM_BLACKOUTING, FSM_BLACKOUT, FSM_ELECTRIFYING):
            log.info('[huayan] Waiting for controller to start...')
            if not self._wait_controller_started():
                return False
            fsm_id, fsm_desc = self._read_fsm()
            if fsm_id is not None:
                log.info(
                    '[huayan] Robot FSM after power-up: %s (%s)'
                    % (fsm_id, fsm_desc))

        allow_controller = (
            electrify_allow_refuse(fsm_id) or self._controller_started())
        if not self._cps_step(
                'HRIF_Connect2Controller',
                lambda: self._cps.HRIF_Connect2Controller(BOX_ID),
                allow_controller):
            return False

        if not already_motion_ready(fsm_id) and not self._controller_started():
            log.info('[huayan] Waiting for controller to start...')
            if not self._wait_controller_started():
                return False

        if already_motion_ready(fsm_id):
            log.info('[huayan] Servos already enabled (FSM=%s)' % fsm_id)
        else:
            if fsm_id == FSM_DISABLE:
                self._cps_step(
                    'HRIF_GrpReset',
                    lambda: self._cps.HRIF_GrpReset(BOX_ID, RBT_ID),
                    True)
                time.sleep(0.3)
            # 20018 is OK if enable raced with the FSM read (already StandBy).
            if not self._cps_step(
                    'HRIF_GrpEnable',
                    lambda: self._cps.HRIF_GrpEnable(BOX_ID, RBT_ID),
                    True):
                return False
            ok, wait_id, wait_desc = self._wait_fsm_standby(timeout_s=5.0)
            if not ok:
                log.error(
                    '[huayan] not StandBy 33 after enable (FSM=%s %s)'
                    % (wait_id, wait_desc))
                return False
            fsm_id = wait_id
        fsm_id, fsm_desc = self._read_fsm()
        if not already_motion_ready(fsm_id):
            log.error(
                '[huayan] refusing READY: need StandBy 33, FSM=%s (%s)'
                % (fsm_id, fsm_desc))
            return False

        self._refresh_positions()
        if not self._read_controller_motion_limits():
            log.error(
                '[huayan] Required controller motion limits unavailable; '
                'refusing hardware readiness'
            )
            return False
        log.info('[huayan] Robot ready.')
        return True

    def _read_controller_motion_limits(self) -> bool:
        """Cache limits and report whether motion-critical reads passed."""
        log = self._node.get_logger()
        version = []
        try:
            n_ver = self._cps.HRIF_ReadVersion(BOX_ID, RBT_ID, version)
        except Exception:
            n_ver = 1
        if n_ver == 0 and version:
            log.info('[huayan] CPS versions: %s' % list(version))
        self._controller_max_velocity_deg = read_positive_joint_limits(
            self._cps, 'HRIF_ReadJointMaxVel',
        )
        self._controller_max_acceleration_deg = read_positive_joint_limits(
            self._cps, 'HRIF_ReadJointMaxAcc',
        )
        self._controller_max_jerk_deg = read_positive_joint_limits(
            self._cps, 'HRIF_ReadJointMaxJerk',
        )
        if (
            self._controller_max_velocity_deg is None
            or self._controller_max_acceleration_deg is None
        ):
            log.warn(
                '[huayan] Required controller velocity/acceleration limits '
                'unavailable; hardware motion is disabled'
            )
            return False
        log.info(
            '[huayan] Controller limits vel=%s deg/s acc=%s deg/s^2 jerk=%s '
            'command_fraction=%.3f'
            % (
                [round(v, 3) for v in self._controller_max_velocity_deg],
                [round(v, 3) for v in self._controller_max_acceleration_deg],
                None if self._controller_max_jerk_deg is None else
                [round(v, 3) for v in self._controller_max_jerk_deg],
                self._controller_limit_fraction,
            )
        )
        if self._controller_max_jerk_deg is None:
            log.warn(
                '[huayan] Controller jerk limits unavailable; continuing '
                'because HRIF_WayPoint does not command jerk'
            )
        return True

    def _cps_step(self, name, fn, allow_refuse: bool) -> bool:
        n_ret = fn()
        if cps_step_ok(n_ret, allow_refuse):
            if n_ret != 0:
                self._node.get_logger().info(
                    '[huayan] %s skipped: %s' % (name, self._get_error_str(n_ret)))
            return True
        self._note_hrif(name, n_ret)
        self._node.get_logger().error(
            '[huayan] %s failed: %s' % (name, self._get_error_str(n_ret)))
        return False

    def _read_fsm(self):
        result = []
        try:
            n_ret = self._cps.HRIF_ReadCurFSM(BOX_ID, RBT_ID, result)
        except Exception:
            return None, None
        if n_ret != 0 or not result:
            return None, None
        try:
            fsm_id = int(result[0])
        except (TypeError, ValueError):
            return None, result[0]
        desc = result[1] if len(result) > 1 else str(result[0])
        lock = self._ensure_hw()
        with lock:
            self._last_fsm_id = fsm_id
            self._last_fsm_desc = desc
        return fsm_id, desc

    def _controller_started(self) -> bool:
        result = []
        try:
            n_ret = self._cps.HRIF_IsControllerStarted(BOX_ID, result)
        except Exception:
            return False
        return n_ret == 0 and result and str(result[0]) == '1'

    def _is_enabled(self) -> bool:
        result = []
        try:
            n_ret = self._cps.HRIF_ReadRobotState(BOX_ID, RBT_ID, result)
        except Exception:
            return False
        return n_ret == 0 and len(result) > 1 and str(result[1]) == '1'

    def _wait_controller_started(self) -> bool:
        deadline = time.monotonic() + CONTROLLER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._controller_started():
                return True
            fsm_id, _ = self._read_fsm()
            if fsm_id in (FSM_ESTOP, FSM_ERROR):
                self._node.get_logger().error(
                    '[huayan] fault/e-stop while waiting for controller (%s)'
                    % fsm_id)
                return False
            time.sleep(0.5)
        self._node.get_logger().error(
            '[huayan] Controller did not start within timeout.')
        return False

    def _ensure_connected(self) -> bool:
        """
        Check connection health; reconnect with backoff if needed.
        Returns True when ready, False after exhausting retries.
        """
        if self._cps is None:
            return False

        tcp_up = False
        try:
            tcp_up = bool(self._cps.HRIF_IsConnected(BOX_ID))
        except Exception:
            tcp_up = False
        if self.is_ready and tcp_up:
            return True
        if tcp_up:
            self._node.get_logger().warn(
                '[huayan] TCP up but not READY; retrying enable sequence')
            if self._do_connect():
                self._set_state(ConnectionState.READY)
                return True
            self._set_state(ConnectionState.ERROR)
            return False

        self._node.get_logger().warn('[huayan] Connection lost. Attempting reconnect...')
        for attempt in range(1, MAX_RECONNECT_RETRIES + 1):
            wait = 2 ** attempt
            self._node.get_logger().info(
                f'[huayan] Reconnect attempt {attempt}/{MAX_RECONNECT_RETRIES} '
                f'in {wait}s...'
            )
            time.sleep(wait)
            if self._do_connect():
                self._set_state(ConnectionState.READY)
                return True

        self._node.get_logger().error('[huayan] Reconnect failed after all retries.')
        self._set_state(ConnectionState.ERROR)
        return False

    def _wait_for_waypoint(
        self,
        is_last: bool,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
    ) -> int:
        """
        Poll for waypoint/motion completion.

        For intermediate points we check HRIF_IsBlendingDone (waypoint queue
        consumed) since the robot keeps moving during blending.
        For the last point we wait for HRIF_IsMotionDone (full stop).

        IsBlendingDone is true while idle. Wait until it goes false (motion
        started) before treating true as "this waypoint consumed"; otherwise
        every TOTG sample is queued immediately and the arm chatters.
        """
        saw_busy = False
        busy_deadline = time.monotonic() + getattr(
            self, "_blend_start_timeout_s", BLEND_START_TIMEOUT_S)
        while True:
            if cancel_flag.is_set():
                self._safe_stop()
                return RESULT_PREEMPTED

            self._refresh_positions()
            feedback_fn(self.current_positions)

            result = []
            if is_last:
                nRet = self._cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, result)
                if (
                    nRet != 0
                    or len(result) != 1
                    or not isinstance(result[0], bool)
                ):
                    return self._poll_failure(
                        'HRIF_IsMotionDone', nRet, result)
                done = result[0]
                if done:
                    return RESULT_SUCCESSFUL
            else:
                nRet = self._cps.HRIF_IsBlendingDone(BOX_ID, RBT_ID, result)
                if (
                    nRet != 0
                    or len(result) != 1
                    or not isinstance(result[0], bool)
                ):
                    return self._poll_failure(
                        'HRIF_IsBlendingDone', nRet, result)
                blending_done = result[0]
                if not blending_done:
                    saw_busy = True
                elif saw_busy:
                    return RESULT_SUCCESSFUL
                elif time.monotonic() >= busy_deadline:
                    return RESULT_SUCCESSFUL

            interval = getattr(self, "_poll_interval_s", POLL_INTERVAL_S)
            if interval > 0.0:
                time.sleep(interval)

    def _poll_failure(self, method_name: str, n_ret: int, result) -> int:
        """Stop and enter ERROR on a failed or malformed CPS completion poll."""
        if n_ret != 0:
            detail = 'code %s: %s' % (n_ret, self._get_error_str(n_ret))
        else:
            detail = 'malformed success payload %r' % (result,)
        self._node.get_logger().error(
            '[huayan] %s failed (%s)' % (method_name, detail))
        self._safe_stop()
        self._set_state(ConnectionState.ERROR)
        return RESULT_ERROR

    def _refresh_positions(self) -> None:
        """One complete CPS snapshot. Never reuse a stale field under a new stamp.

        ``HRIF_ReadActPos`` is one packet (ACS + the pose slice SDK uses as TCP).
        Vel / current / cmd ACS are extra RTTs on the same tick so the bag has
        the controller's own values, not held-over samples from earlier ticks.
        """
        t0 = time.monotonic()
        dt = t0 - self._last_refresh_mono if self._last_refresh_mono else 0.0
        prev = list(self._current_positions_deg)
        prev_vel = list(self._current_velocities_deg)
        self._refresh_n += 1
        self.last_acs_ok = False
        self.last_tcp_ok = False
        self.last_vel_cps_ok = False
        self.last_cur_ok = False
        self.last_cmd_ok = False
        self.last_io_ok = False
        self._cps_velocities_deg = None
        self._current_currents_a = []
        self._command_positions_deg = []
        self._vacuum_di0 = None
        self._vacuum_do0 = None
        self._vacuum_do1 = None

        pose = []
        n_pose = self._cps.HRIF_ReadActPos(BOX_ID, RBT_ID, pose)
        self.last_sample_mono = time.monotonic()
        parsed = acs_from_read_act_pos(pose) if n_pose == 0 else None
        tcp = tcp_from_read_act_pos(pose) if n_pose == 0 else None
        if parsed is None:
            result = []
            n_acs = self._cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, result)
            parsed = as_float_list(result, 6) if n_acs == 0 else None
        if parsed is None:
            self.last_refresh_dt_s = time.monotonic() - t0
            return
        self._current_positions_deg = parsed
        self.last_acs_ok = True
        if tcp is not None:
            self._current_tcp_mm_deg = tcp
            self.last_tcp_ok = True

        vel = []
        nRet = self._cps.HRIF_ReadActJointVel(BOX_ID, RBT_ID, vel)
        parsed_vel = as_float_list(vel, 6) if nRet == 0 else None
        self._cps_velocities_deg = parsed_vel
        self.last_vel_cps_ok = parsed_vel is not None
        fd = finite_diff_deg_s(prev, self._current_positions_deg, dt)
        moved = joints_moved_deg(prev, self._current_positions_deg)
        chosen, src = choose_joint_vel_deg(parsed_vel, fd, moved)
        if chosen is not None:
            self._current_velocities_deg = chosen
            self._vel_source = src
        else:
            self._current_velocities_deg = [0.0] * 6
            self._vel_source = "none"

        acc = finite_diff_deg_s(prev_vel, self._current_velocities_deg, dt)
        if acc is not None:
            self._current_accelerations_deg = acc
            self._acc_source = "finite_diff"
        else:
            self._current_accelerations_deg = [0.0] * 6
            self._acc_source = "none"

        cur = []
        nRet = self._cps.HRIF_ReadActJointCur(BOX_ID, RBT_ID, cur)
        parsed_cur = as_float_list(cur, 6) if nRet == 0 else None
        if parsed_cur is not None:
            self._current_currents_a = parsed_cur
            self.last_cur_ok = True

        cmd = []
        n_cmd = self._cps.HRIF_ReadCmdJointPos(BOX_ID, RBT_ID, cmd)
        parsed_cmd = as_float_list(cmd, 6) if n_cmd == 0 else None
        if parsed_cmd is not None:
            self._command_positions_deg = parsed_cmd
            self.last_cmd_ok = True

        self._vacuum_di0 = self._read_di_bit(self.vacuum_di_bit)
        self._vacuum_do0 = self._read_do_bit(self.vacuum_do0_bit)
        self._vacuum_do1 = self._read_do_bit(self.vacuum_do1_bit)
        self.last_io_ok = (
            self._vacuum_di0 is not None
            and self._vacuum_do0 is not None
            and self._vacuum_do1 is not None
        )

        self.last_refresh_dt_s = time.monotonic() - t0
        self._last_refresh_mono = t0

    def _read_di_bit(self, bit: int) -> Optional[int]:
        result = []
        if self.vacuum_io_kind == "end":
            n_ret = self._cps.HRIF_ReadEndDI(BOX_ID, RBT_ID, int(bit), result)
        else:
            n_ret = self._cps.HRIF_ReadBoxDI(BOX_ID, int(bit), result)
        return as_bit(result) if n_ret == 0 else None

    def _read_do_bit(self, bit: int) -> Optional[int]:
        result = []
        if self.vacuum_io_kind == "end":
            n_ret = self._cps.HRIF_ReadEndDO(BOX_ID, RBT_ID, int(bit), result)
        else:
            n_ret = self._cps.HRIF_ReadBoxDO(BOX_ID, int(bit), result)
        return as_bit(result) if n_ret == 0 else None

    def _safe_stop(self, resume_state=None) -> None:
        """Best-effort emergency stop."""
        try:
            if self._cps is not None:
                self._cps.HRIF_GrpStop(BOX_ID, RBT_ID)
                self._note_hrif("GrpStop", 0)
                settle = getattr(self, "_stop_settle_s", 0.1)
                if settle > 0.0:
                    time.sleep(settle)
                self._cps.HRIF_GrpReset(BOX_ID, RBT_ID)
                self._note_hrif("GrpReset", 0)
        except Exception as exc:
            self._node.get_logger().warn(f'[huayan] Stop error: {exc}')
        finally:
            with self._state_lock:
                if resume_state is not None:
                    self._state = resume_state
                elif self._state == ConnectionState.EXECUTING:
                    self._state = ConnectionState.READY

    def _get_error_str(self, code: int) -> str:
        """Return human-readable description for an HRIF error code."""
        try:
            result = []
            self._cps.HRIF_GetErrorCodeStr(BOX_ID, code, result)
            if result:
                return result[0]
        except Exception:
            pass
        return f'(unknown error {code})'

    def _estimate_velocity(
        self,
        pt,
        joints_deg: List[float],
        next_joints_deg: Optional[List[float]],
        dt: Optional[float],
    ) -> float:
        """Velocity for one MoveJ after TOTG decimation.

        Dense TOTG sample speeds are often << 1 deg/s (Huayan 20070). Flooring
        every sample to default_vel made the arm reverse between nearby
        points. After decimation each MoveJ is a real via, so use at least
        default_vel and never below Huayan's 1 deg/s floor.
        """
        hi = float(self._max_vel)
        fallback = max(HUAYAN_MIN_VELOCITY_DEG, float(self._default_vel))

        def _clamp(vel):
            return max(fallback, min(float(vel), hi))

        if pt.velocities:
            max_vel = max(abs(math.degrees(v)) for v in pt.velocities)
            if max_vel > 0.0:
                return _clamp(max_vel)

        if next_joints_deg is not None and dt is not None and dt > 1e-6:
            vel = _max_joint_delta_deg(next_joints_deg, joints_deg) / dt
            if vel > 0.0:
                return _clamp(vel)

        return fallback

    @staticmethod
    def _import_cps():
        """
        Lazily import the CPSClient class from the Huayan SDK.

        The SDK .so file must be on sys.path / PYTHONPATH.  We rename
        CPS_python3_Linux.so → CPS.so (or CPS.pyd on Windows) as per the
        official documentation so the import works as `from CPS import CPSClient`.
        """
        import os
        import sys
        from pathlib import Path

        candidates = []
        if os.environ.get("HUAYAN_SDK"):
            candidates.append(Path(os.environ["HUAYAN_SDK"]))
        for part in os.environ.get("PYTHONPATH", "").split(os.pathsep):
            if part:
                candidates.append(Path(part))
        for parent in Path(__file__).resolve().parents:
            candidates.append(parent / "third_party" / "huayan_python_sdk")
        for cand in candidates:
            if (cand / "CPS.py").is_file():
                path = str(cand)
                if path not in sys.path:
                    sys.path.insert(0, path)
                break
        from CPS import CPSClient  # noqa: PLC0415
        return CPSClient()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _duration_to_sec(duration) -> float:
    """Convert builtin_interfaces/Duration to float seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9
