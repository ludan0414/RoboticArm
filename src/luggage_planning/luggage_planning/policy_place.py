"""ROS-free, single-box placement from a policy's world-frame box pose.

The policy owns target selection and collision-free cargo routing. These
waypoints still go through PlanMotion with the attached measured payload.
No packing-map commit or placement verification is implied by success.
"""

from dataclasses import dataclass, replace
import math

from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion
from luggage_planning.suction_candidate_waypoints import (
    build_candidate_pick_segments, quat_conjugate, quat_mul, quat_rotate,
)


def _vector(values, count, name):
    if not isinstance(values, (list, tuple)) or len(values) != count:
        raise ValueError("%s needs %d numbers" % (name, count))
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError("%s must be finite" % name)
    return result


def _quaternion(values):
    values = _vector(values, 4, "orientation_xyzw")
    norm = math.sqrt(sum(v * v for v in values))
    if abs(norm - 1.0) > 0.01:
        raise ValueError("orientation_xyzw must be a unit quaternion")
    return Quaternion(*(v / norm for v in values))


def _pose(data):
    return Pose(Point(*_vector(data["position_m"], 3, "position_m")),
                _quaternion(data["orientation_xyzw"]))


@dataclass(frozen=True)
class PlaceTarget:
    box_pose: Pose
    clearance_m: float
    via: tuple = ()
    # Frozen policy input dimensions; never a replacement for measured geometry.
    reference_box_size_m: tuple = ()


def parse_place_target(data):
    """Require an explicit world-frame, box-centre, metre/xyzw contract."""
    if (data.get("frame_id") != "world"
            or data.get("pose_reference") != "box_center"):
        raise ValueError("place target requires frame_id=world, pose_reference=box_center")
    clearance = float(data.get("clearance_m", 0.15))
    if not math.isfinite(clearance) or clearance < 0.05:
        raise ValueError("clearance_m must be finite and >= 0.05")
    via = tuple(_pose(p) for p in data.get("via_box_poses", []))
    return PlaceTarget(_pose(data), clearance, via)


def measured_grasp_transform(box_pose, tool_pose):
    """T_box_tool at actual sealed contact, not the pre-pick IK request."""
    box_q = _quaternion([getattr(box_pose.orientation, k) for k in ("x", "y", "z", "w")])
    tool_q = _quaternion([getattr(tool_pose.orientation, k) for k in ("x", "y", "z", "w")])
    offset = quat_rotate(quat_conjugate(box_q), tuple(
        getattr(tool_pose.position, k)-getattr(box_pose.position, k) for k in ("x", "y", "z")))
    return offset, quat_mul(quat_conjugate(box_q), tool_q)


def box_pose_from_grasp(tool_pose, grasp):
    offset, relative_q = grasp
    q = quat_mul(tool_pose.orientation, quat_conjugate(relative_q))
    rotated = quat_rotate(q, offset)
    return Pose(Point(*(getattr(tool_pose.position, k)-v
                        for k, v in zip(("x", "y", "z"), rotated))), q)


def grasp_transform(detection, candidate):
    """Measured T_box_tool shared by placement and the post-pick connector."""
    if detection.frame != "world" or candidate.frame != "world" or not detection.yaw_valid:
        raise ValueError("world-frame measured grasp and valid box yaw required")
    pick_box_q = _quaternion(detection.box_quat)
    attach = build_candidate_pick_segments(candidate, detection_yaw=detection.detection_yaw,
                                            yaw_valid=detection.yaw_valid)[2].target_pose
    offset = quat_rotate(quat_conjugate(pick_box_q), tuple(
        a - b for a, b in zip((attach.position.x, attach.position.y, attach.position.z), detection.box_xyz)))
    return offset, quat_mul(quat_conjugate(pick_box_q), attach.orientation)


def build_place_segments(target, detection, candidate, measured_grasp=None):
    """Preserve the selected grasp transform: T_world_tool = T_world_box T_box_tool.

    Approach/retreat are vertical in world. Optional policy via poses describe
    the BOX centre and orientation, in execution order, before pre_place.
    """
    if detection.frame != "world" or candidate.frame != "world":
        raise ValueError("pick observation and candidate must be in world")
    if not detection.yaw_valid:
        raise ValueError("policy box orientation requires a measured valid pick yaw")
    offset, relative_q = (measured_grasp if measured_grasp is not None
                          else grasp_transform(detection, candidate))

    def tool_pose(box_pose, lift=0.0):
        rotated = quat_rotate(box_pose.orientation, offset)
        return Pose(Point(box_pose.position.x + rotated[0],
                          box_pose.position.y + rotated[1],
                          box_pose.position.z + rotated[2] + lift),
                    quat_mul(box_pose.orientation, relative_q))

    def segment(name, pose, cartesian=True):
        return MotionSegment(
            name=name, type="cartesian" if cartesian else "pose_target",
            target_pose=pose, allow_ompl_fallback=name.startswith("place_via_") or name == "pre_place",
            required_cartesian_fraction=1.0 if cartesian else 0.0)

    segments = [segment("place_via_%d" % i, tool_pose(p))
                for i, p in enumerate(target.via)]
    segments.extend([
        segment("pre_place", tool_pose(target.box_pose, target.clearance_m),
                cartesian=bool(target.via)),
        segment("place_descend", tool_pose(target.box_pose)),
        segment("place_retreat", tool_pose(target.box_pose, target.clearance_m)),
    ])
    return segments


def connect_place_from_pose(actual, segments, max_translation_m=.15,
                            max_rotation_rad=math.radians(20)):
    """Join measured successful Pick retreat to the first Cartesian carry pose.

    Only subdivides the first connection; downstream route and return targets
    stay unchanged. Each piece still requires complete collision-checked
    Cartesian planning with the attached payload; there is no OMPL fallback.
    """
    segments = list(segments)
    if not 0 < max_translation_m <= .15 or not 0 < max_rotation_rad <= math.radians(20):
        raise ValueError("invalid connector sampling limits")
    if not segments or segments[0].type != "cartesian":
        return segments
    first = segments[0]
    start = _vector([actual.position.x, actual.position.y, actual.position.z], 3, "live TCP")
    end = _vector([first.target_pose.position.x, first.target_pose.position.y,
                   first.target_pose.position.z], 3, "carry TCP")
    quats = []
    for pose in (actual, first.target_pose):
        q = _quaternion([pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w])
        quats.append((q.x, q.y, q.z, q.w))
    a, b = quats
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    angle = math.acos(min(1.0, dot))
    count = max(1, math.ceil(math.dist(start, end) / max_translation_m),
                math.ceil(2 * angle / max_rotation_rad))
    connection = []
    for index in range(1, count):
        t = index / count
        if angle < 1e-6:
            q = a
        else:
            q = tuple((math.sin((1 - t) * angle) * x + math.sin(t * angle) * y)
                      / math.sin(angle) for x, y in zip(a, b))
        pose = Pose(Point(*(x + t * (y - x) for x, y in zip(start, end))), Quaternion(*q))
        connection.append(replace(first, name="place_pickup_connect_%d" % index,
            target_pose=pose, waypoints=[], allow_ompl_fallback=False,
            required_cartesian_fraction=1.0))
    return connection + segments


def run_place(ports, segments, use_scene=True, low_hold_sec=0.5,
              release_deadline_sec=2.0):
    """Return (exit_code, reason). Carry faults preserve vacuum (exit 10).

    Release/scene/retreat faults return 11. Never retreat after an unconfirmed
    release. Ports are the same injected ports used by PickSession.
    """
    def log(stage, **fields):
        ports.log_event(dict(stage=stage, **fields))

    release_started = False
    try:
        for segment in segments[:-1]:
            healthy, reason = ports.graph_health()
            if not healthy or ports.di0() != 1:
                log("place_carry_fault", reason=reason)
                return 10, "PLACE_CARRY_FAULT: %s / DI0" % reason
            outcome = ports.execute_segment(segment, "place")
            log(segment.name, ok=outcome.ok, message=outcome.message)
            if not outcome.ok:
                return 10, "PLACE_CARRY_FAULT [%s]: %s" % (segment.name, outcome.message)
        if ports.di0() != 1:
            return 10, "PLACE_CARRY_FAULT: DI0 lost before release"
        validate_pose = getattr(ports, "validate_place_pose", None)
        if callable(validate_pose):
            ok, reason = validate_pose()
            log("place_pose_check", ok=ok, message=reason)
            if not ok:
                return 10, "PLACE_POSE_NOT_ALIGNED: " + reason
        release_started = True
        ok, reason = ports.vacuum_release("place")
        log("place_release", ok=ok, message=reason)
        if not ok:
            return 11, "PLACE_RELEASE_FAILED: %s" % reason
        start, low_since = ports.now(), None
        while ports.now() - start <= release_deadline_sec:
            now = ports.now()
            if ports.di0() == 0:
                if low_since is None:
                    low_since = now
                if now - low_since >= low_hold_sec:
                    break
            else:
                low_since = None
            ports.sleep(0.05)
        else:
            return 11, "PLACE_RELEASE_UNCONFIRMED"
        if use_scene:
            ok, reason = ports.scene_remove()
            if not ok:
                return 11, "PLACE_SCENE_DETACH_FAILED: %s" % reason
        outcome = ports.execute_segment(segments[-1], "place")
        log("place_retreat", ok=outcome.ok, message=outcome.message)
        return (0, "PLACE_COMPLETE") if outcome.ok else (
            11, "PLACE_RETREAT_FAILED: %s" % outcome.message)
    except Exception as exc:
        # The caller must never enable automatic vacuum cleanup on this path.
        log("place_exception", message=str(exc))
        return (11 if release_started else 10), (
            "PLACE_FAULT: %s; explicit recovery required" % exc)
