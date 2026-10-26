"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Convert a MoveIt FollowJointTrajectory into absolute joint-degree samples
on the PushServoJ time grid. MoveIt TOTG uses ~0.10 s knots; HRIF_PushServoJ
needs a fixed servo_time (default 0.02 s).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .servo_esj import interpolate_joint_grid


def fjt_to_servo_j_deg(
    positions_rad: Sequence[Sequence[float]],
    times_s: Sequence[float],
    *,
    dt_s: float,
    velocities_rad: Optional[Sequence[Optional[Sequence[float]]]] = None,
) -> Tuple[Optional[List[List[float]]], Optional[str]]:
    """Resample FJT knots (rad, seconds) onto a fixed ServoJ grid (deg)."""
    if not positions_rad:
        return [], None
    if len(positions_rad) != len(times_s):
        return None, "FJT position/time count mismatch"
    joints_deg: List[List[float]] = []
    for index, q in enumerate(positions_rad):
        if len(q) != 6:
            return None, "point %d has %d joints; expected 6" % (index, len(q))
        try:
            deg = [math.degrees(float(v)) for v in q]
        except (TypeError, ValueError):
            return None, "point %d has non-numeric positions" % index
        if any(not math.isfinite(v) for v in deg):
            return None, "point %d has non-finite positions" % index
        joints_deg.append(deg)
    vel_deg = None
    if velocities_rad is not None and len(velocities_rad) == len(positions_rad):
        converted: List[Optional[Sequence[float]]] = []
        usable = True
        for item in velocities_rad:
            if item is None or len(item) != 6:
                usable = False
                break
            try:
                converted.append([math.degrees(float(v)) for v in item])
            except (TypeError, ValueError):
                usable = False
                break
            if any(not math.isfinite(v) for v in converted[-1]):
                usable = False
                break
        if usable:
            vel_deg = converted
    return interpolate_joint_grid(
        joints_deg, times_s, dt_s=dt_s, velocities_deg=vel_deg,
    )


def densify_servo_j_path(
    path_deg: Sequence[Sequence[float]],
    dt_s: float,
    max_vel_deg: float,
) -> List[List[float]]:
    """Insert linear samples so |dq|/dt stays at or under max_vel_deg."""
    if not path_deg:
        return []
    if dt_s <= 0.0 or max_vel_deg <= 0.0 or not math.isfinite(max_vel_deg):
        return [list(map(float, q[:6])) for q in path_deg]
    max_step = max_vel_deg * dt_s
    out: List[List[float]] = [list(map(float, path_deg[0][:6]))]
    for raw in path_deg[1:]:
        nxt = list(map(float, raw[:6]))
        prev = out[-1]
        delta = max(abs(nxt[i] - prev[i]) for i in range(6))
        steps = 1 if delta <= max_step else int(math.ceil(delta / max_step))
        for k in range(1, steps + 1):
            u = k / float(steps)
            out.append([prev[j] + u * (nxt[j] - prev[j]) for j in range(6)])
    return out


# ServoJ position pushes have no acceleration field. 60 deg/s^2 on the
# lift (pick_retreat) and on the brake (pre_grasp) both tripped collision
# stop. The executed grid caps both from the first sample.
SERVO_MAX_ACCEL_DEG = 20.0
SERVO_MAX_DECEL_DEG = 20.0


def _clamp_signed_speed(prev: float, desired: float, rate_limit: float) -> float:
    """Move toward ``desired`` by at most ``rate_limit`` in either direction."""
    if desired > prev + rate_limit:
        return prev + rate_limit
    if desired < prev - rate_limit:
        return prev - rate_limit
    return desired


def _max_stop_speed(remaining_deg: float, accel: float, dt_s: float) -> float:
    """Fastest speed that can still stop in whole ``dt_s`` steps."""
    if remaining_deg <= 1e-12 or accel <= 0.0 or dt_s <= 0.0:
        return 0.0
    quantum = accel * dt_s * dt_s
    if remaining_deg <= quantum:
        return remaining_deg / dt_s
    n = int((-1.0 + math.sqrt(1.0 + 8.0 * remaining_deg / quantum)) / 2.0)
    return max(0, n) * accel * dt_s


def _rate_ok(before: float, after: float, dt_s: float, limit_deg: float) -> bool:
    if before * after < 0.0:
        delta = abs(before) + abs(after)
    else:
        delta = abs(abs(before) - abs(after))
    return delta / dt_s <= limit_deg + 1e-4


def _braking_ok(pts, dt_s, max_decel_deg) -> bool:
    prev = [0.0] * 6
    for left, right in zip(pts, pts[1:]):
        vel = [(right[j] - left[j]) / dt_s for j in range(6)]
        for before, after in zip(prev, vel):
            if not _rate_ok(before, after, dt_s, max_decel_deg):
                return False
        prev = vel
    for before in prev:
        if abs(before) / dt_s > max_decel_deg + 1e-4:
            return False
    return True


def _span_deg(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(b[j]) - float(a[j])) for j in range(6))


def _sample_at(pts, knots, arc: float) -> List[float]:
    if arc <= 0.0:
        return list(pts[0])
    if arc >= knots[-1]:
        return list(pts[-1])
    hi = 1
    while hi < len(knots) - 1 and knots[hi] < arc:
        hi += 1
    span = knots[hi] - knots[hi - 1]
    if span <= 1e-12:
        return list(pts[hi])
    u = (arc - knots[hi - 1]) / span
    return [
        pts[hi - 1][j] + u * (pts[hi][j] - pts[hi - 1][j]) for j in range(6)
    ]


def limit_servo_braking(
    path_deg: Sequence[Sequence[float]],
    dt_s: float,
    max_decel_deg: float = SERVO_MAX_DECEL_DEG,
) -> List[List[float]]:
    """Lengthen the path so accel and braking stay at or under the cap.

    PushServoJ has no acceleration field. A 20 ms step that jumps off rest
    or dumps cruise speed to zero is thousands of deg/s^2. Samples stay on
    the original joint path; only the ramps take more time. A path already
    inside the cap is returned as-is.
    """
    if not path_deg:
        return []
    pts = [list(map(float, q[:6])) for q in path_deg]
    if (
        dt_s <= 0.0
        or max_decel_deg <= 0.0
        or not math.isfinite(max_decel_deg)
        or len(pts) < 2
        or _braking_ok(pts, dt_s, max_decel_deg)
    ):
        return pts
    knots = [0.0]
    for index in range(len(pts) - 1):
        knots.append(knots[-1] + _span_deg(pts[index], pts[index + 1]))
    total = knots[-1]
    if total <= 1e-9:
        return pts
    out: List[List[float]] = [pts[0]]
    arc = 0.0
    speed = 0.0
    drop_limit = max_decel_deg * dt_s
    seg = 0
    for _ in range(20000):
        if arc >= total - 1e-9:
            break
        while seg < len(pts) - 2 and knots[seg + 1] <= arc + 1e-12:
            seg += 1
        span = knots[seg + 1] - knots[seg]
        planned = span / dt_s if span > 1e-12 else 0.0
        remaining = total - arc
        vmax = _max_stop_speed(remaining, max_decel_deg, dt_s)
        desired = min(planned, vmax) if planned > 0.0 else vmax
        speed = _clamp_signed_speed(speed, desired, drop_limit)
        step = speed * dt_s
        if step >= remaining:
            finish_speed = remaining / dt_s
            if speed - finish_speed <= drop_limit + 1e-6:
                out.append(list(pts[-1]))
                break
            speed = max(0.0, speed - drop_limit)
            step = speed * dt_s
        arc = min(total, arc + step)
        out.append(_sample_at(pts, knots, arc))
        speed = _span_deg(out[-2], out[-1]) / dt_s
    if out[-1] != pts[-1]:
        out.append(list(pts[-1]))
    return _enforce_joint_acceleration(out, dt_s, max_decel_deg)


def _enforce_joint_acceleration(points, dt_s, limit_deg):
    """Check all signed joint changes, including path corners and endpoint hold.

    The scalar arc limiter above cannot by itself bound changes in tangent at
    corners. Time dilation preserves the sampled polyline and endpoints; no
    unchecked geometric corner rounding is introduced. Fail before streaming if
    a bounded fixed-grid path cannot be produced.
    """
    result = points
    for _ in range(20):
        if _braking_ok(result, dt_s, limit_deg):
            return result
        velocities = [[0.] * 6] + [
            [(b[j] - a[j]) / dt_s for j in range(6)]
            for a, b in zip(result, result[1:])] + [[0.] * 6]
        peak = max(abs(b[j]-a[j]) / dt_s
                   for a, b in zip(velocities, velocities[1:]) for j in range(6))
        stretch = max(1.1, math.sqrt(peak / limit_deg) * 1.05)
        count = int(math.ceil((len(result)-1) * stretch))
        if count > 100000:
            raise ValueError('ServoJ acceleration-limited path exceeds sample budget')
        previous = result
        result = []
        for index in range(count + 1):
            position = index * (len(previous)-1) / count
            lo = min(int(position), len(previous)-2)
            fraction = position - lo
            result.append([a + fraction*(b-a) for a, b in zip(previous[lo], previous[lo+1])])
        result[0], result[-1] = list(previous[0]), list(previous[-1])
    raise ValueError('ServoJ joint acceleration remains above cap; refusing stream')


def servo_j_hold_count(lookahead_time: float, servo_time: float) -> int:
    """Pushes of the final joints after the path, covering lookahead."""
    if servo_time <= 0.0 or lookahead_time <= 0.0:
        return 1
    return max(1, int(math.ceil(float(lookahead_time) / float(servo_time))))
