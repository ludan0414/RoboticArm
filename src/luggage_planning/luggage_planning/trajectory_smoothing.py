"""Bound cubic Hermite dynamics without discarding geometric knots.

Choose the faster of uniform dilation and local monotone interpolation. Local
timing slows corners without stretching the entire carry route. Both candidates
have continuous velocities and analytically bounded Hermite extrema.
"""
from dataclasses import replace
import math


def validate_speed(scale):
    if type(scale) is not float or not math.isfinite(scale) or not .1 <= scale <= 1.:
        raise ValueError('speed_scale must be a double within 0.1..1.0')
    return scale


def hermite_peaks(positions, timed):
    peak_v = peak_a = 0.
    for i, (q0, q1) in enumerate(zip(positions, positions[1:])):
        dt = timed.times_s[i+1] - timed.times_s[i]
        if dt <= 0:
            raise ValueError('trajectory times must increase')
        for a, b, v0, v1 in zip(q0, q1, timed.velocities_rad[i], timed.velocities_rad[i+1]):
            # v(u) = A*u²+B*u+C, acceleration = (2*A*u+B)/dt.
            A = 6*(a-b)/dt + 3*(v0+v1)
            B = 6*(b-a)/dt - 4*v0 - 2*v1
            peak_a = max(peak_a, abs(B)/dt, abs(2*A+B)/dt)
            peak_v = max(peak_v, abs(v0), abs(v1))
            if abs(A) > 1e-15:
                u = -B/(2*A)
                if 0 < u < 1:
                    peak_v = max(peak_v, abs(A*u*u+B*u+v0))
    return peak_v, peak_a


def smooth_timing(positions, timed, speed_scale=1.):
    validate_speed(speed_scale)
    if not timed.times_s:
        return timed
    v_peak, a_peak = hermite_peaks(positions, timed)
    # Keep the uniform solution as a bounded fallback for short paths and when
    # local iteration does not converge. Local interpolation never overshoots
    # a joint's interval endpoints, but may change its curve between knots.
    stretch = max(1., v_peak/timed.v_max, math.sqrt(a_peak/min(timed.a_max, timed.a_decel)))
    uniform = _dilate(timed, stretch)
    local = _local_timing(positions, timed)
    chosen = local if local is not None and local.duration < uniform.duration else uniform
    # Numerical headroom prevents executor-side dilation on the 20ms grid.
    return _dilate(chosen, 1.01 / speed_scale)


def _dilate(timed, stretch):
    return replace(timed, times_s=[t*stretch for t in timed.times_s],
                   velocities_rad=[[v/stretch for v in row] for row in timed.velocities_rad],
                   accelerations_rad=[[a/stretch**2 for a in row] for row in timed.accelerations_rad],
                   duration=timed.duration*stretch, v_peak=timed.v_peak/stretch)


def _monotone_velocities(positions, spans):
    """Weighted harmonic slopes: no joint overshoot, rest at both endpoints."""
    velocities = [[0.] * len(positions[0])]
    for i in range(1, len(positions)-1):
        before, after = spans[i-1], spans[i]
        w_before, w_after = 2*after+before, after+2*before
        row = []
        for a, b, c in zip(positions[i-1], positions[i], positions[i+1]):
            left, right = (b-a)/before, (c-b)/after
            row.append(0. if left*right <= 0. else
                       (w_before+w_after)/(w_before/left+w_after/right))
        velocities.append(row)
    velocities.append([0.] * len(positions[0]))
    return velocities


def _local_timing(positions, timed):
    if len(positions) < 3:
        return None
    spans = [b-a for a, b in zip(timed.times_s, timed.times_s[1:])]
    acceleration_limit = min(timed.a_max, timed.a_decel)
    # Each update only lengthens violating spans; neighbouring slopes are then
    # recomputed. A bounded iteration budget cannot leave a partial result.
    for _ in range(100):
        velocities = _monotone_velocities(positions, spans)
        factors = []
        for i, duration in enumerate(spans):
            interval = replace(timed, times_s=[0., duration],
                               velocities_rad=velocities[i:i+2])
            v_peak, a_peak = hermite_peaks(positions[i:i+2], interval)
            factors.append(max(1., v_peak/timed.v_max,
                               math.sqrt(a_peak/acceleration_limit)))
        if max(factors) <= 1.+1e-8:
            times = [0.]
            for duration in spans:
                times.append(times[-1]+duration)
            result = replace(timed, times_s=times, duration=times[-1],
                             velocities_rad=velocities)
            # Feed-forward acceleration at an internal knot averages the
            # one-sided cubic accelerations; endpoints use their sole side.
            left, right = [], []
            for i, duration in enumerate(spans):
                delta = [(b-a)/duration for a, b in zip(positions[i], positions[i+1])]
                left.append([(6*d-4*v0-2*v1)/duration
                             for d, v0, v1 in zip(delta, velocities[i], velocities[i+1])])
                right.append([(-6*d+2*v0+4*v1)/duration
                              for d, v0, v1 in zip(delta, velocities[i], velocities[i+1])])
            accelerations = [left[0]] + [[(a+b)/2 for a, b in zip(right[i-1], left[i])]
                                         for i in range(1, len(positions)-1)] + [right[-1]]
            return replace(result, accelerations_rad=accelerations,
                           v_peak=hermite_peaks(positions, result)[0])
        spans = [duration*factor*1.001 if factor > 1.+1e-8 else duration
                 for duration, factor in zip(spans, factors)]
    return None


def execution_timeout(trajectory, configured):
    """A deliberately slow complete route must not hit the old per-hop timeout."""
    joint = getattr(trajectory, 'joint_trajectory', None)
    if joint is None or not joint.points:
        return configured
    end = joint.points[-1].time_from_start
    duration = float(end.sec) + float(end.nanosec)*1e-9
    return max(configured, duration*1.5 + 10.)
