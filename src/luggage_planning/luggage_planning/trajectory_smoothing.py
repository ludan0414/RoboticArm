"""Bound cubic Hermite dynamics without changing the interpolated joint curve.

Uniform time dilation keeps the already chosen knot positions and tangents.
Internal velocities stay continuous; only the action endpoints are at rest.
The extrema below are analytic bounds for the interpolation used by ServoJ.
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
    # Small numerical headroom also prevents repeated executor-side dilation.
    stretch = max(1., v_peak/timed.v_max, math.sqrt(a_peak/min(timed.a_max, timed.a_decel)))
    stretch = stretch * 1.01 / speed_scale
    return replace(timed, times_s=[t*stretch for t in timed.times_s],
        velocities_rad=[[v/stretch for v in row] for row in timed.velocities_rad],
        accelerations_rad=[[a/stretch**2 for a in row] for row in timed.accelerations_rad],
        duration=timed.duration*stretch, v_peak=timed.v_peak/stretch)


def execution_timeout(trajectory, configured):
    """A deliberately slow complete route must not hit the old per-hop timeout."""
    joint = getattr(trajectory, 'joint_trajectory', None)
    if joint is None or not joint.points:
        return configured
    end = joint.points[-1].time_from_start
    duration = float(end.sec) + float(end.nanosec)*1e-9
    return max(configured, duration*1.5 + 10.)
