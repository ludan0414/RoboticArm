"""Find the outermost IK-feasible point on a requested shoulder ray.

The caller supplies the real IK/collision predicate. A projected point is a
changed transit goal, never evidence that the original contact was reached.
"""
import math


def furthest_reachable(origin, target, solve, *, step_m=.02, precision_m=.001,
                       minimum_fraction=0.):
    delta = tuple(b-a for a, b in zip(origin, target))
    distance = math.sqrt(sum(v*v for v in delta))
    if len(delta) != 3 or not math.isfinite(distance) or distance <= precision_m:
        return None
    count = max(1, math.ceil(distance*(1-minimum_fraction)/step_m))
    failed = 1.
    for index in range(count+1):
        fraction = 1-index*(1-minimum_fraction)/count
        point = tuple(a+fraction*d for a, d in zip(origin, delta))
        solution = solve(point)
        if solution is None:
            failed = fraction
            continue
        # Search only the bracket next to the first feasible outer sample;
        # do not require all inner points to be feasible (folded-arm workspace).
        low, high = fraction, failed
        while (high-low)*distance > precision_m:
            mid = (low+high)/2
            candidate = tuple(a+mid*d for a, d in zip(origin, delta))
            answer = solve(candidate)
            if answer is None:
                high = mid
            else:
                low, solution, point = mid, answer, candidate
        return dict(point=point, solution=solution,
                    shortfall_m=(1-low)*distance, fraction=low)
    return None
