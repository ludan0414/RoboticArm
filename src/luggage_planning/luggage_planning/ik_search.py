"""Nearby numerical IK starts, never intermediate commanded robot states."""
import math

IK_ATTEMPT_TIMEOUT_SEC = 2


def nearby_ik_seeds(current):
    if not current or len(current)!=6 or any(not math.isfinite(v) for v in current):
        return [None]
    seeds = [list(current)]
    # Perturb the shoulder/elbow sum together while retaining all wrist
    # branches and base winding. These are solver starts, not motion goals.
    for sign in (1., -1.):
        candidate = list(current)
        candidate[1] += sign*.06
        candidate[2] -= sign*.12
        candidate[3] += sign*.06
        seeds.append(candidate)
    return seeds
