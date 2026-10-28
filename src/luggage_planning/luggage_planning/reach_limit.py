"""Shorten an unreachable carry waypoint along its requested travel direction.

Only numerical probes occur here. The accepted endpoint still needs normal
trajectory planning/execution; contact and recorded-return goals stay exact.
"""
from copy import deepcopy
import math


def is_transit_goal(name):
    return str(name).startswith(('place_via_', 'place_pickup_connect_', 'ground_entry_')) or name == 'pre_place'


def closest_transit_pose(start, target, solve, notify=lambda: None):
    a = tuple(getattr(start.position, key) for key in ('x', 'y', 'z'))
    b = tuple(getattr(target.position, key) for key in ('x', 'y', 'z'))
    length = math.sqrt(sum((v-u)**2 for u,v in zip(a,b)))
    if not math.isfinite(length) or length < .001:
        return None
    # Search from the desired pose toward the measured current TCP. Unlike
    # accepting a partial Cartesian prefix, every probe holds target attitude
    # and has its own complete IK solution. Refine the outer feasible bracket.
    high = 1.
    for fraction in (.98, .95, .9, .8, .65, .5, .25, 0.):
        probe = deepcopy(target)
        for key,u,v in zip(('x','y','z'),a,b):
            setattr(probe.position,key,u+fraction*(v-u))
        notify()
        joints = solve(probe)
        if joints is None:
            high = fraction
            continue
        low, best = fraction, (probe,joints)
        while (high-low)*length > .005:
            mid = (low+high)/2
            probe = deepcopy(target)
            for key,u,v in zip(('x','y','z'),a,b):
                setattr(probe.position,key,u+mid*(v-u))
            notify()
            joints = solve(probe)
            if joints is None:
                high = mid
            else:
                low,best = mid,(probe,joints)
        return dict(pose=best[0], joints=best[1], shortfall_m=(1-low)*length)
    return None


def pose_dict(pose):
    return dict(position={k:float(getattr(pose.position,k)) for k in ('x','y','z')},
                orientation={k:float(getattr(pose.orientation,k)) for k in ('x','y','z','w')})
