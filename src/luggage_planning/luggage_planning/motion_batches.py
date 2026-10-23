"""Reduce settled action boundaries while retaining Cartesian route geometry."""
from dataclasses import replace

GROUND_TURN_STOPS = frozenset(('ground_entry_outer_lift', 'ground_entry_outside_retract',
                              'ground_entry_high_lift'))


def stop_indices(count, max_stops, required=()):
    """Keep mandatory corners; subdivide the longest remaining spans."""
    if count == 0:
        return []
    ends = set(required) | {count-1}
    budget = min(count,max_stops)
    if len(ends) > budget:
        raise ValueError('stop budget cannot preserve mandatory corners')
    while len(ends) < budget:
        boundaries = [-1,*sorted(ends)]
        left,right = max(zip(boundaries,boundaries[1:]),key=lambda pair:pair[1]-pair[0])
        ends.add((left+right)//2)
    return sorted(ends)


def batch_cartesian_segments(segments, max_stops=8):
    segments = list(segments)
    if max_stops < 1:
        raise ValueError('max_stops must be positive')
    if len(segments) <= max_stops:
        return segments
    if any(segment.type != 'cartesian' for segment in segments):
        raise ValueError('only Cartesian transit segments may share an action')
    result = []
    start = 0
    required = [i for i,s in enumerate(segments) if s.name in GROUND_TURN_STOPS]
    for end in stop_indices(len(segments),max_stops,required):
        chunk = segments[start:end+1]
        start = end+1
        poses = [pose for segment in chunk
                 for pose in [*segment.waypoints, segment.target_pose]]
        # OMPL could bypass intermediate poses; a batch must follow every via.
        result.append(replace(chunk[-1], waypoints=poses[:-1],
                              required_cartesian_fraction=1.0, allow_ompl_fallback=False))
    return result
