"""ROS-free joint travel diagnostics; thresholds may be advisory."""
_CARTESIAN_EXCURSION_RAD_PER_M = 3.0
_CARTESIAN_EXCURSION_MIN_PATH_M = .05
JOINTS = ['elfin_joint%d' % i for i in range(1, 7)]


def cartesian_excursion(joint_names, points, path_len_m,
                        limit_rad_per_m=_CARTESIAN_EXCURSION_RAD_PER_M,
                        min_path_m=_CARTESIAN_EXCURSION_MIN_PATH_M):
    rows = [list(getattr(p, 'positions', p)) for p in (points or [])]
    rows = [r for r in rows if r]
    if len(rows) < 2 or path_len_m is None:
        return None
    width = min(len(r) for r in rows)
    if width <= 0:
        return None
    names = list(joint_names or JOINTS)
    totals = [0.0] * width
    for before, after in zip(rows, rows[1:]):
        for index in range(width):
            totals[index] += abs(float(after[index]) - float(before[index]))
    worst = max(range(width), key=lambda index: totals[index])
    denominator = max(float(path_len_m), float(min_path_m))
    ratio = totals[worst] / denominator if denominator > 0 else float('inf')
    return dict(joint=names[worst] if worst < len(names) else 'joint%d' % worst,
                excursion_rad=round(totals[worst], 4), path_len_m=round(float(path_len_m), 4),
                rad_per_m=round(ratio, 3), limit_rad_per_m=float(limit_rad_per_m),
                ok=ratio <= float(limit_rad_per_m))


def excursion_diagnostic(joint_names, points, path_len_m, configured_limit):
    """Zero disables rejection, while retaining the original warning metric."""
    measured = cartesian_excursion(joint_names, points, path_len_m,
        limit_rad_per_m=configured_limit if configured_limit > 0 else _CARTESIAN_EXCURSION_RAD_PER_M)
    if measured is not None:
        measured['enforced'] = configured_limit > 0
        measured['reject'] = measured['enforced'] and not measured['ok']
    return measured
