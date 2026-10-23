"""Hold elfin_joint6 (EOF roll) during pick.

The suction cup is coaxial with J6. Rotating that joint spins the panel
about the contact normal and does not help seal. Pick waypoints used to
bake detection PCA yaw into the tool quaternion, which commanded that
roll. The executor pins J6 on the executed joint path instead: J4/J5 may
still fold the wrist to tool-down from observe.
"""

JOINT6_NAME = "elfin_joint6"
JOINT6_INDEX = 5

#: Segment names that execute the pick approach/attach/lift. Place
#: transit/insert keep slot yaw and must not hold J6. The first carry
#: hops after pick (`place_pickup_connect_*`) are short sealed-cup
#: connections; Cartesian IK can spin J6 several tenths of a radian
#: over a few centimetres and trip PLACE_PATH_EXCURSION against the
#: 0.05 m path floor. Those hops pin J6 too.
PICK_HOLD_JOINT6_NAMES = frozenset((
    "pre_grasp",
    "approach",
    "attach",
    "pick_retreat",
    "retry_reverse",
))
_PLACE_CONNECT_PREFIX = "place_pickup_connect_"


def pick_holds_joint6(segment_name):
    name = str(segment_name)
    return name in PICK_HOLD_JOINT6_NAMES or name.startswith(_PLACE_CONNECT_PREFIX)


def joint6_value(current_joints):
    """Current J6 (rad) in elfin_joint1..6 order, or None."""
    if current_joints is None:
        return None
    if len(current_joints) <= JOINT6_INDEX:
        return None
    return float(current_joints[JOINT6_INDEX])


def joint6_index(names, width):
    """Index of elfin_joint6, or None when this vector has no J6 slot."""
    if names:
        try:
            idx = list(names).index(JOINT6_NAME)
        except ValueError:
            return None
    else:
        idx = JOINT6_INDEX
    if idx < 0 or idx >= int(width):
        return None
    return idx


def pin_joint6_positions(positions, current_j6, names=None):
    """Copy of ``positions`` with J6 replaced by ``current_j6``."""
    out = [float(v) for v in positions]
    if current_j6 is None:
        return out
    idx = joint6_index(names, len(out))
    if idx is None:
        return out
    out[idx] = float(current_j6)
    return out


def _zero_joint6_derivatives(point, idx):
    """A held joint must not keep a planned speed or acceleration."""
    for field in ("velocities", "accelerations"):
        values = getattr(point, field, None)
        if not values or idx >= len(values):
            continue
        updated = [float(v) for v in values]
        updated[idx] = 0.0
        setattr(point, field, updated)


def pin_joint6_trajectory(joint_trajectory, current_j6):
    """Overwrite J6 on every knot. Mutates ``joint_trajectory`` in place."""
    if current_j6 is None or joint_trajectory is None:
        return joint_trajectory
    names = list(getattr(joint_trajectory, "joint_names", None) or [])
    points = getattr(joint_trajectory, "points", None) or []
    for point in points:
        positions = list(getattr(point, "positions", None) or [])
        idx = joint6_index(names, len(positions))
        if idx is None:
            continue
        point.positions = pin_joint6_positions(
            positions, current_j6, names)
        _zero_joint6_derivatives(point, idx)
    return joint_trajectory
