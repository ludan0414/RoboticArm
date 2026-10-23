"""Resolve the operator's return offset and camera heading using measured TF."""
from .pose import Point, Pose, Quaternion
from .policy_place import _vector, _quaternion
from .suction_candidate_waypoints import quat_conjugate, quat_mul, quat_rotate

DEFAULT_OFFSET_BASE_M = (.8, 0., -.2)
DEFAULT_CAMERA_FRAME = "d555_color_optical_frame"
# With the current optical-axis heading, closer low targets intersect the
# pedestal. This point has an offline collision-free IK branch; live planning
# remains mandatory for the current scene and the complete return path.
DEFAULT_TARGET_BASE_M = (-1.1, 0., -.2)


def adjusted_return_pose(nominal_world, world_from_base, tool_from_camera,
                         offset_base=DEFAULT_OFFSET_BASE_M, target_base=None):
    """Translate in base axes; optical +Z points exactly along base -X.

    Optical +Y (image down) points along base -Z, keeping the image upright.
    R_base_tool = R_base_camera * inverse(R_tool_camera). This is an empty-tool
    return pose; keeping the suction panel level would not aim this mount at -X.
    """
    offset = _vector(offset_base, 3, "return offset in base")
    base_q = _quaternion([getattr(world_from_base.orientation, k) for k in ("x", "y", "z", "w")])
    camera_q = _quaternion([getattr(tool_from_camera.orientation, k) for k in ("x", "y", "z", "w")])
    nominal = _vector([getattr(nominal_world.position, k) for k in ("x", "y", "z")], 3, "nominal return")
    base_p = _vector([getattr(world_from_base.position, k) for k in ("x", "y", "z")], 3, "base translation")
    local = quat_rotate(quat_conjugate(base_q), tuple(a-b for a,b in zip(nominal, base_p)))
    local_target = (_vector(target_base, 3, 'return target in base') if target_base is not None
                    else tuple(a+b for a,b in zip(local, offset)))
    translated = quat_rotate(base_q, local_target)
    # Rotation columns: camera X=base Y, camera Y=-base Z, camera Z=-base X.
    base_camera = Quaternion(-.5, -.5, .5, .5)
    base_tool = quat_mul(base_camera, quat_conjugate(camera_q))
    return Pose(Point(*(a+b for a,b in zip(base_p, translated))),
                quat_mul(base_q, base_tool))
