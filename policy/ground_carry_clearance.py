"""Conservative payload clearance from the fixed pedestal, independent of IK."""
import numpy as np
from scipy.spatial.transform import Rotation


def pedestal_geometry(config):
    pedestal = config["pedestal"]
    size = np.asarray(pedestal["size"], dtype=float)
    center = np.asarray(pedestal["translation"], dtype=float) + [0, 0, size[2] / 2]
    # Match the pedestal geometry built by roboticarm_newton.add_cell_scene.
    rotation = Rotation.from_euler("z", pedestal["rotation_rpy"][2]).as_matrix()
    return center, size / 2, rotation


def box_pedestal_separated(box_pose, box_size, pedestal, margin_m=.05):
    """15-axis OBB separation after inflating the pedestal by margin per face."""
    center, half, rotation = pedestal
    box_rotation = Rotation.from_quat(box_pose[3:]).as_matrix()
    box_half = np.asarray(box_size) / 2
    delta = np.asarray(box_pose[:3]) - center
    axes = [*rotation.T, *box_rotation.T]
    axes += [np.cross(a, b) for a in rotation.T for b in box_rotation.T]
    for axis in axes:
        if np.linalg.norm(axis) < 1e-8:
            continue
        span = (np.abs(rotation.T @ axis) @ (half + margin_m)
                + np.abs(box_rotation.T @ axis) @ box_half)
        if abs(delta @ axis) > span:
            return True
    return False


def box_bottom(box_pose, box_size):
    rotation = Rotation.from_quat(box_pose[3:]).as_matrix()
    return float(box_pose[2] - np.abs(rotation[2]) @ (np.asarray(box_size) / 2))


def outside_staging_xy(feeder_xy, pedestal, box_size):
    """Retract an extended arm outside the pedestal before the high lift.

    A conservative radial support bound includes the payload's footprint
    circumradius and 15cm extra space. Actual path samples still need checks.
    """
    center, half, rotation = pedestal
    delta = np.asarray(feeder_xy) - center[:2]
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        raise ValueError("pickup lies at pedestal center")
    direction = delta / distance
    support = float(np.abs(rotation.T @ np.r_[direction, 0]) @ half)
    outside_radius = support + float(np.linalg.norm(np.asarray(box_size[:2]) / 2)) + .15
    if distance < outside_radius:
        raise ValueError("pickup lacks outside staging clearance")
    radius = min(distance, max(1.25, outside_radius))
    return center[:2] + direction * radius
