"""Rigid scene transforms shared by packing, Newton IK and rendering."""
import numpy as np
from scipy.spatial.transform import Rotation


def rotation(pose):
    return Rotation.from_euler("xyz", pose.get("rotation_rpy", [0, 0, 0]))


def transform_point(pose, point):
    return np.asarray(pose["translation"], dtype=float) + rotation(pose).apply(point)


def container_pose(config):
    return next(item for item in config["static_transforms"] if item["child"] == "container_link")


def container_tool_orientation(config, world_yaw):
    pose = container_pose(config)
    local_yaw = float(world_yaw) - float(pose["rotation_rpy"][2])
    return (rotation(pose) * Rotation.from_euler("z", local_yaw)
            * Rotation.from_euler("x", np.pi)).as_quat().astype(np.float32)
