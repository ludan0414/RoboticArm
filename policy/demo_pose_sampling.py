"""Shared recorded-pose target selection for deployment export and diagnostics."""
import numpy as np


def sample_pose_indices(poses):
    selected = [0]
    for index, pose in enumerate(poses[1:], 1):
        previous = poses[selected[-1]]
        dot = abs(float(np.dot(pose[3:], previous[3:])))
        angle = 2 * np.arccos(np.clip(dot, 0, 1))
        if np.linalg.norm(pose[:3] - previous[:3]) >= .04 or angle >= .06:
            selected.append(index)
    if not np.array_equal(poses[selected[-1]], poses[-1]):
        selected.append(len(poses)-1)
    return selected
