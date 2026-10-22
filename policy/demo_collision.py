"""Vectorized Newton effective-surface gaps; no simulator imports."""
import numpy as np


def surface_gaps(body_q, body0, body1, point0, point1, normal, margin0, margin1):
    def world_points(indices, points):
        output = np.asarray(points, dtype=float).copy()
        moving = indices >= 0
        if moving.any():
            transforms = body_q[indices[moving]]
            q = transforms[:, 3:]
            q = q / np.linalg.norm(q, axis=1, keepdims=True)
            v = output[moving]
            output[moving] = transforms[:, :3] + v + 2 * np.cross(
                q[:, :3], np.cross(q[:, :3], v) + q[:, 3:] * v)
        return output

    first = world_points(np.asarray(body0), point0)
    second = world_points(np.asarray(body1), point1)
    # Same formula as Newton sim.contacts.contact_surface_separation.
    return np.sum((second - first) * normal, axis=1) - margin0 - margin1
