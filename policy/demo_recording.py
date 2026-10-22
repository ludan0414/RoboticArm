"""Validate the full scene recording: ordered spawn and persistent placements."""
import numpy as np


def validate_recording(sequence, data):
    stages, ids = data["stage"], data["box_id"]
    body_q, labels = data["body_q"], list(data["body_labels"])
    if not np.isfinite(body_q).all() or len(body_q) != len(stages):
        raise ValueError("invalid scene recording")
    spawns = [str(ids[i]) for i in range(len(stages)) if stages[i] == "SPAWN"
              and (i == 0 or stages[i - 1] != "SPAWN" or ids[i - 1] != ids[i])]
    if spawns != [row["id"] for row in sequence]:
        raise ValueError("spawn order does not match sequence")
    results = []
    for row in sequence:
        name = row["id"]
        body = labels.index(name)
        spawn = int(np.flatnonzero((stages == "SPAWN") & (ids == name))[0])
        released = int(np.flatnonzero((stages == "RELEASE") & (ids == name))[0])
        poses = body_q[:, body]
        if "spawn_position_world_m" in row:
            expected = np.asarray(row["spawn_position_world_m"])
            axes = slice(0, 2) if "spawn_bottom_world_z_m" in row else slice(0, 3)
            if not np.allclose(expected[axes], np.asarray(sequence[0]["spawn_position_world_m"])[axes], atol=1e-5, rtol=0):
                raise ValueError(name + " planned feeder is not fixed")
            if "spawn_bottom_world_z_m" in row and abs(
                    poses[spawn, 2] - row["placed_dims_m"][2] / 2 - row["spawn_bottom_world_z_m"]) > 1e-5:
                raise ValueError(name + " spawn is not on specified ground")
            if not np.allclose(poses[spawn, :3], expected, atol=1e-5, rtol=0):
                raise ValueError(name + " spawn differs from fixed feeder")
        if not np.allclose(poses[:spawn, :3], 100.0):
            raise ValueError(name + " appeared before its spawn")
        if not np.allclose(poses[released:], poses[released], atol=1e-6):
            raise ValueError(name + " moved after release")
        error = float(np.linalg.norm(poses[-1, :3] - row["position_world_m"]))
        actual, target = poses[-1, 3:], np.asarray(row["orientation_world_xyzw"])
        dot = abs(float(np.dot(actual / np.linalg.norm(actual), target / np.linalg.norm(target))))
        angle = float(2 * np.arccos(np.clip(dot, 0, 1)))
        if error > .006 or angle > .07:
            raise ValueError(name + " final pose differs from ordered target")
        results.append(dict(id=name, position_error_m=error, orientation_error_rad=angle,
                            spawn_position_world_m=poses[spawn, :3].tolist(),
                            spawn_frame=spawn, release_frame=released))
    return dict(success=True, boxes=len(sequence), frames=len(stages),
                spawn_order=spawns, final_poses=results)
