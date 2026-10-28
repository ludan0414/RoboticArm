"""Greedy ordering and placement over an already explored finite domain."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scene_frames import rotation, transform_point
from online_packing_demo import Box, collides, fully_supported
from demo_feeder import FEEDER_CENTER_M, feeder_route


def greedy_sequence(boxes, hull, domain, workspace, progress=print,
                    feeder_center=FEEDER_CENTER_M, allow_partial=False):
    remaining = list(range(len(boxes)))
    placed, sequence = [], []
    depth_frontier = float("inf")
    while remaining:
        candidates = []
        # Boxes with identical dimensions have identical geometry and route
        # feasibility; retain the first remaining input index for stable IDs.
        seen_dimensions = set()
        for index in remaining:
            source = boxes[index]
            signature = tuple(source)
            if signature in seen_dimensions:
                continue
            seen_dimensions.add(signature)
            for certificate, point in enumerate(domain["poses"]):
                yaw = point["box_yaw_rad"]
                dims = tuple(source if abs(yaw) < 1e-6 else (source[1], source[0], source[2]))
                cx, cy, top = point["contact_local_m"]
                if cx > depth_frontier + 1e-6:
                    continue  # Never return behind an already advanced front.
                pos = (round(cx - dims[0] / 2, 6), round(cy - dims[1] / 2, 6), round(top - dims[2], 6))
                box = Box(f"B{index + 1}", dims, pos)
                if (not hull.fits(box) or any(collides(box, other) for other in placed)
                        or not (hull.floor_supports(box) or fully_supported(box, placed))):
                    continue
                candidates.append((box, point, certificate, index))
        # Compare all remaining boxes, so a shallow large box cannot take
        # priority over a smaller box that can still fill the deep section.
        candidates.sort(key=lambda item: (-item[1]["contact_local_m"][0], item[0].pos[2],
                         -item[0].dims[0] * item[0].dims[1], -float(np.prod(item[0].dims)),
                         item[3], item[0].pos[1]))
        selected = None
        for box, point, certificate, index in candidates:
            workspace.q_last = np.asarray(point["joint_q"], dtype=np.float32)
            route = feeder_route(box, placed, hull, workspace, boxes[index], feeder_center)
            if route is not None:
                selected = box, point, certificate, route, index
                break
        if selected is None:
            if allow_partial and sequence:
                progress(f"GREEDY_PARTIAL: {len(placed)}/{len(boxes)}; no further candidate on this grid")
                break
            raise RuntimeError(f"GREEDY_BLOCKED: {len(placed)}/{len(boxes)} placed; domain candidates={len(candidates)}")
        box, point, certificate, route, index = selected
        feeder_joint_q = np.asarray(workspace.q_last, dtype=float).tolist()
        source = boxes[index]
        depth_frontier = point["contact_local_m"][0]
        remaining.remove(index)
        yaw = point["box_yaw_rad"]
        world_center = list(point["contact_world_m"])
        container_tf = next(t for t in workspace.config["static_transforms"]
                            if t["child"] == "container_link")
        container_rot = rotation(container_tf)
        world_center = (np.asarray(world_center) - container_rot.apply([0, 0, float(box.dims[2]) / 2])).tolist()
        container_yaw = next(t for t in workspace.config["static_transforms"]
                             if t["child"] == "container_link")["rotation_rpy"][2]
        center_local = [box.pos[k] + box.dims[k] / 2 for k in range(3)]
        spawn_world = (np.asarray(world_center) + container_rot.apply(
            np.asarray(feeder_center) - center_local)).tolist()
        sequence.append(dict(step=len(sequence) + 1, input_index=index, id=box.id,
                             input_dims_m=source, placed_dims_m=list(box.dims),
                             spawn_center_container_door_floor_m=list(feeder_center),
                             spawn_position_world_m=spawn_world,
                             center_container_door_floor_m=center_local,
                             orientation=dict(yaw_deg=math.degrees(yaw),
                                 quaternion_xyzw=[0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]),
                             position_world_m=world_center,
                             orientation_world_xyzw=(container_rot * Rotation.from_euler("z", yaw)).as_quat().tolist(),
                             domain_certificate=certificate, feeder_joint_q=feeder_joint_q, route=route))
        placed.append(box)
        progress(f"GREEDY_PLAN {len(sequence)}/{len(boxes)}: {box.id} -> {world_center}, yaw={math.degrees(yaw):.0f}")
    return sequence
