"""Greedy ordering and placement over an already explored finite domain."""
import math
import numpy as np
from online_packing_demo import Box, collides, fully_supported
from demo_feeder import FEEDER_CENTER_M, feeder_route


def greedy_sequence(boxes, hull, domain, workspace, progress=print):
    remaining = list(range(len(boxes)))
    placed, sequence = [], []
    depth_frontier = float("inf")
    while remaining:
        candidates = []
        for index in remaining:
            source = boxes[index]
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
            route = feeder_route(box, placed, hull, workspace, boxes[index])
            if route is not None:
                selected = box, point, certificate, route, index
                break
        if selected is None:
            raise RuntimeError(f"GREEDY_BLOCKED: {len(placed)}/{len(boxes)} placed; domain candidates={len(candidates)}")
        box, point, certificate, route, index = selected
        source = boxes[index]
        depth_frontier = point["contact_local_m"][0]
        remaining.remove(index)
        yaw = point["box_yaw_rad"]
        world_center = list(point["contact_world_m"])
        world_center[2] -= float(box.dims[2]) / 2
        container_yaw = next(t for t in workspace.config["static_transforms"]
                             if t["child"] == "container_link")["rotation_rpy"][2]
        center_local = [box.pos[k] + box.dims[k] / 2 for k in range(3)]
        offset = np.asarray(FEEDER_CENTER_M) - center_local
        c, s = math.cos(container_yaw), math.sin(container_yaw)
        spawn_world = (np.asarray(world_center) + [c * offset[0] - s * offset[1],
                       s * offset[0] + c * offset[1], offset[2]]).tolist()
        sequence.append(dict(step=len(sequence) + 1, input_index=index, id=box.id,
                             input_dims_m=source, placed_dims_m=list(box.dims),
                             spawn_center_container_door_floor_m=list(FEEDER_CENTER_M),
                             spawn_position_world_m=spawn_world,
                             center_container_door_floor_m=center_local,
                             orientation=dict(yaw_deg=math.degrees(yaw),
                                 quaternion_xyzw=[0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]),
                             position_world_m=world_center,
                             orientation_world_xyzw=[0, 0, math.sin((yaw + container_yaw) / 2),
                                                     math.cos((yaw + container_yaw) / 2)],
                             domain_certificate=certificate, route=route))
        placed.append(box)
        progress(f"GREEDY_PLAN {len(sequence)}/10: {box.id} -> {world_center}, yaw={math.degrees(yaw):.0f}")
    return sequence
