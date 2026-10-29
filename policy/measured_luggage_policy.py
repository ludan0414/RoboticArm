"""Dimension-aware, four-layer candidate policy for the measured luggage set."""
import math
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scene_frames import rotation
from online_packing_demo import Box, collides, fully_supported, overlap
from demo_feeder import feeder_route


def candidate_slots(source, placed, levels, hull, frontier, extra_ys=()):
    xs = (.20, .30, .45, .60, .75)
    base_ys = (.28, .40, .53, .65, .78, .90, 1.03, 1.15, 1.28)
    heights = sorted({0.0} | {round(b.pos[2]+b.dims[2], 6) for b in placed})
    for yaw in (0.0, -math.pi/2):
        dims = tuple(source if yaw == 0.0 else (source[1], source[0], source[2]))
        # Include the exact flat-floor edge and centres of existing supports;
        # a fixed 12 cm grid can miss the only remaining third-stack slot.
        ys = sorted(set(base_ys + tuple(extra_ys) + (round(hull.flat_y-dims[1]/2, 6),)
                        + tuple(round(b.pos[1]+b.dims[1]/2, 6) for b in placed)))
        for x in xs:
            if x > frontier + 1e-7:
                continue
            for y in ys:
                for z in heights:
                    box=Box('candidate',dims,(round(x-dims[0]/2,6), round(y-dims[1]/2,6), z))
                    if not hull.fits(box) or any(collides(box,other) for other in placed):
                        continue
                    if z == 0:
                        if not hull.floor_supports(box):
                            continue
                        level = 1
                    else:
                        supports = [i for i,b in enumerate(placed)
                                    if abs(b.pos[2]+b.dims[2]-z)<1e-6
                                    and overlap(box.pos[0], box.pos[0]+dims[0], b.pos[0],b.pos[0]+b.dims[0])
                                    and overlap(box.pos[1], box.pos[1]+dims[1], b.pos[1],b.pos[1]+b.dims[1])]
                        if not supports or not fully_supported(box,placed):
                            continue
                        level = max(levels[i] for i in supports) + 1
                    if level <= 4:
                        yield box, x, y, yaw, level


def plan_measured(inputs, hull, workspace, output, write_json, target=12, prior_witnesses=(),
                  max_layers=4, side_first=False, route_check=None):
    boxes=inputs['boxes']; labels=inputs['box_labels']
    remaining=list(range(len(boxes))); placed=[]; levels=[]; rows=[]; poses=[]
    rejected=[]; frontier=float('inf'); feeder=(-.43,.5,.55)
    prior_by_yaw = {yaw: [p for p in prior_witnesses if abs(p['box_yaw_rad']-yaw)<1e-6]
                    for yaw in (0.0, -math.pi/2)}
    seeded_cache = {}
    while remaining and len(rows)<target:
        candidates=[]
        for i in remaining:
            for box,x,y,yaw,level in candidate_slots(boxes[i],placed,levels,hull,frontier):
                if level > max_layers:
                    continue
                candidates.append((box,x,y,yaw,level,i))
        # Prefer a supported higher layer, while preserving the monotonic
        # back-to-front frontier and using large floor boxes as supports.
        candidates.sort(key=lambda v:((int(hull.flat_y/3 < v[2] < 2*hull.flat_y/3) if side_first else 0),-v[4], abs(v[1]-.60),
            min(abs(v[2]-.53),abs(v[2]-1.28)),
            -v[0].dims[0]*v[0].dims[1],v[5],v[2]))
        selected=None
        print('PLAN_STEP %d remaining=%d geometry_candidates=%d' % (len(rows)+1,len(remaining),len(candidates)),flush=True)
        for candidate_index, (box,x,y,yaw,level,i) in enumerate(candidates):
            if candidate_index and candidate_index % 100 == 0:
                write_json(output/'search_partial.json', dict(sequence=rows, rejected=rejected, remaining=remaining))
                print('CANDIDATES %d/%d rejected=%d' % (candidate_index,len(candidates),len(rejected)),flush=True)
            local=[x,y,round(box.pos[2]+box.dims[2],6)]
            boundary = dict(step=len(rows)+1, candidate=candidate_index, input_index=i,
                            label=labels[i], contact_local_m=local, yaw_rad=yaw,
                            dimensions=list(box.dims), level=level)
            with (output/'candidate_boundary.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(dict(boundary, event='candidate_start'))+'\n')
                log.flush()
            world, world_yaw=workspace.local_contact_world(local,(0,0,0),yaw,hull)
            key = tuple(local) + (yaw,)
            if key in seeded_cache:
                ok, q, diagnostics, collisions = seeded_cache[key]
                workspace.last_diagnostics = dict(diagnostics)
                workspace.last_collision_pairs = list(collisions)
                if ok:
                    workspace.q_last = q.copy()
            else:
                ok, q = False, None
                witnesses=prior_by_yaw[yaw]
                if witnesses:
                    closest=min(witnesses,key=lambda p:sum((p['contact_local_m'][k]-local[k])**2 for k in range(3)))
                    seed=np.asarray(closest['joint_q'],dtype=np.float32)
                    # The prior is only a numerical seed: the current solver,
                    # FK, limits and current-scene collision gate run again.
                    ok,q=workspace._solve_once(world,workspace.tool_orientation(world_yaw),seed,True)
                    if ok:
                        workspace.q_last=q.copy()
                if not ok:
                    ok,q=workspace.solve_world_pose(world,world_yaw)
                seeded_cache[key]=(ok, None if q is None else q.copy(),
                                   dict(workspace.last_diagnostics), list(workspace.last_collision_pairs))
            if not ok:
                rejected.append(dict(input_index=i,label=labels[i],contact_local_m=local,yaw_rad=yaw,
                                     phase='endpoint_ik',diagnostics=dict(workspace.last_diagnostics),
                                     collisions=workspace.last_collision_pairs))
                with (output/'candidate_boundary.jsonl').open('a', encoding='utf-8') as log:
                    log.write(json.dumps(dict(boundary, event='endpoint_rejected', diagnostics=workspace.last_diagnostics))+'\n')
                continue
            route=(feeder_route(box,placed,hull,workspace,boxes[i],feeder)
                   if route_check is None else
                   ([] if route_check(i,box,x,y,yaw,placed,q) else None))
            if route is None:
                rejected.append(dict(input_index=i,label=labels[i],contact_local_m=local,yaw_rad=yaw,phase='route'))
                with (output/'candidate_boundary.jsonl').open('a', encoding='utf-8') as log:
                    log.write(json.dumps(dict(boundary, event='route_rejected'))+'\n')
                continue
            with (output/'candidate_boundary.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(dict(boundary, event='candidate_selected'))+'\n')
            selected=(box,x,y,yaw,level,i,world,world_yaw,q,route)
            break
        if selected is None:
            break
        box,x,y,yaw,level,i,world,world_yaw,q,route=selected
        box=Box('B%d'%(i+1),box.dims,box.pos)
        container=next(t for t in workspace.config['static_transforms'] if t['child']=='container_link')
        rot=rotation(container); center_local=[box.pos[k]+box.dims[k]/2 for k in range(3)]
        center=(np.asarray(world)-rot.apply([0,0,box.dims[2]/2])).tolist()
        spawn=(np.asarray(center)+rot.apply(np.asarray(feeder)-center_local)).tolist()
        cert=len(poses); poses.append(dict(contact_local_m=[x,y,box.pos[2]+box.dims[2]], contact_world_m=world.tolist(),box_yaw_rad=yaw,joint_q=q.tolist()))
        row=dict(step=len(rows)+1,input_index=i,id=box.id,label=labels[i],input_dims_m=boxes[i],placed_dims_m=list(box.dims),
                 spawn_center_container_door_floor_m=list(feeder),spawn_position_world_m=spawn,
                 center_container_door_floor_m=center_local,orientation=dict(yaw_deg=math.degrees(yaw),quaternion_xyzw=[0,0,math.sin(yaw/2),math.cos(yaw/2)]),
                 position_world_m=center,orientation_world_xyzw=(rot*Rotation.from_euler('z',yaw)).as_quat().tolist(),
                 domain_certificate=cert,feeder_joint_q=np.asarray(workspace.q_last).tolist(),route=route,stack_layer=level)
        rows.append(row);placed.append(box);levels.append(level);remaining.remove(i);frontier=x
        write_json(output/'plan_partial.json',dict(sequence=rows,rejected=rejected,remaining=remaining))
        print('SELECTED %d %s layer=%d local=%s' % (len(rows),labels[i],level,center_local),flush=True)
    write_json(output/'search_result.json',dict(requested=target,selected=len(rows),remaining=[labels[i] for i in remaining],rejected=len(rejected),max_layer=max(levels,default=0)))
    write_json(output/'rejected_candidates.json',rejected)
    return rows,poses
