"""Search nearby placements for the three proposed measured-case stacks."""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from demo_feeder import feeder_route
from measured_luggage_policy import candidate_slots
from online_packing_demo import Box, collides, fully_supported
from scene_frames import rotation
from three_stack_layout import STACKS, CENTERS, build_three_stacks, lateral_gap


def plan_three_stacks(inputs,hull,workspace,prior,output,write_json,
                      prefix_rows=(), prefix_poses=(), check_return=True,
                      centers=CENTERS, min_stack_gap=0.0, strict_centers=False):
    layout=build_three_stacks(inputs,hull,centers)
    placed=[]; levels=[]; rows=[]; poses=[]; feeder=(-.43,.5,.55)
    base_centers={}; frontier=float('inf')
    container=next(t for t in workspace.config['static_transforms'] if t['child']=='container_link')
    rot=rotation(container)
    if len(prefix_rows)>len(layout) or len(prefix_rows)!=len(prefix_poses):
        raise ValueError('invalid prefix size')
    for row,(proposed,index,level) in zip(prefix_rows,layout):
        if (row['input_index']!=index or row['id']!=proposed.id
                or row['stack_layer']!=level or row['input_dims_m']!=inputs['boxes'][index]):
            raise ValueError('prefix does not match the inventory and stack order')
        dims=tuple(row['placed_dims_m']); center=row['center_container_door_floor_m']
        box=Box(row['id'],dims,tuple(round(center[k]-dims[k]/2,6) for k in range(3)))
        if (not hull.fits(box) or any(collides(box,other) for other in placed)
                or not (hull.floor_supports(box) or fully_supported(box,placed))
                or center[0]>frontier+1e-6):
            raise ValueError('prefix fails support, collision or depth ordering')
        group=next(i for i,names in enumerate(STACKS) if row['label'] in names)
        if level==1: base_centers[group]=(center[0],center[1])
        placed.append(box);levels.append(level);rows.append(row);frontier=center[0]
    poses.extend(prefix_poses)
    if rows: write_json(output/'plan_partial.json',dict(sequence=rows,source='resumed, not yet revalidated'))
    for proposed,index,level in layout[len(rows):]:
        label=inputs['box_labels'][index]
        group=next(i for i,names in enumerate(STACKS) if label in names)
        desired=base_centers.get(group,centers[group])
        slots=[slot for slot in candidate_slots(inputs['boxes'][index],placed,levels,hull,frontier,
                                                extra_ys=(desired[1],))
               if slot[4]==level and slot[4]<=3]
        # Keep upper cases centred over their actual base and never return to
        # a deeper x after advancing toward the doorway.
        if level>1 or strict_centers:
            slots=[slot for slot in slots if abs(slot[1]-desired[0])<1e-6 and abs(slot[2]-desired[1])<1e-6]
        if min_stack_gap:
            def separated(slot):
                box=slot[0]
                for other,row in zip(placed,rows):
                    other_group=next(i for i,names in enumerate(STACKS) if row['label'] in names)
                    if group==other_group:
                        continue
                    gap=lateral_gap(box,other)
                    if gap < min_stack_gap-1e-7:
                        return False
                return True
            slots=[slot for slot in slots if separated(slot)]
        slots.sort(key=lambda item:(abs(item[1]-desired[0])+abs(item[2]-desired[1]),abs(item[3])))
        print('STACK_SEARCH %s layer=%d candidates=%d'%(label,level,len(slots)),flush=True)
        selected=None
        for number,(candidate,x,y,yaw,_) in enumerate(slots):
            local=[round(x,6),round(y,6),round(candidate.pos[2]+candidate.dims[2],6)]
            world,world_yaw=workspace.local_contact_world(local,(0,0,0),yaw,hull)
            witnesses=sorted((p for p in prior if abs(p['box_yaw_rad']-yaw)<1e-6),
                             key=lambda p:sum((p['contact_local_m'][k]-local[k])**2 for k in range(3)))
            attempts=[];q=None
            for seed_record in witnesses[:4]:
                ok,solution=workspace._solve_once(world,workspace.tool_orientation(world_yaw),np.asarray(seed_record['joint_q'],dtype=np.float32),True)
                attempts.append(dict(seed_contact=seed_record['contact_local_m'],ok=bool(ok),
                                     diagnostics=dict(workspace.last_diagnostics),collisions=list(workspace.last_collision_pairs)))
                if ok:
                    q=solution;workspace.q_last=q.copy();break
            payload=dict(step=len(rows)+1,label=label,candidate=number,contact_local_m=local,
                         yaw_rad=yaw,stack_layer=level,attempts=attempts)
            write_json(output/'latest_candidate.json',payload)
            if q is None:
                reason='endpoint_ik'
            else:
                box=Box(proposed.id,candidate.dims,candidate.pos)
                route=feeder_route(box,placed,hull,workspace,inputs['boxes'][index],feeder,
                                   check_return=check_return)
                reason=None if route is not None else 'payload_or_robot_route'
            with (output/'candidate_boundary.jsonl').open('a',encoding='utf-8') as log:
                log.write(json.dumps(dict(payload,reason=reason))+'\n')
            if reason is not None:
                continue
            selected=(box,x,y,yaw,world,q,route,local)
            break
        if selected is None:
            write_json(output/'failed_candidate.json',dict(label=label,layer=level,tested=len(slots),reason='no_candidate_passed'))
            print('BLOCKED %s layer=%d candidates=%d'%(label,level,len(slots)),flush=True)
            break
        box,x,y,yaw,world,q,route,local=selected
        if level==1: base_centers[group]=(x,y)
        center_local=[box.pos[k]+box.dims[k]/2 for k in range(3)]
        center=(np.asarray(world)-rot.apply([0,0,box.dims[2]/2])).tolist()
        spawn=(np.asarray(center)+rot.apply(np.asarray(feeder)-center_local)).tolist()
        cert=len(poses)
        poses.append(dict(contact_local_m=local,contact_world_m=world.tolist(),box_yaw_rad=yaw,joint_q=q.tolist()))
        rows.append(dict(step=len(rows)+1,input_index=index,id=box.id,label=label,input_dims_m=inputs['boxes'][index],
          placed_dims_m=list(box.dims),spawn_center_container_door_floor_m=list(feeder),spawn_position_world_m=spawn,
          center_container_door_floor_m=center_local,orientation=dict(yaw_deg=math.degrees(yaw),
          quaternion_xyzw=[0,0,math.sin(yaw/2),math.cos(yaw/2)]),
          position_world_m=center,orientation_world_xyzw=(rot*Rotation.from_euler('z',yaw)).as_quat().tolist(),
          domain_certificate=cert,feeder_joint_q=np.asarray(workspace.q_last).tolist(),route=route,stack_layer=level))
        placed.append(box);levels.append(level);frontier=x
        write_json(output/'plan_partial.json',dict(sequence=rows))
        print('SELECTED %d/%d %s stack_layer=%d local=%s'%(len(rows),len(layout),label,level,local),flush=True)
    return rows,poses
