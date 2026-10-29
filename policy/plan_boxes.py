"""Offline, non-LLM placement planning with one frozen route for ROS and Newton.

Run in the Newton environment. Vision Pick remains the existing live process.
Validation starts with a nominal sealed grasp at taught P1, includes P1->P2,
placement and reverse return. Only a fully passing run exports hardware.json.
"""
import argparse,copy,json,math,sys,hashlib,subprocess,platform,time
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1]
for package in ('luggage_planning','luggage_description'):
    sys.path.insert(0,str(ROOT/'src'/package))


def load_inventory(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    entries=data['boxes'] if isinstance(data,dict) else data
    if not isinstance(entries,list) or not entries:raise ValueError('boxes must be a nonempty list')
    # Existing measured inventories store dimensions and names in parallel.
    # Normalize that public format before the common named-row validation.
    if isinstance(entries[0],list):
        if not isinstance(data,dict):
            raise ValueError('dimension arrays require boxes, box_labels and units')
        labels=data.get('box_labels')
        if not isinstance(labels,list) or len(labels)!=len(entries):
            raise ValueError('box_labels must contain one name per box')
        unit=data.get('units')
        if unit not in ('m','cm'):
            raise ValueError('dimension arrays require units m or cm')
        entries=[dict(name=name,**{'size_'+unit:values})
                 for name,values in zip(labels,entries)]
    names=[];boxes=[]
    for row in entries:
        if not isinstance(row,dict):raise ValueError('each box must be a named object or dimension array')
        name=row['name']
        if not isinstance(name,str) or not name.strip() or name in names:
            raise ValueError('box names must be nonempty and unique')
        if ('size_cm' in row)==('size_m' in row):raise ValueError('specify exactly one of size_cm and size_m')
        values=row.get('size_cm',row.get('size_m'));divisor=100. if 'size_cm' in row else 1.
        if not isinstance(values,list) or len(values)!=3:raise ValueError('size needs L,W,H')
        dims=[float(v)/divisor for v in values]
        if not all(math.isfinite(v) and v>0 for v in dims):raise ValueError('dimensions must be positive and finite')
        names.append(name);boxes.append(dims)
    return dict(box_labels=names,boxes=boxes)


def execute(args):
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation
    from demo_contract import write_json,freeze_bundle
    from demo_recording import validate_recording
    from prepare_greedy_demo import model_manifest
    from prepare_full_floor_demo import FullFloorWorkspace
    from measured_luggage_policy import plan_measured
    from online_packing_real import Hull
    from doorway_scene import clearance_hull
    from roboticarm_newton import DEFAULT_SOURCE
    from ground_feeder_bridge import BridgeCompiler
    from ground_carry_clearance import pedestal_geometry
    from luggage_planning.manual_carry_transfer import TRANSFER_JOINTS_RAD
    from luggage_planning.policy_place import PlaceTarget,build_place_segments
    from luggage_planning.placement_payload import release_above_support
    from luggage_planning.pose import Quaternion
    from luggage_planning.doorway_motion import DoorwayMotionGuard
    from luggage_planning.offline_route_contract import validation_stamp,PROFILE_TYPE
    from offline_frozen_planner import find_joint_route,pose_array,pose_data,checked_tool_path,checked_pose,xyz,quat
    inputs=args.inventory;out=args.output
    scene=yaml.safe_load(args.scene_config.read_text(encoding='utf-8'));hull=Hull.from_config(scene)
    write_json(out/'input.json',inputs)
    (out/'scene_tf.yaml').write_bytes(args.scene_config.read_bytes())
    write_json(out/'model_manifest.json',model_manifest(DEFAULT_SOURCE,args.scene_config))
    profile=json.loads(args.template.read_text(encoding='utf-8'));profile=copy.deepcopy(profile)
    profile['profile']=PROFILE_TYPE
    profile['cycles']=[]
    run=BridgeCompiler(out)
    try:
        run.initialize(inputs['boxes'],hull,NS(source_root=DEFAULT_SOURCE,scene_config=args.scene_config,device=args.device))
        workspace=FullFloorWorkspace(DEFAULT_SOURCE,args.scene_config,device=args.device,box_size=inputs['boxes'][0],trace=run.trace)
        run.ik=workspace;run.pedestal=pedestal_geometry(scene)
        run.inventory_sizes={'B%d'%(i+1):dims for i,dims in enumerate(inputs['boxes'])}
        run.high_carry=False;run.high_carry_bottom_z=0.;run.checking=True
        transform=next(t for t in scene['static_transforms'] if t['child']=='container_link')
        rotation=Rotation.from_euler('xyz',transform['rotation_rpy'])
        container=pose_array([*transform['translation'],*rotation.as_quat()])
        geometry=DoorwayMotionGuard(profile).geometry
        from doorway_scene import DoorwayGeometry
        if geometry.settings()!=DoorwayGeometry.from_scene(scene).settings():
            raise ValueError('scene and hardware template doorway settings differ')
        base=pose_array(workspace.model.body_q.numpy()[workspace.base_body])
        for name,actual in (('expected_base_world_pose',base),('expected_container_world_pose',container)):
            expected=profile['motion_safety'][name]
            if (np.linalg.norm(xyz(actual)-expected['position_m'])>.001 or
                (Rotation.from_quat(expected['orientation_xyzw']).inv()*Rotation.from_quat(quat(actual))).magnitude()>.001):
                raise ValueError('template calibration differs from scene: '+name)
        p1,p2=map(np.asarray,TRANSFER_JOINTS_RAD)
        checked_routes={};transfer_cache={};trial_count=0
        def route_check(index,box,x,y,yaw,placed,endpoint_seed):
            nonlocal trial_count
            trial_count+=1;folder=out/('candidate_route_%04d'%trial_count);folder.mkdir()
            run.box_id='B%d'%(index+1);size=inputs['boxes'][index];run.current_box_size=size
            run.route_end_seed=None
            run.placed={}
            for other in placed:
                original=inputs['boxes'][int(other.id[1:])-1]
                angle=workspace.box_yaw(original,other.dims)
                contact,_=workspace.local_contact_world(other.pos,other.dims,angle,hull)
                center=contact-rotation.apply([0,0,other.dims[2]/2])
                run.placed[other.id]=np.r_[center,(rotation*Rotation.from_euler('z',angle)).as_quat()]
            contact,_=workspace.local_contact_world(box.pos,box.dims,yaw,hull)
            target=PlaceTarget(pose_array([*(contact-rotation.apply([0,0,box.dims[2]/2])),
                *(rotation*Rotation.from_euler('z',yaw)).as_quat()]),.075,reference_box_size_m=tuple(size))
            run.attached=True;run.box_pose=np.zeros(7)
            run.grasp_xyz=np.array([0.,0.,size[2]/2]);run.grasp_rotation=Rotation.from_euler('x',np.pi)
            run.refresh(p2);tool=pose_array(run.state.body_q.numpy()[run.contact_body])
            run.q=p1.copy();run.stage='CARRY'
            key=(index,tuple((other.id,other.pos,other.dims) for other in placed))
            if key not in transfer_cache:
                transfer_cache[key]=checked_tool_path(run,[tool],p1) if run.valid(p1) else None
            transfer=transfer_cache[key]
            if transfer is None:
                write_json(folder/'rejected.json',dict(reason='nominal P1-P2 IK/collision'));return False
            grasp=((0.,0.,size[2]/2),Quaternion(1,0,0,0))
            goal=build_place_segments(release_above_support(target),NS(frame='world',yaw_valid=True),NS(frame='world'),measured_grasp=grasp)[-2].target_pose
            run.stage='PLACE'
            goal_q,attempts=checked_pose(run,goal,transfer[-1],[endpoint_seed])
            if goal_q is None:
                write_json(folder/'rejected.json',dict(reason='release pose unreachable on controller wrist branch',attempts=attempts));return False
            run.route_end_seed=goal_q
            run.q=transfer[-1];run.refresh(run.q)
            start=pose_array(run.state.body_q.numpy()[run.body_ids[run.box_id]])
            joint_result=find_joint_route(run,target,grasp,goal_q,folder,write_json)
            if joint_result is not None:
                frozen,carry,descend=joint_result
                checked_routes[index]=(target,tool,frozen,transfer,carry,descend)
                return True
            write_json(folder/'rejected.json',dict(reason='vertical approach or joint-space route could not be certified'))
            return False
        seeds=json.loads(args.seed_domain.read_text(encoding='utf-8'))['poses']
        rows,poses=plan_measured(inputs,clearance_hull(hull,scene),workspace,out,write_json,
            target=len(inputs['boxes']),prior_witnesses=seeds,max_layers=args.max_layers,side_first=True,
            route_check=route_check)
        if len(rows)!=len(inputs['boxes']):raise RuntimeError('inventory not fully planned; inspect search_result.json')
        write_json(out/'domain.json',dict(accepted=len(poses),poses=poses,representation='finite_certified_poses'))
        run.placed={}
        for row in rows:
            size=row['input_dims_m'];run.current_box_size=size;run.box_id=row['id']
            run.q=p1.copy();run.attached=True;run.box_pose=np.zeros(7)
            run.grasp_xyz=np.array([0.,0.,size[2]/2]);run.grasp_rotation=Rotation.from_euler('x',np.pi)
            run.refresh(run.q)
            spawn=run.state.body_q.numpy()[run.body_ids[row['id']]].copy()
            row['spawn_position_world_m']=spawn[:3].tolist()
            run.enter('SPAWN',scope='nominal sealed grasp at P1');run.frame()
            run.enter('ATTACH');run.frame()
            target,p2_tool,frozen,transfer,carry,descend=checked_routes[row['input_index']]
            row['position_world_m']=xyz(target.box_pose).tolist()
            row['orientation_world_xyzw']=quat(target.box_pose).tolist()
            run.enter('CARRY')
            for q in transfer:run.q=q;run.frame()
            run.enter('CARRY')
            for q in carry:run.q=q;run.frame()
            run.enter('PLACE')
            for q in descend:run.q=q;run.frame()
            run.attached=False;run.box_pose=np.r_[xyz(target.box_pose),quat(target.box_pose)]
            run.placed[row['id']]=run.box_pose.copy();run.enter('RELEASE');run.frame()
            run.enter('RETREAT')
            for q in reversed([carry[-1],*descend[:-1]]):run.q=q;run.frame()
            run.enter('RETURN')
            for q in reversed([p1,*transfer,*carry[:-1]]):run.q=q;run.frame()
            run.enter('VERIFY');run.frame()
            target_json=dict(frame_id='world',pose_reference='box_center',**pose_data(frozen.box_pose),
                clearance_m=frozen.clearance_m,route_source='offline_frozen_v1',
                via_box_poses=[pose_data(p) for p in frozen.via])
            profile['cycles'].append(dict(demo_box_id=row['id'],box_label=row['label'],
                expected_demo_input_dims_m=size,place_target=target_json,
                exit_tool_poses=[pose_data(p2_tool)]))
            row['frozen_box_route']=target_json;row.pop('route',None)
        write_json(out/'plan.json',dict(schema_version=1,sequence=rows,policy='deterministic_greedy_bidirectional_rrt',
            position_frame='world',route_frame='world',scope='nominal P1-P2, frozen placement, reverse return'))
        run.enter('COMPLETE',placed=len(rows))
    finally:run.close()
    with np.load(out/'trajectory.npz') as trace:recording=validate_recording(rows,trace)
    validation=dict(success=True,boxes=len(rows),frames=len(run.history),scene_recording=recording,
        scope='nominal P1-P2/place/reverse; vision pick not simulated',hardware_motion_validated=False)
    write_json(out/'validation.json',validation)
    profile['offline_validation']=validation_stamp(profile,validation['scope'])
    from demo_contract import digest
    profile['assumptions'].update(source_bundle=str(out),
        source_sha256={name:digest(out/name) for name in ('plan.json','trajectory.npz','scene_tf.yaml')},
        hardware_motion_validated=False,ground_entry_simulation_validated=False,
        ground_entry_assumed_by_operator=False,nominal_p1_p2_placement_validated=True,
        slope_edge_margin_m=min(hull.flat_y-r['center_container_door_floor_m'][1]-r['placed_dims_m'][1]/2
                               for r in rows if r['stack_layer']==1))
    write_json(out/'waypoints.json',dict(cycles=profile['cycles'],validation=profile['offline_validation']))
    freeze_bundle(out)
    write_json(out/'result.json',validation)
    write_json(out/'hardware.json',profile)
    ready=json.loads((out/'READY.json').read_text())
    ready['files'].update({name:digest(out/name) for name in ('hardware.json','waypoints.json','result.json')})
    write_json(out/'READY.json',ready)
    return validation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scene-config',type=Path,default=ROOT/'deployment_ws/config/scene_tf.loading_site.yaml')
    p.add_argument('--template',type=Path,default=ROOT/'deployment_ws/config/loading_cycle.site.template.json')
    p.add_argument('--seed-domain',type=Path,default=Path(__file__).parent/'demo_data/measured8_sides_first_inside/domain.json')
    p.add_argument('--max-layers',type=int,default=3);p.add_argument('--device',default='cuda:0')
    args=p.parse_args();args.output=args.output.resolve();args.scene_config=args.scene_config.resolve()
    if args.max_layers<1:p.error('--max-layers must be positive')
    try:args.inventory=load_inventory(args.input)
    except (OSError,ValueError,KeyError,TypeError) as exc:p.error(str(exc))
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    tracked=['policy/plan_boxes.py','policy/offline_frozen_planner.py','policy/measured_luggage_policy.py',
             'src/luggage_planning/luggage_planning/offline_route_contract.py',
             'src/luggage_planning/luggage_planning/wrist_check.py']
    manifest=dict(command=sys.argv,python=sys.executable,platform=platform.platform(),device=args.device,
        source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in tracked},
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),status='running')
    try:
        manifest['revision']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        manifest['dirty_paths']=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines()
    except (OSError,subprocess.SubprocessError):manifest['revision']=None
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    try:
        result=execute(args);manifest['success']=True;print(json.dumps(result),flush=True);return 0
    except BaseException as exc:
        (args.output/'failure.json').write_text(json.dumps(dict(success=False,error=str(exc)),indent=2),encoding='utf-8')
        print('OFFLINE_PLANNING_FAILED: '+str(exc),flush=True);return 1
    finally:
        manifest.update(status='passed' if manifest.get('success') else 'failed',elapsed_sec=time.monotonic()-started)
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')

if __name__=='__main__':raise SystemExit(main())
