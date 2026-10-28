"""Export an operator-assumed ground entry from a fully checked inside recording.

The eight inside motions and their returns must have a valid READY bundle.
Ground entry is explicitly unverified: hardware retains live MoveIt, attached
payload, pedestal and doorway checks. This is not a dynamics/hardware proof.
"""
import argparse,copy,hashlib,json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from demo_contract import check_bundle
from export_hardware_loading_profile import pose_dict,sample_poses,post_place_exit_indices
from ground_carry_clearance import pedestal_geometry,safe_carry_entry
from doorway_scene import DoorwayGeometry


def _matches_pose(saved, xyz, rotation):
    actual=Rotation.from_quat(saved['orientation_xyzw'])
    return (np.linalg.norm(np.asarray(saved['position_m'])-xyz)<1e-6
            and (actual.inv()*rotation).magnitude()<1e-6)


def export_assumed_profile(bundle,template):
    bundle=Path(bundle)
    plan=check_bundle(bundle,allow_partial=True)
    validation=json.loads((bundle/'validation.json').read_text(encoding='utf-8'))
    if not validation.get('success') or validation.get('boxes')!=len(plan['sequence']):
        raise ValueError('inside trajectory is not fully validated')
    scene=yaml.safe_load((bundle/'scene_tf.yaml').read_text(encoding='utf-8'))
    profile=copy.deepcopy(json.loads(Path(template).read_text(encoding='utf-8')))
    if profile.get('carry_entry',{}).get('strategy')!='outside_retract_high_lift_v1':
        raise ValueError('missing existing high-entry safety contract')
    if profile.get('motion_safety',{}).get('feeder_base_xy_m')!=[-1.6,0.0]:
        raise ValueError('unexpected feeder position')
    pedestal_scene=scene['pedestal']
    pedestal_rot=Rotation.from_euler('xyz',pedestal_scene['rotation_rpy'])
    base_xyz=np.asarray(pedestal_scene['translation'])+pedestal_rot.apply([0,0,pedestal_scene['size'][2]])
    base_rot=pedestal_rot*Rotation.from_euler('xyz',scene['robot']['rotation_rpy'])
    container=next(t for t in scene['static_transforms'] if t['child']=='container_link')
    safety=profile['motion_safety']
    if not _matches_pose(safety['expected_base_world_pose'],base_xyz,base_rot):
        raise ValueError('template/base calibration does not match checked scene')
    if not _matches_pose(safety['expected_container_world_pose'],np.asarray(container['translation']),
                         Rotation.from_euler('xyz',container['rotation_rpy'])):
        raise ValueError('template/container calibration does not match checked scene')
    if profile['doorway_clearance']!=DoorwayGeometry.from_scene(scene).settings():
        raise ValueError('template/doorway geometry does not match checked scene')
    if len(plan['sequence'])!=8 or max(r['stack_layer'] for r in plan['sequence'])>3:
        raise ValueError('expected eight measured boxes at no more than three layers')
    pedestal=pedestal_geometry(scene)
    cycles=[]
    with np.load(bundle/'trajectory.npz',allow_pickle=False) as trace:
        for row in plan['sequence']:
            carrying=np.flatnonzero((trace['box_id']==row['id'])&(trace['stage']=='CARRY'))
            if not len(carrying):raise ValueError('missing carry '+row['id'])
            entry=safe_carry_entry(carrying,trace['box_pose'],row['input_dims_m'],pedestal)
            suffix=carrying[carrying>=entry]
            exit_indices=post_place_exit_indices(trace,row['id'])
            cycles.append(dict(demo_box_id=row['id'],box_label=row['label'],
                expected_demo_input_dims_m=row['input_dims_m'],
                place_target=dict(frame_id='world',pose_reference='box_center',position_m=row['position_world_m'],
                    orientation_xyzw=row['orientation_world_xyzw'],clearance_m=.15,
                    via_box_poses=sample_poses(trace['box_pose'][suffix])),
                exit_tool_poses=sample_poses(trace['tool_pose'][exit_indices])))
    hashes={name:hashlib.sha256((bundle/name).read_bytes()).hexdigest() for name in ('plan.json','trajectory.npz','scene_tf.yaml')}
    profile['cycles']=cycles
    profile['assumptions'].update(source_bundle='policy/demo_data/'+bundle.name,source_sha256=hashes,
        hardware_motion_validated=False,ground_entry_simulation_validated=False,
        ground_entry_assumed_by_operator=True,inside_and_reverse_simulation_validated=True,
        slope_edge_margin_m=0.0)
    return profile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inside',type=Path,required=True);p.add_argument('--template',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assume-ground-entry',action='store_true',required=True,
                   help='explicitly record that ground connection has not completed simulation validation')
    args=p.parse_args();profile=export_assumed_profile(args.inside,args.template)
    with args.output.open('w',encoding='utf-8',newline='\n') as stream:
        json.dump(profile,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    print('EXPORTED %d cycles; ground entry assumed, not validated'%len(profile['cycles']))
if __name__=='__main__':main()
