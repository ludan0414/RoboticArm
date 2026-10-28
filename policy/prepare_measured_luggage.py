"""Plan measured cases offline and compile checked Newton motion."""
import argparse
import json
import shutil
import time
from pathlib import Path
import numpy as np
import warp as wp
import yaml
from demo_contract import load_box_input, write_json, freeze_bundle
from demo_trajectory import compile_trajectory
from doorway_scene import clearance_hull, DoorwayGeometry
from measured_luggage_policy import plan_measured
from online_packing_real import Hull
from prepare_full_floor_demo import FullFloorWorkspace
from prepare_greedy_demo import model_manifest
from roboticarm_newton import DEFAULT_SOURCE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=Path('demo_inputs/site_thirteen_measured.json'))
    parser.add_argument('--scene-config',type=Path,default=Path('../deployment_ws/config/scene_tf.loading_site.yaml'))
    parser.add_argument('--source-root',type=Path,default=DEFAULT_SOURCE)
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--target',type=int,default=12)
    parser.add_argument('--seed-domain',type=Path,default=Path('demo_data/site_four_colors_inside_clearance5/domain.json'),
                        help='prior IK joint witnesses, used only as untrusted numerical seeds')
    args=parser.parse_args()
    args.source_root=args.source_root.resolve(); args.scene_config=args.scene_config.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic(); stage='INPUT'
    with (args.output/'offline_boundary.jsonl').open('w',encoding='utf-8') as trace:
        try:
            inputs=load_box_input(args.input)
            if not 1 <= args.target <= len(inputs['boxes']): raise ValueError('target exceeds physical inventory')
            write_json(args.output/'input.json',inputs)
            shutil.copyfile(args.scene_config,args.output/'scene_tf.yaml')
            scene=yaml.safe_load(args.scene_config.read_text(encoding='utf-8'))
            physical=Hull.from_config(scene); hull=clearance_hull(physical,scene)
            write_json(args.output/'model_manifest.json',model_manifest(args.source_root,args.scene_config))
            wp.set_device(args.device)
            workspace=FullFloorWorkspace(args.source_root,args.scene_config,device=args.device,box_size=inputs['boxes'][0],trace=trace)
            stage='SEARCH'
            seeds=json.loads(args.seed_domain.read_text(encoding='utf-8'))['poses']
            rows,poses=plan_measured(inputs,hull,workspace,args.output,write_json,target=args.target,prior_witnesses=seeds)
            if not rows: raise RuntimeError('no feasible candidate found in search grid')
            write_json(args.output/'domain.json',dict(schema_version=1,representation='finite_certified_poses',interpolation_allowed=False,
                       frame='container_door_floor',accepted=len(poses),poses=poses,scope='selected top contacts; route checked separately'))
            write_json(args.output/'plan.json',dict(schema_version=1,policy='measured_four_layer_greedy',position_frame='world',
                       route_frame='container_door_floor',sequence=rows,doorway_clearance=DoorwayGeometry.from_scene(scene).settings()))
            stage='COMPILE_TRAJECTORY'
            workspace.q_last=np.asarray(rows[0]['feeder_joint_q'],dtype=np.float32)
            validation=compile_trajectory(args,inputs,physical,rows,workspace)
            write_json(args.output/'validation.json',validation)
            if not validation.get('success'): raise RuntimeError('trajectory validation failed')
            freeze_bundle(args.output)
            write_json(args.output/'prepare_result.json',dict(success=True,boxes=len(rows),target=args.target,partial=len(rows)<args.target,
                       max_layers=max(r['stack_layer'] for r in rows),elapsed_sec=time.monotonic()-start))
            print('READY_FOR_REPLAY %d/%d' % (len(rows),args.target),flush=True)
            return 0
        except BaseException as exc:
            write_json(args.output/'prepare_result.json',dict(success=False,stage=stage,error=str(exc),elapsed_sec=time.monotonic()-start,replay_ready=False))
            print('FAILED %s: %s' % (stage,exc),flush=True)
            return 1

if __name__=='__main__':
    raise SystemExit(main())
