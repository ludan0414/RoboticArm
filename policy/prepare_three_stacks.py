"""Prepare a checked three-stack layout from measured luggage cases."""
import argparse
import json
import shutil
import time
from pathlib import Path
import numpy as np
import warp as wp
import yaml
from demo_contract import load_box_input,write_json,freeze_bundle
from demo_trajectory import compile_trajectory
from doorway_scene import clearance_hull,DoorwayGeometry
from online_packing_real import Hull
from prepare_full_floor_demo import FullFloorWorkspace
from prepare_greedy_demo import model_manifest
from roboticarm_newton import DEFAULT_SOURCE
from three_stack_planner import plan_three_stacks


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--input',type=Path,default=Path('demo_inputs/site_thirteen_measured.json'))
 p.add_argument('--scene-config',type=Path,default=Path('../deployment_ws/config/scene_tf.loading_site.yaml'))
 p.add_argument('--source-root',type=Path,default=DEFAULT_SOURCE)
 p.add_argument('--seed-domain',type=Path,default=Path('demo_data/site_four_colors_inside_clearance5/domain.json'))
 p.add_argument('--output',type=Path,required=True)
 p.add_argument('--device',default='cuda:0')
 p.add_argument('--resume-plan',type=Path,help='prior partial search directory; geometry and inventory rechecked')
 p.add_argument('--placements-only',action='store_true',help='skip empty-return checking and full motion compilation')
 p.add_argument('--stack-y',type=float,nargs=3,help='fixed local-y centres for the three stacks')
 p.add_argument('--min-stack-gap',type=float,default=0.0,help='minimum lateral gap between different stacks, metres')
 args=p.parse_args();args.source_root=args.source_root.resolve();args.scene_config=args.scene_config.resolve()
 args.output.mkdir(parents=True,exist_ok=False)
 started=time.monotonic();stage='INPUT'
 with (args.output/'offline_boundary.jsonl').open('w',encoding='utf-8') as trace:
  try:
   inputs=load_box_input(args.input);write_json(args.output/'input.json',inputs)
   shutil.copyfile(args.scene_config,args.output/'scene_tf.yaml')
   scene=yaml.safe_load(args.scene_config.read_text(encoding='utf-8'))
   hull=Hull.from_config(scene);routing=clearance_hull(hull,scene)
   write_json(args.output/'model_manifest.json',model_manifest(args.source_root,args.scene_config))
   wp.set_device(args.device)
   workspace=FullFloorWorkspace(args.source_root,args.scene_config,device=args.device,box_size=inputs['boxes'][0],trace=trace)
   prior=json.loads(args.seed_domain.read_text(encoding='utf-8'))['poses']
   prefix=[]; prefix_poses=[]
   if args.resume_plan:
    source=args.resume_plan
    if json.loads((source/'input.json').read_text(encoding='utf-8')) != inputs:
     raise ValueError('resume inventory mismatch')
    if yaml.safe_load((source/'scene_tf.yaml').read_text(encoding='utf-8')) != scene:
     raise ValueError('resume scene mismatch')
    prefix=json.loads((source/'plan_partial.json').read_text(encoding='utf-8'))['sequence']
    prefix_poses=json.loads((source/'domain.json').read_text(encoding='utf-8'))['poses']
   stage='THREE_STACK_SEARCH'
   from three_stack_layout import CENTERS
   if args.min_stack_gap < 0: raise ValueError('min-stack-gap must be nonnegative')
   centers=tuple((.60,y) for y in args.stack_y) if args.stack_y else CENTERS
   rows,poses=plan_three_stacks(inputs,routing,workspace,prior,args.output,write_json,
       prefix_rows=prefix,prefix_poses=prefix_poses,check_return=not args.placements_only,
       centers=centers,min_stack_gap=args.min_stack_gap,strict_centers=bool(args.stack_y))
   if not rows: raise RuntimeError('first proposed position has no certified route')
   write_json(args.output/'domain.json',dict(schema_version=1,representation='finite_certified_poses',interpolation_allowed=False,
       frame='container_door_floor',accepted=len(poses),poses=poses,scope='selected contacts, checked motion separate'))
   write_json(args.output/'plan.json',dict(schema_version=1,policy='three_stack_fixed_candidate',position_frame='world',
       route_frame='container_door_floor',sequence=rows,doorway_clearance=DoorwayGeometry.from_scene(scene).settings()))
   if args.placements_only:
    write_json(args.output/'prepare_result.json',dict(success=True,stage='PLACEMENT_SEARCH_ONLY',boxes=len(rows),target=8,
      partial=len(rows)<8,empty_return_checked=False,arm_against_placed_cargo_checked=False,
      replay_ready=False,elapsed_sec=time.monotonic()-started))
    print('PLACEMENT_SEARCH_ONLY %d/8: full motion not verified'%len(rows),flush=True)
    return 0
   stage='COMPILE_TRAJECTORY';workspace.q_last=np.asarray(rows[0]['feeder_joint_q'],dtype=np.float32)
   validation=compile_trajectory(args,inputs,hull,rows,workspace)
   write_json(args.output/'validation.json',validation)
   if not validation.get('success'): raise RuntimeError('trajectory validation failed')
   freeze_bundle(args.output)
   write_json(args.output/'prepare_result.json',dict(success=True,boxes=len(rows),target=8,partial=len(rows)<8,
       max_layers=max(r['stack_layer'] for r in rows),elapsed_sec=time.monotonic()-started))
   print('READY_FOR_REPLAY %d/8'%len(rows),flush=True);return 0
  except BaseException as exc:
   write_json(args.output/'prepare_result.json',dict(success=False,stage=stage,error=str(exc),elapsed_sec=time.monotonic()-started,replay_ready=False))
   print('FAILED %s: %s'%(stage,exc),flush=True);return 1
if __name__=='__main__':raise SystemExit(main())
