"""Compile a placement search with checked original-route returns."""
import argparse,json,shutil,time
from pathlib import Path
import numpy as np, warp as wp, yaml
from demo_contract import write_json,freeze_bundle
from demo_trajectory import compile_trajectory
from online_packing_real import Hull
from prepare_full_floor_demo import FullFloorWorkspace
from roboticarm_newton import DEFAULT_SOURCE

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--device',default='cuda:0');args=p.parse_args()
 args.source_root=DEFAULT_SOURCE.resolve();args.output.mkdir(parents=True,exist_ok=False);start=time.monotonic();stage='INPUT'
 with (args.output/'offline_boundary.jsonl').open('w',encoding='utf-8') as trace:
  try:
   for name in ('input.json','plan.json','domain.json','scene_tf.yaml','model_manifest.json'):shutil.copyfile(args.plan_dir/name,args.output/name)
   inputs=json.loads((args.output/'input.json').read_text(encoding='utf-8'));plan=json.loads((args.output/'plan.json').read_text(encoding='utf-8'))
   args.scene_config=(args.output/'scene_tf.yaml').resolve();scene=yaml.safe_load(args.scene_config.read_text(encoding='utf-8'));hull=Hull.from_config(scene)
   wp.set_device(args.device);stage='INITIALIZE'
   workspace=FullFloorWorkspace(args.source_root,args.scene_config,device=args.device,box_size=inputs['boxes'][0],trace=trace)
   workspace.q_last=np.asarray(plan['sequence'][0]['feeder_joint_q'],dtype=np.float32)
   stage='COMPILE_WITH_RETURN';validation=compile_trajectory(args,inputs,hull,plan['sequence'],workspace)
   write_json(args.output/'validation.json',validation)
   if not validation.get('success'):raise RuntimeError('full trajectory verification failed')
   freeze_bundle(args.output)
   write_json(args.output/'prepare_result.json',dict(success=True,boxes=len(plan['sequence']),elapsed_sec=time.monotonic()-start,return_checked=True))
   print('FULL_RETURN_READY %d boxes'%len(plan['sequence']),flush=True);return 0
  except BaseException as exc:
   write_json(args.output/'prepare_result.json',dict(success=False,stage=stage,error=str(exc),elapsed_sec=time.monotonic()-start,replay_ready=False))
   print('FULL_RETURN_FAILED %s: %s'%(stage,exc),flush=True);return 1
if __name__=='__main__':raise SystemExit(main())
