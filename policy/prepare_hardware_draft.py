"""Write a review-only eight-box hardware-intent draft; cannot launch it."""
import argparse,json
from pathlib import Path
import numpy as np
from export_hardware_loading_profile import pose_dict,sample_poses
from demo_contract import write_json


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preview',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 plan=json.loads((args.preview/'plan.json').read_text(encoding='utf-8'))
 result=json.loads((args.preview/'result.json').read_text(encoding='utf-8'))
 if not result.get('success') or result.get('scope')!='pick_carry_place_only':raise ValueError('expected successful placement-only preview')
 rows=[]
 with np.load(args.preview/'trajectory.npz',allow_pickle=False) as trace:
  for row in plan['sequence']:
   indices=np.flatnonzero((trace['box_id']==row['id']) & (trace['stage']=='CARRY'))
   if not len(indices):raise ValueError('missing carry '+row['id'])
   rows.append(dict(step=row['step'],box_label=row['label'],box_id=row['id'],size_m=row['input_dims_m'],stack_layer=row['stack_layer'],
                    target_box_pose=pose_dict(np.r_[row['position_world_m'],row['orientation_world_xyzw']]),
                    proposed_carry_box_poses=sample_poses(trace['box_pose'][indices])))
 draft=dict(kind='review_only_not_a_hardware_profile',deployable=False,
            restrictions=['empty return not validated','ground pickup and connection not validated',
              'third stack has zero floor-slope edge margin','MoveIt/real hardware execution not validated'],
            intended_return='existing real controller reverses measured successful carry path, then moves to fixed joint home',
            source_preview=str(args.preview),cases=rows)
 args.output.parent.mkdir(parents=True,exist_ok=True)
 if args.output.exists():raise FileExistsError(args.output)
 write_json(args.output,draft)
 print('REVIEW_ONLY %s cases=%d deployable=false'%(args.output,len(rows)),flush=True)
if __name__=='__main__':main()
