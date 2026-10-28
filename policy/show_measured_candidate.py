"""Display a measured arrangement while full motion checking is in progress."""
import argparse,json,time
from pathlib import Path
import numpy as np, yaml
from container_demo import make_ground_grid
import newton,warp as wp
from online_packing_real import Hull
from online_packing_newton import make_model
from roboticarm_newton import DEFAULT_SOURCE

def main():
 p=argparse.ArgumentParser();p.add_argument('--plan-dir',type=Path,required=True);p.add_argument('--device',default='cuda:0');args=p.parse_args()
 folder=args.plan_dir;plan=json.loads((folder/'plan.json').read_text(encoding='utf-8'))
 inputs=json.loads((folder/'input.json').read_text(encoding='utf-8'))
 scene=folder/'scene_tf.yaml';hull=Hull.from_config(yaml.safe_load(scene.read_text(encoding='utf-8')))
 palette={'blue':[.12,.38,.90],'black':[.08,.08,.10],'grey':[.55,.57,.60],
          'red':[.85,.10,.10],'silver':[.76,.78,.80],'pink':[.94,.54,.68],
          'smallgrey':[.48,.51,.54],'bronzewhite':[.70,.56,.38]}
 sample=dict(container=hull.as_dict(),steps=[dict(incoming=dict(id='B%d'%(i+1),dims=d,
    color_rgb=palette.get(inputs['box_labels'][i],[.62,.40,.28]))) for i,d in enumerate(inputs['boxes'])])
 wp.set_device(args.device);model,ids,_,_=make_model(sample,DEFAULT_SOURCE.resolve(),scene)
 state=model.state();newton.eval_fk(model,model.joint_q,model.joint_qd,model)
 transforms=model.body_q.numpy()
 for row in plan['sequence']:transforms[ids[row['id']]]=np.r_[row['position_world_m'],row['orientation_world_xyzw']]
 state.body_q.assign(transforms)
 from newton import viewer as newton_viewer
 viewer=newton_viewer.ViewerGL(width=1280,height=800);viewer.set_model(model)
 viewer.set_camera(pos=wp.vec3(3.7,-5.8,3.2),pitch=-12,yaw=136)
 viewer.renderer.set_title('Newton | candidate %d boxes | MOTION VALIDATION IN PROGRESS'%len(plan['sequence']))
 grid=make_ground_grid(z=.003);print('CANDIDATE_LAYOUT %d boxes; full motion unverified'%len(plan['sequence']),flush=True)
 try:
  while viewer.is_running():
   viewer.begin_frame(0.);viewer.log_state(state);viewer.log_lines('ground_grid',*grid);viewer.end_frame();time.sleep(1/60)
 finally:viewer.close()
if __name__=='__main__':main()
