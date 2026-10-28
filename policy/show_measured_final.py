"""Show a completed measured-case placement probe as a still Newton scene."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import yaml
from container_demo import make_ground_grid
import warp as wp
from online_packing_real import Hull
from online_packing_newton import make_model
from roboticarm_newton import DEFAULT_SOURCE


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--bundle',type=Path,required=True)
 p.add_argument('--device',default='cuda:0')
 args=p.parse_args();folder=args.bundle
 result=json.loads((folder/'result.json').read_text(encoding='utf-8'))
 if not result.get('success'):raise ValueError('placement probe is not complete')
 inputs=json.loads((folder/'input.json').read_text(encoding='utf-8'))
 scene=folder/'scene_tf.yaml';hull=Hull.from_config(yaml.safe_load(scene.read_text(encoding='utf-8')))
 palette={'blue':[.12,.38,.90],'black':[.08,.08,.10],'grey':[.55,.57,.60],
          'red':[.85,.10,.10],'silver':[.76,.78,.80],'pink':[.94,.54,.68],
          'smallgrey':[.48,.51,.54],'bronzewhite':[.70,.56,.38]}
 labels=inputs['box_labels']
 sample=dict(container=hull.as_dict(),steps=[dict(incoming=dict(id='B%d'%(i+1),dims=dims,
       color_rgb=palette.get(labels[i],[.62,.40,.28]))) for i,dims in enumerate(inputs['boxes'])])
 wp.set_device(args.device)
 model,_,_,_=make_model(sample,DEFAULT_SOURCE.resolve(),scene);state=model.state()
 with np.load(folder/'trajectory.npz') as data:
  if list(data['body_labels'])!=list(model.body_label):raise ValueError('model body order differs from probe')
  state.joint_q.assign(data['joint_q'][-1]);state.body_q.assign(data['body_q'][-1])
 from newton import viewer as newton_viewer
 viewer=newton_viewer.ViewerGL(width=1280,height=800);viewer.set_model(model)
 viewer.set_camera(pos=wp.vec3(3.7,-5.8,3.2),pitch=-12,yaw=136)
 viewer.renderer.set_title('Newton | %d placed | empty return not checked'%result['placed'])
 grid=make_ground_grid(z=.003)
 print('FINAL_PLACEMENT_SCENE %d; empty returns unverified'%result['placed'],flush=True)
 try:
  while viewer.is_running():
   viewer.begin_frame(0.);viewer.log_state(state);viewer.log_lines('ground_grid',*grid);viewer.end_frame();time.sleep(1/60)
 finally:viewer.close()
if __name__=='__main__':main()
