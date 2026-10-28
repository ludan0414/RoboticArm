"""Validate loaded pick/carry/place phases; omit empty returns by request.

Each new pickup starts from a teleport to a pre-pick pose, so this is not a
continuous or hardware-executable trajectory.  No real-robot claim is made.
"""
import argparse
import json
import math
import shutil
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import warp as wp
import yaml
from scipy.spatial.transform import Rotation
from demo_contract import write_json
from demo_trajectory import MotionCompiler
from demo_workspace import CertifiedWorkspace
from online_packing_newton import Pose,world_pose
from online_packing_real import Hull
from roboticarm_newton import DEFAULT_SOURCE


class PlacementProbe(MotionCompiler):
    def execute_placements(self, rows):
        for row in rows:
            self.box_id, self.box_pose = row['id'], None
            dims=np.asarray(row['placed_dims_m'])
            local_q=np.asarray(row['orientation']['quaternion_xyzw'])
            route=[world_pose(Pose(np.asarray(p)+dims/2,local_q),self.container,self.hull) for p in row['route']]
            contacts=[p.center+Rotation.from_quat(p.quat).apply([0,0,dims[2]/2]) for p in route]
            yaw=float(self.container['rotation_rpy'][2])+math.radians(row['orientation']['yaw_deg'])
            self.ik.q_last=np.asarray(row['feeder_joint_q'],dtype=np.float32)
            self.enter('RESET_HOME',note='teleport between cases; empty return not validated')
            self.q=self.solve(contacts[0]+[0,0,.12],yaw);self.frame()
            self.box_pose=np.r_[route[0].center,route[0].quat]
            self.enter('SPAWN',input_index=row['input_index'],pose_world=self.box_pose.tolist());self.hold(1)
            self.enter('APPROACH');self.move(contacts[0]+[0,0,.08],yaw)
            self.enter('PICK');self.move(contacts[0],yaw)
            tool=self.state.body_q.numpy()[self.contact_body]; inverse=Rotation.from_quat(tool[3:]).inv()
            self.grasp_xyz=inverse.apply(self.box_pose[:3]-tool[:3])
            self.grasp_rotation=inverse*Rotation.from_quat(self.box_pose[3:]);self.attached=True
            self.enter('ATTACH');self.hold(1)
            self.enter('LIFT');self.move(contacts[0]+[0,0,.08],yaw)
            self.enter('CARRY');self.move(contacts[0],yaw)
            for contact in contacts[1:-1]:self.move(contact,yaw)
            self.enter('PLACE');self.move(contacts[-1],yaw)
            self.attached=False;self.placed[self.box_id]=self.box_pose.copy()
            self.enter('RELEASE');self.hold(1)
            error=float(np.linalg.norm(self.box_pose[:3]-route[-1].center))
            angle=float((Rotation.from_quat(self.box_pose[3:]).inv()*Rotation.from_quat(route[-1].quat)).magnitude())
            if error>.006 or angle>.07:raise RuntimeError('final placement outside tolerance')
            self.enter('VERIFY',position_error_m=error,rotation_error_rad=angle)
            print('PLACEMENT_VERIFIED %d/%d %s layer=%d'%(row['step'],len(rows),row['label'],row['stack_layer']),flush=True)
        self.enter('COMPLETE',placed=len(rows),empty_return_checked=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',default='cuda:0')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    for name in ('input.json','plan.json','domain.json','scene_tf.yaml','model_manifest.json'):
        shutil.copyfile(args.plan_dir/name,args.output/name)
    inputs=json.loads((args.output/'input.json').read_text(encoding='utf-8'))
    plan=json.loads((args.output/'plan.json').read_text(encoding='utf-8'))
    scene=args.output/'scene_tf.yaml';hull=Hull.from_config(yaml.safe_load(scene.read_text(encoding='utf-8')))
    run=PlacementProbe(args.output); wp.set_device(args.device)
    try:
        run.initialize(inputs['boxes'],hull,SimpleNamespace(source_root=DEFAULT_SOURCE.resolve(),scene_config=scene,device=args.device))
        run.ik=CertifiedWorkspace(DEFAULT_SOURCE.resolve(),scene,device=args.device,box_size=inputs['boxes'][0],trace=run.trace)
        run.execute_placements(plan['sequence'])
        result=dict(success=True,placed=len(plan['sequence']),scope='pick_carry_place_only',
                    empty_return_checked=False,transitions_between_boxes='teleport',physics_dynamics='not_evaluated',
                    payload_checked_frames=run.payload_checked_frames)
        write_json(args.output/'result.json',result);print('PLACEMENT_CHECK_COMPLETE '+json.dumps(result),flush=True)
    except BaseException as exc:
        write_json(args.output/'result.json',dict(success=False,failed_stage=run.stage,box_id=run.box_id,error=str(exc),empty_return_checked=False))
        print('PLACEMENT_CHECK_FAILED %s %s: %s'%(run.box_id,run.stage,exc),flush=True)
        raise
    finally:
        run.close()

if __name__=='__main__':main()
