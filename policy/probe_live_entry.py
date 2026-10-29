"""Check the live entry builder with nominal sealed grasp at taught P2.

Not live telemetry: actual measured grasp and calibration can differ.
"""
import argparse,json,sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/luggage_planning'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/luggage_description'))
from luggage_planning.pose import Pose,Point,Quaternion
from luggage_planning.manual_carry_transfer import TRANSFER_JOINTS_RAD
from luggage_planning.hardware_loading_cycle import parse_profile
from luggage_planning.carry_entry import CarryConfig
from luggage_planning.doorway_entry import CenteredDoorwayEntry
from luggage_planning.doorway_motion import DoorwayMotionGuard
from luggage_planning.policy_place import box_pose_from_grasp,build_place_segments
from luggage_planning.placement_payload import release_above_support
from luggage_planning.placement_approach import lower_final_approach
from ground_feeder_bridge import BridgeCompiler
from ground_carry_clearance import pedestal_geometry
from prepare_full_floor_demo import FullFloorWorkspace
from online_packing_real import Hull
from roboticarm_newton import DEFAULT_SOURCE
from demo_contract import write_json

def vector(pose):return np.array([getattr(pose.position,k) for k in 'xyz'])
def quat(pose):return np.array([getattr(pose.orientation,k) for k in 'xyzw'])

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 args.output.mkdir(parents=True,exist_ok=False)
 root=Path(__file__).resolve().parents[1];scene=root/'deployment_ws/config/scene_tf.loading_site.yaml'
 profile=json.loads((root/'deployment_ws/config/loading_cycle.site.template.json').read_text());_,cycles=parse_profile(profile)
 inputs=json.loads((root/'policy/demo_inputs/site_thirteen_measured.json').read_text());config=yaml.safe_load(scene.read_text())
 run=BridgeCompiler(args.output)
 try:
  run.initialize(inputs['boxes'],Hull.from_config(config),NS(source_root=DEFAULT_SOURCE,scene_config=scene,device='cuda:0'))
  run.ik=FullFloorWorkspace(DEFAULT_SOURCE,scene,trace=run.trace)
  target=cycles[3][0];size=target.reference_box_size_m
  run.box_id=profile['cycles'][3]['demo_box_id'];run.current_box_size=size;run.pedestal=pedestal_geometry(config)
  run.high_carry=False;run.high_carry_bottom_z=0.;run.attached=True
  run.grasp_xyz=np.array([0.,0.,size[2]/2]);run.grasp_rotation=Rotation.from_euler('x',np.pi)
  run.box_pose=np.zeros(7)
  for row,(prior,_) in zip(profile['cycles'][:3],cycles[:3]):
   run.placed[row['demo_box_id']]=np.r_[vector(prior.box_pose),quat(prior.box_pose)]
  run.q=np.array(TRANSFER_JOINTS_RAD[1]);run.ik.q_last=run.q.astype(np.float32);run.refresh(run.q)
  tool=run.state.body_q.numpy()[run.contact_body];actual=Pose(Point(*tool[:3].tolist()),Quaternion(*tool[3:].tolist()))
  grasp=((0.,0.,size[2]/2),Quaternion(1,0,0,0));start=box_pose_from_grasp(actual,grasp)
  transform=next(t for t in config['static_transforms'] if t['child']=='container_link')
  container=Pose(Point(*transform['translation']),Quaternion(*Rotation.from_euler('xyz',transform['rotation_rpy']).as_quat()))
  target=lower_final_approach(release_above_support(target));guard=DoorwayMotionGuard(profile)
  target,_=guard.align_target(target,size,container)
  target,report=CenteredDoorwayEntry(guard.geometry,CarryConfig(profile['carry_entry'])).build(target,size,container,[t for t,_ in cycles[:3]],start)
  write_json(args.output/'entry_report.json',report)
  segments=build_place_segments(target,NS(frame='world',yaw_valid=True),NS(frame='world'),measured_grasp=grasp)
  run.checking=True;run.enter('CARRY');run.frame();last=actual
  for segment in segments:
   if segment.name=='place_descend':break
   goal=segment.target_pose;rotations=Slerp([0,1],Rotation.from_quat([quat(last),quat(goal)]))
   count=max(12,int(np.ceil(np.linalg.norm(vector(goal)-vector(last))/.015)))
   for alpha in np.linspace(0,1,count+1)[1:]:
    point=vector(last)+alpha*(vector(goal)-vector(last));rotation=rotations(alpha).as_quat()
    ok,q=run.ik.solve_world_pose(point,0.,orientation_xyzw=rotation)
    if not ok:raise RuntimeError('live entry IK failed '+segment.name+' '+str(run.ik.last_diagnostics))
    run.q=q;run.frame()
   last=goal
  write_json(args.output/'probe_result.json',dict(success=True,frames=len(run.history),scope='box4 live entry, nominal P2 grasp'))
  return 0
 except BaseException as exc:
  write_json(args.output/'probe_result.json',dict(success=False,error=str(exc),frames=len(run.history)))
  return 1
 finally:run.close()
if __name__=='__main__':raise SystemExit(main())
