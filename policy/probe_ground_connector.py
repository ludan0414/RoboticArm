"""Replay one preserved ground-carry boundary without repeating earlier boxes."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from demo_contract import write_json
from ground_carry_clearance import pedestal_geometry
from ground_feeder_bridge import BridgeCompiler
from online_packing_real import Hull
from prepare_full_floor_demo import FullFloorWorkspace
from roboticarm_newton import DEFAULT_SOURCE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    boundary=json.loads(args.capture.read_text(encoding='utf-8'))
    inputs=json.loads((args.bundle/'input.json').read_text(encoding='utf-8'))
    scene=(args.bundle/'scene_tf.yaml').resolve()
    config=yaml.safe_load(scene.read_text(encoding='utf-8'))
    run=BridgeCompiler(args.output)
    try:
        run.initialize(inputs['boxes'],Hull.from_config(config),
            SimpleNamespace(source_root=DEFAULT_SOURCE,scene_config=scene,device='cuda:0'))
        run.ik=FullFloorWorkspace(DEFAULT_SOURCE,scene,trace=run.trace)
        run.current_box_size=boundary['dimensions'];run.box_id=boundary['box_id']
        run.pedestal=pedestal_geometry(config)
        run.high_carry=boundary['high_carry'];run.high_carry_bottom_z=boundary['high_carry_bottom_z']
        run.grasp_xyz=np.asarray(boundary['grasp_xyz'])
        run.grasp_rotation=Rotation.from_quat(boundary['grasp_quat'])
        run.placed={name:np.asarray(pose) for name,pose in boundary['placed'].items()}
        run.attached=True;run.box_pose=np.zeros(7)
        run.refresh(np.asarray(boundary['goal']))
        target=run.state.body_q.numpy()[run.contact_body,:3].copy()
        run.q=np.asarray(boundary['start']);run.ik.q_last=run.q.astype(np.float32)
        run.checking=True;run.enter('CARRY');run.frame()
        run.move(target,float(run.container['rotation_rpy'][2]))
        write_json(args.output/'probe_result.json',dict(success=True,scope='failed connector only',
            frames=len(run.history),endpoint_tool_m=target.tolist()))
        return 0
    except BaseException as exc:
        write_json(args.output/'probe_result.json',dict(success=False,error=str(exc)))
        return 1
    finally:
        run.close()


if __name__=='__main__':raise SystemExit(main())
