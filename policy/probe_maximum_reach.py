"""Measure reach projection with the real Newton URDF and scene collisions."""
import argparse
import json
from pathlib import Path
import numpy as np
import newton
from demo_contract import write_json
from prepare_full_floor_demo import FullFloorWorkspace
from roboticarm_newton import DEFAULT_SOURCE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scene-config',type=Path,default=Path('../deployment_ws/config/scene_tf.loading_site.yaml'))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    with (args.output/'boundary.jsonl').open('w',encoding='utf-8') as trace:
        workspace=FullFloorWorkspace(DEFAULT_SOURCE,args.scene_config.resolve(),trace=trace)
        target=workspace.shoulder_world+np.array([3.2,0.,.2])
        exact,_=workspace.solve_world_pose(target,0.)
        ok,q=workspace.solve_reachable_world_pose(target,0.)
        result=dict(requested_position_m=target.tolist(),original_reachable=bool(exact),projected=bool(ok),
                    diagnostics=dict(workspace.last_diagnostics))
        if ok:
            workspace.model.joint_q.assign(q)
            newton.eval_fk(workspace.model,workspace.model.joint_q,workspace.model.joint_qd,workspace.model)
            actual=workspace.model.body_q.numpy()[workspace.contact_body]
            result.update(actual_tool_pose=actual.tolist(),joint_degrees=np.degrees(q).tolist(),
                          actual_shortfall_m=float(np.linalg.norm(actual[:3]-target)),
                          collision_free=bool(workspace._collision_free()),
                          elbow_extension_error_deg=abs(float(np.degrees(q[2]))))
        result['success']=bool(not exact and ok and result.get('collision_free')
            and result['diagnostics'].get('reach_projected')
            and result['diagnostics']['position_error_m']<=workspace.position_tolerance_m
            and result['diagnostics']['rotation_error_rad']<=.035
            and result['elbow_extension_error_deg']<=1.)
        write_json(args.output/'result.json',result)
        print(json.dumps(result),flush=True)
        return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
