"""Extract a completed, checked box from durable trace and open Newton.

FK reconstruction only: no new IK or motion search, no robot commands.
"""
import argparse
import json
from pathlib import Path
import shutil

import newton
import numpy as np
import warp as wp
import yaml
from scipy.spatial.transform import Rotation
from demo_contract import write_json, freeze_bundle
from demo_recording import validate_recording
from online_packing_newton import make_model
from online_packing_real import Hull
from replay_prepared_demo import replay
from roboticarm_newton import DEFAULT_SOURCE
from verify_pick_place_run import verify


def extract(source, output, box_id):
    bridges = json.loads((source / 'bridges.json').read_text())
    if box_id not in [b['id'] for b in bridges]:
        raise ValueError('box has not completed checked placement and return')
    output.mkdir(parents=True, exist_ok=False)
    for name in ('input.json', 'domain.json', 'scene_tf.yaml', 'model_manifest.json'):
        shutil.copyfile(source / name, output / name)
    plan = json.loads((source / 'plan.json').read_text())
    plan['sequence'] = [r for r in plan['sequence'] if r['id'] == box_id]
    write_json(output / 'plan.json', plan)
    events = [json.loads(s) for s in (source / 'stages.jsonl').read_text().splitlines()]
    events = [e for e in events if e.get('box_id') == box_id]
    events.append(dict(stage='COMPLETE', box_id=box_id))
    (output / 'stages.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    rows = []
    for line in (source / 'boundary.jsonl').read_text().splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue  # final write may be incomplete in a live source
        if 'frame' in row and row.get('box_id') == box_id:
            rows.append(row)
    # Reused placement frames are stored in the immutable base recording,
    # not duplicated in the live boundary trace. Splice those exact joints.
    bridge = next(b for b in bridges if b['id'] == box_id)
    base = Path(__file__).parent / 'demo_data/site55_greedy10'
    with np.load(base / 'trajectory.npz') as reference:
        start, end = bridge['original_frame_range']
        offset = bridge['reused_output_frame_range'][0]
        rows.extend(dict(frame=offset+i-start, box_id=box_id, stage=str(reference['stage'][i]),
                         attached=bool(reference['attached'][i]), joint_q=reference['joint_q'][i].tolist(),
                         box_pose=None, reference=True) for i in range(start, end))
    rows.sort(key=lambda row: row['frame'])
    if not rows or rows[-1]['stage'] != 'VERIFY':
        raise ValueError('missing completed return/verify frames')
    inputs = json.loads((output / 'input.json').read_text())
    hull = Hull.from_config(yaml.safe_load((output / 'scene_tf.yaml').read_text()))
    sample = dict(container=hull.as_dict(), steps=[dict(incoming=dict(id=f'B{i+1}', dims=d))
                                                  for i, d in enumerate(inputs['boxes'])])
    wp.set_device('cuda:0')
    model, ids, _, _ = make_model(sample, DEFAULT_SOURCE, output / 'scene_tf.yaml')
    state = model.state()
    contact = model.body_label.index('S20/suction_contact_frame')
    bodies, tools = [], []
    last_pose = last_tool = frozen = grasp_xyz = grasp_rotation = None
    for row in rows:
        state.joint_q.assign(np.asarray(row['joint_q']))
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
        poses = state.body_q.numpy()
        tool = poses[contact]
        if row.get('reference'):
            if grasp_xyz is None:
                inverse = Rotation.from_quat(last_tool[3:]).inv()
                grasp_xyz = inverse.apply(last_pose[:3] - last_tool[:3])
                grasp_rotation = inverse * Rotation.from_quat(last_pose[3:])
            if row['attached']:
                rot = Rotation.from_quat(tool[3:])
                payload = np.r_[tool[:3] + rot.apply(grasp_xyz), (rot * grasp_rotation).as_quat()]
            else:
                if frozen is None:
                    frozen = last_pose.copy()
                payload = frozen
            row['box_pose'] = payload.tolist()
        for body in ids.values():
            poses[body] = [100, 100, 100, 0, 0, 0, 1]
        if row['box_pose'] is not None:
            poses[ids[box_id]] = row['box_pose']
            last_pose = np.asarray(row['box_pose'])
        last_tool = tool.copy()
        bodies.append(poses)
        tools.append(poses[contact])
    np.savez_compressed(output / 'trajectory.npz',
        joint_q=np.asarray([r['joint_q'] for r in rows]), tool_pose=np.asarray(tools),
        box_pose=np.asarray([r['box_pose'] if r['box_pose'] is not None else [np.nan]*7 for r in rows]),
        stage=np.asarray([r['stage'] for r in rows]), box_id=np.asarray([box_id]*len(rows)),
        attached=np.asarray([r['attached'] for r in rows]), body_q=np.asarray(bodies),
        body_labels=np.asarray(model.body_label))
    write_json(output / 'result.json', dict(success=True, placed=1))
    validation = verify(output)
    with np.load(output / 'trajectory.npz') as data:
        validation['scene_recording'] = validate_recording(plan['sequence'], data)
    validation['collision_check_provenance'] = dict(
        source=str(source.resolve()), completed_box=box_id,
        method='reuse completed checked source frames; reconstruct FK only')
    write_json(output / 'validation.json', validation)
    freeze_bundle(output)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    folder = args.output if (args.output / 'READY.json').exists() else extract(args.source, args.output, 'B1')
    replay(folder, manual_pick=True, allow_partial=True)
