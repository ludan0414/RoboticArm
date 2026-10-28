"""Offline Newton preparation of a 32-case full-floor candidate demo."""
import argparse
import json
import shutil
import time
from pathlib import Path
import numpy as np
import warp as wp
import yaml

from demo_contract import write_json, freeze_bundle
from demo_workspace import CertifiedWorkspace
from demo_greedy import greedy_sequence
from demo_trajectory import compile_trajectory
from doorway_scene import clearance_hull, DoorwayGeometry
from full_floor_layout import full_floor_cells
from online_packing_real import Hull
from prepare_greedy_demo import model_manifest
from roboticarm_newton import DEFAULT_SOURCE


class FullFloorWorkspace(CertifiedWorkspace):
    """Use the motion compiler's Cartesian sampling for candidate routes."""

    def route_is_feasible(self, route, dims, yaw, hull):
        points = [self.local_contact_world(p, dims, yaw, hull) for p in route]
        if not points:
            return False
        ok, _ = self.solve_world_pose(*points[0])
        if not ok:
            return False
        for (start, _), (end, world_yaw) in zip(points, points[1:]):
            count = max(12, int(np.ceil(float(np.linalg.norm(end-start)) / .015)))
            for alpha in np.linspace(0, 1, count + 1)[1:]:
                smooth = alpha * alpha * (3 - 2 * alpha)
                ok, _ = self.solve_world_pose(start + smooth * (end-start), world_yaw)
                if not ok:
                    return False
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene-config', type=Path, default=Path('../deployment_ws/config/scene_tf.loading_site.yaml'))
    parser.add_argument('--source-root', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--count', type=int, default=32)
    parser.add_argument('--allow-partial', action='store_true',
                        help='compile a nonempty partial plan when the candidate grid blocks')
    args = parser.parse_args()
    args.source_root, args.scene_config = args.source_root.resolve(), args.scene_config.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    stage, start = 'INPUT', time.monotonic()
    trace = (args.output / 'offline_boundary.jsonl').open('w', encoding='utf-8')
    try:
        manifest = dict(schema_version=1, units='m', boxes=[[.55, .35, .2] for _ in range(args.count)],
                        box_labels=[f'Box {i+1:02d}' for i in range(args.count)])
        write_json(args.output/'input.json', manifest)
        shutil.copyfile(args.scene_config, args.output/'scene_tf.yaml')
        scene = yaml.safe_load(args.scene_config.read_text(encoding='utf-8'))
        physical_hull = Hull.from_config(scene)
        hull = clearance_hull(physical_hull, scene)
        cells = full_floor_cells(physical_hull)
        write_json(args.output/'layout_candidates.json', [vars(c) for c in cells])
        write_json(args.output/'model_manifest.json', model_manifest(args.source_root, args.scene_config))
        wp.set_device(args.device)
        workspace = FullFloorWorkspace(args.source_root, args.scene_config, device=args.device,
                                       box_size=manifest['boxes'][0], trace=trace)
        stage = 'EXPLORE_DOMAIN'
        accepted, rejected = [], []
        for index, cell in enumerate(cells):
            local = [cell.x, cell.y, cell.top]
            position, yaw = workspace.local_contact_world(local, (0,0,0), cell.yaw, hull)
            ok, q = workspace.solve_world_pose(position, yaw)
            row = dict(contact_local_m=local, contact_world_m=position.tolist(), box_yaw_rad=cell.yaw,
                       tool_orientation_xyzw=workspace.tool_orientation(yaw).tolist(),
                       diagnostics=dict(workspace.last_diagnostics))
            if ok:
                row['joint_q'] = q.tolist()
                accepted.append(row)
            else:
                row['collisions'] = workspace.last_collision_pairs
                rejected.append(row)
            # Preserve partial exploration if the process is interrupted.
            write_json(args.output/'domain_partial.json', dict(accepted=accepted, rejected=rejected))
            print(f'EXPLORE_DOMAIN {index+1}/{len(cells)} accepted={len(accepted)}', flush=True)
        domain = dict(schema_version=1, representation='finite_certified_poses',
                      interpolation_allowed=False, frame='container_door_floor',
                      tested=len(cells), accepted=len(accepted), poses=accepted,
                      rejected=rejected, scope='static robot/tool only; payload checked during routing')
        write_json(args.output/'domain.json', domain)
        stage = 'PLAN'
        rows = greedy_sequence(manifest['boxes'], hull, domain, workspace,
                               feeder_center=(-.345, .5, .55), allow_partial=args.allow_partial)
        for row in rows:
            row['label'] = manifest['box_labels'][row['input_index']]
        plan = dict(schema_version=1, policy='full_floor_candidates_back_to_front',
                    position_frame='world', route_frame='container_door_floor',
                    sequence=rows, doorway_clearance=DoorwayGeometry.from_scene(scene).settings())
        write_json(args.output/'plan.json', plan)
        stage = 'COMPILE_TRAJECTORY'
        # Later candidate searches change the workspace's last IK seed.
        # Start the first pick with the branch certified at its feeder route,
        # rather than whichever branch happened to finish the final search.
        workspace.q_last = np.asarray(rows[0]['feeder_joint_q'], dtype=np.float32)
        validation = compile_trajectory(args, manifest, physical_hull, rows, workspace)
        write_json(args.output/'validation.json', validation)
        if not validation.get('success'):
            raise RuntimeError('trajectory validation did not pass')
        freeze_bundle(args.output)
        write_json(args.output/'prepare_result.json', dict(success=True, boxes=len(rows),
                   requested_boxes=args.count, partial=len(rows) < args.count,
                   elapsed_sec=time.monotonic()-start))
        print(f'READY_FOR_REPLAY: {args.output}', flush=True)
        return 0
    except BaseException as exc:
        write_json(args.output/'prepare_result.json', dict(success=False, stage=stage, error=str(exc),
                   elapsed_sec=time.monotonic()-start, replay_ready=False))
        print(f'FAILED {stage}: {exc}', flush=True)
        return 1
    finally:
        trace.close()

if __name__ == '__main__':
    raise SystemExit(main())
