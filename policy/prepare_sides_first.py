"""Reorder a measured layout and regenerate routes against earlier cargo.

No READY marker is written here. compile_spaced_full.py must check every
frame, including reverse returns, before this plan is replayable.
"""
import argparse
import copy
import json
import shutil
from pathlib import Path

from demo_contract import check_bundle, write_json
from demo_feeder import feeder_route
from doorway_scene import clearance_hull
from online_packing_demo import Box
from online_packing_real import Hull
from three_stack_layout import STACKS, STACK_LOADING_ORDER
import yaml


def sides_first(rows):
    """Preserve bottom-up ordering and physical box identity in each aisle."""
    groups = {label: group for group, names in enumerate(STACKS) for label in names}
    rank = {group: index for index, group in enumerate(STACK_LOADING_ORDER)}
    ordered = sorted(copy.deepcopy(rows), key=lambda row: (
        rank[groups[row['label']]], row['stack_layer']))
    for step, row in enumerate(ordered, 1):
        row['step'] = step
    return ordered


def prepare(source, output):
    plan = check_bundle(source, allow_partial=True)
    output.mkdir(parents=True, exist_ok=False)
    for name in ('input.json', 'domain.json', 'scene_tf.yaml', 'model_manifest.json'):
        shutil.copyfile(source / name, output / name)
    hull = clearance_hull(Hull.from_config(yaml.safe_load(
        (output / 'scene_tf.yaml').read_text(encoding='utf-8'))),
        yaml.safe_load((output / 'scene_tf.yaml').read_text(encoding='utf-8')))

    class GeometryOnly:
        # Numerical/robot checks belong to the full compiler, never to this
        # geometry prepass. Its output is deliberately not certified.
        @staticmethod
        def box_yaw(source_dims, dims):
            from newton_ik_planner import box_yaw
            return box_yaw(source_dims, dims)

        @staticmethod
        def route_is_feasible(*args):
            return True

    rows = sides_first(plan['sequence'])
    placed = []
    for row in rows:
        dims = row['placed_dims_m']
        center = row['center_container_door_floor_m']
        box = Box(row['id'], tuple(dims), tuple(center[k]-dims[k]/2 for k in range(3)))
        route = feeder_route(box, placed, hull, GeometryOnly(), row['input_dims_m'],
                             row['spawn_center_container_door_floor_m'])
        if route is None:
            write_json(output / 'failure.json', dict(label=row['label'], stage='geometry_route'))
            raise ValueError('No cargo-clear route for ' + row['label'])
        row['route'] = route
        placed.append(box)
    plan.update(sequence=rows, policy='three_stack_sides_before_centre')
    write_json(output / 'plan.json', plan)
    write_json(output / 'prepare_result.json', dict(stage='GEOMETRY_ONLY', replay_ready=False,
        order=[row['label'] for row in rows]))
    return plan


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.output)
