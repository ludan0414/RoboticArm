"""Plan a box sequence and replay the result in the Newton viewer.

Examples:
  python run_packing_visual.py --boxes "0.62,0.48,0.24;0.58,0.44,0.22;0.66,0.50,0.25"
  python run_packing_visual.py --input packing_policy_input_example.json --checkpoint packing_policy_stateaware.pt
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml

from online_packing_real import Hull, validate_sample
from packing_policy import _load_manifest, _make_policy, plan


def parse_inline(value: str):
    rows = []
    for item in value.split(";"):
        dims = tuple(float(part.strip()) for part in item.split(","))
        if len(dims) != 3:
            raise ValueError(f"invalid box '{item}'; expected L,W,H")
        rows.append(dims)
    return rows


def make_sample(boxes, hull, rows, policy_name, ik_checker):
    placed = []
    steps = []
    for row in rows:
        box = {
            "id": row["id"],
            "dims": row["placed_dims_m"],
            "pos": list(row["route"][-1]),
        }
        placed.append(box)
        steps.append({
            "incoming": {"id": row["id"], "dims": row["input_dims_m"],
                         "input_index": row["input_index"]},
            "actions": [{"kind": "place", "box": box, "route": row["route"]}],
            "status": "placed",
            "boxes": list(placed),
            "utilization": sum(
                float(b["dims"][0] * b["dims"][1] * b["dims"][2]) for b in placed
            ) / hull.volume,
            "objective": 0.0,
            "relocation_distance_m": 0.0,
            "planning_seconds": 0.0,
        })
    sample = {
        "seed": 1,
        "container": hull.as_dict(),
        "steps": steps,
        "relocations": 0,
        "online": False,
        "policy": policy_name,
        "boxes_input": [list(box) for box in boxes],
        "relocation_depth": 0,
        "relocation_distance_budget_m": 0.0,
        "ik": {
            "enabled": True,
            "solver": "newton.ik.IKSolver",
            "orientation_constraint": "largest_face_down",
            "route_sample_step_m": 0.05,
            "position_tolerance_m": 0.003,
            "rotation_tolerance_rad": 0.035,
            "collision_check": "non-base arm links vs container mesh",
            "box_size_m": list(ik_checker.box_size),
        },
    }
    validate_sample(sample)
    return sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source_root = Path(__file__).resolve().parent / "vendor" / "roboticarm"
    parser.add_argument("--boxes", help="semicolon-separated L,W,H dimensions in meters")
    parser.add_argument("--input", type=Path, help="JSON list or {'boxes': [...]} file")
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("packing_policy_visual_sample.json"))
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--source-root", type=Path, default=source_root)
    parser.add_argument("--candidates", type=int, default=12)
    parser.add_argument("--ik-x-step", type=float, default=0.15)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--no-show", action="store_true", help="only write and validate the sample")
    parser.add_argument("--hide-robot", action="store_true")
    parser.add_argument("--allow-partial", action="store_true",
                        help="visualize boxes placed before the policy gets stuck")
    args = parser.parse_args()
    if bool(args.boxes) == bool(args.input):
        parser.error("provide exactly one of --boxes or --input")
    if args.scene_config is None:
        args.scene_config = args.source_root / "elfin_humble_ws" / "src" / "luggage_description" / "config" / "scene_tf.yaml"
    boxes = parse_inline(args.boxes) if args.boxes else _load_manifest(args.input)
    hull = Hull.from_config(yaml.safe_load(args.scene_config.read_text(encoding="utf-8")))
    from newton_ik_planner import NewtonIKWorkspace

    ik_checker = NewtonIKWorkspace(
        args.source_root, args.scene_config, device=args.device,
        box_size=boxes[0])
    policy = _make_policy(args.checkpoint, args.device)
    rows = plan(boxes, hull, ik_checker, policy, args.device, args.candidates,
                args.ik_x_step, allow_partial=args.allow_partial)
    sample = make_sample(boxes, hull, rows,
                         "checkpoint" if policy else "volume_depth_baseline", ik_checker)
    args.output.write_text(json.dumps([sample], indent=2), encoding="utf-8")
    print(json.dumps({"sample": str(args.output), "boxes": len(rows),
                      "requested_boxes": len(boxes),
                      "partial": len(rows) < len(boxes),
                      "policy": sample["policy"], "validated": True}, indent=2), flush=True)
    if args.no_show:
        return
    command = [sys.executable, str(Path(__file__).with_name("online_packing_newton.py")),
               "--samples", str(args.output), "--mode", "show", "--seed", "1",
               "--device", args.device]
    if args.hide_robot:
        command.append("--hide-robot")
    raise SystemExit(subprocess.call(command, cwd=Path(__file__).resolve().parent))


if __name__ == "__main__":
    main()
