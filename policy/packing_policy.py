"""Sequence-to-placement policy inference.

Input is a complete ordered manifest of box dimensions. Output is a reordered
placement plan in container-local coordinates. Every emitted placement is
selected from candidates that pass boundary, collision, support, orientation,
and swept-entry checks.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
import yaml

from online_packing_demo import Box
from online_packing_real import Hull, candidate_positions
from packing_rl import (ScoreNet, candidate_features, placement_candidates,
                        ACTION_FEATURE_COUNT, BOX_FEATURE_COUNT)


def _load_manifest(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("boxes", data.get("sequence"))
    if not isinstance(data, list) or not data:
        raise ValueError("input JSON must be a non-empty list or {'boxes': [...]} ")
    boxes = []
    for index, dims in enumerate(data):
        if not isinstance(dims, (list, tuple)) or len(dims) != 3:
            raise ValueError(f"box {index} must contain exactly three dimensions")
        values = tuple(float(value) for value in dims)
        if min(values) <= 0:
            raise ValueError(f"box {index} dimensions must be positive")
        boxes.append(values)
    return boxes


def _pose_orientation(yaw_rad):
    """Return NewtonIKWorkspace's tool/box yaw convention as a quaternion."""
    half = float(yaw_rad) / 2.0
    return {"yaw_deg": float(np.rad2deg(yaw_rad)),
            "quaternion_xyzw": [0.0, 0.0, float(np.sin(half)), float(np.cos(half))]}


def _make_policy(checkpoint: Path | None, device):
    if checkpoint is None:
        return None
    saved = torch.load(checkpoint, map_location=device, weights_only=False)
    policy = ScoreNet(int(saved["hidden"])).to(device)
    policy.load_state_dict(saved["state_dict"])
    policy.eval()
    return policy


def plan(boxes, hull: Hull, ik_checker, policy=None, device="cpu", candidates=12,
         ik_x_step=0.15, allow_partial=False):
    """Return an IK- and geometry-validated plan for a complete manifest."""
    if ik_checker is None:
        raise ValueError("IK checker is mandatory; refusing geometry-only planning")
    device = torch.device(device)
    remaining = list(range(len(boxes)))
    placed = []
    plan_rows = []
    scale = max(hull.length, hull.width, hull.height)
    manifest_features = torch.as_tensor(
        np.asarray([np.asarray(dims, dtype=np.float32) / scale for dims in boxes]),
        device=device)
    while remaining:
        actions = []
        # The non-learned baseline is volume-first. Searching every remaining
        # box's IK routes at every step is redundant: as soon as the largest
        # box with a feasible action is found, smaller boxes cannot win the
        # baseline's primary score. A learned scorer still evaluates all boxes.
        search_order = (sorted(remaining,
                               key=lambda i: (-float(np.prod(boxes[i])), i))
                        if policy is None else remaining)
        for index in search_order:
            for candidate, route in candidate_positions(
                    boxes[index], placed, hull,
                    ik_checker=ik_checker,
                    source_dims=boxes[index],
                    ik_x_step=ik_x_step,
                    candidate_limit=None,
                    feasible_candidate_limit=1 if policy is None else candidates,
                    rank_by_depth=True,
                    max_face_down=True,
                    min_entry_z=0.4):
                features = candidate_features(
                    boxes[index], candidate, placed,
                    [boxes[item] for item in remaining], hull)
                actions.append((index, candidate, route, features))
            if actions and policy is None:
                break
        if not actions:
            if allow_partial:
                break
            raise RuntimeError(
                f"no feasible placement remains after {len(placed)} boxes; "
                "increase candidate coverage or enable relocation planning")

        if policy is None:
            selected = max(actions, key=lambda row: (
                float(np.prod(row[1].dims)),
                (row[1].pos[0] + row[1].dims[0] / 2) / hull.length,
                -row[1].pos[2], -row[1].pos[1]))
        else:
            action_features = torch.as_tensor(np.asarray([
                np.concatenate((np.asarray(boxes[index], dtype=np.float32) / scale, features))
                for index, _, _, features in actions], dtype=np.float32), device=device)
            with torch.no_grad():
                placed_features = torch.as_tensor(np.asarray([
                    [*item.dims, *item.pos] for item in placed], dtype=np.float32
                ).reshape(-1, 6) / scale, device=device)
                scores = policy(manifest_features, placed_features, action_features)
            selected = actions[int(torch.argmax(scores).item())]

        index, candidate, route, _ = selected
        box = Box(f"B{index + 1}", candidate.dims, candidate.pos)
        center = tuple(float(candidate.pos[k] + candidate.dims[k] / 2) for k in range(3))
        plan_rows.append({
            "step": len(plan_rows) + 1,
            "input_index": index,
            "id": box.id,
            "input_dims_m": list(boxes[index]),
            "placed_dims_m": list(candidate.dims),
            "center_m": list(center),
            "orientation": _pose_orientation(ik_checker.box_yaw(boxes[index], candidate.dims)),
            "route": route,
        })
        placed.append(box)
        remaining.remove(index)
    return plan_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True,
                        help="JSON list of [length, width, height] in meters")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--source-root", type=Path,
                        default=Path(__file__).resolve().parent / "vendor" / "roboticarm")
    parser.add_argument("--candidates", type=int, default=12)
    parser.add_argument("--ik-x-step", type=float, default=0.15)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    args = parser.parse_args()
    if args.scene_config is None:
        args.scene_config = args.source_root / "elfin_humble_ws" / "src" / "luggage_description" / "config" / "scene_tf.yaml"
    if args.candidates < 1:
        parser.error("--candidates must be positive")
    boxes = _load_manifest(args.input)
    scene_data = yaml.safe_load(args.scene_config.read_text(encoding="utf-8"))
    hull = Hull.from_config(scene_data)
    from newton_ik_planner import DEFAULT_BOX_SIZE, NewtonIKWorkspace

    checker = NewtonIKWorkspace(
        args.source_root, args.scene_config, device=args.device,
        box_size=boxes[0] if boxes else DEFAULT_BOX_SIZE)
    policy = _make_policy(args.checkpoint, args.device)
    rows = plan(boxes, hull, checker, policy, args.device, args.candidates,
                args.ik_x_step)
    output = {"container": hull.as_dict(), "coordinate_frame": "container_local", "placements": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(args.output), "placements": len(rows),
                      "policy": "checkpoint" if policy else "volume_depth_baseline"}, indent=2))


if __name__ == "__main__":
    main()
