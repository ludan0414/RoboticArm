"""Measure one greedy placement without changing the planner."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import yaml

import online_packing_real
from newton_ik_planner import NewtonIKWorkspace
from packing_policy import _load_manifest, plan
from roboticarm_newton import DEFAULT_SOURCE, scene_paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path(__file__).with_name("packing_policy_input_example.json"))
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    args = parser.parse_args()
    boxes = _load_manifest(args.input)
    scene = scene_paths(DEFAULT_SOURCE)[1]
    hull = online_packing_real.Hull.from_config(yaml.safe_load(scene.read_text(encoding="utf-8")))
    start = time.perf_counter()
    checker = NewtonIKWorkspace(DEFAULT_SOURCE, scene, device=args.device, box_size=boxes[0])
    print(json.dumps({"stage": "init", "seconds": round(time.perf_counter() - start, 3)}), flush=True)
    original_ik = checker.solve_world_pose
    original_route = online_packing_real.feasible_route
    metrics = {"ik_calls": 0, "ik_failures": 0, "ik_seconds": 0.0,
               "route_calls": 0, "route_seconds": 0.0, "first_failures": []}

    def measured_ik(*call_args, **call_kwargs):
        start = time.perf_counter()
        result = original_ik(*call_args, **call_kwargs)
        metrics["ik_seconds"] += time.perf_counter() - start
        metrics["ik_calls"] += 1
        metrics["ik_failures"] += not result[0]
        if not result[0] and len(metrics["first_failures"]) < 8:
            metrics["first_failures"].append({"target": [float(v) for v in call_args[0]],
                                               "diagnostics": dict(checker.last_diagnostics)})
        return result

    def measured_route(*call_args, **call_kwargs):
        start = time.perf_counter()
        result = original_route(*call_args, **call_kwargs)
        metrics["route_seconds"] += time.perf_counter() - start
        metrics["route_calls"] += 1
        return result

    checker.solve_world_pose = measured_ik
    online_packing_real.feasible_route = measured_route
    start = time.perf_counter()
    rows = []
    try:
        rows = plan(boxes[:1], hull, checker)
    finally:
        print(json.dumps({"stage": "plan", "seconds": round(time.perf_counter() - start, 3),
                          "placements": len(rows), **metrics}), flush=True)
        online_packing_real.feasible_route = original_route


if __name__ == "__main__":
    main()
