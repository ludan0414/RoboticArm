"""Replay a placement plan with manually staged boxes in Newton.

The planner runs separately. This command only reads its JSON output.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from online_packing_real import Hull, validate_sample


def sample_from_plan(plan: dict) -> dict:
    if plan.get("coordinate_frame") != "container_local":
        raise ValueError("plan must use container_local coordinates")
    hull = Hull.from_dict(plan["container"])
    rows = plan["placements"]
    if not rows:
        raise ValueError("plan has no placements")
    placed = []
    steps = []
    for number, row in enumerate(rows, 1):
        if row["step"] != number:
            raise ValueError("placement steps must be consecutive")
        route = row["route"]
        box = {"id": row["id"], "dims": row["placed_dims_m"],
               "pos": route[-1]}
        placed.append(box)
        steps.append({
            "incoming": {"id": row["id"], "dims": row["input_dims_m"],
                         "input_index": row["input_index"]},
            "actions": [{"kind": "place", "box": box, "route": route}],
            "status": "placed", "boxes": list(placed),
            "utilization": sum(float(b["dims"][0] * b["dims"][1] * b["dims"][2])
                               for b in placed) / hull.volume,
        })
    sample = {"seed": 1, "container": plan["container"], "steps": steps,
              "relocations": 0, "policy": "manual_loading",
              "manual_loading": True, "ik": {"enabled": True}}
    validate_sample(sample)
    return sample


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--mode", choices=("show", "inspect", "bench"), default="show")
    args = parser.parse_args()
    sample = sample_from_plan(json.loads(args.plan.read_text(encoding="utf-8")))
    print(json.dumps({"loading_order": [
        {"step": i, "input_number": step["incoming"]["input_index"] + 1,
         "box_id": step["incoming"]["id"]}
        for i, step in enumerate(sample["steps"], 1)]}, indent=2), flush=True)
    with tempfile.TemporaryDirectory(prefix="newton_loading_") as directory:
        path = Path(directory) / "sample.json"
        path.write_text(json.dumps([sample]), encoding="utf-8")
        command = [sys.executable, str(Path(__file__).with_name("online_packing_newton.py")),
                   "--samples", str(path), "--mode", args.mode, "--device", args.device,
                   "--once"]
        if args.scene_config:
            command += ["--scene-config", str(args.scene_config)]
        return subprocess.call(command, cwd=Path(__file__).resolve().parent)


if __name__ == "__main__":
    raise SystemExit(main())
