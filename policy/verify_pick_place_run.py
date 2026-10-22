"""Inspect a saved Newton pipeline run without importing Newton or using GPU."""
import argparse
import json
from pathlib import Path

import numpy as np


def quaternion_product(first, second):
    xyz = (first[:, 3:] * second[:, :3] + second[:, 3:] * first[:, :3]
           + np.cross(first[:, :3], second[:, :3]))
    w = first[:, 3] * second[:, 3] - np.sum(first[:, :3] * second[:, :3], axis=1)
    return np.column_stack((xyz, w))


BOX_STAGES = ["SPAWN", "APPROACH", "PICK", "ATTACH", "LIFT", "CARRY",
              "PLACE", "RELEASE", "RETREAT", "VERIFY"]


def verify(path):
    path = Path(path)
    result = json.loads((path / "result.json").read_text(encoding="utf-8"))
    if not result["success"]:
        raise ValueError(f"run failed at {result.get('failed_stage')}: {result.get('error')}")
    events = [json.loads(line) for line in (path / "stages.jsonl").read_text(encoding="utf-8").splitlines()]
    plan = json.loads((path / "plan.json").read_text(encoding="utf-8"))
    if isinstance(plan, dict):
        plan = plan["sequence"]
    if events[-1]["stage"] != "COMPLETE" or result["placed"] != len(plan):
        raise ValueError("incomplete manifest")
    with np.load(path / "trajectory.npz") as data:
        if not np.isfinite(data["joint_q"]).all() or not np.isfinite(data["tool_pose"]).all():
            raise ValueError("nonfinite arm state")
        max_grasp_drift = 0.0
        for row in plan:
            box_id = row["id"]
            stages = [event["stage"] for event in events
                      if event["box_id"] == box_id and event["stage"] in BOX_STAGES]
            if stages != BOX_STAGES:
                raise ValueError(f"{box_id}: wrong stage order {stages}")
            # RESET_HOME already has the next box ID, but that box does not
            # exist yet; its trace uses NaN as an absent-payload sentinel.
            mask = (data["box_id"] == box_id) & np.isin(data["stage"], BOX_STAGES)
            carried = mask & data["attached"]
            if not carried.any():
                raise ValueError(f"{box_id}: no attached frames")
            if not np.isfinite(data["box_pose"][mask]).all():
                raise ValueError(f"{box_id}: nonfinite payload")
            tool = data["tool_pose"][carried]
            box = data["box_pose"][carried]
            tool_q = tool[:, 3:].astype(np.float64)
            inv = tool_q / np.linalg.norm(tool_q, axis=1, keepdims=True)
            inv[:, :3] *= -1
            delta = box[:, :3] - tool[:, :3]
            offsets = delta + 2 * np.cross(inv[:, :3],
                np.cross(inv[:, :3], delta) + inv[:, 3:] * delta)
            box_q = box[:, 3:].astype(np.float64)
            box_q /= np.linalg.norm(box_q, axis=1, keepdims=True)
            rotations = quaternion_product(inv, box_q)
            rotations /= np.linalg.norm(rotations, axis=1, keepdims=True)
            drift = float(np.max(np.linalg.norm(offsets - offsets[0], axis=1)))
            first_inverse = rotations[:1].copy()
            first_inverse[:, :3] *= -1
            delta_q = quaternion_product(np.repeat(first_inverse, len(rotations), axis=0), rotations)
            rotation_drift = float(np.max(2 * np.arctan2(
                np.linalg.norm(delta_q[:, :3], axis=1), np.abs(delta_q[:, 3]))))
            if drift > 1e-5 or rotation_drift > 1e-5:
                raise ValueError(f"{box_id}: grasp slipped {drift} m / {rotation_drift} rad")
            max_grasp_drift = max(max_grasp_drift, drift)
            released = mask & np.isin(data["stage"], ["RELEASE", "RETREAT", "VERIFY"])
            if not released.any() or data["attached"][released].any():
                raise ValueError(f"{box_id}: bad release state")
            payload = data["box_pose"][released]
            if not np.allclose(payload, payload[0], atol=1e-6):
                raise ValueError(f"{box_id}: payload moved after release")
            verified = [e for e in events if e["stage"] == "VERIFY_RESULT" and e["box_id"] == box_id]
            if len(verified) != 1 or verified[0]["position_error_m"] > 0.006 or verified[0]["rotation_error_rad"] > 0.07:
                raise ValueError(f"{box_id}: placement error exceeds gate")
        return dict(success=True, boxes=len(plan), frames=len(data["stage"]),
                    max_grasp_drift_m=max_grasp_drift)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.run), indent=2))
