"""Re-certify saved witnesses and motions without repeating policy or IK search.

Useful when tightening a collision validator. Writes a new immutable bundle;
the original rehearsal is preserved.
"""
import argparse
import json
from pathlib import Path
import shutil

from container_demo import FRAME_DT
import newton
import numpy as np
import warp as wp
import yaml
from demo_contract import freeze_bundle, write_json
from demo_recording import validate_recording
from demo_workspace import CertifiedWorkspace, recheck_domain
from online_packing_real import Hull
from prepare_greedy_demo import model_manifest
from roboticarm_newton import DEFAULT_SOURCE
from verify_pick_place_run import verify


def audit(source, output, source_root=DEFAULT_SOURCE):
    source, output = Path(source), Path(output)
    result = json.loads((source / "result.json").read_text(encoding="utf-8"))
    if not result["success"] or result["placed"] != 10:
        raise ValueError("cannot publish an incomplete rehearsal")
    output.mkdir(parents=True, exist_ok=False)
    for name in ("input.json", "scene_tf.yaml", "trajectory.npz", "stages.jsonl", "result.json"):
        shutil.copyfile(source / name, output / name)
    shutil.copytree(source / "inputs", output / "inputs")
    trace = (output / "audit_boundary.jsonl").open("w", encoding="utf-8")
    try:
        scene = output / "scene_tf.yaml"
        identity = model_manifest(source_root.resolve(), scene)
        prior = json.loads((source / "model_manifest.json").read_text(encoding="utf-8"))
        for key in ("urdf_sha256", "scene_sha256", "meshes", "srdf_sha256"):
            if prior[key] != identity[key]:
                raise ValueError("source model differs: " + key)
        write_json(output / "model_manifest.json", identity)
        wp.set_device("cuda:0")
        workspace = CertifiedWorkspace(source_root, scene, device="cuda:0", trace=trace)
        hull = Hull.from_config(yaml.safe_load(scene.read_text(encoding="utf-8")))
        domain = recheck_domain(workspace, hull, source / "domain.json", output / "domain.json")
        domain["model_identity"] = identity["identity"]
        write_json(output / "domain.json", domain)
        original_domain = json.loads((source / "domain.json").read_text(encoding="utf-8"))
        lookup = {(tuple(p["contact_local_m"]), p["box_yaw_rad"]): i for i, p in enumerate(domain["poses"])}
        plan = json.loads((source / "plan.json").read_text(encoding="utf-8"))
        for row in plan["sequence"]:
            certificate = original_domain["poses"][row["domain_certificate"]]
            key = (tuple(certificate["contact_local_m"]), certificate["box_yaw_rad"])
            if key not in lookup:
                raise ValueError("target failed tightened domain check: " + row["id"])
            row["domain_certificate"] = lookup[key]
            row["center_container_door_floor_m"] = row.pop("center_m", row.get("center_container_door_floor_m"))
        plan.update(route_frame="container_door_floor", position_frame="world")
        write_json(output / "plan.json", plan)
        validation = verify(output)
        with np.load(output / "trajectory.npz") as data:
            validation["scene_recording"] = validate_recording(plan["sequence"], data)
            robot_indices = [list(data["body_labels"]).index(name) for name in workspace.model.body_label]
            checked, unique = 0, set()
            for frame, joints in enumerate(data["joint_q"]):
                if str(data["box_id"][frame]) == "":
                    continue  # unexecuted pre-reset model initialization
                key = joints.astype(np.float32).tobytes()
                if key not in unique:
                    workspace.model.joint_q.assign(joints)
                    newton.eval_fk(workspace.model, workspace.model.joint_q, workspace.model.joint_qd, workspace.model)
                    actual = workspace.model.body_q.numpy()
                    recorded = data["body_q"][frame, robot_indices]
                    if not np.allclose(actual[:, :3], recorded[:, :3], atol=1e-5):
                        raise ValueError("recorded robot FK mismatch at frame " + str(frame))
                    if not workspace._collision_free():
                        write_json(output / "audit_failure.json", dict(frame=frame, joint_q=joints.tolist(),
                                   collisions=workspace.last_collision_pairs))
                        raise ValueError("robot collision at recorded frame " + str(frame))
                    unique.add(key)
                checked += 1
                if checked % 400 == 0:
                    print(f"AUDIT_MOTION {checked}/{len(data['joint_q'])}", flush=True)
        validation.update(robot_scene_self_checked_frames=checked, unique_joint_states=len(unique),
                          source_rehearsal=str(source), replay_planning_calls=0)
        write_json(output / "validation.json", validation)
        freeze_bundle(output)
        write_json(output / "prepare_result.json", dict(success=True, stage="READY_FOR_REPLAY", boxes=10,
                   source_rehearsal=str(source), certified_domain_poses=domain["accepted"]))
        return validation
    except BaseException as exc:
        write_json(output / "prepare_result.json", dict(success=False, failed_stage="AUDIT", error=str(exc)))
        raise
    finally:
        trace.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    audit(args.source, args.output)
