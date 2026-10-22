"""Playback only: load an immutable prepared bundle; never plan or solve IK."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from demo_contract import check_bundle, digest, recorded_urdf_digest, text_digest_matches
from container_demo import make_ground_grid
import newton
import warp as wp
from online_packing_newton import make_model
from roboticarm_newton import DEFAULT_SOURCE, make_robot_urdf, scene_paths


def replay(folder, *, source_root=DEFAULT_SOURCE, headless=False, fps=30, device="cuda:0",
           unvalidated_preview=False, manual_pick=False):
    if manual_pick and headless:
        raise ValueError("manual pick needs the visual window")
    folder, source_root = Path(folder), Path(source_root).resolve()
    plan = (json.loads((folder / "plan.json").read_text(encoding="utf-8"))
            if unvalidated_preview else check_bundle(folder))
    if unvalidated_preview:
        print("UNVALIDATED SIMULATION PREVIEW: no feasibility certificate", flush=True)
    inputs = json.loads((folder / "input.json").read_text(encoding="utf-8"))
    identity = json.loads((folder / "model_manifest.json").read_text(encoding="utf-8"))
    source, _, models = scene_paths(source_root)
    if recorded_urdf_digest(make_robot_urdf(source, models), source_root,
                            identity["source_root"]) != identity["urdf_sha256"]:
        raise ValueError("URDF changed since preparation")
    for path, expected in identity["meshes"].items():
        if digest(source_root / path.replace("\\", "/")) != expected:
            raise ValueError("mesh changed since preparation: " + path)
    srdf = Path(__file__).resolve().parents[1] / "src/luggage_description/config/S20_with_camera.srdf"
    if not text_digest_matches(srdf, identity["srdf_sha256"]):
        raise ValueError("SRDF changed since preparation")
    wp.set_device(device)
    domain = json.loads((folder / "domain.json").read_text(encoding="utf-8"))
    # make_model only needs the hull and input-body definitions. No solver
    # or placement policy is constructed anywhere in this playback path.
    import yaml
    from online_packing_real import Hull
    hull = Hull.from_config(yaml.safe_load((folder / "scene_tf.yaml").read_text(encoding="utf-8")))
    sample = dict(container=hull.as_dict(), steps=[dict(incoming=dict(id=f"B{i+1}", dims=dims))
                  for i, dims in enumerate(inputs["boxes"])])
    model, _, container, _ = make_model(sample, source_root, folder / "scene_tf.yaml")
    state = model.state()
    from manual_pick_gate import ManualPickGate
    gate = ManualPickGate(len(plan["sequence"])) if manual_pick else None
    viewer = None
    with np.load(folder / "trajectory.npz") as data:
        if list(data["body_labels"]) != list(model.body_label):
            raise ValueError("body order mismatch")
        if len(data["body_q"]) != len(data["stage"]) or not np.isfinite(data["body_q"]).all():
            raise ValueError("invalid recorded body poses")
        if not headless:
            import newton.viewer
            viewer = newton.viewer.ViewerGL(width=1280, height=800)
            viewer.set_model(model)
            viewer.show_ui = manual_pick
            if gate:
                if viewer.gui is None:
                    raise RuntimeError("Start Pick button requires Newton ImGui support")
                viewer.register_ui_callback(gate.render, position="free")
            viewer.set_camera(pos=wp.vec3(3.7, -5.8, 3.2), pitch=-12, yaw=136)
        grid = make_ground_grid(z=.003)
        title_prefix = "Newton PREVIEW (unvalidated) | " if unvalidated_preview else "Newton greedy demo | "
        last_label, spawned = None, []
        frame = 0
        try:
            while frame < len(data["stage"]):
                started = time.monotonic()
                if viewer and not viewer.is_running():
                    if unvalidated_preview:
                        return dict(success=True, stage="PREVIEW_CLOSED", spawned=spawned,
                                    frames=frame, motion_validation="not_run")
                    raise RuntimeError("viewer closed before ten-box completion")
                if viewer and not viewer.should_step():
                    viewer.begin_frame(frame / fps)
                    viewer.log_state(state)
                    viewer.end_frame()
                    continue
                stage, box_id = str(data["stage"][frame]), str(data["box_id"][frame])
                label = (stage, box_id)
                if label != last_label:
                    if gate:
                        gate.on_stage(stage, box_id)
                    if stage == "SPAWN":
                        spawned.append(box_id)
                    text = f"{len(spawned)}/10 | {box_id or '-'} | {stage}"
                    if gate and gate.waiting:
                        text += " | WAIT_PICK_BUTTON"
                    print("当前阶段: " + text, flush=True)
                    if viewer:
                        viewer.renderer.set_title(title_prefix + text)
                    last_label = label
                state.joint_q.assign(data["joint_q"][frame])
                state.body_q.assign(data["body_q"][frame])
                if viewer:
                    viewer.begin_frame(frame / fps)
                    viewer.log_state(state)
                    viewer.log_lines("ground_grid", *grid)
                    viewer.end_frame()
                    time.sleep(max(0, 1 / fps - (time.monotonic() - started)))
                # Keep rendering this spawn frame until its own button click.
                # A click is consumed immediately; no input can queue a cycle.
                if not (gate and gate.waiting):
                    frame += 1
            if viewer:
                if gate:
                    gate.on_stage("COMPLETE", "")
                viewer.renderer.set_title(title_prefix + "10/10 | COMPLETE")
                for _ in range(fps * 2):
                    if not viewer.is_running():
                        break
                    started = time.monotonic()
                    viewer.begin_frame(frame / fps)
                    viewer.log_state(state)
                    viewer.log_lines("ground_grid", *grid)
                    viewer.end_frame()
                    time.sleep(max(0, 1 / fps - (time.monotonic() - started)))
        finally:
            if viewer:
                viewer.close()
        expected = [row["id"] for row in plan["sequence"]]
        if spawned != expected:
            raise ValueError("recorded spawn order differs from frozen plan")
        result = dict(success=True, stage="COMPLETE", spawned=spawned, boxes=10,
                      frames=frame, planning_calls=0, certified_domain_poses=domain["accepted"])
        if unvalidated_preview:
            result["motion_validation"] = "not_run"
        if gate:
            result["operator_confirmations"] = gate.confirmations
        print(json.dumps(result), flush=True)
        return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path("demo_data/greedy10"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("fps must be positive")
    replay(args.bundle, headless=args.headless, fps=args.fps, device=args.device)
