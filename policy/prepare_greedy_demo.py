"""Prepare random inputs, a URDF reachability certificate and a frozen 10-box demo.

No viewer is created by this command. Replay is a separate process.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

from container_demo import FRAME_DT
import warp as wp
import yaml
from demo_contract import digest, generate_boxes, load_box_input, write_json, freeze_bundle
from demo_workspace import CertifiedWorkspace, explore_domain, recheck_domain
from demo_greedy import greedy_sequence
from demo_feeder import FEEDER_CENTER_M
from online_packing_real import Hull
from roboticarm_newton import DEFAULT_SOURCE, make_robot_urdf, scene_paths


def model_manifest(source_root, scene):
    source, _, models = scene_paths(source_root)
    urdf = make_robot_urdf(source, models)
    paths = {Path(m.get("filename")) for m in ET.fromstring(urdf).iter("mesh")}
    config = yaml.safe_load(scene.read_text(encoding="utf-8"))
    paths.add(models / config["gazebo"]["container_model"] / "meshes/container_collision.stl")
    data = dict(schema_version=1, urdf_sha256=hashlib.sha256(urdf.encode()).hexdigest(),
                scene_sha256=digest(scene), meshes={str(p.relative_to(source_root)): digest(p) for p in sorted(paths)},
                certificate_code_sha256=digest(Path(__file__).with_name("demo_workspace.py")),
                collision_code_sha256=digest(Path(__file__).with_name("demo_collision.py")),
                source_root=str(source_root.resolve()))
    data["srdf_sha256"] = digest(Path(__file__).resolve().parents[1] / "src/luggage_description/config/S20_with_camera.srdf")
    data["identity"] = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261022)
    parser.add_argument("--input", type=Path, help="existing JSON containing exactly 10 box sizes")
    parser.add_argument("--output", type=Path, default=Path("demo_data/greedy10"))
    parser.add_argument("--domain-cache", type=Path, default=Path("demo_data/urdf_domain.json"))
    parser.add_argument("--domain-source", type=Path,
                        help="re-certify a pre-explored domain's stored joint witnesses")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--scene-config", type=Path)
    args = parser.parse_args()
    args.source_root = args.source_root.resolve()
    scene = args.scene_config or scene_paths(args.source_root)[1]
    scene = Path(scene).resolve()
    args.scene_config = scene
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    trace = (args.output / "offline_boundary.jsonl").open("w", encoding="utf-8")
    stage = "GENERATE_INPUTS"
    try:
        inputs = args.output / "inputs"
        inputs.mkdir()
        for seed in range(args.seed, args.seed + 3):
            write_json(inputs / f"boxes_seed_{seed}.json", generate_boxes(seed))
        manifest = load_box_input(args.input) if args.input else generate_boxes(args.seed)
        write_json(args.output / "input.json", manifest)
        # All later phases consume the persisted test fixture.
        manifest = load_box_input(args.output / "input.json")
        shutil.copyfile(scene, args.output / "scene_tf.yaml")
        identity = model_manifest(args.source_root, scene)
        write_json(args.output / "model_manifest.json", identity)
        hull = Hull.from_config(yaml.safe_load(scene.read_text(encoding="utf-8")))
        wp.set_device(args.device)
        stage = "EXPLORE_DOMAIN"
        print("当前阶段: 预先探索 URDF 可行域 (EXPLORE_DOMAIN)", flush=True)
        workspace = CertifiedWorkspace(args.source_root, scene, device=args.device,
                                       box_size=manifest["boxes"][0], trace=trace)
        cached = json.loads(args.domain_cache.read_text(encoding="utf-8")) if args.domain_cache.is_file() else None
        if args.domain_source:
            prior_identity = json.loads(args.domain_source.with_name("model_manifest.json").read_text(encoding="utf-8"))
            for key in ("urdf_sha256", "scene_sha256", "meshes", "srdf_sha256"):
                if prior_identity[key] != identity[key]:
                    raise ValueError("pre-explored domain model mismatch: " + key)
            domain = recheck_domain(workspace, hull, args.domain_source, args.output / "domain.json")
            domain["model_identity"] = identity["identity"]
            write_json(args.output / "domain.json", domain)
            write_json(args.domain_cache, domain)
        elif cached and cached.get("model_identity") == identity["identity"]:
            domain = cached
            write_json(args.output / "domain.json", domain)
            print(f"REUSE_DOMAIN certified={domain['accepted']}", flush=True)
        else:
            domain = explore_domain(workspace, hull, args.output / "domain.json")
            domain["model_identity"] = identity["identity"]
            write_json(args.output / "domain.json", domain)
            args.domain_cache.parent.mkdir(parents=True, exist_ok=True)
            write_json(args.domain_cache, domain)
        if not domain["poses"]:
            raise RuntimeError("no certified poses in explored grid")
        stage = "GREEDY_PLAN"
        print("当前阶段: 贪心生成完整顺序、位置和 orientation (GREEDY_PLAN)", flush=True)
        rows = greedy_sequence(manifest["boxes"], hull, domain, workspace)
        plan = dict(schema_version=1, policy="greedy_back_to_front", seed=manifest.get("seed"),
                    feeder_center_container_door_floor_m=list(FEEDER_CENTER_M),
                    position_frame="world", route_frame="container_door_floor", sequence=rows)
        write_json(args.output / "plan.json", plan)
        stage = "COMPILE_TRAJECTORY"
        print("当前阶段: 离线验算全部轨迹 (COMPILE_TRAJECTORY)，尚未创建可视化窗口", flush=True)
        from demo_trajectory import compile_trajectory
        validation = compile_trajectory(args, manifest, hull, rows, workspace)
        write_json(args.output / "validation.json", validation)
        freeze_bundle(args.output)
        write_json(args.output / "prepare_result.json", dict(success=True, stage="READY_FOR_REPLAY",
                   boxes=len(rows), elapsed_sec=time.monotonic() - started))
        print(f"当前阶段: READY_FOR_REPLAY | 10/10 | {args.output}", flush=True)
        return 0
    except BaseException as exc:
        write_json(args.output / "prepare_result.json", dict(success=False, failed_stage=stage,
                   error=str(exc), elapsed_sec=time.monotonic() - started, replay_ready=False))
        print(f"FAILED {stage}: {exc}", flush=True)
        return 1
    finally:
        trace.close()


if __name__ == "__main__":
    raise SystemExit(main())
