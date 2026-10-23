"""Replay an online packing plan in the real RoboticArm loading cell.

This is a kinematic storyboard: Newton renders the original collision shapes,
while the planner's box poses drive them. No contact solver is used here; IK
samples also replay the corresponding S20 joint poses while holding a box.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scene_frames import rotation

from container_demo import make_ground_grid  # configures Warp and OpenGL first
from online_packing_real import Hull, validate_sample
from newton_ik_planner import box_yaw
from roboticarm_newton import DEFAULT_SOURCE, add_cell_scene, scene_paths

import newton
import warp as wp


SAMPLES = Path(__file__).with_name("online_packing_real_samples.json")
FPS = 60
HIDDEN = np.array([100.0, 100.0, 100.0], dtype=np.float64)


def path_label(path: Path, root: Path) -> str:
    """Keep diagnostics portable when the asset is inside this repository."""
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


@dataclass(frozen=True)
class Pose:
    center: np.ndarray
    quat: np.ndarray  # xyzw


@dataclass(frozen=True)
class Frame:
    poses: dict[str, Pose]
    caption: str
    step: int
    joint_q: np.ndarray | None = None


def load_sample(path: Path, seed: int) -> dict:
    samples = json.loads(path.read_text(encoding="utf-8"))
    matches = [sample for sample in samples if sample["seed"] == seed]
    if not matches:
        raise ValueError(f"seed {seed} unavailable; choose one of {[x['seed'] for x in samples]}")
    sample = matches[0]
    validate_sample(sample)
    return sample


def quaternion_for_dims(source_dims, target_dims) -> np.ndarray:
    """Find a right-handed axis permutation giving the requested AABB size."""
    from itertools import permutations

    source = np.asarray(source_dims, dtype=float)
    target = np.asarray(target_dims, dtype=float)
    for perm in permutations(range(3)):
        if not np.allclose(source[list(perm)], target, atol=1e-8):
            continue
        matrix = np.zeros((3, 3), dtype=float)
        for world_axis, local_axis in enumerate(perm):
            matrix[world_axis, local_axis] = 1.0
        if np.linalg.det(matrix) < 0:
            matrix[:, perm[0]] *= -1.0
        return Rotation.from_matrix(matrix).as_quat()
    raise ValueError(f"{target_dims} is not a rotation of {source_dims}")


def box_pose(box: dict, source_dims) -> Pose:
    pos = np.asarray(box["pos"], dtype=float)
    dims = np.asarray(box["dims"], dtype=float)
    return Pose(pos + dims / 2, quaternion_for_dims(source_dims, dims))


def moved_to(pose: Pose, center=None, quat=None) -> Pose:
    return Pose(np.asarray(center if center is not None else pose.center, dtype=float),
                np.asarray(quat if quat is not None else pose.quat, dtype=float))


def interpolate(start: Pose, end: Pose, alpha: float) -> Pose:
    ease = alpha * alpha * (3.0 - 2.0 * alpha)
    rotations = Rotation.from_quat(np.stack((start.quat, end.quat)))
    quat = Slerp([0.0, 1.0], rotations)([ease]).as_quat()[0]
    return Pose((1.0 - ease) * start.center + ease * end.center, quat)


def make_timeline(sample: dict, config: dict, ik_workspace=None) -> list[Frame]:
    source_dims = {s["incoming"]["id"]: s["incoming"]["dims"] for s in sample["steps"]}
    hull = Hull.from_dict(sample["container"])
    container_pose = next(x for x in config["static_transforms"] if x["child"] == "container_link")
    container_rotation_inv = Rotation.from_euler("z", -float(container_pose["rotation_rpy"][2]))
    platform = config["pickup_platform"]
    platform_top_world = np.asarray(platform["translation"], dtype=float) + np.array([0.0, 0.0, float(platform["size"][2])])
    platform_top_local = container_rotation_inv.apply(platform_top_world - np.asarray(container_pose["translation"], dtype=float))
    stage_center = platform_top_local - np.array([hull.door_x, hull.inner_y0, hull.floor_z])
    poses: dict[str, Pose] = {}
    frames: list[Frame] = []
    arm_q = None if ik_workspace is None else ik_workspace.q_default.copy()

    def solve_pose(box: dict, planned_pose: Pose) -> np.ndarray | None:
        if ik_workspace is None:
            return None
        dims = tuple(float(v) for v in box["dims"])
        world = world_pose(planned_pose, container_pose, hull)
        target = world.center
        world_yaw = Rotation.from_quat(world.quat).as_euler("zyx")[0]
        ok, solution = ik_workspace.solve_world_pose(target, world_yaw)
        if not ok or solution is None:
            raise RuntimeError(f"IK replay failed for {box['id']} at {tuple(world.center)}")
        return solution.copy()

    def solve_stage(box: dict) -> np.ndarray | None:
        if ik_workspace is None:
            return None
        return solve_pose(box, stage_pose(box))

    def hold(caption: str, step: int, count: int = 20):
        for _ in range(count):
            frames.append(Frame(dict(poses), caption, step,
                                  None if arm_q is None else arm_q.copy()))

    def animate(box_id: str, start: Pose, end: Pose, caption: str, step: int,
                count: int = 45, joint_end: np.ndarray | None = None,
                ik_box: dict | None = None):
        nonlocal arm_q
        joint_start = None if arm_q is None else arm_q.copy()
        if joint_end is not None:
            joint_end = np.asarray(joint_end, dtype=np.float32).copy()
        for i in range(1, count + 1):
            alpha = i / count
            poses[box_id] = interpolate(start, end, alpha)
            if ik_box is not None and ik_workspace is not None:
                arm_q = solve_pose(ik_box, poses[box_id])
            elif joint_start is not None and joint_end is not None:
                ease = alpha * alpha * (3.0 - 2.0 * alpha)
                arm_q = (1.0 - ease) * joint_start + ease * joint_end
            frames.append(Frame(dict(poses), caption, step,
                                  None if arm_q is None else arm_q.copy()))

    def at(box: dict, lower_left) -> Pose:
        return box_pose({**box, "pos": lower_left}, source_dims[box["id"]])

    def stage_pose(box: dict) -> Pose:
        dims = np.asarray(box["dims"], dtype=float)
        center = stage_center + np.array([0.0, 0.0, dims[2] / 2])
        return Pose(center, quaternion_for_dims(source_dims[box["id"]], dims))

    def follow(box: dict, route, step_number: int, verb: str):
        box_id = box["id"]
        for waypoint in route[1:]:
            start, end = poses[box_id], at(box, waypoint)
            axis = int(np.argmax(np.abs(end.center - start.center)))
            phrase = ("through opening" if axis == 0 else
                      "shift sideways" if axis == 1 else "change height")
            joint_end = solve_pose(box, end)
            animate(box_id, start, end, f"B{step_number}: {verb} {box_id} | {phrase}",
                    step_number, 38 if axis == 0 else 24,
                    joint_end=joint_end, ik_box=box)

    hold("Empty container | incoming boxes are revealed one at a time", 0, 35)
    for step_number, step in enumerate(sample["steps"], 1):
        incoming = step["incoming"]
        if not step["actions"]:
            hold(f"B{step_number}: no legal place/rearrangement | stop", step_number, 100)
            break
        for action in step["actions"]:
            kind = action["kind"]
            box = action["box"]
            box_id = box["id"]
            target = box_pose(box, source_dims[box_id])
            route = action["route"]
            if kind == "place":
                poses[box_id] = at(box, route[0])
                joint_start = solve_pose(box, poses[box_id])
                if joint_start is not None:
                    arm_q = joint_start
                hold(f"B{step_number}: incoming {box_id} at the real doorway", step_number, 16)
                follow(box, route, step_number, "insert")
                hold(f"B{step_number}: {box_id} placed", step_number)
            elif kind == "remove":
                hold(f"B{step_number}: remove {box_id} first", step_number, 16)
                follow(box, route, step_number, "remove")
                stage = stage_pose(box)
                animate(box_id, poses[box_id], stage,
                        f"B{step_number}: stage {box_id} outside", step_number, 34,
                        joint_end=solve_stage(box), ik_box=box)
                hold(f"B{step_number}: {box_id} waits outside", step_number, 14)
            elif kind == "reinsert":
                staged = poses[box_id]
                rotated = stage_pose(box)
                animate(box_id, staged, rotated,
                        f"B{step_number}: rotate {box_id} outside", step_number, 32,
                        joint_end=solve_stage(box), ik_box=box)
                approach = at(box, route[0])
                animate(box_id, rotated, approach,
                        f"B{step_number}: line up {box_id}", step_number, 35,
                        joint_end=solve_pose(box, approach), ik_box=box)
                follow(box, route, step_number, "reinsert")
                hold(f"B{step_number}: staging cleared; next box may arrive", step_number, 25)
            else:
                raise ValueError(kind)
        # The final storyboard pose must match the planner's committed state.
        for final_box in step["boxes"]:
            expected = box_pose(final_box, source_dims[final_box["id"]])
            actual = poses[final_box["id"]]
            assert np.allclose(actual.center, expected.center, atol=1e-6)
            assert np.allclose(actual.quat, expected.quat, atol=1e-6)
    return frames


def make_manual_loading_timeline(sample: dict, config: dict, ik_workspace) -> list[Frame]:
    """Spawn each box at the route entrance, pick it, carry it, and release."""
    if ik_workspace is None:
        raise ValueError("attached-box replay requires Newton IK")
    hull = Hull.from_dict(sample["container"])
    container = next(item for item in config["static_transforms"]
                     if item["child"] == "container_link")
    poses: dict[str, Pose] = {}
    frames: list[Frame] = []
    arm_q = ik_workspace.q_default.copy()

    def solve(pose, *, loading=False):
        world = world_pose(pose, container, hull)
        yaw = Rotation.from_quat(world.quat).as_euler("zyx")[0]
        ok, joints = ik_workspace.solve_world_pose(world.center, yaw,
                                                   check_collision=not loading)
        if not ok or joints is None:
            raise RuntimeError(f"Newton IK cannot reach carried box center {world.center.tolist()}")
        return joints.copy()

    def hold(label, step, count=20):
        for _ in range(count):
            frames.append(Frame(dict(poses), label, step, arm_q.copy()))

    def move_arm(target, label, step, count=30, *, loading=False):
        nonlocal arm_q
        joint_start = arm_q.copy()
        joint_end = solve(target, loading=loading)
        for index in range(1, count + 1):
            alpha = index / count
            ease = alpha * alpha * (3.0 - 2.0 * alpha)
            arm_q = (1.0 - ease) * joint_start + ease * joint_end
            frames.append(Frame(dict(poses), label, step, arm_q.copy()))

    def carry(box_id, target, label, step):
        nonlocal arm_q
        start = poses[box_id]
        distance = float(np.linalg.norm(target.center - start.center))
        count = max(12, int(math.ceil(distance / 0.015)))
        joint_start = arm_q.copy()
        joint_end = solve(target)
        for index in range(1, count + 1):
            alpha = index / count
            ease = alpha * alpha * (3.0 - 2.0 * alpha)
            poses[box_id] = interpolate(start, target, alpha)
            frames.append(Frame(dict(poses), label, step,
                                ((1.0 - ease) * joint_start + ease * joint_end).copy()))
        arm_q = joint_end

    for step_number, step in enumerate(sample["steps"], 1):
        if len(step["actions"]) != 1 or step["actions"][0]["kind"] != "place":
            raise ValueError("attached-box replay supports one place action per step")
        action = step["actions"][0]
        box = action["box"]
        box_id = box["id"]
        source = np.asarray(step["incoming"]["dims"], dtype=float)
        route = action["route"]
        first = box_pose({**box, "pos": route[0]}, source)
        yaw = float(box_yaw(source, box["dims"]))
        first = moved_to(first, quat=Rotation.from_euler("z", yaw).as_quat())
        poses[box_id] = first
        hold(f"B{step_number}: spawn input #{step['incoming'].get('input_index', step_number - 1) + 1} ({box_id})",
             step_number, 30)
        above = moved_to(first, center=first.center + [0.0, 0.0, 0.12])
        move_arm(above, f"B{step_number}: approach {box_id}", step_number, loading=True)
        move_arm(first, f"B{step_number}: contact center of {box_id}",
                 step_number, 20, loading=True)
        hold(f"B{step_number}: suction ON {box_id}", step_number, 12)
        for point in route[1:]:
            target = moved_to(box_pose({**box, "pos": point}, source), quat=first.quat)
            carry(box_id, target, f"B{step_number}: carry {box_id}", step_number)
        hold(f"B{step_number}: suction OFF, release {box_id}", step_number, 25)
    return frames


def make_model(sample: dict, source_root: Path, scene_config_path: Path,
               container_y_override: float | None = None,
               *, show_robot: bool = True):
    builder = newton.ModelBuilder()
    builder.default_shape_cfg.mu = 0.7
    builder.default_joint_cfg.target_ke = 6000.0
    builder.default_joint_cfg.target_kd = 120.0
    config = add_cell_scene(builder, source_root, show_robot=show_robot,
                            scene_config_path=scene_config_path,
                            container_y_override=container_y_override)
    if Hull.from_config(config) != Hull.from_dict(sample["container"]):
        raise ValueError("Planning sample geometry differs from the loaded RoboticArm scene; regenerate it")

    body_ids = {}
    for step in sample["steps"]:
        incoming = step["incoming"]
        box_id = incoming["id"]
        sx, sy, sz = map(float, incoming["dims"])
        body = builder.add_body(xform=wp.transform(p=wp.vec3(*HIDDEN), q=wp.quat_identity()), label=box_id)
        color = incoming.get("color_rgb",
                    (0.83, 0.46, 0.20) if int(box_id[1:]) % 2 else (0.70, 0.52, 0.28))
        builder.add_shape_box(body, hx=sx / 2, hy=sy / 2, hz=sz / 2,
                              color=wp.vec3(*color), label=f"{box_id}_collision_box")
        body_ids[box_id] = body
    model = builder.finalize()
    # Keep the arm visuals, but hide only the static URDF payload. The animated
    # cuboid bodies below are the source of truth for the loading storyboard.
    flags = model.shape_flags.numpy()
    shape_bodies = model.shape_body.numpy()
    for i, body in enumerate(shape_bodies):
        if body >= 0 and model.body_label[int(body)] == "S20/payload_box":
            flags[i] &= ~1
    model.shape_flags.assign(flags)
    newton.eval_fk(model, model.joint_q, model.joint_qd, model)
    container_pose = next(x for x in config["static_transforms"] if x["child"] == "container_link")
    return model, body_ids, container_pose, config


def world_pose(pose: Pose, container_pose: dict, hull: Hull) -> Pose:
    yaw_rotation = rotation(container_pose)
    local_center = np.array([hull.door_x, hull.inner_y0, hull.floor_z]) + pose.center
    center = np.asarray(container_pose["translation"], dtype=float) + yaw_rotation.apply(local_center)
    quat = (yaw_rotation * Rotation.from_quat(pose.quat)).as_quat()
    return Pose(center, quat)


def set_state(model, state, frame: Frame, body_ids, container_pose: dict, hull: Hull):
    if frame.joint_q is not None:
        state.joint_q.assign(frame.joint_q)
        state.joint_qd.assign(np.zeros_like(state.joint_qd.numpy()))
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
    q = state.body_q.numpy()
    for i in body_ids.values():
        q[i, :3] = HIDDEN
        q[i, 3:] = np.array([0.0, 0.0, 0.0, 1.0])
    for box_id, planned_pose in frame.poses.items():
        pose = world_pose(planned_pose, container_pose, hull)
        i = body_ids[box_id]
        q[i, :3] = pose.center
        q[i, 3:] = pose.quat
    state.body_q.assign(q)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, default=SAMPLES)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--scene-config", type=Path, default=None,
                        help="scene_tf YAML; defaults to the vendored scene_tf.yaml")
    parser.add_argument("--container-y", type=float, default=None,
                        help="override container world Y from the scene configuration")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--mode", choices=("inspect", "bench", "show"), default="show")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--frames", type=int, default=0, help="show: 0 runs until window closes")
    parser.add_argument("--once", action="store_true", help="stop after the plan; otherwise loop")
    parser.add_argument("--hide-robot", action="store_true",
                        help="hide S20 visuals while keeping the scene and planner unchanged")
    parser.add_argument("--no-ik-follow", action="store_true",
                        help="keep the arm static while replaying any sample")
    parser.add_argument("--ik-follow", action="store_true",
                        help="drive the arm with IK while replaying a geometric sample")
    args = parser.parse_args()
    scene_config_path = args.scene_config or scene_paths(args.source_root)[1]
    container_y_override = args.container_y
    wp.set_device(args.device)
    sample = load_sample(args.samples, args.seed)
    hull = Hull.from_dict(sample["container"])
    model, body_ids, container_pose, config = make_model(
        sample, args.source_root, scene_config_path, container_y_override,
        show_robot=not args.hide_robot)
    ik_workspace = None
    use_ik_follow = (
        not args.no_ik_follow
        and (sample.get("ik", {}).get("enabled") or args.ik_follow
             or sample.get("policy") == "baseline")
    )
    if use_ik_follow:
        from newton_ik_planner import NewtonIKWorkspace

        first_dims = tuple(sample["steps"][0]["incoming"]["dims"])
        ik_workspace = NewtonIKWorkspace(
            args.source_root,
            scene_config_path,
            container_y_override=container_y_override,
            device=args.device,
            box_size=first_dims,
        )
    manual_loading = bool(sample.get("manual_loading"))
    timeline = (make_manual_loading_timeline(sample, config, ik_workspace)
                if manual_loading else make_timeline(sample, config, ik_workspace=ik_workspace))
    state = model.state()
    info = {"seed": args.seed, "boxes": len(body_ids), "actions": sum(len(s["actions"]) for s in sample["steps"]),
            "relocations": sample["relocations"], "timeline_frames": len(timeline),
            "duration_seconds_at_60fps": round(len(timeline) / FPS, 2),
            "scene_shapes": model.shape_count, "container_model": "airport_container_real",
            "scene_config": path_label(scene_config_path, args.source_root),
            "container_pose_world": container_pose["translation"],
            "pickup_platform_world": config["pickup_platform"]["translation"],
            "robot_visual": not args.hide_robot,
            "robot_payload_visual": False,
            "ik_joint_follow": ik_workspace is not None,
            "pipeline": "scripted poses; no physics solver",
            "manual_loading": manual_loading}
    print(json.dumps(info, ensure_ascii=False), flush=True)
    set_state(model, state, timeline[0], body_ids, container_pose, hull)
    if args.mode == "inspect":
        return
    if args.mode == "bench":
        durations = []
        for frame in timeline:
            start = time.perf_counter()
            set_state(model, state, frame, body_ids, container_pose, hull)
            wp.synchronize()
            durations.append((time.perf_counter() - start) * 1000)
        print(json.dumps({"pose_update_mean_ms": round(statistics.mean(durations), 3),
                          "pose_update_p95_ms": round(sorted(durations)[int(.95 * (len(durations) - 1))], 3),
                          "finite": bool(np.isfinite(state.body_q.numpy()).all())}, ensure_ascii=False), flush=True)
        return

    import newton.viewer

    grid_starts, grid_ends, grid_colors = make_ground_grid(z=0.003)
    viewer = newton.viewer.ViewerGL()
    viewer.set_model(model)
    viewer.show_ui = False  # Tab restores Newton's controls.
    door_faces_negative_x = math.cos(float(container_pose["rotation_rpy"][2])) > 0
    camera_x = -3.7 if door_faces_negative_x else 3.7
    camera_yaw = 44.0 if door_faces_negative_x else 136.0
    viewer.set_camera(pos=wp.vec3(camera_x, -5.8, 3.2), pitch=-12.0, yaw=camera_yaw)
    frame_number = 0
    last_caption = None
    try:
        while viewer.is_running():
            if viewer.should_step():
                idx = frame_number % len(timeline)
                frame = timeline[idx]
                set_state(model, state, frame, body_ids, container_pose, hull)
                if frame.caption != last_caption:
                    viewer.renderer.set_title(f"Newton packing | {scene_config_path.name} | seed {args.seed} | {frame.caption}")
                    print(f"[{idx / FPS:6.1f}s] {frame.caption}", flush=True)
                    last_caption = frame.caption
                frame_number += 1
            viewer.begin_frame(frame_number / FPS)
            viewer.log_state(state)
            viewer.log_lines("ground_grid", grid_starts, grid_ends, grid_colors)
            viewer.end_frame()
            if args.frames and frame_number >= args.frames:
                break
            if args.once and frame_number >= len(timeline):
                break
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
