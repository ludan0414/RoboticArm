"""Execute a finite, staged Newton kinematic pick/place pipeline.

Run from policy/ with the newton-rope Python environment. No robot connection.
Grasp at the top centre with a rigid attachment, without suction dynamics.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from container_demo import make_ground_grid
import newton
import warp as wp
from newton_ik_planner import NewtonIKWorkspace, tool_down_quaternion
from online_packing_newton import HIDDEN, Pose, make_model, world_pose
from online_packing_real import Hull
from packing_policy import _load_manifest, _make_policy, plan
from run_packing_visual import make_sample, parse_inline
from joint_approach_path import nearest_revolute_solution, plan_joint_approach
from scene_frames import container_pose, rotation


STAGE_NAMES = {
    "INPUT": "读取输入", "INITIALIZE": "初始化仿真", "PLAN": "策略规划",
    "RESET_HOME": "初始化机械臂（仿真 reset）",
    "SPAWN": "生成箱子", "APPROACH": "接近箱子", "PICK": "抓取接触",
    "ATTACH": "吸附（运动学约束）", "LIFT": "抬起", "CARRY": "搬运",
    "PLACE": "下降放置", "RELEASE": "释放", "RETREAT": "退回",
    "VERIFY": "检查落位", "COMPLETE": "完成", "FAILED": "失败",
}


class TracedIKWorkspace(NewtonIKWorkspace):
    """Retain planning boundary requests too, including pre-execution failure."""

    def __init__(self, *args, trace, **kwargs):
        self.trace = trace
        super().__init__(*args, **kwargs)

    def solve_world_pose(self, target_position, yaw, *, check_collision=True, orientation_xyzw=None):
        seed = self.q_last.copy()
        ok, joints = super().solve_world_pose(target_position, yaw,
            check_collision=check_collision, orientation_xyzw=orientation_xyzw)
        if ok:
            # Legacy cache rounds targets to millimetres. Re-measure cached
            # solutions against this exact request before accepting a path.
            self.model.joint_q.assign(joints)
            newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)
            actual = self.model.body_q.numpy()[self.contact_body]
            self.last_diagnostics = dict(self.last_diagnostics,
                position_error_m=float(np.linalg.norm(actual[:3] - target_position)),
                rotation_error_rad=self._quat_error(actual[3:], self.tool_orientation(yaw)
                    if orientation_xyzw is None else orientation_xyzw))
            ok = (self.last_diagnostics["position_error_m"] <= self.position_tolerance_m
                  and self.last_diagnostics["rotation_error_rad"] <= 0.035)
            if not ok:
                joints = None
        self.trace.write(json.dumps(dict(boundary="ik", target_world=list(map(float, target_position)),
                                        yaw_rad=float(yaw), check_collision=check_collision,
                                        seed=seed.tolist(), ok=ok,
                                        diagnostics=self.last_diagnostics,
                                        collision_pairs=getattr(self, "last_collision_pairs", []) if not ok else [])) + "\n")
        self.trace.flush()
        return ok, joints

    def route_is_feasible(self, route, dims, yaw, hull):
        ok = super().route_is_feasible(route, dims, yaw, hull)
        self.trace.write(json.dumps(dict(boundary="policy_route", route=route,
                                        dims=dims, yaw_rad=yaw, ok=ok)) + "\n")
        self.trace.flush()
        return ok

    def local_contact_world(self, lower_left, dims, yaw, hull):
        # The legacy checker targets the box centre. This pipeline targets
        # the top contact; preserve the policy's box-centre output contract.
        world, world_yaw = super().local_contact_world(lower_left, dims, yaw, hull)
        world += rotation(container_pose(self.config)).apply([0, 0, float(dims[2]) / 2])
        return world, world_yaw


class Pipeline:
    """Owns one Newton scene and stage/evidence state for a finite run."""

    def __init__(self, output: Path, mode: str, dt=1 / 60):
        self.output, self.mode, self.dt = output, mode, dt
        self.stage, self.box_id, self.attached = "INPUT", "", False
        self.started = time.monotonic()
        self.events = (output / "stages.jsonl").open("w", encoding="utf-8")
        self.trace = (output / "boundary.jsonl").open("w", encoding="utf-8")
        self.history, self.placed = [], {}
        self.viewer = None
        self.model = self.state = None
        self.q = None
        self.max_position_error = self.max_rotation_error = 0.0

    def enter(self, stage, **fields):
        self.stage = stage
        event = dict(stage=stage, label=STAGE_NAMES[stage], box_id=self.box_id,
                     attached=self.attached, elapsed_sec=time.monotonic() - self.started,
                     **fields)
        line = json.dumps(event, ensure_ascii=False)
        print("当前阶段: %s (%s) | 箱子: %s | %.2fs" % (
            STAGE_NAMES[stage], stage, self.box_id or "—", event["elapsed_sec"]), flush=True)
        self.events.write(line + "\n")
        self.events.flush()
        (self.output / "status.json").write_text(line, encoding="utf-8")
        if self.viewer:
            self.viewer.renderer.set_title(f"Newton | {stage} | {self.box_id}")

    def initialize(self, boxes, hull, args):
        manifest = dict(container=hull.as_dict(), steps=[
            dict(incoming=dict(id=f"B{i + 1}", dims=dims))
            for i, dims in enumerate(boxes)])
        self.model, self.body_ids, self.container, self.config = make_model(
            manifest, args.source_root, args.scene_config)
        self.state = self.model.state()
        self.ik = TracedIKWorkspace(args.source_root, args.scene_config,
                                    device=args.device, box_size=boxes[0], trace=self.trace)
        self.hull = hull
        self.q = self.ik.q_default.copy()
        self.contact_body = self.model.body_label.index("S20/suction_contact_frame")
        self.box_pose = None
        if self.mode == "show":
            import newton.viewer
            self.viewer = newton.viewer.ViewerGL(width=1280, height=800)
            self.viewer.set_model(self.model)
            self.viewer.show_ui = False
            door_negative_x = math.cos(float(self.container["rotation_rpy"][2])) > 0
            self.viewer.set_camera(pos=wp.vec3(-3.7 if door_negative_x else 3.7, -5.8, 3.2),
                                   pitch=-12.0, yaw=44.0 if door_negative_x else 136.0)
            self.grid = make_ground_grid(z=0.003)
        self.frame()

    def frame(self):
        self.state.joint_q.assign(self.q)
        self.state.joint_qd.assign(np.zeros_like(self.state.joint_qd.numpy()))
        newton.eval_fk(self.model, self.state.joint_q, self.state.joint_qd, self.state)
        bodies = self.state.body_q.numpy()
        for body in self.body_ids.values():
            bodies[body] = np.r_[HIDDEN, [0, 0, 0, 1]]
        for name, pose in self.placed.items():
            bodies[self.body_ids[name]] = pose
        if self.box_pose is not None:
            if self.attached:
                tool = bodies[self.contact_body]
                rotation = Rotation.from_quat(tool[3:])
                self.box_pose = np.r_[tool[:3] + rotation.apply(self.grasp_xyz),
                                      (rotation * self.grasp_rotation).as_quat()]
            bodies[self.body_ids[self.box_id]] = self.box_pose
        self.state.body_q.assign(bodies)
        if not np.isfinite(bodies).all():
            raise RuntimeError("nonfinite Newton body state")
        self.history.append((self.q.copy(), bodies[self.contact_body].copy(),
                             np.full(7, np.nan) if self.box_pose is None else self.box_pose.copy(),
                             self.stage, self.box_id, self.attached))
        self.trace.write(json.dumps(dict(frame=len(self.history) - 1, stage=self.stage,
                                        box_id=self.box_id, attached=self.attached,
                                        joint_q=self.q.tolist(),
                                        box_pose=None if self.box_pose is None else self.box_pose.tolist())) + "\n")
        if self.viewer:
            if not self.viewer.is_running():
                raise RuntimeError("viewer closed before pipeline completion")
            while not self.viewer.should_step():
                if not self.viewer.is_running():
                    raise RuntimeError("viewer closed while paused")
                self.viewer.begin_frame(len(self.history) * self.dt)
                self.viewer.log_state(self.state)
                self.viewer.end_frame()
            self.viewer.begin_frame(len(self.history) * self.dt)
            self.viewer.log_state(self.state)
            self.viewer.log_lines("ground_grid", *self.grid)
            self.viewer.end_frame()

    def solve(self, point, yaw, orientation_xyzw=None):
        if orientation_xyzw is None:
            ok, solution = self.ik.solve_world_pose(point, yaw)
        else:
            ok, solution = self.ik.solve_world_pose(point, yaw, orientation_xyzw=orientation_xyzw)
        diagnostics = dict(self.ik.last_diagnostics)
        self.trace.write(json.dumps(dict(stage=self.stage, target_world=list(map(float, point)),
                                        yaw_rad=yaw, ok=ok, diagnostics=diagnostics)) + "\n")
        self.trace.flush()
        if not ok or solution is None:
            raise RuntimeError(f"IK/collision rejected {self.stage}: {point}; {diagnostics}")
        self.max_position_error = max(self.max_position_error, diagnostics.get("position_error_m", 0))
        self.max_rotation_error = max(self.max_rotation_error, diagnostics.get("rotation_error_rad", 0))
        solution = nearest_revolute_solution(
            solution, self.q, self.model.joint_limit_lower.numpy()[:6],
            self.model.joint_limit_upper.numpy()[:6])
        self.ik.q_last = solution.astype(np.float32)
        return solution

    def move(self, point, yaw, *, joint_approach=False):
        point = np.asarray(point, dtype=float)
        if joint_approach:
            end = self.solve(point, yaw)
            start = self.q.copy()
            lower = self.model.joint_limit_lower.numpy()[:6]
            upper = self.model.joint_limit_upper.numpy()[:6]

            def is_valid(joints):
                if not np.isfinite(joints).all() or np.any(joints < lower) or np.any(joints > upper):
                    return False
                self.ik.model.joint_q.assign(joints)
                newton.eval_fk(self.ik.model, self.ik.model.joint_q,
                               self.ik.model.joint_qd, self.ik.model)
                ok = self.ik._collision_free()
                if not ok:
                    self.trace.write(json.dumps(dict(boundary="approach_collision", joint_q=joints.tolist())) + "\n")
                    self.trace.flush()
                return ok

            path = plan_joint_approach(start, end, lower, upper, is_valid)
            (self.output / (self.box_id + "_approach.json")).write_text(
                json.dumps([j.tolist() for j in path]), encoding="utf-8")
            for first, last in zip(path, path[1:]):
                count = max(12, math.ceil(float(np.max(np.abs(last - first))) / 0.04))
                for alpha in np.linspace(0, 1, count + 1)[1:]:
                    self.q = first + (alpha * alpha * (3 - 2 * alpha)) * (last - first)
                    if not is_valid(self.q):
                        raise RuntimeError("interpolated approach hits container")
                    self.frame()
            return
        start = self.state.body_q.numpy()[self.contact_body, :3].copy()
        count = max(12, math.ceil(float(np.linalg.norm(point - start)) / 0.015))
        for alpha in np.linspace(0, 1, count + 1)[1:]:
            target = start + (alpha * alpha * (3 - 2 * alpha)) * (point - start)
            self.q = self.solve(target, yaw)
            self.frame()

    def hold(self, count=20):
        for _ in range(count):
            self.frame()

    def execute(self, rows):
        for box_index, row in enumerate(rows):
            self.box_id = row["id"]
            dims = np.asarray(row["placed_dims_m"])
            local_quat = np.asarray(row["orientation"]["quaternion_xyzw"])
            route = [world_pose(Pose(np.asarray(p) + dims / 2, local_quat),
                                self.container, self.hull) for p in row["route"]]
            contacts = [pose.center + Rotation.from_quat(pose.quat).apply(
                [0, 0, float(dims[2]) / 2]) for pose in route]
            yaw = float(self.container["rotation_rpy"][2]) + math.radians(row["orientation"]["yaw_deg"])
            if box_index == 0:
                # Episode initial condition, before spawning any box. This is
                # an explicit simulation reset, not a certified motion from
                # the URDF's arbitrary all-zero joint state.
                self.enter("RESET_HOME")
                self.q = self.solve(contacts[0] + [0, 0, 0.12], yaw)
                self.frame()
            self.box_pose = np.r_[route[0].center, route[0].quat]
            self.enter("SPAWN", input_index=row["input_index"], pose_world=self.box_pose.tolist())
            self.hold()
            self.enter("APPROACH")
            if box_index:
                self.move(contacts[0] + [0, 0, 0.12], yaw, joint_approach=True)
            self.move(contacts[0] + [0, 0, 0.08], yaw)
            self.enter("PICK")
            self.move(contacts[0], yaw)
            tool = self.state.body_q.numpy()[self.contact_body]
            inverse = Rotation.from_quat(tool[3:]).inv()
            self.grasp_xyz = inverse.apply(self.box_pose[:3] - tool[:3])
            self.grasp_rotation = inverse * Rotation.from_quat(self.box_pose[3:])
            self.attached = True
            self.enter("ATTACH")
            self.hold()
            self.enter("LIFT")
            self.move(contacts[0] + [0, 0, 0.08], yaw)
            self.enter("CARRY")
            self.move(contacts[0], yaw)
            for contact in contacts[1:-1]:
                self.move(contact, yaw)
            self.enter("PLACE")
            self.move(contacts[-1], yaw)
            self.attached = False
            self.placed[self.box_id] = self.box_pose.copy()
            self.enter("RELEASE")
            self.hold()
            self.enter("RETREAT")
            self.move(contacts[-1] + [0, 0, 0.12], yaw)
            self.enter("VERIFY")
            actual = self.box_pose.copy()
            position_error = float(np.linalg.norm(actual[:3] - route[-1].center))
            rotation_error = float((Rotation.from_quat(actual[3:]).inv()
                                    * Rotation.from_quat(route[-1].quat)).magnitude())
            if position_error > 0.006 or rotation_error > 0.07:
                raise RuntimeError(f"placement mismatch: {position_error} m, {rotation_error} rad")
            self.events.write(json.dumps(dict(stage="VERIFY_RESULT", box_id=self.box_id,
                                              position_error_m=position_error,
                                              rotation_error_rad=rotation_error)) + "\n")
            self.hold()

    def close(self):
        self.trace.close()
        self.events.close()
        if self.state is not None:
            np.savez_compressed(self.output / "last_state.npz",
                                body_q=self.state.body_q.numpy(),
                                joint_q=self.q if self.q is not None else self.state.joint_q.numpy())
        if self.history:
            np.savez_compressed(self.output / "trajectory.npz",
                                joint_q=np.asarray([r[0] for r in self.history]),
                                tool_pose=np.asarray([r[1] for r in self.history]),
                                box_pose=np.asarray([r[2] for r in self.history]),
                                stage=np.asarray([r[3] for r in self.history]),
                                box_id=np.asarray([r[4] for r in self.history]),
                                attached=np.asarray([r[5] for r in self.history]))
        if self.viewer:
            self.viewer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boxes", default=None, help="L,W,H;L,W,H in metres")
    parser.add_argument("--input", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--mode", choices=("headless", "show"), default="headless")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--candidates", type=int, default=2)
    parser.add_argument("--source-root", type=Path, default=Path(__file__).parent / "vendor/roboticarm")
    parser.add_argument("--scene-config", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.input and args.boxes:
        parser.error("choose --input or --boxes")
    args.scene_config = args.scene_config or args.source_root / "elfin_humble_ws/src/luggage_description/config/scene_tf.yaml"
    args.output = args.output or Path(__file__).resolve().parents[1] / "docs/status/evidence/newton_pick_place" / datetime.now().strftime("%Y%m%d_%H%M%S")
    args.output.mkdir(parents=True, exist_ok=False)
    source_text = Path(__file__).read_text(encoding="utf-8")
    (args.output / "runner_snapshot.py").write_text(source_text, encoding="utf-8")
    source_hashes = {}
    sources = args.output / "sources"
    sources.mkdir()
    for name in ("run_pick_place_pipeline.py", "joint_approach_path.py",
                 "newton_ik_planner.py", "packing_policy.py", "online_packing_real.py",
                 "roboticarm_newton.py", "online_packing_newton.py", "run_packing_visual.py"):
        content = Path(__file__).with_name(name).read_bytes()
        (sources / name).write_bytes(content)
        source_hashes[name] = hashlib.sha256(content).hexdigest()
    run = Pipeline(args.output, args.mode)
    revision = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True)
    dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True)
    result = dict(success=False, profile="Newton kinematic top-centre grasp; no contact solver",
                  argv=sys.argv, started_utc=datetime.now(timezone.utc).isoformat(),
                  device=args.device, scene_config=str(args.scene_config.resolve()),
                  revision=revision.stdout.strip(), dirty_paths=dirty.stdout.splitlines(),
                  runner_sha256=hashlib.sha256(source_text.encode("utf-8")).hexdigest(),
                  source_sha256=source_hashes, python=sys.executable,
                  newton_version=newton.__version__, warp_version=wp.__version__,
                  checkpoint=None if args.checkpoint is None else str(args.checkpoint.resolve()),
                  checkpoint_sha256=None)
    try:
        run.enter("INPUT")
        if args.checkpoint:
            result["checkpoint_sha256"] = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
        boxes = _load_manifest(args.input) if args.input else parse_inline(args.boxes or "0.55,0.40,0.25")
        if not boxes or any(len(d) != 3 or not np.isfinite(d).all() or min(d) <= 0 for d in boxes):
            raise ValueError("box dimensions must be finite and positive")
        (args.output / "input.json").write_text(json.dumps(dict(boxes=boxes)), encoding="utf-8")
        config_text = args.scene_config.read_text(encoding="utf-8")
        (args.output / "scene_tf.yaml").write_text(config_text, encoding="utf-8")
        hull = Hull.from_config(yaml.safe_load(config_text))
        wp.set_device(args.device)
        run.enter("INITIALIZE")
        run.initialize(boxes, hull, args)
        run.enter("PLAN", policy="checkpoint" if args.checkpoint else "baseline")
        policy = _make_policy(args.checkpoint, args.device)
        rows = plan(boxes, hull, run.ik, policy, args.device, args.candidates)
        (args.output / "plan.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
        sample = make_sample(boxes, hull, rows, "checkpoint" if policy else "baseline", run.ik)
        (args.output / "sample.json").write_text(json.dumps([sample]), encoding="utf-8")
        run.execute(rows)
        run.enter("COMPLETE", placed=len(run.placed))
        result.update(success=True, placed=len(run.placed), frames=len(run.history),
                      max_ik_position_error_m=run.max_position_error,
                      max_ik_rotation_error_rad=run.max_rotation_error)
    except BaseException as exc:
        result.update(failed_stage=run.stage, error=str(exc), attached=run.attached)
        run.enter("FAILED", failed_stage=run.stage, error=str(exc))
    finally:
        run.close()
        result.update(ended_utc=datetime.now(timezone.utc).isoformat(),
                      elapsed_sec=time.monotonic() - run.started,
                      artifact_inventory=sorted(p.name for p in args.output.iterdir()),
                      capture_complete=True, physics_verification="not_evaluated")
        (args.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
