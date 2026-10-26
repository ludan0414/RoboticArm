"""Collision-checked ground connector spliced onto unchanged recorded motions."""
import copy
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import newton
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from demo_contract import write_json, freeze_bundle, check_bundle
from demo_recording import validate_recording
from demo_trajectory import MotionCompiler
from demo_workspace import CertifiedWorkspace
from joint_approach_path import plan_joint_approach, nearest_revolute_solution
from run_pick_place_pipeline import Pipeline, STAGE_NAMES
from online_packing_real import Hull
from roboticarm_newton import DEFAULT_SOURCE
from newton_ik_planner import tool_down_quaternion
from ground_carry_clearance import (pedestal_geometry, box_bottom, box_pedestal_separated,
                                    outside_staging_xy, safe_carry_entry)
from verify_pick_place_run import verify


def portal_staging_point(dock_tool_xyz,container,distance_m=.20):
    """Align outside the door before entering its narrow clearance corridor.

    The extra20cm is a transit standoff, not a reduced/alternative door margin.
    Keep world height fixed so lateral transfer retains pedestal clearance.
    """
    direction=Rotation.from_euler('xyz',container['rotation_rpy']).apply([-1.,0.,0.])
    horizontal=float(np.linalg.norm(direction[:2]))
    if horizontal<.5 or not np.isfinite(distance_m) or distance_m<=0:
        raise ValueError('invalid doorway transit staging geometry')
    point=np.asarray(dock_tool_xyz,dtype=float).copy()
    point[:2]+=direction[:2]/horizontal*distance_m
    return point


class BridgeCompiler(MotionCompiler):
    checking = False
    validate_motion = True
    high_carry = False

    def payload_contact_failures(self):
        failures = super().payload_contact_failures()
        if self.box_pose is not None and self.attached:
            pose = self.state.body_q.numpy()[self.body_ids[self.box_id]]
            bottom = box_bottom(pose, self.current_box_size)
            if not box_pedestal_separated(pose, self.current_box_size, self.pedestal, .05):
                failures.append(dict(reason="PEDESTAL_CLEARANCE", required_margin_m=.05))
            if self.high_carry and bottom < self.high_carry_bottom_z - .003:
                failures.append(dict(reason="HIGH_CARRY_FLOOR", box_bottom_z_m=bottom,
                                     minimum_z_m=self.high_carry_bottom_z))
        return failures

    def solve(self, point, yaw):
        # Transit goals need practical positioning, not contact precision.
        # Cache identity includes this tolerance; Pick stays at the default.
        self.ik.position_tolerance_m = .01 if self.stage in (
            "RETURN", "APPROACH", "LIFT", "CARRY") else .003
        if self.validate_motion:
            return super().solve(point, yaw, orientation_xyzw=tool_down_quaternion(yaw))
        ok, joints = self.ik.solve_world_pose(point, yaw, check_collision=False,
                                             orientation_xyzw=tool_down_quaternion(yaw))
        if not ok or joints is None:
            raise RuntimeError(f"Animation IK could not reach {self.stage}: {point}")
        joints = nearest_revolute_solution(joints, self.q,
            self.model.joint_limit_lower.numpy()[:6], self.model.joint_limit_upper.numpy()[:6])
        self.ik.q_last = joints.astype(np.float32)
        return joints

    def refresh(self, joints):
        """FK and visible cargo state for a planning probe, without recording it."""
        self.state.joint_q.assign(joints)
        newton.eval_fk(self.model, self.state.joint_q, self.state.joint_qd, self.state)
        bodies = self.state.body_q.numpy()
        for body in self.body_ids.values():
            bodies[body] = [100, 100, 100, 0, 0, 0, 1]
        for name, pose in self.placed.items():
            bodies[self.body_ids[name]] = pose
        if self.box_pose is not None:
            pose = self.box_pose
            if self.attached:
                tool = bodies[self.contact_body]
                rot = Rotation.from_quat(tool[3:])
                pose = np.r_[tool[:3] + rot.apply(self.grasp_xyz), (rot * self.grasp_rotation).as_quat()]
            bodies[self.body_ids[self.box_id]] = pose
        self.state.body_q.assign(bodies)

    def valid(self, joints):
        lower, upper = self.model.joint_limit_lower.numpy()[:6], self.model.joint_limit_upper.numpy()[:6]
        if not np.isfinite(joints).all() or np.any(joints < lower) or np.any(joints > upper):
            return False
        self.ik.model.joint_q.assign(joints)
        newton.eval_fk(self.ik.model, self.ik.model.joint_q, self.ik.model.joint_qd, self.ik.model)
        if not self.ik._collision_free():
            return False
        self.refresh(joints)
        return not self.payload_contact_failures()

    def frame(self):
        if not self.validate_motion:
            Pipeline.frame(self)
            self.body_frames.append(self.state.body_q.numpy().copy())
            return
        # valid() checks robot/static and payload contacts together; avoid
        # repeating the same expensive collision pass three times per frame.
        Pipeline.frame(self)
        self.body_frames.append(self.state.body_q.numpy().copy())
        if self.checking:
            if self.box_pose is None and self.placed:
                self.check_payload_contacts()
            if not self.valid(self.q):
                write_json(self.output / "failure_frame.json", dict(stage=self.stage, box_id=self.box_id,
                    frame=len(self.history)-1, q=self.q.tolist(), static=self.ik.last_collision_pairs,
                    payload=self.payload_contact_failures()))
                raise RuntimeError(f"collision in {self.stage} for {self.box_id}")
            self.payload_checked_frames += 1

    def joint_path(self, start, goal, label):
        if self.stage != label:
            self.enter(label)
        attempt=getattr(self,'joint_path_attempt',0)
        self.joint_path_attempt=attempt+1
        boundary=dict(box_id=self.box_id,stage=label,start=np.asarray(start).tolist(),goal=np.asarray(goal).tolist(),
            dimensions=list(self.current_box_size),attached=bool(self.attached),
            high_carry=bool(self.high_carry),high_carry_bottom_z=float(self.high_carry_bottom_z),
            grasp_xyz=np.asarray(self.grasp_xyz).tolist(),grasp_quat=self.grasp_rotation.as_quat().tolist(),
            placed={key:value.tolist() for key,value in self.placed.items()},valid_calls=0,rejected=0,rejection_samples=[])
        capture=self.output/f'joint_path_{attempt:03d}.json'
        write_json(capture,boundary)

        def checked(q):
            ok=self.valid(q)
            boundary['valid_calls']+=1
            if not ok:
                boundary['rejected']+=1
                if len(boundary['rejection_samples'])<12:
                    if np.isfinite(q).all():
                        self.refresh(q)
                    boundary['rejection_samples'].append(dict(q=np.asarray(q).tolist(),
                        robot=self.ik.last_collision_pairs,payload=self.payload_contact_failures()))
            if boundary['valid_calls']%100==0:
                write_json(capture,boundary)
            return ok

        try:
            path = ([np.asarray(start), np.asarray(goal)] if not self.validate_motion else
                    plan_joint_approach(start, goal, self.model.joint_limit_lower.numpy()[:6],
                                      self.model.joint_limit_upper.numpy()[:6], checked,
                                      resolution=.035, max_seconds=60))
            boundary['success']=True
            boundary['path']=[q.tolist() for q in path]
            write_json(capture,boundary)
        except BaseException as exc:
            boundary.update(success=False,error=str(exc))
            write_json(capture,boundary)
            self.q=np.asarray(start).copy()
            self.refresh(self.q)
            raise
        self.q = np.asarray(start).copy()
        for first, last in zip(path, path[1:]):
            count = max(12, int(np.ceil(np.max(np.abs(last-first))/.018)))
            for alpha in np.linspace(0, 1, count+1)[1:]:
                self.q = first + (alpha*alpha*(3-2*alpha))*(last-first)
                self.frame()
        return [p.tolist() for p in path]

    def reference_frame(self, data, index):
        """Reuse original joints exactly; retain the actual new rigid grasp."""
        stage = str(data["stage"][index])
        if stage != self.stage:
            self.enter(stage)
        self.q = data["joint_q"][index].copy()
        self.attached = bool(data["attached"][index])
        if stage == "RELEASE":
            self.placed[self.box_id] = self.box_pose.copy()
        if self.validate_motion:
            if not self.valid(self.q):
                write_json(self.output / "failure_frame.json", dict(stage=self.stage, box_id=self.box_id,
                    source_frame=index, q=self.q.tolist(), static=self.ik.last_collision_pairs,
                    payload=self.payload_contact_failures()))
                raise RuntimeError(f"recorded motion contact at frame {index}")
        else:
            self.refresh(self.q)
        bodies = self.state.body_q.numpy()
        self.box_pose = bodies[self.body_ids[self.box_id]].copy()
        if self.validate_motion:
            self.payload_checked_frames += 1  # valid() above already checked contacts
        self.history.append((self.q.copy(), bodies[self.contact_body].copy(), self.box_pose.copy(),
                             stage, self.box_id, self.attached))
        self.body_frames.append(bodies.copy())


def prepare_bridge(base_folder, output, feeder_xy, *, validate=True, observe_at_base=False,
                   feeder_world_xy=None, observation_base_xy=None):
    base_folder, output = Path(base_folder), Path(output)
    source_plan = (check_bundle(base_folder) if validate else
                   json.loads((base_folder/"plan.json").read_text(encoding="utf-8")))
    output.mkdir(parents=True, exist_ok=False)
    for name in ("input.json", "domain.json", "scene_tf.yaml", "model_manifest.json"):
        shutil.copyfile(base_folder/name, output/name)
    manifest = json.loads((output/"input.json").read_text(encoding="utf-8"))
    scene = output/"scene_tf.yaml"
    hull = Hull.from_config(yaml.safe_load(scene.read_text(encoding="utf-8")))
    scene_config = yaml.safe_load(scene.read_text(encoding="utf-8"))
    plan = copy.deepcopy(source_plan)
    plan.pop("feeder_center_container_door_floor_m", None)
    plan["feeder"] = dict(frame="world" if feeder_world_xy is not None else "elfin_base",
                          xy_m=list(feeder_world_xy if feeder_world_xy is not None else feeder_xy),
                          bottom_world_z_m=0)
    plan["ground_carry_clearance"] = dict(pedestal_margin_m=.05,
        connector_bottom_above_pedestal_m=.15,
        strategy="outer_lift_retract_outside_pedestal_high_lift_checked_carry")
    write_json(output/"plan.json", plan)
    run = BridgeCompiler(output)
    run.pedestal = pedestal_geometry(scene_config)
    run.high_carry_bottom_z = float(run.pedestal[0][2] + run.pedestal[1][2] + .15)
    run.validate_motion = validate
    STAGE_NAMES.update(OBSERVE="抬高观察箱子", WAIT="返回观察位，等待下一箱")
    bridges, copied_frames = [], 0
    try:
        run.initialize(manifest["boxes"], hull,
                       SimpleNamespace(source_root=DEFAULT_SOURCE, scene_config=scene, device="cuda:0"))
        if validate:
            run.ik = CertifiedWorkspace(DEFAULT_SOURCE, scene, device="cuda:0",
                                       box_size=manifest["boxes"][0], trace=run.trace)
        run.checking = validate
        base = run.ik.model.body_q.numpy()[run.ik.base_body]
        xy = (base[:3]+Rotation.from_quat(base[3:]).apply([*feeder_xy, 0]))[:2]
        if feeder_world_xy is not None:
            xy = np.asarray(feeder_world_xy, dtype=float)
        observation_z = float(base[2])
        observation_xy = (xy if observation_base_xy is None else
            (base[:3] + Rotation.from_quat(base[3:]).apply([*observation_base_xy, 0]))[:2])
        plan["observation"] = dict(tool_contact_world_z_m=observation_z,
                                   feeder_world_xy_m=xy.tolist(),
                                   tool_contact_world_xy_m=observation_xy.tolist(),
                                   base_xy_m=observation_base_xy)
        with np.load(base_folder/"trajectory.npz") as data:
            for number, row in enumerate(plan["sequence"]):
                run.current_box_size = row["input_dims_m"]
                run.box_id, run.box_pose, run.attached = row["id"], None, False
                indices = np.flatnonzero(data["box_id"] == row["id"])
                carry_indices = indices[data["stage"][indices] == "CARRY"]
                # The inside-demo feeder can pass near the pedestal, outside
                # the extra ground-carry margin. Bypass that prefix using a
                # newly checked high connection to its last safe entry rail.
                start = safe_carry_entry(carry_indices, data["box_pose"], row["input_dims_m"], run.pedestal)
                end = int(indices[data["stage"][indices] == "VERIFY"][0])
                dock_q = data["joint_q"][start].copy()
                dock_box, dock_tool = data["box_pose"][start], data["tool_pose"][start]
                inv = Rotation.from_quat(dock_tool[3:]).inv()
                run.grasp_xyz = inv.apply(dock_box[:3]-dock_tool[:3])
                run.grasp_rotation = inv*Rotation.from_quat(dock_box[3:])
                height = row["placed_dims_m"][2]
                yaw = float(run.container["rotation_rpy"][2]) + np.deg2rad(row["orientation"]["yaw_deg"])
                pickup = np.array([*xy, height/2])
                contact = np.array([*xy, height])
                hover = contact+[0, 0, .12]
                # Lift in place before any lateral transfer. Keep the lowest
                # payload corner >=15cm above the pedestal throughout connector.
                high_contact = contact.copy()
                high_contact[2] = run.high_carry_bottom_z + height + .005
                observation = np.array([*observation_xy, observation_z]) if observe_at_base else hover
                previous_q = run.q.copy()
                run.q = dock_q.copy()
                run.ik.q_last = dock_q.astype(np.float32)
                run.stage = "RETURN"
                hover_q = run.solve(hover, yaw)
                observation_q = run.solve(observation, yaw) if observe_at_base else hover_q
                if number == 0:
                    run.q = observation_q
                    run.enter("RESET_HOME")
                    run.frame()
                else:
                    run.q = previous_q
                    run.joint_path(previous_q, observation_q, "RETURN")
                run.box_pose = np.r_[pickup, Rotation.from_euler("z", yaw).as_quat()]
                row["spawn_position_world_m"] = pickup.tolist()
                row["spawn_bottom_world_z_m"] = 0
                row.pop("spawn_center_container_door_floor_m", None)
                run.enter("SPAWN", input_index=row["input_index"], pose_world=run.box_pose.tolist())
                run.hold(30)
                if observe_at_base:
                    run.enter("OBSERVE")
                    run.hold(60)
                run.enter("APPROACH")
                run.move(contact+[0,0,.08], yaw)
                run.enter("PICK")
                run.move(contact, yaw)
                tool = run.state.body_q.numpy()[run.contact_body]
                inv = Rotation.from_quat(tool[3:]).inv()
                run.grasp_xyz = inv.apply(run.box_pose[:3]-tool[:3])
                run.grasp_rotation = inv*Rotation.from_quat(run.box_pose[3:])
                run.attached = True
                run.enter("ATTACH")
                run.hold()
                run.enter("LIFT")
                staging_xy = outside_staging_xy(xy, run.pedestal, run.current_box_size)
                outer_lift_z = max(float(hover[2]), min(observation_z, float(high_contact[2])))
                run.move([*xy, outer_lift_z], yaw)
                run.move([*staging_xy, outer_lift_z], yaw)
                run.move([*staging_xy, high_contact[2]], yaw)
                # A larger payload can have a low original feeder/dock pose.
                # Complete the lateral transfer above the pedestal first,
                # then descend vertically into that reviewed docking pose.
                high_dock = dock_tool[:3].copy()
                high_dock[2] += max(0.0, run.high_carry_bottom_z + .005
                                   - box_bottom(dock_box, run.current_box_size))
                high_dock_q = run.solve(high_dock, yaw)
                run.high_carry = True
                portal_alignment=None
                if float(scene_config['container']['opening'].get('clearance_m',0))>0:
                    outside_dock=portal_staging_point(high_dock,run.container)
                    outside_q=run.solve(outside_dock,yaw)
                    connector=run.joint_path(run.q.copy(),outside_q,'CARRY')
                    alignment_start=len(run.history)
                    run.move(high_dock,yaw)
                    portal_alignment=dict(outside_tool_world_m=outside_dock.tolist(),
                        dock_tool_world_m=high_dock.tolist(),
                        recorded_joints=[frame[0].tolist() for frame in run.history[alignment_start:]])
                else:
                    connector = run.joint_path(run.q.copy(), high_dock_q, "CARRY")
                run.high_carry = False
                run.move(dock_tool[:3], yaw)
                docking = run.joint_path(run.q.copy(), dock_q, "CARRY")
                if validate and np.linalg.norm(run.box_pose[:3]-dock_box[:3]) > .006:
                    raise RuntimeError("connector does not match original feeder")
                first_output_frame = len(run.history)
                for index in range(start, end):
                    run.reference_frame(data, index)
                    copied_frames += 1
                if observe_at_base and observation_base_xy is not None:
                    empty_return = run.joint_path(run.q.copy(), observation_q, "RETURN")
                else:
                    empty_return = run.joint_path(run.q.copy(), hover_q, "RETURN")
                if observe_at_base and observation_base_xy is None:
                    run.joint_path(run.q.copy(), observation_q, "RETURN")
                run.enter("VERIFY" if validate else "WAIT")
                error = float(np.linalg.norm(run.box_pose[:3]-row["position_world_m"]))
                angle = float((Rotation.from_quat(run.box_pose[3:]).inv()*
                               Rotation.from_quat(row["orientation_world_xyzw"])).magnitude())
                run.events.write(json.dumps(dict(stage="VERIFY_RESULT", box_id=row["id"],
                    position_error_m=error, rotation_error_rad=angle))+"\n")
                run.events.flush()
                run.hold()
                bridges.append(dict(id=row["id"], carry_joint_path=connector,
                    docking_joint_path=docking,
                    empty_return_joint_path=empty_return, original_frame_range=[start,end]))
                if portal_alignment is not None:
                    bridges[-1]['portal_alignment']=portal_alignment
                bridges[-1]["reused_output_frame_range"] = [first_output_frame, first_output_frame+end-start]
                write_json(output/"bridges.json", bridges)
                write_json(output/"plan.json", plan)
        run.enter("COMPLETE", placed=len(bridges))
        write_json(output/"result.json", dict(success=True, placed=len(bridges)))
    except BaseException as exc:
        write_json(output/"result.json", dict(success=False, failed_stage=run.stage,
                   box_id=run.box_id, error=str(exc)))
        raise
    finally:
        run.close()
    if not validate:
        write_json(output/"MOTION_READY.json", dict(boxes=len(bridges), feeder=dict(plan["feeder"]),
                   observation_world_z_m=observation_z, ground_world_z_m=0,
                   collision_validation="not_run", trajectory_audit="not_run"))
        return dict(boxes=len(bridges), motion_validation="not_run")
    validation = verify(output)
    with np.load(output/"trajectory.npz") as recording:
        validation["scene_recording"] = validate_recording(plan["sequence"], recording)
    validation.update(copied_original_frames=copied_frames,
                      payload_checked_frames=run.payload_checked_frames,
                      penetration_tolerance_m=run.contact_tolerance_m,
                      original_joint_motion_replayed_exactly=True)
    write_json(output/"validation.json", validation)
    freeze_bundle(output)
    return validation
