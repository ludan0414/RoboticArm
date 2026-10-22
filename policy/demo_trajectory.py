"""Compile and validate all ten pick/place/return motions before any viewer."""
import math
import json
import numpy as np
from scipy.spatial.transform import Rotation

from demo_contract import write_json
from demo_recording import validate_recording
from online_packing_newton import Pose, world_pose
from run_pick_place_pipeline import Pipeline, STAGE_NAMES
from verify_pick_place_run import verify

STAGE_NAMES.update(RETURN="空手返回供料位", ROTATE="空手调整工具朝向")


class MotionCompiler(Pipeline):
    def __init__(self, output):
        super().__init__(output, "headless")
        self.body_frames = []
        self.payload_checked_frames = 0
        self.contact_tolerance_m = .001

    def frame(self):
        super().frame()
        self.body_frames.append(self.state.body_q.numpy().copy())
        if self.box_pose is not None:
            self.check_payload_contacts()

    def payload_contact_failures(self):
        """Return payload/scene and arm/cargo penetrations without recording."""
        contacts = self.model.contacts()
        self.model.collide(self.state, contacts)
        count = int(contacts.rigid_contact_count.numpy()[0])
        a = contacts.rigid_contact_shape0.numpy()[:count]
        b = contacts.rigid_contact_shape1.numpy()[:count]
        point0 = contacts.rigid_contact_point0.numpy()[:count]
        point1 = contacts.rigid_contact_point1.numpy()[:count]
        normal = contacts.rigid_contact_normal.numpy()[:count]
        margins = (contacts.rigid_contact_margin0.numpy()[:count]
                   + contacts.rigid_contact_margin1.numpy()[:count])
        shape_body = self.model.shape_body.numpy()
        body_q = self.state.body_q.numpy()
        visible = {self.body_ids[name] for name in self.placed} | {self.body_ids[self.box_id]}
        all_boxes = set(self.body_ids.values())
        current = self.body_ids[self.box_id]
        virtual = self.model.body_label.index("S20/payload_box")

        def world_point(body, point):
            if body < 0:
                return point
            return body_q[body, :3] + Rotation.from_quat(body_q[body, 3:]).apply(point)

        failures = []
        for i, (first, second) in enumerate(zip(a, b)):
            ba, bb = int(shape_body[int(first)]), int(shape_body[int(second)])
            if virtual in (ba, bb) or not ({ba, bb} & visible):
                continue
            if (ba in all_boxes and ba not in visible) or (bb in all_boxes and bb not in visible):
                continue
            other = bb if ba == current else ba
            if (current in (ba, bb) and other >= 0
                    and self.model.body_label[other] == "S20/suction_panel"
                    and self.stage in ("PICK", "ATTACH", "LIFT", "CARRY", "PLACE")):
                continue
            gap = float(np.dot(world_point(bb, point1[i]) - world_point(ba, point0[i]), normal[i]) - margins[i])
            if gap < -self.contact_tolerance_m:
                failures.append(dict(shapes=[self.model.shape_label[int(first)], self.model.shape_label[int(second)]],
                                     bodies=["world" if ba < 0 else self.model.body_label[ba],
                                             "world" if bb < 0 else self.model.body_label[bb]], gap_m=gap))
        return failures

    def check_payload_contacts(self):
        """Check payload/scene and arm/cargo contact at every compiled frame."""
        failures = self.payload_contact_failures()
        self.payload_checked_frames += 1
        if failures:
            write_json(self.output / "failure_contacts.json", dict(stage=self.stage, box_id=self.box_id,
                       frame=len(self.history) - 1, contacts=failures))
            raise RuntimeError(f"payload/arm-cargo collision at {self.stage}: {failures[:2]}")

    def execute(self, rows):
        previous_yaw = None
        for index, row in enumerate(rows):
            self.box_id, self.box_pose = row["id"], None
            dims = np.asarray(row["placed_dims_m"])
            local_q = np.asarray(row["orientation"]["quaternion_xyzw"])
            route = [world_pose(Pose(np.asarray(p) + dims / 2, local_q), self.container, self.hull)
                     for p in row["route"]]
            contacts = [p.center + [0, 0, dims[2] / 2] for p in route]
            yaw = float(self.container["rotation_rpy"][2]) + math.radians(row["orientation"]["yaw_deg"])
            if index == 0:
                self.enter("RESET_HOME")
                self.q = self.solve(contacts[0] + [0, 0, .12], yaw)
                self.frame()
            else:
                position = self.state.body_q.numpy()[self.contact_body, :3].copy()
                if abs(yaw - previous_yaw) > 1e-6:
                    self.enter("ROTATE")
                    for angle in np.linspace(previous_yaw, yaw, 25)[1:]:
                        self.q = self.solve(position, float(angle))
                        self.frame()
                self.enter("RETURN")
                self.move(contacts[0] + [0, 0, .12], yaw)
            self.box_pose = np.r_[route[0].center, route[0].quat]
            self.enter("SPAWN", input_index=row["input_index"], pose_world=self.box_pose.tolist())
            self.hold(30)
            self.enter("APPROACH")
            self.move(contacts[0] + [0, 0, .08], yaw)
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
            self.move(contacts[0] + [0, 0, .08], yaw)
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
            self.move(contacts[-2], yaw)
            self.enter("RETURN")
            for contact in reversed(contacts[:-2]):
                self.move(contact, yaw)
            self.move(contacts[0] + [0, 0, .12], yaw)
            self.enter("VERIFY")
            position_error = float(np.linalg.norm(self.box_pose[:3] - route[-1].center))
            rotation_error = float((Rotation.from_quat(self.box_pose[3:]).inv()
                                   * Rotation.from_quat(route[-1].quat)).magnitude())
            if position_error > .006 or rotation_error > .07:
                raise RuntimeError("final placement outside tolerance")
            self.events.write(json.dumps(dict(stage="VERIFY_RESULT", box_id=self.box_id,
                              position_error_m=position_error, rotation_error_rad=rotation_error)) + "\n")
            self.events.flush()
            self.hold()
            previous_yaw = yaw

    def close(self):
        super().close()
        if self.history:
            with np.load(self.output / "trajectory.npz") as saved:
                data = {name: saved[name] for name in saved.files}
            data["body_q"] = np.asarray(self.body_frames, dtype=np.float32)
            data["body_labels"] = np.asarray(self.model.body_label)
            np.savez_compressed(self.output / "trajectory.npz", **data)


def compile_trajectory(args, manifest, hull, rows, workspace):
    run = MotionCompiler(args.output)
    try:
        run.initialize(manifest["boxes"], hull, args)
        run.ik = workspace
        run.execute(rows)
        run.enter("COMPLETE", placed=len(rows))
        write_json(args.output / "result.json", dict(success=True, placed=len(rows),
                   physics_verification="not_evaluated", episode_reset_count=1))
    except BaseException as exc:
        write_json(args.output / "result.json", dict(success=False, failed_stage=run.stage,
                   box_id=run.box_id, error=str(exc)))
        raise
    finally:
        run.close()
    validation = verify(args.output)
    with np.load(args.output / "trajectory.npz") as saved:
        validation["scene_recording"] = validate_recording(rows, saved)
    validation.update(payload_checked_frames=run.payload_checked_frames,
                      penetration_tolerance_m=run.contact_tolerance_m,
                      initial_reset_count=1, online_planning_during_replay=False)
    return validation
