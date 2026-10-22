"""URDF/scene-conditioned finite reachability certificates, with joint witnesses.

Certificates cover discrete top-contact poses only. They do not fill or
interpolate a continuous six-dimensional workspace.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import newton
from demo_collision import surface_gaps

from run_pick_place_pipeline import TracedIKWorkspace
from newton_ik_planner import tool_down_quaternion


class CertifiedWorkspace(TracedIKWorkspace):
    def __init__(self, *args, **kwargs):
        self.srdf_path = Path(__file__).resolve().parents[1] / "src/luggage_description/config/S20_with_camera.srdf"
        self.allowed_pairs = {frozenset((r.get("link1"), r.get("link2")))
                              for r in ET.parse(self.srdf_path).getroot().findall("disable_collisions")}
        super().__init__(*args, **kwargs)
        self.groups = list(range(self.model.body_count))

        def root(index):
            while self.groups[index] != index:
                index = self.groups[index]
            return index

        parents = self.model.joint_parent.numpy()
        children = self.model.joint_child.numpy()
        for parent, child, kind in zip(parents, children, self.model.joint_type.numpy()):
            if int(kind) == 3 and parent >= 0:
                self.groups[root(int(child))] = root(int(parent))
        self.groups = [root(i) for i in range(len(self.groups))]
        self.adjacent = {frozenset((self.groups[int(p)], self.groups[int(c)]))
                         for p, c in zip(parents, children) if p >= 0}
        self.last_collision_pairs = []
        self.penetration_tolerance_m = .001
        self.allowed_shapes = np.zeros((self.model.shape_count, self.model.shape_count), dtype=bool)
        bodies = self.model.shape_body.numpy()
        for a, ba in enumerate(bodies):
            for b, bb in enumerate(bodies):
                allowed = self.payload_body in (ba, bb) or (ba < 0 and bb < 0)
                if ba >= 0 and bb >= 0:
                    ga, gb = self.groups[ba], self.groups[bb]
                    labels = frozenset((self.model.body_label[ba].split("/")[-1],
                                        self.model.body_label[bb].split("/")[-1]))
                    allowed |= ga == gb or frozenset((ga, gb)) in self.adjacent or labels in self.allowed_pairs
                elif ba >= 0 or bb >= 0:
                    static_shape = a if ba < 0 else b
                    allowed |= (max(ba, bb) == self.base_body
                                and self.model.shape_label[static_shape] == "robot_pedestal")
                self.allowed_shapes[a, b] = allowed

    def solve_world_pose(self, target_position, yaw, *, check_collision=True, orientation_xyzw=None):
        if not check_collision:
            raise ValueError("certified workspace never disables collision checking")
        ok, q = super().solve_world_pose(target_position, yaw, check_collision=True,
                                         orientation_xyzw=orientation_xyzw)
        if ok:
            self.model.joint_q.assign(q)
            newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)
            clear = self._collision_free()
            self.last_diagnostics = dict(self.last_diagnostics, collision_free=clear)
            if not clear:
                return False, None
        return ok, q

    def _collision_free(self):
        state = self.model.state()
        state.body_q.assign(self.model.body_q)
        contacts = self.model.contacts()
        self.model.collide(state, contacts)
        count = int(contacts.rigid_contact_count.numpy()[0])
        first = contacts.rigid_contact_shape0.numpy()[:count]
        second = contacts.rigid_contact_shape1.numpy()[:count]
        p0 = contacts.rigid_contact_point0.numpy()[:count]
        p1 = contacts.rigid_contact_point1.numpy()[:count]
        normals = contacts.rigid_contact_normal.numpy()[:count]
        margins = (contacts.rigid_contact_margin0.numpy()[:count]
                   + contacts.rigid_contact_margin1.numpy()[:count])
        transforms = self.model.body_q.numpy()
        bodies = self.model.shape_body.numpy()
        rejected = {}
        keep = ~self.allowed_shapes[first, second]
        first, second = first[keep], second[keep]
        gaps = surface_gaps(transforms, bodies[first], bodies[second], p0[keep], p1[keep],
                            normals[keep], margins[keep], np.zeros(int(keep.sum())))
        for i in np.flatnonzero(gaps < -self.penetration_tolerance_m):
            key = (self.model.shape_label[int(first[i])], self.model.shape_label[int(second[i])])
            rejected[key] = min(rejected.get(key, 0), float(gaps[i]))
        self.last_collision_pairs = [dict(shapes=list(pair), gap_m=gap) for pair, gap in rejected.items()]
        return not rejected


def explore_domain(workspace, hull, output, *, progress=print, top_heights=None,
                   y_positions=None, prior=None):
    # Coordinates are top-contact centres, referenced to the door/floor.
    xs = [.20, .30, .45, .60]
    ys = list(y_positions) if y_positions is not None else [.28, .53, .78, 1.03, 1.28]
    heights = (list(top_heights) if top_heights is not None else
               [round(.25 + i * .05, 2) for i in range(22)])
    yaws = [0.0, -math.pi / 2]
    points, rejected = list(prior["poses"]) if prior else [], []
    previous = set()
    if prior:
        grid = prior["grid"]
        if not (set(grid["x_m"]) <= set(xs) and set(grid["y_m"]) <= set(ys)
                and set(grid["top_height_m"]) <= set(heights)
                and set(grid["yaw_rad"]) <= set(yaws)):
            raise ValueError("source domain grid is not a subset of requested grid")
        previous = {(x, y, height, yaw) for x in grid["x_m"] for y in grid["y_m"]
                    for height in grid["top_height_m"] for yaw in grid["yaw_rad"]}
    total = len(xs) * len(ys) * len(heights) * len(yaws)
    for yaw in yaws:
        for height in heights:
            for x in xs:
                for y in ys:
                    if (x, y, height, yaw) in previous:
                        continue
                    # Zero dimensions make local_contact_world's result the
                    # exact top point (no box-half-height adapter offset).
                    position, world_yaw = workspace.local_contact_world((x, y, height), (0, 0, 0), yaw, hull)
                    ok, q = workspace.solve_world_pose(position, world_yaw)
                    record = dict(contact_local_m=[x, y, height], contact_world_m=position.tolist(),
                                  box_yaw_rad=yaw, tool_orientation_xyzw=workspace.tool_orientation(world_yaw).tolist())
                    if ok:
                        record.update(joint_q=q.tolist(), diagnostics=dict(workspace.last_diagnostics))
                        points.append(record)
                    else:
                        rejected.append(dict(**record, diagnostics=dict(workspace.last_diagnostics),
                                             collisions=workspace.last_collision_pairs))
                    tried = len(previous) + len(points) - (len(prior["poses"]) if prior else 0) + len(rejected)
                    if tried % 40 == 0:
                        progress(f"EXPLORE_DOMAIN {tried}/{total}, certified={len(points)}")
    domain = dict(schema_version=1, representation="finite_certified_poses",
                  interpolation_allowed=False, frame="container_door_floor",
                  grid=dict(x_m=xs, y_m=ys, top_height_m=heights, yaw_rad=yaws),
                  scope="robot/tool versus static scene and self; no payload",
                  exclusions=["virtual URDF payload", "fixed rigid group internal contacts",
                              "adjacent URDF rigid groups", "SRDF disable_collisions pairs", "robot base against pedestal"],
                  srdf=str(workspace.srdf_path),
                  thresholds=dict(position_m=.003, orientation_rad=.035, penetration_m=.001),
                  tested=total, accepted=len(points), poses=points)
    if prior:
        domain["reused_rechecked_poses"] = len(prior["poses"])
        domain["previously_explored_grid_points"] = len(previous)
    output.write_text(json.dumps(domain, indent=2), encoding="utf-8")
    output.with_name("domain_rejected.json").write_text(json.dumps(rejected), encoding="utf-8")
    return domain


def recheck_domain(workspace, hull, source, output, progress=print):
    """Re-certify pre-explored joint witnesses under the current exact gate."""
    original = json.loads(source.read_text(encoding="utf-8"))
    accepted, rejected = [], []
    for index, record in enumerate(original["poses"]):
        q = np.asarray(record["joint_q"], dtype=np.float32)
        target, yaw = workspace.local_contact_world(record["contact_local_m"], (0, 0, 0),
                                                   record["box_yaw_rad"], hull)
        workspace.model.joint_q.assign(q)
        newton.eval_fk(workspace.model, workspace.model.joint_q, workspace.model.joint_qd, workspace.model)
        actual = workspace.model.body_q.numpy()[workspace.contact_body]
        position_error = float(np.linalg.norm(actual[:3] - target))
        angle_error = workspace._quat_error(actual[3:], workspace.tool_orientation(yaw))
        limits = bool(np.all(q >= workspace.model.joint_limit_lower.numpy()[:6] - 1e-4)
                      and np.all(q <= workspace.model.joint_limit_upper.numpy()[:6] + 1e-4))
        clear = workspace._collision_free()
        diagnostics = dict(position_error_m=position_error, rotation_error_rad=angle_error,
                           joint_limits=limits, collision_free=clear)
        ok = position_error <= .003 and angle_error <= .035 and limits and clear
        if ok:
            accepted.append(dict(record, diagnostics=diagnostics))
        else:
            rejected.append(dict(source_index=index, diagnostics=diagnostics,
                                 collisions=workspace.last_collision_pairs))
        workspace.trace.write(json.dumps(dict(boundary="domain_recheck", source_index=index,
                                              ok=ok, diagnostics=diagnostics)) + "\n")
        if (index + 1) % 100 == 0:
            workspace.trace.flush()
            progress(f"RECHECK_DOMAIN {index+1}/{len(original['poses'])}, certified={len(accepted)}")
    result = dict(original, poses=accepted, accepted=len(accepted),
                  rechecked_source_count=len(original["poses"]),
                  effective_surface_gap_checked=True)
    result["thresholds"]["penetration_m"] = workspace.penetration_tolerance_m
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    output.with_name("domain_rejected.json").write_text(json.dumps(rejected), encoding="utf-8")
    return result
