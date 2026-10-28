"""Newton-backed IK feasibility checks for the online packing planner.

The checker is deliberately conservative: every sampled Cartesian waypoint
must have a Newton IK solution with the real S20 URDF, joint limits, and no
contact between a non-base arm link and the container collision mesh. The
payload/container relation is checked by the packing hull itself.
"""

from __future__ import annotations

import math
from pathlib import Path

import newton
import numpy as np
import warp as wp
from scene_frames import container_pose, container_tool_orientation, transform_point
from newton import ik

from roboticarm_newton import add_cell_scene


# This is the smallest carried-box profile with a useful tool-down workspace
# in the deploy placement. It also matches the existing S20 reachability data.
DEFAULT_BOX_SIZE = (0.55, 0.40, 0.25)
IK_POSITION_TOLERANCE = 0.003
IK_ROTATION_TOLERANCE = 0.035
ROUTE_SAMPLE_STEP = 0.05
# Conservative shoulder-to-tool-plus-payload bound for the deploy S20. This is
# the sum of the URDF joint-origin offsets (including the suction panel) with
# 8 cm slack for the adapter and the grasped box. It is only a broad-phase
# rejection radius; in-range targets still require strict IK and collision checks.
S20_CONSERVATIVE_REACH_M = 2.65


def tool_down_quaternion(yaw: float) -> np.ndarray:
    """Return ROS-compatible xyzw tool-down orientation for a world yaw."""
    half = 0.5 * float(yaw)
    return np.array([math.cos(half), math.sin(half), 0.0, 0.0], dtype=np.float32)


def box_yaw(source_dims, dims) -> float:
    """Yaw for the two supported flat orientations of a box."""
    source = tuple(round(float(v), 8) for v in source_dims)
    target = tuple(round(float(v), 8) for v in dims)
    if target == source:
        return 0.0
    if target == (source[1], source[0], source[2]):
        return -math.pi / 2.0
    raise ValueError(
        "IK mode only supports flat box orientations; got %s from %s" %
        (target, source)
    )


class NewtonIKWorkspace:
    """Reusable S20 IK solver bound to the active Newton loading scene."""

    position_tolerance_m = IK_POSITION_TOLERANCE

    def __init__(
        self,
        source_root: Path,
        scene_config_path: Path,
        container_y_override: float | None = None,
        device: str = "cuda:0",
        box_size=DEFAULT_BOX_SIZE,
    ):
        wp.set_device(device)
        self.box_size = tuple(float(v) for v in box_size)
        builder = newton.ModelBuilder()
        builder.default_shape_cfg.mu = 0.7
        builder.default_joint_cfg.target_ke = 6000.0
        builder.default_joint_cfg.target_kd = 120.0
        self.config = add_cell_scene(
            builder,
            source_root,
            show_robot=True,
            scene_config_path=scene_config_path,
            container_y_override=container_y_override,
            box_size=self.box_size,
        )
        self.model = builder.finalize()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)
        self.contact_body = next(
            i for i, label in enumerate(self.model.body_label)
            if label.endswith("/suction_contact_frame")
        )
        self.base_body = next(
            i for i, label in enumerate(self.model.body_label)
            if label.endswith("/elfin_base")
        )
        base_pose = self.model.body_q.numpy()[self.base_body]
        self.shoulder_world = np.asarray(base_pose[:3], dtype=np.float32) + np.array(
            [0.0, 0.0, 0.171], dtype=np.float32
        )
        self.reach_radius = S20_CONSERVATIVE_REACH_M
        self.payload_body = next(
            i for i, label in enumerate(self.model.body_label)
            if label.endswith("/payload_box")
        )
        self.container_shape = self.model.shape_label.index("container_collision_full")
        self.robot_shape_bodies = {
            int(body) for body in self.model.shape_body.numpy() if int(body) >= 0
        }
        self.q_default = self.model.joint_q.numpy().astype(np.float32).copy()
        self.q_last = self.q_default.copy()
        self._target_position = wp.array([wp.vec3(0.0, 0.0, 0.0)], dtype=wp.vec3)
        self._target_rotation = wp.array(
            [wp.vec4(1.0, 0.0, 0.0, 0.0)], dtype=wp.vec4
        )
        self.position_objective = ik.IKObjectivePosition(
            link_index=self.contact_body,
            link_offset=wp.vec3(0.0, 0.0, 0.0),
            target_positions=self._target_position,
            weight=1.0,
        )
        self.rotation_objective = ik.IKObjectiveRotation(
            link_index=self.contact_body,
            link_offset_rotation=wp.quat_identity(),
            target_rotations=self._target_rotation,
            weight=0.7,
        )
        self.limit_objective = ik.IKObjectiveJointLimit(
            joint_limit_lower=self.model.joint_limit_lower,
            joint_limit_upper=self.model.joint_limit_upper,
            weight=10.0,
        )
        self.solver = ik.IKSolver(
            model=self.model,
            n_problems=1,
            objectives=[
                self.position_objective,
                self.rotation_objective,
                self.limit_objective,
            ],
            lambda_initial=0.1,
            jacobian_mode=ik.IKJacobianType.ANALYTIC,
        )
        self.cache: dict[tuple[float, ...], tuple[bool, np.ndarray | None]] = {}
        self.last_diagnostics = {}

    @staticmethod
    def box_yaw(source_dims, dims) -> float:
        return box_yaw(source_dims, dims)

    @staticmethod
    def _quat_error(actual: np.ndarray, target: np.ndarray) -> float:
        dot = abs(float(np.dot(actual, target)))
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    def _collision_free(self) -> bool:
        state = self.model.state()
        state.body_q.assign(self.model.body_q)
        contacts = self.model.contacts()
        self.model.collide(state, contacts)
        count = int(contacts.rigid_contact_count.numpy()[0])
        if count <= 0:
            return True
        shape0 = contacts.rigid_contact_shape0.numpy()[:count]
        shape1 = contacts.rigid_contact_shape1.numpy()[:count]
        bodies = self.model.shape_body.numpy()
        for first, second in zip(shape0, shape1):
            first = int(first)
            second = int(second)
            if first != self.container_shape and second != self.container_shape:
                continue
            other = second if first == self.container_shape else first
            body = int(bodies[other])
            # The grasped box is expected to touch the sill/floor at the
            # insertion and final placement waypoints. Its swept AABB is
            # checked independently by the packing hull, so only arm links
            # are hard-rejected here.
            if body >= 0 and body not in (self.base_body, self.payload_body):
                return False
        return True

    def _solve_once(self, target_position, target_rotation, seed, check_collision=True) -> tuple[bool, np.ndarray]:
        self.position_objective.set_target_position(0, wp.vec3(*target_position))
        self.rotation_objective.set_target_rotation(0, wp.vec4(*target_rotation))
        q = wp.array(np.asarray(seed, dtype=np.float32).reshape(1, -1), dtype=wp.float32)
        self.solver.step(q, q, iterations=35)
        solution = q.numpy()[0].astype(np.float32)
        self.model.joint_q.assign(q.flatten())
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)
        actual = self.model.body_q.numpy()[self.contact_body]
        position_error = float(np.linalg.norm(actual[:3] - target_position))
        rotation_error = self._quat_error(actual[3:7], target_rotation)
        limits_lower = self.model.joint_limit_lower.numpy()[:6]
        limits_upper = self.model.joint_limit_upper.numpy()[:6]
        in_limits = bool(np.all(solution[:6] >= limits_lower - 1e-4) and
                         np.all(solution[:6] <= limits_upper + 1e-4))
        collision_free = self._collision_free() if check_collision else True
        ok = (
            position_error <= self.position_tolerance_m
            and rotation_error <= IK_ROTATION_TOLERANCE
            and in_limits
            and collision_free
        )
        self.last_diagnostics = {
            "position_error_m": position_error,
            "rotation_error_rad": rotation_error,
            "joint_limits": in_limits,
            "collision_free": collision_free,
        }
        return ok, solution

    def tool_orientation(self, yaw):
        return container_tool_orientation(self.config, yaw)

    def solve_reachable_world_pose(self, target_position, yaw, *, orientation_xyzw=None):
        """Extend along the requested ray, recording a distinct achieved goal.

        This opt-in transit API retains orientation, joint limits and scene
        collision checks. Pick/place contacts must use solve_world_pose.
        """
        from reach_projection import furthest_reachable
        original_seed = self.q_last.copy()
        def solve(point):
            self.q_last = original_seed.copy()
            ok, joints = self.solve_world_pose(point, yaw, orientation_xyzw=orientation_xyzw)
            return joints if ok else None
        delta = np.asarray(target_position, dtype=float)-self.shoulder_world
        distance = float(np.linalg.norm(delta))
        # Begin inside the URDF-derived broad-phase bound even for a target
        # arbitrarily far away. Search the full radial interval, not just an
        # arbitrary fraction of the original request.
        search_target = (self.shoulder_world + delta*self.reach_radius/distance
                         if distance > self.reach_radius else target_position)
        answer = furthest_reachable(self.shoulder_world, search_target, solve)
        if answer is None:
            self.q_last = original_seed
            return False, None
        # The last bisection probe may have failed; diagnostics and FK must
        # describe the accepted answer rather than that rejected probe.
        target_rotation = (self.tool_orientation(yaw) if orientation_xyzw is None
                           else np.asarray(orientation_xyzw, dtype=np.float32))
        ok, solution = self._solve_once(answer['point'], target_rotation,
                                        answer['solution'], True)
        if not ok:
            self.q_last = original_seed
            return False, None
        answer['solution'] = solution
        answer['shortfall_m'] = max(0., distance-float(np.linalg.norm(
            np.asarray(answer['point'])-self.shoulder_world)))
        self.q_last = answer['solution'].copy()
        self.last_diagnostics = dict(self.last_diagnostics,
            reach_projected=answer['shortfall_m'] > self.position_tolerance_m,
            requested_position_m=list(map(float, target_position)),
            achieved_goal_m=list(answer['point']), reach_shortfall_m=answer['shortfall_m'])
        return True, answer['solution']

    def solve_world_pose(self, target_position, yaw: float, *, check_collision=True,
                         orientation_xyzw=None) -> tuple[bool, np.ndarray | None]:
        target_position = np.asarray(target_position, dtype=np.float32)
        target_rotation = (self.tool_orientation(yaw) if orientation_xyzw is None else
                           np.asarray(orientation_xyzw, dtype=np.float32))
        key = (bool(check_collision), float(self.position_tolerance_m),
               *np.round(np.r_[target_position, target_rotation], 5))
        if key in self.cache:
            return self.cache[key]
        seeds = [self.q_last]
        if not np.allclose(self.q_last, self.q_default, atol=1e-5):
            seeds.append(self.q_default)
        for seed in seeds:
            ok, solution = self._solve_once(target_position, target_rotation, seed, check_collision)
            if ok:
                self.q_last = solution.copy()
                result = (True, solution.copy())
                self.cache[key] = result
                return result
        result = (False, None)
        self.cache[key] = result
        return result

    def local_contact_world(self, lower_left, dims, yaw, hull) -> tuple[np.ndarray, float]:
        lx, ly, lz = map(float, dims)
        x, y, z = map(float, lower_left)
        container = next(
            item for item in self.config["static_transforms"]
            if item["child"] == "container_link"
        )
        container_yaw = float(container["rotation_rpy"][2])
        local = np.array(
            [hull.door_x + x + lx / 2.0,
             hull.inner_y0 + y + ly / 2.0,
             hull.floor_z + z + lz / 2.0],
            dtype=np.float32,
        )
        world = transform_point(container, local).astype(np.float32)
        return world, container_yaw + float(yaw)

    def reachable_x_interval(self, dims, y, z, hull, radius=None):
        """Return a conservative local-x interval for endpoint targets.

        The contact point moves on a straight line as container-local x changes,
        so intersecting it with the S20 reach sphere can be solved analytically.
        """
        lx, ly, lz = map(float, dims)
        p0, _ = self.local_contact_world((0.0, y, z), dims, 0.0, hull)
        p1, _ = self.local_contact_world((1.0, y, z), dims, 0.0, hull)
        direction = p1 - p0
        rel = p0 - self.shoulder_world
        a = float(np.dot(direction, direction))
        b = 2.0 * float(np.dot(rel, direction))
        c = float(np.dot(rel, rel)) - float(radius or self.reach_radius) ** 2
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0 or a <= 1e-12:
            return None
        root = math.sqrt(discriminant)
        low = (-b - root) / (2.0 * a)
        high = (-b + root) / (2.0 * a)
        return max(0.0, low), min(float(hull.length - lx), high)

    def route_is_feasible(self, route, dims, yaw, hull) -> bool:
        points = [np.asarray(point, dtype=np.float32) for point in route]
        if not points:
            return False
        # Reject an entire Cartesian route before sampling it into many IK
        # solves. A straight segment is inside a sphere iff both endpoints
        # are inside it, so this test is conservative and constant-time.
        for point in points:
            world, _ = self.local_contact_world(point, dims, yaw, hull)
            if float(np.linalg.norm(world - self.shoulder_world)) > self.reach_radius:
                return False
        # A failed placement endpoint or bend invalidates the route. Check
        # those few poses before paying for the dense Cartesian samples.
        for point in reversed(points):
            world, world_yaw = self.local_contact_world(point, dims, yaw, hull)
            ok, _ = self.solve_world_pose(world, world_yaw)
            if not ok:
                return False
        for start, end in zip(points, points[1:]):
            distance = float(np.linalg.norm(end - start))
            samples = max(1, int(math.ceil(distance / ROUTE_SAMPLE_STEP)))
            for i in range(1, samples):
                point = start + (end - start) * (i / samples)
                world, world_yaw = self.local_contact_world(point, dims, yaw, hull)
                ok, _ = self.solve_world_pose(world, world_yaw)
                if not ok:
                    return False
        return True
