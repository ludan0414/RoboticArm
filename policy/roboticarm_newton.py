"""Port the RoboticArm Gazebo loading cell to Newton using source meshes.

The S20's six revolute joints and collision STLs are retained. The real
container uses its full collision STL. A cuboid payload is fixed to the suction
contact frame while joint 6 rotates it. ROS/Gazebo controllers and sensors are
not emulated here; this is a mechanical motion and collision benchmark.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import statistics
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import yaml
from scene_frames import rotation

from container_demo import make_ground_grid  # configures Warp cache first

import newton
import warp as wp


DEFAULT_SOURCE = Path(__file__).resolve().parent / "vendor" / "roboticarm"
FRAME_DT = 1.0 / 60.0
XACRO_NS = "{http://www.ros.org/wiki/xacro}"
BOX_SIZE = (0.35, 0.25, 0.18)
BOX_MASS = 5.0
WRIST_AMPLITUDE = math.radians(40.0)
WRIST_PERIOD = 3.0


def scene_paths(root: Path) -> tuple[Path, Path, Path]:
    source = root / "elfin_humble_ws" / "src"
    return (
        source,
        source / "luggage_description" / "config" / "scene_tf.yaml",
        source / "luggage_gazebo" / "models",
    )


def xacro_properties(path: Path) -> dict[str, str]:
    xml = ET.parse(path).getroot()
    return {
        node.attrib["name"]: node.attrib["value"]
        for node in xml
        if node.tag == XACRO_NS + "property"
    }


def add_inertial(link: ET.Element, mass: float, diag: tuple[float, float, float]):
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(inertial, "inertia", ixx=str(diag[0]), ixy="0", ixz="0", iyy=str(diag[1]), iyz="0", izz=str(diag[2]))


def add_geometry(
    link: ET.Element,
    tag: str,
    *,
    mesh: Path | None = None,
    box: tuple[float, float, float] | None = None,
    origin: tuple[str, str] = ("0 0 0", "0 0 0"),
    scale: str = "1 1 1",
    color: str | None = None,
):
    node = ET.SubElement(link, tag)
    ET.SubElement(node, "origin", xyz=origin[0], rpy=origin[1])
    geometry = ET.SubElement(node, "geometry")
    if mesh is not None:
        ET.SubElement(geometry, "mesh", filename=mesh.as_posix(), scale=scale)
    elif box is not None:
        ET.SubElement(geometry, "box", size=" ".join(map(str, box)))
    if color and tag == "visual":
        material = ET.SubElement(node, "material", name="")
        ET.SubElement(material, "color", rgba=color)


def fixed_joint(robot: ET.Element, name: str, parent: str, child: str, xyz: str, rpy: str):
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)


def make_robot_urdf(source: Path, models: Path, box_size=BOX_SIZE) -> str:
    """Flatten only the S20 xacro pieces used by the Gazebo loading cell."""
    original = ET.parse(source / "elfin_description" / "urdf" / "S20.urdf.xacro").getroot()
    robot = ET.Element("robot", name="S20")
    omitted = {"world", "elfin_base_link", "elfin_dummy_gripper"}
    omitted_joints = {"world_base", "elfin_base_joint", "elfin_dummy_joint"}
    for child in original:
        if child.tag == "link" and child.get("name") not in omitted:
            robot.append(copy.deepcopy(child))
        elif child.tag == "joint" and child.get("name") not in omitted_joints:
            robot.append(copy.deepcopy(child))

    mesh_prefix = "package://elfin_description/"
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(mesh_prefix):
            mesh.set("filename", (source / "elfin_description" / filename[len(mesh_prefix):]).as_posix())

    description = source / "luggage_description"
    suction = xacro_properties(description / "config" / "suction_flange_origin.xacro")
    adapter = xacro_properties(description / "config" / "eef_mount_adapter_origin.xacro")
    camera = xacro_properties(description / "config" / "camera_mount_origin.xacro")
    lidar = xacro_properties(description / "config" / "mid360_origin.xacro")

    # The source xacro defines five separate CAD meshes on one fixed link.
    panel_link = ET.SubElement(robot, "link", name="suction_panel")
    panel_mesh_dir = models / "suction_panel" / "collision"
    panel_origin = ("-0.0909 -0.28583 -0.0909", "0 0 0")
    for i in range(1, 6):
        mesh = panel_mesh_dir / f"p{i:02d}.stl"
        add_geometry(panel_link, "visual", mesh=mesh, origin=panel_origin, color="0.75 0.75 0.78 1")
        add_geometry(panel_link, "collision", mesh=mesh, origin=panel_origin)
    add_inertial(panel_link, 0.5, (0.0012, 0.0006, 0.0012))
    fixed_joint(robot, "suction_panel_joint", suction["suction_flange_parent"], "suction_panel", suction["suction_flange_xyz"], suction["suction_flange_rpy"])

    contact = ET.SubElement(robot, "link", name="suction_contact_frame")
    fixed_joint(robot, "suction_contact_frame_joint", "suction_panel", contact.get("name"), "0 -0.286 0", "1.57079632679 0 0")

    # This kinematic model attaches the payload at its geometric center.
    payload = ET.SubElement(robot, "link", name="payload_box")
    for tag in ("visual", "collision"):
        add_geometry(payload, tag, box=tuple(map(float, box_size)), color="0.88 0.52 0.18 1")
    lx, ly, lz = map(float, box_size)
    add_inertial(payload, BOX_MASS, (
        BOX_MASS * (ly * ly + lz * lz) / 12,
        BOX_MASS * (lx * lx + lz * lz) / 12,
        BOX_MASS * (lx * lx + ly * ly) / 12,
    ))
    fixed_joint(robot, "payload_grip_joint", "suction_contact_frame", "payload_box", "0 0 0", "0 0 0")

    adapter_link = ET.SubElement(robot, "link", name="eef_mount_adapter")
    adapter_mesh = models / "arm_realsense" / "arm_realsense_v1.3.stl"
    for tag in ("visual", "collision"):
        add_geometry(adapter_link, tag, mesh=adapter_mesh, scale="0.001 0.001 0.001", color="0.6 0.6 0.65 1")
    add_inertial(adapter_link, 0.12, (0.00018, 0.00025, 0.00020))
    fixed_joint(robot, "eef_mount_adapter_joint", adapter["adapter_mount_parent"], "eef_mount_adapter", adapter["adapter_mount_xyz"], adapter["adapter_mount_rpy"])

    camera_link = ET.SubElement(robot, "link", name="camera_link")
    for tag in ("visual", "collision"):
        add_geometry(camera_link, tag, box=(0.02505, 0.090, 0.025), origin=("0 -0.0175 0", "0 0 0"), color="0.3 0.3 0.3 1")
    add_inertial(camera_link, 0.072, (0.003881243, 0.000498940, 0.003879257))
    fixed_joint(robot, "camera_mount_joint", camera["cam_mount_parent"], "camera_link", camera["cam_mount_xyz"], camera["cam_mount_rpy"])

    lidar_link = ET.SubElement(robot, "link", name="mid360_mount_frame")
    add_geometry(lidar_link, "visual", box=(0.065, 0.065, 0.060), origin=("0 0 0.030", "0 0 0"), color="0.15 0.45 0.75 1")
    add_inertial(lidar_link, 0.265, (0.00018, 0.00018, 0.00018))
    fixed_joint(robot, "mid360_mount_joint", "eef_mount_adapter", "mid360_mount_frame", lidar["mid360_mount_xyz"], lidar["mid360_mount_rpy"])

    return ET.tostring(robot, encoding="unicode")


def load_mesh(path: Path) -> newton.Mesh:
    if not path.is_file():
        raise FileNotFoundError(path)
    mesh = trimesh.load(path, force="mesh", process=False)
    return newton.Mesh(
        np.asarray(mesh.vertices, dtype=np.float32),
        np.asarray(mesh.faces, dtype=np.int32).reshape(-1),
        compute_inertia=False,
    )


def yaw_xform(position, yaw: float) -> wp.transform:
    return wp.transform(
        p=wp.vec3(*map(float, position)),
        q=wp.quat_from_axis_angle(wp.vec3(0, 0, 1), float(yaw)),
    )


def add_static_box(builder, label: str, position, size, yaw: float, color):
    builder.add_shape_box(
        body=-1,
        xform=yaw_xform(position, yaw),
        hx=float(size[0]) / 2,
        hy=float(size[1]) / 2,
        hz=float(size[2]) / 2,
        color=wp.vec3(*color),
        label=label,
    )


def add_cell_scene(builder, source_root: Path, *, show_robot: bool = True,
                   scene_config_path: Path | None = None,
                   container_y_override: float | None = None,
                   box_size=BOX_SIZE):
    source, default_config_path, models = scene_paths(source_root)
    config_path = scene_config_path or default_config_path
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if container_y_override is not None:
        container_transform = next(
            item for item in config["static_transforms"] if item["child"] == "container_link"
        )
        container_transform["translation"][1] = float(container_y_override)
    platform = config["pickup_platform"]

    # Gazebo's world ground and the static loading area.
    add_static_box(builder, "world_ground", (0, 0, -0.025), (12, 12, 0.05), 0, (0.15, 0.18, 0.22))
    mark_x = math.copysign(1.4, float(platform["translation"][0]))
    add_static_box(builder, "loading_zone_marking", (mark_x, -0.7, 0.002), (1.6, 1.4, 0.002), 0, (0.95, 0.90, 0.25))

    pedestal = config["pedestal"]
    px, py, pz = map(float, pedestal["translation"])
    psize = tuple(map(float, pedestal["size"]))
    pyaw = float(pedestal["rotation_rpy"][2])
    add_static_box(builder, "robot_pedestal", (px, py, pz + psize[2] / 2), psize, pyaw, (0.45, 0.45, 0.48))

    sx, sy, sz = map(float, platform["translation"])
    ssize = tuple(map(float, platform["size"]))
    syaw = float(platform["rotation_rpy"][2])
    if platform.get("enabled", True):
        add_static_box(builder, "pickup_platform", (sx, sy, sz + ssize[2] / 2), ssize, syaw, (0.35, 0.40, 0.45))

    container_model = config["gazebo"]["container_model"]
    container_dir = models / container_model / "meshes"
    container_pose = next(
        item for item in config["static_transforms"] if item["child"] == "container_link"
    )
    c_xform = wp.transform(wp.vec3(*container_pose["translation"]),
                           wp.quat(*rotation(container_pose).as_quat()))
    visual_cfg = builder.default_shape_cfg.copy()
    visual_cfg.density = 0.0
    visual_cfg.has_shape_collision = False
    visual_cfg.has_particle_collision = False
    collision_cfg = builder.default_shape_cfg.copy()
    collision_cfg.density = 0.0
    collision_cfg.is_visible = False
    # Full MoveIt collision STL, rather than the decimated Gazebo physics STL.
    builder.add_shape_mesh(-1, xform=c_xform, mesh=load_mesh(container_dir / "container_collision.stl"), cfg=collision_cfg, label="container_collision_full")
    builder.add_shape_mesh(-1, xform=c_xform, mesh=load_mesh(container_dir / "container_visual.stl"), cfg=visual_cfg, color=wp.vec3(0.35, 0.45, 0.55), label="container_visual")
    if float(config['container'].get('opening',{}).get('clearance_m',0.0))!=0:
        from doorway_scene import DoorwayGeometry
        doorway=DoorwayGeometry.from_scene(config)
        for guard in doorway.world_guard_boxes(container_pose['translation'],
                rotation(container_pose).as_quat().tolist(),collision_tolerance_m=.001):
            builder.add_shape_box(-1,xform=wp.transform(wp.vec3(*guard['xyz']),wp.quat(*guard['quat'])),
                hx=guard['size'][0]/2,hy=guard['size'][1]/2,hz=guard['size'][2]/2,
                cfg=collision_cfg,label=guard['id'])

    robot_urdf = make_robot_urdf(source, models, box_size=box_size)
    if not show_robot:
        robot_xml = ET.fromstring(robot_urdf)
        for link in robot_xml.findall("link"):
            for visual in link.findall("visual"):
                link.remove(visual)
        robot_urdf = ET.tostring(robot_xml, encoding="unicode")
    robot_pose = (px, py, pz + psize[2])
    builder.add_urdf(
        robot_urdf,
        xform=yaw_xform(robot_pose, pyaw + float(config["robot"]["rotation_rpy"][2])),
        floating=False,
        enable_self_collisions=True,
        ignore_inertial_definitions=False,
    )

    return config


def make_scene(source_root: Path):
    builder = newton.ModelBuilder()
    builder.default_shape_cfg.mu = 0.7
    builder.default_joint_cfg.target_ke = 6000.0
    builder.default_joint_cfg.target_kd = 120.0
    config = add_cell_scene(builder, source_root)
    px, py, pz = map(float, config["pedestal"]["translation"])
    psize = tuple(map(float, config["pedestal"]["size"]))
    robot_pose = (px, py, pz + psize[2])
    container_pose = next(item for item in config["static_transforms"] if item["child"] == "container_link")
    container_model = config["gazebo"]["container_model"]

    model = builder.finalize()
    # The payload is rigidly held by the end-effector assembly. Suppress
    # internal contacts within that one rigid tool assembly, while retaining
    # its full box collision against the arm, container, platform and ground.
    shape_bodies = model.shape_body.numpy()
    payload_body = model.body_label.index("S20/payload_box")
    tool_bodies = {model.body_label.index(f"S20/{name}") for name in (
        "suction_panel", "eef_mount_adapter", "camera_link", "mid360_mount_frame"
    )}
    payload_shapes = [i for i, body in enumerate(shape_bodies) if body == payload_body]
    tool_shapes = [i for i, body in enumerate(shape_bodies) if body in tool_bodies]
    model.shape_collision_filter_pairs.update(
        (min(a, b), max(a, b)) for a in payload_shapes for b in tool_shapes
    )
    # The full container and arm meshes can generate more contacts than
    # Newton's automatic estimate once self-collision is enabled.
    model.rigid_contact_max = 20000
    # URDF import stores joint coordinates; initialize body poses from them.
    # Without FK, every arm link starts at the world origin and interpenetrates.
    newton.eval_fk(model, model.joint_q, model.joint_qd, model)
    return model, {"robot": "S20", "container_model": container_model, "container_pose": container_pose["translation"], "robot_pose": robot_pose, "mesh_collisions": True, "robot_self_collisions": True, "payload": "gripped_cuboid", "box_size_m": BOX_SIZE, "box_mass_kg": BOX_MASS, "actuation": "prescribed_joint_fk"}


class Simulation:
    def __init__(self, source_root: Path, iterations: int, substeps: int, motion: str, pipeline: str):
        self.model, self.info = make_scene(source_root)
        self.solver = newton.solvers.SolverXPBD(self.model, iterations=iterations) if pipeline == "xpbd" else None
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.motion = motion
        self.pipeline = pipeline
        self.target_q = self.model.joint_q.numpy().copy()
        self.wrist_q_index = int(self.model.joint_q_start.numpy()[self.model.joint_label.index("S20/elfin_joint6")])
        self.wrist_q_initial = float(self.target_q[self.wrist_q_index])
        self.control.joint_target_q.assign(self.target_q)
        self.contacts = self.model.contacts()
        self.substeps = substeps
        self.time = 0.0

    def step(self):
        if self.motion == "rotate":
            ramp = min(1.0, self.time / 0.5)
            self.target_q[self.wrist_q_index] = self.wrist_q_initial + ramp * WRIST_AMPLITUDE * math.sin(2 * math.pi * self.time / WRIST_PERIOD)
            self.control.joint_target_q.assign(self.target_q)
            # Prescribe the joint path with FK before each contact/physics step.
            # XPBD's motor target alone does not advance this imported arm in
            # Newton 1.3; the collision meshes still participate in each step.
            self.state_0.joint_q.assign(self.target_q)
            newton.eval_fk(self.model, self.state_0.joint_q, self.state_0.joint_qd, self.state_0)
        if self.pipeline == "contact":
            self.model.collide(self.state_0, self.contacts)
        else:
            for _ in range(self.substeps):
                self.state_0.clear_forces()
                self.model.collide(self.state_0, self.contacts)
                self.solver.step(self.state_0, self.state_1, self.control, self.contacts, FRAME_DT / self.substeps)
                self.state_0, self.state_1 = self.state_1, self.state_0
        self.time += FRAME_DT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--mode", choices=("inspect", "bench", "show"), default="show")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--frames", type=int, default=0, help="0 runs viewer until closed")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--motion", choices=("idle", "rotate"), default="rotate")
    parser.add_argument("--pipeline", choices=("contact", "xpbd"), default="xpbd")
    args = parser.parse_args()
    wp.set_device(args.device)
    start = time.perf_counter()
    sim = Simulation(args.source_root, args.iterations, args.substeps, args.motion, args.pipeline)
    wp.synchronize()
    build_seconds = time.perf_counter() - start
    print(json.dumps({**sim.info, "motion": args.motion, "pipeline": args.pipeline, "body_count": sim.model.body_count, "shape_count": sim.model.shape_count, "build_seconds": round(build_seconds, 3)}, ensure_ascii=False))
    if args.mode == "inspect":
        return
    if args.mode == "bench":
        if args.frames < 1:
            parser.error("--mode bench requires --frames > 0")
        initial_q = sim.state_0.body_q.numpy().copy()
        box_body_index = sim.model.body_label.index("S20/payload_box")
        for _ in range(args.warmup):
            sim.step()
        wp.synchronize()
        samples = []
        for _ in range(args.frames):
            t = time.perf_counter()
            sim.step()
            wp.synchronize()
            samples.append(1000 * (time.perf_counter() - t))
        final_q = sim.state_0.body_q.numpy()
        finite = bool(np.isfinite(final_q).all())
        max_translation_m = float(np.max(np.linalg.norm(final_q[:, :3] - initial_q[:, :3], axis=1)))
        box_quat_dot = float(np.dot(initial_q[box_body_index, 3:], final_q[box_body_index, 3:]))
        box_rotation_rad = 2 * math.acos(np.clip(abs(box_quat_dot), 0.0, 1.0)) if finite else float("nan")
        joint6_actual = float(sim.state_0.joint_q.numpy()[sim.wrist_q_index])
        print(json.dumps({"frames": args.frames, "mean_ms": round(statistics.mean(samples), 3), "median_ms": round(statistics.median(samples), 3), "p95_ms": round(sorted(samples)[int(0.95 * (len(samples)-1))], 3), "finite_body_poses": finite, "max_body_translation_m": round(max_translation_m, 5), "box_rotation_rad": round(box_rotation_rad, 4), "joint6_target_rad": round(float(sim.target_q[sim.wrist_q_index]), 4), "joint6_actual_rad": round(joint6_actual, 4)}, ensure_ascii=False))
        if not finite:
            raise RuntimeError("Non-finite body pose after simulation")
        return

    import newton.viewer

    grid_starts, grid_ends, grid_colors = make_ground_grid(z=0.003)
    viewer = newton.viewer.ViewerGL()
    viewer.set_model(sim.model)
    viewer.set_camera(pos=wp.vec3(3.7, -5.8, 3.2), pitch=-12.0, yaw=120.0)
    frames = 0
    try:
        while viewer.is_running():
            if viewer.should_step():
                sim.step()
                frames += 1
            viewer.begin_frame(sim.time)
            viewer.log_state(sim.state_0)
            viewer.log_lines("ground_grid", grid_starts, grid_ends, grid_colors)
            viewer.end_frame()
            if args.frames and frames >= args.frames:
                break
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
