"""Coarse Newton container simulation and repeatable timing probe.

The container's -X face is open. This scene benchmarks rigid boxes and
contacts; it does not yet model a robot or solve the packing problem.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import time
from pathlib import Path

# Warp needs a writable kernel cache. Configure it before importing Warp.
os.environ.setdefault("WARP_CACHE_PATH", str(Path(__file__).resolve().parent.parent / ".wp"))
# The environment's OpenGL_accelerate wheel is incompatible with its NumPy.
# PyOpenGL's pure-Python fallback works without changing installed packages.
os.environ.setdefault("PYOPENGL_USE_ACCELERATE", "0")

import newton  # noqa: E402
import warp as wp  # noqa: E402


SIDE = 3.2  # metres, internal container length/width/height
WALL = 0.025
GRID = 4
CELL = SIDE / GRID
FRAME_DT = 1.0 / 60.0
GRID_RADIUS = 6.0
GRID_SPACING = 0.5


def box_size(column: int, level: int) -> tuple[float, float, float]:
    """Vary dimensions between stacks while preserving full support within each."""
    return (
        0.77 + 0.005 * (column % GRID),
        0.77 + 0.005 * ((column // GRID) % GRID),
        0.76 + 0.005 * ((column + level) % GRID),
    )


def make_scene(box_count: int):
    if not 1 <= box_count <= 64:
        raise ValueError("--boxes must be in 1..64")

    builder = newton.ModelBuilder()
    builder.default_shape_cfg.mu = 0.7
    half = SIDE / 2.0
    outer = half + WALL / 2.0

    # Dark ground slab outside the container. Grid lines are drawn by ViewerGL.
    builder.add_shape_box(
        body=-1,
        xform=wp.transform(p=wp.vec3(0, 0, -0.075), q=wp.quat_identity()),
        hx=GRID_RADIUS,
        hy=GRID_RADIUS,
        hz=0.025,
        color=wp.vec3(0.15, 0.18, 0.22),
        label="world_ground",
    )

    # Static walls. The -X face is intentionally absent.
    walls = (
        ("floor", (0, 0, -WALL / 2), (outer, outer, WALL / 2)),
        ("back", (outer, 0, half), (WALL / 2, outer, half + WALL)),
        ("left", (0, -outer, half), (outer, WALL / 2, half + WALL)),
        ("right", (0, outer, half), (outer, WALL / 2, half + WALL)),
        ("ceiling", (0, 0, SIDE + WALL / 2), (outer, outer, WALL / 2)),
    )
    for name, position, half_size in walls:
        builder.add_shape_box(
            body=-1,
            xform=wp.transform(p=wp.vec3(*position), q=wp.quat_identity()),
            hx=half_size[0],
            hy=half_size[1],
            hz=half_size[2],
            color=wp.vec3(0.36, 0.43, 0.51),
            label=name,
        )

    packed_volume = 0.0
    for i in range(box_count):
        column = i % (GRID * GRID)
        level = i // (GRID * GRID)
        ix, iy = divmod(column, GRID)
        sx, sy, sz = box_size(column, level)
        x = -half + (ix + 0.5) * CELL
        y = -half + (iy + 0.5) * CELL
        bottom = sum(box_size(column, lower)[2] for lower in range(level))
        # A 2 mm numerical clearance prevents unstable contact resolution at
        # t=0. Footprints match the support below; gravity closes the gap.
        z = bottom + sz / 2.0 + 0.002 * (level + 1)

        body = builder.add_body(
            xform=wp.transform(p=wp.vec3(x, y, z), q=wp.quat_identity()),
            label=f"box_{i:02d}",
        )
        builder.add_shape_box(
            body,
            hx=sx / 2.0,
            hy=sy / 2.0,
            hz=sz / 2.0,
            color=wp.vec3(0.87 - 0.10 * (level % 3), 0.47 + 0.08 * (ix % 3), 0.18 + 0.12 * (iy % 3)),
            label=f"box_shape_{i:02d}",
        )
        packed_volume += sx * sy * sz

    return builder.finalize(), packed_volume / SIDE**3


def make_ground_grid(z: float = -0.048):
    """Line arrays for a robotics-style 0.5 m grid around the container."""
    starts, ends, colors = [], [], []
    count = round(GRID_RADIUS / GRID_SPACING)
    # Place just above the ground slab, below the container floor by default.
    for k in range(-count, count + 1):
        coordinate = k * GRID_SPACING
        if k == 0:
            x_color = (0.72, 0.30, 0.30)
            y_color = (0.30, 0.67, 0.38)
        elif k % 2 == 0:
            x_color = y_color = (0.37, 0.45, 0.53)
        else:
            x_color = y_color = (0.27, 0.34, 0.41)
        starts.extend((wp.vec3(-GRID_RADIUS, coordinate, z), wp.vec3(coordinate, -GRID_RADIUS, z)))
        ends.extend((wp.vec3(GRID_RADIUS, coordinate, z), wp.vec3(coordinate, GRID_RADIUS, z)))
        colors.extend((wp.vec3(*x_color), wp.vec3(*y_color)))
    return (
        wp.array(starts, dtype=wp.vec3),
        wp.array(ends, dtype=wp.vec3),
        wp.array(colors, dtype=wp.vec3),
    )


class Simulation:
    def __init__(self, box_count: int, iterations: int, substeps: int):
        self.model, self.utilization = make_scene(box_count)
        self.solver = newton.solvers.SolverXPBD(self.model, iterations=iterations)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.contacts()
        self.substeps = substeps
        self.time = 0.0

    def step(self):
        for _ in range(self.substeps):
            self.state_0.clear_forces()
            self.model.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, FRAME_DT / self.substeps)
            self.state_0, self.state_1 = self.state_1, self.state_0
        self.time += FRAME_DT


def benchmark(args):
    start = time.perf_counter()
    sim = Simulation(args.boxes, args.iterations, args.substeps)
    wp.synchronize()
    build_s = time.perf_counter() - start

    start = time.perf_counter()
    for _ in range(args.warmup):
        sim.step()
    wp.synchronize()
    warmup_s = time.perf_counter() - start

    samples = []
    for _ in range(args.frames):
        start = time.perf_counter()
        sim.step()
        wp.synchronize()
        samples.append((time.perf_counter() - start) * 1000)

    result = {
        "newton_version": newton.__version__,
        "device": str(wp.get_device()),
        "boxes": args.boxes,
        "container_inner_side_m": SIDE,
        "initial_volume_utilization": round(sim.utilization, 4),
        "solver": "XPBD",
        "iterations": args.iterations,
        "substeps_per_frame": args.substeps,
        "simulated_fps": 60,
        "build_seconds": round(build_s, 3),
        "warmup_seconds": round(warmup_s, 3),
        "warmup_frames": args.warmup,
        "timed_frames": args.frames,
        "frame_ms_mean": round(statistics.mean(samples), 3),
        "frame_ms_median": round(statistics.median(samples), 3),
        "frame_ms_p95": round(sorted(samples)[int(0.95 * (len(samples) - 1))], 3),
        "wall_fps": round(1000 / statistics.mean(samples), 1),
        "realtime_factor": round(1000 / (60 * statistics.mean(samples)), 2),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


def show(args):
    import newton.viewer

    sim = Simulation(args.boxes, args.iterations, args.substeps)
    grid_starts, grid_ends, grid_colors = make_ground_grid()
    viewer = newton.viewer.ViewerGL()
    viewer.set_model(sim.model)
    viewer.set_camera(pos=wp.vec3(-6.5, -6.0, 4.0), pitch=-13.0, yaw=43.0)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("bench", "show"), default="bench")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--boxes", type=int, default=64)
    parser.add_argument("--frames", type=int, default=120, help="0 means unlimited in show mode")
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--substeps", type=int, default=2)
    parser.add_argument("--iterations", type=int, default=5)
    args = parser.parse_args()
    if args.frames < 0 or args.warmup < 0 or args.substeps < 1 or args.iterations < 1:
        parser.error("frames/warmup must be nonnegative; substeps/iterations must be positive")
    if args.mode == "bench" and args.frames == 0:
        parser.error("benchmark requires frames > 0")
    wp.set_device(args.device)
    if args.mode == "bench":
        benchmark(args)
    else:
        show(args)


if __name__ == "__main__":
    main()
