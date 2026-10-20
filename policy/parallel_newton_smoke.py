"""Benchmark independent rigid-body worlds in one Newton model."""

from __future__ import annotations

import argparse
import json
import os
import statistics
import time
from pathlib import Path

os.environ.setdefault("WARP_CACHE_PATH", str(Path(__file__).resolve().parent.parent / ".wp"))

import newton
import numpy as np
import warp as wp


def build_model(env_count: int, device: str):
    template = newton.ModelBuilder()
    template.default_shape_cfg.mu = 0.6
    template.default_shape_cfg.ke = 2.0e4
    template.default_shape_cfg.kd = 2.0e2

    template.add_shape_box(
        body=-1,
        xform=wp.transform(p=wp.vec3(0.0, 0.0, -0.05), q=wp.quat_identity()),
        hx=1.0,
        hy=1.0,
        hz=0.05,
        label="ground",
    )
    box_size = 0.2
    body = template.add_body(
        xform=wp.transform(p=wp.vec3(0.0, 0.0, 0.55), q=wp.quat_identity()),
        label="falling_box",
    )
    template.add_shape_box(
        body,
        hx=box_size / 2,
        hy=box_size / 2,
        hz=box_size / 2,
        label="falling_box_shape",
    )

    builder = newton.ModelBuilder()
    builder.replicate(template, env_count)
    model = builder.finalize(device=device)
    state_0, state_1 = model.state(), model.state()
    q = state_0.body_q.numpy()
    # Different initial heights make accidental cross-world state sharing visible.
    for env_id in range(env_count):
        q[env_id, 2] += 0.02 * env_id
    state_0.body_q.assign(q)
    state_1.body_q.assign(q)
    return model, state_0, state_1, model.control(), model.contacts(), box_size


def run(args):
    wp.set_device(args.device)
    build_started = time.perf_counter()
    model, state_0, state_1, control, contacts, box_size = build_model(args.envs, args.device)
    wp.synchronize()
    build_seconds = time.perf_counter() - build_started
    solver = newton.solvers.SolverXPBD(model, iterations=args.iterations)

    def step():
        nonlocal state_0, state_1
        state_0.clear_forces()
        model.collide(state_0, contacts)
        solver.step(state_0, state_1, control, contacts, args.dt)
        state_0, state_1 = state_1, state_0

    warm_started = time.perf_counter()
    for _ in range(args.warmup):
        step()
    wp.synchronize()
    warmup_seconds = time.perf_counter() - warm_started

    timings = []
    for _ in range(args.steps):
        started = time.perf_counter()
        step()
        wp.synchronize()
        timings.append((time.perf_counter() - started) * 1000)

    final_q = state_0.body_q.numpy()
    expected_z = box_size / 2
    errors = np.abs(final_q[:, 2] - expected_z)
    result = {
        "newton_version": newton.__version__,
        "device": str(wp.get_device()),
        "parallel_worlds": args.envs,
        "dynamic_bodies": model.body_count,
        "steps": args.steps,
        "warmup_steps": args.warmup,
        "dt_seconds": args.dt,
        "build_seconds": round(build_seconds, 4),
        "warmup_seconds": round(warmup_seconds, 4),
        "step_ms_mean": round(statistics.mean(timings), 4),
        "step_ms_p95": round(sorted(timings)[int(0.95 * (len(timings) - 1))], 4),
        "final_box_z_m": np.round(final_q[:, 2], 4).tolist(),
        "max_settle_error_m": round(float(errors.max()), 5),
        "independent_worlds_settled": bool(np.all(errors < args.settle_tolerance)),
    }
    print(json.dumps(result, indent=2))
    if not result["independent_worlds_settled"]:
        raise SystemExit("Newton parallel-world smoke test did not settle all bodies")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", type=int, default=16)
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--dt", type=float, default=1 / 120)
    parser.add_argument("--settle-tolerance", type=float, default=0.025)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    args = parser.parse_args()
    if min(args.envs, args.steps, args.iterations) < 1 or args.warmup < 0:
        parser.error("envs, steps and iterations must be positive; warmup must be nonnegative")
    run(args)


if __name__ == "__main__":
    main()
