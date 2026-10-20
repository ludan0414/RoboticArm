"""Deterministic online cuboid packing examples with bounded one-box relocation.

The planner receives only one box at a time. It uses axis-aligned rotations,
full-footprint support, straight-line insertion from x=0, and an atomic
remove/place/reinsert transaction when direct placement fails.
"""

from __future__ import annotations

import argparse
import itertools
import json
import random
from dataclasses import dataclass
from pathlib import Path

EPS = 1e-7


@dataclass(frozen=True)
class Box:
    id: str
    dims: tuple[float, float, float]
    pos: tuple[float, float, float]

    def as_dict(self):
        return {"id": self.id, "dims": self.dims, "pos": self.pos}


def overlap(a0, a1, b0, b1):
    return min(a1, b1) - max(a0, b0) > EPS


def collides(a: Box, b: Box):
    return all(overlap(a.pos[k], a.pos[k] + a.dims[k], b.pos[k], b.pos[k] + b.dims[k]) for k in range(3))


def fully_supported(box: Box, placed: list[Box]):
    x, y, z = box.pos
    lx, ly, _ = box.dims
    if abs(z) < EPS:
        return True
    supports = [b for b in placed if abs(b.pos[2] + b.dims[2] - z) < EPS]
    xs = sorted({x, x + lx} | {v for b in supports for v in (max(x, b.pos[0]), min(x + lx, b.pos[0] + b.dims[0])) if x + EPS < v < x + lx - EPS})
    ys = sorted({y, y + ly} | {v for b in supports for v in (max(y, b.pos[1]), min(y + ly, b.pos[1] + b.dims[1])) if y + EPS < v < y + ly - EPS})
    for xa, xb in zip(xs, xs[1:]):
        for ya, yb in zip(ys, ys[1:]):
            cx, cy = (xa + xb) / 2, (ya + yb) / 2
            if not any(b.pos[0] - EPS <= cx <= b.pos[0] + b.dims[0] + EPS and b.pos[1] - EPS <= cy <= b.pos[1] + b.dims[1] + EPS for b in supports):
                return False
    return True


def entrance_clear(box: Box, placed: list[Box]):
    # Full box travels from the opening along +x at its final y/z.
    _, y, z = box.pos
    _, ly, lz = box.dims
    for b in placed:
        if b.pos[0] >= box.pos[0] + box.dims[0] - EPS:
            continue
        if overlap(y, y + ly, b.pos[1], b.pos[1] + b.dims[1]) and overlap(z, z + lz, b.pos[2], b.pos[2] + b.dims[2]):
            return False
    return True


def coordinates(length, size, placed, axis):
    vals = {0.0, round(length - size, 3)}
    for b in placed:
        p, d = b.pos[axis], b.dims[axis]
        vals.update((p, p + d, p - size, p + d - size))
    return sorted({round(v, 3) for v in vals if -EPS <= v <= length - size + EPS})


def candidates(dims, placed, container):
    L, W, H = container
    results = []
    for size in sorted(set(itertools.permutations(dims))):
        lx, ly, lz = size
        if lx > L + EPS or ly > W + EPS or lz > H + EPS:
            continue
        xs = coordinates(L, lx, placed, 0)
        ys = coordinates(W, ly, placed, 1)
        zs = sorted({0.0} | {round(b.pos[2] + b.dims[2], 3) for b in placed})
        for z in zs:
            if z + lz > H + EPS:
                continue
            for x in xs:
                for y in ys:
                    box = Box("new", size, (x, y, z))
                    if any(collides(box, b) for b in placed):
                        continue
                    if not fully_supported(box, placed) or not entrance_clear(box, placed):
                        continue
                    results.append(box)
    return results


def score(box: Box, placed: list[Box], container):
    x, y, z = box.pos
    lx, ly, lz = box.dims
    L, W, _ = container
    # Back-first, then low and side-flush. Contacts reward compact packing.
    touch = sum(abs(a - b) < EPS for a, b in ((x + lx, L), (y, 0), (y + ly, W), (z, 0)))
    for b in placed:
        bx, by, bz = b.pos
        blx, bly, blz = b.dims
        if abs(x - bx - blx) < EPS or abs(x + lx - bx) < EPS:
            touch += overlap(y, y + ly, by, by + bly) and overlap(z, z + lz, bz, bz + blz)
        if abs(y - by - bly) < EPS or abs(y + ly - by) < EPS:
            touch += overlap(x, x + lx, bx, bx + blx) and overlap(z, z + lz, bz, bz + blz)
    return 5 * (x + lx) / L - 3 * z - 0.4 * y / W + 0.28 * touch


def best_direct(box_id, dims, placed, container):
    options = candidates(dims, placed, container)
    if not options:
        return None
    candidate = max(options, key=lambda b: score(b, placed, container))
    return Box(box_id, candidate.dims, candidate.pos)


def top_free(box: Box, placed):
    top = box.pos[2] + box.dims[2]
    return not any(abs(other.pos[2] - top) < EPS and overlap(box.pos[0], box.pos[0] + box.dims[0], other.pos[0], other.pos[0] + other.dims[0]) and overlap(box.pos[1], box.pos[1] + box.dims[1], other.pos[1], other.pos[1] + other.dims[1]) for other in placed if other.id != box.id)


def plan_one(box_id, dims, placed, container):
    direct = best_direct(box_id, dims, placed, container)
    if direct:
        return [dict(kind="place", box=direct.as_dict())], placed + [direct]
    # One external staging slot. Search only one existing, top-free box.
    solutions = []
    for moved in placed:
        if not top_free(moved, placed):
            continue
        base = [b for b in placed if b.id != moved.id]
        if not entrance_clear(moved, base):
            continue
        for put in candidates(dims, base, container):
            current = Box(box_id, put.dims, put.pos)
            restored = best_direct(moved.id, moved.dims, base + [current], container)
            if restored:
                solutions.append((score(current, base, container) + score(restored, base + [current], container), moved, current, restored, base))
    if not solutions:
        return [], placed
    _, moved, current, restored, base = max(solutions, key=lambda row: row[0])
    actions = [dict(kind="remove", box=moved.as_dict()), dict(kind="place", box=current.as_dict()), dict(kind="reinsert", box=restored.as_dict())]
    return actions, base + [current, restored]


def run_stream(seed, sizes, container):
    placed = []
    steps = []
    for i, size in enumerate(sizes, 1):
        dims = tuple(size)
        actions, next_state = plan_one(f"B{i}", dims, placed, container)
        status = "placed" if actions else "blocked"
        placed = next_state
        steps.append({"incoming": {"id": f"B{i}", "dims": dims}, "actions": actions, "status": status, "boxes": [b.as_dict() for b in placed], "utilization": round(sum(b.dims[0] * b.dims[1] * b.dims[2] for b in placed) / (container[0] * container[1] * container[2]), 4)})
        if not actions:
            break
    return {"seed": seed, "container": container, "steps": steps, "relocations": sum(any(a["kind"] == "remove" for a in step["actions"]) for step in steps)}


def validate_sample(sample):
    """Replay every action and verify all geometry and the staging invariant."""
    placed = []
    for step in sample["steps"]:
        staged = None
        for action in step["actions"]:
            raw = action["box"]
            box = Box(raw["id"], tuple(raw["dims"]), tuple(raw["pos"]))
            if action["kind"] == "remove":
                assert staged is None
                assert box in placed and top_free(box, placed)
                placed.remove(box)
                assert entrance_clear(box, placed)
                staged = box
                continue
            assert action["kind"] in ("place", "reinsert")
            if action["kind"] == "reinsert":
                assert staged is not None and staged.id == box.id
                staged = None
            assert all(-EPS <= box.pos[k] and box.pos[k] + box.dims[k] <= sample["container"][k] + EPS for k in range(3))
            assert not any(collides(box, other) for other in placed)
            assert fully_supported(box, placed)
            assert entrance_clear(box, placed)
            placed.append(box)
        assert staged is None, "A box remained outside when the next box arrived"
        assert [b.as_dict() for b in placed] == [
            Box(raw["id"], tuple(raw["dims"]), tuple(raw["pos"])).as_dict()
            for raw in step["boxes"]
        ]


def random_sizes(seed, count):
    rng = random.Random(seed)
    return [tuple(round(rng.uniform(a, b), 2) for a, b in ((0.34, 0.78), (0.3, 0.65), (0.24, 0.58))) for _ in range(count)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("online_packing_samples.json"))
    args = parser.parse_args()
    container = (1.5, 1.2, 0.95)
    samples = [run_stream(seed, random_sizes(seed, 16), container) for seed in (1, 8, 13)]
    for sample in samples:
        validate_sample(sample)
    args.output.write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps([{"seed": s["seed"], "steps": len(s["steps"]), "placed": sum(v["status"] == "placed" for v in s["steps"]), "relocations": s["relocations"], "utilization": s["steps"][-1]["utilization"]} for s in samples], ensure_ascii=False))


if __name__ == "__main__":
    main()
