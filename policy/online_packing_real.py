"""Online cuboid packing in the airport_container_real cargo hull.

Coordinates are local to the cargo space: x=0 at the door, y=0 at the
negative-Y inner wall, z=0 at the flat cargo floor. The planner sees only the
current box. Every accepted placement has a checked, axis-aligned doorway path.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from online_packing_demo import Box, EPS, collides, coordinates, fully_supported, overlap, random_sizes, top_free

DEFAULT_SOURCE = Path(__file__).resolve().parent / "vendor" / "roboticarm"
SETTLE_CLEARANCE_M = 0.02


def scene_config_path(root: Path) -> Path:
    return root / "elfin_humble_ws" / "src" / "luggage_description" / "config" / "scene_tf.yaml"


def config_label(config_path: Path, source_root: Path) -> str:
    """Store a portable label for the config used by a generated sample."""
    try:
        return config_path.resolve().relative_to(source_root.resolve()).as_posix()
    except ValueError:
        return config_path.as_posix()


@dataclass(frozen=True)
class Hull:
    length: float
    width: float
    height: float
    flat_y: float
    slope_rise: float
    door_y0: float
    door_y1: float
    door_z0: float
    door_z1: float
    floor_z: float
    door_x: float
    inner_y0: float

    @classmethod
    def from_config(cls, config: dict):
        data = config["container"]
        inner = data["inner"]
        aperture = data["opening"]["aperture"]["corners"]
        length, width = float(inner["length"]), float(inner["width"])
        floor_z = float(inner["floor_z"])
        return cls(length, width, float(inner["ceiling_z"]) - floor_z,
                   float(inner["chamfer"]["floor_y"]) + width / 2,
                   float(inner["chamfer"]["wall_z"]) - floor_z,
                   min(float(p[1]) for p in aperture) + width / 2,
                   max(float(p[1]) for p in aperture) + width / 2,
                   min(float(p[2]) for p in aperture) - floor_z,
                   max(float(p[2]) for p in aperture) - floor_z,
                   floor_z, min(float(p[0]) for p in aperture), -width / 2)

    def as_dict(self):
        return vars(self)

    @classmethod
    def from_dict(cls, data):
        return cls(**data)

    @property
    def volume(self):
        return self.length * (self.width * self.height -
                              (self.width - self.flat_y) * self.slope_rise / 2)

    def floor_at(self, y_right):
        if y_right <= self.flat_y:
            return 0.0
        return self.slope_rise * (y_right - self.flat_y) / (self.width - self.flat_y)

    def fits(self, box: Box):
        x, y, z = box.pos
        lx, ly, lz = box.dims
        return (x >= -EPS and x + lx <= self.length + EPS and
                y >= -EPS and y + ly <= self.width + EPS and
                z + EPS >= self.floor_at(y + ly) and z + lz <= self.height + EPS)

    def floor_supports(self, box: Box):
        return abs(box.pos[2]) < EPS and box.pos[1] + box.dims[1] <= self.flat_y + EPS

    def door_fits(self, y, z, ly, lz):
        return (y >= self.door_y0 - EPS and y + ly <= self.door_y1 + EPS and
                z >= self.door_z0 - EPS and z + lz <= self.door_z1 + EPS)


def path_clear(dims, points, placed, hull: Hull):
    """Check each translation's swept AABB, hull contour and door crossing."""
    lx, ly, lz = dims
    for a, b in zip(points, points[1:]):
        moving_axes = sum(abs(a[i] - b[i]) > EPS for i in range(3))
        if moving_axes != 1:
            return False
        low = tuple(min(a[i], b[i]) for i in range(3))
        high = tuple(max(a[i], b[i]) + dims[i] for i in range(3))
        if high[0] > EPS:
            if high[0] > hull.length + EPS or low[1] < -EPS or high[1] > hull.width + EPS:
                return False
            if low[2] + EPS < hull.floor_at(high[1]) or high[2] > hull.height + EPS:
                return False
        crosses_door = (
            abs(a[0] - b[0]) > EPS and
            min(a[0], b[0]) < -EPS <= max(a[0], b[0])
        )
        if crosses_door and not hull.door_fits(low[1], low[2], ly, lz):
            return False
        if any(all(overlap(low[k], high[k], box.pos[k], box.pos[k] + box.dims[k])
                   for k in range(3)) for box in placed):
            return False
    return True


def route_to(box: Box, placed: list[Box], hull: Hull, *, min_entry_z: float = 0.0):
    lx, ly, lz = box.dims
    x, y, z = box.pos
    if not hull.fits(box) or ly > hull.door_y1 - hull.door_y0 + EPS:
        return None
    # Leave a small vertical settling gap before the final drop (and, when
    # reversed for removal, before lifting clear of the neighboring boxes).
    settle_z = z + SETTLE_CLEARANCE_M
    if settle_z + lz > hull.height + EPS:
        return None
    if y < hull.door_y0 - EPS or y + ly > hull.door_y1 + EPS:
        if x < 0.04 - EPS:  # leave room behind the door frame for a side shift
            return None
    entry_ys = {min(max(y, hull.door_y0), hull.door_y1 - ly),
                hull.door_y0, hull.door_y1 - ly}
    for obstacle in placed:
        entry_ys.update((obstacle.pos[1] + obstacle.dims[1], obstacle.pos[1] - ly))
    entry_ys = sorted((v for v in entry_ys if hull.door_y0 - EPS <= v <= hull.door_y1 - ly + EPS),
                      key=lambda v: abs(v - y))
    entry_zs = {max(z, hull.door_z0), hull.door_z0, settle_z, min_entry_z}
    entry_zs.update(max(hull.door_z0, b.pos[2] + b.dims[2]) for b in placed)
    entry_zs = sorted(
        (v for v in entry_zs
         if v >= hull.door_z0 - EPS and v >= settle_z - EPS and v >= min_entry_z - EPS
         and v + lz <= hull.door_z1 + EPS),
        key=lambda v: abs(v - settle_z),
    )
    outside_x = -lx - 0.10
    for entry_z in entry_zs:
        for entry_y in entry_ys:
            # Keep the box at a safe height while aligning x/y, then make one
            # vertical settling move. entry_z is constrained to be at least
            # the requested clearance above the final lower-left z.
            waypoints = [(outside_x, entry_y, entry_z),
                         (x, entry_y, entry_z)]
            if abs(entry_y - y) > EPS:
                waypoints.append((x, y, entry_z))
            if abs(entry_z - z) > EPS:
                waypoints.append((x, y, z))
            if path_clear(box.dims, waypoints, placed, hull):
                return [tuple(round(v, 4) for v in point) for point in waypoints]
    return None


def feasible_route(box: Box, placed: list[Box], hull: Hull, *,
                   ik_checker=None, source_dims=None, reverse=False,
                   min_entry_z=0.0):
    """Choose the first route whose sampled poses are IK-feasible.

    The minimum settling gap remains 2 cm.  If a low horizontal segment is
    outside the robot's reachable workspace, retry it at progressively higher
    heights before rejecting the placement.
    """
    route = route_to(box, placed, hull, min_entry_z=min_entry_z)
    if ik_checker is None:
        return route
    if source_dims is None:
        raise ValueError("source_dims is required with IK filtering")
    yaw = ik_checker.box_yaw(source_dims, box.dims)
    max_entry_z = hull.height - box.dims[2]
    heights = sorted({
        # The deploy arm's low doorway posture is numerically unstable near
        # the sill. Keep the horizontal transport segment above this height;
        # the final vertical drop still ends at the planned z coordinate.
        0.4,
        0.5,
        0.6,
        round(max_entry_z, 3),
    })
    for candidate_min_entry_z in heights:
        candidate_min_entry_z = max(min_entry_z, candidate_min_entry_z)
        if candidate_min_entry_z > max_entry_z + EPS:
            continue
        route = route_to(box, placed, hull, min_entry_z=candidate_min_entry_z)
        if route is None or not ik_checker.route_is_feasible(route, box.dims, yaw, hull):
            continue
        if reverse and not ik_checker.route_is_feasible(
                list(reversed(route)), box.dims, yaw, hull):
            continue
        return route
    return None


def candidate_positions(
    dims,
    placed,
    hull: Hull,
    *,
    ik_checker=None,
    source_dims=None,
    ik_x_step=0.15,
    first_only=False,
    candidate_limit=None,
    feasible_candidate_limit=None,
    rank_by_depth=False,
    max_face_down=False,
    min_entry_z=0.0,
):
    candidates = []
    for size in sorted(set(itertools.permutations(dims))):
        lx, ly, lz = size
        if max_face_down and abs(lz - min(float(value) for value in dims)) > EPS:
            continue
        if ik_checker is not None:
            if source_dims is None:
                raise ValueError("source_dims is required with IK filtering")
            # Keep the largest rectangular face horizontal: its normal is the
            # shortest box dimension. Only yaw permutations are allowed, so a
            # wheel side can never become the bottom or the top face.
            min_dim = min(float(v) for v in source_dims)
            if abs(lz - min_dim) > EPS:
                continue
            try:
                candidate_yaw = ik_checker.box_yaw(source_dims, size)
            except ValueError:
                continue
        else:
            candidate_yaw = 0.0
        if lx > hull.length or ly > hull.width or lz > hull.height:
            continue
        xs = coordinates(hull.length, lx, placed, 0)
        if ik_checker is not None:
            # The coarse lattice is clipped analytically by the S20 reach
            # sphere below, before placement objects or IK routes are created.
            if ik_x_step <= 0.0:
                raise ValueError("ik_x_step must be positive")
            xs = sorted(set(xs) | {
                round(ik_x_step * index, 3)
                for index in range(int((hull.length - lx) / ik_x_step) + 1)
            })
        ys = coordinates(hull.width, ly, placed, 1)
        ys = sorted(set(ys) | {round(hull.flat_y - ly, 3), round(hull.door_y0, 3),
                               round(hull.door_y1 - ly, 3)})
        zs = sorted({0.0} | {round(b.pos[2] + b.dims[2], 3) for b in placed})
        for z in zs:
            if z + lz > hull.height + EPS:
                continue
            for y in ys:
                candidate_xs = xs
                if ik_checker is not None:
                    interval = ik_checker.reachable_x_interval(
                        size, y, z, hull
                    )
                    if interval is None or interval[0] > interval[1] + EPS:
                        continue
                    x_min, x_max = interval
                    candidate_xs = [x for x in xs
                                    if x_min - EPS <= x <= x_max + EPS]
                    # Include the exact farthest endpoint admitted by the
                    # sphere, even when it falls between lattice points.
                    candidate_xs.append(round(x_max, 3))
                    candidate_xs = sorted(set(candidate_xs))
                for x in candidate_xs:
                    candidate = Box("new", size, (x, y, z))
                    if not hull.fits(candidate):
                        continue
                    if any(collides(candidate, b) for b in placed):
                        continue
                    if not (hull.floor_supports(candidate) or fully_supported(candidate, placed)):
                        continue
                    candidates.append(candidate)

    # IK is the expensive part. Test the most useful geometric placements
    # first, so an online direct placement can stop as soon as one is proven.
    if rank_by_depth and not placed:
        # Empty-scene bootstrap: do not spend dozens of full path checks
        # searching for a deeper optimum. Try the deepest candidate first;
        # the first reachable, collision-free candidate is a valid greedy seed.
        candidates.sort(key=lambda candidate: (
            -depth_value(candidate, hull), candidate.pos[2],
            candidate.pos[1], -candidate.pos[0],
        ))
    elif rank_by_depth:
        candidates.sort(
            key=lambda candidate: (
                depth_value(candidate, hull),
                candidate.pos[2],
                -candidate.pos[1],
            ),
            reverse=True,
        )
    else:
        candidates.sort(
            key=lambda candidate: (
                insertion_priority(candidate, hull),
                score(candidate, placed, hull),
            ),
            reverse=True,
        )
    if candidate_limit is not None:
        candidates = candidates[:max(0, int(candidate_limit))]
    feasible_count = 0
    for candidate in candidates:
        route = feasible_route(
            candidate,
            placed,
            hull,
            ik_checker=ik_checker,
            source_dims=source_dims,
            min_entry_z=min_entry_z,
        )
        if route is not None:
            yield candidate, route
            feasible_count += 1
            if first_only:
                return
            if (feasible_candidate_limit is not None and
                    feasible_count >= max(0, int(feasible_candidate_limit))):
                return


def score(box: Box, placed: list[Box], hull: Hull):
    x, y, z = box.pos
    lx, ly, _ = box.dims
    # Reward deep, low, compact placements and keep the door area accessible.
    contact = sum(abs(a - b) < 0.002 for a, b in
                  ((x + lx, hull.length), (y, 0), (y + ly, hull.width), (z, 0)))
    for other in placed:
        for axis in (0, 1):
            if abs(box.pos[axis] + box.dims[axis] - other.pos[axis]) < 0.002 or abs(other.pos[axis] + other.dims[axis] - box.pos[axis]) < 0.002:
                contact += 1
    return 5 * (x + lx) / hull.length - 2.5 * z / hull.height - 0.3 * y / hull.width + 0.22 * contact


def depth_value(box: Box, hull: Hull) -> float:
    """User-requested objective contribution: normalized center depth × volume."""
    center_depth = (box.pos[0] + box.dims[0] / 2.0) / hull.length
    volume = box.dims[0] * box.dims[1] * box.dims[2]
    return center_depth * volume


def packing_value(boxes, hull: Hull) -> float:
    return sum(depth_value(box, hull) for box in boxes)


def insertion_priority(box: Box, hull: Hull):
    """Prefer the back wall first so the stream is packed from inside out."""
    return (round((box.pos[0] + box.dims[0]) / hull.length, 6),
            -round(box.pos[2], 6),
            -round(box.pos[1], 6))


def best_direct(
    box_id,
    dims,
    placed,
    hull: Hull,
    *,
    ik_checker=None,
    source_dims=None,
    ik_x_step=0.15,
):
    best = None
    best_key = None
    for candidate, route in candidate_positions(
            dims, placed, hull, ik_checker=ik_checker, source_dims=source_dims,
            ik_x_step=ik_x_step, first_only=ik_checker is not None):
        key = (*insertion_priority(candidate, hull), score(candidate, placed, hull))
        if best_key is None or key > best_key:
            best_key = key
            best = (Box(box_id, candidate.dims, candidate.pos), route)
    return best


def best_direct_baseline(
    box_id,
    dims,
    placed,
    hull: Hull,
    *,
    candidate_limit=None,
    ik_checker=None,
    source_dims=None,
):
    """Greedy baseline: maximize depth-volume value with simple tie-breaks."""
    best = None
    for candidate, route in candidate_positions(
            dims, placed, hull, ik_checker=ik_checker,
            source_dims=source_dims or dims,
            candidate_limit=candidate_limit,
            rank_by_depth=True,
            first_only=True,
            max_face_down=True,
            min_entry_z=0.4):
        best = (Box(box_id, candidate.dims, candidate.pos), route)
        break
    return best


def frozen_by_front_column(box: Box, placed: list[Box]) -> bool:
    """Do not disturb a rear box once its insertion lane has been filled in front."""
    for other in placed:
        if other.id == box.id:
            continue
        in_front = other.pos[0] + other.dims[0] <= box.pos[0] + EPS
        overlaps_lane = (
            overlap(box.pos[1], box.pos[1] + box.dims[1],
                    other.pos[1], other.pos[1] + other.dims[1])
            and overlap(box.pos[2], box.pos[2] + box.dims[2],
                        other.pos[2], other.pos[2] + other.dims[2])
        )
        if in_front and overlaps_lane:
            return True
    return False


def largest_free_floor_rectangle(placed: list[Box], hull: Hull) -> float:
    """Area of the largest empty axis-aligned rectangle on the flat floor."""
    floor_boxes = [box for box in placed if abs(box.pos[2]) <= EPS]
    xs = sorted({0.0, hull.length} | {
        min(max(value, 0.0), hull.length)
        for box in floor_boxes for value in (box.pos[0], box.pos[0] + box.dims[0])
    })
    ys = sorted({0.0, hull.flat_y} | {
        min(max(value, 0.0), hull.flat_y)
        for box in floor_boxes for value in (box.pos[1], box.pos[1] + box.dims[1])
    })
    best = 0.0
    for xi, x0 in enumerate(xs[:-1]):
        for x1 in xs[xi + 1:]:
            for yi, y0 in enumerate(ys[:-1]):
                for y1 in ys[yi + 1:]:
                    if any(
                        overlap(x0, x1, box.pos[0], box.pos[0] + box.dims[0])
                        and overlap(y0, y1, box.pos[1], box.pos[1] + box.dims[1])
                        for box in floor_boxes
                    ):
                        continue
                    best = max(best, (x1 - x0) * (y1 - y0))
    return best


def action(kind, box: Box, route):
    return {"kind": kind, "box": box.as_dict(), "route": route}


def plan_one(
    box_id,
    dims,
    placed,
    hull: Hull,
    *,
    ik_checker=None,
    source_dims=None,
    ik_x_step=0.15,
):
    direct = best_direct(
        box_id,
        dims,
        placed,
        hull,
        ik_checker=ik_checker,
        source_dims=source_dims,
        ik_x_step=ik_x_step,
    )
    if direct:
        box, route = direct
        return [action("place", box, route)], placed + [box]
    solutions = []
    for moved in placed:
        if not top_free(moved, placed):
            continue
        base = [b for b in placed if b.id != moved.id]
        exit_route = feasible_route(
            moved, base, hull, ik_checker=ik_checker,
            source_dims=tuple(moved.dims), reverse=True)
        if exit_route is None:
            continue
        for candidate, entry_route in candidate_positions(
                dims, base, hull, ik_checker=ik_checker, source_dims=source_dims,
                ik_x_step=ik_x_step):
            current = Box(box_id, candidate.dims, candidate.pos)
            restored = best_direct(
                moved.id, moved.dims, base + [current], hull,
                ik_checker=ik_checker, source_dims=tuple(moved.dims),
                ik_x_step=ik_x_step)
            if restored is None:
                continue
            restored_box, return_route = restored
            value = score(current, base, hull) + score(restored_box, base + [current], hull)
            priority = insertion_priority(current, hull)
            solutions.append((priority, value, moved, exit_route, current, entry_route,
                              restored_box, return_route, base))
    if not solutions:
        return [], placed
    _, _, moved, exit_route, current, entry_route, restored, return_route, base = max(
        solutions, key=lambda row: (row[0], row[1]))
    return [action("remove", moved, list(reversed(exit_route))),
            action("place", current, entry_route),
            action("reinsert", restored, return_route)], base + [current, restored]


def plan_one_baseline(
    box_id,
    dims,
    placed,
    hull: Hull,
    *,
    max_depth=2,
    max_distance_m=4.0,
    beam_width=4,
    ik_checker=None,
):
    """One-box-rearrangement baseline optimized by total depth-volume value.

    It first checks every direct legal placement. If relocating one top-free
    box can improve the objective, it evaluates the atomic remove/place/
    reinsert transaction and commits the best result. Relocation is only a
    one-box Hanoi-like operation in this baseline.
    """
    direct = best_direct_baseline(
        box_id, dims, placed, hull, ik_checker=ik_checker, source_dims=dims)
    best_plan = None
    best_state = None
    best_key = None
    if direct:
        box, route = direct
        best_state = placed + [box]
        best_key = (packing_value(best_state, hull), 0, 0.0)
        best_plan = [action("place", box, route)]
        # A legal direct placement is the online baseline action. Rearrangement
        # search is only a recovery path when this box has no direct placement;
        # exploring relocations after success caused the common early boxes to
        # pay for a large search that could not improve feasibility.
        return best_plan, best_state

    beam_width = 2
    destinations_per_box = 3
    frontier = [(list(placed), [], 0.0, frozenset())]
    seen = set()
    initial_free_area = largest_free_floor_rectangle(placed, hull)

    for _depth in range(1, max_depth + 1):
        children = []
        for state, prior_actions, distance_used, moved_ids in frontier:
            for moved in state:
                if (moved.id in moved_ids or not top_free(moved, state)
                        or frozen_by_front_column(moved, state)):
                    continue
                base = [box for box in state if box.id != moved.id]
                exit_route = feasible_route(
                    moved, base, hull, ik_checker=ik_checker,
                    source_dims=tuple(moved.dims), reverse=True,
                    min_entry_z=0.4)
                if exit_route is None:
                    continue
                remove_distance = route_length(exit_route)
                for destination, entry_route in candidate_positions(
                        moved.dims, base, hull,
                        candidate_limit=destinations_per_box,
                        rank_by_depth=True,
                        max_face_down=True,
                        ik_checker=ik_checker,
                        source_dims=tuple(moved.dims),
                        min_entry_z=0.4):
                    if destination.pos == moved.pos and destination.dims == moved.dims:
                        continue
                    move_distance = remove_distance + route_length(entry_route)
                    total_distance = distance_used + move_distance
                    if total_distance > max_distance_m + EPS:
                        continue
                    relocated = Box(moved.id, destination.dims, destination.pos)
                    next_state = base + [relocated]
                    # Hanoi-like move: only keep a move if it opens a larger
                    # contiguous usable floor patch for the incoming stream.
                    if largest_free_floor_rectangle(next_state, hull) <= initial_free_area + 1e-4:
                        continue
                    state_key = tuple(sorted(
                        (b.id, tuple(round(v, 3) for v in b.dims),
                         tuple(round(v, 3) for v in b.pos)) for b in next_state))
                    if state_key in seen:
                        continue
                    seen.add(state_key)
                    move_actions = prior_actions + [
                        action("remove", moved, list(reversed(exit_route))),
                        action("reinsert", relocated, entry_route),
                    ]
                    move_actions[-1]["relocation_distance_m"] = round(move_distance, 4)

                    incoming = best_direct_baseline(
                        box_id, dims, next_state, hull,
                        ik_checker=ik_checker, source_dims=dims)
                    if incoming is not None:
                        current, current_route = incoming
                        final_state = next_state + [current]
                        key = (packing_value(final_state, hull), -_depth, -total_distance)
                        if best_key is None or key > best_key:
                            best_key = key
                            best_state = final_state
                            best_plan = move_actions + [action("place", current, current_route)]
                    children.append((next_state, move_actions, total_distance,
                                     moved_ids | {moved.id}))
        children.sort(
            key=lambda node: (packing_value(node[0], hull), -node[2]),
            reverse=True,
        )
        frontier = children[:beam_width]
        if not frontier:
            break
    return (best_plan, best_state) if best_plan else ([], placed)


def route_length(route):
    return sum(
        math.dist(first, second)
        for first, second in zip(route, route[1:])
    )


class OnlinePackingPolicy:
    """Stateful online policy that plans one observed box at a time.

    The policy never receives a list of future boxes.  Its complete state is
    the already accepted placement list and the next box id, so it can be
    connected directly to a detector, conveyor, or simulator event stream.
    """

    def __init__(
        self,
        hull: Hull,
        *,
        ik_checker=None,
        ik_x_step=0.15,
        policy="heuristic",
        relocation_depth=2,
        relocation_distance_budget_m=4.0,
        seed=None,
        progress=True,
    ):
        self.hull = hull
        self.ik_checker = ik_checker
        self.ik_x_step = float(ik_x_step)
        if policy not in ("heuristic", "baseline"):
            raise ValueError("policy must be 'heuristic' or 'baseline'")
        self.policy = policy
        self.relocation_depth = max(0, int(relocation_depth))
        self.relocation_distance_budget_m = float(relocation_distance_budget_m)
        if self.relocation_distance_budget_m < 0.0:
            raise ValueError("relocation distance budget must be nonnegative")
        self.seed = seed
        self.progress = progress
        self.placed: list[Box] = []
        self.next_id = 1
        self.relocations = 0

    def step(self, dims):
        """Plan and commit exactly one currently observed box.

        ``dims`` is the only new box information consumed by this call.  The
        returned record is self-contained and can be serialized immediately.
        A blocked step leaves the committed state unchanged.
        """
        size = tuple(round(float(value), 6) for value in dims)
        if len(size) != 3 or any(value <= 0.0 for value in size):
            raise ValueError("dims must contain three positive lengths")
        box_id = f"B{self.next_id}"
        started = time.perf_counter()
        if self.progress:
            seed_label = f" seed={self.seed}" if self.seed is not None else ""
            print(
                f"[planner]{seed_label} box={self.next_id} dims={size} planning...",
                file=sys.stderr,
                flush=True,
            )
        if self.policy == "baseline":
            actions, next_placed = plan_one_baseline(
                box_id,
                size,
                self.placed,
                self.hull,
                max_depth=self.relocation_depth,
                max_distance_m=self.relocation_distance_budget_m,
                ik_checker=self.ik_checker,
            )
        else:
            actions, next_placed = plan_one(
                box_id,
                size,
                self.placed,
                self.hull,
                ik_checker=self.ik_checker,
                source_dims=size,
                ik_x_step=self.ik_x_step,
            )
        elapsed = time.perf_counter() - started
        blocked = not actions
        if not blocked:
            self.placed = next_placed
            self.relocations += int(any(a["kind"] == "remove" for a in actions))
        record = {
            "incoming": {"id": box_id, "dims": size},
            "actions": actions,
            "status": "blocked" if blocked else "placed",
            "boxes": [b.as_dict() for b in self.placed],
            "utilization": round(
                sum(b.dims[0] * b.dims[1] * b.dims[2] for b in self.placed)
                / self.hull.volume,
                4,
            ),
            "objective": round(packing_value(self.placed, self.hull), 6),
            "relocation_distance_m": round(sum(
                float(a.get("relocation_distance_m", 0.0))
                for a in actions if a["kind"] == "reinsert"
            ), 4),
            "planning_seconds": round(elapsed, 6),
        }
        if self.progress:
            print(
                f"[planner]{seed_label} box={self.next_id} status={record['status']} "
                f"elapsed={elapsed:.3f}s placed={len(self.placed)}",
                file=sys.stderr,
                flush=True,
            )
        self.next_id += 1
        return record


def run_stream(
    seed,
    sizes,
    hull: Hull,
    *,
    ik_checker=None,
    count=None,
    ik_x_step=0.15,
    policy="heuristic",
    relocation_depth=2,
    relocation_distance_budget_m=4.0,
    progress=True,
):
    """Consume an iterable incrementally; future sizes are never inspected."""
    policy = OnlinePackingPolicy(
        hull,
        ik_checker=ik_checker,
        ik_x_step=ik_x_step,
        policy=policy,
        relocation_depth=relocation_depth,
        relocation_distance_budget_m=relocation_distance_budget_m,
        seed=seed,
        progress=progress,
    )
    steps = []
    for size in itertools.islice(sizes, count) if count is not None else sizes:
        step = policy.step(size)
        steps.append(step)
        if step["status"] == "blocked":
            break
    return {
        "seed": seed,
        "container": hull.as_dict(),
        "steps": steps,
        "relocations": policy.relocations,
        "online": True,
        "policy": policy.policy,
        "relocation_depth": policy.relocation_depth,
        "relocation_distance_budget_m": policy.relocation_distance_budget_m,
    }


def random_luggage_stream(seed):
    """Generate varied boxes from a 20--32 inch front-face diagonal.

    The quoted luggage size is sampled as the rectangle diagonal, then each
    dimension receives independent proportions/noise so the stream is not a
    set of scaled copies of one box.
    """
    rng = random.Random(seed)
    while True:
        diagonal = rng.uniform(20.0, 32.0) * 0.0254
        diagonal *= rng.uniform(0.90, 1.10)
        aspect = rng.uniform(1.15, 1.80)
        long_side = diagonal * aspect / (1.0 + aspect * aspect) ** 0.5
        short_side = diagonal / (1.0 + aspect * aspect) ** 0.5
        thickness = rng.uniform(0.20, 0.35) * rng.uniform(0.90, 1.10)
        yield (round(long_side, 3), round(short_side, 3), round(thickness, 3))


def random_luggage_sizes(seed, count):
    """Compatibility helper for callers that explicitly need a finite list."""
    return list(itertools.islice(random_luggage_stream(seed), count))


def validate_sample(sample):
    hull = Hull.from_dict(sample["container"])
    placed = []
    for step in sample["steps"]:
        staged = None
        for item in step["actions"]:
            raw = item["box"]
            box = Box(raw["id"], tuple(raw["dims"]), tuple(raw["pos"]))
            route = [tuple(point) for point in item["route"]]
            if item["kind"] == "remove":
                assert staged is None and box in placed and top_free(box, placed)
                placed.remove(box)
                assert route[0] == box.pos
                assert path_clear(box.dims, route, placed, hull)
                staged = box
            else:
                assert item["kind"] in ("place", "reinsert")
                if item["kind"] == "reinsert":
                    assert staged is not None and staged.id == box.id
                    staged = None
                assert route[-1] == box.pos
                assert hull.fits(box) and not any(collides(box, b) for b in placed)
                assert hull.floor_supports(box) or fully_supported(box, placed)
                assert path_clear(box.dims, route, placed, hull)
                placed.append(box)
        assert staged is None
        assert [b.as_dict() for b in placed] == [
            Box(raw["id"], tuple(raw["dims"]), tuple(raw["pos"])).as_dict()
            for raw in step["boxes"]
        ]
        if sample.get("policy") == "baseline":
            moved_distance = sum(
                float(item.get("relocation_distance_m", 0.0))
                for item in step["actions"] if item["kind"] == "reinsert"
            )
            assert moved_distance <= sample["relocation_distance_budget_m"] + EPS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--scene-config", type=Path, default=None,
                        help="scene_tf YAML; defaults to the vendored scene_tf.yaml")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("online_packing_real_samples.json"))
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--seeds", type=int, nargs="+", default=[1, 8, 13])
    parser.add_argument("--ik", action="store_true",
                        help="require Newton S20 IK at every sampled route waypoint")
    parser.add_argument(
        "--policy",
        choices=("heuristic", "baseline"),
        default="heuristic",
        help="geometric planning policy; baseline maximizes depth-volume value",
    )
    parser.add_argument(
        "--relocation-depth",
        type=int,
        default=2,
        help="maximum number of existing boxes relocated in one baseline search",
    )
    parser.add_argument(
        "--relocation-distance-budget",
        type=float,
        default=4.0,
        metavar="METERS",
        help="maximum summed remove/reinsert path length per incoming box",
    )
    parser.add_argument("--ik-device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument(
        "--ik-x-step",
        type=float,
        default=0.15,
        help="coarse x candidate spacing in IK mode (m); final checks stay strict",
    )
    parser.add_argument(
        "--no-progress",
        action="store_true",
        help="suppress per-box online planning timing logs",
    )
    parser.add_argument("--box-size", type=float, nargs=3, default=None,
                        metavar=("L", "W", "H"),
                        help="same box size for every incoming box")
    parser.add_argument("--luggage-20-32", action="store_true",
                        help="random suitcase-like sizes with 20--32 inch face diagonals")
    args = parser.parse_args()
    config_path = args.scene_config
    if config_path is None:
        config_path = (
            scene_config_path(args.source_root)
        )
    hull = Hull.from_config(yaml.safe_load(config_path.read_text(encoding="utf-8")))
    ik_checker = None
    if args.ik:
        from newton_ik_planner import DEFAULT_BOX_SIZE, NewtonIKWorkspace

        box_size = tuple(args.box_size or DEFAULT_BOX_SIZE)
        if any(value <= 0.0 for value in box_size):
            raise SystemExit("--box-size values must be positive")
        ik_checker = NewtonIKWorkspace(
            args.source_root,
            config_path,
            container_y_override=None,
            device=args.ik_device,
            box_size=box_size,
        )
        if args.luggage_20_32:
            sizes_by_seed = [random_luggage_stream(seed) for seed in args.seeds]
        else:
            sizes_by_seed = [itertools.repeat(box_size) for _ in args.seeds]
    else:
        sizes_by_seed = [
            (random_luggage_stream(seed) if args.luggage_20_32
             else iter(random_sizes(seed, args.count)))
            for seed in args.seeds
        ]
    samples = []
    for seed, sizes in zip(args.seeds, sizes_by_seed):
        samples.append(
            run_stream(
                seed,
                sizes,
                hull,
                ik_checker=ik_checker,
                count=args.count,
                ik_x_step=args.ik_x_step,
                policy=args.policy,
                relocation_depth=args.relocation_depth,
                relocation_distance_budget_m=args.relocation_distance_budget,
                progress=not args.no_progress,
            )
        )
    if ik_checker is not None:
        for sample in samples:
            sample["ik"] = {
                "enabled": True,
                "solver": "newton.ik.IKSolver",
                "scene_config": config_label(config_path, args.source_root),
                "box_size_m": list(ik_checker.box_size),
                "size_profile": "luggage_20_32" if args.luggage_20_32 else "uniform",
                "orientation_constraint": "largest_face_down",
                "route_sample_step_m": 0.05,
                "ik_x_candidate_step_m": args.ik_x_step,
                "settle_clearance_m": 0.02,
                "position_tolerance_m": 0.003,
                "rotation_tolerance_rad": 0.035,
                "collision_check": "non-base arm links vs container mesh",
            }
    for sample in samples:
        validate_sample(sample)
    args.output.write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps([{"seed": s["seed"], "placed": sum(v["status"] == "placed" for v in s["steps"]),
                       "relocations": s["relocations"], "utilization": s["steps"][-1]["utilization"]}
                      for s in samples], ensure_ascii=False))


if __name__ == "__main__":
    main()
