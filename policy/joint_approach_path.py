"""Bounded bidirectional RRT for a kinematic free-space joint approach.

Collision/limit ownership stays with the injected is_valid(q) callback.
Edges are sampled at <= resolution radians in every joint, never certified
as continuous collision-free trajectories.
"""
import numpy as np
import time


def nearest_revolute_solution(solution, reference, lower, upper):
    """Choose equivalent 2pi windings within limits, nearest the live joints."""
    result = np.asarray(solution, dtype=float).copy()
    for i, value in enumerate(result):
        candidates = [value + 2 * np.pi * k for k in range(-2, 3)
                      if lower[i] <= value + 2 * np.pi * k <= upper[i]]
        if not candidates:
            raise RuntimeError("IK solution outside revolute joint limits")
        result[i] = min(candidates, key=lambda candidate: abs(candidate - reference[i]))
    return result


def plan_joint_approach(start, goal, lower, upper, is_valid, *, seed=7,
                        max_nodes=1200, step=0.45, resolution=0.08,
                        max_seconds=90):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    deadline = time.monotonic() + max_seconds

    def edge(first, second):
        if time.monotonic() >= deadline:
            raise RuntimeError("joint approach search wall-clock deadline")
        count = max(1, int(np.ceil(np.max(np.abs(second - first)) / resolution)))
        return all(is_valid(first + (second - first) * alpha)
                   for alpha in np.linspace(0, 1, count + 1)[1:])

    if not is_valid(start) or not is_valid(goal):
        raise RuntimeError("joint approach start or goal is invalid")
    if edge(start, goal):
        return [start, goal]
    # Cheap deterministic candidates for a serial arm: fold the shoulder/
    # elbow before sweeping the base. Every edge still passes the same gate.
    if len(start) == 6:
        for order in ((1, 0, 2, 3, 4, 5), (1, 3, 4, 5, 0, 2),
                      (1, 4, 3, 5, 0, 2), (2, 0, 1, 3, 4, 5),
                      (1, 2, 3, 4, 5, 0), (1, 2, 0, 3, 4, 5),
                      (2, 1, 3, 4, 5, 0), (2, 1, 0, 3, 4, 5)):
            path = [start]
            for joint in order:
                point = path[-1].copy()
                point[joint] = goal[joint]
                if not edge(path[-1], point):
                    break
                path.append(point)
            else:
                return path
    rng = np.random.default_rng(seed)
    trees = [([start], [-1]), ([goal], [-1])]

    def extend(tree, target):
        points, parents = tree
        nearest = int(np.argmin([np.linalg.norm(p - target) for p in points]))
        delta = target - points[nearest]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return nearest, True
        point = points[nearest] + delta * min(1, step / distance)
        if not edge(points[nearest], point):
            return None, False
        points.append(point)
        parents.append(nearest)
        return len(points) - 1, distance <= step

    def ancestry(tree, index):
        chain = []
        while index >= 0:
            chain.append(tree[0][index])
            index = tree[1][index]
        return chain[::-1]

    for attempt in range(max_nodes):
        if sum(len(t[0]) for t in trees) >= max_nodes:
            break
        active = attempt % 2
        other = 1 - active
        sample = trees[other][0][0] if rng.random() < 0.2 else rng.uniform(lower, upper)
        index, _ = extend(trees[active], sample)
        if index is None:
            continue
        target = trees[active][0][index]
        while sum(len(t[0]) for t in trees) <= max_nodes:
            peer, reached = extend(trees[other], target)
            if peer is None:
                break
            if reached:
                first, second = ancestry(trees[active], index), ancestry(trees[other], peer)
                path = first + second[::-1][1:]
                return path if active == 0 else path[::-1]
    raise RuntimeError("joint approach search exhausted (%d nodes)" % max_nodes)
