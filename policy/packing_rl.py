"""Train a small full-information box-and-placement policy with REINFORCE.

A fast geometric candidate generator enforces non-overlap, full support, flat
orientations, and insertion paths; placed boxes are immutable. Newton
vectorized physics is tested separately by parallel_newton_smoke.py and is not
currently part of the policy's training reward.
"""

from __future__ import annotations

import argparse
import json
import os
import random
from pathlib import Path

os.environ.setdefault("WARP_CACHE_PATH", str(Path(__file__).resolve().parent.parent / ".wp"))

import numpy as np
import torch
from torch import nn

from online_packing_demo import Box, EPS, collides, fully_supported
from online_packing_real import Hull, random_luggage_sizes, route_to, validate_sample


ACTION_FEATURE_COUNT = 14
BOX_FEATURE_COUNT = 3
PLACED_FEATURE_COUNT = 6
SET_EMBED = 16


def valid_flat_poses(dims):
    shortest = min(range(3), key=lambda i: dims[i])
    top_axis = [axis for axis in range(3) if axis != shortest]
    first = tuple(float(dims[i]) for i in top_axis) + (float(dims[shortest]),)
    second = (first[1], first[0], first[2])
    return tuple(dict.fromkeys((first, second)))


def placement_candidates(dims, placed, hull: Hull, limit=12, check_route=True):
    """Fast extreme-point search; returns validated no-relocation actions."""
    options = []
    for pose in valid_flat_poses(dims):
        lx, ly, lz = pose
        if lx > hull.length + EPS or ly > hull.width + EPS or lz > hull.height + EPS:
            continue
        xs = {0.0, round(hull.length - lx, 3)}
        ys = {round(hull.door_y0, 3), round(hull.door_y1 - ly, 3),
              round(hull.flat_y - ly, 3)}
        zs = {0.0}
        for box in placed:
            x, y, z = box.pos
            bx, by, bz = box.dims
            xs.update((round(x + bx, 3), round(x - lx, 3)))
            ys.update((round(y + by, 3), round(y - ly, 3)))
            zs.add(round(z + bz, 3))
        for z in sorted(zs):
            if z + lz > hull.height + EPS:
                continue
            for x in sorted(xs):
                if x < -EPS or x + lx > hull.length + EPS:
                    continue
                for y in sorted(ys):
                    if y < -EPS or y + ly > hull.width + EPS:
                        continue
                    box = Box("candidate", pose, (x, y, z))
                    if not hull.fits(box) or any(collides(box, old) for old in placed):
                        continue
                    if not (hull.floor_supports(box) or fully_supported(box, placed)):
                        continue
                    # Straight horizontal insertion at the final y/z.
                    blocked = any(
                        old.pos[0] + old.dims[0] <= x + EPS
                        and min(y + ly, old.pos[1] + old.dims[1])
                            - max(y, old.pos[1]) > EPS
                        and min(z + lz, old.pos[2] + old.dims[2])
                            - max(z, old.pos[2]) > EPS
                        for old in placed
                    )
                    if blocked:
                        continue
                    route = route_to(box, placed, hull, min_entry_z=0.4) if check_route else []
                    if check_route and route is None:
                        continue
                    options.append((box, route))
    options.sort(key=lambda row: (
        (row[0].pos[0] + row[0].dims[0] / 2) / hull.length * np.prod(row[0].dims),
        row[0].pos[2], -row[0].pos[1],
    ), reverse=True)
    # options contain only placements with a complete geometric insertion route.
    # Preserve diversity in depth and orientation, then cap branching.
    return options[:limit]


def candidate_features(dims, candidate, placed, remaining, hull):
    volume = float(np.prod(dims))
    depth = (candidate.pos[0] + candidate.dims[0] / 2) / hull.length
    footprint = candidate.dims[0] * candidate.dims[1]
    height = candidate.pos[2] / hull.height
    remaining_volume = sum(float(np.prod(box)) for box in remaining)
    largest_remaining_side = max(max(box) for box in remaining)
    # Scale features to order-one magnitudes for stable small-network training.
    return np.asarray([
        volume / (hull.length * hull.width * hull.height),
        *[axis / max(hull.length, hull.width, hull.height) for axis in candidate.dims],
        depth,
        footprint / (hull.length * hull.width),
        height,
        candidate.pos[1] / hull.width,
        len(placed) / 30.0,
        len(remaining) / 30.0,
        remaining_volume / hull.volume,
        largest_remaining_side / max(hull.length, hull.width, hull.height),
        float(np.prod(dims)) / max(remaining_volume, 1e-8),
        candidate.pos[0] / hull.length,
    ], dtype=np.float32)


class ScoreNet(nn.Module):
    def __init__(self, hidden=16):
        super().__init__()
        self.box_encoder = nn.Sequential(
            nn.Linear(BOX_FEATURE_COUNT, SET_EMBED), nn.Tanh(),
            nn.Linear(SET_EMBED, SET_EMBED), nn.Tanh(),
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(PLACED_FEATURE_COUNT, SET_EMBED), nn.Tanh(),
            nn.Linear(SET_EMBED, SET_EMBED), nn.Tanh(),
        )
        self.action_net = nn.Sequential(
            nn.Linear(4 * SET_EMBED + BOX_FEATURE_COUNT + ACTION_FEATURE_COUNT, hidden),
            nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, 1),
        )

    @staticmethod
    def normalize_box(dims, hull):
        scale = max(hull.length, hull.width, hull.height)
        return [float(axis) / scale for axis in dims]

    def forward(self, all_boxes, placed_boxes, features):
        """Score actions conditioned on the full manifest and current packed geometry."""
        encoded = self.box_encoder(all_boxes)
        set_summary = torch.cat((encoded.mean(dim=0), encoded.max(dim=0).values))
        if placed_boxes.shape[0]:
            state_encoded = self.state_encoder(placed_boxes)
            state_summary = torch.cat((state_encoded.mean(dim=0), state_encoded.max(dim=0).values))
        else:
            state_summary = torch.zeros(2 * SET_EMBED, dtype=features.dtype, device=features.device)
        n_actions = features.shape[0]
        context = torch.cat((set_summary, state_summary)).expand(n_actions, -1)
        return self.action_net(torch.cat((context, features), dim=-1)).squeeze(-1)


def lookahead_action(indices, action_map, env_id, ep, boxes, hull, beam_limit):
    """One-step geometric lookahead: preserve current value and future fit."""
    best = None
    remaining = [i for i in ep["remaining"]]
    for action_index in indices:
        _, box_index, candidate, _ = action_map[action_index]
        placed_next = ep["placed"] + [Box("lookahead", candidate.dims, candidate.pos)]
        current_volume = float(np.prod(candidate.dims))
        current_depth = (candidate.pos[0] + candidate.dims[0] / 2) / hull.length
        future = []
        # Only inspect the largest few successors; smaller boxes rarely
        # determine whether this placement blocks the packing horizon.
        successor_order = sorted(
            (next_index for next_index in remaining if next_index != box_index),
            key=lambda index: float(np.prod(boxes[index])), reverse=True)[:3]
        for next_index in successor_order:
            # Lookahead only estimates geometric fit. The selected action itself
            # has already passed the full swept-route check.
            next_options = placement_candidates(
                boxes[next_index], placed_next, hull,
                limit=1, check_route=False)
            if next_options:
                future.append(max(
                    float(np.prod(option.dims)) *
                    (option.pos[0] + option.dims[0] / 2) / hull.length
                    for option, _ in next_options
                ))
        future_value = sum(sorted(future, reverse=True)[:2])
        key = (current_volume * current_depth + 0.8 * future_value,
               current_volume, current_depth, -candidate.pos[2], -candidate.pos[1])
        if best is None or key > best[0]:
            best = (key, action_index)
    return best[1]


def beam_plan(boxes, hull, beam_width=4, candidate_limit=6):
    """Small sequence search used as a stronger teacher/reference policy."""
    states = [{"remaining": tuple(range(len(boxes))), "placed": [], "steps": [],
               "volume": 0.0, "depth": 0.0}]
    for _ in range(len(boxes)):
        expanded = []
        for state in states:
            if not state["remaining"]:
                expanded.append(state)
                continue
            actions = []
            for index in state["remaining"]:
                for candidate, route in placement_candidates(
                        boxes[index], state["placed"], hull, limit=candidate_limit):
                    volume = float(np.prod(candidate.dims))
                    depth = (candidate.pos[0] + candidate.dims[0] / 2) / hull.length
                    actions.append((volume * depth, index, candidate, route))
            actions.sort(reverse=True, key=lambda item: item[0])
            for _, index, candidate, route in actions:
                volume = float(np.prod(candidate.dims))
                depth = (candidate.pos[0] + candidate.dims[0] / 2) / hull.length
                box = Box(f"B{index + 1}", candidate.dims, candidate.pos)
                expanded.append({
                    "remaining": tuple(i for i in state["remaining"] if i != index),
                    "placed": state["placed"] + [box],
                    "steps": state["steps"] + [(index, box, route)],
                    "volume": state["volume"] + volume,
                    "depth": state["depth"] + volume * depth,
                })
        def state_score(state):
            potential = sum(float(np.prod(boxes[i])) for i in state["remaining"])
            return state["volume"] / hull.volume + 0.25 * state["depth"] / hull.volume + 0.03 * potential
        states = sorted(expanded, key=state_score, reverse=True)[:beam_width]
        if not states:
            break
    return max(states, key=lambda state: state["volume"] / hull.volume +
               0.25 * state["depth"] / hull.volume)


def batch_episodes(box_sets, hull, policy, device, greedy=False, beam_limit=12,
                   depth_weight=0.25, baseline_mode="volume", imitation=False,
                   fast_candidates=False):
    """Evaluate actions for several independent packing worlds as one torch batch per step."""
    episodes = [
        {"remaining": list(range(len(boxes))), "placed": [], "steps": [],
         "log_probs": [], "entropies": [], "step_rewards": [],
         "bc_losses": [],
         "placed_volume": 0.0, "depth_value": 0.0}
        for boxes in box_sets
    ]
    while any(ep["remaining"] for ep in episodes):
        active_features, action_map = [], []
        for env_id, (boxes, ep) in enumerate(zip(box_sets, episodes)):
            if not ep["remaining"]:
                continue
            start = len(active_features)
            for index in ep["remaining"]:
                for candidate, route in placement_candidates(
                        boxes[index], ep["placed"], hull, limit=beam_limit,
                        check_route=not fast_candidates):
                    action_features = candidate_features(
                        boxes[index], candidate, ep["placed"],
                        [boxes[i] for i in ep["remaining"]], hull)
                    scale = max(hull.length, hull.width, hull.height)
                    active_features.append(np.concatenate((
                        np.asarray(boxes[index], dtype=np.float32) / scale,
                        action_features,
                    )))
                    action_map.append((env_id, index, candidate, route))
            if start == len(active_features):
                # This environment is done; a sentinel is added after policy evaluation.
                ep["remaining"].clear()
        if not active_features:
            break
        feats = torch.as_tensor(np.stack(active_features), dtype=torch.float32, device=device)
        # Group candidate scores by environment and sample one independent action per world.
        for env_id, ep in enumerate(episodes):
            if not ep["remaining"]:
                continue
            indices = [i for i, action in enumerate(action_map) if action[0] == env_id]
            if not indices:
                ep["remaining"].clear()
                continue
            if policy is None:
                # Deterministic size-first/deep-first reference heuristic.
                def baseline_key(i):
                    _, box_index, candidate, _ = action_map[i]
                    volume = float(np.prod(candidate.dims))
                    depth = (candidate.pos[0] + candidate.dims[0] / 2) / hull.length
                    return (volume, depth, -candidate.pos[2], -candidate.pos[1])
                if baseline_mode == "lookahead":
                    selected = lookahead_action(indices, action_map, env_id, ep,
                                                boxes, hull, beam_limit)
                else:
                    selected = max(indices, key=baseline_key)
                local = indices.index(selected)
                distribution = None
            else:
                set_features = torch.as_tensor(
                    np.asarray([ScoreNet.normalize_box(box, hull) for box in box_sets[env_id]],
                               dtype=np.float32), device=device)
                scale = max(hull.length, hull.width, hull.height)
                state_features = torch.as_tensor(np.asarray([
                    [*(axis / scale for axis in item.dims),
                      *(axis / scale for axis in item.pos)] for item in ep["placed"]
                ], dtype=np.float32).reshape(-1, PLACED_FEATURE_COUNT), device=device)
                logits = policy(set_features, state_features, feats[indices])
                distribution = torch.distributions.Categorical(logits=logits)
                if imitation:
                    oracle_global = lookahead_action(indices, action_map, env_id, ep,
                                                     boxes, hull, beam_limit)
                    oracle_local = indices.index(oracle_global)
                    ep["bc_losses"].append(torch.nn.functional.cross_entropy(
                        logits.unsqueeze(0), torch.tensor([oracle_local], device=device)))
                    local = oracle_local
                else:
                    local = int(torch.argmax(logits).item()) if greedy else int(distribution.sample().item())
            selected = indices[local]
            # During training, route checks are deferred until after scoring.
            # Try candidates in policy order and commit only the first strict
            # swept-route-valid action.
            if fast_candidates:
                if distribution is None:
                    ranked = sorted(indices, key=baseline_key, reverse=True)
                else:
                    ranked = [indices[i] for i in torch.argsort(logits, descending=True).tolist()]
                    if selected in ranked:
                        ranked.remove(selected)
                        ranked.insert(0, selected)
                valid_action = None
                for candidate_index in ranked:
                    _, candidate_box_index, candidate_box, _ = action_map[candidate_index]
                    checked_box = Box("route_check", candidate_box.dims, candidate_box.pos)
                    checked_route = route_to(checked_box, ep["placed"], hull, min_entry_z=0.4)
                    if checked_route is not None:
                        valid_action = (candidate_index, checked_route)
                        break
                if valid_action is None:
                    ep["remaining"].clear()
                    continue
                selected, selected_route = valid_action
            else:
                selected_route = action_map[selected][3]
            if distribution is not None:
                ep["log_probs"].append(distribution.log_prob(torch.tensor(local, device=device)))
                ep["entropies"].append(distribution.entropy())
            _, box_index, candidate, route = action_map[selected]
            route = selected_route
            dims = candidate.dims
            box = Box(f"B{box_index + 1}", dims, candidate.pos)
            ep["placed"].append(box)
            ep["remaining"].remove(box_index)
            volume = float(np.prod(dims))
            depth = (candidate.pos[0] + dims[0] / 2) / hull.length
            ep["placed_volume"] += volume
            ep["depth_value"] += volume * depth
            ep["step_rewards"].append(volume / hull.volume + depth_weight * volume * depth / hull.volume)
            ep["steps"].append({
                "incoming": {"id": box.id, "dims": tuple(box_sets[env_id][box_index])},
                "actions": [{"kind": "place", "box": box.as_dict(), "route": route}],
                "status": "placed", "boxes": [item.as_dict() for item in ep["placed"]],
                "utilization": ep["placed_volume"] / hull.volume,
                "objective": ep["depth_value"], "relocation_distance_m": 0.0,
                "planning_seconds": 0.0,
            })
    results = []
    for ep in episodes:
        results.append({
            "steps": ep["steps"], "placed_count": len(ep["placed"]),
            "placed_volume": ep["placed_volume"], "depth_value": ep["depth_value"],
            "log_probs": ep["log_probs"], "entropies": ep["entropies"],
            "step_rewards": ep["step_rewards"],
            "bc_losses": ep["bc_losses"],
        })
    return results


def make_box_sets(seed, envs, count):
    rng = random.Random(seed)
    return [
        [
            tuple(round(float(x) * rng.uniform(0.92, 1.08), 3)
                  for x in random_luggage_sizes(rng.randrange(1 << 30), 1)[0])
            for _ in range(count)
        ]
        for _ in range(envs)
    ]


def evaluate(args, policy, hull, device):
    policy.eval()
    boxes = make_box_sets(args.eval_seed, args.eval_envs, args.boxes)
    with torch.no_grad():
        evaluation = batch_episodes(boxes, hull, policy, device, greedy=True, beam_limit=args.candidates)
    baseline = batch_episodes(boxes, hull, None, device, greedy=True,
                              beam_limit=args.candidates, baseline_mode=args.baseline)
    eval_rewards = np.asarray([
        ep["placed_volume"] / hull.volume + args.depth_weight * ep["depth_value"] / hull.volume
        for ep in evaluation
    ], dtype=np.float64)
    baseline_rewards = np.asarray([
        ep["placed_volume"] / hull.volume + args.depth_weight * ep["depth_value"] / hull.volume
        for ep in baseline
    ], dtype=np.float64)
    beam_reference = [beam_plan(manifest, hull, beam_width=args.beam_width,
                               candidate_limit=args.beam_candidates)
                      for manifest in boxes[:args.beam_eval_envs]]
    beam_rewards = np.asarray([
        state["volume"] / hull.volume + args.depth_weight * state["depth"] / hull.volume
        for state in beam_reference
    ], dtype=np.float64)
    sample = {
        "seed": args.eval_seed,
        "container": hull.as_dict(),
        "steps": evaluation[0]["steps"],
        "relocations": 0,
        "online": False,
        "policy": "learned_greedy",
        "relocation_depth": 0,
        "relocation_distance_budget_m": 0.0,
        "boxes_input": boxes[0],
    }
    validate_sample(sample)
    args.sample_output.parent.mkdir(parents=True, exist_ok=True)
    args.sample_output.write_text(json.dumps([sample], indent=2), encoding="utf-8")
    summary = {
        "checkpoint": str(args.output),
        "sample": str(args.sample_output),
        "eval_seed": args.eval_seed,
        "eval_envs": args.eval_envs,
        "boxes": args.boxes,
        "baseline": args.baseline,
        "beam_mean_reward": float(beam_rewards.mean()) if len(beam_rewards) else None,
        "beam_mean_placed": float(np.mean([len(state["placed"]) for state in beam_reference])) if beam_reference else None,
        "beam_width": args.beam_width,
        "eval_mean_reward": float(eval_rewards.mean()),
        "baseline_mean_reward": float(baseline_rewards.mean()),
        "mean_reward_delta": float((eval_rewards - baseline_rewards).mean()),
        "policy_win_rate": float(np.mean(eval_rewards > baseline_rewards)),
        "eval_mean_placed": float(np.mean([ep["placed_count"] for ep in evaluation])),
        "baseline_mean_placed": float(np.mean([ep["placed_count"] for ep in baseline])),
        "eval_mean_utilization": float(np.mean([ep["placed_volume"] / hull.volume for ep in evaluation])),
        "baseline_mean_utilization": float(np.mean([ep["placed_volume"] / hull.volume for ep in baseline])),
        "parameters": sum(parameter.numel() for parameter in policy.parameters()),
    }
    print(json.dumps(summary, indent=2), flush=True)
    return summary


def train(args, policy, hull, device):
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    optimizer = torch.optim.Adam(policy.parameters(), lr=args.learning_rate)
    history = []

    data_rng = random.Random(args.seed)
    for epoch in range(args.epochs):
        unique_n = max(1, args.envs // args.rollouts_per_instance)
        unique_boxes = make_box_sets(data_rng.randrange(1 << 30), unique_n, args.boxes)
        boxes, groups = [], []
        for group_id, manifest in enumerate(unique_boxes):
            repeat = args.envs // unique_n + int(group_id < args.envs % unique_n)
            boxes.extend([manifest] * repeat)
            groups.extend([group_id] * repeat)
        episodes = batch_episodes(boxes, hull, policy, device,
                                  beam_limit=args.candidates, depth_weight=args.depth_weight,
                                  imitation=args.behavior_cloning,
                                  fast_candidates=args.fast_training)
        rewards = torch.as_tensor([
            ep["placed_volume"] / hull.volume + args.depth_weight * ep["depth_value"] / hull.volume
            for ep in episodes
        ], dtype=torch.float32, device=device)
        objective_samples = {group: [] for group in set(groups)}
        for group_id, ep in zip(groups, episodes):
            returns = np.cumsum(np.asarray(ep["step_rewards"], dtype=np.float32)[::-1])[::-1]
            for step_id, value in enumerate(returns):
                objective_samples[group_id].append((step_id, float(value)))
        group_baselines = {}
        for group_id, rows in objective_samples.items():
            by_step = {}
            for step_id, value in rows:
                by_step.setdefault(step_id, []).append(value)
            group_baselines[group_id] = {step: float(np.mean(values)) for step, values in by_step.items()}
        losses = []
        if args.behavior_cloning:
            losses.extend(loss for ep in episodes for loss in ep["bc_losses"])
        for env_id, ep in enumerate(episodes):
            returns = np.cumsum(np.asarray(ep["step_rewards"], dtype=np.float32)[::-1])[::-1]
            for step_id, (log_prob, entropy, value) in enumerate(zip(
                    ep["log_probs"], ep["entropies"], returns)):
                baseline = group_baselines[groups[env_id]].get(step_id, float(value))
                advantage = torch.as_tensor(float(value - baseline), dtype=torch.float32, device=device)
                losses.append(-advantage.detach() * log_prob - args.entropy * entropy)
        if losses:
            optimizer.zero_grad(set_to_none=True)
            torch.stack(losses).mean().backward()
            nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            optimizer.step()
        row = {
            "epoch": epoch + 1,
            "mean_reward": float(rewards.mean().item()),
            "mean_placed": float(np.mean([ep["placed_count"] for ep in episodes])),
            "mean_volume_utilization": float(np.mean([ep["placed_volume"] / hull.volume for ep in episodes])),
            "mean_depth_value": float(np.mean([ep["depth_value"] / hull.volume for ep in episodes])),
        }
        history.append(row)
        if (epoch + 1) % args.log_every == 0 or epoch == 0 or epoch + 1 == args.epochs:
            print(json.dumps(row), flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "state_dict": policy.cpu().state_dict(),
            "action_feature_count": ACTION_FEATURE_COUNT,
        "box_feature_count": BOX_FEATURE_COUNT,
        "set_embed": SET_EMBED,
        "hidden": args.hidden,
        "seed": args.seed,
        "reward": "placed_volume_ratio + depth_weight * depth_value_ratio",
        "history": history,
    }, args.output)
    if args.history:
        args.history.parent.mkdir(parents=True, exist_ok=True)
        args.history.write_text(json.dumps(history, indent=2), encoding="utf-8")

    policy.to(device)
    evaluate(args, policy, hull, device)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("train", "eval"), default="train")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--envs", type=int, default=32)
    parser.add_argument("--rollouts-per-instance", type=int, default=4)
    parser.add_argument("--eval-envs", type=int, default=32)
    parser.add_argument("--boxes", type=int, default=10)
    parser.add_argument("--candidates", type=int, default=12)
    parser.add_argument("--hidden", type=int, default=16)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--eval-seed", type=int, default=20261019)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--depth-weight", type=float, default=0.25)
    parser.add_argument("--baseline", choices=("volume", "lookahead"), default="lookahead")
    parser.add_argument("--beam-width", type=int, default=4)
    parser.add_argument("--beam-candidates", type=int, default=4)
    parser.add_argument("--beam-eval-envs", type=int, default=1)
    parser.add_argument("--entropy", type=float, default=0.01)
    parser.add_argument("--behavior-cloning", action="store_true",
                        help="pretrain on finite-lookahead oracle actions before policy updates")
    parser.add_argument("--fast-training", action="store_true",
                        help="defer strict route checks until after policy scoring")
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cuda:0")
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parent / "vendor" / "roboticarm")
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("packing_policy.pt"))
    parser.add_argument("--history", type=Path, default=Path(__file__).with_name("packing_training.json"))
    parser.add_argument("--sample-output", type=Path, default=Path(__file__).with_name("packing_policy_sample.json"))
    args = parser.parse_args()
    if min(args.epochs, args.envs, args.eval_envs, args.boxes, args.candidates,
           args.hidden, args.rollouts_per_instance) < 1:
        parser.error("epochs, envs, eval-envs, boxes, candidates, hidden and rollouts-per-instance must be positive")
    if args.scene_config is None:
        args.scene_config = args.source_root / "elfin_humble_ws" / "src" / "luggage_description" / "config" / "scene_tf.yaml"
    hull_data = __import__("yaml").safe_load(args.scene_config.read_text(encoding="utf-8"))
    hull = Hull.from_config(hull_data)
    device = torch.device(args.device)
    if args.mode == "eval":
        checkpoint = torch.load(args.output, map_location=device, weights_only=False)
        policy = ScoreNet(checkpoint["hidden"]).to(device)
        policy.load_state_dict(checkpoint["state_dict"])
        evaluate(args, policy, hull, device)
    else:
        train(args, ScoreNet(args.hidden).to(device), hull, device)


if __name__ == "__main__":
    main()
