# Placement

How a box is proposed inside the container after detection. Hull identity,
clipping, and the `ComputePlacement` failure taxonomy stay in
[container_geometry.md](container_geometry.md). What may fill the occupancy
map and the payload box is [real_scenario.md](real_scenario.md): live
perception plus allowed priors, not catalog/GT/planned pose.

The live ROS 2 Humble planner is `placement_planner_node` calling
`placement_solver.generate_candidates` / `solve_placement` on
`surface_map_2d`. A separate FreeSpaceModel + `placement_scoring` path exists
for offline replay and the unported ROS 1 planner. That path is not the live
node until the node and this document switch together.

Hard rules:

- Candidate generation enumerates discrete support-surface poses. It is not
  Monte Carlo sampling from a pose prior.
- Each enumerated pose is a complete oriented payload box. Center-point
  placement checks are invalid (same rule as the hull document).
- Hull, overlap, and top-clearance gates run before ranking, retention,
  `top_n`, or first-feasible selection.
- Commissioning doorway clearance is a transport constraint, independent of
  physical hull/capacity geometry. The shared description doorway contract
  creates inflated front-frame collision prisms; real planning and Newton must
  apply them to the full robot/payload trajectory. Current site minimum is5cm.
  Unknown/missing live guard geometry or attached payload geometry fail closed.
  Per operator instruction, the commissioning doorway gate does not read or
  rewrite the ACM; it uses normal MoveIt trajectory collision checking.
  Final support/placement poses must not be silently shifted to satisfy
  doorway clearance; replan the entry corridor or reject the slot.
- For the operator's rectangular commissioning payload model, wheels/front/back
  are intentionally ignored. The console may select one box-local Z half-turn
  symmetry for the complete route to keep yaw within90deg of the picked box.
  This preserves the full occupied cuboid and its centre; a90deg axis swap is
  still a policy decision. Regenerate TCP goals from the unchanged measured
  grasp, and use the selected heading for release validation and expected proxy.
- Stacking on previously placed cargo is allowed only on **known** support.
  Unobserved columns may land on the a-priori floor (`peak ≈ floor_z`,
  `support_source=floor_prior`). Stacking on an unobserved surface
  (`peak > floor_z`) is `unknown_above_floor`.
- `surface_2d` occupancy must come from live depth and/or a verified
  measured commit ([real_scenario.md](real_scenario.md)). An unverified
  planned slot, catalog size, or `GetCurrentBox` is not occupancy. Current
  Humble planned-geometry commit is `RS-2` / `RS-3`, not the contract.
  Node-level depth integrate is closed (`RS-1`).
- `request.placed` is a second occupancy source for overlap and the
  insertion corridor. It must not be a duplicate of the same unverified plan
  used to hide commit–reality disagreement (`RS-3`).
- The floor prior is legal when no map has been published, as empty-floor
  **existence**. It must not claim the interior is empty of cargo (`RS-4`).
  A rejected map must fail closed, not answer from the floor prior.
- `ComputePlacement` must fail closed until `height_source=MEASURED_SUPPORT`.
  TOP_ONLY is a detection, not a placement payload (`RS-8` closed).
- Place **motion** (cartesian, OMPL, or any later search) must plan against
  the same cargo occupancy the solver reads. A collision world of only URDF
  plus `scene_manager` boxes is not occupancy. Current Humble traverse is
  tracked defect `RS-27`.

## Candidate generation (live Humble)

`generate_candidates` slides an axis-aligned footprint over the
container-local 2.5D height map:

- Yaw is snapped to `{0, π/2}` footprints.
- `peak` is the maximum `height` under the footprint.
- The box rests on that surface: `z = peak + box_height / 2` in
  floor-relative coordinates.
- Default map resolution is 0.05 m with `stride_cells=1`.

`floor_prior` is a synthetic all-unknown, height-zero grid used when the
planner has no accepted `surface_2d`. The floor's **existence** is geometric
(scene_tf), not a perception claim. It is not a sampling distribution.

## Stacking

A previously placed box becomes a support surface when its footprint columns
are `occupied` (commit rasterized as `SOURCE_GEOMETRY`, or sensor-occupied
once depth integration exists). Face contact with a placed AABB is allowed;
positive-volume overlap is a capacity reject.

A stack candidate is feasible when:

- the footprint has no unknown cells, or every unknown cell is still at the
  floor (`peak ≈ floor_z` within `support_tol`, default 0.05 m);
- `support_ratio ≥ min_support_ratio` (default 0.6): fraction of footprint
  cells within `support_tol` of `peak`;
- remaining top clearance is at least `clearance_margin` (default 0.03 m);
- hull, overlap, aperture, and insertion-corridor gates pass.

There is no API to target a specific prior box. Stacking is an automatic
candidate at XY cells whose `peak` is that box's top.

## Scoring (flatten first)

Live score, defaults in `placement_solver._default_params`:

```text
score = 0.5 * support_ratio
      + 0.3 * clearance_score      # remaining height above the box, in [0, 1]
      + 0.2 * compactness          # 1 - peak / inner_h
      + 0.15 * confidence_ratio    # fraction of sensor cells only
```

Weights do not change after a commit. The height map changes, so stack
candidates appear. `compactness` and remaining clearance both reward **low**
placements: fill the floor before climbing. `w_compactness`'s comment
("prefer low placements (good stacking)") means leave headroom for later
stacks, not stack now.

`confidence_ratio` counts only `confidence=sensor` cells. Geometry-committed
tops and `floor_prior` columns score 0 on that term.

`solve_placement` selects `max(feasible, key=score)`. The Humble node keeps
`top_n=2000` so corridor and aperture gates see deep slots, not only the
highest scores.

Stacking wins once no feasible floor slot remains (floor tiled, remaining
floor lost to overlap / hull / corridor, or clearance under a full-floor
occupied map).

## Occupancy update after place

Required source of `surface_2d` is live depth integration and/or a verified
measured commit. Gazebo physics and MoveIt collision objects are not the
cargo map. See [real_scenario.md](real_scenario.md).

Current Humble path (tracked defect `RS-2`, `RS-3`, `RS-6`; `RS-1` closed
at the node/service):

`/cargo_map/integrate_cargo_view` → stamp-matched untracked cargo cloud →
`CargoVolumeMapper.integrate_points` (`confidence=sensor`). Fail-closed on
unsettled, hash/revision mismatch, stale/duplicate stamp, missing TF, or
empty view. Input is `/luggage/semantic/cargo_points_untracked`, not the
tracker-gated pickup track.

`/cargo_map/add_placed_box` → `CargoVolumeMapper.mark_placed_box`:

1. Record the box on the commit ledger (idempotent within 0.05 m center/size
   and matching yaw).
2. Rasterize the interior as `SOURCE_GEOMETRY` voxels (lock against
   free-space misses).
3. Bump `map_revision` and republish `/luggage/cargo_map/surface_2d` plus the
   commit ledger.

Eval pack drivers currently pass the **planned** slot into AddPlacedBox, not
a measured settled pose (`RS-2`). Humble has no `VerifyPlacedBox` node
(`RS-6`); eval labels `VERIFIED` from `ign model --pose`. Nothing in the
product loop calls `IntegrateCargoView` yet (`RS-7`); the OCC-1 eval driver
is the node-level caller.

`surface_map_2d` remains the solver join contract: per-column top occupied
height above the inner floor, plus `state`, `clearance`, `confidence`,
`geometry_hash`, and `map_revision`. Next `ComputePlacement` reads that
peak. The defect is what is allowed to raise the peak, not that the solver
reads a height field.

`/scene_manager/add_placed_box` only inserts a MoveIt collision object. It is
not a cargo-map commit.

A skipped map commit leaves the height field empty. The next plan can then
propose a floor slot through the real box. Duplicate commits do not bump
`map_revision`. Voxel tops are `(top_occ + 1) * resolution`, so a committed
top can sit slightly above the geometric lid.

## Place motion through occupancy

`ComputePlacement` choosing a feasible slot is not permission to carry.
Every place-motion planner (cartesian interpolation, OMPL, or a later
search) must query the **current** cargo occupancy at the request
`geometry_hash` / `map_revision`. Occupied cells and unknown-above-floor
cells are obstacles for the swept payload and arm. Empty-floor existence
(`floor_prior`) is legal only when no accepted map exists.

A single straight cartesian to the slot, collision-checked only against
MoveIt URDF and `AddPlacedBox` scene objects, is a defect (`RS-27`).
Gazebo contacts are not a substitute for that occupancy query.

### Candidate paths

When a place segment is planned, the planner must produce a **set** of
paths (N ≥ 2 whenever the first candidate is in collision, below the
cartesian fraction gate, or has no IK), not one discarded line plus one
failed OMPL call. Methods may include raised/offset cartesian waypoints,
alternative yaws already feasible at the slot, and free-space search.
Each candidate that collides with occupancy is infeasible, not a scored
backup.

### Path selector

A selector picks among remaining collision-free paths by a scalar cost
with three occupancy terms (weights live in planner config, not in
call-site constants). When `ComputePlacement` returns more than one
feasible slot, the selector scores every feasible `(placement, path)`
pair. Placement quality is a fourth cost term so a cheap path into a
worse slot does not beat a slightly longer path into a better slot.

| Term | Meaning |
|---|---|
| Placement | Solver score of the target slot, inverted so better slots cost less. |
| Dimension | Swept payload size versus remaining occupancy clearance and opening aperture. Penalize paths that need more width/height than free columns allow. |
| Safety | Minimum clearance from the swept payload and arm to occupied and unknown cells. Collision is infeasible; near-miss is a cost. |
| Efficiency | Path length, estimated duration, and cartesian fraction. Prefer a short high-fraction carry when it is still safe. |

Execute the minimum-cost feasible `(placement, path)`. If the set is empty, fail closed
with a motion/path reason (`PLACE_PLAN_*` or a dedicated path-infeasible
code). Do not relabel that as `BIN_FULL` or `PLACE_CANDIDATE_EXHAUSTED`.
Retain every tried path and rejection reason in the dump (same rule as
exhausted placement candidates).

Current Humble `motion_executor` cartesian `avoid_collisions` plus OMPL
`RRTConnect` fallback does not read `surface_2d` and does not keep a
scored path set (`RS-27`).

## Continuous commissioning motion

The commissioning console sends compatible carry-phase samples as ordered
Cartesian waypoints in one action. A complete Cartesian path uses the existing
collision checks. Incomplete coverage automatically falls back to complete
collision-checked IK/joint goals through every original waypoint in order,
even when the legacy allow_ompl_fallback flag is false. Never execute the
Cartesian partial prefix or shortcut directly past the route's via poses.
This fallback can depart from Cartesian chords and may stop at each pose;
it also applies to descent/retreat after finer Cartesian retries are exhausted.
Release still requires full target success and the existing measured-pose check.
Lift/retract/high-lift phases, descent, release and retreat remain
independent boundaries. Successful carry poses may be reversed as a single
empty Cartesian return; only success consumes the reverse route. For a
`return_pick_stop_` segment, incomplete Cartesian coverage must fail closed:
do not execute a partial prefix or replan joint-space hops between waypoints,
because either may depart from the recorded return path. Other commissioning
segments retain the complete-waypoint recovery described above.
Fallback subgoal results and their trajectories are captured in the motion
boundary; failed/cancelled subgoals stop the route and preserve vacuum.

Operator speed_scale is10..100% of existing planned tempo and is captured once
per PlanMotion. Apply it to the full trajectory time law before ExecuteTrajectory
so MoveIt sees the slowed duration. Hardware launch uses velocity/acceleration
scaling 0.30/0.30 (about 27deg/s, capped to 20deg/s²). Compare uniform
time dilation with locally dilated, monotone joint Hermite interpolation and
choose the shorter analytically bounded result. Keep every geometric knot;
local interpolation must not overshoot either endpoint of any joint interval.
Intermediate velocities stay continuous, with zero velocity at joint reversals.
Bound actual cubic-Hermite velocity/acceleration analytically, including corners.
Neither UI tempo nor continuous execution relaxes collision, payload or existing
hardware dynamics limits. Speed changes require an idle/fault console and typed
ROS parameter readback before confirmation.

PlanMotion clients use phase-feedback inactivity deadlines, not a total
per-waypoint timeout for a continuous route. The executor publishes waiting
heartbeats only within its bounded ExecuteTrajectory wait, whose deadline is
derived from the planned duration. Feedback proves liveness only; success
requires the final action result. Missing feedback cancels the client goal;
server timeout/failure still propagates and preserves the held payload.
Console faults include the originating place stage and reason.

Commissioning sessions record passive telemetry by default in a unique
session JSONL, independently of overwriteable per-box traces. Record actual
joints, currents in ampere, field-validity flags, source and receipt timestamps,
vacuum feedback, controller errors, original requests and full command grids.
Use the executor's existing CPS connection and already-read samples; do not
open a separate monitor connection or infer missing actual samples from plans.
Preserve deployed dirty sources, topic coverage and file errors for diagnosis.

## Reach-limit handling

Commissioning transit IK may shorten an unreachable intermediate carry goal
along the measured current-TCP-to-target line. Search the outer feasible
bracket, retaining target attitude, configured joint limits and the active
scene policy. This is a changed transit endpoint, not exact-goal success.
Pick contact, place descent and recorded-return requests must never use this
fallback. Preserve requested/achieved poses and shortfall in boundary logs;
return recording uses achieved transit poses, never shortened-away requests.
Contact placement and its configured support remain unchanged. Newton's
equivalent transit search uses the shoulder ray and checks every resulting
frame against the scene; its achieved goal is logged independently of the
original target. A closest endpoint does not certify the swept path.

## Replay / redesign path (not the live node)

`FreeSpaceModel` replaces the sliding window with a persistent heightmap,
committed-box AABBs, LBCP support polygons, and exact stack-center poses.
`placement_scoring.score_candidates` adds `proxy_score` (EMS regularity,
corridor, reachability atlas prior, …) plus `w_floor_first=0.60`.
EMS tiles stay axis-aligned empty cuboids for indexing; regularity and the
blocked-deep term clip those tiles to the seven-face hull.

`proxy_score` without that term is structurally stack-biased: a committed box
top scores high on support and observation confidence, an unobserved floor
scores 0. The floor-first term restores flatten-first for replay. The
reachability atlas is a **score prior**, not the candidate generator.

Offline replay (`packing_replay.FreeSpaceModelStrategy`) and
`scripts/ros1_reference/placement_planner_node.py` use this path. Humble
`placement_planner_node.py` does not.
