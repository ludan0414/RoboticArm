# Real-scenario packing and occupancy

A loading cell does not know exact box size or exact free space. Online
geometry comes from live perception plus a small set of geometric priors.
Gazebo truth, the box catalog, and a planned place pose are not perception.

This document is the product contract. The numbered RS items are **tracked
defects** on the current tree, not allowed designs. New code must not copy
them. A later generation may close items only with perception-honest
acceptance criteria.

Related contracts: [container_geometry.md](container_geometry.md) (hull
identity), [placement.md](placement.md) (how candidates are enumerated on a
map), [production_orchestration.md](production_orchestration.md) (cycle and
no-GT rule).

## Allowed priors

- Container usable hull / STL from `scene_tf` and
  `luggage_description.container_geometry`.
- Empty-floor **existence** when no accepted cargo map exists (`floor_prior`,
  `peak ≈ floor_z`). This is not a claim that the interior is empty of cargo.
- Eval / Gazebo truth as a **score numerator** or dump, never as the payload
  of `DetectLuggage`, `ComputePlacement`, cargo-map commit, or MoveIt collision
  for the online path.

Catalog suitcase WDH is **spawn-only** (`pickup_box_spawner`). It must not
fill `DetectedLuggage.height`, MoveIt pickup AABBs, or `ComputePlacement`.

The operator explicitly selects configured dimensions for the separate real-cell
`site_commissioning_v1` console (`payload_geometry_source=configured_cycle`).
That console may build its MoveIt payload from the current cycle's configured
WDH and the visual pickup top/yaw. It labels the source as configured, retains
the original detector observation, and does not certify measured occupancy or
alter production `DetectLuggage` / `ComputePlacement` / cargo-map contracts.

## Forbidden successive priors

The following loop is a defect wherever it drives those APIs:

```text
catalog or GetCurrentBox exact WDH
  -> ComputePlacement
  -> planned slot
  -> AddPlacedBox SOURCE_GEOMETRY
  -> surface_2d exact occupancy
  -> next ComputePlacement
```

Hard rules:

- Do not feed Gazebo, `GetCurrentBox`, catalog AABB, or
  `eval_perfect_geometry` into `DetectLuggage` / `ComputePlacement` / cargo
  map / vacuum attach / MoveIt collision as if it were measured.
- Do not stamp GT or catalog size as `height_valid=true` or
  `HEIGHT_SOURCE_MEASURED_SUPPORT`.
- Do not treat `HEIGHT_SOURCE_CONFIGURED_SUPPORT` (`platform_z`) as measured
  FULL_3D geometry.
- Cargo-map occupancy for the next place must come from live depth and/or a
  **verified measured** commit. An unverified planned slot is not occupancy.
- `request.placed` must not be a second copy of the same unverified plan used
  to hide commit–reality disagreement.
- `floor_prior` must not stand in for occupancy after boxes have been placed
  but not perceived or verified.
- Production and eval drivers that call production services must not branch on
  `sim_mode`, skip perception, or use a fake success path. Eval GT stays in
  the dump / score table.
- Humble pick requires `height_source=HEIGHT_SOURCE_MEASURED_SUPPORT`.
  TOP_ONLY is a valid detection (cargo present, height unknown) and never
  grasp authority. Wait or change viewpoint; do not invent height.
- ROS1 `allow_gt_fallback` / `strict_perception=False` / gazebo inspect are
  forbidden patterns, including in unported scripts.

Honest product loop (pick gate on Humble; occupancy node is closed at
`RS-1`, production caller still `RS-7`):

```text
DetectLuggage (TOP_ONLY is a detection, not a grasp)
  -> wait / reobserve until MEASURED_SUPPORT
  -> pick
  -> ComputePlacement (height_source=MEASURED_SUPPORT)
  -> place
  -> RGBD / depth update of cargo map
  -> commit measured or verify-gated geometry
```

## Branch check

Inventory at `09fb4843f586b46ed8fae0efd71caa93f58a5dd7` (local `master`,
ahead of `origin/master`). Other local/remote branches are merged, behind, or
older (`ros2_humble` lacks place-only perfect geometry). None adds a worse
oracle. `deployment_ws` is a site trajectory executor, not packing GT.

## Defect catalog

IDs are stable (`RS-1` .. `RS-27`). Closing an item requires a
higher-generation plan with metrics, tests, and `dispatch_ready: yes`.

### A. Occupancy without perception

- **RS-1.** Closed at the node/service level: Humble
  `cargo_volume_mapper_node` exposes `/cargo_map/integrate_cargo_view`
  (`IntegrateCargoView`). It buffers `/luggage/semantic/cargo_points_untracked`
  (label-filtered, pre-`CargoInstanceTracker`), selects by acquisition stamp
  inside a 0.20 s window, looks up TF at that stamp into `container_link`,
  voxel-downsamples, then calls `CargoVolumeMapper.integrate_points`.
  Latest-cloud integrate and tracker-gated `/luggage/semantic/cargo_points`
  are forbidden. The production pack/orchestrator still does not call the
  service (`RS-7`). Planned `AddPlacedBox` remains a `SOURCE_GEOMETRY` lock
  (`RS-2`). `src/luggage_perception/scripts/cargo_volume_mapper_node.py`,
  `src/luggage_perception/luggage_perception/cargo_view_integration.py`.
- **RS-2.** Cargo-map commit is the planned `ComputePlacement` slot, not a
  measured settled box. `src/luggage_gazebo/scripts/place_only_eval_driver.py`,
  `src/luggage_gazebo/scripts/pack_eval_driver.py`.
- **RS-3.** `ComputePlacement.request.placed` is the same planned AABBs as the
  map, so overlap/corridor cannot see commit–reality drift.
- **RS-4.** `floor_prior` (all unknown, height 0) is used as if the interior
  were known empty after unperceived places. Legal only as empty-floor
  existence with no accepted map.
- **RS-5.** Place-only fixture boxes are authored into the map (`in_map` →
  `AddPlacedBox`) before scoring, so the planner starts with declared
  occupancy, not exploration.
- **RS-6.** Humble `VerifyPlacedBox` is missing. Contracts require
  `COMMIT_AND_VERIFY`; the node is only
  `src/luggage_perception/scripts/ros1_reference/placed_pose_verifier_node.py`.
  Eval labels `VERIFIED` from `ign model --pose`.
- **RS-7.** No Humble production orchestrator runs `EXPLORE_CONTAINER` then
  live-map updates. Only `orchestration_contracts.py` plus eval drivers.

### B. Exact box size into packing APIs

- **RS-8.** Closed: `ComputePlacement` and the pick drivers refuse
  `height_source!=MEASURED_SUPPORT`. TOP_ONLY waits then fail-closed
  (`DETECT_FULL_GEOMETRY_REQUIRED`); catalog/configured height never
  authorize a grasp. `src/luggage_planning/luggage_planning/pick_authorization.py`.
- **RS-9.** `HEIGHT_SOURCE_CONFIGURED_SUPPORT` still sets `height_valid=true`.
  Param `platform_z` is treated as a fitted support plane in configured
  mode. Pick authorization ignores it. Launches stay `support_mode=auto`.
  `src/luggage_perception/luggage_perception/top_support_estimator.py`.
- **RS-10.** Place-only injects `eval_perfect_geometry` into production
  services (`height_source=2`, `height_valid=true`) with
  `use_perception: false`. Solver isolation only; it must not call
  `DetectLuggage` and is not the product pick loop.
  `src/luggage_gazebo/luggage_gazebo/place_only_fixture.py`,
  `src/luggage_gazebo/config/place_only_profile.yaml`.
- **RS-11.** Pack-eval ledger still records spawn WDH as score metadata.
  `ComputePlacement` now takes `DetectLuggage`. Vacuum/sim follow still
  reads `GetCurrentBox` (see RS-13).
  `src/luggage_gazebo/scripts/pack_eval_driver.py`,
  `src/luggage_gazebo/scripts/pickup_box_spawner_node.py`.
- **RS-12.** Closed: place-smoke `--dry-run` / `--payload none` require
  an explicit `--synthetic-size W,D,H` flagged `synthetic_plan_only`.
  `src/luggage_gazebo/scripts/place_smoke_driver.py`.
- **RS-13.** Vacuum gate reads spawner JSON size/mass/pose, not
  `DetectLuggage`. `src/luggage_planning/scripts/vacuum_controller_node.py`.
- **RS-14.** Legacy `match_catalog` is forbidden (raises). Humble detector
  does not call it. ROS1 reference still passes `catalog_entries` into
  `estimate_box`, which now ignores them.
  `src/luggage_perception/luggage_perception/luggage_box_estimator.py`.

### C. ROS1 leftover oracles

Unported / `ros1_reference` / `COLCON_IGNORE`. Forbidden to copy into Humble.

- **RS-15.** Detector `allow_gt_fallback=True` publishes `GetCurrentBox` as
  `DetectLuggage` success.
  `src/luggage_perception/scripts/ros1_reference/luggage_detector_node.py`.
- **RS-16.** Orchestrator default `strict_perception=False` falls back to
  spawner GT; default inspect `gazebo_gt`; optional
  `/gazebo/get_model_state`. `src/luggage_bringup/scripts/orchestrator_node.py`.
- **RS-17.** Container inspector occupancy from `/gazebo/model_states`.
- **RS-18.** Pickup cloud filter carves an exact GT OBB.
- **RS-19.** Scene manager pickup collision from `GetCurrentBox` as
  `spawner_gt`.
- **RS-20.** `bin_packer_node` places from `/luggage/current_box` size.
- **RS-21.** Reachability atlas hard-rejects on the ROS1 planner. Humble live
  `placement_planner_node` does not call the atlas.

### D. Smaller online shortcuts

- **RS-22.** `RobotSelfPointFilter` latest-TF fallback. Already in
  [README.md](README.md) known deviations.
- **RS-23.** Semantic segmenter self-mask looks up TF at `now()`, not the
  image stamp. `src/luggage_perception/scripts/semantic_segmenter_node.py`.
- **RS-24.** Hardware pick `use_perception_approach:=false`; Humble waypoint
  node never passes `perception_info`.
- **RS-25.** Closed: `site_pick_replay` skips the cargo collision object
  when `height_valid=false` and records `scene_skipped_height_invalid`.
  Plan-success is not comparable to earlier 0.30 m runs.
- **RS-26.** Closed: pack-eval utilization denominators are `volume()` /
  `floor_area()` of the scene_tf seven-face hull, not the enclosing AABB.

### E. Place motion without occupancy

- **RS-27.** Place motion (cartesian `GetCartesianPath` and OMPL
  `RRTConnect` in `motion_executor`) collision-checks MoveIt URDF, the
  attached payload, and `scene_manager` boxes only. It does not query
  cargo occupancy (`surface_2d` / voxel map) at `geometry_hash` /
  `map_revision`, does not emit a scored multi-path set, and has no
  dimension/safety/efficiency selector. OCC-1 trial 1
  `PLACE_PLAN_traverse` fraction 0.859 then OMPL `FAILURE` is this gap.
  Contract: [placement.md](placement.md) Place motion through occupancy.
  `src/luggage_planning/luggage_planning/motion_executor.py`.

## Not defects

- Container seven-face hull from `scene_tf`.
- Humble detector: measured top/XY; no catalog height; `height=0` and
  `HEIGHT_SOURCE_UNAVAILABLE` when support is unmeasured; no `GetCurrentBox`
  in the estimate.
- `pick_retreat_eval` scoring Detect vs `GetCurrentBox` as a metric only.
- Unit-test synthetic maps that do not call production services as oracles.
- Catalog YAML consumed by `pickup_box_spawner` to spawn Gazebo models.

## Severity

Highest for a real cell: **RS-27** (place carry does not read occupancy;
second-box `PLACE_PLAN_traverse` is this), then **RS-11, RS-10** (packing
GT metadata / solver isolation) plus **RS-2, RS-3** (planned-geometry
commit still fills the map). `RS-1` is closed at the node/service; the
production caller is `RS-7`. Next: occupancy-aware multi-path motion,
then **RS-6, RS-7**, then ROS1 fallbacks **RS-15, RS-16** if that stack
is run. **RS-8, RS-12, RS-25** are closed by the pick-authorization gate.
