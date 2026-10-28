# Policy target -> hardware pick and place

The hardware launch still starts nodes only. One driver invocation authorizes
one pick. With `--place-target`, a successful pick continues to policy via
poses, pre-place, descend, vacuum release, confirmed DI0 low, and retreat.
Without the flag the pick-only behaviour is unchanged.

## Input contract

Have the policy integration write one JSON file per box, then pass its path.
The file is loaded once at driver startup. Values are metres and a unit
quaternion in **x, y, z, w** order. The target describes the **box centre**, not
the suction TCP, box corner, or support surface. All poses must already be
transformed into the measured deployment `world` frame.

```json
{
  "frame_id": "world",
  "pose_reference": "box_center",
  "position_m": [0.8, 0.0, 0.3],
  "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
  "clearance_m": 0.15,
  "via_box_poses": []
}
```

Numbers above illustrate the schema; they are not a measured site target.
`clearance_m` defaults to 0.15 and must be at least 0.05. It offsets pre-place
and retreat in world +Z. Each optional `via_box_poses` item has `position_m`
and `orientation_xyzw` and is executed in order before pre-place. Use these
to retain the policy's doorway/cargo routing. Incomplete Cartesian coverage
no longer fails solely because fraction is below1: discard the partial prefix
and automatically execute complete collision-checked MoveIt pose goals through
all original via poses and the final target, in order, with the payload attached.
The legacy fallback=false flag does not disable this recovery. This can depart
from Cartesian chords and is recorded as an OMPL fallback, including each
subgoal's trajectory/result in the motion boundary. Final descent first retries
half and quarter Cartesian interpolation step; if still incomplete, it uses
the same full-goal recovery. Descent/retreat targets remain separate actions;
failed or cancelled full goals still stop, preserve vacuum and prevent release.
Carry errors include the failing segment name. A fraction alone cannot tell
whether the cause was IK, collision, or a singularity; preserve the boundary
trace when diagnosing a real-robot failure.

The selected successful suction candidate defines the actual grasp, including
off-centre contact and its orientation. The adapter computes
`T_world_tool_target = T_world_box_target * inverse(T_world_box_pick) * T_world_tool_attach`.
It does not reuse the legacy centre-top place formula or discard roll/pitch.
Malformed quaternions, nonfinite values and unknown frames fail before pick.
Detected and candidate poses must both be in `world`.
The detected box yaw must be valid to establish the box-to-grasp transform.

The existing `policy/online_packing_real.py` sample format (`box.pos`, rotated
dimensions, `route`) is not this interface: it uses container-local box corner
coordinates. Its caller must convert corners to centres, recover the matching
box rotation, and apply the measured container-to-world transform first.
Do not feed that sample JSON directly to this driver.

## Run

Start the existing deployment graph with the measured scene/site configuration
and required hardware gates from `deployment_ws/README.md`. Build the updated
`luggage_planning` package in the root ROS workspace and source the overlay.
Then, with a person on e-stop:

```bash
# Preview candidate-specific pick AND place targets. No motion or vacuum.
./deployment_ws/scripts/run_floor_box_pick.sh plan --place-target /path/to/target.json

# One full pick -> place cycle. Trace file's parent directory must exist.
./deployment_ws/scripts/run_floor_box_pick.sh execute \
  --place-target /path/to/target.json --trace-out /path/to/evidence/pick_place.jsonl
```

The direct command is `ros2 run luggage_planning hardware_pick_driver.py
--place-target /path/to/target.json`. `--plan-only` prints waypoints; it is not
an IK/collision feasibility certificate. `--dry-run` remains detection-only.
`--release`, `--no-scene`, and `--no-vacuum` cannot combine with place.

Every move waits for PlanMotion success before advancing. A failed carry move
stops with exit 10 and keeps vacuum enabled. Release must succeed and DI0 must
stay low for 0.5 s within 2 s (existing driver flags configure these budgets)
before detaching the collision payload and retreating. Release, detach, or
retreat failure returns 11. Place mode does not install automatic vacuum
release on exit or Ctrl-C; a fault requires explicit operator recovery.

The policy owns target and cargo-path validity. PlanMotion checks the current
MoveIt scene with the measured attached payload. This adapter does not add
cargo occupancy, infer that a container is empty, verify final box pose, or
commit planned geometry into a packing map. Success means the requested
motion/release/retreat sequence completed; it is not physical placement
verification or a full production orchestration cycle. After release the
existing scene adapter removes the pickup collision object; subsequent box
cycles require a measured scene/map update by the caller.
