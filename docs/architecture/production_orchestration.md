# Production Orchestration

This document defines the production loading-cell orchestration contract. The
same state machine and exploration contracts apply in simulation and on
hardware. Adapters may differ by backend; authorization, state progression, and
online map inputs may not.

## Explicit site commissioning profile

`hardware_loading_console.py` is a separate `site_commissioning_v1` adapter for
operator-supervised commissioning of the existing single-box driver. It is not
the production orchestrator and must say so in its UI and documentation. It
does not report reset/exploration, live packing, commit, or verification success.
Startup is inert; explicit initialization authorizes a MoveIt move to the
configured observation TCP pose. Alternatively an explicit correlated
`skip_initialize` checks readiness and an empty tool without motion, detection,
or vacuum commands and waits for a separate pickup confirmation at the current
pose. It does not assert that the configured observe pose was reached; later
cycles still return to that configured pose. Each pickup confirmation authorizes
one detect/pick/place/release/exit/return cycle and also confirms the reviewed
scene and previous placement. Only settled return re-enables confirmation.
Any failure latches; no automatic retry, target reuse, or vacuum cleanup.

Two explicit commissioning recovery commands are authorized only with the
current request ID, schema version and scene confirmation, while no cycle is
running. `recover` uses the existing executor connection to stop and establish
StandBy (resetting a genuine controller fault or enabling a disabled group as
needed), without vacuum or return motion. It clears the application fault only
after positive feedback and requires a new initialization/reset afterward.
`reset` additionally authorizes release/drop of a held payload. It requires new
post-command empty-tool and pump-off feedback, removes only the pickup
attachment, and plans a return to the profile observation pose. It never marks
an unknown drop as a placement or deletes existing placed-box obstacles.
The operator reviews the drop area and return path; the dropped box is not
measured cargo occupancy. Pending successful placements count only after
confirmed return, including after controller-only recovery.

Its frozen site target list is planning intent. Released payload dimensions
come from pickup perception; their target-position collision proxies are
explicitly expected placements, retained for collision checking only, never
published as measured cargo occupancy or a successful placement verification.
The operator must stop on a mismatch and correct the scene before proceeding.
This profile does not relax the production cycle below or qualify as production
acceptance. A future production adapter must use measured exploration and
commit/verification and the production reducer.

## Startup Boundary

`WAIT_START` is the initial state. Launch completion, lifecycle activation,
service readiness, sensor readiness, timers, and ordinary `run` / `step`
commands must not authorize robot motion, vacuum changes, or detection.

The only initial authorization event is an explicit operator `start` command.
After it is accepted, the orchestrator resets the cargo map and performs the
initial container exploration before prompting for a placed box.

## Operator Request Correlation

The orchestrator publishes each operator prompt with a generated request ID,
prompt kind, current state, and display text. Prompt messages must not contain
box geometry, pose, identity, dimensions, or simulation truth.

An operator pickup-ready event authorizes one cycle only when it carries the
exact current request ID and the expected schema version. Missing, blank,
stale, duplicate, malformed, or wrong-ID confirmations must not emit motion or
detection effects.

## Pure State Contract

The state reducer is a ROS-free contract. It may consume plain decoded events
and return declarative effects, but it must not call ROS, sleep, read YAML,
lookup TF, inspect Gazebo, or execute motion.

Effects describe work for a node layer, for example `CALL_ACTION`,
`CALL_SERVICE`, `PUBLISH_PROMPT`, and `PUBLISH_STATUS`. Effects that may move
the robot, change vacuum, or trigger detection must mark that capability
explicitly so tests can assert startup and request-correlation safety.

## Required Cycle

The required nominal progression is:

```text
WAIT_START
  -> RESET_CARGO_MAP
  -> EXPLORE_CONTAINER
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
  -> DETECT
  -> COMPUTE_PLACEMENT
  -> PLAN_PICK
  -> EXEC_PICK
  -> PLAN_PLACE
  -> EXEC_PLACE
  -> COMMIT_AND_VERIFY
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
```

Every return to `WAIT_PICKUP_READY` must happen after reaching
`pick_observe_pose`. The cargo map persists across placed boxes and is updated
only from live depth integration and explicit placement commits.

## Failure Boundaries

Failure handling must distinguish pre-pick, carrying, release, verification,
abort, and recovery boundaries.

Before payload attachment, failures return to a non-moving wait or explicit
retry/recovery boundary. While carrying, failures transition to a carry fault
and preserve vacuum until an explicit recovery confirms payload release.
Placement commits are idempotent and must not double-count attempts.

## Exploration Boundary

Exploration policy contracts are immutable, serializable, ROS-free plain data.
Policies may propose or score candidate views, but they must not execute
motion, mutate the map, read TF, or receive ground truth.

Exploration integration must correlate the settled view request ID, source
acquisition stamp, expected map revision, and container geometry hash. Stale,
ambiguous, repeated, or mismatched view integration fails closed.

Simulation evaluation may read Gazebo truth only offline or in test/eval scope.
The online production path must not branch on `sim_mode`, consume GT, or use a
fake production success path. Drivers that call production services
(`DetectLuggage`, `ComputePlacement`, cargo-map commit, MoveIt) are in that
online path even when they live under `luggage_gazebo`. Catalog size,
`GetCurrentBox`, and unverified planned poses are GT for this rule. The
defect catalog is [real_scenario.md](real_scenario.md).
