# 2026-10-28 — Site P1/P2 carry transfer

- role: eng
- agent: codex
- model: gpt-6
- base revision: 8d1e5e8, with pre-existing uncommitted site changes preserved
- status: implemented; live robot validation pending

## Decision and implementation

For every commissioned box, successful sealed Pick retreat is followed by the
operator's photographed J1..J6 P1 then P2. MotionExecutor sends each as a
MoveGroup joint goal with the attached configured box and scene; no pose IK or
Cartesian partial-prefix execution is used for these taught targets. The old
pedestal/high-lift connector is skipped. Place planning starts only after P2
settles, its actual joints match the target within 0.02 rad, and fresh tool TF
is read. The return breadcrumb includes actual P1/P2 tool poses.

An interrupted transfer has its own durable phase and next-target index.
Resumption repeats the unfinished target, then builds place from actual TF.
Failed transfer retains vacuum. Legacy ACM permissions for attached-box contact
with arm links 5/6 are explicitly cleared; only suction hardware is allowed to
touch the box. Failure to update those permissions aborts before Pick motion.

## Evidence and limits

Focused tests cover exact signed angles, order, P1 failure blocking P2,
checkpoint continuation from P2, pressure gate, measured-TF placement, and
joint-arrival rejection of a ±2π equivalent. All luggage_planning tests passed
under sourced Jazzy environment: 685 passed. `colcon build --packages-up-to
luggage_planning --symlink-install` completed (8 packages). See
`docs/status/evidence/manual_carry_p1_p2_20261028/summary.md`.

No robot was moved in this task, and no live MoveIt scene was available for
eight physical box trajectories. Therefore real collision clearance and
reliability remain not evaluated; the runtime gate must reject an infeasible
P1/P2 path rather than claim it is clear.
