# P1/P2 transfer verification, 2026-10-28

Revision: base `8d1e5e8` plus dirty working tree; site profile unchanged.
Environment: Ubuntu/Jazzy workspace, Python 3.12, no robot motion or Gazebo.

Observations:

- Exact photographed six-axis targets are encoded for P1 and P2, in that order.
- Focused tests: 59 passed before final arrival gate; final broad test suite:
  685 passed in 2.96 s (Jazzy and local package paths sourced).
- `colcon build --packages-up-to luggage_planning --symlink-install`:
  8 packages finished successfully in 26.1 s.
- Final `colcon build --packages-select luggage_planning --symlink-install`:
  1 package finished successfully in 0.39 s.
- Legacy source-text-only fraction test was replaced with a behavioral test
  asserting that an incomplete Cartesian prefix is not executed.

Not evaluated: live MoveIt planning or execution with each of the eight
physical boxes, mounted camera/cables, movable door leaf, and measured pickup
pose. These inputs are needed to measure actual collision and success rate.
Each P1/P2 goal remains fail-closed with attached-payload collision planning,
joint-arrival verification, vacuum retention, and a resumable checkpoint.
