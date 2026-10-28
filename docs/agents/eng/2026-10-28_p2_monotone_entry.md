# 2026-10-28 — P1/P2 direct transfer and P2 entry

- role: eng
- agent: codex
- base revision: f330fa2
- status: implementation built; live attached-box validation pending

The `loading_093` trace showed an RRTConnect detour on P1→P2: J6 travelled
about 212° for a 64° endpoint change, while J3/J4 also reversed. The old
outside waypoint made the first pre-place connection move from about world
X=-0.44 to X=-0.32, away from the container. P1→P2 now uses a monotone
one-degree-sampled joint line and checks each state with MoveIt's live
`/check_state_validity`; any invalid state blocks execution. Resumption must
start on that line. Placement route generation starts from the measured box
pose at P2, first centres at the same door-outside depth with 1cm advance,
then proceeds into the container. Previous doorway strategy checkpoints are
versioned out by the v3 strategy name.

Focused offline tests passed before the operator requested immediate build.
`colcon build --packages-select luggage_planning --symlink-install` completed.
No live motion was commanded by this agent. The check-state-validity probe of
the running scene had no attached box, so it cannot certify the loaded route.
