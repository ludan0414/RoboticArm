# P1→P2 scene-check bypass

Revision: follows c00b27f on greedy. The live UI reported `CARRY_JOINT_LINE: state 6/66 invalid (site_door_clearance_right/pickup_box)` while carrying at P1. The operator explicitly authorized the taught P1→P2 leg without collision checks.

The P2 executor now sends the same monotone sampled joint line without `/check_state_validity` or occupancy checks. The console bypasses doorway scene guarding for that segment. Start-on-line, wrist branch and wrist singularity checks remain. P1 and all later planned segments retain their existing scene checks. This removes the reported software collision gate; physical clearance remains operator-assessed and has not been demonstrated by this change.
