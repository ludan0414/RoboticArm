# Offline inventory planner evidence

Baseline: c7af59d. Implementation: e7c1492; integrated remote ae45b22 without replacing its live fixes.
Environment: Windows, Python from newton-rope, Newton/Warp CUDA device cuda:0.
Input: policy/demo_inputs/offline_one_box.json, sample55, 0.55 x 0.35 x 0.20 m.

Command from repository root:

```powershell
$env:WARP_CACHE_PATH='C:/warp_cache_roboticarm'
python policy/plan_boxes.py --input policy/demo_inputs/offline_one_box.json --output docs/status/evidence/2026-10-29-offline-entry/run10
```

Run10 passed in about108s,253 recorded frames. Classical deterministic greedy candidate selection plus
bidirectional RRT seed7; no LLM or trained policy invocation. Selected box yaw90deg automatically.
Checked nominal sealed P1→P2 transfer, placement, release, inverse retreat and inverse return to P1.
Each sampled motion state checks arm, payload, fixed geometry and prior cargo; joint edges are sampled,
not a continuous collision proof. Final box position error3.593e-8m, rotation error0rad.
Release settling5mm is a kinematic assumption. No perception, vacuum dynamics or hardware motion tested.

Packaged output: policy/demo_data/offline_sample55. Text normalized toLF, scene/model/profile and READY
hashes regenerated without changing geometry or motion. Both visible replay and final packaged headless
replay completed with1box/253frames, planning_calls0. Replay logs retained locally.

Tests use PYTHONPATH=policy;src/luggage_planning;src/luggage_description;src/luggage_perception;src/luggage_planning/test.
Command:

```powershell
python -m unittest test_offline_parity test_offline_entry test_frozen_route test_measured_luggage_policy test_hardware_loading_ports test_centered_doorway_entry test_ik_search test_place_cartesian_recovery test_reach_limit
```

Actual console nominal tool waypoints match Newton carry frames within2e-5m/2e-5rad.
Final integrated available suite:76tests passed in24.051s (tests-merged-available.log).
Tests reject altered/uncertified profiles and verify frozen Cartesian failure never executes a partial
prefix, projects target, changes release height or substitutes a live route. Existing runtime profiles
retain their previous behavior. Only1inventory workload tested; no packing success-rate claim.

Remote-added test_site_collision_override cannot run in this environment: pytest and ROS moveit_msgs
are unavailable. One expanded invocation therefore failed at importing that module after76 tests;
the available suite was rerun separately. No product failure was inferred from this dependency issue.

Development failed/interrupted attempts run01–06, raw request traces and bounded collision/IK rejection
artifacts are retained locally under this evidence directory. Initial portal search could not certify
the descent branch; replaced with release-seeded vertical approach and generic joint-space search.
Runs07–10 each passed the same single-box workload. These are development checks, not independent
reliability trials. User-owned packing_policy.py/online_packing_real.py edits were preserved and excluded.
