# Real Deployment, Pickup XY, And Smooth Control

Frozen laptop pick for the ThinkPad + Orin cell, 2026-09-22. Jazzy,
`ROS_DOMAIN_ID=7`. Orin owns the D555, Livox, and perception. The laptop
owns CPS, scene TF, planning, and the pick driver. Do not start a second
camera, Livox, or YOLO on the laptop.

## Laptop nodes

One graph. From `deployment_ws`, with a person on the e-stop:

```bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
./scripts/hardware_pick_servo_j.sh
```

That script forces `execution_backend:=servo_j`, `start_d555:=false`, and
`start_perception:=false`. Ctrl+C stops the launch. It does not BlackOut.

Nodes that start:

| Node | Role |
|---|---|
| `trajectory_executor` | Huayan CPS. `StartServo` then `InitServoEsJ`, then `PushServoJ` at 0.02 s. |
| `scene_hardware` | robot_state_publisher and container TF. |
| `scene_manager` | planning scene. |
| `waypoint_generator` | pick segment poses. |
| `motion_planner` | PlanMotion / ExecuteTrajectory. Holds J6 on pick segments. Times both ramps at 20 deg/s^2. |
| `vacuum_controller` | hardware vacuum. |
| `move_group` | MoveIt, because `use_moveit:=true`. |

Nodes that stay off on the laptop: D555, Livox, preprocessor,
`semantic_segmenter`, `semantic_point_filter`, `luggage_detector`.

Second terminal, same domain, after the graph is up:

```bash
source scripts/env_jazzy_real.sh
./scripts/run_floor_box_pick.sh plan
./scripts/run_floor_box_pick.sh execute
```

`plan` does not move the arm. `execute` runs the pick. Observe pose stays
`current` (no simulated `pickup_observe`).

Waypoint all-in-one, only when this laptop also owns the sensors:

```bash
./scripts/hardware_pick.sh
```

`record_perception_case.sh` starts a local D555, Livox, and perception.
Do not run it while Orin owns those devices. Do not `ros2 bag play` on
domain 7.

## What the laptop subscribes

Orin publishes raw images, not JPEG/PNG:

- `/camera/d555/color/image_raw` and `camera_info`
- `/camera/d555/aligned_depth_to_color/image_raw` and `camera_info`
- `/livox/lidar`, `/livox/imu`
- `/luggage/preprocessed/camera/{color,depth}/image` and camera_info
- `/luggage/semantic/yolo_detections`, overlay, mask, `cargo_points`
- service `/luggage_detector/detect_luggage`

The laptop publishes `/joint_states`, `/tf`, and `/tf_static` for Orin.

## Pick flow

1. Pendant in StandBy (FSM 33). Vacuum off.
2. Orin perception already up. Laptop graph as above.
3. `run_floor_box_pick.sh plan`, then `execute`.
4. Segments: `pre_grasp`, `approach`, `attach`, vacuum seal, `pick_retreat`.
   Pick segments pin `elfin_joint6` to the segment-start angle. Place
   segments do not.
5. The planner writes a rest-to-rest trapezoid at 60 deg/s and 20 deg/s^2
   for both accel and decel. The executor resamples that trajectory onto
   the 0.02 s ServoJ grid and refuses a steeper rise or drop.
6. After a `CARRY_FAULT`, vacuum stays on until an explicit release.
   Clear FSM 21/22 on the pendant before another `StartServo`.

`servo_esj` / `PushServoEsJ` is rejected on this S20 (20006/20007). Do not
use it.

## Problems hit on this cell

These are the failures from the live ServoJ pick, and what the code does
about them. The 20 deg/s^2 ramp has not been re-run on the arm since the
last edit; restart `hardware_pick_servo_j.sh` before the next pick so the
planner and executor load it.

| What happened | Cause | What the code does now |
|---|---|---|
| `ExecuteTrajectory -4` on the first `PushServoJ`, code 40071 | `StartServo` does not leave ServoEsJ initialized. The same 40071 came back on `approach` after `pre_grasp` and on `pick_retreat` after vacuum DO. Init-then-Start left the buffer empty; Stop on the first 40025 lost the session. | `StartServo`, then `InitServoEsJ`. A 40071 retries Init only (no Stop). If that still 40071, Stop, wait stable StandBy 33, then Start+Init and push the same point again. |
| `-4` near the end of `pre_grasp`, FSM 21 `RobotCollisionStop` | Braking a 60 deg/s^2 ServoJ stream. `PushServoJ` has no acceleration field, so a 20 ms step that dumps speed looks like a hit. | Planner and the pushed grid both cap accel and decel at 20 deg/s^2, including the first sample. |
| `pick_retreat` lifted too fast, then `-4` / `CARRY_FAULT` | Retreat leaves rest, so the fast part is acceleration. Only the brake had been lowered. Vacuum stayed on because retreat never finished. | The accel ramp is 20 deg/s^2 as well. Release vacuum and return the pendant to FSM 33 before retrying. |
| Waypoint `pre_grasp` crawled, then the driver timed out | MoveJ serialized small vias and waited for blending. Effective speed was about 1 deg/s. | The live pick uses ServoJ, not that waypoint chain. |
| `PLACE_PATH_EXCURSION` on approach, J6 several rad over a short path | J6 hold ran after the excursion check, and a short path uses a 0.05 m floor in the ratio. | J6 is pinned before the excursion check (pick approach and `place_pickup_connect_*`). |
| `DETECT_STALE_CLOUD` with a multi-second age | The detection-window stamp was frozen. A healthy cargo cloud lags about 0.8 s, not 9–60 s. | Not a laptop residual node. Do not raise the age limit to hide a wedged detector. |
| `DETECT_NO_SEALABLE_PATCH` | Dynamic top was valid and every suction cell failed coverage, boundary, flatness, or peak-to-valley. | Failure text lists the rejecting gates. The short-side size of the top hull still walks, so the contact center is the hull center, not a measured box center. |
| FastDDS `fastrtps_port7010` lock | Leftover `/dev/shm/fastrtps_*`. Traffic falls back to UDP. | Do not delete those files while any ROS process on this machine is up. |

## Split ownership

Both machines use native DDS on domain 7. One owner each for D555, Mid-360,
CPS, and scene TF.

Orin (Mid-360 + D555 raw 640×360@15 + YOLO + DetectLuggage):

```bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
~/ros2_ws/src/luggage_perception/scripts/perception_site.sh
```

The Orin segmenter is its own node: preprocessed colour in, mask / YOLO /
overlay / `stats_json` out. Hardware launch on the laptop does not start
`pickup_box_spawner`. Do not bind site DetectLuggage to a spawn id.

## Pickup XY Contract

Do not change `luggage_msgs/DetectedLuggage.msg` for pickup XY selection.
The selected suction contact XY is encoded in both:

- `DetectedLuggage.pose.position.x/y`
- `DetectedLuggage.top_surface_pose.position.x/y`

`top_surface_pose.position.z` remains the measured contact Z. Consumers should
continue to refuse motion when `top_surface_valid=false`.

## Offline Pickup Benchmark

Use real bags from `~/robotarm_bags` plus the preserved floor-pick bag. Manual
labels are the ground truth:

```json
{
  "labels": [
    {
      "bag_path": "/home/user/robotarm_bags/site_pick_001",
      "stamp": 1790000000.123,
      "frame_id": "site_pick_001@1790000000.123000000",
      "suction_safe_lid_center_pixel": [321, 188],
      "suction_safe_lid_center_world_xy": [0.482, -0.117],
      "lid_polygon_pixel": [[280, 150], [360, 148], [370, 230], [274, 226]]
    }
  ]
}
```

Candidate extraction should compare at least:

- `pca_center`: current live output.
- `yolo_bbox_center_top_plane`: YOLO bbox center projected to the top plane.
- `lid_inlier_center`: top-surface inlier center.
- `robust_blended_center`: robust blend selected after scoring.

Score labels and extracted candidates with:

```bash
python3 -m luggage_perception.eval.pickup_xy_benchmark \
  --labels labels.json \
  --candidates candidates.json \
  --strategy robust_blended_center \
  --output summary.json
```

Acceptance for the selected strategy:

- median world-XY error <= 0.030 m.
- P95 world-XY error <= 0.060 m.
- improves over `pca_center` on at least 80% of comparable labeled frames.

Outlier frames must keep replay artifacts under `docs/status/evidence/` before
rerunning the same gate.
