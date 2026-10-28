本轮 Pick 参数、真空连接诊断及空载复位见 [Pick 恢复说明](docs/pick_recovery.md)。

# 实机代码 — Real-robot runtime (`deployment_ws`)

当前 Orin（Humble 感知）＋笔记本（Jazzy 规划/执行）的逐窗口启动、人工上箱确认、
单箱 pick/place 和返回等待位的流程，见专门的
**[真机操作文档](docs/real_robot_runbook.md)**。
现已增加 [真机按钮操作台](docs/loading_console.md)：逐箱点击 Pick，自动 place、
退舱、归位；默认采用用户提供的 `config/scene_tf.loading_site.yaml`，
名义上料基座坐标 `(-1.6, 0)`（面对航空箱的右手边），
属于尚待真机验收的调试入口。
L2 初始化、自动返回和掉箱复位共用示教器照片中的固定关节角：
J1～J6 `188.299, -135.510, -95.754, -36.132, 91.924, 12.500` 度。
通过 MoveIt 关节目标规划，不做 TCP/相机方向 IK，详见 [复位说明](docs/pick_recovery.md)。
接触抓取仍由原视觉 Pick 决定。L2 支持 `--start-box N` 从配置第N箱开始（1起算）；
会话 JSONL 和同箱重试明细允许覆盖，不再报 `File exists`。
携箱按长方体前后对称选择近侧等价朝向，避免多余180°接驳旋转；按政策长短边
换向时最多90°偏航，并保留配置占用空间及实测抓取偏心。
当前默认配置为**四色四箱高位搬运路线**，黑→红→蓝→灰；碰撞箱体和放置规划统一
使用本轮配置尺寸，视觉提供抓取顶面/位置/朝向。原视觉 Pick 后按实测抓取变换生成
外侧抬起、收臂、箱底高于底座顶面 15cm 的接驳，再自动 place/归位。
携箱过程中额外检查膨胀 5cm 的底座；MoveIt 仍使用实际机械臂和已附着箱体重新规划，
门框另有至少5cm净空：操作台每段运动前安装并读回四个门框保护体，确认携箱模型
仍附着；按现场要求，不再额外写入或读回校验ACM碰撞权限，沿用MoveIt原有
几何碰撞检查。5cm版轨迹重新
规划，旧profile缺少doorway_clearance时会拒绝启动；需同时更新配置和规划代码。
不直接执行 Newton 关节记录。按操作台的颜色/尺寸提示上料，四箱结束显示 COMPLETE。
新八箱方案目前仅有[不可运行的候审草案](docs/measured8_status.md)，尚未替换真机默认 profile。
实机入口默认使用 ServoJ（20ms 周期，lookahead 100ms），复用现有执行后端；
同一携箱阶段和空载反向回程连续通过全部路点，保留下降/松吸/上退停止边界。
WebUI 可在停稳后应用10–100%速度倍率，下一动作生效，仍遵守原有限速和20deg/s²保护。
L2默认连续录制实际关节、电流、吸盘和控制器错误，生成独立的
`<journal>.telemetry.<session>.jsonl`及源码包；详见操作台文档的“默认连续录制”。
需要兼容旧执行方式时显式传 `execution_backend:=waypoint`。
下文保留早期单机与底层检查说明；当前双机操作优先从上述文档开始。

`scripts/hardware_pick.sh`（含 ServoJ 包装入口）现在通过 Linux 监督进程启动：
同一用户在本机只能运行一套 pick 后台，跨 checkout、跨 ROS domain 也不能重复启动。
启动前检查即将启用的本地节点及旧 launch；发现残留时列出 PID/PPID 并拒绝启动，
不会自动杀其他终端的进程。`start_perception:=false` 等关闭的角色不参与节点冲突检查。
Ctrl+C、终端断开、SIGTERM 或 launch 自行退出时，监督进程依次用 SIGINT（15 秒）、
SIGTERM（5 秒）、必要时 SIGKILL（2 秒）清理本次进程树，并回收被收养的孤儿节点。
正常退出应打印 `cleanup complete: no owned child processes remain`。
强制杀死监督进程或断电仍可能留下进程；下次启动会检查并拒绝叠加。
遇到拒绝启动，先回原终端正常停止旧后台；若为历史孤儿，核对打印的 PID 对应本次现场后，
逐个 `kill -INT <PID>` 并复查退出，勿用宽泛的 `pkill -f ros`。
这些措施处理进程残留和重复实例导致的资源占用，不代表已经诊断出节点内部的内存泄漏。

无需 ROS/机械臂的生命周期回归：Linux 上运行
`python3 -m unittest discover -s deployment_ws/test -p test_hardware_pick_supervisor.py -v`。

**实机代码 (site / real-cell).** This tree is not part of the Gazebo
simulation stack. Do not treat it as a replacement for
`src/luggage_perception` canonical preprocessor configs.

The site D555 launch (host-stamp JPEG/PNG) and
`src/luggage_perception/config/preprocessor_d555_live.yaml` are two
preprocessor setups. Live pick A/B is:

- **A** `preprocessor_d555_live.yaml` — motion gate on, pair 5 ms
- **B** `preprocessor_d555_site.yaml` — humble site thresholds, wall clock

Do not pass `preprocessor_d555_replay.yaml` into a live pick graph
(`use_sim_time` is true; bag replay only).

No-GPU cell procedure: [`docs/status/site_no_gpu_verify.md`](../docs/status/site_no_gpu_verify.md).
`hardware_pick.sh` defaults `semantic_device:=cpu` when `nvidia-smi` fails.
Mid-360 overlay is required: `./scripts/setup_livox_driver.sh` then `source livox_ws/env.sh`
(gitignored local build; `record_site.sh` / `hardware_pick.sh` refuse to start without it).

TCP execution for the real Elfin lives here, as a sibling colcon
workspace next to the luggage stack (`src/`). Build it from
`deployment_ws/` so site CPS/D555 nodes stay separate from simulation.

| Path | ROS | Build | Role |
|---|---|---|---|
| `src/elfin_trajectory_executor` | Humble or Jazzy | `colcon` | `FollowJointTrajectory` via Huayan CPS |
| `noetic/elfin_cps_executor` | Noetic | catkin in Docker | Same TCP path, plus `/execute_trajectory` |

The Noetic package is not under `src/` so `colcon build` does not touch it.
The Noetic container bind-mounts it to `/catkin_ws/src/elfin_cps_executor`.

The laptop executor runs against **Jazzy** (`/opt/ros/jazzy`). Orin perception
uses Humble in its own host environment. Do not source Humble (or a Humble
colcon overlay) in a Jazzy shell.

Site worksheet: copy [`config/site_vars.yaml.example`](config/site_vars.yaml.example)
to `config/site_vars.yaml` and fill measured IPs / geometry.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
# FollowJointTrajectory lives here; required on a Jazzy-only host:
#   sudo apt install ros-jazzy-control-msgs
cd deployment_ws
python3 scripts/check_site.py --init
python3 scripts/check_site.py          # Gate 0: NIC + ping
eval "$(python3 scripts/check_site.py --export)"
python3 scripts/check_gate1.py         # CPS import + TCP port
bash scripts/run_gate1_sim_handshake.sh  # 2° FJT + READY_FOR_NEXT, no Gazebo

colcon build --packages-select elfin_trajectory_executor
source install/setup.bash

# Real arm: no Gazebo. Only after ping succeeds and a person is on e-stop.
ros2 launch elfin_trajectory_executor jazzy_real.launch.py

# Other terminal, same ROS_DOMAIN_ID. Blocks until the arm finishes.
ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
```

Hardware TF (after Gate 1 `/joint_states` is live):

```bash
ros2 launch luggage_description scene_hardware.launch.py \
  scene_tf_config:=/abs/path/to/measured/scene_tf.yaml
```

Mid-360:

```bash
./scripts/setup_livox_driver.sh
source livox_ws/env.sh   # overlay + Livox-SDK2 rpath
python3 scripts/check_gate2.py
ros2 launch luggage_description mid360.launch.py \
  user_config_path:=/abs/path/to/MID360s_config.json
```

D435 + weights (Gate 3):

```bash
bash ../src/luggage_perception/scripts/download_yolo_world.sh
bash ../src/luggage_perception/scripts/setup_clip_vendor.sh
python3 scripts/check_gate3.py
ros2 launch luggage_perception realsense_d435.launch.py
ros2 run luggage_perception dump_camera_frames.py --out ~/robotarm_site_frames
```

`real.launch.py` is the same graph as `jazzy_real.launch.py`.

## Jazzy vs Humble

Do not mix their overlays in one shell or process:

- The original Jazzy laptop had no `/opt/ros/humble`. The luggage stack was written for
  Humble (Python 3.10, Gazebo Fortress). Jazzy is Python 3.12 and Harmonic.
- Do not `source /opt/ros/humble` then overlay Jazzy, or the reverse.
- Do not run `luggage_gazebo/sim_world.launch.py` on Jazzy expecting the
  Humble Fortress launch to work.
- The current Orin Humble perception + laptop Jazzy execution deployment
  intentionally shares `ROS_DOMAIN_ID=7` with matched interfaces, as recorded
  in the [dual-host baseline](../docs/hardware/2026-09-22_orin_deploy_baseline.md).
  Unrelated simulation or legacy test graphs must stay isolated; do not
  apply a different domain to one half of this paired hardware graph.
- The executor package itself is distro-light (`rclpy` + `control_msgs`)
  and is the Jazzy real-robot target. Full luggage closed-loop on Jazzy
  lives on `origin/ros2_jazzy` (`elfin_jazzy_ws`), not this launch.

## Execution handshake

For policy-driven placement after pick, see
[`docs/policy_place.md`](docs/policy_place.md). The existing floor-box driver
accepts `--place-target target.json` for a world-frame box-centre position and
quaternion, including optional policy via poses.

The planner/test client **must wait for the FollowJointTrajectory result**.
That result is the only "you may start the next step" signal.

```text
client                         executor
  |-- send_goal -------------->|
  |<-- accepted ---------------|
  |<-- feedback (joints) ------|   (motion in progress)
  |<-- result SUCCEEDED -------|   ready_for_next
  |-- next goal -------------->|
```

| Channel | Type | Use |
|---|---|---|
| `/elfin_arm_controller/follow_joint_trajectory` | action | **Sequencing.** Next step only if status is SUCCEEDED and `error_code` is SUCCESSFUL. |
| `/trajectory_executor/events` | `std_msgs/String` JSON, transient-local | Observers. `ready_for_next` is true only on `"event":"succeeded"`. |
| `/trajectory_executor/status` | `idle` / `executing` / `error` | Debug. Do **not** sequence on `idle`. |
| `/joint_states` | 100 Hz | FK / RViz. Not a completion signal. |

`send_joint_trajectory` prints `READY_FOR_NEXT` only after a successful
result. Cancel, abort, timeout, or reject must not continue the pipeline.

Do not start Gazebo, `zero_joint_state_publisher`, or EtherCAT together
with this executor: they publish competing `/joint_states`.

## Site blockers on this host (2026-09-02)

`scripts/check_site.py` last report: `enp0s31f6` is DOWN, ICMP to
`192.168.0.10` fails, no D435 on USB, no NVIDIA GPU. TCP `192.168.0.10:10003`
answered once — treat that as untrusted until wired ping works. Do not send
the hardware 2° FJT until ping is green and someone is on the e-stop.

Livox needs a second NIC on `192.168.1.0/24` and the sensor serial suffix in
`MID360s_config.json`. YOLO/CLIP weights are on disk; semantic overlay uses
`device: cpu` until `nvidia-smi` works.
