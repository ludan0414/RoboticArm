# 真机操作：Orin＋笔记本，人工上箱，逐箱 pick / place / 返回

适用：当前仓库的 Elfin S20、D555、Mid-360 双机部署。
这里的 Orin 就是讨论中说的 Orion。命令在各机器的 Linux Bash 中执行；
笔记本运行 ROS 2 Jazzy，Orin 运行 ROS 2 Humble，不能在同一终端混 source。
当前 Windows PowerShell 不是这些 ROS 命令的执行环境。

## 先明确现在能做到哪里

目前双机启动、真机感知、单箱 pick，以及 `--place-target` 单箱放置接口已有代码。
**“人工放好箱子 → 点确认 → 自动 pick/place → 自动退舱并回等待位 → 再等确认”
还没有被当前入口完整串起来。** 本文把可执行命令和待补流程分别标明。
本次仅整理文档，未连接设备、未运行硬件命令，也没有验证当前现场网络和标定。

Newton 演示的 `run_ground_greedy10_demo.py` 已加入逐箱 **Start Pick** 按钮：
箱子出现后等待，点击才继续 observe/pick/place/返回，下一箱再次等待。
它用于演示人工授权节奏；这个按钮没有连接真机 ROS 图，不能据此认为下面的
真机人工确认和返回流程已经完成集成。

| 能力 | 当前状态 |
|---|---|
| Orin 采集 D555 / Mid-360，预处理、YOLO、箱子检测 | 已有双机入口 |
| 笔记本 TF、MoveIt、规划、CPS 执行、吸盘控制 | 已有双机入口 |
| 单箱抓取 | `run_floor_box_pick.sh execute`；不带放置目标时抓完保持吸附 |
| 单箱抓取＋放置＋松吸＋放置点上方退让 | `execute --place-target target.json`；已有接口，需现场验收 |
| 整个箱单自动逐项送给真机 | 当前驱动不读取整个列表，每次只加载一个目标 JSON |
| 人工上箱后的确认按钮、请求编号、防重复点击 | 现有生产状态契约有定义，这个单箱脚本入口尚未接入 |
| 放完后自动沿出口退舱，再返回固定等待位 | 当前 place 只到 `place_retreat`，尚未接入完整返回段 |
| 已放箱子实测验证、更新货舱地图、推进下一箱 | 单箱 place 接口不做，需要外层流程接入 |

Newton 的 10 箱 demo 已打通的是仿真准备和回放；不能直接把它的轨迹 NPZ
发送给真机，也不能把仿真世界坐标、供料坐标、关节复位姿态直接当作现场数据。

## 开哪些窗口

**最少三个终端：Orin 一个，笔记本两个。** 不需要为每个 ROS 节点另开窗口。

| 机器 / 窗口 | 做什么 | 使用期间 |
|---|---|---|
| Orin O1：感知 | D555、Mid-360、预处理、YOLO、检测服务 | 一直运行 |
| 笔记本 L1：机器人后台 | 唯一 CPS 执行器、真实关节状态、场景 TF、MoveIt、规划、吸盘 | 一直运行 |
| 笔记本 L2：操作台 | 检查状态、预览、发起一个箱子的任务；以后放确认按钮的地方 | 每箱用一次 |
| 笔记本 RViz（可选 GUI） | 查看机器人、TF、场景 | L1 改成 `use_rviz:=true` 即可自动打开 |

录包可以用后面的 `record_floor_box_pick.sh` 替换 L2 的驱动命令，不必另开终端。
若从笔记本 SSH 到 Orin，O1 可以是笔记本上的 SSH 窗口，但程序仍在 Orin 执行。

```text
Orin O1：相机 / 雷达 → 检测箱子 ───────────┐
                                       ↓ DDS，ROS_DOMAIN_ID=7
笔记本 L2：人工确认 → 单箱任务 → 笔记本 L1：MoveIt → CPS → 机械臂 / 吸盘
                                       ↑
                         真机关节状态、TF、动作结果
```

## 首次现场配置

下面命令中的 `/ABS/RoboticArm` 是该机器实际仓库根目录，`/ABS/site/` 是现场配置目录，
执行前替换。仓库记录过的笔记本路径为 `/home/adamliao/work/RoboticArm-master`，
Orin 的路径以现场为准。两机路径可以不同，标定内容和消息接口必须匹配。

1. 两机均已构建当前代码；Orin 的 Humble 感知 overlay、CUDA YOLO、D555 DDS 库、
   Livox overlay 要就绪，笔记本的 Jazzy 主工作空间和 `deployment_ws` overlay 要就绪。
   更新本次 pick/place 代码后，笔记本需在 Jazzy 环境重新构建 `luggage_planning`。
2. 两机使用 `ROS_DOMAIN_ID=7`，允许跨机 DDS 通信、时钟同步；
   检查检测服务、消息类型与 TF 时间戳能跨机使用。相同 domain 数字本身不是通信成功证明。
3. `/ABS/site/scene_tf.yaml` 必须是实测容器、机器人、相机外参等配置；两机使用一致的现场版本。
   `/ABS/site/robot_poses.site.yaml` 必须是笔记本的实测命名姿态文件。
4. 核实机械臂 IP/端口、现场连线、吸盘 DO/DI，以及 README 中的 Gate 0/1。
   仓库默认控制器是 `192.168.0.10:10003`，以现场配置为准。
   首次动作需要有人在急停旁。`check_site.py` 的旧 D435/无 GPU 检查不是 Orin 感知的完整验收。
5. 选定物理上料区域和机械臂等待位；等待位应使机械臂和工具让开人工上料区域。
   等待位、抓取位、放置退让位是不同的位姿，不要混用。

目前约定的上料区域是现场照片里机械臂右侧、银灰色行李箱所在的地面，
水平位置按演示画面纠正后采用**机械臂基座坐标 `x=+1.2 m、y=0`**，箱底贴实际地面。
这里“地面”不是基座坐标的 `z=0`（基座在机柜上）；本轮检测仍提供实际箱体位置和高度。
Newton 当前对应的 world 水平位置约为 `(0,+1.2)`，不能将它和现场 world 直接混用。

当前配置 `src/luggage_description/config/robot_poses.site.yaml` 的 `poses` 为空，
默认 `current` 表示“保持调用时所在的位置”，**不是保存过的原位，也不会返回上一轮起点**。

记录等待位时，先由现场操作人员将空工具移动到确认过的等待姿态，再在 L2 执行：

```bash
cd /ABS/RoboticArm/deployment_ws
source scripts/env_jazzy_real.sh
./scripts/dump_observe_pose.sh pickup_observe
```

将输出按其结构合并到实测 `robot_poses.site.yaml` 的 `poses` / `defaults` 中。
该命令只打印当前真实关节，不会移动机械臂，也不会自动写入配置。
不要复制 `robot_poses.yaml.example` 的仿真 `pickup_observe` 数值。
以下启动命令显式传入现场文件，避免落到安装目录的示例配置。

## 启动三个终端

### O1：在 Orin 启动感知

```bash
cd /ABS/RoboticArm/deployment_ws
export ROS_DOMAIN_ID=7
./scripts/orin_perception.sh \
  scene_tf_config:=/ABS/site/scene_tf.yaml
```

保持运行。脚本负责 Humble 环境、CUDA 检查和感知 launch；默认启动 D555 和 Livox。
此终端不运行 MoveIt 或 CPS。不要再单独启动第二个相机、YOLO 或 Livox 驱动。

### L1：在笔记本启动机器人后台

```bash
cd /ABS/RoboticArm/deployment_ws
export ROS_DOMAIN_ID=7
source scripts/env_jazzy_real.sh
./scripts/hardware_pick.sh \
  start_d555:=false start_perception:=false \
  start_planning:=true start_executor:=true start_scene:=true \
  use_moveit:=true use_rviz:=false execution_backend:=waypoint \
  scene_tf_config:=/ABS/site/scene_tf.yaml \
  robot_poses_config:=/ABS/site/robot_poses.site.yaml \
  robot_ip:=192.168.0.10 robot_port:=10003
```

保持运行。该命令已经包含执行器、硬件场景、规划和吸盘节点；
不要再叠加运行 `jazzy_real.launch.py` 或另一个 `hardware_pick.sh`。
不启动 Gazebo、假关节发布器或 EtherCAT 控制栈。

这里采用现有部署基线的 waypoint 后端。`hardware_pick_servo_j.sh` 是另一个
显式选择的测试后端，不是需要多开的窗口；如现场另有已验收的 ServoJ 配置，
应停掉 L1 后再切换，不能同时运行。当前 S20 不用 ServoEsJ。

### L2：在笔记本检查和操作

```bash
cd /ABS/RoboticArm/deployment_ws
export ROS_DOMAIN_ID=7
source scripts/env_jazzy_real.sh

ros2 topic info /joint_states --verbose
ros2 action list -t
ros2 service list -t
ros2 run luggage_planning hardware_pick_driver.py --detect-only --observe-pose current
```

应看到唯一真机 `/joint_states` 发布源、
`/elfin_arm_controller/follow_joint_trajectory`、`/motion_planner/plan_motion`、
`/motion_planner/go_to_robot_pose`、`/luggage_detector/detect_luggage` 和 `/vacuum/command`。
检测调用还应实际返回有效箱体和候选，不以“节点都在列表中”代替数据就绪。
`current` 在这里避免检查时额外转动；固定等待位的返回需要后面定义的实际动作。

## 当前可以执行的单箱操作

下列命令是现有接口，不等于下一节尚待实现的自动人工确认循环。
目标文件需由现场坐标转换后的 policy/贪心结果提供，不能使用文档里的示例数值当目标。

```bash
# L2：选定本箱目标，预览抓取与放置目标，不执行搬运。
TARGET=/ABS/site/targets/box_001_target.json
./scripts/run_floor_box_pick.sh plan --place-target "$TARGET"

# 人工箱子放好、退出操作区域并确认后，再手动发出这一条。
# execute 本身就会进入执行流程，不要假定它还会弹出第二次 yes 确认。
mkdir -p "$HOME/robotarm_runs/box_001"
./scripts/run_floor_box_pick.sh execute \
  --place-target "$TARGET" \
  --trace-out "$HOME/robotarm_runs/box_001/pick_place.jsonl"
```

`plan` 只是目标预览，不是完整 IK/碰撞验收。`execute --place-target` 成功结束时
工具只到放置位置的上方退让点，**没有保证已经退出货舱，也没有回到上料等待位**。
所以当前不能紧接着让人上第二个箱子，更不能用 shell 的 `for` 循环跑十次。

需要同时录包时，在同一个 L2 用下面命令替换上面的 `execute`，不要两者都启动：

```bash
./scripts/record_floor_box_pick.sh execute --place-target "$TARGET" \
  --trace-out "$HOME/robotarm_runs/box_001/pick_place.jsonl"
```

它启动 bag、执行一次驱动并在结束时停 bag；终端会打印 evidence 目录。
JSON 的 world 箱子中心、xyzw 姿态和可选 `via_box_poses` 见
[放置接口](policy_place.md)。驱动不直接接受整个 Newton `plan.json`。

## 要实现的人工上箱循环

这是本次确定的目标流程，**尚无一条现成命令可启动整段循环**。
沿用 [生产编排契约](../../docs/architecture/production_orchestration.md)：
初始 `WAIT_START` 必须等操作员开始，再完成货舱状态初始化/探索和回到等待位；
仅仅启动上述节点不能授权机械臂自动开始一个箱子。

```text
WAIT_START：操作员开始 → 初始化/探索 → 回到实测上料等待位
  → WAIT_PICKUP_READY：提示“请放第 N 箱”，显示本箱编号
  → 人工放箱、退出区域、点“已放好，执行本箱”
  → DETECT：测量本次箱子，校验身份/尺寸与本项计划的匹配
  → COMPUTE_PLACEMENT / VALIDATE_TARGET：校验本箱目标和当前货舱状态
  → PICK：接近、吸附确认、抬起
  → PLACE：携箱入舱、下降、松吸、确认释放
  → COMMIT_AND_VERIFY：实测确认放置结果并更新货舱状态
  → EXIT_CONTAINER：空工具沿已验证的路径退到舱外
  → RETURN_PICK_OBSERVE：回到记录的实测等待位，确认实际到位
  → WAIT_PICKUP_READY：此时才提示下一箱，等待新的人工确认
最后一箱也要返回等待位，再显示 COMPLETE；不再要求放下一箱。
```

“复位”就是 `EXIT_CONTAINER + RETURN_PICK_OBSERVE` 的实际运动。
不重启节点、不清空已经装箱的货舱地图、不把关节数值改写成初始值。
箱子由人逐个放到实测上料区域，不在真机模式下自动“刷出”；位置和姿态以本轮检测为准。

| 约束 | 确认/返回的具体行为 |
|---|---|
| 一次确认只执行一个箱子 | 提示带本轮 `request_id`、箱号和 schema 版本，确认必须匹配当前请求 |
| 防重复/提前点击 | 只在 `WAIT_PICKUP_READY` 接受确认；运动中点击、旧请求、重复请求均不排队、不再执行 |
| 人工确认的界限 | 确认授权这一轮；箱体检测、吸附、路径、释放检查仍必须通过 |
| 等待状态不搬运 | 未确认时不发起本轮 DetectLuggage 请求或抓放动作，不自动推进到下一项；后台相机/YOLO继续运行 |
| 必须真的完成 | 每段等动作成功结果；不能靠 `idle`、目标被接受或固定延时进入下一段 |
| 返回原位 | 返回到固定实测关节目标并核对实际关节；`current`、`place_retreat` 或打印 COMPLETE 均不能代替 |
| 安全退出路径 | 先空手退到舱外，再回等待位；返回路径应考虑容器和已放箱子，不能从深处直接关节插值回家 |
| 承载/释放/返回失败 | 转入明确故障状态，禁止提示下一箱；承载故障保持吸附，恢复需操作员明确处理 |
| 状态可见 | 显示 `第 N/总数箱、箱号、当前阶段、等待人工/执行中/故障`，保存本轮目标、动作和结果 |

已有 `GoToRobotPose` action 可读取命名关节姿态，但当前实现直接构造关节轨迹，
不是一条经过 MoveIt 避障的完整退舱路径。接入返回阶段时需要先规划并验证退出和回程，
不能只在 place 后拼一个命名姿态调用就宣称循环已完成。

外层编排需补的连接点是：人工确认请求/按钮、每轮目标加载、实测结果与货舱状态更新、
空手退出/返回、返回到位后才发下一轮请求。现有单箱 pick/place 接口可以作为其中的执行模块。
离线列表用于顺序和目标建议；现场测量/货舱状态不满足时应停在重新规划或恢复边界，
不能以离线计划的位置冒充实际装箱结果。

## 故障和结束

携箱故障时不要盲目 `--release` 或重跑同一箱；place 模式保留吸附用于明确恢复。
`Ctrl+C` 停 launch 不等于安全释放，也不等于已回等待位，不能把它用作“复位”按钮。
正常结束应先完成释放、退舱、返回和确认，再停 L1，最后停 O1；故障按现场急停和恢复流程处理。

代码依据：双机入口见 [Orin 部署基线](../../docs/hardware/2026-09-22_orin_deploy_baseline.md)，
启动和旧检查见 [deployment README](../README.md)，目标/失败返回码见 [policy place](policy_place.md)。
