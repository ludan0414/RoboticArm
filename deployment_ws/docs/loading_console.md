# 真机按钮操作台（现场调试模式）

保持 Orin 感知、笔记本机器人后台两个终端运行，第三个终端启动常驻操作台。
浏览器每次按“开始 Pick”，执行一箱：观察/检测 → pick → place → 松吸确认 →
放置点上退 → 沿现场路径退舱 → 回到实测观察位 → 等下一次点击。
最后一箱也归位，然后显示 COMPLETE。无需逐箱重启脚本或单独运行 place。

这是 `site_commissioning_v1` 调试入口，不是完整生产编排：没有自动扫舱、
实测放置验收、货舱地图提交和在线生成下一箱目标。
操作员每次勾选确认，包含检查上一箱放置、当前场景和上料区域。
已放箱子保留“目标位置 + 抓取时实测尺寸”的**预期碰撞占位**供 MoveIt 避障；
它不写入实测货舱地图，也不能证明实际箱子没有偏移。放偏时停止并修正场景，
不能继续沿用占位。此入口尚未经过真机现场验收。

## 现场配置

复制 [默认配置](../config/loading_cycle.site.template.json) 到 `/ABS/site/loading_cycle.json`。
坐标依据是用户提供的 `deployment_ws/config/scene_tf.loading_site.yaml`：
底座 yaw `-90°`，面对航空箱时基座 `+X` 向左、`+Y` 向后、`+Z` 向上。
名义右手边上料点为**基座 `(-1.6, 0)`**，对应 world 约 `(0, 1.6)`；箱底 world `z=0`。
本次部署默认采用**四色四箱**贪心路线，源包为
`policy/demo_data/site_four_colors_right160_observe120/`。顺序是黑色 67×43×26、红色 59×37×24、
蓝色 55×36×23、灰色 55×36×23 cm。观察位 world `z≈0.86`，与基座同高。
**箱子上料点与 EEF 等待位分开**：箱子仍在基座 `(-1.6, 0)` 地面；
EEF 等待/观察位在基座 `(-1.2, 0, 0)`，对应 world 约 `(0, 1.2, 0.86)`。
空载归位直接返回该观察位，不再绕到箱子上料点。实际接触点由原视觉 Pick 决定。
上料点允许偏移，继续使用原视觉 Pick。成功后保持吸附，按实际抓取变换生成
外侧抬起 → 在底座外侧收臂 → 抬至箱底高于底座顶面 15cm → 已审核的高位入舱路线。
附加 5cm 底座箱体净空检查；实际箱体尺寸、抓取偏心和姿态都来自本次检测。
新箱型在原供料位附近较低的路径前缀由高位接驳绕过：先横向到安全入口上方，再垂直下降接原入舱轨迹。
15cm 高度用于横向接驳；下降和入舱仍保留底座 5cm 余量及完整场景碰撞检查。
每箱完成后归位，操作台显示下一箱的颜色与尺寸；四箱完成后显示 COMPLETE。
不自动重复同一放置目标，避免堆叠覆盖。
航空箱的平移和完整 roll/pitch/yaw 均来自该 YAML，不能再使用此前底座 `+90°` 的配置。
`assumptions` 保存来源和哈希，并不意味着执行过现场验收。
O1 的 `orin_perception.sh` 和 L1 的 `hardware_pick.sh` 默认加载上述文件；
如果显式传 `scene_tf_config`，两机都应指向各自仓库里的这个文件。
它只导出位姿供真机重新规划，不直接回放仿真关节轨迹。

- `observe_tool_pose`：工具 `suction_contact_frame` 在现场 `world` 中的等待/观察位，
  单位 m、姿态 xyzw。应能看见地面上料区，并让开人工上料空间。
- `cycles`：按执行顺序每箱一项，当前默认四箱；启动时冻结整个列表，
  不在线计算或重读目标。若现场布局不同，先把位姿转换到实际 world。
  `expected_demo_input_dims_m` 仅记录 demo 箱子尺寸，不替代本轮视觉测量或身份校验。
  `box_label` 标识本次应上料的实体箱子；必须按页面顺序上料，程序没有自动颜色身份识别。
- `place_target`：箱子中心目标，遵循 [放置接口](policy_place.md)。
  `via_box_poses` 是携箱入舱路径中的箱中心位姿。
- `exit_tool_poses`：松吸并上退之后，空工具依次经过的退舱位姿。
  **这里是工具位姿**，不是箱中心。至少一项，每段要求完整笛卡尔路径；
  不允许部分执行或 OMPL 回退。最后自动经 MoveIt 回到 `observe_tool_pose`。

上料坐标是人工放箱区域的约定；抓取仍使用本轮视觉检测，不按该坐标盲抓。
初始化现在可选：点“初始化归位”才通过 MoveIt 运动到配置的观察位；
也可以点“跳过初始化，从当前位置开始”，只检查后台、场景和空工具状态，
不发送运动、检测或真空命令。跳过后仍需单独点击 Pick，第一箱从当前姿态观察/抓取；
之后每箱完成的自动回程仍回到配置的观察位，不会省略。
初次运行前需现场检查场景、工具、路径和可达性。初始化不会清空场景或已放箱子占位；
重新开启会话前需核对留存场景与实际货舱。

## 笔记本 L2 命令

先按 [真机操作文档](real_robot_runbook.md) 启动 O1、L1。笔记本在 Jazzy 环境重新构建：

```bash
cd /ABS/RoboticArm
source deployment_ws/scripts/env_jazzy_real.sh
colcon build --symlink-install --packages-select luggage_planning
```

在 L2 启动：

```bash
cd /ABS/RoboticArm/deployment_ws
mkdir -p /ABS/site/runs
cp config/loading_cycle.site.template.json /ABS/site/loading_cycle.json
bash scripts/run_loading_console.sh \
  --site-profile /ABS/site/loading_cycle.json \
  --journal /ABS/site/runs/loading_001.jsonl
```

本次必须更新旧路线：将新模板覆盖到实际使用的 profile 路径，重启 L2。
只 `git pull` 不会更新仓库外已经复制的 `/ABS/site/loading_cycle.json`。
确认 JSON 有 `carry_entry.strategy=outside_retract_high_lift_v1`、名义 feeder `[-1.6,0]`、
`cycles` 只有一项。此前十箱 profile 不会自动换成新路线。
同时重启 O1、L1 加载新 YAML；只更新 L2 不能修正旧机器人 TF。
需要重新从同一 demo 导出默认配置时，在仓库根目录用 Newton Python 执行：

```bash
python policy/export_hardware_loading_profile.py \
  --output /ABS/site/loading_cycle_from_demo.json
```

`runs` 目录需存在；journal 必须为新文件，已有文件会拒绝启动，避免覆盖故障记录或
重启后默默重放原箱单。配置快照、哈希、确认和阶段存入该文件；
每箱另存 `<journal>.box001.jsonl` 抓取/放置明细。

在笔记本浏览器打开 `http://127.0.0.1:8765`：

1. 启动为 WAIT_START，不触发检测、运动或吸盘操作。
   可先勾选确认并点“坐标方向测试”：按基座 `+X/-X/+Y/-Y/+Z/-Z` 各走 2cm，
   每个方向后返回点击时的工具位姿，保持姿态。全程空工具、吸盘关闭、完整 Cartesian 路径，
   不做箱子检测或 Pick。每段验证到位后才继续；失败锁定，不盲目追加复位动作。
   在 WAIT_PICKUP_READY 也能执行此可选测试。完成后仍需人工观察确认方向并单独授权 Pick。
2. 检查现场并勾选确认，选择“初始化归位（可选）”或“跳过初始化，从当前位置开始”；
   对应动作/检查成功后进入 WAIT_PICKUP_READY。
3. 人工上箱、退出运动范围，检查并重新勾选，点“开始 Pick”。
4. 执行中按钮禁用，显示 DETECT、PICK_ATTACH、PLACE_DESCEND、PLACE_RETREAT、
   EXIT_AND_RETURN、RETURN_OBSERVE 等阶段。
5. 成功归位后产生新请求，等待下一箱。网页刷新不会重发 Pick。

每次确认带 request_id 和 schema_version；忙碌、重复、过期请求不排队。
控制台仅绑定本机，关闭网页不会停止后台动作。
不要同时启动旧的单箱执行命令或其他运动操作台。

## 故障

检测、吸附、携箱、释放、碰撞场景更新、退舱、归位任一失败均锁定 FAULT。
不自动下一箱、重试或在携箱故障时松吸；没有网页故障重置。
释放未确认不接着退舱；空工具/真空关闭未确认不初始化或开始下一次 Pick。
动作超时请求取消，但取消请求不证明硬件已停。

新 console 在初始化、方向测试和每段运动前校验实际 base/container TF 与 profile，
拦截旧 `+90°` 底座或其他场景不一致。`(-1.6,0)` 仅为名义上料点，不要求精确摆放；
不再使用旧 profile 的 20cm 半径或地面高度偏差拦截。实际箱位、候选抓取、可达性和吸附由原视觉 Pick 负责，
原 Pick 的接近、吸附、抬起及失败反退不套用 console 的固定短段限制。
成功 Pick 后保持吸附，从新鲜实测 TCP 接至首个 carry 目标；接驳分成不超过 15cm/20° 的段，
逐段携箱碰撞检查并要求完整 Cartesian 路径，失败停止并保持吸附，不强行接入。
后续 place、退出及最终观察位保持不变。普通段位移/转角启发式现在只告警、不拦停；
模板设 `enforce_segment_heuristics=false`，旧 profile 没有该字段时也默认只告警。
初始化和最终归位交给 MoveIt 自由空间规划，不再用 TCP 直线距离上限拦停。
原 45° 单段关节转幅限制已在真机 launch 中关闭（`max_segment_joint_excursion_rad=0`）；
原 `3 rad/m` 笛卡尔转动比拒绝分支已经删除，只记录诊断/告警；
即使旧 launch 仍传 `cartesian_excursion_rad_per_m=3`，更新后的执行器也不会因比值超出而中断。
这两项需要重新构建并重启笔记本 L1，运行中的 executor 不会因 `ros2 param set` 自动重建。
工具 TF 仍必须为最近 2s 数据。MoveIt 机械限位、碰撞、腕部翻转检查、失吸停机和释放确认保留；
硬件 launch 的速度/加速度比例降至 15%/10%。这些限制不替代现场方向观察和实体急停。

实体急停是停止手段；网页关闭、Ctrl-C 和 HTTP 均不是急停。

默认执行后端已改为 ServoJ：使用已有 StartServo/PushServoJ，20ms 周期、100ms lookahead。
这需要更新并重启 L1；旧窗口仍会继续用启动时的后端。需要旧后端时显式传
`execution_backend:=waypoint`。位置/姿态仍由 MoveIt 规划，ServoJ 负责平滑发送关节轨迹。
搬运 via/pre-place 可从失败的 Cartesian 规划回退到完整 MoveIt 终点规划；最终下降仍要求
fraction=1.0，并在失败时尝试更细采样，不执行部分路径、不提前松吸。
中断执行中的控制台不会追加归位或松吸；需现场明确恢复，保留 journal、trace。
此调试入口不自动推进感知 tracker 实例生命周期；检测仍返回旧实例时，
应先按现场感知流程更新。现有身份校验仍生效。
