# Pick 配置、真空 IO 与显式恢复

本次沿用原 PickSession 的检测、接近、吸附和抬起，再接既有 carry/place。
四箱顺序保持。名义箱位基座 x=-1.6m；初始化/复位/每箱返回位置在原观察位的底座
坐标上增加 `(0.8, 0, -0.2)m`，相机 optical +Z 光轴朝底座 -X，图像上方朝底座 +Z。

## 停止并恢复待机

L2 网页「停止并恢复待机（不松吸、不回位）」在等待和 FAULT 时可用；BUSY
时拒绝，先取消并等当前动作退出。请求由 L1 的 `/elfin/recover_standby`
（`std_srvs/Trigger`）处理，复用现有 CPS 连接，与轨迹和真空 DO 命令互斥。

恢复按真实 FSM 执行：Moving=25、点动=26、停止中=27、保持=32 可停止并等待；
碰撞停止=21、错误=22 在这次明确授权下各最多尝试一次 GrpReset；未使能=24
最多使能一次。8 秒内需要持续确认 StandBy=33 至少 0.1 秒，命令返回 0 不代表
状态已到位。急停、未知 FSM、自由驱动和脚本状态 34..38 拒绝自动恢复。
恢复不发送 InitServoEsJ、不清空真空、不执行轨迹；异常仍需要现场处理，不承诺
所有控制器故障都能免重启恢复。常规停止也不再暗中发送 GrpReset。

恢复成功清除软件当前错误、保留历史故障和箱序，回到 WAIT_START，需要重新选择
初始化/跳过初始化。若已有放置成功但归位未完成，只允许继续恢复或复位，禁止重新 Pick。
页面把历史 HRIF 记录与当前 FSM 分开显示；超过 2 秒未收到状态时显示过期。

## 松吸掉箱并回位

「松吸掉箱并回位（保留箱序）」明确允许当前箱子掉落。检查掉落区域和回位路径，
勾选现场确认后点击。顺序为：恢复待机 → VacuumCommand 关闭吸盘 → 等待新反馈
（命令之后收到、未过期、attached=false、vacuum_on=false、DI0=0）→ 解除 pickup
携箱模型并恢复碰撞权限 → 反向经过本轮成功的搬运停留点 → Pick 抬起后的停留位
→ 相机观察位。回程段仍要求完整碰撞检查，任一步失败均停止后续动作。
没有本轮 Pick 记录时先经过配置的原观察等待位。回程进度只在该段完成后消耗。

自动 place 结束、初始化和“松吸掉箱并回位”共用同一个归位函数。
`return_pose_adjustment.target_base_m` 是绝对底座目标，默认 `[-1.1,0,-0.2]m`。
由实际 `world ← elfin_base_link` TF 转成 world，完全独立于原名义 `observe_tool_pose`；
旧的相对 `offset_base_m` 不再决定操作台终点。相机方向仍使用实际
`suction_contact_frame ← d555_color_optical_frame` 固定 TF，光轴朝底座 -X。
缺少 TF 时拒绝移动；网页显示底座目标及解析后的 world XYZ。
`loading_066` 中，旧偏移使四箱终点变成 `[-0.4,0,-0.2]`，位于底座内部；
它和旧单箱 `[-0.8,0,-0.2]` 不是同一点。当前工具朝向下，离线模型中的更内侧候选
仍有前臂/手腕碰撞；`[-1.1,0,-0.2]` 找到无静态世界碰撞的 IK 分支。
这只是离线候选检查，完整路径仍须当前 MoveIt 场景验证。

携箱去程最多8个搬运停靠段，下降、松吸和上退保持独立。
回程最多7段加最终归位，共8次停靠。原路线中间点仍作为 Cartesian waypoints
连续经过，每段要求100%路径，不允许 OMPL 绕过中间点。
失败段不会消耗回程进度；重试仍保留该段全部中间点。
复位不会创建预定放置位的箱体代理，不清除已放箱子的障碍或舱内占位，也不清零箱序。
掉落后箱子的位置未测量；场景不会自动获得它的新位置，不应据此推断回位路径无碰撞。

- Pick/place 失败：复位后仍等待同一箱，不能猜测已完成放置。
- Pick/place 已成功，仅空载归位失败：成功复位后该箱只计数一次，等待下一箱。
- 如果箱子实际已放入，但上层未收到完整成功结果，先人工处理该箱与现场状态。
  重启 console 会新建会话、从第一箱开始，不能当作中途续跑。

## 箱体朝向与末段进场

此调试流程统一使用当前四箱配置，顺序黑、红、蓝、灰。视觉提供抓取顶面和朝向；
MoveIt 箱体尺寸、携箱检查和预期占位统一用 `expected_demo_input_dims_m`，
箱中心按视觉顶面减去配置箱高的一半推算，不修改原始检测消息。
封吸成功、携箱抬起前记录该箱体位姿和新鲜工具 TF，冻结 `T_box_tool`。
放置和底座净空检查共用这个变换，不再把抓取前请求的工具姿态当成实际姿态。
携箱接驳段不再强制改写 IK 结果的 J6；箱体目标仍对齐航空箱坐标轴，保留容器实测倾角。
松吸前用当前工具 TF 和冻结变换估算箱体位姿，要求位置误差 ≤1cm、姿态误差 ≤2°。
不满足时保留吸附并报错。这是刚性抓持估计，不是再次视觉测量，不能检测抓持后的滑移。

`loading_064` 的 place_via_44 失败发生在松吸之前。离线 URDF 对照中旧目标接近
肘部伸直，只能带姿态残差逼近。最后距落位 XY 30cm 的路径后缀现在平滑下降到
pre_place 高度；远处高位搬运、XY、箱体朝向保持。松吸目标沿箱体上方向留 5mm
支撑间隙，避免舱底 STL 起伏引发接触碰撞。完整 Cartesian
比例和碰撞检查不放宽。离线模型不能排除原现场碰撞等其他原因，仍需真机验收。

每箱 trace 增加 `sealed_grasp`、`motion_result`、`pre_release_pose`；L1 脚本默认将
规划边界记录保存到 `docs/status/evidence/hardware_motion/<启动时间_pid>/`，包括失败
起始关节和场景摘要。复现失败前先保留该目录，不覆盖本轮证据。

`loading_065` 报告视觉箱高 24cm，但旧配置按 20cm 计算放置中心。
离线 FCL 使用该报告尺寸复现出距原目标约 1.9cm 处的箱体/舱底碰撞，吻合
87.1% 的下降失败边界；这不证明实体箱子真高 24cm，也未独立验证视觉支撑面。
此处按操作员明确要求统一配置尺寸，避免在一条搬运链中混用两种箱高。

## 共享配置

源文件为 `src/luggage_description/config/pick_runtime.site.yaml`。
可复制到两机各自的 `~/site/pick_runtime.site.yaml`，保持内容一致。

视觉 `component.*` 参数现在通过 ROS 参数读取：区域腐蚀、深度带、相邻深度跳变、
覆盖率和像素数等可从现场 YAML 配置。仓库默认值保持原值，覆盖率仍为 0.35，
腐蚀仍为 bbox 5px / mask 3px；吸附接触判定保持原实现。
`support_mode: auto` 仍要求测量支持面。看不到地面/支持面并没有被改成猜高度授权抓取。
识别不完整时日志打印 COMPONENT 原因、有效阈值和 diagnostics，区分区域覆盖不足与
吸盘局部接触失败，不能据此声称已提高现场成功率。

真空配置包括 DI0 最大数据年龄、服务发现次数、发现/RPC 超时、吸附和释放等待时间。
DO 仍经持有 CPS 连接的 executor 服务发送，不另开争抢控制器的 TCP 连接。
只在尚未发送命令的服务发现阶段重试；RPC 超时后物理执行结果未知，不自动重发。
默认 DI0/state 有效期 2 秒，发现最多 2 次，RPC 2 秒；驱动命令等待 45 秒（覆盖发现、吸附超时后的释放序列及响应余量）。
松吸只有新鲜 DI0 确认低电平才返回成功，失败不会冒充已松吸。

## 更新与启动

先停止本轮动作，按现场流程关闭旧 O1/L1/L2。两机拉取相同提交。
在仓库根目录、各自正确 ROS 环境和既有依赖 overlay 已加载的终端构建：

Orin（Humble）：
```bash
colcon build --symlink-install --packages-select luggage_description luggage_perception
source install/setup.bash
```

笔记本（Jazzy）：
```bash
colcon build --symlink-install --packages-select luggage_description luggage_perception luggage_planning
source install/setup.bash
```

保持 [真机运行手册](real_robot_runbook.md) 中 O1/L1 的原有启动参数，额外追加 launch 参数：
```text
pick_runtime_config:=/绝对路径/site/pick_runtime.site.yaml
```
不追加时自动使用安装后的默认配置。L2 使用：
```bash
cd ~/RoboticArm/deployment_ws
mkdir -p ~/site/runs
bash scripts/run_loading_console.sh \
  --site-profile "$PWD/config/loading_cycle.site.template.json" \
  --runtime-config "$HOME/site/pick_runtime.site.yaml" \
  --journal "$HOME/site/runs/loading_$(date +%Y%m%d_%H%M%S).jsonl"
```
若未复制自定义 YAML，省略 `--runtime-config`。本次修改需要重新启动 O1/L1/L2；
只换放置 JSON 时才是仅重启 L2。配置不是运行时热更新。

## 日志

- O1 detector 启动打印实际 component 参数；拒绝日志含 reason/thresholds/diagnostics，2 秒节流。
- L1 `vacuum_io` 日志含 DO 位、值、发现次数、结果和耗时；真空命令开始/结束及健康变化也打印。
- L2 `pick runtime` 打印配置路径和超时；`detect attempt`、`pick_event`、`loading_event`
  记录检测耗时、阶段、错误及复位前真空状态。journal 和每箱独立 trace 保留故障记录。
- `VACUUM_IO_STALE` 表示缺少新鲜 DI0；`SERVICE_UNAVAILABLE` 是服务发现失败；
  `CALL_TIMEOUT` 是结果未知；`RELEASE_UNCONFIRMED` 是松吸后未确认低电平。

本机仅完成无 ROS 的故障注入/状态机检查，未执行真机或跨机 ROS 验收。
