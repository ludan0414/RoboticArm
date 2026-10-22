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
根据本次约定，默认值假设真机布局与当前 Newton demo 一致，已经填好十箱配置，没有 null：
上料点是基座坐标 `(1.2, 0)`，对应 world 约 `(0, 1.2)`，箱底 world `z=0`；
观察位 world `z≈0.86`，与基座同高。姿态、十箱放置目标和途经点来自当前 demo 记录。
`assumptions` 保存来源和哈希，明确这是布局假设而非现场标定记录。
本配置需与 `policy/demo_data/ground_greedy10_observe/scene_tf.yaml` 的 world/基座/航空箱布局一致。
沿用该布局启动 O1、L1 时，把其 `scene_tf_config` 参数设为各机器仓库中的
`/ABS/RoboticArm/policy/demo_data/ground_greedy10_observe/scene_tf.yaml`。
它只导出位姿供真机重新规划，不直接回放仿真关节轨迹。

- `observe_tool_pose`：工具 `suction_contact_frame` 在现场 `world` 中的等待/观察位，
  单位 m、姿态 xyzw。应能看见地面上料区，并让开人工上料空间。
- `cycles`：按执行顺序每箱一项，默认使用 demo 的贪心十箱结果；启动时冻结整个列表，
  不在线计算或重读目标。若现场布局不同，先把位姿转换到实际 world。
  `expected_demo_input_dims_m` 仅记录 demo 箱子尺寸，不替代本轮视觉测量或身份校验。
- `place_target`：箱子中心目标，遵循 [放置接口](policy_place.md)。
  `via_box_poses` 是携箱入舱路径中的箱中心位姿。
- `exit_tool_poses`：松吸并上退之后，空工具依次经过的退舱位姿。
  **这里是工具位姿**，不是箱中心。至少一项，每段要求完整笛卡尔路径；
  不允许部分执行或 OMPL 回退。最后自动经 MoveIt 回到 `observe_tool_pose`。

上料坐标是人工放箱区域的约定；抓取仍使用本轮视觉检测，不按该坐标盲抓。
初始化也通过 MoveIt 运动到等待位，不使用直接插值的命名关节姿态。
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

`cp` 在首次配置时执行；已有自定义现场配置时保留它，不重复覆盖。
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
2. 检查现场、勾选确认并点“初始化”；归到实测观察位后进入 WAIT_PICKUP_READY。
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

实体急停是停止手段；网页关闭、Ctrl-C 和 HTTP 均不是急停。
中断执行中的控制台不会追加归位或松吸；需现场明确恢复，保留 journal、trace。
此调试入口不自动推进感知 tracker 实例生命周期；检测仍返回旧实例时，
应先按现场感知流程更新。现有身份校验仍生效。
