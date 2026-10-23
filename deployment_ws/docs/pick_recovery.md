# Pick 配置、真空 IO 与空载复位

本次沿用原 PickSession 的检测、接近、吸附和抬起，再接既有 carry/place。
没有改四箱顺序、携箱路线或观察位。名义箱位基座 x=-1.6m，最终观察 EEF x=-1.2m。

## 一键空载复位

L2 网页新增「空载复位到观察位（保留箱序）」，在等待和 FAULT 时可用。
先确认现场机械臂已经停止、工具空载，再勾选现场确认并点击。BUSY 时不能复位。

复位读取新鲜真空状态和 DI0；DI0 非 0、数据过期或 attached 未清除时，拒绝移动和松吸。
若工具已确认空载但泵仍开着，先发送关闭并确认，再由 MoveIt 从当前位置规划到 profile
中的 observe_tool_pose。不清除舱内占位、不删除箱序、不重复执行 Pick。
如仍携箱或状态未知，先现场处理；这个按钮不是急停、控制器清故障或强制释放。

- Pick/place 失败：复位后仍等待同一箱，不能猜测已完成放置。
- Pick/place 已成功，仅空载归位失败：成功复位后该箱只计数一次，等待下一箱。
- 如果箱子实际已放入，但上层未收到完整成功结果，先人工处理该箱与现场状态。
  重启 console 会新建会话、从第一箱开始，不能当作中途续跑。

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
