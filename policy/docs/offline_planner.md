# 从箱子清单离线生成轨迹

入口是 `policy/plan_boxes.py`。使用确定性的贪心装箱、数值 IK 和双向 RRT
（固定随机种子 7），不调用大模型、网络服务或训练好的 policy。箱子名字只作标识，
不参与路线特判。P1/P2、场景、机械臂模型属于现场标定配置。

## 输入和运行

从仓库根目录，在已安装 Newton 的环境中运行：

```powershell
conda activate newton-rope
python policy/plan_boxes.py --input policy/demo_inputs/offline_one_box.json --output policy/demo_data/my_run
```

输出目录必须尚不存在。Windows 若 Warp 的缓存路径过长，可以先设置
`$env:WARP_CACHE_PATH='C:/warp_cache_roboticarm'`。

输入例子（长、宽、高，单位厘米）：

```json
[
  {"name": "silver", "size_cm": [56, 36, 21]},
  {"name": "blackbronze", "size_cm": [75, 49, 29]}
]
```

也可用 `{"boxes": [...]}`；单项可换成 `size_m`，但不能同时提供两种单位。
名字必须唯一。`--max-layers` 默认 3；`--scene-config`、`--template`、
`--seed-domain` 可替换现场配置和预探索域。候选位置按支撑、层数、两侧优先的规则排序，
逐项检查机械臂、载荷、已放箱子和舱体的碰撞以及关节分支，再搜索路线。
这是有预算的启发式搜索，不保证最优装箱，也不能把搜索失败解释为数学上的不可达证明。

只有所有输入箱子都规划、逐帧检查成功，才会生成 `hardware.json` 和 `READY.json`。
失败保留 `failure.json`、`manifest.json` 和候选路线的诊断文件，供离线排查。

## 可视化与输出

```powershell
cd policy
python replay_prepared_demo.py --bundle demo_data/my_run
```

现成的一箱成功样例：

```powershell
python replay_prepared_demo.py --bundle demo_data/offline_sample55
```

`plan.json` 保存顺序、位置、姿态；`waypoints.json` 保存冻结的箱体路径；
`trajectory.npz` 是 Newton 的关节与场景回放；`validation.json` 保存检查结果；
`manifest.json` 保存输入、代码指纹、环境和运行耗时。READY 对可回放和部署文件校验哈希。

## 与真机共用的范围

本入口从**在示教 P1 已经吸住箱子**开始，验证 P1→P2、入舱放置、原路退出并返回 P1。
名义抓取为箱顶中心、工具向下；释放后的 5mm 落位按运动学假设处理。
没有模拟视觉识别、地面 Pick、实际吸附动力学或最终回 home。这些继续使用原有真机代码。

冻结 waypoint 来自 Newton 检查通过的实际箱体采样；真机根据实际测得的抓取偏移转换为
工具位姿。仿真和真机消费者的名义 waypoint 有直接数值对比测试。
实际抓取偏移、IK 求解、控制器执行和现场误差仍可能不同，不能据此声称真机运动已验收。

部署时，将生成的整个目录复制到现场，更新并重新编译 `luggage_planning`，
重启笔记本规划端与 console；在原有 console 启动命令中，将
`--site-profile` 指向生成的 `hardware.json` 的绝对路径。原有 Pick 流程及按钮操作不变。
新 profile 类型为 `site_offline_frozen_v1`，旧版 console 会拒绝它，防止静默使用旧规划逻辑。
规划端也必须更新，以支持 `offline_frozen_cartesian`。

此模式不再按旧逻辑改朝向、降低目标或重新生成入舱点。现场 Cartesian 检查若无法完整通过，
以 `OFFLINE_ROUTE_RECHECK_REQUIRED` 停止并保留诊断，不执行部分路径或自动换路线。
生成器不会自动改变现场的 `software_collision_checks` 等调试配置；现场的实际配置仍需对应。

默认八箱现场 profile 和已保存的 c7af59d 版本保持原路径逻辑；新入口仅在显式选择生成的
profile 时启用。已验证样例为 55×35×20cm 单箱，253 帧；这不代表任意箱子组合都已通过。
