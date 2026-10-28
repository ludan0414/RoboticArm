# 八箱真机配置与验证范围

默认现场 profile `deployment_ws/config/loading_cycle.site.template.json` 现在包含八箱，
顺序：红、银、蓝、粉、小灰、青铜白、黑、灰。blackwhite 与 blackbronze 是两只
独立箱子，均未选入这次方案。三堆分别为红/银/蓝、粉/小灰、青铜白/黑/灰。

舱内源包 `policy/demo_data/measured8_spaced_inside_full/` 的八箱抓取、携箱、
放置和仿真原路退回已完成逐帧碰撞检查。真机不播放这些关节帧，而用每箱尺寸、
目标和携箱箱体位姿实时规划；Pick 仍由视觉决定。放置后沿本轮完成的携箱路点
反向至 Pick 抬起停留位，再规划到示教器固定关节位。反向笛卡尔路径不完整或
碰撞受阻时停止，不执行部分路径或重新规划绕路；未完成路点保留。

**地面上料接驳未全部仿真验证。** 地面基座坐标 `(-1.6, 0)` 的桥接检查已完成
前三箱，第四箱停在退回阶段，余下未运行。用户要求赶紧录制，按现场指示假定接驳
可行。profile 用既有现场高位接驳与 MoveIt 实时碰撞检查，并明确记录
`ground_entry_simulation_validated=false`、`ground_entry_assumed_by_operator=true`、
`hardware_motion_validated=false`；不要把它说成八箱地面接驳或真机已验证。

堆间最小间距约 10 cm、6 cm，但第三堆底箱紧贴平底/斜面交界，**该边界没有余量**。
现场尺寸、摆放、吸附和标定误差可能改变间隙，实际环境及碰撞几何须现场核对。
候审草案 `config/loading_cycle.measured8.review.json` 保留供对照，并非运行 profile。

拉取后重新构建受影响 ROS 包并重启后台和操作台；如果之前把 JSON 复制到仓库外，
还须更新操作台实际读取的那份 profile。完整窗口命令见 [操作台文档](loading_console.md)。
