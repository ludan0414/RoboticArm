# Newton 集装箱粗仿真

使用 Newton Physics 1.3.0。内边长 3.2 m 的立方体集装箱有五面厚 2.5 cm 的静态碰撞壁，`-X` 面开口。场景外是深色地面，带 0.5 m 间距的机器人仿真风格网格线和红/绿坐标轴。小箱子为尺寸略有差别的动态长方体，按 4×4 列密堆，最多四层；同列箱子的底面投影由下方箱子完整覆盖。64 箱时初始体积利用率约 90.6%。初始垂向留 2 mm 数值间隙，重力使箱子落到支撑面；直接贴合初始化会让当前 XPBD 参数下的堆叠不稳定。默认 XPBD、60 Hz、每帧两个子步、五次迭代。

进入 `newton_container_demo` 目录并激活 Newton 环境；脚本默认读取仓库内的 `vendor/roboticarm` 资源，不需要外部 `RoboticArm` 路径：

下面命令假定当前目录就是 `newton_container_demo`；如果当前在仓库根目录，请先执行 `cd .\newton_container_demo`。

```powershell
conda activate newton-rope
python .\container_demo.py --mode bench --device cuda:0 --boxes 64 --frames 120
```

打开 Newton OpenGL 窗口：

```powershell
python .\container_demo.py --mode show --device cuda:0 --boxes 64 --frames 0
```

`--boxes` 支持 1–64，`--device cpu` 可测 CPU。JSON 输出中，`build_seconds` 是构建场景时间，`warmup_seconds` 包括首次内核编译，`frame_ms_*` 是每帧同步后的稳定耗时。Warp 缓存写在项目根目录 `.wp`，短路径可避免 Windows 的路径长度限制。

## 本机实测

RTX 5060 Laptop GPU，`cuda:0`，XPBD 5 次迭代、2 子步/帧；每组预热 10 帧、计时 120 帧。以下耗时只包含无界面物理仿真和每帧 GPU 同步，不含画面渲染。

| 箱数 | 体积占比 | 平均每帧 | 95% 帧耗时 | 相对 60 Hz 实时速度 |
|---:|---:|---:|---:|---:|
| 8 | 11.3% | 12.01 ms | 16.66 ms | 1.39× |
| 32 | 45.3% | 8.00 ms | 13.99 ms | 2.08× |
| 64 | 90.6% | 9.11 ms | 15.14 ms | 1.83× |

首次运行时内核编译使预热耗时约 65 秒，缓存建立后预热约 0.5–0.8 秒。不同箱数间的耗时差异会受 GPU 时钟与调度波动影响，不宜由此推断箱数增加会加速仿真。64 箱运行 120 帧后高度均为有限值，位置最大变化约 2.5 cm。脚本关闭了 PyOpenGL 的可选加速模块，绕开当前环境中它与 NumPy 的二进制版本不匹配问题。

上面是原始长方体粗仿真，仅衡量箱子与容器的刚体接触。

## RoboticArm Gazebo 场景移植

`roboticarm_newton.py` 读取仓库内 `vendor/roboticarm` 中的 S20 URDF/Xacro、CAD STL、场景 YAML 和 Gazebo 模型资源。场景包含六关节 Elfin S20、吸盘面板、末端相机安装件和雷达安装框、机器人台座、取件平台，以及 `airport_container_real`。机械臂和集装箱位姿都由场景 YAML 提供；代码不依赖外部 RoboticArm 仓库。集装箱使用完整的 `container_collision.stl` 和视觉 STL。仅对已固定连接的箱体与吸盘工具内部形状过滤接触，避免把刚性抓持接合面当作相互穿透。

关节 6 按 3 秒周期、约 ±40° 的正弦轨迹转动。当前 Newton XPBD 对此导入机械臂的关节目标未产生可见运动，因此演示用正向运动学逐帧施加关节位姿，再进行原始网格碰撞检测；`xpbd` 模式随后还运行接触求解。这是规定轨迹的演示，不是电机动力学或带碰撞停机的轨迹控制。

```powershell
python .\roboticarm_newton.py --mode inspect
python .\roboticarm_newton.py --mode bench --motion rotate --pipeline contact --frames 120
python .\roboticarm_newton.py --mode bench --motion rotate --pipeline xpbd --frames 120
python .\roboticarm_newton.py --mode show --motion rotate --pipeline contact --frames 0
```

本机 RTX 5060 Laptop GPU 上，场景构建（缓存命中）约 3.7–4.6 秒。60 Hz 目标步长，5 帧预热后测 120 帧，含每帧 GPU 同步，不含窗口渲染。`xpbd` 为每帧 2 子步、8 次迭代；`contact` 为每帧一次完整网格碰撞检测。

| 状态 | 管线 | 平均每帧 | 95% 帧耗时 | 约等于实时速度 |
|---|---|---:|---:|---:|
| 静置 | contact | 56.30 ms | 59.58 ms | 0.30× |
| 转动 | contact | 105.55 ms | 212.65 ms | 0.16× |
| 静置 | xpbd | 110.44 ms | 114.65 ms | 0.15× |
| 转动 | xpbd | 219.03 ms | 439.24 ms | 0.08× |

转动两秒后箱体相对起点约旋转 0.647 rad，刚体位姿均保持有限值。运动时接触组合变化会带来较大的帧耗时波动。首次运行有额外的 Warp 内核编译开销。

当前是机械场景与规定轨迹基线。ROS 控制器、相机和雷达数据流、真实吸盘抓取约束、碰撞时停止轨迹与装箱优化尚未移植；Newton 运行所需的 Gazebo 资产已随本仓库提供。

## 在线装箱算法演示

`online_packing_demo.py` 与 Newton、机械臂无关。它把容器简化为内尺寸 1.50 × 1.20 × 0.95 m、`x=0` 全面开口的长方体；每次仅接收当前箱子的尺寸，允许 6 种轴对齐旋转。候选必须无体积重叠、底面全支撑，并能沿 `+x` 从开口直线送入。没有直接位置时，最多取出一个顶面无载物且能沿入口移出的已有箱子，暂存在箱外；当前箱放入并把暂存箱重新放回后，才接收下一箱。失败时停止。这里的直线通道只是高层几何代理，不包含机械臂可达性。

```powershell
python .\online_packing_demo.py
```

固定随机种子 1、8、13，各生成最多 16 个箱子；输出 `online_packing_samples.json`，逐操作记录尺寸、左前下角坐标、取出和重新放回动作。每个动作都经回放验证：容器边界、碰撞、全支撑、入口通道，以及箱外暂存槽在事务结束时为空。三组分别成功放入 9、10、9 箱，触发 0、2、2 次二次搬移；遇到第一件无法在这些约束内解决的箱子时停止。这是可视化用的基线启发式，不代表全局最优空间利用率。

### 真实异形集装箱在线规划与 Newton 回放

`online_packing_real.py` 默认从仓库内 `vendor/roboticarm/.../scene_tf.yaml` 读取货舱内尺寸、斜切底角和矩形门洞，独立为每个到来的箱子重新求解位置、朝向和进出路径，输出 `online_packing_real_samples.json`。候选位置必须在异形货舱内、无重叠、底面全支撑；箱子按水平直线穿过门洞，进入后可沿横向和垂向移动。整段路径的扫掠长方体需避开已放箱子和货舱内壁。直接放不下时，最多暂取一个顶部无载物的旧箱；当前箱放入并把旧箱放回后，才接收下一箱。算法只知道当前箱的尺寸，不预知后续箱子。

规划器也可作为状态化在线接口使用：`OnlinePackingPolicy.step(dims)` 一次只读入当前箱尺寸，已确认箱子的状态由策略自己保存；`run_stream` 增量消费 iterable，不检查剩余输入。成功步骤立即提交；阻塞步骤不修改已提交状态。默认会打印每箱规划时间，样例 JSON 也记录 `planning_seconds`。信息流是在线的，但严格 IK 是同步的慢速规划阶段，机械臂执行应等待该箱规划返回后再开始。

初版贪心 baseline 用 `--policy baseline` 启用。箱体姿态只允许最大矩形面朝上/下，仍检查无碰撞、容器边界、全底面支撑和入口通道。评分最大化 `Σ(volume × normalized_center_depth)`，其中深度从门口 `x=0` 指向货舱内部；估值相同时优先更高支撑层、再靠左。搬移搜索只考虑无遮挡顶层箱；前方已有箱子挡住同一通道时冻结后方箱子；搬移还必须增大地板最大连续空矩形。搜索宽度为 2，候选去处最多 3 个。每个新箱最多搜索 2 次搬移，累计取出和放回路径长度不能超过 4 m。可用 `--relocation-depth` 和 `--relocation-distance-budget` 调整。

要让真实 S20 在预览中抓着箱子移动，规划本身必须带 IK 约束；纯几何 baseline 会优先把箱子送深，那里可能超出机械臂工作空间。启用 `--ik` 后每个搬运路径都会做 Newton IK 和碰撞检查，运行会明显变慢：

```powershell
python .\online_packing_real.py --policy baseline --ik --count 5 --seeds 8 --output .\online_packing_baseline_ik_samples.json
python .\online_packing_newton.py --samples .\online_packing_baseline_ik_samples.json --mode show --seed 8
```

```powershell
python .\online_packing_real.py --count 30
python .\online_packing_real.py --policy baseline --count 30 --seeds 1 --output .\online_packing_baseline_samples.json
```

`online_packing_newton.py` 读取规划数据并在 Newton ViewerGL 中逐帧回放。默认使用仓库内的 `scene_tf.yaml`，所有场景位姿从 YAML 读取；可用 `--container-y` 或 `--scene-config` 显式覆盖。它复用 `roboticarm_newton.py` 的 Gazebo 移植场景：完整的 `airport_container_real` 碰撞与视觉 STL、台座、取件平台和 S20 机械臂视觉/碰撞模型；机械臂上原有演示箱的视觉保持隐藏，避免与动画箱重复。

```powershell
python .\online_packing_newton.py --mode show --seed 1
python .\online_packing_newton.py --mode show --seed 13
```

`--mode inspect` 验证场景与动作序列；`--mode bench` 仅测逐帧位姿写入。固定随机种子 1、8、13，最多各尝试 30 箱；分别放入 17、16、12 箱，体积利用率 40.23%、39.00%、30.75%，二次搬移 3、0、0 次。默认种子 1 的时间轴为 2641 帧、约 44 秒。这里的路径是箱体轴向平移的保守高层几何规划；不搜索任意六维姿态，也不验证机械臂的抓取可达性。Newton 回放是规定轨迹动画，保留完整碰撞网格但不运行接触求解或碰撞停机。斜底上不能给水平箱底提供完整面支撑，因此当前策略不把箱子直接落在斜面上。

### IK 约束规划（同尺寸箱）

`online_packing_real.py --ik` 使用 Newton 的 `newton.ik.IKSolver` 对当前 deploy 场景重新筛选候选。默认箱体尺寸为 `0.55 × 0.40 × 0.25 m`，只允许平放的 0°/90° 水平旋转。每条箱体进出路径按 5 cm 采样；每个采样位姿都必须满足末端位置误差不超过 3 mm、姿态误差不超过 0.035 rad、六个关节限位，并且 Newton 碰撞检测中机械臂非基座连杆不能撞集装箱。箱体与门槛/地板的接触由 hull 的箱体扫掠几何检查处理。

```powershell
python .\online_packing_real.py --ik --count 5 --seeds 1 --output .\online_packing_ik_samples.json
python .\online_packing_newton.py --samples .\online_packing_ik_samples.json --mode show --seed 1
```

十个同尺寸箱子的回放数据保存在 `online_packing_ik_10_samples.json`，可用下面的命令重新生成和播放：

```powershell
python .\online_packing_real.py --ik --count 10 --seeds 1 --output .\online_packing_ik_10_samples.json
python .\online_packing_newton.py --samples .\online_packing_ik_10_samples.json --mode show --seed 1 --device cuda:0
```

当前 deploy 场景下十箱全部放入，二次搬移 0 次，体积利用率约 13.02%。最终有 3 层，按箱子下表面高度统计为 `z=0.00 m: 4`、`z=0.25 m: 4`、`z=0.50 m: 2`。水平搬运段会从至少 0.40 m 的 IK 可达高度进入，再对准目标 x/y 后一次垂直下降；这个高度可以大于 2 cm，落位间隙仍保持至少 2 cm。

这条模式会在 IK 和碰撞过滤后优先选择更深的可达位置，因此箱子按“里到外”顺序填充。箱子会保持安全高度直接对准目标 x/y，再从距离最终高度至少 `2 cm` 的位置垂直下降到支撑面；取出箱子时先抬高同样的最小间隙再向外移动。实际高度由门洞和障碍物决定，可以大于 `2 cm`。它保证的是当前 Newton 模型下的离散路径可行性，连续关节轨迹和真实控制器动态仍需后续用 MoveIt/现场规划器验证。

## 并行 Newton 仿真与轻量 RL pipeline

### Sequence-to-placement 推理接口

当前 policy 的上层抽象是：输入完整箱子序列的尺寸，输出一个可执行的重排放置计划。容器长度、宽度、高度、斜底、门洞和相对坐标都从仓库内的 `scene_tf.yaml` 先验读取；输出坐标是 `container_local` 局部坐标。每个输出项包含原输入下标、放置后的尺寸、箱子中心 `center_m`、绕 z 轴的平放方向 `yaw_deg`/四元数，以及已验证的入口 `route`。

输入可以是：

```json
{"boxes": [[0.62, 0.48, 0.24], [0.58, 0.44, 0.22]]}
```

用已训练 checkpoint 推理：

```powershell
python .\packing_policy.py `
  --input .\packing_policy_input_example.json `
  --checkpoint .\packing_policy_stateaware.pt `
  --output .\packing_policy_plan.json `
  --device cpu
```

不提供 checkpoint 时使用确定性的体积/深度 baseline：

```powershell
python .\packing_policy.py --input .\packing_policy_input_example.json --output .\packing_policy_plan_baseline.json
```

直接输入箱子序列并用已训练 checkpoint 生成 Newton 可视化（如果后续箱子无可行解，则回放已成功放置的部分）：

```powershell
conda activate newton-rope
cd D:\Programming\test\newton_container_demo

python .\run_packing_visual.py `
  --boxes "0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25;0.55,0.40,0.25" `
  --checkpoint .\packing_policy_stateaware.pt `
  --output .\packing_policy_ik_10_learned_partial.json `
  --device cuda:0 `
  --candidates 2 `
  --allow-partial
```

已有回放样本可直接打开 Viewer，不需要重新规划：

```powershell
python .\online_packing_newton.py `
  --samples .\packing_policy_ik_10_learned_partial.json `
  --mode show --seed 1 --device cuda:0
```

`input_index` 表示该箱子在输入序列中的索引，`step` 是实际放置顺序。策略允许重排输入箱子；如果业务流程要求保持输入顺序，把每一步候选限制为当前未放置序列头即可。该接口强制对每个输出动作执行 S20 Newton IK：入口路线按 5 cm 采样，检查关节限位、末端位置/姿态误差和机械臂连杆与集装箱网格的碰撞；候选同时满足容器边界、箱间无碰撞、全底面支撑和最大面朝下。不可达时继续搜索其他候选，找不到严格可行解就报错，不退回纯几何计划。可视化也会启用机械臂 IK 跟随。这里验证的是离散路线采样点，真实控制器连续轨迹/动力学仍需部署侧验证。

先验证 Newton 在一个模型里批量推进多个彼此独立的物理世界：

```powershell
conda activate newton-rope
python .\parallel_newton_smoke.py --envs 16 --steps 240 --device cuda:0
```

程序会用 `ModelBuilder.replicate()` 复制独立世界，并检查每个自由落体箱是否分别稳定在地面上。可用 `--device cpu` 测 CPU，`--envs` 调整并行世界数。该脚本是并行能力 smoke/benchmark，不含装箱策略。

轻量 REINFORCE 装箱 policy 使用 PyTorch 小型评分网络。网络输入完整箱单（置换不变的集合编码）、当前已放箱状态和候选动作特征；策略从候选中联合选择“哪只箱子、哪个位置、哪个朝向”，然后按顺序放置。候选生成器检查容器边界、无碰撞、全底面支撑、最大面朝下，以及当前高层几何入口路径，因此网络只会在可行候选之间选择。放好的箱子不再移动。训练回报是装入体积比例加权的中心深度目标；每个训练批次对同一箱单做多条 rollout，并用按时间步的回报差降低 REINFORCE 方差。独立评估箱单会同时报告 learned-greedy 与“大箱优先、再取更深可行候选”的简单基线：

```powershell
python .\packing_rl.py --epochs 100 --envs 32 --boxes 10 --device cuda:0
```

推荐用有限 lookahead oracle 做行为克隆暖启动，再进行 REINFORCE：

```powershell
python .\packing_rl.py --behavior-cloning --epochs 80 --envs 12 --rollouts-per-instance 4 --boxes 16 --candidates 8 --device cuda:0 --output .\packing_policy_bc.pt
```

`--baseline lookahead` 会在当前动作后预览下一批可行箱位，奖励未来仍可放置的体积；它只用于 oracle/评估，不绕过碰撞和路径检查。

训练速度较慢时加入 `--fast-training`。它只把严格入口路径检查延迟到策略选中候选之后；最终写入样例的每个动作仍经过完整 `route_to` 验证，失败候选会按策略分数继续尝试。

评估还支持小宽度 sequence beam search：

```powershell
python .\packing_rl.py --mode eval --output .\packing_policy_bc.pt --boxes 20 --eval-envs 4 --beam-width 4 --beam-candidates 4
```

beam 会保留多个完整的放置序列，直到箱单结束，再按总体积和深度目标选择序列；它比单步贪心更适合作为行为克隆 teacher。宽度增大时计算量近似线性增加，但每个候选仍需通过完整几何入口路径检查。

短 smoke 运行可用 `--epochs 2 --envs 4 --eval-envs 4 --boxes 4 --candidates 4 --hidden 8`。训练输出默认写入 `packing_policy.pt`、`packing_training.json` 和 `packing_policy_sample.json`；可用 `--output`、`--history`、`--sample-output` 分别改路径。样例生成后会调用现有 `validate_sample` 回放检查。隐藏层默认 16，网络约 2300 个参数。可单独加载 checkpoint，在更拥挤的箱单上对照基线：

```powershell
python .\packing_rl.py --mode eval --output .\packing_policy_stateaware.pt --eval-envs 8 --boxes 20 --candidates 8 --sample-output .\packing_policy_crowded_sample.json
```

10 箱评估对当前容器偏简单，两种策略经常全部装入。结果应优先看固定 `--eval-seed` 的更拥挤箱单上平均放入数量、体积利用率和配对回报差，而不是单次训练回报。

当前这两条管线是分层打通的：Newton 已实测支持单模型并行推进复制物理世界；RL 训练则并行批量计算候选策略分数并用几何模型计算回报，**Newton 物理还没有接入 RL 每一步的状态转移或训练 reward**。所以 RL 命令验证的是策略训练、数据输出和几何约束，不应解读为物理仿真内训练。下一步可把选中的 episode 汇入 replicated Newton worlds 做物理验证，再决定是否引入物理 reward。

IK 模式还强制最大矩形面朝下：箱子的最小尺寸轴固定为 z 轴，只允许绕 z 轴的 0°/90° 旋转，因此不会把轮子所在的侧面放到下面或上面。20--32 寸随机箱可用正面矩形对角线采样，并对对角线和厚度加入 ±10% 噪声：

```powershell
python .\online_packing_real.py --ik --luggage-20-32 --count 10 --seeds 1 --output .\online_packing_ik_luggage_maxface10_samples.json
python .\online_packing_newton.py --samples .\online_packing_ik_luggage_maxface10_samples.json --mode show --seed 1 --device cuda:0
```

在当前 deploy 场景和严格机械臂碰撞检查下，seed 1 的第一只随机箱成功放入，第二只没有可行的最大面朝下 IK 路径，因此规划和动画在第二只处停止。

IK 候选生成先按 S20 URDF 的肩部到末端保守臂展半径，解析求出每个 y/z 层对应的 x 区间，只在区间内生成候选；整条笛卡尔路线也会先做球形包络检查，明显超出臂展的路线不会逐点跑 IK。空场景第一箱按门边角落优先，只需找到一个有效落位即可。baseline 发现直接放置可行时立即提交，不再继续搬移搜索。IK 求解删掉相同的重复初始种子及额外的 ±π 盲目重试。这个球形范围只是快速剪枝，区间内仍须通过严格 IK 与 Newton 网格碰撞检查。`--ik-x-step` 只控制区间内候选采样密度。

严格 IK 的历史实测（RTX 5060 Laptop GPU，Warp 缓存已建立）中，同尺寸首箱曾耗时 45--57 秒；本轮移除重复 IK 种子后，同尺寸首箱实测 34.0 秒（seed 8，1 箱，规划部分）。单箱首次规划还包含很多不可达候选的严格路径搜索，后续应继续用 MoveIt 离线可达性图或对空场景入口姿态预缓存，避免每次重新发现同一工作空间边界。严格 IK 模式目前仍不适合直接放进控制周期。
