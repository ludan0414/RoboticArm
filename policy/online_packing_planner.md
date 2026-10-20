# 在线近箱规划器（不考虑机械臂）

这里把问题定义成**在线三维装箱**。规划器只看到当前箱子的长宽高，不能读取后续箱子，也不假设箱子尺寸服从某个已知分布。每次当前箱子必须完成“放入或明确失败”后，才能接收下一个箱子。

## 1. 状态和硬约束

把集装箱内坐标固定为

\[
C=[0,L]\times[0,W]\times[0,H],
\]

开口默认为 `x=0`，箱子沿 `+x` 深入。每个已经放置的箱子保存

```text
Box(id, size=(lx, ly, lz), pose=(x, y, z), orientation)
```

第一版只使用轴对齐的 6 个长方体朝向，也就是 `size` 的 6 个排列。这样候选位置是有限的、容易验证的；连续旋转留给后续几何规划器。

一个候选放置必须满足：

1. **在容器内**：箱子的 AABB 完全位于内部有效空间；
2. **不穿透**：和任何已有箱子的体积交集为零，面接触和边接触允许；
3. **全支撑**：箱子底面投影必须完全落在容器地板或同一高度的已有箱子顶面并集内；不允许只由几个角点支撑；
4. **入口可达**：从开口到目标位置的箱体扫掠体不能穿过已有箱子。高层规划暂时用 AABB 走廊检查，机械臂和 6D 碰撞检查以后再接入；
5. **事务闭合**：本次如果把某个箱子暂放到箱外，必须在本次事务结束前放回箱内，不能带着未解决的箱子接收下一个箱子。

全支撑的判定不是“检查四个角点”。设候选底面是矩形 `R`，高度为 `z`，支撑矩形集合为 `S(z)`，要求

\[
R\subseteq\bigcup_{s\in S(z)}s.
\]

实现上可以用 `R` 和所有支撑矩形的 x/y 边界切分成小网格，检查每个小网格中心是否被支撑。这个方法能正确处理多个箱子共同支撑一个箱子。

## 2. 不知道未来箱子时的策略

没有未来信息时，不存在对所有输入序列都最优的算法。一个反例是：当前箱子放满一个大空腔后，下一箱可能刚好是只能进入该空腔的小箱子；提前知道下一箱时会做不同决定。因此这里优化的是**当前可见信息下的稳健性**，而不是假装预测未来。

稳健性用几何量表示，不使用箱子尺寸分布：

- 保留从开口到内部的连续可达通道；
- 避免把空腔切成许多细小碎片；
- 保留较大的最大空矩形空间；
- 保留更多不同朝向和支撑高度的可行候选；
- 尽量贴后、贴侧、贴底放置，减少不可利用的薄缝；
- 当前箱子能直接放入时，默认不搬已经放好的箱子。

因此默认顺序是：**后到先放、低处先放、从后壁向开口推进**。但它不是固定的“按列堆满”：每次仍然枚举候选并按评分选择，避免当前箱子把未来空间切碎。

## 3. 候选生成

对当前箱子 `b`：

```text
for orientation in six_axis_aligned_orientations(b):
    for z in {0} ∪ {top_z(existing_box)}:
        for x in {0, L-lx} ∪ {existing.x, existing.x+existing.lx,
                              existing.x-lx, existing.x+existing.lx-lx}:
            for y in {0, W-ly} ∪ {existing.y, existing.y+existing.ly,
                                  existing.y-ly, existing.y+existing.ly-ly}:
                candidate = (x, y, z, orientation)
                if inside(candidate) and collision_free(candidate):
                    if fully_supported(candidate) and access_ok(candidate):
                        keep(candidate)
```

只在已有箱子边界、容器边界和支撑顶面产生坐标，避免把连续空间离散成很密的体素。候选生成后做去重，并保留每个局部区域的前 `K` 个候选。

建议先用 `K=64`；箱子数量较多时把坐标压到 1–2 cm 网格，或使用空间索引（AABB tree / sweep-and-prune）加速碰撞查询。

## 4. 评分函数

候选评分采用“硬约束过滤 + 软评分”，而不是把穿透或悬空当成高额罚分。对合法候选 `p`，定义归一化指标：

```text
compact(p)      # 与后壁、侧壁、地板和已有箱子的接触程度
access(p)       # 放入后剩余入口通道的宽度/面积
largest_void(p) # 放入后最大剩余空矩形体积
options(p)      # 以剩余空间生成的可行支撑/朝向候选数
fragment(p)     # 空间碎片化程度
height(p)       # 重心高度或顶面高度
```

第一版建议用下面的目标：

\[
\begin{aligned}
S(p)=&\; 3.0\,compact(p)+2.0\,largest\_void(p)
       +1.5\,options(p)\\
   &+1.0\,access(p)-1.5\,fragment(p)-0.4\,height(p).
\end{aligned}
\]

如果存在“直接放置”和“需要搬箱”的两个可行方案，不要只按空间分数选。用事务代价比较：

\[
J= -S(p)+\lambda_m N_{move}+\lambda_d D_{move}
       +\lambda_e N_{external\_staging},
\]

其中 `N_move` 是已有箱子被重新拿放的次数，`D_move` 是被移动箱子上方依赖箱子的数量，`N_external_staging` 是箱外暂放次数。初始可取

```text
lambda_m = 8.0
lambda_d = 3.0
lambda_e = 12.0
```

这使同等空间质量下优先零搬移；只有直接放置会明显破坏入口、空腔或后续可行性时，才接受重排。

## 5. 没有直接位置时的有限重排

不要全局重新优化。对当前箱子执行一个小范围的滚动事务：

1. 先保存当前状态；
2. 生成当前箱子的所有合法目标候选；
3. 找出阻挡目标箱体或阻挡入口走廊的已有箱子；
4. 按“上面的先搬、外面的先搬”构造依赖图；
5. 只搜索最多 `R` 个已有箱子的重排，默认 `R=2`，困难场景提高到 `R=4`；
6. 被移出的箱子先进入事务暂存区；每移出一个箱子，必须立即找到一个容器内临时合法位置，或者使用唯一的箱外暂存槽；
7. 放入当前箱子后，按逆序把暂存箱子重新插入；全部成功才提交事务；
8. 任意一个箱子无法回插，回滚整个事务，尝试下一候选；
9. 所有候选都失败时返回 `WAIT/MANUAL_REVIEW`，而不是继续接收下一个箱子。

搜索节点可以表示为 `(state, pending_boxes, actions, cost)`，用 beam search 保留最低代价的 32 个节点。节点扩展只做三类动作：

```text
PLACE(current_box, candidate)
REMOVE(box_id, staging_slot)
REINSERT(box_id, candidate)
```

由于支撑约束，`REMOVE` 只能从上到下；依赖图可以在扩展前做拓扑排序，避免尝试明显非法的搬移顺序。

## 6. 在线主循环

```text
state = empty_container()

while input_stream.has_next():
    b = input_stream.next_box()          # 只知道当前箱子
    assert state.external_staging_empty

    direct = enumerate_candidates(state, b)
    plan = choose_best(direct, move_cost=0)

    if plan is None:
        plan = bounded_rearrangement_search(state, b, max_relocations=2)

    if plan is None:
        return WAIT_OR_MANUAL_REVIEW   # 不拿下一个箱子

    execute_logical_plan(state, plan)
    assert state.external_staging_empty
```

这里的 `execute_logical_plan` 只改变箱子状态和动作日志，不调用机械臂。以后接 Newton 时，可以把每个 `PLACE/REMOVE/REINSERT` 转换成：

1. 几何放置结果；
2. 入口扫掠体和箱体碰撞检查；
3. 机械臂可达性和姿态规划；
4. 实际执行后的视觉更新；
5. 失败后回滚到最近一次已确认状态。

## 7. 最先实现的版本

为了先验证规划质量，建议按以下顺序实现：

1. 轴对齐 6 朝向、边界候选、AABB 碰撞；
2. 多箱顶面并集的全支撑检查；
3. `compact + largest_void + access + move_cost` 评分；
4. `R=0` 的纯在线装箱；
5. `R=2` 的事务式有限重排；
6. 随机箱子流和对抗箱子流测试；
7. 再接 Newton 的碰撞验证和机械臂规划。

验收指标不要只看最终体积利用率，还要记录：

```text
volume_utilization
placed_ratio
secondary_move_count
external_staging_count
largest_remaining_void
入口堵塞次数
单箱规划时间的 p50/p95
```

其中“没有未来箱子信息”应通过测试中的随机/对抗箱子流体现，而不是在规划器里偷偷使用完整序列。
