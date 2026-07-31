# G1-F0B `SearchLocalPoints()` 后关联反事实 SPEC

日期：2026-07-30
状态：实现前冻结
范围：默认关闭的原始状态记录＋离线反事实；不改变 SLAM

## 1. 唯一问题

G1-F0A 只看到初始 PoseOptimization 后、局部地图搜索前的 MapPoint 关联。
G1-F0B 回答：

> `SearchLocalPoints()` 重新加入局部 MapPoint 后，若复用同一帧 F1 连续证据
> 假设清除 q6/q8/q10 候选，局部地图优化前和现有优化后的支持量会减少多少？

这是实现原计划“SearchLocalPoints 后再次检查”的安全审计，不是真实过滤。

## 2. 方法与文献边界

- `[L/A]` F1 ego-flow residual 的物理依据来自 FlowFusion 类 observed-flow
  minus ego-flow；
- `[L/A]` 连续鲁棒尺度表达借鉴 Li and Lee 的静态权重思想；
- `[S]` 在 ORB-SLAM2 两个既有跟踪阶段记录关联快照，是本项目的工程安全协议；
- `[S]` q6/q8/q10 和 0.20% 预算不是论文给定阈值。

G1-F0B 不增加新的 motion score，也不声称复现 FlowFusion 或 Li and Lee。

## 3. 最小在线改动

新增默认关闭开关：

```yaml
Geometry.SparseFlowCounterfactualShadowEnable: 1
```

要求：

```text
Geometry.Enable=1
Geometry.SparseEgoFlowShadowEnable=1
RGB-D
```

输出路径：

```text
DT_SLAM_GEOMETRY_ASSOCIATION_SNAPSHOT_CSV
```

C++ 只记录原始 SLAM 状态，不在 C++ 中计算 q 或选择候选。每帧记录两个阶段：

```text
post_search_pre_pose:
  SearchLocalPoints()
  → RemoveDynamicAssociations()（保持现有 semantic baseline）
  → snapshot
  → existing PoseOptimization()

post_existing_pose:
  existing PoseOptimization()
  → snapshot
  → existing TrackLocalMap statistics/decision
```

每个 feature 记录：

- frame/timestamp/stage/feature index；
- has MapPoint；
- MapPoint bad flag；
- MapPoint observations；
- current outlier flag；
- 在现有 `mnMatchesInliers` 循环中是否被实际计数；
- semantic flag；
- only-tracking flag；
- 是否位于 relocalization 的严格窗口。

不在该 CSV 中复制 residual。离线通过 `(frame, feature_index)` 与同次 F1 CSV
精确连接，避免 C++ 和 Python 各实现一套 q 定义。

## 4. 离线候选

严格复用 G1-F0A：

```text
measured
FB <= 0.25 px
scale support >=20
q = residual / max(0.001, 1.4826*median(residual))
semantic_nonzero == 0
q >= {6,8,10}
```

并列运行：

```text
semantic_blind_all_eligible
combined_semantic_excluded
```

scale 无效时 fail-open，候选数为 0。

## 5. 阶段计数

### post_search_pre_pose

baseline proxy：

```text
has_mappoint && !mappoint_bad
```

报告候选和固定关联下的 remaining。该计数不预测重新优化后的 outlier 变化。

### post_existing_pose

mapping-mode inlier proxy：

```text
has_mappoint
&& !mappoint_bad
&& !current_outlier
&& observations > 0
```

它与当前 `TrackLocalMap()` 的 mapping-mode `mnMatchesInliers` 计数条件一致，但
离线“删除后 remaining”仍是固定现有位姿/现有 outlier 的 counterfactual；
不能声称等价于真实删除后重新运行优化。

实现审阅时进一步收紧：post-pose 主计数不在导出函数中重新读取
`MapPoint::isBad()/Observations()` 推导，而是在原 `mnMatchesInliers` 循环中
同步记录 `counted_tracking_inlier`。原因是 LocalMapping 并发可能在计数后改变
MapPoint 状态。重新读取的字段只作诊断，精确布尔量才是 post-pose 主口径。

## 6. 输入

与 F0A 相同：

```text
true static:
  TUM fr1/xyz static150
  Bonn static_close_far static150

known semantic:
  TUM walking150
  TUM sitting150

unknown development:
  Bonn balloon
  Bonn balloon2
```

online run 必须生成同一次执行的 F1 CSV 和 association snapshot CSV。不得把
不同 nondeterministic SLAM run 的 feature index 直接拼接。

继续禁止打开：

```text
balloon_tracking sealed holdout
```

## 7. 预注册检查

实现正确性：

- 开关默认关闭；
- 关闭时无新 CSV、行为不变；
- 每个 snapshot `(frame,stage,feature_index)` 唯一；
- 同一帧 snapshot feature index 与 F1 feature index 集合完全一致；
- `counted_tracking_inlier` 之和等于同帧当前 `mnMatchesInliers`；
- 重放输出确定。

初步支持条件：

```text
两个 scale mode、某一固定 q：
  两个 true-static 域 candidate / eligible MapPoint <=0.20%
  六序列 post_existing_pose baseline>=30 → remaining<30 = 0
  relocalization strict window 中 baseline>=50 → remaining<50 = 0
```

若多个 q 通过，不在本轮选择 q。

## 8. 明确非目标

```text
不修改 mvbDynamic
不清空 mvpMapPoints
不增加 PoseOptimization
不修改 Optimizer/g2o/YOLO
不评价 ATE/RPE
不声称 counterfactual 等于真实删除后结果
不解锁 G1-F 或 G1-D
```

## 9. 停止条件

- 若必须重复计算 LK/F1 才能记录 post-search 状态，停止并重新设计；
- 若无法精确连接同次执行的 feature index，结果无效；
- 若静态预算或 30/50 支持线条件失败，不进入真实过滤；
- 若 unknown development 候选仍接近零，只能说明风险低，不能据此启动 ATE
  改善实验。
