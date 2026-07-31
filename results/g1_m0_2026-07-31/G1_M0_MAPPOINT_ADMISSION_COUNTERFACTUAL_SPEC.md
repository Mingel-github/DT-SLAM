# G1-M0 MapPoint 写入 Counterfactual SPEC

日期：2026-07-31
状态：冻结后实现
身份：只读 mapping-admission counterfactual；不阻止任何地图写入

## 1. 目标

G1-F1 已能在 `TrackLocalMap()` 的第二次既有位姿优化前清除少量 q10
高 sparse ego-flow residual 关联，但这不能自动保证它们不进入地图。

本阶段回答：

> 当一个实际启用 G1-F1 q10 的成功跟踪帧被选为 KeyFrame 时，几何候选有多少
> 仍进入 KeyFrame，有多少会由 RGB-D 深度路径立即创建 MapPoint，以及有多少
> 是刚被 tracking filter 清除后又被 `CreateNewKeyFrame()` 重新创建？

本阶段只记录原始行为，不设置 dynamic flag，不阻止 MapPoint。

## 2. 依据与身份

- `[L]` ORB-SLAM2 RGB-D：初始化和新关键帧可直接从单帧深度创建 MapPoint；
- `[L]` Ji 2021：只保留静态关键帧和静态地图点是动态 SLAM 的关键系统约束；
- `[L]` DynaSLAM：动态观测不进入静态 tracking/mapping；
- `[E]` 当前 DT-SLAM 调用图：tracking 中清空关联后，
  `CreateNewKeyFrame()` 会把“无 MapPoint 且有深度”的 feature 重新作为新点；
- `[S]` 只读 counterfactual、字段和 fail-fast invariant 是本项目工程审计。

G1-M0 不是新的动态检测算法，也不是 Ji/DynaSLAM 复现。

## 3. 当前代码中的三个写图位置

### 3.1 RGB-D 初始化

```text
Tracking::StereoInitialization()
→ 对 z>0 且非 semantic dynamic 的 feature
→ new MapPoint
```

相邻帧 ego-flow 在首帧没有参考，因此几何 q10 不可用。G1-M0 必须把这个限制
记录为 `reference_unavailable`，不能伪造首帧几何候选。

### 3.2 Tracking 线程的新关键帧深度写入

```text
Tracking::CreateNewKeyFrame()
→ 有效深度 feature 排序
→ 无 MapPoint 或 Observations()<1
→ new MapPoint
```

这是 G1-M0 的主要审计位置。

### 3.3 LocalMapping 三角化

当前 `LocalMapping::CreateNewMapPoints()` 已读取 `KeyFrame::mvbDynamic`，并跳过
任一关键帧端被标记 dynamic 的匹配。G1-M0 不修改该文件，只统计当前 q10
候选若写入 KeyFrame 时形成的潜在暴露。

## 4. 精确运行条件

G1-M0 仅允许在以下条件下启用：

```text
RGB-D
Geometry.SparseEgoFlowShadowEnable = 1
G1-F1 tracking filter              = 1
G1-F1 q                            = 10
mapping counterfactual CSV         非空
```

原因是本阶段审计已经冻结的实际工作点，而不是重新比较 q6/q8/q10。

## 5. 每个写图事件记录

初始化或每次 `CreateNewKeyFrame()` 生成一条 summary，至少包含：

- frame/time/stage；
- q、scale valid、candidate vector valid；
- 当前 feature 和 geometry candidate 数；
- candidate 中仍有 MapPoint association 的数量；
- G1-F1 实际清除过 association 的 candidate 数；
- 有效深度 feature/candidate；
- 实际进入 ORB-SLAM2 depth-admission loop 的 feature/candidate；
- baseline 实际创建 MapPoint 数；
- 其中 geometry candidate 数；
- 被 G1-F1 清除后又立即重建的 candidate 数；
- `counterfactual_only=true`；
- `mapping_veto=none`；
- `direct_mapping_state_mutation=none`。

## 6. 不变量

- G1-M0 本身不得改变 `mvpMapPoints`、`mvbDynamic` 或 KeyFrame；
- `candidate_created <= total_created`；
- `recreated_after_tracking <= candidate_created`；
- candidate vector 不匹配时只记录 invalid，不能猜测；
- 初始化几何状态必须为 reference unavailable；
- 不修改 LocalMapping、Optimizer、g2o 或 YOLO；
- 不新增线程或位姿优化。

## 7. 第一轮数据

固定使用 G1-F1 已完成的三个 semantic 序列：

```text
TUM fr3/walking_xyz
TUM fr3/sitting_static
Bonn balloon
```

统一运行 q10、Viewer OFF、在线 CUDA semantic。

评价量：

```text
candidate keyframe exposure
candidate immediate MapPoint creation
tracking-removed then recreated count
candidate / all created fraction
keyframe frequency
counterfactual active time（若可忽略则由 tracking timing 间接确认）
```

## 8. G1-M1 决策

只有满足以下条件才设计真实 mapping veto：

1. 动态序列存在非零且可重复的 candidate MapPoint admission；
2. counterfactual 不变量完全通过；
3. veto 能复用 `mvbDynamic` 与既有 LocalMapping guard，不新增另一套后端状态；
4. 真实 veto 继续默认关闭，并重新跑 ATE/RPE/FPS 和地图点数量。

若 candidate admission 基本为零，则停止 G1-M1，因为新增地图修改没有实际作用。
