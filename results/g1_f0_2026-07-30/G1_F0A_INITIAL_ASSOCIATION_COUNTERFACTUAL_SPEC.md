# G1-F0A 初始 MapPoint 关联反事实审计 SPEC

日期：2026-07-30
状态：实现前冻结
范围：离线计数；不改变 Frame；不重新优化位姿

## 1. 目标

G2-5A 已支持 F1 continuous residual，但没有产生部署阈值。G1-F0A 只回答：

> 对固定 q=6/8/10，如果把 high-residual、非 semantic、已有 MapPoint 的
> feature 视为“假设删除”，当前初始跟踪阶段还剩多少 MapPoint 关联？

这是安全余量审计，不是动态检测效果实验。

## 2. 为什么先审计初始关联

`RunSparseEgoFlowShadow()` 当前调用位置：

```text
TrackWithMotionModel / TrackReferenceKeyFrame
→ 第一次 PoseOptimization
→ outlier 清理
→ RunSparseEgoFlowShadow
→ TrackLocalMap
```

F1 CSV 的 `has_mappoint` 因此描述初始位姿完成后的当前关联状态，适合先检查：

```text
如果此刻清除候选 mvpMapPoints，是否会把初始支持推近已有失败阈值？
```

它还不能描述 `SearchLocalPoints()` 后新匹配回来的 MapPoint。若 G1-F0A 通过，
后续 G1-F0B 才增加默认关闭的在线 counterfactual instrumentation，在
`SearchLocalPoints()` 后再次统计。

## 3. 与 ORB-SLAM2 当前阈值的关系

当前代码：

```text
TrackReferenceKeyFrame:
  搜索匹配 <15 直接失败
  动态移除后 <15 失败
  优化后有效 MapPoint >=10 成功

TrackWithMotionModel:
  搜索匹配 <20 失败
  动态移除后 <20 失败
  优化后有效 MapPoint >=10 成功

TrackLocalMap:
  最终 mnMatchesInliers >=30
  重定位后短窗口要求 >=50
```

F1 CSV 没有记录 `MapPoint::Observations()` 和调用分支，因此
`has_mappoint` 总数只是初始关联支持 proxy，不等于源码中的精确
`nmatchesMap`。

本阶段只报告：

```text
remaining associated MapPoint proxy
threshold-crossing risk
```

不得声称已经证明真实跟踪不会失败。

## 4. 输入

```text
true static:
  TUM fr1/xyz static150
  Bonn static_close_far static150

known semantic development:
  TUM walking150 online semantic
  TUM sitting150 online semantic

unknown development:
  Bonn balloon
  Bonn balloon2
```

不读取：

```text
balloon_tracking holdout
Optimizer residual
ATE/RPE
future frame tracking result
```

## 5. 固定候选

沿用 G2-5A/F2 表示，并明确区分两个尺度：

```text
measurement valid
FB <= 0.25 px
semantic_blind_all_eligible:
  frame scale from all eligible residual

combined_semantic_excluded:
  frame scale from nonsemantic eligible residual

scale support >=20
q = residual / scale
```

只统计：

```text
semantic_nonzero == 0
has_mappoint == 1
q >= {6,8,10}
```

q=6/8/10 是预冻结安全—灵敏度诊断网格，不是最终阈值。两个 scale mode
必须并列报告；semantic candidate 不计入几何反事实，因为语义 baseline 已经
独立处理它们。

scale invalid 时必须 fail-open：

```text
counterfactual removed = 0
state = insufficient_scale
```

## 6. 每帧指标

- baseline associated MapPoint proxy；
- quality-eligible nonsemantic MapPoint；
- counterfactual candidate MapPoint；
- remaining MapPoint proxy；
- candidate fraction；
- 是否从 baseline `>=10` 跨到 remaining `<10`；
- 是否跨越 15/20/30/50 支持线；
- feature candidate 数和比例；
- scale support/state。

所有 10/15/20/30/50 均称为：

```text
source-code threshold proximity diagnostics
```

不能把不同阶段阈值混成一个统一“安全阈值”。

## 7. 聚合指标

每序列、每 q 报告：

- evaluable frames；
- baseline MapPoint median/P10/min；
- candidate MapPoint total；
- candidate / quality-eligible MapPoint rate（与 F2 的 0.20% 预算同口径）；
- candidate / baseline associated MapPoint rate；
- per-frame removal median/P95/max；
- remaining median/P10/min；
- 10/15/20/30/50 crossing frame count；
- baseline 本来就在各支持线以下的帧数；
- fail-open frame count。

## 8. 预注册放行条件

G1-F0A 只在某个 q 同时满足以下条件时通过：

```text
两个 true-static 序列：
  total MapPoint counterfactual rate <=0.20%
  baseline>=10 → remaining<10 crossing = 0

walking / sitting：
  baseline>=10 → remaining<10 crossing = 0

balloon / balloon2 development：
  baseline>=10 → remaining<10 crossing = 0
```

`0.20%` 沿用 F2 已冻结的保守 static MapPoint 删除预算 `[S]`。它不是论文
阈值，也不是严格 false-positive rate。

若多个 q 通过：

```text
只允许说“这些工作点有初始支持可行性”
```

不得根据本轮结果选择最敏感的 q。G1-F0B 仍需成对比较三个 q。

## 9. 输出与不变量

```text
per_frame.csv
summary.json
G1_F0A_*_RESULT.md
```

固定字段：

```text
counterfactual_only=true
dynamic_decision=none
direct_slam_state_mutation=none
pose_reoptimization=none
```

## 10. 明确非目标

```text
不修改 mvpMapPoints / mvbDynamic
不修改 Tracking C++
不调用 PoseOptimization
不评价 ATE/RPE
不解锁 G1-F
不访问旧 holdout 调参
不把 has_mappoint proxy 写成精确 nmatchesMap
```
