# G1-F0A 初始 MapPoint 关联反事实审计结果

日期：2026-07-30
状态：初始关联支持条件通过；允许设计 G1-F0B；真实过滤仍锁定

## 1. 结论

对固定诊断网格 `q=6/8/10`，本阶段只计数：

```text
如果 high-residual、非 semantic、已有 MapPoint 的 feature 被假设清除，
初始 PoseOptimization 后的关联支持会减少多少？
```

六组输入、两种 frame-scale 身份下均得到：

```text
baseline >=10 → remaining <10             0 帧
跨越 10/15/20/30/50 任一支持线            0 帧
TUM fr1/xyz true-static candidate rate     <=0.0721%
Bonn static true-static candidate rate      <=0.1832%
预冻结 true-static MapPoint 预算             0.20%
```

因此：

> 当前三个 q 工作点都具有“初始关联支持可行性”，可以继续测量
> `SearchLocalPoints()` 后的反事实影响。

但这不是效果放行。unknown balloon 开发帧中的候选 MapPoint 很少：

```text
balloon:
  q6/q8/q10 = 2/0/0

balloon2:
  q6/q8/q10 = 3/2/1
```

这同时意味着：

- 删除风险很低；
- 对未知动态 MapPoint 的实际作用可能也很弱；
- 当前结果不能推出 ATE/RPE 会改善。

## 2. 审计位置与含义

F1 CSV 生成于：

```text
TrackWithMotionModel / TrackReferenceKeyFrame
→ 初始 PoseOptimization
→ outlier 清理
→ RunSparseEgoFlowShadow()
→ TrackLocalMap()
```

所以 `has_mappoint` 是初始位姿之后、`SearchLocalPoints()` 之前的关联 proxy。
它不等于源码中的精确 `nmatchesMap`，因为 CSV 没有：

- 当前调用分支；
- `MapPoint::Observations()`；
- `SearchLocalPoints()` 后新增关联；
- TrackLocalMap 最终 inlier 数。

10/15/20/30/50 只是源码支持线附近的诊断，不是一个统一的跟踪安全阈值。

## 3. 尺度身份修正

为避免把组合系统误称为 semantic-blind，本次权威输出并列运行：

```text
semantic_blind_all_eligible:
  用全部 quality-eligible feature 估计 frame scale

combined_semantic_excluded:
  只用 semantic_nonzero=0 feature 估计 frame scale
```

两种模式中的反事实候选都必须满足：

```text
semantic_nonzero == 0
has_mappoint == 1
FB <= 0.25 px
q >= {6,8,10}
```

初版 `audit_v1` 只有 combined scale，现保留为历史记录；
`audit_v2_scale_identity_correction` 是权威结果。

## 4. Semantic-blind scale 主结果

表中比例为 `candidate / quality-eligible nonsemantic MapPoint`。

| 序列 | 角色 | q6 候选/比例 | q8 候选/比例 | q10 候选/比例 | baseline MapPoint 中位/最小 |
| --- | --- | ---: | ---: | ---: | ---: |
| fr1/xyz static150 | true static | 26 / 0.0721% | 24 / 0.0665% | 24 / 0.0665% | 278 / 169 |
| Bonn static150 | true static | 62 / 0.1831% | 42 / 0.1240% | 31 / 0.0916% | 282.5 / 144 |
| walking150 | known semantic development | 30 / 0.1737% | 9 / 0.0521% | 3 / 0.0174% | 134 / 93 |
| sitting150 | low-motion semantic challenge | 0 / 0% | 0 / 0% | 0 / 0% | 177.5 / 138 |
| balloon | unknown development | 2 / 0.0923% | 0 / 0% | 0 / 0% | 198.5 / 134 |
| balloon2 | unknown development | 3 / 0.2389% | 2 / 0.1592% | 1 / 0.0796% | 176 / 71 |

Bonn static 有 11 帧因合格尺度支持少于 20 而 fail-open；这些帧假设删除数为
0，不能被计作“已证明静态”。其余序列没有 scale fail-open。

## 5. Combined scale 敏感性

两种尺度的支持结论相同。唯一可见差异是 walking q10：

```text
semantic-blind scale        3 candidate MapPoints
combined scale              4 candidate MapPoints
```

其余表中 q6/q8/q10 的 MapPoint 总数相同。这说明本批数据上的初始风险结论
不依赖 semantic-excluded scale，但不能据此假定所有序列都不敏感。

## 6. 预注册条件

| 条件 | q6 | q8 | q10 |
| --- | ---: | ---: | ---: |
| 两个 true-static 域可用 | PASS | PASS | PASS |
| 两个 static MapPoint rate 均 <=0.20% | PASS | PASS | PASS |
| 六序列 baseline>=10 → remaining<10 为 0 | PASS | PASS | PASS |
| semantic-blind 与 combined scale 都有初始支持 | PASS | PASS | PASS |

因此：

```text
G1-F0A initial-support counterfactual     PASS
G1-F0B post-SearchLocalPoints design      ALLOWED
G1-F real filtering                       LOCKED
G1-D                                      LOCKED
dynamic_decision                          none
direct_slam_state_mutation                none
pose_reoptimization                       none
```

## 7. 方法与证据边界

- `[L/A]` sparse ego-flow residual 的物理依据来自 FlowFusion 类自运动补偿残差；
- `[L/A]` 连续鲁棒尺度表达借鉴 Li and Lee 的静态权重思想；
- `[S]` q 网格、0.20% 静态预算和本反事实计数协议是项目工程安全设计，不是论文
  已验证阈值；
- 本阶段没有提出新的 dynamic classifier，也没有复现上述论文完整系统。

## 8. 验证

```text
Python self-test                  PASS
双尺度输出                         PASS
确定性重放 summary SHA256         ec79db683de2ffafa365ddb5e969111d781a3cc30c6ceedb01d8e59ef2186bfa
确定性重放 per_frame SHA256       11b8f398f870606b2655673eb4df0cbeb55e0d2234f4c803c530c3728e4d5a36
counterfactual_only               true
dynamic_decision                  none
direct_slam_state_mutation        none
pose_reoptimization               none
```

权威输出：

```text
results/g1_f0_2026-07-30/
  G1_F0A_INITIAL_ASSOCIATION_COUNTERFACTUAL_SPEC.md
  G1_F0A_INITIAL_ASSOCIATION_COUNTERFACTUAL_RESULT.md
  audit_v1/                              # 旧 combined-only 审计
  audit_v2_scale_identity_correction/    # 权威双尺度审计
```

## 9. 下一步

下一步是 G1-F0B，只增加默认关闭的在线反事实统计：

```text
TrackLocalMap()
→ SearchLocalPoints()
→ 统计此时 q6/q8/q10 假设候选
→ 记录剩余局部地图关联和最终 inlier 风险
→ 不清空 mvpMapPoints
→ 不调用额外 PoseOptimization
```

只有 F0B 同样显示低支持风险，且 unknown development 上存在足够候选作用，
才讨论一个可逆、默认关闭的 G1-F 小规模真实过滤实验。
