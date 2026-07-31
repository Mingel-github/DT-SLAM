# G2-5A Semantic-reference / Semantic-blind Sparse F1 Shadow 结果

日期：2026-07-30
状态：连续证据条件通过；允许设计 G1-F0 counterfactual；真实过滤仍锁定

## 1. 结论

G2-5A 得到当前几何路线中较明确的正向结果：

```text
walking person-region proxy:
  comparable frames                         113
  semantic raw median > background          108/113 = 95.58%
  semantic-blind q median > background      108/113 = 95.58%
  per-frame AUC median                      0.857

sitting person-region challenge:
  comparable frames                         144
  semantic raw median > background          114/144 = 79.17%
  per-frame AUC median                      0.659
```

walking 同时通过全部预注册连续证据条件。sitting 的分离明显较弱，符合“person
类别相同、但运动强度通常更低”的预期方向。

结合此前冻结的 unknown balloon 开发结果：

```text
inside residual median > outside:
  15/15 comparable frames
```

可以写成：

> raw sparse ego-flow residual，以及用全部合格特征估计尺度得到的
> semantic-blind continuous evidence，在已知人物运动区域和未知气球代理中
> 均显示跨类别方向性。

仍不能写成：

> 已经获得可靠动态二值分类器。

## 2. 为什么没有修改 C++

代码核查确认：

```text
ComputeSparseEgoFlow()
  对全部 mCurrentFrame.mvKeys 计算 residual

semantic_nonzero
  只在 F1 CSV 记录时读取
```

F1 的分数不读取 semantic mask。semantic exclusion 发生在后续 F3 rigidity
图，而不是 F1 residual 计算前。

因此本阶段只新增离线工具：

```text
DT-SLAM/tools/audit_semantic_reference_sparse_flow.py
```

没有增加 C++ semantic bypass，也没有改变现有 pipeline。

## 2.1 尺度身份更正

初版离线审计虽然 raw residual 完全不读取 semantic，但 q 的 frame scale
排除了 semantic feature。该 q 属于当前 semantic+geometry combined pipeline，
不能称为 semantic-blind。

现已保留初版输出作为审计记录，并以 `audit_v2_scale_identity_correction`
为权威结果，同时并列计算：

```text
q_blind:
  scale 使用全部 quality-eligible feature
  semantic_blind_scale_uses_semantic=false

q_combined:
  scale 只使用 semantic_nonzero=0 feature
  combined_scale_excludes_semantic=true
```

候选删除在两种模式下始终排除 semantic feature。这个修正改变的是归一化尺度
的身份，不改变 raw residual、逐帧方向比例或 rank AUC。

## 3. 在线输入完整性

两条序列均使用：

```text
TUM3_GeometrySparseEgoFlowShadow.yaml
CUDAExecutionProvider
YOLOv8n-seg person mask
mask age median=0, max=0
Viewer OFF（正式统计）
```

| 序列 | 输入帧 | F1 frame rows | mask ready | eligible semantic | eligible nonsemantic |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 149 | 148 | 149/149 | 960 | 57256 |
| sitting | 149 | 148 | 149/149 | 1596 | 64864 |

semantic baseline 已经阻止 person feature 关联 MapPoint，因此当前 semantic
eligible 子集中没有可用于评价的 MapPoint。这里评价的是 feature residual，
不是 semantic MapPoint 删除效果。

## 4. 连续分布

### Walking

| 指标 | 中位 |
| --- | ---: |
| 每帧 semantic eligible features | 6 |
| 每帧 nonsemantic eligible features | 396 |
| semantic raw residual | 2.025 px |
| nonsemantic raw residual | 0.560 px |
| semantic/nonsemantic median ratio | 3.203× |
| per-frame AUC | 0.857 |

逐帧 AUC：

```text
P10 = 0.655
P90 = 0.955
```

不是所有帧都强分离，最低帧 AUC 为 0.144；因此仍需 unknown/no-evidence 和
逐帧保底，不能把聚合中位数解释成每帧可靠。

### Sitting

| 指标 | 中位 |
| --- | ---: |
| 每帧 semantic eligible features | 10 |
| 每帧 nonsemantic eligible features | 438.5 |
| semantic raw residual | 0.485 px |
| nonsemantic raw residual | 0.321 px |
| semantic/nonsemantic median ratio | 1.573× |
| per-frame AUC | 0.659 |

sitting 不是纯静态负样本；人物仍可能说话、手势和轻微运动。它只说明 F1 对
walking 的响应不是简单的“只要位于 person mask 就同样高”。

## 5. 固定 q 曲线

本阶段只报告预冻结网格，不选择新阈值。

以下为预注册主判定使用的 `q_blind`。本阶段没有从曲线中选择阈值。

| q | walking semantic | walking background | sitting semantic | sitting background | fr1 static MapPoint | Bonn static MapPoint |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 55.94% | 11.36% | 10.15% | 5.02% | 2.00% | 2.22% |
| 3 | 36.56% | 6.75% | 2.26% | 0.982% | 0.383% | 0.827% |
| 4 | 25.42% | 4.68% | 1.25% | 0.284% | 0.155% | 0.449% |
| 5 | 18.65% | 3.47% | 0.439% | 0.125% | 0.086% | 0.286% |
| 6 | 13.65% | 2.37% | 0.251% | 0.085% | 0.072% | 0.183% |
| 8 | 5.94% | 1.08% | 0% | 0.032% | 0.067% | 0.124% |
| 10 | 3.54% | 0.554% | 0% | 0.019% | 0.067% | 0.092% |

`q_combined` 与 `q_blind` 在当前数据上的差异较小。例如 walking 的
q6/q8/q10 semantic candidate rate 分别为
`13.85%/6.25%/3.75%`，但它属于利用语义先验保护尺度的组合系统诊断，
不用于支持“几何分数完全 semantic-blind”的表述。

该表说明：

- q 越高，真静态 MapPoint 风险下降；
- 同时 walking person-region sensitivity 迅速下降；
- q10 在 walking 只保留 3.54%，与此前 balloon holdout sensitivity 不足的
  结果一致；
- 不能仅靠提高 q 获得“既高灵敏又极安全”的通用 hard gate。

因此 G2-5A 的正结果属于 continuous separability，不是 F2 hard threshold
失败被推翻。

## 6. 运行性能

| 序列 | actual FPS | F1 active 中位 | F1 active P95 | semantic total 中位 |
| --- | ---: | ---: | ---: | ---: |
| walking | 27.99 | 2.65 ms | 3.24 ms | 10.07 ms |
| sitting | 27.96 | 2.47 ms | 2.87 ms | 10.22 ms |

这是同步语义完整 pipeline 的短序列测量。用户已放宽 30 FPS 硬要求；这里仍
如实记录，没有把短序列约 28 FPS 写成 30 FPS。

## 7. 判定

预注册条件：

| 条件 | 要求 | 结果 |
| --- | ---: | ---: |
| walking comparable frames | ≥20 | 113 |
| raw direction | ≥80% | 95.58% |
| semantic-blind q direction | ≥80% | 95.58% |
| per-frame AUC median | ≥0.75 | 0.857 |
| unknown development direction | ≥80% | 15/15 |

因此：

```text
G2-5A continuous evidence support       PASS
G1-F0 counterfactual design             ALLOWED
G1-F real filtering                     LOCKED
G1-D                                    LOCKED
dynamic_decision                        none
direct_slam_state_mutation              none
```

## 8. 下一步

下一步只做 G1-F0 counterfactual：

```text
对 q=6/8/10
统计“若删除将影响哪些非 semantic MapPoint”
统计每帧剩余 MapPoint 数、候选比例和失效风险
不实际清空 mvpMapPoints
不调用额外 PoseOptimization
```

q=6/8/10 是固定安全—灵敏度诊断网格，不代表已经选择部署阈值。G1-F0 必须
使用 true-static、walking、sitting 和 unknown development 成对报告；已经
打开的 balloon holdout 不得用于重新选 q。

## 9. 验证

```text
Python self-test                  PASS
malformed/header handling         复用 F2 parser
raw residual uses semantic        false
semantic-blind scale uses semantic false
combined scale excludes semantic  true
tied-rank AUC                     PASS（synthetic）
deterministic replay              PASS
dynamic_decision                  none
direct_slam_state_mutation        none
```

输出：

```text
results/g2_5a_2026-07-30/
  G2_5A_SEMANTIC_REFERENCE_SPARSE_FLOW_SHADOW_SPEC.md
  G2_5A_SEMANTIC_REFERENCE_SPARSE_FLOW_SHADOW_RESULT.md
  online_runs/
  audit_v1/                              # 初版，q scale 身份表述已被更正
  audit_v2_scale_identity_correction/    # 权威结果
```
