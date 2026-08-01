# Bonn 箱子两帧运动一致性输入审计结果

日期：2026-08-01
状态：完成；允许继续设计最小 motion-hypothesis shadow，不允许过滤

## 1. 执行与修正

新增只读工具：

```text
DT-SLAM/tools/audit_sparse_motion_coherence.py
```

它只读取已有 C++ 逐特征 CSV、粗箱框和可选的预冻结 RGB motion proxy。没有
重跑或修改 SLAM，没有聚类、动态阈值、对象标签和状态写入。

第一版错误地混合 moving/stationary 箱框。该结果没有删除，保存在：

```text
nonobstructing_v1_failed_mixed_motion/
obstructing_v1_unlabeled/
```

修正只连接在查看 geometry/flow 前已生成的 24 帧 motion proxy；详见
`G2_BOX_TWO_FRAME_MOTION_COHERENCE_INPUT_CORRECTION.md`。

## 2. 非遮挡箱子：motion-state 分层结果

既有 RGB 时序代理：

```text
moving       5 frames
stationary  19 frames
is_ground_truth=false
geometry_or_flow_seen=false
```

所有 24 帧框内均有至少 10 个 quality-eligible、semantic-static feature。

| 连续统计 | moving 中位 | stationary 中位 | moving-vs-stationary proxy AUC |
|---|---:|---:|---:|
| 框内 feature 数 | 59 | 229 | — |
| 框内 residual magnitude | 1.682 px | 0.235 px | 0.989 |
| 框内 direction concentration | 0.975 | 0.676 | 0.947 |
| 框内 residual dispersion | 0.537 px | 0.163 px | 1.000 |
| 框内/外 centroid separation | 1.934 px | 0.190 px | 0.989 |
| normalized separation | 3.069 | 0.846 | 0.874 |

`coherent_proxy_frame` 的描述结果：

```text
moving       5/5
stationary  10/19
```

因此：

- 当前 moving proxy 中确实存在多点、方向一致、相对背景分离的连续运动证据；
- 目标支持量不是这 5 帧的主要限制；
- 但简单“方向集中＋中心分离超过组内离散”仍会在约一半 stationary proxy 上
  触发，不能成为动态判决；
- moving 的 dispersion 也更大，说明真实目标未必由一个纯二维平移向量完整
  解释，或粗框/遮挡混入了其他表面。

AUC 是在 5 个 moving 与 19 个 stationary RGB-only proxy 上的描述性排序，样本
很小、不是 GT、也没有 sealed holdout，不能解释为检测精度。

## 3. 强遮挡箱子：无标签描述

17 个 frame 160--320 粗框均有至少 10 个 eligible feature；13 帧框外仍有至少
3 点可比较。结果：

```text
inside magnitude median > outside        4/13
inside concentration > outside            4/13
centroid separation > inside dispersion  11/13
同时满足后二者                          4/13
```

这些帧没有同索引、预冻结的 motion-state proxy，且部分箱体占满画面，所以不能
把 4/13 解释成 sensitivity。结果只说明该段存在大量目标支持，但目标/背景二维
残余结构不稳定。

## 4. 与既有结果的关系

本轮补充而不推翻：

- q10 strict holdout 失败：连续 residual 有信息不等于 q10 硬候选可靠；
- balloon/balloon2 track-support：两帧 transient 输入可用，长轨迹不稳定；
- F3/F3U/R2 失败：简单 scalar edge/CC 不足；
- R1 失败：单帧 normal+distance 区域严重过分割；
- G1 Bonn 结果：过滤链执行成功不等于目标特异性成立。

## 5. 决策

允许的下一步：

> 只设计一个有 Lee 2019 刚体 motion-hypothesis 原型依据的 shadow SPEC，检查
> 当前稀疏 RGB-D 两帧对应能否形成与背景相区别的三维运动假设。

下一步不能直接做二维 DBSCAN。Lee 原方法使用 grid RGB-D scene flow、局部 7 点
刚体变换 hypothesis、refinement、hypothesis clustering 和 temporal segment
matching。将其换成 ORB/LK 是显著适配，必须逐项写清 `[L]/[A]/[S]/[H]`。

继续禁止：

```text
将 coherence 指标写入 G1-F1/G1-M1
根据本轮 AUC 选择阈值
将 q10 改名为高置信动态点
把 bbox 用于部署
G1-D
Optimizer/g2o/backend 修改
```

当前结论：

> 稀疏两帧数据在少量明确运动箱子帧上具备研究共同运动假设的输入条件，但现有
> 二维一致性统计本身不具备足够静态排斥能力。

## 6. 产物

- `G2_MOTION_GROUPING_LITERATURE_AND_PRIOR_EVIDENCE_AUDIT.md`
- `G2_BOX_TWO_FRAME_MOTION_COHERENCE_INPUT_SPEC.md`
- `G2_BOX_TWO_FRAME_MOTION_COHERENCE_INPUT_CORRECTION.md`
- `nonobstructing_v2_motion_stratified/per_frame.csv`
- `nonobstructing_v2_motion_stratified/summary.json`
- `obstructing_v2_unlabeled_descriptive/per_frame.csv`
- `obstructing_v2_unlabeled_descriptive/summary.json`
- `literature/Lee_2019_Rigid_Motion_RGBD_VO_arXiv1907.08388.pdf`
