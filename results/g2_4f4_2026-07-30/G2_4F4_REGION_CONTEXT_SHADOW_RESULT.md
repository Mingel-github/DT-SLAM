# G2-4F4 区域上下文连续残差 Shadow 结果

日期：2026-07-30

## 1. 结论

G2-4F4 完成了一个有界的离线 representation audit，但没有通过预冻结的区域
保真停止条件：

```text
Python partition self-test                    PASS
combined support >=3 features                 14 frames
point-level inside median > background        14/14
proxy-selected region median > background     11/14 = 78.57%
required regional direction retention         >=80%
regional representation gate                  FAILED
dynamic_decision                              none
direct_slam_state_mutation                    none
G1-F / G1-D                                   locked
```

准确结论是：

> 连续 sparse ego-flow residual 本身仍然在冻结粗框中局部富集；当前轻量
> depth-discontinuity connected component 有时横跨大块背景，用整区 median
> 聚合反而稀释了运动信号。

因此不把 G2-3R0 区域直接提升为 F1 residual 的对象级容器，不修改区域选择规则
追结果，也不进入 online shadow 或真实过滤。

## 2. 文献身份

本阶段先读取本地 PaperNotes，再核对 DetectFusion 与 SInDSLAM 原始 PDF。

- `[L]` DetectFusion 把 ICP motion mask 与 normal/distance geometry segment
  做 IoU；
- `[L]` SInDSLAM 把 dense optical-flow residual 的判定和传播限制在
  geometric re-cluster 内；
- `[A/S/H]` 当前实验只把既有 sparse ego-flow continuous residual 聚合到
  G2-3R0 轻量 depth-discontinuity connected component。

当前没有：

- DetectFusion 的 surfel/ICP residual；
- SInDSLAM 的 K-means、plane edge、RAG/depth-histogram re-clustering、
  dense flow、PROSAC、Triangle 双阈值或时序先验。

所以本阶段不是两篇论文中任何一篇的复现。

## 3. 输入与防泄漏

只使用已经打开的 development 数据：

```text
rgbd_bonn_balloon
rgbd_bonn_balloon2
```

输入来自：

- exact C++ G2-4F3U node/frame CSV；
- geometry/flow 计算前冻结的 RGB-only coarse bbox；
- 冻结 candidate frame/depth path；
- Bonn 原始 depth archive；
- 与在线运行相同的 rectified `P=K` 域。

没有读取已完成一次性 F2 holdout 的 `balloon_tracking`，没有按 residual 重新
选帧或改粗框。

## 4. 实现

新增只读工具：

```text
DT-SLAM/tools/audit_region_context_sparse_flow.py
```

它执行：

```text
uint16 depth archive
→ INTER_NEAREST rectification
→ CV_32F meters
→ G2-3R0 boundary + 4-connected partition
→ exact measured nonsemantic feature 到 region 映射
→ region residual median/P90/MAD/support
→ frozen coarse-box paired proxy audit
```

`proxy-selected region` 在读取 residual 前按 bbox 内 eligible feature 数最多的
规则确定。无同分调参、无动态阈值。

## 5. 分序列结果

| 序列 | bbox 帧 | selected region 可得 | 支持 >=3 | selected median > background | point inside > background | selected ratio median |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| balloon | 8 | 6 | 5 | 2/5 | 5/5 | 0.884× |
| balloon2 | 9 | 9 | 9 | 9/9 | 9/9 | 20.253× |
| 合计 | 17 | 15 | 14 | **11/14** | **14/14** | **17.526×** |

合并后：

```text
selected feature bbox-purity median = 0.875
selected pixel bbox-purity median   = 0.985
selected bbox coverage median       = 0.481
point inside/background ratio       = 19.389x median
selected region/background ratio    = 17.526x median
```

这些 median 看起来较好，但不能隐藏逐帧失败。代表性泄漏：

| 序列/帧 | bbox 内 selected features / selected total | pixel bbox-purity | selected/background | point-inside/background |
| --- | ---: | ---: | ---: | ---: |
| balloon 39 | 6 / 610 | 0.0135 | 0.884× | 19.995× |
| balloon 200 | 7 / 78 | 0.0896 | 0.696× | 19.370× |
| balloon 252 | 19 / 601 | 0.0159 | 0.146× | 1.609× |
| balloon2 61 | 13 / 233 | 0.0393 | 1.113× | 55.696× |
| balloon2 318 | 7 / 430 | 0.0283 | 2.226× | 8.760× |

在这些帧中，bbox 内 residual 仍很高，但所选 depth component 包含大量 bbox 外
静态 features。整区 median 因而不再代表运动区域。

## 6. 为什么这不是 residual 方法失败

同一批 14 个有支持帧：

```text
point-level coarse-box inside > background = 14/14
region-level selected > background         = 11/14
```

所以当前被否定的是：

```text
G2-3R0 lightweight depth component
可以直接作为 sparse residual 的整对象聚合单位
```

而不是：

```text
sparse observed-flow minus ego-flow 没有运动信号
```

这也与 SInDSLAM 为什么需要 K-means、plane edges、depth-histogram RAG
re-clustering 相吻合：纯深度突变 connected component 不能可靠处理与背景深度
连续或边界不完整的对象。

## 7. 停止决策

预冻结停止条件中至少两项触发：

1. 区域方向保持率 `78.57% < 80%`；
2. point-level `14/14` 降到 region-level `11/14`；
3. 多个帧出现 selected region 横跨数百个背景 features 的灾难性泄漏。

因此：

```text
online F4 implementation         = not approved
region threshold tuning          = forbidden
alternate bbox-region selector   = not attempted
G1-F / G1-D                      = locked
```

没有继续跑静态序列或在线性能，因为 representation 必要条件已失败；继续增加
实验不会改变“不采用当前区域容器”的决定。

## 8. 下一步含义

当前证据已经较明确：

- F1 连续 residual 是目前最有希望的类别无关局部证据；
- whole-frame hard normalization 失败；
- 两帧 edge rigidity 及其 uncertainty normalization 未改善判决；
- 简单 depth component 聚合会在部分帧稀释 F1 信号。

下一步不能再做轻微阈值修补。只剩两个合理方向：

1. 实现有完整文献依据的更强几何区域表示，例如 SInDSLAM re-clustering 的
   明确子集，并接受更高复杂度；
2. 不追求完整区域，回到 feature-level continuous evidence，重新设计一个
   独立验证数据和保守 fail-safe 协议，而不是复用失败的 F2 hard gate。

在选择前应先做一次总路线决策，不继续自动写代码。

## 9. 产物

- `G2_4F4_REGION_CONTEXT_LITERATURE_DECISION.md`
- `G2_4F4_REGION_CONTEXT_SHADOW_SPEC.md`
- `G2_4F4_REGION_CONTEXT_SHADOW_RESULT.md`
- `DT-SLAM/tools/audit_region_context_sparse_flow.py`
- `development_audit/balloon_per_frame.csv`
- `development_audit/balloon_per_region.csv`
- `development_audit/balloon_summary.json`
- `development_audit/balloon2_per_frame.csv`
- `development_audit/balloon2_per_region.csv`
- `development_audit/balloon2_summary.json`
- `development_audit/combined_summary.json`
