# S1 Gradient-only RAG merge shadow 结果

日期：2026-08-03  
状态：本增量完成；仍为 shadow-only

## 1. 实现身份

本轮实现的是：

> `[A]` SInDSLAM 论文 RAG/深度直方图合并在当前 gradient-only split 上的
> clean-room adaptation。

它保留了论文的区域排序、空间邻接、fake edge、深度直方图相似度、rank
weight 和两阶段合并结构；当前没有 plane edge，因此不是完整 SInDSLAM
re-clustering reproduction。

为避免早期区域传播实验出现的链式塌缩，本轮冻结了以下可审计约束：

- shared fake boundary 是不同 initial region core component 的 4 邻接直接
  接触 `[A]`；
- 同一个 initial region 被 gradient edge 切开的 component 永不回并；
- candidate 同时要求 `M2>0`、7×7 dilation 邻接和深度相似度通过；
- 合并后重新计算 mask、直方图、面积/深度与候选分数，不累加旧矩阵；
- 未合并的小区域保留，不丢入 invalid；
- 不产生 dynamic/static 判决。

## 2. 工程验证

- `sin_style_shadow_test` 与 `rgbd_tum` 构建通过；
- 原配置默认关闭 smoke 运行 30 帧，未出现 SIn 输出，`deadline_missed=0/30`、
  `actual_fps=27.93`；
- 合成测试覆盖：同深度 fake-boundary 合并、真实 gradient 不跨越、明显
  深度差拒绝、invalid gap 不连接、重复输出确定；
- TUM3 walking 30 帧运行两次，RAG PNG 逐像素一致 30/30；
- 上游 gradient audit 继续通过；
- RAG CSV、区域数、merge 数、core support、label 连续性、gradient 身份和
  runtime invariant 全部通过；
- `cross_gradient_merge_violations=0`；
- `dynamic_decision=none`、`actual_slam_removed=0`、
  `direct_slam_state_mutation=none`。

## 3. 30 帧结果

### 3.1 区域结构

| 指标 | 结果 |
| --- | ---: |
| gradient split component 中位数 | 72.0 |
| RAG output region 中位数 | 60.5 |
| 区域数平均下降 | 15.52% |
| 中高分 merge / 帧 | 10.00 |
| 低分 merge / 帧 | 1.10 |
| 未合并低分区域 / 帧 | 20.03 |
| 初始 shared-fake-edge pair / 帧 | 39.10 |
| 初始 depth-rejected pair / 帧 | 14.73 |
| 初始 eligible pair / 帧 | 23.40 |
| 最大 group component 数（帧均值） | 6.30 |
| cross-gradient violation | 0 |

RAG 有选择地合并了一部分由初始 K-means 人工边界分开的同深度区域，没有
出现全图塌缩。但输出仍约 60 个区域，明显高于作者 final partition 的约
11 个 positive labels；因此 gradient-only RAG 仍然欠合并，不能称为对象
区域。

### 3.2 与作者 final partition 的描述性对照

| 指标 | gradient split | gradient-only RAG |
| --- | ---: | ---: |
| ARI | 0.5567 | 0.7438 |
| NMI | 0.7892 | 0.8338 |
| boundary precision @2px | 0.3421 | 0.3330 |
| author boundary recall @2px | 0.8255 | 0.8405 |

RAG 的区域成员关系更接近作者 final partition，说明合并并非随机；边界
precision 没有提高，且作者标签不是 GT，不能据此宣称物体分割质量已经
通过。

### 3.3 运行成本

| 部分 | 30 帧均值 |
| --- | ---: |
| RAG attribute / graph preparation | 23.04 ms |
| initial pair scoring | 0.86 ms |
| iterative merge | 4.71 ms |
| RAG total | 29.02 ms |
| 端到端 active total | 96.59 ms |
| 进程 actual FPS | 10.35 |

第二次运行的端到端 active total 为 96.13 ms、actual FPS 为 10.40，量级
一致。当前 attribute/preparation 为正确性优先实现，包含每区域 full-image
mask 和 dilation；本轮不把它包装成实时版本。PNG 写盘和 author reference
replay 也包含在端到端数字中。

## 4. 客观结论

通过的是：

> 在严格禁止跨 gradient 回并的条件下，论文式 RAG 结构能够确定、有限地
> 修复 initial K-means 的部分人工分界，并显著提高与作者 final partition
> 的描述性成员一致性。

没有通过或尚未测试的是：

- 完整几何重聚类（plane edge 仍缺失）；
- 区域是否对应物体；
- 动态/static/unknown 判决；
- dense-flow residual 与时序先验；
- 未知箱子 mask；
- Tracking、ATE、MapPoint 或深度地图改善。

因此本轮不会开放 S2。下一步仍在 S1：核对并实现 plane-edge 支持，或在
明确记录缺失条件下进入同区域上的 dense-flow residual 行为对照；不能把
当前 60.5 个区域直接当成动态对象。

## 5. 证据

- `S1_RAG_MERGE_LITERATURE_AND_SOURCE_AUDIT.md`；
- `S1_GRADIENT_ONLY_RAG_MERGE_SHADOW_SPEC.md`；
- `rag_merge_30.csv/.log` 与 repeat；
- `rag_merge_30_audit.json`；
- `rag_merge_30_gradient_audit.json`；
- `rag_merge_30_initial_labels/`；
- `rag_merge_30_gradient_outputs/`；
- `rag_merge_30_outputs/` 及 repeat 目录。
