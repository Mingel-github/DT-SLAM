# G2-4F 直接 ORB Feature Evidence 文献与方法审计

日期：2026-07-29
范围：决定 G2-4E 之后的最小 shadow 实验；不实现动态判决或 SLAM 过滤。

## 1. 结论

G2-4E 已否决“当前固定 depth region 聚合已经足以隔离未知箱子”，但没有否定
multi-reference depth-warp 的直接像素证据。

下一步应先做：

```text
G2-4F0 =
已有 multi-reference comparison/positive/negative/consistent counts
→ 在当前 ORB keypoint 中心只读采样
→ 与独立 RGB-only box 粗框和真实 person mask 做 development 审计
```

这不是新动态检测算法，而是把已有 G2 证据投影到未来 G1-F 的实际观测单位，
回答：

> 去掉失败的 region aggregation 后，是否存在一个小而局部的 ORB feature
> evidence 子集值得继续研究？

本阶段不选择任何 vote threshold，不把 bbox 当 motion/pixel GT，也不修改
`mvbDynamic` 或 `mvpMapPoints`。

## 2. 本地材料

优先核对：

```text
/home/zhu/Desktop/paper_notes/Ji2021_RealTime_Semantic_RGBD_SLAM.md
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/detectfusion.md
/home/zhu/Desktop/paper_notes/ngd_slam.md
/home/zhu/Desktop/paper_notes/dvi_slam.md
/home/zhu/Desktop/papers/Ji 等 - 2021 -
  Towards Real-time Semantic RGB-D SLAM in Dynamic Environments.pdf
/home/zhu/Desktop/papers/Qi 等 - 2025 -
  Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering
  and Optical Flow Residuals.pdf
/home/zhu/Desktop/papers/2019_DetectFusion_Known_Unknown_Dynamic_Objects.pdf
```

并复核当前项目已经完成的 Ji baseline：

```text
results/gj_2026-07-28/GJ1_DEPTH_CLUSTERING_RESULT.md
results/gj2_2026-07-28/GJ2_CLUSTER_REPROJECTION_RESULT.md
results/gj2a_2026-07-28/GJ2A_CLUSTER_PROXY_AUDIT_RESULT.md
results/gj3_literature_audit_2026-07-28/GJ3_PARAMETER_PROVENANCE_AUDIT.md
results/gj3a_2026-07-28/GJ3A_THRESHOLD_SHADOW_RESULT.md
```

## 3. 来源账本

### 3.1 Ji et al., ICRA 2021

`[L]` 论文对深度图做三维 K-means，并在每个 cluster 中聚合已匹配
MapPoint/ORB observation 的重投影误差。它支持：

- feature-associated geometry 是合理的前端观测单位；
- 无地图匹配支持的区域没有该类几何证据；
- cluster error 是证据，不应与单个 optimizer outlier 等同。

它不直接支持本阶段的 multi-reference depth-warp vote；G2-4F0 只能称为
`[S]` 将已有 G2 证据映射到 ORB feature 的诊断。

当前 GJ 已经证明：

- 全分辨率 CPU K-means 约 `54–58 ms/frame`；
- 公开材料没有 `rho`、最小支持数和动态阈值；
- 相对阈值适配会在静态序列选择约一半区域。

因此不能把“再做一次 Ji”作为主路线，也不能用 Ji 名义补一个 feature threshold。

### 3.2 SInDSLAM, TCSVT 2025

`[L]` 原文使用：

- 几何重聚类形成对象性更强的区域；
- dense optical flow 与 homography 的 residual；
- Triangle 双阈值；
- residual-aware flood fill 被限制在单个 cluster 内；
- 每五帧的 depth reprojection 用于慢速/间歇运动的地图 mask 精修。

原文明确说明 flood fill 被限制在单个 cluster 内以降低 false positive。它支持
G2-4E 的反面结论：

```text
运动残差不能沿任意大 depth-connected component 无界传播。
```

但完整 SInDSLAM 包含 K-means、边缘切分、RAG 合并、dense optical flow、
PROSAC、形态学遍历和 OctoMap；论文报告 `117.8 ms/frame (9 Hz)`。它不能作为
当前 30 FPS 项目的下一步整块移植。

### 3.3 DetectFusion, BMVC 2019

`[L]` DetectFusion 用法线和深度不连续性生成几何 segments，再用静态
surfel map 的 ICP residual 做 K=2 motion segmentation，并通过 segment IoU
扩展运动区域。

它支持：

- 对象性区域需要法线/深度边界等比单纯局部深度连通更强的约束；
- 运动 residual 和几何 segment 应是两个不同信号。

它不适合直接搬入当前 ORB-SLAM2：

- 当前没有静态 surfel/TSDF model；
- 没有 dense ICP registration residual；
- 没有对象地图；
- 完整系统约 22 FPS，且依赖 GTX 1080 Ti。

因此不能把当前 temporal depth-warp residual 改名为 DetectFusion residual，
也不能用 K=2 强制每帧产生 dynamic class。

### 3.4 NGD-SLAM 与 DVI-SLAM

`[L]` NGD-SLAM 的 LK/DBSCAN 主要传播已有 YOLO mask；DVI-SLAM 的
LK/homography 主要验证 semantic candidate。两者都支持光流是低成本的
时序运动工具，但不直接证明它能从零发现 unknown object。

所以：

- 光流可以保留为 failure-driven 辅助证据；
- 不能把 semantic-conditioned mask propagation 当成类别无关发现方法；
- 在 G2-4F0 之前加入 LK/DBSCAN 会同时改变证据与区域表示，无法定位失败来源。

## 4. 为什么先做 direct feature audit

G2-4E 已发现：

```text
bbox dominant depth region score ≈ whole-frame score
```

这说明当前 region label 破坏了局部性。ORB-SLAM2 的 G1-F 最终操作单位却是
feature/map-point association，而不是整块深度 region。

G2-4F0 只回答三件事：

1. ORB feature 中有多少获得 multi-reference comparison；
2. box 粗框内、person mask 外的 feature evidence 是否相对背景富集；
3. evidence 是否只来自少量重复的 scale-2 native cells、边界或无效深度邻域。

如果 direct feature evidence 没有局部富集，就没有理由继续设计 feature
threshold；应进入有文献依据的稀疏自运动补偿 flow 或对象性更强的 region
representation。

如果有富集，也仍需独立 motion-state proxy/label，不能立即进入 G1-F。

## 5. 方法身份

| 组件 | 身份 |
| --- | --- |
| 多参考有符号 depth residual | `[A]` 受 DynaSLAM 多视图一致性启发 |
| comparison/positive/negative/consistent vote | `[S]` 当前项目证据表示 |
| 在 ORB center 读取 vote | `[S]` 诊断映射，不是论文方法 |
| scale-2 native cell 去重复审计 | `[S]` 防止把 2×2 展开误当独立测量 |
| RGB-only coarse bbox | `[S]` unverified development proxy |
| person mask exclusion | `[S]` 当前语义分支实际输出 |
| dynamic threshold | 不存在 |
| feature filtering | 不存在 |

## 6. 驳回的立即行动

- 重做 Ji K-means/reprojection；
- 在当前 region score 上继续调组合阈值；
- 直接实现完整 SInDSLAM re-clustering；
- 搬入 DetectFusion ICP/surfel pipeline；
- 直接加入 dense optical flow；
- 以 bbox visibility 代替 object motion；
- 解封 strict hold-out；
- 让 G2 evidence 写入 SLAM 状态。

## 7. G2-4F0 后的分叉

| 结果 | 后续 |
| --- | --- |
| box-local feature evidence 无富集 | 冻结为负结果；审计稀疏 ego-flow 或更强对象区域 |
| 有富集但只有边界/invalid 风险 | 不放行；先做风险独立性与 motion-state proxy |
| 有富集且跨多 native cells/多参考稳定 | 再冻结独立 motion-label protocol |
| motion label 与 feature evidence 都通过 | 才设计 G1-F 最小判决门 |

无论哪种结果，G2-4F0 都不改变 ATE；只有 G1-F 真正进入过滤后才能评价几何
ATE/RPE 改善。
