# G2-4F4 区域上下文连续残差：文献决策

日期：2026-07-30
状态：路线选择完成；尚未实现
前提：G2-4F2 hard candidate 失败，G2-4F3/F3U 局部刚性增强未形成可靠判决

## 1. 当前证据要求解决什么

现有实验已经稳定区分出两件事：

```text
F1 continuous sparse ego-flow residual:
  moving proxy 局部富集方向存在

F2 whole-frame normalized hard candidate:
  strict holdout sensitivity / safety gate 失败
```

F3/F3U 说明两帧局部边长一致性不能可靠替代对象边界、对应可见性或区域上下文。
因此下一步不能继续堆叠 edge score，也不能继续调整 F2 的 `q/FB` 阈值。

需要检验的最小问题是：

> 把已经存在的连续 F1 residual 限制在独立生成的深度区域中做鲁棒聚合，能否
> 保留 moving proxy 的局部富集，同时明确暴露区域泄漏、低支持量和静态背景
> 风险？

## 2. 本地资料与原始论文核对

优先读取：

```text
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/detectfusion.md
/home/zhu/Desktop/papers/
  Qi 等 - 2025 - Semantic-Independent Dynamic SLAM Based on
  Geometric Re-Clustering and Optical Flow Residuals.pdf
/home/zhu/Desktop/papers/
  2019_DetectFusion_Known_Unknown_Dynamic_Objects.pdf
```

### 2.1 SInDSLAM `[L]`

原论文先形成几何 re-cluster，再计算 dense optical-flow residual。高/低 residual
判断和 residual-aware filling 被限制在单个 cluster 内；论文明确指出这种限制
用于减少假阳性。

原方法同时包含：

- 三维 K-means；
- 深度梯度和平面边缘；
- 深度直方图 RAG 重聚类；
- dense Brox/DIP flow；
- PROSAC homography；
- Triangle 双阈值；
- 时序动态先验。

当前工程没有复现这些组件。

### 2.2 DetectFusion `[L]`

原论文使用 normal 与 distance discontinuity 产生 geometry segments，再将
ICP residual 的 binary motion mask 与 geometry segments 做 IoU，以扩展局部
运动证据并排除 segment edge 上的伪响应。

原 residual 来自静态 surfel map 的 ICP registration，不是当前相邻帧稀疏
ego-flow。

### 2.3 当前可借用的共同原理 `[A]`

两篇论文共同支持：

```text
motion residual 不应脱离独立几何区域直接全图传播；
区域边界和区域内支持量必须与 residual 分开建模。
```

它们不支持直接复制原阈值，也不证明当前轻量深度连通区域就是对象实例。

## 3. 三条候选路线比较

| 路线 | 当前依据 | 当前主要缺口 | 决策 |
| --- | --- | --- | --- |
| 继续改 LK/FB 对应可靠性 | Kalal FB consistency 已在 F2 使用 | holdout 失败后继续调 FB 会泄漏 | 否决 |
| 增加遮挡/深度边界 veto | DynaSLAM parallax/depth-patch；F1 已有风险字段 | 只能删风险点，不能恢复对象级支持 | 保留为诊断，不单独成阶段 |
| 区域内聚合连续 F1 residual | SInDSLAM、DetectFusion | 当前 G2-3R0 不是完整 object clustering | **批准最小 shadow 审计** |

## 4. 批准的最小适配

G2-4F4 第一版只做离线、确定性的 representation audit：

```text
exact C++ F1/F3 node CSV
+ 同帧 rectified CV_32F depth
→ G2-3R0 depth-discontinuity partition
→ feature 到 region label 的映射
→ 每 region 的 residual median / P90 / MAD / support
→ frozen RGB-only coarse bbox 的成对 proxy 审计
```

方法身份：

> `[A/S/H]` Region-constrained aggregation of the existing sparse ego-flow
> residual, inspired by the region-context principle of SInDSLAM and
> DetectFusion, using the already implemented lightweight depth-discontinuity
> partition.

它不是：

```text
SInDSLAM reproduction
DetectFusion reproduction
object segmentation
dynamic classifier
pixel motion ground truth
```

## 5. 为什么先离线而不是改 Tracking

当前 exact C++ CSV 已保存：

- feature 坐标；
- semantic exclusion；
- forward-backward quality；
- continuous flow residual；
- current depth validity。

Bonn archive 和冻结 candidate/bbox 也已存在。因此可以在不重新运行 SLAM、不
读取 holdout 调参、不增加在线耗时的情况下先回答 representation 是否值得。

只有离线 representation audit 通过，才讨论是否需要把 region mapping 放回
online shadow。

## 6. 禁止项

```text
不选择 residual threshold
不把 bbox 当 pixel/object GT
不把 region label 当 dynamic
不读取 sealed balloon_tracking 回调规则
不修改 mvbDynamic、mvpMapPoints 或 MapPoint
不修改 YOLO、Optimizer、g2o 或后端
不增加 PoseOptimization
不重新启用 G2-3R4 低分辨率区域路径
```
