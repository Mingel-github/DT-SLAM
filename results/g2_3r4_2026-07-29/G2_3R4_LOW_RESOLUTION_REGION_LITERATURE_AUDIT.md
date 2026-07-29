# G2-3R4 低分辨率区域表示文献边界审计

日期：2026-07-29
状态：实现前审计；尚未修改代码或运行 G2-3R4 实验

## 1. 审计问题

G2-3R4 只研究：

```text
现有 scale-2 boundary-preserving depth
→ 在 320×240 域运行当前深度不连续区域划分
→ 将区域关系映射到 640×480 ORB/mask 域
→ 与当前 full-resolution partition 做近似保真和成本审计
```

本审计不为上述整体组合寻找“论文背书”。它只区分：

- `[L]` 原始文献明确做了什么；
- `[A]` DT-SLAM 对文献组件做了什么改造；
- `[S]` 本项目自己的工程设计；
- `[H]` 必须由实验验证的假设。

## 2. 来源和优先级

本轮按以下顺序核对：

1. 本地 PaperNotes；
2. 本地原始 PDF；
3. 本地缺失时使用作者机构或出版方提供的原始论文。

本地材料：

```text
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/detectfusion.md
/home/zhu/Desktop/papers/Qi 等 - 2025 - Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering and Optical Flow Residuals.pdf
/home/zhu/Desktop/papers/2019_DetectFusion_Known_Unknown_Dynamic_Objects.pdf
```

KinectFusion 本地没有原始 PDF。本轮核对的是 Microsoft Research 提供的原文：

```text
Newcombe et al., KinectFusion: Real-Time Dense Surface Mapping and Tracking,
ISMAR 2011.
https://www.microsoft.com/en-us/research/publication/
kinectfusion-real-time-dense-surface-mapping-tracking/

原始 PDF：
https://www.microsoft.com/en-us/research/wp-content/uploads/2016/11/ismar_2011.pdf
```

未使用二手综述为 G2-3R4 的方法归属背书。

## 3. KinectFusion

### 3.1 原文事实 `[L]`

KinectFusion：

- 对原始深度先做保留深度不连续性的 bilateral filtering；
- 构造三层 depth/vertex/normal pyramid；
- 下一层深度由 block averaging 后 2× subsampling 得到；
- 只有与中心像素深度差在 `3σr` 内的值参与平均，以避免跨深度边界平滑；
- 使用对应层的深度生成 vertex/normal map；
- 多尺度表示服务于 live depth 对全局 TSDF surface prediction 的
  coarse-to-fine ICP tracking。

原文没有：

- 低分辨率动态区域划分；
- 低分辨率 region label 上采样；
- temporal multi-reference depth residual；
- 用低分辨率 region 替代 full-resolution region；
- ORB 特征或语义 mask 映射。

### 3.2 DT-SLAM 改造 `[A]`

G2-3R3 保留了：

- 2× 深度下采样；
- block 内边界保持平均；
- 同步缩放相机内参；
- 在低分辨率域运行 dense projective computation。

G2-3R3 改变了：

- 使用 2×2 左上像素作为 anchor，而不是复现原文中心像素定义；
- 使用
  `max(0.025 * anchor_depth, 0.08 m)`，
  而不是 KinectFusion 的 `3σr`；
- 计算 temporal multi-reference evidence，而不是 ICP；
- 不使用 TSDF、vertex/normal pyramid 或 coarse-to-fine pose optimization。

### 3.3 对 G2-3R4 的边界

KinectFusion 只能支持：

```text
[L/A] 边界保持的低分辨率深度预处理是成熟的多尺度计算组件。
```

它不能支持：

```text
[S/H] 在低分辨率域运行当前 region partition；
[S/H] 将 region label/evidence 映射回 640×480；
[H] 该近似可替代 full-resolution partition。
```

## 4. SInDSLAM

### 4.1 原文事实 `[L]`

SInDSLAM 的 Image Clustering：

- 将深度像素反投影到 3D 后运行 K-means；
- 构造 coarse-to-fine depth Gaussian pyramid，加速 K-means 收敛；
- 粗层 clustering result 用于初始化细层；
- 顶层使用上一帧 clustering result 初始化；
- 使用
  `δ_depth > max(τ1 * depth, τ2)`
  提取 gradient depth edge；
- 另提取 plane edge；
- 用边缘切分欠分割 cluster；
- 用 RAG、fake edge、深度直方图和合并权重修补过分割。

SInDSLAM 明确承认：

- 边缘参数会影响过分割和欠分割；
- geometric re-clustering 不能保证完美；
- 远距离和小区域容易出现 cluttered clustering；
- 完整系统约 117.8 ms/frame，广泛图像遍历和 morphology 是主要成本之一。

其 Gaussian pyramid 用于 K-means 初始化，不是：

- 低分辨率 temporal dynamic evidence；
- 低分辨率 connected-component partition；
- full-resolution region 的替代实现。

### 4.2 DT-SLAM 改造 `[A]`

G2-3R0 只保留了 relative-plus-absolute depth boundary 的形式，并在
full-resolution 深度上执行：

```text
depth discontinuity mask
→ 排除 boundary pixel
→ 4-neighbour connected components
```

DT-SLAM 没有复制：

- K-means；
- Gaussian-pyramid K-means 初始化；
- plane edge；
- RAG；
- fake edge；
- depth-histogram re-clustering；
- optical flow residual；
- dynamic region decision。

因此当前区域表示是轻量改造，不是 SInDSLAM reproduction。

### 4.3 对 G2-3R4 的边界

SInDSLAM 只能支持：

```text
[L] 多尺度深度处理可以服务几何 clustering；
[L] 深度边界和区域粒度会产生过分割/欠分割风险；
[L] 图像遍历和 morphology 可能成为显著成本。
```

它不能支持：

```text
[S/H] 对 G2-3R0 connected components 做 scale-2 替代；
[S/H] 直接上采样 region label；
[H] 该替代保留当前区域结构或动态区分能力。
```

## 5. DetectFusion

### 5.1 原文事实 `[L]`

DetectFusion：

- 使用 Tateno et al. 的几何分割；
- 依据邻域 normal 和 distance discontinuity 生成 normal/edge mask；
- 合并二者并运行 connected-component analysis，得到每像素 segment label；
- 用 segment 与 YOLO bounding box 的 IoU 生成已知对象 instance mask；
- 用 ICP residual K-means 得到 binary motion mask；
- 再将 motion mask 与 geometric segments 做 IoU，扩展未知运动区域；
- 三层 coarse-to-fine image pyramid 用于 photometric/geometric tracking；
- 640×480 geometric segmentation 报告为约 6.16 ms。

DetectFusion 同时指出，bounding-box 与 geometric segment 的启发式匹配会在
复杂遮挡、小物体和非凸物体上失败。

### 5.2 DT-SLAM 借用边界 `[A]`

G2-3R1 只借用了：

```text
motion evidence 与固定 geometric segment 做区域级重叠/聚合
```

DT-SLAM 没有：

- surfel 静态地图；
- ICP motion mask；
- K-means 动静二分；
- normal-based segmentation；
- object map；
- 两遍 tracking；
- DetectFusion 的 segment-to-object assignment。

DetectFusion 的 tracking pyramid 不能为 G2-3R4 region approximation 背书。

## 6. 文献来源账本

| 组件 | 分类 | 原方法 | DT-SLAM 保留/改变 | G2-3R4 可否依赖 |
| --- | --- | --- | --- | --- |
| boundary-preserving depth pyramid | `[L/A]` | KinectFusion 为 vertex/normal pyramid 和 ICP 做边界保持 block average | 改为 scale-2、左上 anchor、`0.025/0.08m`、temporal evidence | 只能支持低分辨率深度预处理原则 |
| depth Gaussian pyramid | `[L]` | SInDSLAM 为 K-means coarse-to-fine 初始化 | G2-3R4 不实现 K-means | 不能支持低分辨率 region 替代 |
| relative-plus-absolute depth edge | `[L/A]` | SInDSLAM gradient edge 的阈值形式 | G2-3R0 用于 boundary mask + connected components | 可保留同一阈值形式，但不是完整重聚类 |
| geometric segment + motion evidence | `[L/A]` | DetectFusion 将 ICP motion mask 与 geometric segments 做 IoU | G2-3R1 只聚合 multi-reference evidence | 只能支持区域聚合结构 |
| scale-2 connected-component partition | `[S/H]` | 无对应原方法 | 在现有 half-depth 上运行 G2-3R0 partition | 必须实验验证 |
| region label 映射到 640×480 | `[S/H]` | 无对应原方法 | full pixel/ORB 映射到所属 half cell | 必须实验验证 |
| 替代 full-resolution partition | `[H]` | 无对应原方法 | 以结构损失换约 3 ms | 必须实验验证 |

## 7. 不能由文献推出的结论

以下说法禁止出现：

```text
G2-3R4 reproduces KinectFusion/SInDSLAM/DetectFusion.
文献证明低分辨率 region 与 full-resolution region 等价。
文献证明低分辨率 region 可以检测动态对象。
与 full-resolution partition 一致就等于对象区域正确。
```

当前 full-resolution partition 本身只是 G2-3R0 的高分辨率参考实现，不是
object-region GT。

## 8. G2-3R4 的研究性质

G2-3R4 的准确分类是：

```text
[L/A] 复用已经核查的 boundary-preserving scale-2 depth；
[A]   复用 G2-3R0 的轻量 depth-boundary partition；
[S]   在 native half-resolution cell 域聚合 evidence；
[S]   以 label-permutation-invariant 指标比较两种 partition；
[H]   结构损失可以接受；
[H]   能净回收约 3 ms 中足够大的部分；
[H]   该表示值得保留到后续动态/静态区分实验。
```

## 9. 审计结论

文献核对允许开展 G2-3R4，但只允许把它描述为：

> 一次有界的低分辨率区域近似与成本审计。

它不能被描述为完整文献方法、动态分割方法或实时性解决方案。G2-3R4 通过后，
下一门槛应是动态/静态区分能力，而不是默认开启新的长期 CPU 优化路线。
