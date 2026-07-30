# G2-4 运动一致性分组：Dai 点关联与几何分段路线决策

日期：2026-07-30
状态：文献与工程对照完成；选择下一项 representation shadow；G1 继续锁定

## 1. 决策

下一项只批准：

> **G2-4R1：Tateno/DetectFusion-style normal+distance frame-wise
> geometric segmentation representation audit。**

第一版为离线、确定性、只读审计，不进入 `Tracking`，不输出动态类别，不修改
SLAM 状态。

暂不实现：

> Dai-style point-correlation grouping on transient LK nodes。

这里没有否定 Dai 的方法。延期原因是当前动态代理内已有 `MapPoint` 只占
`3.28%–10.53%`，而把 Dai 的 tracked MapPoint、三维 edge state、协方差和
point-correlation optimization 替换成 transient LK node，已经是显著改造，
不是当前最小可验证子集。

## 2. 这一步服务于什么

当前最强类别无关运动证据仍是：

```text
G2-4F1 sparse observed flow - RGB-D/SE(3) ego flow
```

但现有实验说明：

- 单点 hard gate 未通过 strict holdout；
- 两帧局部刚性及其不确定度归一化没有形成可靠判决；
- 简单 depth-discontinuity component 会跨入背景，稀释 F1 residual；
- 稀疏长轨迹和稳定 MapPoint 对未知动态代理的支持不足；
- 稀疏路线即使成功，也不能独立输出低纹理物体所需的 `M_depth`。

因此当前缺口不是再增加一个 residual，而是先获得比 G2-3R0 更可信、且与运动
证据独立生成的几何区域表示。

G2-4R1 只回答：

> normal+distance 分段是否能减少当前 component 的背景泄漏，同时保留箱体/
> 气球区域和低纹理深度表面？

它还不回答：

> 哪个 segment 正在运动？

## 3. 原始文献核对

### 3.1 Dai et al. point correlations `[L]`

本地原文：

```text
results/g2_4f3u_2026-07-30/literature/
  Dai_2022_Point_Correlations_arXiv1811.03217.pdf
```

原方法前端：

1. 对上一帧成功跟踪的 `MapPoint` 在图像域做 Delaunay；
2. edge state 是地图点间三维相对位置；
3. 用 RGB-D 测量协方差建立 point-correlation objective；
4. 迭代优化并按 Mahalanobis/chi-square 剔除不一致 edge；
5. DFS 得到 connected components；
6. 以三维空间体积最大的 component 作为可靠静态组；
7. 该前端确实只用相邻两帧测量，但后端另有滑窗复核。

当前 F3/F3U 只复用了：

```text
Delaunay adjacency
+ adjacent-frame 3-D edge-length change
+ depth uncertainty diagnosis
```

当前没有复现：

```text
edge state optimization
Mahalanobis iterative culling
connected-component motion groups
largest-volume static selection
backend sliding-window verification
```

所以不能把“给 F3 再加 DFS”称为 Dai reproduction。

### 3.2 Tateno frame-wise depth segmentation `[L]`

Tateno 2015 的完整工作是增量三维分段系统，包含：

```text
frame-wise depth segmentation
segment label propagation
segment merging
global segment-map update
```

它明确允许替换 frame-wise segmentation。论文实验中，深度先转换为 vertex map，
做 bilateral smoothing，再以 central differences 生成 normal map。

其帧级分段使用：

1. 八邻域 concavity-aware normal operator；
2. 八邻域三维 point-to-plane distance operator；
3. 深度噪声自适应的 distance threshold；
4. normal edge 与 geometric edge 的并集；
5. connected components 生成 label map；
6. 边界、无效深度和未标注像素不属于任何可靠 segment。

2017 扩展论文显式给出：

\[
\Phi_i(u)=
\begin{cases}
1,&(v(u_i)-v(u))^\top n(u)>0\\
n(u)^\top n(u_i),&\text{otherwise}
\end{cases}
\]

\[
\Phi(u)=\min_{i=1,\ldots,8}\Phi_i(u),
\qquad \tau_\Phi=0.94,
\]

以及：

\[
\Gamma(u)=
\max_{i=1,\ldots,8}
\left|(v(u_i)-v(u))^\top n(u)\right|.
\]

`Γ` 使用随深度增长的测量不确定度做阈值。Tateno 引用的 Nguyen Kinect
轴向噪声模型在其主要有效角度范围内为：

\[
\sigma_z(z)=0.0012+0.0019(z-0.4)^2\ {\rm m}.
\]

本地原文：

```text
results/g2_4_motion_grouping_2026-07-30/
  Nguyen_2012_Kinect_Sensor_Noise.pdf
```

注意：该 Kinect 模型不能未经验证地解释为所有 RGB-D 相机的真实噪声；在
G2-4R1 中它只是固定的文献参数，不做数据驱动调参。

### 3.3 DetectFusion `[L]`

本地原文：

```text
/home/zhu/Desktop/papers/
  2019_DetectFusion_Known_Unknown_Dynamic_Objects.pdf
```

DetectFusion 使用 Tateno-style normal+distance geometric segments，但其未知
动态分支还依赖：

```text
static surfel map rendering
→ ICP geometric residual mask
→ residual K-means(K=2)
→ binary motion mask
→ 与 geometric segments 做 IoU 扩展
```

因此：

- DetectFusion 的 unknown branch 不需要未知对象的 YOLO proposal；
- 但只有 normal+distance segment 并不能判断运动；
- 当前 F1/G2 residual 与 DetectFusion 的 static-map ICP residual 不同；
- G2-4R1 不是 DetectFusion reproduction。

## 4. 两条候选的工程对照

| 维度 | Dai two-frame point correlation | Tateno/DetectFusion frame-wise segment |
| --- | --- | --- |
| 原始输入 | tracked MapPoint、两帧 RGB-D、协方差 | 当前 metric depth、内参 |
| 当前可复用 | F1 LK、F3 Delaunay/3D edge/noise | rectified depth、K、现有 component 审计工具 |
| 关键缺口 | MapPoint 支持、edge optimization、CC/static group | normals、point-to-plane edge、noise-adaptive edge |
| 可自然输出 | 高置信 `D_feat` | region labels，未来可形成 `M_depth` |
| 低纹理支持 | 弱 | 较强，但依赖有效深度和几何边界 |
| 当前主要风险 | transient LK 改造过大；最大组静态假设 | 过分割；相邻凸表面可能合并；传感器噪声模型迁移 |
| 与现有负实验关系 | F3/F3U 已显示简化 edge 指标不足 | F4 只否定简单 depth component，尚未测试 normal+distance |
| 核心代码增量 | 中到高，并易继续挤入 Tracking | 中；可先完全离线 |
| 本轮决策 | 延期 | **批准 representation shadow** |

## 5. 运行时间表述修正

不能把 DetectFusion 的 `6.16 ms` 直接当作当前算法预算。

- Tateno 2015 在 CPU 上报告：
  - `160×120`：depth segmentation `1.81 ms`，完整增量分段 `3.52 ms`；
  - `320×240`：`7.63 ms` / `13.82 ms`；
  - `640×480`：`32.39 ms` / `63.58 ms`。
- DetectFusion 在其整套系统和硬件上报告 geometric segmentation
  `6.16 ms`，完整系统约 `22 FPS`。

两者不是同一可直接搬用的耗时口径。当前用户已放宽 `30 FPS` 硬约束，因此本轮
先测表示质量，再报告实际 CPU 时间，不因预估超过 `33.3 ms` 提前否决。

## 6. 方法身份

下一阶段准确身份：

> `[L/A]` Frame-wise normal- and distance-discontinuity segmentation
> adapted from Tateno and used as geometric support in DetectFusion.

其中：

- `[L]` vertex/normal、`\Phi`、`\Gamma`、noise-aware edge 和 CC 的核心表达；
- `[A]` 仅取单帧分段，不实现全局 segment map；
- `[S]` 在 DT-SLAM rectified `P=K` 域中的输入输出、unknown 状态和审计指标；
- `[H]` 该表示能减少当前 F4 背景泄漏并保留未知动态区域。

不得表述为：

```text
Tateno reproduction
DetectFusion reproduction
unknown dynamic object detector
object identity segmentation
```

## 7. 停止边界

G2-4R1 完成前继续禁止：

```text
dynamic threshold
mvbDynamic / mvpMapPoints mutation
G1-F / G1-D
Optimizer / g2o / backend changes
third PoseOptimization
holdout tuning
```

如果 normal+distance segment 仍在代表失败帧中严重泄漏或把目标碎成不可用
小块，应将其归档为负结果；不继续靠 merge threshold、area threshold 或 residual
selector 修补。届时再回到“Dai 完整子集是否值得承担更高改造成本”的决策。

## 8. 参考

- Dai et al., *RGB-D SLAM in Dynamic Environments Using Point
  Correlations*: <https://arxiv.org/abs/1811.03217>
- Tateno et al., *Real-Time and Scalable Incremental Segmentation on Dense
  SLAM*: <https://doi.org/10.1109/IROS.2015.7354011>
- Tateno et al., *Large Scale and Long Standing Simultaneous Reconstruction
  and Segmentation*: <https://doi.org/10.1016/j.cviu.2016.05.013>
- Hachiuma et al., *DetectFusion*: <https://arxiv.org/abs/1907.09127>
- Nguyen et al., *Modeling Kinect Sensor Noise for Improved 3D
  Reconstruction and Tracking*:
  <https://users.cecs.anu.edu.au/~nguyen/papers/conferences/Nguyen2012-ModelingKinectSensorNoise.pdf>
