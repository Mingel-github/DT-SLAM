# G2-4R1 Normal+Distance 几何分段表示 Shadow SPEC

日期：2026-07-30
状态：SPEC 冻结；尚未实现
运行身份：offline、development-only、shadow-only

## 1. 唯一问题

> 在不读取语义 mask、F1 residual 或动态代理来生成分段的前提下，
> Tateno-style normal+distance frame-wise segmentation 是否比当前
> G2-3R0 depth-discontinuity component 更少泄漏静态背景，并为低纹理未知
> 动态提供可用的深度区域容器？

本阶段只审计 `region representation`，不检测动态。

## 2. 输入

算法输入仅为：

```text
rectified CV_32FC1 depth in meters
rectified pinhole K (P=K domain)
```

开发审计可另外读取，但不得参与分段生成：

```text
frozen Bonn balloon/balloon2 candidate frames
frozen RGB-only coarse bbox
exact C++ F1 eligible feature coordinates/residual
existing G2-3R0 labels
```

禁止读取：

```text
YOLO/person mask as segmentation input
F1/G2 residual as edge or merge input
balloon_tracking sealed holdout
future ATE result
```

## 3. 冻结算法

### 3.1 有效性与 unknown

```text
invalid depth                 = unknown
normal unavailable           = unknown
normal/distance boundary      = boundary, not a segment
valid non-boundary component  = geometric segment
```

不得把 invalid、boundary 或 normal-unavailable 像素记为静态。

### 3.2 Vertex 与 normal

按针孔模型反投影：

\[
v(u)=D(u)K^{-1}\tilde u.
\]

遵循 Tateno 的预处理次序：

```text
metric vertex/depth
→ edge-preserving bilateral smoothing
→ central-difference normal
→ unit normalization
```

文献未给出的 bilateral 实现细节必须作为 `[S]` 参数完整记录，第一轮固定一次，
不得根据 balloon/balloon2 的 residual 结果调节。

### 3.3 Concavity-aware normal edge

对八邻域：

\[
\Phi_i(u)=
\begin{cases}
1,&(v(u_i)-v(u))^\top n(u)>0\\
n(u)^\top n(u_i),&\text{otherwise}
\end{cases}
\]

\[
\Phi(u)=\min_i\Phi_i(u).
\]

第一轮使用原论文固定值：

```text
tau_phi = 0.94
```

`\Phi(u) < tau_phi` 记为 normal/concavity boundary。

### 3.4 Point-to-plane distance edge

\[
\Gamma(u)=
\max_i |(v(u_i)-v(u))^\top n(u)|.
\]

第一轮使用 Tateno 引用的 Nguyen axial-noise 形式：

\[
\sigma_z(z)=0.0012+0.0019(z-0.4)^2\ {\rm m}.
\]

固定规则：

```text
Gamma(u) > sigma_z(D(u))  → distance boundary
```

该噪声模型在 Bonn 上只是文献参数迁移实验，不解释为传感器标定真值；单独记录
`noise_model_out_of_domain`。

### 3.5 Labels

```text
final boundary = normal boundary OR distance boundary
```

在八邻域 edge 定义后，对 valid、normal-valid、non-boundary 像素做四连通
connected components，输出 `CV_32SC1` labels。

标签规范：

```text
-3 = invalid depth / unknown
-2 = normal unavailable / unknown
-1 = geometric boundary
>=0 = segment id
```

不同 `>=0` label 只表示几何分段，不表示不同对象，也不表示动态。

## 4. 输出

计划新增只读工具：

```text
DT-SLAM/tools/audit_normal_distance_segments.py
```

每帧输出：

```text
valid_depth_mask
valid_normal_mask
normal_boundary_mask
distance_boundary_mask
combined_boundary_mask
segment_label_visualization
per_segment.csv
per_frame.csv
```

全局输出：

```text
summary.json
paired_comparison.csv
contact_sheet.png
```

所有产物必须包含：

```text
dynamic_decision=none
direct_slam_state_mutation=none
proxy_is_not_gt=true
```

## 5. 与 G2-3R0 的成对审计

相同帧、相同 rectified depth、相同 bbox 和相同 feature 坐标，比较：

### 5.1 纯表示统计

```text
valid/unknown/boundary coverage
segment count
singleton and <=64-pixel segment ratio
largest/top-five segment coverage
runtime:
  smoothing_ms
  vertex_normal_ms
  edge_ms
  connected_component_ms
  total_ms
```

### 5.2 仅用于 development review 的 proxy 上界

用冻结 coarse bbox 做 oracle review，不作为部署 selector：

```text
best single-segment bbox IoU
bbox pixel coverage
selected-segment bbox purity
number of segments required to cover 50% / 80% of bbox-valid depth
```

用 exact F1 eligible features 做：

```text
inside-box feature coverage
selected-segment feature purity
selected-segment residual median / background median
catastrophic background leakage count
```

oracle review 只衡量“表示是否有容纳目标的上限”，不能宣称自动检测到了对象。

## 6. 预注册判定

### 6.1 继续条件

只有同时观察到以下方向，才允许设计“segment + motion evidence”的下一项
shadow：

1. 对 F4 已知泄漏帧 `balloon 39/200/252`，背景 feature 泄漏明显低于
   G2-3R0；
2. 在原 F4 的 14 个可比较帧上，segment 聚合不再把
   `point-inside > background` 的方向从 `14/14` 降到 `11/14` 或更差；
3. 目标 proxy 不是只能由大量微小 segment 拼接才能覆盖；
4. fr1/xyz 不出现“几乎全图一个 segment”或“绝大多数有效像素为 boundary”
   的退化；
5. 输出和重跑完全确定。

这些是 representation 条件，不是 dynamic precision/recall 条件。

### 6.2 停止条件

任一明显结构失败都停止：

```text
代表泄漏帧仍包含数百背景 features
目标区域严重碎裂且无稳定容器
normal edge 把大部分有效深度变成 boundary
文献固定参数在 Bonn/TUM 坐标域系统性失效
结果依赖 residual/bbox 驱动的参数调整才成立
```

停止后保留负结果，不添加：

```text
merge threshold
area threshold
residual-aware selector
temporal prior
```

## 7. 实现边界

本阶段不得修改：

```text
Tracking.cc / Tracking.h
GeometricDynamicDetector.h/.cc
Frame
YOLOSegment
Optimizer / g2o
LocalMapping / LoopClosing
```

不得：

```text
写 mvbDynamic
清空 mvpMapPoints
创建 dynamic MapPoint 状态
增加 PoseOptimization
运行 G1-F / G1-D
```

如果离线表示审计通过，才另写 online C++ SPEC；当前不预先承诺上线。

## 8. Viewer 与性能

本阶段是离线工具，没有 Pangolin Viewer。若后续进入在线 shadow：

- 定性运行保留 Viewer，供人工观察；
- 正式时间统计另跑 Viewer OFF；
- 同时报告 ATE/RPE/FPS 只发生在 G1 获准之后。

当前不以 `30 FPS` 为硬否决，但必须报告实际耗时。
