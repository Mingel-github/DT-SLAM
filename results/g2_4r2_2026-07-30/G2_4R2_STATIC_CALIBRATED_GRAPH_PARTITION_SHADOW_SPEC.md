# G2-4R2 静态标定图分割 shadow 审计规范

日期：2026-07-30
状态：实现前冻结

## 1. 唯一问题

本审计只回答：

> F3 两帧三维边测量加上 connected components 后，是否存在一个由真正静态
> 数据标定、静态误分裂较低，同时在动态开发代理中产生目标富集的工作点？

不回答：

```text
是否已检测到动态对象
是否可以进入 tracking 过滤
是否优于 Dai et al.
是否能输出完整 M_depth
```

## 2. 输入冻结

### 静态标定与验证

```text
results/g2_4f3_2026-07-30/static150_nodes.csv
results/g2_4f3_2026-07-30/static150_nodes.csv.edges.csv
```

序列：Bonn `rgbd_bonn_static_close_far`。

按时间顺序拆分可测帧：

```text
前半：static calibration
后半：static validation
```

只允许 calibration edge 分布产生阈值；validation 不参与阈值计算。

### 动态开发代理

```text
results/g2_4f3u_2026-07-30/balloon_nodes.csv
results/g2_4f3u_2026-07-30/balloon2_nodes.csv
results/g2_4f1_development_data_2026-07-29/
  balloon_f1d_rgb_only_coarse_bboxes.csv
```

只评价 bbox 中标为 `visible` 或主要 `partial` 的冻结开发帧。bbox 是 RGB-only
coarse review proxy，不是 pixel/object/motion GT。

`balloon_tracking` strict holdout 保持封存。

## 3. 图构造

对每帧分别建立：

```text
all_transient
mappoint_only
```

节点必须：

- `evidence_state=measured`；
- 有有限的 current/reference 3D 坐标；
- all-transient 接受所有上述节点；
- MapPoint-only 额外要求 `has_mappoint=1`。

在当前图像坐标 `(u_current,v_current)` 上重新做 Delaunay。重复像素只保留
`feature_index` 最小者，以保证确定性。

每条边计算：

\[
s_{ij} =
\left|
\|\mathbf X_{i,t}-\mathbf X_{j,t}\|_2
-
\|\mathbf X_{i,r}-\mathbf X_{j,r}\|_2
\right|.
\]

这是 F3 已实现的 absolute edge strain `[A/H]`，不是 Dai 的完整
Mahalanobis point-correlation residual。

## 4. 静态阈值冻结

只从 static calibration 的全部有效 edge strain 计算：

```text
q90
q95
q97.5
q99
q99.5
```

每个分位点都是预注册诊断工作点，不根据动态代理调整。

对每个工作点：

```text
删除 strain > threshold 的边
保留所有节点
求 connected components
以节点数最多的 component 作为 primary component
其余节点记为 outside_primary
```

`primary component = 最大节点数` 是确定性的 `[S]` 审计选择，不冒充论文
`largest volume` 的完整实现。并列时取最小 feature index 所在 component。

## 5. 关键状态

每帧必须区分：

```text
measured
insufficient_nodes      (<3 nodes)
insufficient_edges      (Delaunay 无有效边)
single_component
partitioned
```

任何 insufficient 状态都不能被解释成 static。

## 6. 指标

### 静态 validation

- node count、edge count；
- connected component 数；
- primary node fraction；
- outside-primary node fraction；
- isolated node fraction；
- outside-primary fraction 的 median/p90/p95；
- outside-primary >1%、>5%、>10% 的帧比例；
- insufficient frame count。

### 动态开发代理

仅在 bbox 内至少有一个 measured node 时统计：

- bbox node count；
- bbox nodes outside primary 的 recall proxy；
- outside-primary nodes 位于 bbox 的 precision proxy；
- bbox/outside enrichment：

\[
E =
\frac{
|B\cap O|/(|B|+\epsilon)
}{
|O_{\neg B}|/(|V_{\neg B}|+\epsilon)+\epsilon
};
\]

- 有至少三个 bbox nodes 的支持帧数；
- MapPoint-only insufficient 比例。

这里的 recall/precision/enrichment 均带 `proxy` 后缀。

## 7. 预注册判定

只有存在至少一个静态分位工作点同时满足：

```text
static validation outside-primary p95 <= 5%
至少一半动态候选帧具有 >=3 个 bbox nodes
dynamic supported frames 中至少一半的 bbox recall proxy >= 50%
dynamic median enrichment proxy >= 3
```

才允许结论：

> topology feasibility 有条件通过，可进入更接近论文的 covariance/
> edge-culling shadow 设计。

若只有 all-transient 通过：

> 只证明 transient adaptation 值得继续；不能声称 Dai/MapPoint 路线通过。

若只有 MapPoint-only 通过但支持帧不足一半：

> 记录为机会式证据，不能成为通用前端。

若没有工作点通过：

> 冻结为负结果，停止用简单 scalar strain + CC 修补 F3。

这些阈值是项目的 `[S]` 风险标准，不是论文参数。

## 8. 输出与不变式

输出：

```text
static_thresholds.json
per_frame_partition.csv
aggregate_summary.json
G2_4R2_*_RESULT.md
```

每条输出必须带：

```text
dynamic_decision=none
direct_slam_state_mutation=none
proxy_identity=frozen RGB-only coarse bbox; not motion ground truth
```

确定性要求：相同输入运行两次，JSON 和 CSV 字节一致。

## 9. 明确非目标

```text
不修改 C++ Tracking/Frame/GeometricDynamicDetector
不增加在线配置
不运行完整 TUM/Bonn SLAM
不选择最终动态阈值
不运行 sealed holdout
不实现 Dai edge-state optimizer
不修改 YOLO 或 Optimizer
```
