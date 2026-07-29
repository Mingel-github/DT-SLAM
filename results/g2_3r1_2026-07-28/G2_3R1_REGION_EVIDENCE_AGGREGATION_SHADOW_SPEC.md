# G2-3R1 区域内多参考证据聚合 Shadow 规格

日期：2026-07-28

## 1. 目标

固定 G2-3R0 的 depth-boundary region labels，在每个区域内部统计 G2 多参考
证据分布：

```text
region pixels
comparison-covered pixels
positive / negative / consistent presence pixels
comparison / positive / negative / consistent vote sums
semantic-proxy overlap
```

本阶段只生成逐区域 CSV，不选择动态阈值，不生成动态 mask，不影响 SLAM。

## 2. 文献依据

### DetectFusion `[L]`

DetectFusion 将 ICP residual 二值运动 mask 与几何 segments 做 IoU，以避免局部
运动 residual 只覆盖对象的一部分。它证明“先形成几何区域，再统计区域与运动证据
的重叠”是有文献依据的结构。

当前工程没有 TSDF/surfel ICP residual，因此只借区域—证据重叠结构，不复现
DetectFusion。

### SInDSLAM `[L/A]`

SInDSLAM 将 residual 分成 high、low 和 static，并把动态判断限制在单个 cluster
内部；其 dense-map refinement 还使用：

```text
new_dynamic_pixels / cluster_pixels
```

决定是否扩展到整个 cluster。

当前 G2 的对应证据为：

```text
positive / negative / consistent / no-comparison
```

因此本阶段仅记录区域覆盖率、证据像素比例和票数比例。这是 `[A]` 证据统计适配，
不是其光流 residual、双阈值或 flood-fill 复现。

## 3. 必须保持的状态语义

```text
comparison == 0       -> no geometric evidence
positive > 0          -> at least one foreground-inconsistency vote
negative > 0          -> at least one disocclusion diagnostic vote
consistent > 0        -> at least one static-consistency vote
```

同一像素在不同参考下可以同时含 positive 和 consistent vote，所以三种 presence
不是互斥类别；但所有 vote sum 必须满足：

```text
positive_votes + negative_votes + consistent_votes
= comparison_votes
```

区域 label `-1`（invalid）和 `-2`（boundary）不进入任何静态或动态统计。

## 4. 输出，不输出判决

每区域输出：

- region id 与 region pixels；
- semantic proxy pixels；
- semantic proxy 内 comparison/positive/negative/consistent presence；
- comparison-covered pixels；
- positive/negative/consistent presence pixels；
- 四类 vote sums；
- comparison coverage；
- 三种 presence / compared-pixel ratios；
- 三种 vote / comparison-vote ratios；
- region partition 和 aggregation 耗时。

明确不输出：

- dynamic/static 二值标签；
- dynamic mask；
- ORB feature 动态状态；
- MapPoint 写入否决；
-任何用于 G1 的阈值。

## 5. 验收

1. 小矩阵测试验证 vote 守恒、unknown/boundary 排除和重叠 presence；
2. walking、sitting、fr1/xyz 各 199 帧，同一 G2-2G stride-4 输入；
3. 分开报告 partition 与 aggregation 耗时；
4. 使用 semantic person mask 只作 proxy，不当真实运动 GT；
5. 静态 fr1/xyz 必须作为负对照；
6. 在看到跨序列可分性之前，不批准任何 region threshold。
