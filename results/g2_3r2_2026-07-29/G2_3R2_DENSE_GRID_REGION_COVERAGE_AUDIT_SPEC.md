# G2-3R2 dense-vs-grid 区域覆盖上限审计规格

日期：2026-07-29

## 1. 研究问题

G2-3R1 使用 stride-4 规则深度网格，以较低成本把多参考深度不一致证据聚合到
固定深度区域。当前尚不清楚区域支持不足主要来自：

```text
多视图深度几何本身没有有效比较
```

还是：

```text
stride-4 轻量采样丢失了 dense warp 原本具有的比较与正残差证据。
```

G2-3R2 在完全相同的：

- 当前帧；
- 最多 5 个共视参考帧；
- 初始 SLAM 位姿；
- 0.10 m signed-depth residual 阈值；
- G2-3R0 深度区域标签；

下，对比：

```text
grid_depth stride-4
vs
dense_same_reference_audit
```

## 2. 方法来源和性质

本阶段不引入新的动态检测方法。

- 历史深度投影和多参考深度一致性结构受 DynaSLAM 启发 `[A]`；
- 将运动证据统计到几何区域的结构由 DetectFusion 支持 `[L/A]`；
- 区域内计算证据比例的结构由 SInDSLAM 支持 `[L/A]`；
- dense 与规则网格使用同参考、同位姿、同区域进行成对比较，是实验控制
  `[S]`，不是论文算法或本文创新。

不得表述为 DynaSLAM、DetectFusion 或 SInDSLAM 的复现。

## 3. 实现边界

复用当前已经存在的：

```text
DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT
GeometricMultiReferenceResult denseAuditResult
G2-3R0 regionPartition
AggregateMultiReferenceEvidenceByRegion()
```

只增加：

1. 将 `denseAuditResult` 聚合到同一个 `regionPartition`；
2. 在现有 region CSV 中以
   `sampling_policy=dense_same_reference_audit` 保存；
3. 离线成对审计脚本与结果文档。

明确不做：

```text
不生成 dynamic/static 判决
不选择 positive 阈值
不修改 mvbDynamic 或 mvpMapPoints
不修改 Optimizer、LocalMapping 或 MapPoint
不增加 PoseOptimization
不加入光流、MAD、时序投票或形态学传播
不重新计算第二份区域划分
```

## 4. 主要指标

按相同 `(frame, region_label)` 成对比较：

- grid 和 dense comparison coverage；
- grid/dense comparison pixel retention ratio；
- grid 和 dense positive presence pixels；
- grid/dense positive pixel count retention ratio；
- dense 有支持而 grid 无支持的区域比例；
- semantic proxy 内 grid/dense comparison coverage；
- region positive vote ratio 的成对差异；
- grid multi-reference、dense audit 和两次 region aggregation 的耗时。

`positive pixel count retention ratio` 只表示数量保留，不是真实动态 recall；
semantic person mask 仍是 proxy，不是真实运动 GT。

## 5. 验收与决策

工程验收：

1. 相同帧中 grid 与 dense 的 region label、region pixels 和 semantic proxy pixels
   必须完全一致；
2. dense 和 grid 均保持 vote 守恒；
3. 只执行一次 region partition；
4. Geometry 继续为 shadow-only；
5. 单元测试、编译和 `git diff --check` 通过。

研究决策：

- 若 dense 区域支持明显高于 grid，说明轻量采样是主要覆盖瓶颈，需要寻找更好的
  采样或低分辨率 dense 实现；
- 若 dense 也缺乏区域支持，说明当前深度 warp/参考/位姿证据本身是瓶颈，不应
  继续优化 stride；
- 无论哪种结果，本阶段都不批准 G1-F 或 G1-D。
