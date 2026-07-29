# G2-3R3 二分辨率 dense evidence Shadow 规格

日期：2026-07-29

## 1. 目标

G2-3R2 证明 full-resolution dense 多参考证据覆盖明显高于 stride-4，但当前
CPU 实现需要约 14–19 ms。G2-3R3 测试一个中间点：

```text
full-resolution depth
→ boundary-preserving 2x depth pyramid
→ 320x240 dense multi-reference warp
→ nearest cell expansion 到 640x480 evidence domain
→ 与同帧 full dense upper bound 成对比较
```

本阶段只回答：

```text
二分辨率 approximation 能保留多少 full dense coverage 和 evidence，
以及实际 CPU 成本是多少？
```

不输出动态判决，不进入 SLAM 过滤。

## 2. 文献核对与准确归属

### KinectFusion `[L/A]`

KinectFusion 使用三层深度/vertex/normal 金字塔进行 coarse-to-fine dense ICP。
其深度金字塔由 block averaging 和 2x subsampling 产生，并且只平均与中心像素
深度差在范围阈值内的值，以避免跨深度边界平滑。

G2-3R3 只借：

- 2x depth pyramid；
- 边界保持的 block average；
- 对应分辨率使用缩放相机内参；
- dense projective computation 可以在多尺度上运行。

当前没有 TSDF、vertex/normal map、ICP 或 coarse-to-fine pose optimization，
因此不是 KinectFusion 复现。

### SInDSLAM `[L]`

SInDSLAM 使用 depth Gaussian pyramid 加速 K-means clustering convergence，并用
粗层结果初始化细层。它没有用低分辨率 depth warp 代替多视图动态 residual。

因此不得写：

```text
following SInDSLAM, we compute low-resolution dynamic evidence
```

只能写：

```text
SInDSLAM supports multi-scale depth processing for geometric regions,
but G2-3R3 low-resolution evidence is our engineering hypothesis.
```

### DetectFusion `[L]`

DetectFusion 的三层 coarse-to-fine pyramid 用于稠密相机跟踪，不是未知动态
motion mask 的降分辨率近似。这里只作为动态 RGB-D 系统采用多尺度 dense
计算的旁证。

### 本阶段性质 `[S/H]`

将低分辨率 evidence count 以 nearest cell expansion 映射回全分辨率区域，是
本项目为了测量覆盖/成本折衷的工程假设，不是上述论文的算法。

## 3. 深度金字塔

第一版只允许：

```text
scale = 2
640x480 → 320x240
```

每个 2x2 block：

1. 使用与输出采样射线对应的左上像素作为 anchor；
2. anchor 深度无效，则输出无效深度；
3. 只平均有效且满足

```text
|d - anchor| <= max(τ_rel * anchor, τ_abs)
```

的 block 深度；
4. `τ_rel=0.025`、`τ_abs=0.08m`，与当前区域边界配置保持一致。

这是：

- KinectFusion boundary-preserving block averaging 的适配；
- SInDSLAM relative-plus-absolute depth boundary threshold 的复用；
- 不是 KinectFusion 原始 `3σr` 参数复现。

## 4. 相机模型和坐标域

采用左上 decimation convention：

```text
u_full = 2 * u_half
v_full = 2 * v_half
```

相机内参：

```text
fx_half = fx / 2
fy_half = fy / 2
cx_half = cx / 2
cy_half = cy / 2
```

参考和当前深度必须同时降采样，位姿保持不变。语义 mask 和最终 region labels
仍保持全分辨率。

## 5. Evidence expansion

低分辨率的：

```text
comparison / positive / negative / consistent vote counts
```

使用 nearest-neighbor 扩展到当前全分辨率。扩展后的每个 2x2 cell 共享一份
低分辨率证据。

必须明确：

```text
expanded evidence != 四个独立像素测量
```

它只是用于区域支持和覆盖近似。无效深度、depth boundary 和 region label 仍由
全分辨率 G2-3R0 partition 决定。

## 6. 缓存和计时

- 参考帧 half-depth 在进入 20 帧参考缓存时计算一次；
- 当前 half-depth 每个计算帧生成一次；
- 在线计时包括当前深度降采样、half dense warp、evidence expansion；
- full dense 仅在 audit 开关下作为上限，不属于候选在线成本；
- region partition 仍只执行一次。

## 7. 输出与对照

运行标签：

```text
pyramid_dense_s2
dense_same_reference_audit
```

同一 `(frame, region_label)` 比较：

- comparison coverage；
- pyramid/dense comparison retention；
- positive count retention；
- semantic/nonsemantic conditional positive；
- dense 支持但 pyramid 无支持的区域；
- pixel-level comparison/positive agreement；
- pyramid evidence 时间；
- full dense upper-bound 时间；
- region partition/aggregation 时间；
- actual FPS 和 deadline miss。

## 8. 非目标

```text
不选择动态阈值
不把 expanded cell 当独立像素 GT
不修改 mvbDynamic、mvpMapPoints 或 MapPoint
不修改 Optimizer、g2o、LocalMapping、LoopClosing
不增加 PoseOptimization
不实现多层 coarse-to-fine optimization
不实现 Gaussian blur 跨无效深度
不实现 CUDA geometry
不同时在线保留 full dense
```

## 9. 验收

1. 确定性小矩阵验证 invalid、边界保持平均和尺寸；
2. identity pose 下低分辨率 evidence 保持 vote 守恒；
3. 扩展后四类 vote 仍严格守恒；
4. walking、sitting、fr1/xyz 各约 199 帧；
5. 与相同 full dense 参考集合成对比较；
6. 单独报告 pyramid 候选成本和 full dense audit 成本；
7. 不依据 semantic proxy 批准 G1。
