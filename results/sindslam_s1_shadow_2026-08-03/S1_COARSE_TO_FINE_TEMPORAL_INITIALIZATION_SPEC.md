# S1 coarse-to-fine 与上一帧初始化规范

日期：2026-08-03  
阶段：S1 内部第三个增量  
状态：已实现并完成 30 帧成对审计

## 1. 身份与目的

本增量在已通过的初始 3D K-means 区域接口内增加：

```text
同帧 coarse-to-fine 初始化
+ 上一输入帧 initial labels 初始化
```

方法身份：

> `[A] SIn-style clean-room coarse-to-fine initial partition with temporal initialization`

它仍只输出初始区域，不产生 static/dynamic 判决，不是完整 SInDSLAM
geometric re-clustering。

## 2. 依据边界

### `[L]` 论文明确内容

- 深度反投影为三维点；
- 使用 coarse-to-fine Gaussian pyramid；
- 细层由上一粗层结果初始化；
- 顶层由上一帧聚类结果初始化；
- `K = width * height / 25600`；
- 无效或超过 6 m 深度归入独立区域。

论文没有给出金字塔层数、插值、K-means 迭代次数、Reset 和新显露像素的
补齐规则。

### `[C]` 公开源码行为

- 4 层：1、1/2、1/4、1/8；
- 640×480 固定 12 簇；
- 4 次迭代、epsilon 0.07、一次 attempt；
- `KMEANS_USE_INITIAL_LABELS`；
- 首次最粗层用 3×4 网格，后续用上一帧标签；
- 细层使用上一粗层标签。

### `[A]` DT-SLAM clean-room 选择

- 层数可配置，默认 4；
- 深度与分类标签都使用最近邻缩放，避免无效深度或 label ID 被线性混合；
- 每层保持真实米制深度，XYZ 的 z 权重为 1；作者源码另将 resized depth
  乘当前 pyramid scale，并使用 `z*1.5`，二者属于实质性特征空间差异；
- `finite && 0 < z < 6 m` 才参与 K-means；
- 保存上一帧“初始 K-means 标签”，不使用作者最终重聚类标签；
- prior 缺失、新显露、越界或输入序号不连续时使用确定性空间网格补齐；
- Reset 清空 prior，但不改变 Tracking 的单调外部输入序号；
- 保留 full-resolution from-scratch 模式作为成对成本对照。

不得复制 `DynaDetect.cc`、PEAC/AHC 或作者 RAG 实现。

## 3. 状态与接口

```text
previousInitialLabels : CV_32SC1，-1 invalid，>0 region
previousInputIndex
hasPreviousLabels
```

`Compute(depth, inputIndex)`：

1. 构建 4 层最近邻深度金字塔；
2. 最粗层使用合法 previous prior，否则使用网格；
3. 逐层最近邻上采样粗层 label；
4. 每个有效样本都获得 `[0,K-1]` 初始标签；
5. 运行 `KMEANS_USE_INITIAL_LABELS`；
6. 输出全分辨率 `-1/>0` 标签并提交 temporal prior。

作者对齐模式：input 0 可计算但不提交 prior，input 1 用网格并首次提交，
input 2 起使用 previous。该模式只用于与现有 S0 reference 对照；普通 native
运行可从 input 0 开始提交。

## 4. 失败与 Reset

- 输入序号连续且 prior 尺寸/标签域合法：使用 prior；
- 输入序号跳变：整帧回退网格，不静默使用陈旧 prior；
- prior 中 invalid 或新显露像素：逐样本走网格 fallback；
- `Reset()`：清空 prior；reset 后同一输入应等价于全新对象；
- Tracking LOST 不自动改变区域语义；shadow 阶段每个有效 RGB-D 输入仍可
  更新区域 prior，并另由既有 CSV 记录最终 tracking state。

## 5. 新增统计

```text
coarse_to_fine
pyramid_levels
initialization_source = grid / previous / mixed
previous_prior_samples
grid_fallback_samples
previous_prior_coverage
temporal_prior_committed
per-level size / valid samples / compactness / runtime
```

守恒：

```text
previous_prior_samples + grid_fallback_samples
= coarsest_valid_samples
```

## 6. 验收

- 全分辨率基线继续通过；
- 相同 depth/prior 输出一致；
- Reset 后等于新对象首次运行；
- 奇数尺寸的 4 层大小合法；
- invalid 和 `z>=6 m` 最终仍为 -1；
- 新显露像素进入 grid fallback；
- 输入序号跳变回退 grid；
- 不污染 OpenCV RNG 状态；
- 30 帧 author-aligned run 中 input 0/1 为 grid，input 2 起使用 previous；
- `dynamic_state_available=0`、`actual_removed=0`。

## 7. 本增量仍不能回答

- 区域是否对应对象；
- 哪个区域动态；
- 是否改善 ATE/RPE；
- 是否可开放 S2。
