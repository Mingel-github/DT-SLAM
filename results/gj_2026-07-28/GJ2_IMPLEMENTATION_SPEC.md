# GJ-2 Ji 2021 Cluster Reprojection Shadow 实现规范

日期：2026-07-28  
前置阶段：GJ-1 depth clustering shadow 已完成  
本阶段：ORB-to-cluster 映射与 cluster 重投影误差统计

## 1. 文献依据

Ji et al. ICRA 2021 明确描述：

```text
初始跟踪得到初始位姿
→ 深度图三维 K-means
→ 将 cluster 内特征与匹配三维地图点计算平均重投影误差
→ 相对误差较大的 cluster 判为动态
→ 删除动态 cluster 特征后继续跟踪局部地图
```

论文公式可统一写为：

```text
r_j = (1 / m_j) Σ rho(||u_i - pi(Tcw P_i)||²)
```

其中论文没有公开：

- `rho` 的具体形式和参数；
- 动态 cluster 阈值；
- “相对较大”的完整判定规则；
- cluster 的最小匹配支持数；
- 公开实现代码。

因此 GJ-2 只测量证据，不判定动态。

## 2. 当前工程适配

### 2.1 两个像素域必须分开

cluster label 位于 raw registered RGB/depth 像素域：

```text
Frame::mvKeys[i].pt -> cluster label
```

ORB-SLAM2 Optimizer 使用去畸变针孔域：

```text
Frame::mvKeysUn[i].pt -> observed reprojection pixel
```

不得用 `mvKeysUn` 查询 raw depth cluster，也不得用 `mvKeys` 直接计算针孔
重投影误差。

### 2.2 使用初始跟踪快照

为了保留 Ji 论文的两阶段时序，在：

```text
TrackWithMotionModel / TrackReferenceKeyFrame / Relocalization 成功
→ TrackLocalMap 之前
```

只快照：

```text
initial Tcw
raw keypoint
undistorted keypoint
matched MapPoint world position
```

K-means 和重投影统计仍在 `GrabImageRGBD()` 中 `Track()` 返回后执行。这样不会把
约 58 ms 的全分辨率 K-means 放进 `Tracking::Track()` 的地图互斥锁作用域。

这是 `[A]` 工程适配，不是 Ji 论文原始线程实现。

### 2.3 纳入的地图匹配

快照只纳入：

- `mvpMapPoints[i] != nullptr`；
- MapPoint 非 bad；
- MapPoint 至少有一个正式 observation；
- 世界坐标有限。

快照发生在初始 `PoseOptimization()` 返回后、Tracking清除outlier关联之前。
因此正式MapPoint的optimizer inlier与outlier都会保留，并分别统计
`optimizerOutlierSupport`。这是必要的，因为先清除高残差关联会系统性压低Ji方法
需要测量的cluster残差。临时VO MapPoint不作为论文意义上的地图对应。

正常`TrackReferenceKeyFrame`和`TrackWithMotionModel`路径保留清除前的outlier。
复杂`Relocalization`路径当前只在其成功返回后快照仍保留的正式关联，并在CSV中
通过支持数体现；本阶段不侵入或重构Relocalization内部多候选优化。

## 3. GJ-2统计

每个 cluster 输出：

```text
depth pixel count
ORB feature count
matched map support
valid reprojection support
behind-camera / invalid projection count
mean squared reprojection error [px²]
mean reprojection error [px]
median reprojection error [px]
p90 reprojection error [px]
maximum reprojection error [px]
evidence state
```

定义：

```text
e_i² = ||u_i - pi(Tcw P_i)||²
```

论文未说明 `rho`。本阶段将：

```text
rho(e_i²) = e_i²
```

作为最透明的 identity engineering baseline `[E]`，同时输出未平方的均值、
中位数、P90 和最大值用于诊断。不得将 identity `rho` 写成论文参数。

P90使用nearest-rank定义：

```text
index = ceil(0.9 * n) - 1
```

该定义同样属于`[E]`诊断选择。

## 4. Unknown约定

```text
valid reprojection support == 0
→ unknown / no geometry evidence
```

GJ-2不设置最小支持阈值，也不把支持不足解释成静态。support 数量原样输出，后续
GJ-3若需要最小支持数，必须作为论文未公开参数单独审批。

## 5. 代码边界

允许修改：

```text
include/JiGeometryBaseline.h
src/JiGeometryBaseline.cc
include/Tracking.h
src/Tracking.cc
Examples/RGB-D/ji_geometry_test.cc
CMakeLists.txt（如无需新target则不再改）
新增GJ-2配置文件
```

禁止修改：

```text
YOLOSegment
Optimizer.cc
g2o
LocalMapping
LoopClosing
Frame::mvbDynamic
mvpMapPoints内容
MapPoint写入
```

## 6. 验收

- 合成投影的 cluster support 和像素误差正确；
- raw `mvKeys` 与 `mvKeysUn` 的职责明确；
- 无匹配 cluster 输出 unknown；
- 不产生 dynamic mask；
- 不增加 PoseOptimization；
- 配置关闭时不保存初始快照、不运行统计；
- GJ-1 K-means输出保持不变；
- TUM短序列输出每帧每cluster CSV；
- 单独统计 reprojection 阶段耗时；
- 不把短序列结果解释为 ATE/RPE 或最终动态检测效果。
