# GJ-1 Ji 2021 深度聚类 Shadow 结果

日期：2026-07-28  
范围：只实现 Ji et al. ICRA 2021 中的三维深度 K-means 区域生成，不实现
cluster 动态判定，不过滤特征，不修改地图。

## 1. 结论

GJ-1 的工程链路和数据验收通过，但当前全分辨率 CPU 实现的 30 Hz 实时性验收
失败。

- 合成测试通过：无效深度保持 `-1`，近/远平面分离，固定输入标签可重复；
- TUM fr3/walking 与 fr1/xyz 短序列均稳定输出每帧 24 个非空 cluster；
- 可视化表明聚类主要遵循三维表面，但一个对象会被拆成多个 cluster，邻近表面
  也可能合并；这与论文对该方法边界的描述一致；
- fr3/walking 上 K-means 平均约 `55.92 ms`，GJ-1 总计平均约 `57.84 ms`；
- fr1/xyz 上 K-means 平均约 `56.62 ms`，GJ-1 总计平均约 `58.45 ms`；
- fr3/walking 同一 19 帧关闭 GJ 时端到端约 `28.21 FPS`，开启 GJ 后约
  `12.56 FPS`；
- 因而 GJ 可继续作为论文对照 baseline，但当前实现不能作为 DT-SLAM 的
  30 FPS 主几何模块。

## 2. 文献边界

论文明确给出：

- 对新深度图中的三维点做 K-means；
- `640×480` 使用 `N=24`；
- cluster 内匹配地图点的平均重投影误差用于后续动态判断。

论文没有明确给出：

- K-means 初始化、attempts、停止条件和随机种子；
- 是否对深度点降采样；
- 鲁棒函数 `rho` 的定义；
- 动态 cluster 的数值阈值；
- cluster 最小地图匹配支持数。

因此本阶段只有 `K=24` 记为论文参数。`KMEANS_PP_CENTERS`、一次 attempt、
20 次迭代、`epsilon=1e-3` 和 seed 2021 都是显式工程选择。

## 3. 实现边界

新增：

```text
DT-SLAM/include/JiGeometryBaseline.h
DT-SLAM/src/JiGeometryBaseline.cc
DT-SLAM/Examples/RGB-D/ji_geometry_test.cc
DT-SLAM/Examples/RGB-D/TUM3_JiGeometryShadow.yaml
DT-SLAM/Examples/RGB-D/TUM1_JiGeometryShadow.yaml
```

最小接入：

```text
DT-SLAM/CMakeLists.txt
DT-SLAM/include/Tracking.h
DT-SLAM/src/Tracking.cc
```

没有修改：

```text
YOLOSegment
Optimizer.cc
g2o
LocalMapping
LoopClosing
Frame::mvbDynamic
mvpMapPoints
MapPoint 写入
```

GJ-1 在 `GrabImageRGBD()` 的 `Track()` 返回后运行，不再放在
`Tracking::Track()` 的地图互斥锁作用域内。它没有直接写 SLAM 状态，但同步耗时
仍计入端到端延迟，也可能改变线程调度；不能声称轨迹逐位等价。

## 4. 验证结果

### 合成确定性测试

```text
[Ji GJ-1 Test] PASS
```

### TUM fr3/walking，19 帧

关闭 GJ：

```text
tracking mean: 11.6463 ms
tracking median: 11.4232 ms
actual_fps: 28.21
deadline_missed: 0/19
```

开启 GJ：

```text
cluster rows: 456 = 19 × 24
empty clusters: 0
GJ total mean: 57.8417 ms
K-means mean: 55.9173 ms
GJ total min/max: 53.8674 / 62.8633 ms
tracking mean: 70.7147 ms
tracking median: 69.7432 ms
actual_fps: 12.5592
deadline_missed: 19/19
```

两次短跑的初始地图点数分别为 718 和 715。当前证据不能判断差异来自系统原有
非确定性还是同步工作造成的线程调度变化，因此不将其解释为算法状态修改，也不宣称
shadow 开关后轨迹完全不变。

### TUM fr1/xyz，14 帧

```text
cluster rows: 336 = 14 × 24
empty clusters: 0
GJ total mean: 58.45 ms
K-means mean: 56.6151 ms
actual_fps: 12.1838
deadline_missed: 14/14
```

## 5. 输出

```text
results/gj_2026-07-28/walking20_no_gj.log
results/gj_2026-07-28/walking20_v2.log
results/gj_2026-07-28/walking20_v2_clusters.csv
results/gj_2026-07-28/walking20_v2_debug/
results/gj_2026-07-28/fr1_xyz15.log
results/gj_2026-07-28/fr1_xyz15_clusters.csv
results/gj_2026-07-28/fr1_xyz15_debug/
```

## 6. GJ-2 门控决定

可以进入 GJ-2，但身份限定为“论文对照 baseline 的 shadow 统计”，不是主方法
实时化：

```text
ORB feature -> raw-depth cluster
MapPoint + Tcw -> undistorted image reprojection error
每 cluster 输出 map support、误差均值/中位数/分布
```

GJ-2 仍不得：

- 判断或过滤动态 cluster；
- 修改 `mvbDynamic`、`mvpMapPoints` 或 MapPoint 写入；
- 增加 PoseOptimization；
- 把没有匹配地图点的 cluster 当作静态；
- 凭空补写论文未公开的 `rho`、最小支持数或动态阈值。

在进入 GJ-3 前，应先形成一份单独的“论文未公开参数适配规范”，并做交叉审批。

## 7. 风险

1. 全分辨率 CPU K-means 已明显超出 30 Hz 预算；
2. 一个物体可跨多个 cluster，cluster 不是对象实例；
3. 邻近的不同表面可能合并；
4. GJ-2 的重投影误差依赖地图匹配支持，低纹理物体可能保持 unknown；
5. 同步 shadow 即使不写 SLAM 状态，也不是零运行影响；
6. 本次只跑短序列，不能由此给出 ATE/RPE 结论。
