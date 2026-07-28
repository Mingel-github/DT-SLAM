# G0-2A：TUM `fr1/xyz` 静态负样本审计

日期：2026-07-27

## 1. 目的

本实验只审计单参考帧 depth warp 产生的直接几何证据，不让几何结果影响
Tracking、Optimizer、MapPoint 或稠密深度写入。

主要问题：

1. 在没有刻意加入运动物体的办公桌序列中，正深度残差 seed 的背景误报底线是多少；
2. 将 ORB-SLAM2 初始位姿替换为数据集 GT 位姿后，seed 是否明显减少；
3. 当前 seed 的空间分布是否支持直接进入区域生长或特征过滤。

## 2. 数据与坐标域

- 序列：TUM `rgbd_dataset_freiburg1_xyz`
- 一对一 RGB-D 关联：792 对，无复用
- RGB-depth 时间差：均值 6.30 ms，最大 17.23 ms
- GT 可插值帧：788/792
- 得到成对 SLAM/GT geometry 诊断：786 帧
- 深度：16-bit PNG，`DepthMapFactor=5000`，转换为 `CV_32F` 米制深度

TUM 官方说明：

- RGB 和 depth 已由 OpenNI 预配准，像素一一对应；
- 对预配准 PNG 生成点云时，官方建议使用 ROS 默认零畸变针孔模型
  `fx=fy=525, cx=319.5, cy=239.5`；
- Freiburg 1 的精标 RGB 模型含非零畸变，但直接校正已经预配准的 depth
  并不简单。

因此本实验明确分离两个模型：

| 使用方 | 相机模型 |
| --- | --- |
| ORB-SLAM2 Tracking | 原生 TUM1 RGB 标定，含非零畸变 |
| raw registered depth geometry | ROS 默认零畸变针孔模型 |

参考：

- <https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats>
- <https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download>

## 3. 本轮最小代码修正

新增可选配置：

```yaml
Geometry.Camera.fx
Geometry.Camera.fy
Geometry.Camera.cx
Geometry.Camera.cy
```

四项必须全部给出或全部省略。给出时，它们只用于 raw-depth geometry；
ORB-SLAM2 的 `Camera.*` Tracking 标定保持不变。geometry 模型仍严格限定为
零畸变针孔，不是绕过畸变检查。

新增配置：

```text
DT-SLAM/Examples/RGB-D/TUM1_GeometryPoseDiagnostic.yaml
```

本轮没有修改 YOLO、Optimizer、g2o、Local BA 或地图写入逻辑。

## 4. 验证

### 编译与确定性测试

```text
rgbd_tum: build PASS
geometric_warp_test: build PASS
[Geometry G0-3R Test] PASS
```

### 轨迹质量

在 790 个匹配轨迹位姿上：

| 指标 | 数值 |
| --- | ---: |
| ATE translation RMSE | 0.009922 m |
| ATE translation mean | 0.008326 m |
| RPE translation RMSE，delta=1 frame | 0.006052 m |
| RPE translation mean，delta=1 frame | 0.004990 m |

这说明本次 SLAM 轨迹整体正常，但不能据此断言每帧局部位姿完全无误。

## 5. 全量 shadow 统计

阈值：

```text
positive seed: signed residual > 0.10 m
negative diagnostic: signed residual < -0.10 m
region growing: disabled
```

| 指标 | SLAM pose | GT pose |
| --- | ---: | ---: |
| valid comparison coverage，mean | 71.68% | 71.63% |
| absolute residual，mean | 0.00964 m | 0.01154 m |
| absolute residual，median | 0.00776 m | 0.00918 m |
| positive seed ratio，mean | 0.521% | 0.687% |
| positive seed ratio，median | 0.353% | 0.440% |
| positive seed ratio，p95 | 1.601% | 2.200% |
| positive seed ratio，max | 4.989% | 5.953% |
| negative ratio，mean | 0.632% | 0.791% |
| geometry total，mean | 3.48 ms | 3.91 ms |

SLAM 与 GT 的 positive ratio 相关系数为 `0.914`，absolute residual mean 的相关
系数为 `0.921`。GT pose 没有降低 seed，反而在本次统计中略高。

这个结果不能解释为“GT 比 SLAM 差”。它表明当前 seed 不能主要归因于
ORB-SLAM2 位姿误差；共同的深度边界、遮挡关系、forward rasterization、
RGB-depth 时间差和标定近似仍然混在测量中。

## 6. seed 与深度边缘关系

对保存的 27 个抽样帧、共 41,171 个 positive seed 做诊断。这里将以下像素定义
为“深度边缘”：

- 四邻域有效/无效深度发生转换；或
- 四邻域深度跳变超过 0.05 m。

这只是诊断定义，不是新增检测算法。

| 与深度边缘的 Chebyshev 距离 | seed 加权占比 |
| --- | ---: |
| 边缘像素本身 | 40.17% |
| 1 像素内 | 70.80% |
| 2 像素内 | 86.20% |
| 3 像素内 | 93.53% |
| 5 像素内 | 98.39% |

可视化也显示红/蓝证据主要沿显示器、桌面物体、纸张和深度空洞边界分布。

## 7. 性能

本次同时运行 SLAM pose 与 GT pose 两套 geometry，并每 30 帧保存调试图：

| 指标 | 数值 |
| --- | ---: |
| tracking mean | 21.32 ms |
| active total mean | 29.66 ms |
| sequence wall time | 26.97 s |
| actual FPS | 29.37 |
| deadline misses | 77/792 |

单套 SLAM-pose shadow geometry 均值为 3.48 ms。该数值只说明 G0 测量开销，
不能直接证明未来 geometry filtering 仍可稳定达到 30 FPS。

## 8. 客观结论

1. TUM1 的 Tracking 标定与 raw-depth geometry 标定已经正确分域；
2. G0-1/G0-2 在 `fr1/xyz` 上稳定运行，覆盖率和运行时间可测；
3. GT pose 没有消除背景 seed，因此不能把当前背景误报主要归因于 SLAM 位姿；
4. `0.10 m` positive seed 在静态序列中仍有约 0.52% 的平均误报底线；
5. 绝大多数抽样 seed 靠近深度跳变或无效深度边界；
6. 当前 positive seed 是“几何不一致证据”，不是可直接过滤的动态 mask；
7. 当前结果不支持重启无限 flood fill，也不支持进入 G1；
8. 本轮没有调参，没有用面积阈值掩盖误报。

## 9. 下一步门控

下一步应继续做诊断，而不是添加传播规则：

1. 在少量明确动态帧上人工标注运动区域；
2. 分别统计 direct seed 在动态区域和静态背景上的 precision、conditional recall；
3. 统计 ORB keypoint 周围直接证据，判断 feature-level shadow 是否有足够精度；
4. 单独记录 RGB-depth 时间差，并检查高误报帧是否与时间差或深度边界比例相关；
5. 只有 direct seed 质量通过后，才比较 DynaSLAM code-style 与 ReFusion-style
   区域传播。

