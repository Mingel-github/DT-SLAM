# DT-SLAM G0-2C 坐标域与同步审计记录

日期：2026-07-27  
阶段：G0-2C  
性质：shadow 输入门控；不生成可用于过滤的 geometry mask

## 1. 本阶段目标

在进入 GT/SLAM pose 对照和 direct seed 标注审计前，确认：

1. RGB、depth、semantic mask、ORB keypoint 和 geometry 输出的坐标域；
2. 当前纯针孔 depth warp 对标定畸变的适用边界；
3. RGB-depth association 是否一对一、时间差是否可接受；
4. ground-truth 时间戳是否能用于下一阶段的位姿敏感性诊断；
5. geometry reference 和 current frame 的时间间隔是否被记录。

本阶段不修改 Optimizer、g2o、BA、YOLO 推理或动态过滤逻辑。

## 2. 坐标域核对

本地源码直接确认：

| 数据 | 当前坐标域 | 代码事实 |
|---|---|---|
| RGB/depth | 原始注册像素栅格 | `GrabImageRGBD()` 直接将输入交给 `Frame` 和 geometry |
| semantic mask | 原始 RGB 像素域 | 与 `Frame::mvKeys` 对齐 |
| `mvKeys` | 原始 RGB 像素域 | ORB extractor 输出 |
| `mvKeysUn` | 去畸变针孔域 | `Frame::UndistortKeyPoints()` 生成 |
| geometry warp | 原始 depth 像素 + 纯针孔 `K` | 当前 `ComputeWarp()` 不读取畸变参数 |
| optimizer/matching | 主要使用 `mvKeysUn` | ORB-SLAM2 原路径 |

因此，当前 geometry 仅在“原始像素已经满足零畸变针孔模型”时成立。

TUM3 配置中：

```text
k1 = k2 = p1 = p2 = 0
```

当前 TUM3 可以继续使用该实现。非零畸变配置下，geometry 现在会 fail-fast，提示必须统一 rectification 或实现 distortion-aware warp。该保护只在 `Geometry.Enable=1` 时生效，不影响普通 ORB-SLAM2/semantic 模式。

需要强调：这项保护不能解释已经完成的 TUM3 G0-3 失败，因为该序列使用零畸变配置。

## 3. 输入尺寸和时间戳保护

`Tracking::GrabImageRGBD()` 新增 shadow 输入门控：

- RGB 和 depth 必须非空；
- 注册 depth 尺寸必须与 RGB 相同；
- RGB timestamp 必须为有限数；
- semantic mask 原有的 `CV_8UC1` 和尺寸检查保留。

geometry reference 现在保存 frame timestamp；G0 日志新增：

```text
current_ts
ref_ts
dt_s
```

这只增加诊断信息，不改变 relative pose 或残差计算。

## 4. association 审计发现

原始运行文件：

```text
TUM/rgbd_dataset_freiburg3_walking_xyz/associations.txt
```

审计结果：

| 指标 | 原 association |
|---|---:|
| association 行数 | 859 |
| 唯一 depth 路径 | 827 |
| depth 复用行数 | 32 |
| 单个 depth 最大复用 | 3 帧 RGB |
| RGB-depth 绝对时间差中位数 | 0.0219 ms |
| RGB-depth 绝对时间差 P95 | 1.4811 ms |
| RGB-depth 最大绝对时间差 | 38.0960 ms |

这说明大多数配对很好，但 32 个 RGB 帧复用了已有 depth，其中若干时间差约 25–38 ms。该现象可能影响时序 depth residual，尤其是“不同 RGB/pose 使用同一 depth”的相邻帧；目前没有对照实验支持把 G0-3 失败归因于此，因此只记录为候选混杂因素。

## 5. 一对一诊断 association

新增工具：

```text
DT-SLAM/tools/make_rgbd_association.py
```

它使用最大时间差内的全局候选，从小到大贪心选择一对一 RGB/depth 配对，并拒绝覆盖已有输出文件。

使用 `20 ms` 门限生成：

```text
results/g0_2c_2026-07-27/
  fr3_walking_xyz_associations_one_to_one_20ms.txt
```

结果：

| 指标 | 一对一 association |
|---|---:|
| 配对数 | 827 |
| 未配对 RGB | 32 |
| 未配对 depth | 6 |
| depth 复用 | 0 |
| RGB-depth 绝对时间差中位数 | 0.0210 ms |
| RGB-depth 绝对时间差 P95 | 0.8430 ms |
| RGB-depth 最大绝对时间差 | 6.1681 ms |

原始 association 没有被覆盖。后续 G0-2P 和 seed 审计应使用该一对一版本，避免 depth 复用成为额外变量。

## 6. GT 时间同步审计

审计工具：

```text
DT-SLAM/tools/audit_rgbd_geometry_inputs.py
```

对一对一 association 与 TUM groundtruth 做最近时间戳诊断：

| 指标 | RGB→最近 GT | depth→最近 GT |
|---|---:|---:|
| 中位绝对时间差 | 2.2120 ms | 2.2399 ms |
| P95 绝对时间差 | 4.5030 ms | 4.6389 ms |
| 最大绝对时间差 | 45.7740 ms | 45.7120 ms |
| 10 ms 内 | 826/827 | 826/827 |

绝大多数帧可支持下一阶段插值，但有一个边界/空缺帧超过 10 ms。G0-2P 必须显式规定：

- 仅在 GT 时间范围内插值；
- 平移线性插值；
- 四元数 SLERP；
- 最大允许时间间隔；
- 超限帧标为 `GT unavailable`，不能外推；
- TUM 文本 pose 先按 `Twc` 构造，再求逆得到 detector 使用的 `Tcw`。

GT 路径仍然只是 pose sensitivity diagnostic，不是零误差 residual 真值。

## 7. 运行验证

### 7.1 构建与单元测试

```text
make rgbd_tum geometric_warp_test -j$(nproc)
```

结果：成功。已有 ONNX Runtime C++17 和 Eigen deprecated 警告仍存在，本阶段没有引入新的编译错误。

```text
./Examples/RGB-D/geometric_warp_test
```

在完整运行库路径下结果：

```text
[Geometry G0-3R Test] PASS
```

### 7.2 一对一 association、geometry shadow 开启

配置：

```text
Examples/RGB-D/TUM3_GeometryShadow.yaml
Geometry.Enable: 1
Geometry.RegionGrowEnable: 0
无 YOLO 模型
viewer disabled
```

结果：

| 指标 | 数值 |
|---|---:|
| 输入帧 | 827 |
| geometry 日志样本的 `dt_s` 中位数 | 35.55 ms |
| geometry 日志样本的 `dt_s` 最大值 | 68.03 ms |
| geometry total 日志样本中位数 | 2.97 ms |
| geometry total 日志样本最大值 | 5.05 ms |
| tracking mean | 19.53 ms |
| active total mean | 28.63 ms |
| deadline missed | 103/827 |
| actual FPS | 28.29 |
| 保存轨迹 pose | 809 |
| APE RMSE（align） | 0.646 m |

`dt_s` 偶尔大于一个 frame interval，可能来自数据时间间隔和跟踪失败后的 reference 重建；需要在 G0-2P 逐帧记录有效 reference，而不能假定永远严格相邻。

### 7.3 同 association、geometry 关闭对照

| 指标 | 数值 |
|---|---:|
| 输入帧 | 827 |
| tracking mean | 18.26 ms |
| active total mean | 26.81 ms |
| deadline missed | 73/827 |
| actual FPS | 28.45 |
| 保存轨迹 pose | 827 |
| APE RMSE（align） | 0.542 m |

两次运行的新地图初始点数也不同（717 与 716）。ORB-SLAM2 存在 LocalMapping 等异步线程，单次运行结果不具有确定性。

因此只能得出：

```text
[O] geometry shadow 在本次运行增加了约 1–2 ms 的 tracking/active mean；
[O] 两次轨迹数量和 APE 不同；
[H] 额外前端耗时可能通过异步线程调度间接影响跟踪；
[禁止] 不能用一次 A/B 运行证明 geometry 计算导致轨迹退化；
[禁止] 也不能再宣称 shadow 在实验上“完全不影响轨迹”。
```

后续若要量化 timing-only 影响，应至少多次重复，并比较：

1. geometry 完整计算；
2. 等时但不读取 depth 的 sleep/busy-control；
3. geometry 关闭；
4. 相同 CPU/GPU、viewer、association 和线程设置。

这不是 G0-2P 的前置阻塞，但属于 30 FPS 与可重复性风险。

## 8. `rgbd_tum` association 解析修正

原 `LoadImages()`：

- 不跳过注释；
- 不检查四列是否解析成功；
- association 文件打不开时不立即报错。

现已最小修正：

- 跳过空行和 `#` 注释；
- 每行必须解析为 `rgb_ts rgb_path depth_ts depth_path`；
- 文件打不开或行格式错误时 fail-fast。

首次验证确实捕获了旧解析器把注释行当成空图像的问题。该修改不改变正常四列 association 的配对内容。

## 9. 当前结论与下一步

G0-2C 的主要门控已经完成：

- TUM3 零畸变坐标域可用；
- 非零畸变数据被明确阻止；
- mask/keypoint 域关系已记录；
- RGB-depth association 中的 depth 复用已发现并隔离；
- 一对一诊断 association 已生成；
- RGB/depth/GT 时间差已量化；
- geometry reference 时间差已进入日志。

下一步进入 G0-2P：

```text
同一 reference depth/current depth
+ 同一 rasterization/K
+ SLAM initial pose 路径
+ 插值 GT pose 路径
→ 分别输出 valid coverage、signed residual 和 positive/negative ratio
```

G0-2P 仍保持 shadow-only，不进入 `mvbDynamic`，不调用额外 `PoseOptimization()`。

## 10. 文件清单

代码与配置：

- `DT-SLAM/tools/audit_rgbd_geometry_inputs.py`
- `DT-SLAM/tools/make_rgbd_association.py`
- `DT-SLAM/Examples/RGB-D/TUM3_GeometryShadow.yaml`
- `DT-SLAM/Examples/RGB-D/rgbd_tum.cc`
- `DT-SLAM/include/GeometricDynamicDetector.h`
- `DT-SLAM/src/GeometricDynamicDetector.cc`
- `DT-SLAM/src/Tracking.cc`

结果：

- `tum_fr3_walking_xyz_input_audit.json`
- `tum_fr3_walking_xyz_one_to_one_input_audit.json`
- `fr3_walking_xyz_associations_one_to_one_20ms.txt`
- `g0_2c_walking_xyz_827.log`
- `baseline_walking_xyz_827.log`
- `CameraTrajectory_geometry_shadow.txt`
- `CameraTrajectory_baseline.txt`
- `KeyFrameTrajectory_geometry_shadow.txt`
- `KeyFrameTrajectory_baseline.txt`
- `ape_geometry_shadow.txt`
- `ape_baseline.txt`

