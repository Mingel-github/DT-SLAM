# 作者 SInDSLAM 非 ROS Tracking mask：Gazebo 箱子片段复现与普通点云结果

日期：2026-08-06

## 1. 本轮问题与边界

本轮只回答一个有限问题：

> 作者 SInDSLAM 的非 ROS Tracking detector 在同一段 Gazebo 移动箱子数据上，会产生怎样的轨迹、动态 mask 和普通累计点云？

明确不包含：

- ROS1 运行链；
- 五帧长间隔建图精修；
- OctoMap；
- 是否应搭建上述完整建图链的决策。

因此，本文中的“作者过滤点云”仅表示：使用作者非 ROS Tracking detector 输出的当前帧动态 mask，将对应深度像素从普通累计点云中剔除。它不是作者论文完整静态地图的复现。

## 2. 复现身份

- 作者仓库版本：`6e465591cd2c08f962c11654c410e527c3389d32`；
- 本轮执行入口：本地兼容构建的 `rgbd_tum_noros`；
- 数据：Gazebo 人＋箱子 TUM 格式数据第 4001–4600 个关联，共 600 帧；
- 相机：640×480，`fx=fy=554.3827`，`cx=320.5`，`cy=240.5`；
- detector：作者区域、DeepFlow、homography、区域分类和短时时序主逻辑；
- flow 后端：CPU DeepFlow；
- 兼容性修改：构建库路径可配置、允许关闭强制 OpenCV 窗口、导出审计图；没有改变本轮所用的动态判定公式和阈值。

作者 mask 是三值图：

```text
255 = dynamic
125 = static region
0   = unknown / uncovered
```

点云过滤和本轮审计只将 `mask >= 240` 解释为动态，不能把 `125` 错当成动态。

## 3. 运行结果

### 3.1 轨迹和速度

作者非 ROS 运行完整处理了 600/600 帧，输出 600 个相机位姿。

| 指标 | 作者非 ROS SInDSLAM |
| --- | ---: |
| ATE RMSE（绝对轨迹误差均方根） | 0.038781 m |
| RPE RMSE（逐帧相对位姿误差均方根） | 0.013086 m |
| detector 平均耗时 | 169.457 ms/帧 |
| ORB tracking 平均耗时 | 8.982 ms/帧 |
| 完整进程墙钟吞吐 | 约 4.37 FPS |

同一片段的已有 DT-SLAM 结果仅作描述性对照：

| 系统/模式 | ATE RMSE | RPE RMSE | 实测 FPS |
| --- | ---: | ---: | ---: |
| 纯 ORB-SLAM2 | **0.028619 m** | **0.010163 m** | 19.95 |
| 当前 DT-SLAM SIn 风格 geometry-only | 0.037247 m | 0.010713 m | 5.30 |
| 作者非 ROS SInDSLAM | 0.038781 m | 0.013086 m | 约 4.37 |

作者系统与 DT-SLAM 不是同一个可执行文件，因此小差异不能归因于某一个 detector 组件。可以确认的是：作者原版 Tracking detector 在该片段上也没有优于纯 ORB-SLAM2。

## 4. 箱子 mask 审计

参考由 Gazebo 相机真值、箱子真值、实测深度和已知 0.6 m 箱体体积共同生成。它近似可见箱体深度像素，但不是严格的 instance-segmentation 真值；因此“mask 中属于箱子的比例”只作为对象特异性诊断，不称为标准 precision。

### 4.1 作者原版与当前 DT-SLAM 对照

| 指标 | 作者非 ROS SIn | 当前 DT-SLAM S3 mask |
| --- | ---: | ---: |
| 有 mask 的帧 | 599 | 599 |
| 箱子足够可见的帧 | 143 | 143 |
| 箱子像素加权覆盖率 | 60.91% | 64.15% |
| 逐帧箱子覆盖率中位数 | 2.35% | 0% |
| 覆盖低于 25% 的可见帧 | 85/143 | 82/143 |
| 覆盖至少 75% 的可见帧 | 54/143 | 58/143 |
| 全部 mask 像素中落入箱子代理的比例 | 3.18% | 3.60% |
| 箱子可见帧 mask 像素中落入箱子代理的比例 | 13.84% | 15.49% |
| 平均整图动态 mask 比例 | 17.41% | 16.19% |
| 箱子不可见但仍有非零 mask | 452/454 | 452/454 |

作者原版并未消除当前 DT-SLAM 已观察到的主要失败：

1. 箱子较小或较远时经常几乎完全漏检；
2. 箱子不可见时仍在大量背景区域产生动态 mask；
3. mask 对箱子的对象特异性很低。

### 4.2 尺度分组

| 箱子图像面积分组 | 作者 SIn 平均覆盖 | 当前 DT-SLAM 平均覆盖 |
| --- | ---: | ---: |
| 最小四分之一 | 8.26% | 6.28% |
| 次小四分之一 | 11.35% | 11.09% |
| 次大四分之一 | 42.97% | 50.96% |
| 最大四分之一 | 97.14% | 98.43% |

两套实现的形态一致：远小箱子覆盖弱，近大箱子覆盖强。该结果说明当前 Gazebo 失败不是 DT-SLAM clean-room 实现单独引入的现象；但它还不能区分根因来自 DeepFlow、homography、区域合并还是 classifier。

可视化：

- 绿色轮廓：箱子深度体积代理；
- 红色：作者动态 mask；
- 黄色：二者交集。

文件：`box_mask_audit/contact_sheet.png`。

## 5. 普通累计点云

使用作者输出的相机轨迹，在完全相同的 299 个采样帧、像素步长 8、最大深度 6 m 下导出两份点云。第 0 帧没有光流参考，因此从成对比较中跳过。

三份 Viewer 点云与轨迹指标的对应关系如下。PCL Viewer 本身只显示点云，不显示 ATE/RPE：

| Viewer 点云 | 所用相机轨迹 | ATE RMSE | RPE RMSE |
| --- | --- | ---: | ---: |
| 作者轨迹、未过滤深度 | 作者非 ROS SInDSLAM 的同一条 600 帧轨迹 | 0.038781 m | 0.013086 m |
| 作者轨迹、作者 Tracking mask 过滤深度 | 与上一项完全相同；只离线改变写入点云的深度 | 0.038781 m | 0.013086 m |
| 当前 DT-SLAM S3 过滤点云 | 当前 DT-SLAM geometry-only 轨迹 | 0.037247 m | 0.010713 m |

前两份点云不是两次 Tracking 实验，因此不能从它们得到两个不同 ATE。它们用于在固定作者位姿下隔离观察深度过滤效果。作为额外定位参照，同片段纯 ORB-SLAM2 的 ATE/RPE 为 `0.028619/0.010163 m`。

| 点云 | 点数 | 相对未过滤变化 |
| --- | ---: | ---: |
| 作者轨迹、未过滤深度 | 1,025,622 | — |
| 作者轨迹、作者 Tracking mask 过滤 | 777,726 | 删除 247,896（24.17%） |
| 当前 DT-SLAM 轨迹、当前 S3 mask 过滤 | 791,311 | 删除 234,311（22.85%） |

作者 mask 比当前 DT-SLAM mask 删除了更多深度观测，但箱子加权覆盖率反而略低，说明增加的删除量主要没有落在箱子代理内。仅凭“删除更多点”不能解释为地图更干净。

Viewer 中仍能看到较长的箱子运动轨迹，这与逐帧 mask 审计一致：作者 mask 在 143 个箱子足够可见帧中，有 85 帧的箱子覆盖低于 25%，逐帧覆盖率中位数只有 2.35%。箱子靠近相机后覆盖接近完整，所以残影密度会下降；但较远阶段的大量漏检深度已经写入普通累计点云，最终仍保留连续轨迹。当前结果只能称为“减轻箱子残影”，不能称为“清除移动箱子”。

本轮点云不是同位姿跨 detector 对照：作者点云使用作者轨迹，DT-SLAM S3 点云使用 DT-SLAM 轨迹。因此点数删除率可比较，空间重合细节只能作视觉参考，不能把差异全部归因于 mask。

点云文件：

```text
/data/dynaslam/large_results/sindslam_original_gazebo_2026-08-06/pointclouds/author_unfiltered_stride8_step2.pcd
/data/dynaslam/large_results/sindslam_original_gazebo_2026-08-06/pointclouds/author_tracking_mask_filtered_stride8_step2.pcd
```

当前 DT-SLAM S3 对照：

```text
results/sindslam_gazebo_moving_box_2026-08-04/map_geometry_filtered_stride8_step2.pcd
```

## 6. 客观结论

本轮支持以下结论：

1. 作者非 ROS SInDSLAM Tracking detector 已在相同 Gazebo 600 帧片段上完整运行；
2. 它与当前 DT-SLAM SIn 风格实现呈现相近的尺度依赖和背景误响应；
3. 它没有改善该片段的 ATE，也没有解决远小箱子漏检；
4. 将作者 Tracking mask直接用于普通累计点云，会删除约四分之一深度观测，但仍不能仅凭这一点宣称移动箱子残影已被清除；
5. 当前 DT-SLAM 的 Gazebo问题不能简单归因于 clean-room实现偏离作者代码，作者原版当前帧 detector也出现了同类现象。

本结论只覆盖作者非 ROS Tracking mask。本文没有运行、评价或决定 ROS1 五帧精修与 OctoMap 路线。

## 7. 证据文件

- `author_run.log`：600 帧作者运行日志；
- `CameraTrajectory.txt`：作者相机轨迹；
- `author_ape.zip`、`author_rpe.zip`：evo 轨迹评价；
- `box_mask_audit/summary.json`、`per_frame.csv`、`contact_sheet.png`：作者 mask 审计；
- `current_dt_mask_audit_recomputed/`：用同一评测器重算的 DT-SLAM S3 mask 对照；
- `author_pointcloud_summary.json`：点云点数与过滤比例；
- `/data/dynaslam/large_results/sindslam_original_gazebo_2026-08-06/masks/`：作者逐帧三值 mask、labels 和 overlay；
- `/data/dynaslam/large_results/sindslam_original_gazebo_2026-08-06/pointclouds/`：作者未过滤/过滤点云。
