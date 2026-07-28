# G0-2A-dynamic：TUM `fr3_walking_xyz` 自动代理审计

日期：2026-07-28

## 1. 目的与边界

本实验检查 G0-2 的 `positive_seed_mask` 是否在明确动态序列中具有足够的
空间定位能力。由于 TUM `fr3_walking_xyz` 没有逐像素运动真值，本次使用离线
YOLOv8n-seg 的 `person` mask 作为**动态区域代理标签**，避免人工逐帧标注。

必须保留以下解释边界：

- person mask 是类别代理，不是运动真值；
- person mask 内可能包含当前静止的人体像素；
- person mask 外可能包含漏检的人体或其他动态区域；
- 离线 Ultralytics 后处理与项目 C++ `YOLOSegment` 不是逐像素等价实现；
- 因此本实验可以判断“几何证据与人物区域的空间重合程度”，不能给出严格的
  motion precision/recall。

Geometry 全程保持 shadow-only：

- `Geometry.RegionGrowEnable: 0`；
- 不写入 `Frame::mvbDynamic`；
- 不清除 `mvpMapPoints`；
- 不修改 `Optimizer.cc`；
- 不增加 PoseOptimization；
- 不改变关键帧或 MapPoint 创建。

## 2. 数据与复现实验

### 数据

- 序列：TUM `rgbd_dataset_freiburg3_walking_xyz`
- 使用帧数：827
- RGB-depth 关联：
  `results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt`
- 关联最大时间差：6.168 ms
- 配置：`DT-SLAM/Examples/RGB-D/TUM3_GeometryDynamicProxy.yaml`
- 几何残差阈值：0.10 m（实验参数，不是 DynaSLAM 参数）

没有使用原序列中存在重复匹配的 859 对 `associations.txt`。

### 离线 person 代理 mask

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

python3 tools/generate_semantic_proxy_masks.py \
  weights/yolov8n-seg.onnx \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt \
  /home/zhu/dynaslam_ws/results/g0_2a_dynamic_2026-07-28/offline_person_proxy \
  --device cpu --confidence 0.5 --iou 0.45 --image-size 640
```

共生成 827 张 mask，其中 678 张非空、149 张为空。mask 使用 7x7 椭圆核膨胀，
近似当前 C++ 语义路径的边界留量。生成器复用同一个模型会话，但逐帧推理，因为
当前 ONNX 模型固定 batch=1。

### SLAM shadow 运行

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

export DT_SLAM_DISABLE_VIEWER=1
export DT_SLAM_PRECOMPUTED_MASK_DIR=/home/zhu/dynaslam_ws/results/g0_2a_dynamic_2026-07-28/offline_person_proxy
export DT_SLAM_GT_TRAJECTORY=/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt
export DT_SLAM_GT_DIAGNOSTIC_CSV=/home/zhu/dynaslam_ws/results/g0_2a_dynamic_2026-07-28/walking_pose_diagnostic.csv
export DT_SLAM_GEOMETRY_PROXY_CSV=/home/zhu/dynaslam_ws/results/g0_2a_dynamic_2026-07-28/walking_semantic_proxy.csv
export DT_SLAM_GEOMETRY_DEBUG_DIR=/home/zhu/dynaslam_ws/results/g0_2a_dynamic_2026-07-28/walking_debug

./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3_GeometryDynamicProxy.yaml \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt
```

本次执行环境看不到 `/dev/nvidia*`，在线 CUDA `YOLOSegment` 无法启动；项目又有意
禁止 CPU fallback，因此使用预计算 mask 输入完成诊断，没有修改 YOLO 推理逻辑。

## 3. 统计定义

令：

- `V`：当前帧具有有效 reference-to-current depth comparison 的像素；
- `S`：`residual > 0.10 m` 的 positive seed；
- `P`：离线 person proxy mask。

记录：

```text
proxy coverage       = |V ∩ P| / |P|
proxy precision      = |S ∩ P| / |S|
conditional recall   = |S ∩ P| / |V ∩ P|
proxy-background rate= |S ∩ ¬P| / |V ∩ ¬P|
```

最后一项不能称为严格的静态背景 FPR，因为 `¬P` 不是静态真值。

## 4. 全序列结果

共记录 826 个相邻帧几何比较，825 个同时具有插值 GT pose。以下加权指标只统计
person proxy 非空的 677 个可比较帧。

| 指标 | SLAM pose | GT pose |
| --- | ---: | ---: |
| proxy coverage | 3.314% | 3.477% |
| proxy precision | 4.260% | 4.391% |
| conditional recall | 14.519% | 15.740% |
| proxy-background positive rate | 5.330% | 5.879% |

SLAM pose 的逐帧分布：

| 指标 | mean | median | p95 | max |
| --- | ---: | ---: | ---: | ---: |
| proxy coverage | 4.225% | 1.819% | 10.816% | 89.808% |
| proxy precision | 3.988% | 1.729% | 15.466% | 69.983% |
| conditional recall | 13.559% | 8.979% | 46.226% | 94.051% |
| proxy-background positive rate | 5.592% | 4.665% | 12.365% | 25.378% |

只保留 `|V ∩ P| >= 1000` 的帧后，proxy precision 仍只有 6.64%，conditional
recall 为 14.60%。因此低重合不能仅用“人物有效像素过少”解释。

## 5. GT pose 敏感性

GT pose 没有使人物区域覆盖、precision 或 conditional recall 出现实质改善。
这与 G0-2P 的结论一致：

> 当前失败不能主要归因于 ORB-SLAM2 初始位姿误差。

但该结论只排除了“pose 是主要单因”的解释，并不排除深度噪声、遮挡、时间同步、
标定近似和 forward rasterization 的共同影响。

## 6. 深度边缘诊断

对每 30 帧保存的 28 个样本、共 206,807 个 positive seed 做诊断。深度边缘定义
与 G0-2A-static 一致：

- 四邻域发生有效/无效深度转换；或
- 四邻域深度跳变大于 0.05 m。

这只是测量，不是新增动态检测算法。

| 与深度边缘的 Chebyshev 距离 | 全部 seed | person 内 seed | person 外 seed |
| --- | ---: | ---: | ---: |
| 边缘像素本身 | 57.39% | 36.64% | 57.89% |
| 1 像素内 | 86.75% | 68.53% | 87.18% |
| 2 像素内 | 95.43% | 84.03% | 95.71% |
| 3 像素内 | 98.36% | 90.52% | 98.55% |
| 5 像素内 | 99.66% | 96.15% | 99.75% |

静态 `fr1/xyz` 的对应结果为 86.20% 位于 2 像素内、93.53% 位于 3 像素内。
动态序列的 seed 更强烈地集中在深度边缘，尤其是 person mask 外的 seed。

代表帧可视化显示：

- 人物进入或遮挡边界上存在正残差证据；
- 人物内部大多没有有效 reference depth comparison；
- 墙面、屏风、显示器、桌椅和深度空洞边缘也产生大量正残差；
- 在 person proxy 为空的帧中仍可出现大面积背景边缘 seed。

## 7. 性能与轨迹完整性

| 指标 | 数值 |
| --- | ---: |
| tracking mean | 18.307 ms |
| active total mean | 27.631 ms |
| sequence wall time | 29.339 s |
| actual FPS | 28.189 |
| deadline misses | 30/827 |
| ATE translation RMSE | 0.014995 m |
| ATE translation mean | 0.012933 m |
| ATE translation max | 0.060115 m |

该运行同时计算 SLAM pose 与 GT pose 两套 geometry，并每 30 帧写调试图。它证明
当前 shadow 测量可运行，不证明加入后续过滤后仍能稳定达到 30 FPS。

## 8. 阶段结论

G0-2A-dynamic 自动代理审计完成，不需要用户人工逐帧标注。

可以确认：

1. 单参考帧 signed depth residual 在动态人物边界上确实产生类别无关证据；
2. 该证据目前主要是**遮挡/深度不连续边界证据**，不是完整对象 mask；
3. person 内有效比较覆盖仅约 3.3%，未覆盖区域必须保持 `unknown`，不能解释为静态；
4. direct positive seed 对 person proxy 的空间精度不足，不能进入 `mvbDynamic`；
5. GT pose 未实质修复该问题；
6. 当前证据不满足恢复 G0-3B/C 区域传播对照的门控条件。

因此冻结决策为：

```text
G0-2A-dynamic  = 完成（自动代理审计）
G0-3/G0-3R     = 失败 baseline，继续默认关闭
G0-3B/G0-3C    = 继续暂缓
G1-F/G1-D      = 不进入
```

下一步只允许进入 `G0-4F` 的 feature-level **shadow 诊断**：统计 ORB 特征是否能从
直接正残差或其小邻域获得高精度证据。该步骤仍不得过滤特征；只有 feature-level
precision 通过独立门控后，才讨论 G1-F。

## 9. 产物

- `walking_semantic_proxy.csv`：SLAM/GT pose 的代理重合统计
- `walking_pose_diagnostic.csv`：SLAM/GT pose 残差统计
- `walking_debug/`：valid、positive、negative、person proxy 和 overlay
- `walking_dynamic_proxy.log`：完整运行与计时日志
- `CameraTrajectory_walking_proxy.txt`
- `KeyFrameTrajectory_walking_proxy.txt`
- `evo_ape.txt`
- `offline_person_proxy/manifest.csv`

