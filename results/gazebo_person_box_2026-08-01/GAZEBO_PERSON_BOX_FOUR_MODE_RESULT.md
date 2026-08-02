# DT-SLAM Gazebo 人与箱子组合动态场景四模式结果

日期：2026-08-01

## 1. 实验边界

本实验复用 `dynamic_rtabmap_sim` 的环形走廊：

- 东侧 `actor_patrol` 行人由 Gazebo trajectory 自动循环运动；
- 西侧 `dynamic_box` 由 `box_patrol` 在 `y=[-5,5] m` 往返运动；
- 机器人由用户通过 `turtlebot3_teleop` 人工驾驶一圈；
- 只录制原始 RGB-D、相机信息、仿真里程计、箱子里程计和 TF；
- 未启动旧工程的 `semantic_filter_node`、P5/P6/P7、RTAB-Map、RViz 或过滤后图像话题；
- DT-SLAM 四模式均在离线转换后的同一数据上运行。

## 2. 数据记录与转换

原始 rosbag：

```text
/data/dynaslam/datasets/dtslam_gazebo_person_box_manual_run1_2026-08-01
```

TUM 格式数据：

```text
/data/dynaslam/datasets/dtslam_gazebo_person_box_tum_run1_2026-08-01
```

记录结果：

| 项目               | 结果                                   |
| ---------------- | ------------------------------------:|
| bag 时长           | 342.557 s                            |
| RGB              | 6846 帧                               |
| depth            | 6846 帧                               |
| RGB CameraInfo   | 6846 条                               |
| depth CameraInfo | 6846 条                               |
| RGB-depth 最大时间差  | 0 s                                  |
| 原始 bag           | 13.8 GiB                             |
| 转换后 TUM 数据       | 2.2 GiB                              |
| 分辨率              | 640 x 480                            |
| 深度输入             | ROS `32FC1`，米                        |
| TUM 深度输出         | PNG `uint16`，`DepthMapFactor=5000`   |
| 相机内参             | fx=fy=554.382713, cx=320.5, cy=240.5 |
| 畸变               | 全零                                   |

机器人轨迹：

| 项目    | 结果               |
| ----- | ----------------:|
| 路径长度  | 59.632 m         |
| 终点到起点 | 0.442 m          |
| x 范围  | -8.277 至 7.491 m |
| y 范围  | -8.719 至 8.299 m |

箱子轨迹：

| 项目   | 结果             |
| ---- | --------------:|
| 路径长度 | 171.160 m      |
| x 范围 | 约 -7.0 m       |
| y 范围 | -5.01 至 5.01 m |

`/odom` 在录制前与 `gz model -m geo_bot -p` 对照，位置和姿态一致。导出时将
`base_footprint -> camera_link_optical` 固定外参组合到相机真值中。

抽样图像确认：

- 行人在东侧走廊清晰进入 RGB-D 视野；
- 移动箱子在西侧走廊清晰进入 RGB-D 视野；
- 无旧滤波 mask 或过滤图像介入。

## 3. 数据链 smoke 验证

首先只运行前 300 帧原始 ORB-SLAM2：

| 指标                | 结果         |
| ----------------- | ----------:|
| 初始化 MapPoint      | 856        |
| ATE RMSE          | 0.001466 m |
| tracking mean     | 8.930 ms   |
| active total mean | 13.342 ms  |
| actual FPS        | 19.944     |
| deadline miss     | 0 / 300    |

该结果说明 bag、RGB/depth 同步、深度尺度、内参和相机真值转换在短段上成立。

## 4. 四模式完整结果

所有 ATE/RPE 使用 SE(3) Umeyama 对齐；RPE 为相邻输出位姿的平移误差。

| 模式                | 输出位姿/6846 | 覆盖率    | ATE RMSE (m) | RPE RMSE (m) | active mean / p95 (ms) | actual FPS | deadline miss | 回环检测次数 |
| ----------------- | ---------:| ------:| ------------:| ------------:| ----------------------:| ----------:| -------------:| ------:|
| ORB baseline      | 6223      | 90.90% | 5.979586     | 0.265606     | 15.918 / 19.366        | 19.931     | 5             | 4      |
| semantic-only     | 6522      | 95.27% | 6.323276     | 0.227608     | 23.357 / 28.735        | 19.921     | 7             | 5      |
| geometry-only     | 6681      | 97.59% | 8.492072     | 0.314744     | 18.076 / 20.797        | 19.932     | 3             | 2      |
| semantic+geometry | 5763      | 84.18% | 5.386112     | 0.134932     | 26.396 / 31.854        | 19.929     | 13            | 3      |

语义侧：

- 两个语义模式均得到 `6846/6846` 个 mask；
- mask age 的 median 和 max 均为 0；
- semantic-only YOLO mean 7.390 ms；
- semantic+geometry YOLO mean 7.809 ms；
- CUDAExecutionProvider，RTX 4060 Ti。

几何侧：

| 模式                | G1-F1 应用帧 | 实际删除关联 | G1-M1 应用事件 | veto depth features |
| ----------------- | ---------:| ------:| ----------:| -------------------:|
| geometry-only     | 1269      | 5101   | 324        | 3203                |
| semantic+geometry | 1109      | 3897   | 230        | 1864                |

几何动作真实进入了 `TrackLocalMap` 关联清除和新 MapPoint 写入限制；这些计数只证明
执行链路工作，不证明所删除观测都属于动态箱子。

## 5. 客观结论

1. 本次原始仿真数据记录成功，人和箱子在整段录制中持续运动，并分别进入过相机视野。
2. 前 300 帧达到毫米级 ATE，证明输入和真值转换链路正确。
3. 完整一圈中四种模式均出现严重全局轨迹错误；因此该序列不能被描述为成功定位。
4. 日志中出现 2 至 5 次回环检测，且环境为重复纹理环形走廊。完整轨迹错误与回环事件同时出现，但当前实验尚未单独证明每次回环都是错误回环。
5. semantic-only 没有改善 ATE；geometry-only 明显恶化 ATE。
6. semantic+geometry 的 ATE 和逐帧 RPE 是四者中最低，但只输出 84.18% 的位姿。它是有限的相对改善，不足以证明系统可靠。
7. 四模式均维持约 20 FPS；semantic+geometry 的 active p95 为 31.854 ms，在 50 ms 仿真帧周期内仍有余量。
8. 当前结果再次表明：高残差特征过滤会改变 Tracking，但现有规则尚不能稳定等同于未知动态箱子检测。

## 6. 结果位置

```text
/home/zhu/dynaslam_ws/results/gazebo_person_box_2026-08-01/
  orb_baseline/
  semantic_only/          # 受限环境看不到 GPU 的失败启动记录
  semantic_only_gpu/      # 正式结果
  geometry_only/
  semantic_geometry_gpu/
```

每个正式目录保存：

- `run.log`；
- `run_manifest.json`；
- `CameraTrajectory.txt`；
- `KeyFrameTrajectory.txt`；
- `ape.zip`；
- `rpe.zip`；
- 几何模式另含 tracking/mapping CSV。

## 7. 本轮工程改动

新增：

```text
DT-SLAM/tools/export_rosbag2_tum_rgbd.py
DT-SLAM/Examples/RGB-D/GAZEBO.yaml
DT-SLAM/Examples/RGB-D/GAZEBO_GeometrySparseEgoFlow.yaml
```

未修改：

- YOLO 推理代码；
- `Optimizer.cc`；
- g2o；
- Tracking/Mapping 核心逻辑；
- ROS2 旧 P5/P6/P7 滤波实现；
- RTAB-Map。

上述新增文件和本报告当前尚未提交。

## 8. Viewer 独立回放

正式计时结束后，单独回放 associations 第 4001 至 4600 行的 600 帧西侧箱子片段：

- 模式：semantic+geometry；
- Pangolin Viewer：开启；
- CUDA 语义：开启；
- 运行时长：30.075 s；
- actual FPS：19.950；
- deadline miss：0/600；
- 正式四模式结果未被该回放覆盖，临时诊断 CSV 写入 `/tmp`。

当前 Viewer 只显示 ORB-SLAM2 相机、关键帧和 MapPoint，不显示彩色几何候选框，
因此该回放用于观察跟踪与建图过程，不作为箱子检测精度证据。
