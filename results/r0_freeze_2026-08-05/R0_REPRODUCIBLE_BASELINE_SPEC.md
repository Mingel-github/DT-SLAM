# R0 可复现基线冻结协议

日期：2026-08-05
算法代码检查点：`16ea79dff58d7d20e7d18bf5ef1b41f6ed4a9284`
本阶段性质：实验基础设施冻结，不改变动态检测算法

## 1. 目的

R0 为后续 Gazebo 失败层审计提供唯一共享基线。后续实验不得在没有记录的情况下同时改变输入帧、相机参数、稠密光流、区域参数、动态判决、时序状态和 SLAM 过滤动作。

本阶段还专门回答一个运行歧义：Gazebo 600 帧片段中 YOLOv8-seg 的语义动态 mask 全零，但此前单轮纯 ORB-SLAM2 与 semantic-only 的 ATE 不同。R0 用三轮配对重复实验测量这类差异的自然范围。

## 2. 冻结输入

### 2.1 数据

- 数据集：`/data/dynaslam/datasets/dtslam_gazebo_person_box_tum_run1_2026-08-01`
- 片段：原始 `associations.txt` 第 4001–4600 行
- 帧数：600
- 时间范围：约 30 秒
- RGB-D 频率：20 Hz
- 分辨率：640×480
- 深度单位：输入 PNG `uint16`，`DepthMapFactor=5000`
- 轨迹参考：Gazebo `/odom` 与固定相机外参组合得到的相机轨迹

冻结 association：

```text
results/sindslam_gazebo_moving_box_2026-08-04/inputs/
moving_box_4001_4600_associations.txt
```

SHA-256：

```text
5345eb73d525bf3c909c430d192c4e235d6ab3190f58d6cf560629eb39d71a92
```

### 2.2 相机与 ORB 配置

基础配置：`DT-SLAM/Examples/RGB-D/GAZEBO.yaml`
SIn 风格配置：`DT-SLAM/Examples/RGB-D/GAZEBO_SInStyleNativeRegionDecisionCPU.yaml`

关键相机参数：

```text
fx = fy = 554.3827128226441
cx = 320.5
cy = 240.5
distortion = 0
fps = 20
ORB nFeatures = 1000
```

SIn 风格检测冻结为：

```text
CPU DeepFlow
3D coarse-to-fine K-means initial regions
depth-gradient split
RAG merge
homography ego-motion compensation
region-level double-threshold decision
previous-frame temporal prior
S2 Tracking/MapPoint filtering with fail-open
S3 depth mask output
```

### 2.3 正式运行条件

- Viewer：关闭；
- 纯 ORB 与 semantic-only 使用同一基础配置；
- semantic-only 使用 `yolov8n-seg.onnx` 和 CUDAExecutionProvider；
- 语义 mask 必须 600/600 同帧可用；
- 语义动态像素总数必须为 0；
- 不固定 ORB-SLAM2 线程调度，重复运行用于测量其系统级波动；
- 不将不同模式轨迹逐位一致作为要求。

## 3. 正式模式定义

### 3.1 纯 ORB-SLAM2

```text
semantic = off
geometry = off
depth filtering = off
```

### 3.2 全零语义路径

```text
semantic = on
geometry = off
semantic mask = all zero on this fragment
depth filtering mode = semantic_only
rejected depth pixels = 0
```

该模式仍会启动 YOLO 推理线程并通过语义/深度过滤代码路径，因此它与纯 ORB 模式并非相同的运行时路径。实验目的正是测量这一系统差异，而不是预设二者轨迹完全相同。

## 4. 运行接口

统一 runner：

```text
DT-SLAM/tools/run_sin_style_mode.py
```

每次运行独立保存：

- `run_manifest.json`；
- `run.log`；
- `CameraTrajectory.txt`；
- `KeyFrameTrajectory.txt`；
- semantic-only 的 `depth_filter.csv`。

汇总工具：

```text
DT-SLAM/tools/summarize_r0_gazebo_equivalence.py
```

评价协议：

- ATE：绝对轨迹误差，SE(3) Umeyama 对齐；
- RPE：相邻输出位姿间的平移相对位姿误差；
- 时间戳最大差：0.02 秒；
- 同时记录轨迹完整性、关键帧数、FPS 和 deadline miss。

## 5. 文件哈希

```text
ee5a9d1ed8088c4af9158ddd2139e53963fc273871be979a8f5132d64a701623  GAZEBO.yaml
76f82b23a8660ff47ea18dd2c6524242b560cf142df78bd4aec5bef28f2e5647  GAZEBO_SInStyleNativeRegionDecisionCPU.yaml
b07443764214df70f6a88d515be99716fe0b7b1f63fa30be8cf90c32ae5a9de1  run_sin_style_mode.py
8322eb07eb35045cc2181a90394386509a87133fae71a814b94f51b287e19786  summarize_r0_gazebo_equivalence.py
212f2834108da25a101bac6713aef696976eec6fe1c1f4c1ed1978e7cb398bd4  rgbd_tum
f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570  ORBvoc.txt
41be4a175399bae83d25c8ad4b95f963fa08734cae816da8c9be17c23e30c20a  yolov8n-seg.onnx
```

二进制哈希用于重现实验，不要求将二进制、词袋或模型提交 Git。

## 6. 完成标准

- 三轮纯 ORB 与全零语义配对实验全部完成；
- 每轮均输出 600/600 位姿；
- semantic-only 均使用 CUDA，mask 均为全零；
- ATE、RPE、关键帧和配对差异形成机器可读摘要；
- 配置、runner、协议、报告和小型摘要进入 Git；
- 大型图像、点云、数据集、模型和重复日志不进入 Git。

## 7. R0 边界

R0 不评价 SIn 风格几何是否正确，也不改变 detector。完成 R0 后，只进入 R1 的只读中间量审计。
