# DT-SLAM G0-2P：GT 与 SLAM 位姿敏感性诊断记录

日期：2026-07-27  
阶段：G0-2P  
性质：shadow-only pose sensitivity diagnostic

## 1. 目的与边界

G0-2P 在完全相同的：

- reference depth；
- current depth；
- 纯针孔 `K`；
- dense forward warp；
- z-buffer；
- signed residual；
- `0.10 m` 临时 evidence threshold；

上并行比较两种相对位姿：

```text
SLAM:
T_current<-reference =
    Tcw_current_initial · inverse(Tcw_reference_final)

GT:
T_current<-reference =
    Tcw_current_GT · inverse(Tcw_reference_GT)
```

GT 只进入第二个 `GeometricDynamicDetector`。它不进入 Tracking、Optimizer、MapPoint、KeyFrame 或最终轨迹。

本阶段不评价 region mask，也不把 positive seed 写入 `mvbDynamic`。

## 2. GT 定义与转换

TUM groundtruth 文本格式：

```text
timestamp tx ty tz qx qy qz qw
```

官方定义中，平移是彩色相机光心在世界坐标中的位置，四元数是彩色相机相对于世界坐标的方向。因此本实现按 `Twc` 读取，再计算：

```text
Rcw = Rwc^T
tcw = -Rcw · twc
```

得到 DT-SLAM 使用的 `Tcw`。

时间处理：

- 位移线性插值；
- 四元数 SLERP；
- 只在前后 GT 样本都不超过 `20 ms` 时插值；
- 不做轨迹范围外外推；
- 827 个 RGB timestamp 中 826 个可用。

官方资料同时说明 TUM fr3 RGB/depth 已预注册且已去畸变，fr3 畸变参数为零。这与 G0-2C 的坐标域门控一致。

## 3. 实现

### 3.1 API

`System::TrackRGBD()` 增加一个默认空矩阵参数：

```cpp
const cv::Mat &TcwGroundTruth = cv::Mat()
```

已有调用不传 GT 时行为不变。

### 3.2 启用方式

```text
DT_SLAM_GT_TRAJECTORY=/path/to/groundtruth.txt
DT_SLAM_GT_MAX_BRACKET_DELTA_S=0.02
DT_SLAM_GT_DIAGNOSTIC_CSV=/path/to/output.csv
```

配置：

```text
Examples/RGB-D/TUM3_GeometryPoseDiagnostic.yaml
```

### 3.3 测量扰动修正

第一次试跑使用 `Geometry.LogEveryN=1`，大量 stdout 使运行只保存 666/827 个轨迹 pose。该结果不能用于比较 tracking。

随后改为：

- G0 终端日志每 30 个计算帧输出；
- 每帧 SLAM/GT 配对统计保存在内存；
- `System::Shutdown()` 后一次写 CSV。

这消除了逐帧磁盘/终端 I/O，但双 geometry 计算本身仍有约 6 ms/frame，仍会改变 ORB-SLAM2 异步线程调度。因此 G0-2P 的帧内 residual 对照可用，运行轨迹和 ATE 不应与 baseline 做因果比较。

## 4. 数据与运行条件

序列：

```text
TUM fr3/walking_xyz
```

association：

```text
results/g0_2c_2026-07-27/
fr3_walking_xyz_associations_one_to_one_20ms.txt
```

条件：

- 827 个一对一 RGB-depth pair；
- 无 depth 复用；
- 最大 RGB-depth 差 6.17 ms；
- 无 YOLO；
- region grow 关闭；
- viewer 关闭。

## 5. 缓冲版结果

共保存 682 个具有同一 reference/current depth 的 SLAM/GT 配对。

| 指标 | SLAM pose 路径均值 | GT pose 路径均值 | SLAM−GT |
|---|---:|---:|---:|
| comparison coverage | 0.5461 | 0.5493 | -0.0033 |
| mean absolute residual | 0.0763 m | 0.0858 m | -0.0095 m |
| positive ratio | 0.0581 | 0.0651 | -0.0070 |
| negative ratio | 0.0612 | 0.0656 | -0.0044 |
| geometry total | 2.97 ms | 3.17 ms | -0.20 ms |

GT mean absolute residual 低于 SLAM 的帧数：

```text
206 / 682
```

因此，walking_xyz 上不存在“GT residual 总是更低”的关系。

reference 时间差：

| 指标 | 数值 |
|---|---:|
| 中位数 | 32.21 ms |
| P95 | 38.67 ms |
| 最大值 | 69.86 ms |

## 6. 正确解释

不能从上述结果得出：

```text
GT pose 比 SLAM pose 差
```

原因至少包括：

1. `walking_xyz` 含真实独立运动人物；
2. 物理 GT 只描述相机运动，不会吸收人体运动；
3. SLAM 初始位姿由当前图像/地图匹配估计，可能部分吸收动态观测，从而在同一帧上降低 depth residual；
4. GT 较高的 positive ratio 既可能是真动态证据，也可能包含深度噪声、遮挡和 rasterization；
5. forward warp 的离散化误差会随旋转、深度边缘和可见性变化；
6. RGB、depth 和 GT 虽已对齐到毫秒级，仍不是硬件同时曝光；
7. 当前没有逐像素 dynamic/static ground-truth mask。

例如最大差异附近的部分帧具有约 4.5–5.2 度/帧的 GT 旋转；较大的 GT residual 与快速旋转同时出现，但这只是相关观察，不能单独证明原因。

当前能够支持的结论只有：

```text
[O] direct residual 和 positive/negative ratio 对位姿来源明显敏感；
[O] GT 路径不保证 residual 数值更小；
[O] walking 动态序列无法判断哪条 seed precision 更高；
[H] SLAM pose 可能通过吸收动态观测降低当前帧 residual；
[需验证] 真正静态序列上 GT 是否系统性降低背景 FPR。
```

## 7. 实时性与轨迹扰动

缓冲版双路径运行：

| 指标 | 数值 |
|---|---:|
| tracking mean | 22.38 ms |
| active total mean | 30.93 ms |
| actual FPS | 27.64 |
| deadline missed | 260/827 |
| 保存轨迹 pose | 683/827 |

这不是最终系统性能：

- G0-2P 同时计算 SLAM 和 GT 两套 dense warp；
- 它是诊断模式，不属于最终在线 pipeline；
- 轨迹数量减少表明额外计算确实能通过线程调度改变 ORB-SLAM2 行为；
- 因而不能拿本次 ATE 评价 geometry 方法。

最终在线系统不会包含 GT 分支。

## 8. 阶段判断

G0-2P 的基础设施已经建立并通过运行验证，但科学门控尚未完成：

```text
基础设施：通过
pose sensitivity：已观察
seed precision：未知
GT 是否改善静态背景：未知
是否进入 region growing：否
是否进入 G1-F/G1-D：否
```

## 9. 下一步

严格按冻结路线，下一步是 G0-2A，而不是继续改 propagation：

1. 获取真正静态负样本；
2. 准备少量逐像素 `dynamic/static/ignore/invalid` 标注；
3. 在真正静态序列比较 SLAM/GT background FPR；
4. 在 walking/运动箱子帧比较 direct seed precision 与 conditional recall；
5. 扫描 residual threshold，输出 precision-recall-threshold 曲线。

当前本地没有真正静态 TUM 序列，因此不应使用 `sitting_static` 替代。数据获取或自采静态序列是下一实际门控。

## 10. 产物

代码/配置：

- `DT-SLAM/Examples/RGB-D/TUM3_GeometryPoseDiagnostic.yaml`
- `DT-SLAM/Examples/RGB-D/rgbd_tum.cc`
- `DT-SLAM/include/System.h`
- `DT-SLAM/src/System.cc`
- `DT-SLAM/include/Tracking.h`
- `DT-SLAM/src/Tracking.cc`
- `DT-SLAM/tools/summarize_geometry_pose_diagnostic.py`

结果：

- `g0_2p_walking_xyz_buffered.log`
- `g0_2p_walking_xyz_buffered.csv`
- `g0_2p_walking_xyz_buffered_summary.json`
- `CameraTrajectory_g0_2p_buffered.txt`
- `KeyFrameTrajectory_g0_2p_buffered.txt`

