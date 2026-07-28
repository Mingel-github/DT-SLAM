# DT-SLAM G0-2V 可视化诊断实施与检查记录

日期：2026-07-26

## 1. 本阶段边界

G0-2V 只把 G0-2 已有证据保存为诊断图，不改变几何计算、SLAM 跟踪或地图状态。

保存内容：

- `valid.png`：存在参考预测深度和当前有效深度的像素；
- `positive.png`：`predicted_depth - current_depth > 0.10 m`；
- `negative.png`：`predicted_depth - current_depth < -0.10 m`；
- `overlay.png`：原 RGB 图像上的证据叠加，红色为 positive，蓝色为 negative。

明确未做：

- 区域生长；
- ORB 特征动态标记；
- 动态特征过滤；
- MapPoint 写入过滤；
- YOLO、Optimizer、g2o 或后端修改。

## 2. 软件接口

`Examples/RGB-D/TUM3.yaml` 新增两个默认关闭的诊断配置：

```yaml
Geometry.DebugSave: 0
Geometry.DebugEveryN: 30
```

只有同时满足以下条件才保存：

1. `Geometry.Enable: 1`；
2. `Geometry.DebugSave: 1`；
3. 环境变量 `DT_SLAM_GEOMETRY_DEBUG_DIR` 指向一个已存在的输出目录。

未设置输出目录时，程序输出警告并关闭保存，不影响几何 shadow 或 SLAM。

PNG 编码与写盘耗时通过独立的 `[Geometry G0-2V]` 日志报告，不计入
`GeometricWarpStats::totalMs`。

测试结束后，`TUM3.yaml` 已恢复：

```yaml
Geometry.Enable: 0
Geometry.DebugSave: 0
Geometry.DebugEveryN: 30
```

## 3. 构建与确定性测试

构建目标：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM
make -C build geometric_warp_test rgbd_tum -j"$(nproc)"
```

结果：

```text
[100%] Built target geometric_warp_test
[100%] Built target rgbd_tum
```

确定性测试：

```text
[Geometry G0-2 Test] PASS
```

编译仍会输出工程已有的 Eigen deprecated 和 ONNX Runtime C++17 相关警告；
本次没有新增编译错误。

## 4. TUM 短片段运行

数据：

- `rgbd_dataset_freiburg3_walking_xyz`
- associations 前 50 行
- 无 YOLO 模型参数
- Pangolin viewer 关闭
- 每 10 个几何结果保存一次，并额外保存第一个结果

生成帧：

```text
1, 10, 20, 30, 40
```

每帧 4 张图，共 20 张，均为 `640 x 480`：

- mask：8-bit grayscale；
- overlay：8-bit RGB PNG。

完整日志：

- `g0_2v_walking50.log`
- SHA-256：
  `4f36a08ac646a35a814f41bc5d6035f10c2dff325989c1b91e7d378c7dad27b6`

## 5. 运行数值

五次 PNG 保存耗时：

```text
12.0092, 12.4250, 12.0595, 11.6114, 11.7942 ms
```

平均约：

```text
11.9799 ms / 保存帧
```

这是 4 张 PNG 的同步编码与写盘诊断开销，不是几何算法开销。默认关闭时不存在该开销。

50 帧端到端统计：

```text
tracking mean       = 16.4773 ms
active_total mean   = 26.1061 ms
actual_fps          = 29.0808
deadline_missed     = 5 / 50
```

这些数字包含采样帧 PNG 写盘，不能用于替代此前 G0-2 默认关闭诊断输出时的实时性结果。

日志中的两个 G0-2 采样：

```text
frame 1:
positive_ratio = 0.0148839
negative_ratio = 0.0182898
total_ms       = 3.66836

frame 30:
positive_ratio = 0.0314831
negative_ratio = 0.0317386
total_ms       = 3.45652
```

## 6. 图像检查结果

人工检查以下文件：

- `walking50_images/frame_000001_overlay.png`
- `walking50_images/frame_000030_valid.png`
- `walking50_images/frame_000030_positive.png`
- `walking50_images/frame_000030_negative.png`
- `walking50_images/frame_000030_overlay.png`
- `walking50_images/frame_000040_overlay.png`

观察到：

1. `valid` 覆盖图像中大部分具有 RGB-D 观测的区域，并在无效深度、遮挡边界和
   投影未覆盖处保持为 0；因此 unknown 没有被编码成 static。
2. positive 和 negative 主要呈稀疏轮廓或窄带分布，而不是填满整个人体或完整物体。
3. 人物轮廓附近可见红/蓝证据，说明当前 residual 对相邻帧间遮挡变化有响应。
4. 桌椅、隔板边界及图像上方远处或细结构区域也存在明显红/蓝响应。
5. 正负证据在多处深度断层附近成对出现，符合重投影、遮挡转换和深度边界混合均会
   产生 signed residual 的现象；仅凭这些图不能把每一个响应归因为独立运动。

## 7. 对 G0-3 的门控结论

当前结果证明了诊断链路和空间对齐可用，但不支持“对所有 positive seed 直接做
无约束区域生长”。

原因是：种子不仅位于人物附近，也位于静态深度断层和远处细结构。若所有种子权重相同，
区域生长可能把边界误差扩展到静态表面。

因此，进入 G0-3 前至少应把以下内容写成可测的门控条件：

- 区域生长只能在 `validComparisonMask` 内进行；
- negative 继续只作诊断，不能作为当前动态前景种子；
- 生长必须受当前深度连续性约束，不能跨越强深度跳变；
- 需要记录 seed 所属连通域面积、增长后面积和增长倍率；
- G0-3 第一轮仍保持 shadow，不进行特征过滤；
- 用静态和动态短片段分别检查误扩展，再决定是否进入 G0-4 特征投影。

0.10 m 仍是诊断阈值，不在本阶段冻结。

## 8. 当前结论

G0-2V 已完成：

- 可重复输出 valid / positive / negative / overlay；
- mask 与 TUM RGB 图像均为 `640 x 480`；
- unknown 与静态证据保持分离；
- 可视化默认关闭；
- 写盘耗时与几何算法耗时分离；
- 未改变 SLAM 结果路径。

下一步可以设计受约束的 G0-3 shadow region grow，但不能把当前稀疏 positive mask
直接当作完整动态 mask。
