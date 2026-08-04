# SInDSLAM mask 与模块贡献审计

日期：2026-08-02

## 1. 审计边界

本轮只审计独立 SInDSLAM 复现，不修改 DT-SLAM，也不改变作者的动态判决、阈值、区域重聚类或 ORB-SLAM2 优化逻辑。

在本地兼容分支的 non-ROS runner 中增加了默认关闭的调试输出：

- `SIND_SLAM_DEBUG_DIR`：输出目录；
- `SIND_SLAM_DEBUG_START/END`：只保存指定帧号区间；
- 保存 `DynaDetect` 返回的 mask、runner 再做 15x15 膨胀后的最终 mask、区域标签和 RGB overlay。

算法仍从序列第 0 帧完整运行。这样目标帧保留真实的 ORB-SLAM2 地图、上一帧 mask、区域标签和时序状态，避免从中途截断序列造成状态重置。

## 2. 重要实现事实

源码核对表明，当前公开 runner 的跟踪 mask 链路是：

```text
3D depth K-means
→ depth/plane edge re-clustering
→ RAG / histogram merge
→ dense optical flow
→ weighted homography camera-motion approximation
→ residual dual threshold
→ cluster-confined flood fill
→ internal 9x9 dilation
→ runner 15x15 dilation
→ ORB extraction skips mask == 255
```

需要特别区分：

1. 跟踪阶段的 ego-motion 补偿使用由 dense-flow correspondence 估计的全局 homography，不使用 ORB-SLAM2 的 `Tcw` 或 RGB-D SE(3) 投影；`DetectDynaArea()` 的接口也没有接收相机位姿。
2. 上一帧高残差 mask 会直接并入当前帧低残差 seed，上一帧动态区域还参与 homography 采样权重。
3. `DynaDetect` 内部已经做 9x9 膨胀；本轮所谓 `pre_runner_dilate` 只表示尚未执行 runner 的额外 15x15 膨胀，不是完全未经形态学处理的原始区域。
4. 论文 Dense Map Reconstruction 中每五帧利用 ORB-SLAM2 pose 做深度重投影 refinement 的 Eq. (17) 没有在当前公开源码中检索到实现；因此本轮只能审计实际参与 ORB 特征过滤的公开 tracking mask。
5. 论文没有报告逐像素 mask 的 precision/recall/IoU；其动态区域证据主要是定性图和轨迹/地图指标。因此论文 ATE 不能替代本轮 mask 审计。

## 3. 干净运行协议

数据：Bonn `rgbd_bonn_moving_nonobstructing_box`，778 个有效 RGB-depth 对。

配置：作者 `Bonn.yaml`，CPU DeepFlow，Pangolin viewer 关闭，动态检测开启。

保存两个完整序列运行中的帧段：

- `270--330`：人和箱子交互明显的片段；
- `720--777`：画面中无人、箱子已出现在地面的后期片段。后者没有逐像素运动真值，因此只作为强静态风险审计，不能把“看起来静止”写成严格 GT。

有一轮早期运行因两个执行会话同时写同一目录而被判无效，目录 `nonobstructing_270_330/` 不用于结论。正式统计只来自带 `_clean` 后缀且单进程退出码为 0 的目录。

## 4. mask 面积结果

### 4.1 帧 270--330

| 指标 | 结果 |
|---|---:|
| 帧数 | 61 |
| 算法内部 dynamic ratio 均值 / 中位数 | 12.92% / 13.53% |
| runner 膨胀后 dynamic ratio 均值 / 中位数 | 15.29% / 15.95% |
| runner 膨胀面积放大均值 / 中位数 | 1.237x / 1.196x |
| 全零动态帧 | 8/61 |
| 单帧最终最大覆盖 | 58.60% |

代表性观察：

- 帧 306、313 的 mask 覆盖了移动箱子的一部分；
- 帧 297、318、322 同时出现大块墙面、桌面、柜体或椅子区域；
- 大块区域在 `pre_runner_dilate` 中已经存在，额外 15x15 膨胀通常再增加约 20% 面积，但不是大块误标的根因。

### 4.2 帧 720--777

| 指标 | 结果 |
|---|---:|
| 帧数 | 58 |
| 算法内部 dynamic ratio 均值 / 中位数 | 3.80% / 0% |
| runner 膨胀后 dynamic ratio 均值 / 中位数 | 4.48% / 0% |
| runner 膨胀面积放大均值 / 中位数（仅非零帧） | 1.273x / 1.225x |
| 全零动态帧 | 37/58 |
| 单帧最终最大覆盖 | 39.48% |

代表性观察：

- 多数帧输出全零，说明算法不是持续无差别删除背景；
- 少数帧会突然把墙、桌面或柜体的大块区域标为动态；
- 前景箱子没有得到持续、稳定的区域覆盖；
- 这与全局 homography 在有深度视差的 RGB-D 场景中只能近似解释相机诱导光流的模型限制相符，但因没有像素运动 GT，当前只能把它列为有源码和图像证据支持的解释，不写成唯一因果结论。

## 5. 近似 box/person proxy 核对

现有箱框和 YOLO person mask 位于 DT-SLAM 的 Bonn 去畸变 `P=K` 域，而公开 SInDSLAM runner 直接使用 Bonn 原始像素并在 `Bonn.yaml` 中将畸变置零。两者坐标域并不严格一致，只能作粗略提示，不能报告成真值 precision/recall。

在可用的帧 297、306、313 上，最终 SIn mask 的大部分像素落在 coarse box/person proxy 之外；帧 306、313 对箱框有部分覆盖，帧 297 对箱框和人都几乎没有覆盖。这与 overlay 的人工目视结果一致：存在真实目标命中，但对象特异性不足。

## 6. 源码风险点

以下是公开实现的客观风险，不在本轮修改：

1. 单个 homography 无法精确表达一般 RGB-D 三维场景中的相机平移视差；静态深度层可能保留残余流。
2. 高/低阈值按每帧最大 residual 归一化，并由 Otsu/Triangle 自适应产生，极值和残差分布变化可能造成帧间判决跳变。
3. 上一帧高残差位置未经几何 warp 就并入当前低残差 seed，快速相机/对象运动时可能产生位置滞后。
4. `DynaDetect.cc` 的一个分支写成 `cv::countNonZero(thred2)`，其中 `thred2` 是标量；对称分支使用的是 `imgThhd1`。这很可能是公开源码中的实现错误，应在独立消融前单独验证，但本轮没有擅自修复。
5. flood-fill 的 `seedPoint` 只在 contour 与 low-error mask 相交时赋值，源码没有显式 `found` 检查；若没有交点，seed 的语义不可靠。
6. 图像尺寸和多项参数硬编码为 640x480/TUM-Bonn 风格，泛化到其他 RGB-D 输入需要额外审计。

## 7. 与轨迹结果的联合解释

本轮带调试输出的干净 non-obstructing 运行 ATE RMSE 为 `0.023843 m`，此前无调试单次运行为 `0.023097 m`，均处于论文同类结果量级。调试运行不是严格确定性复现，不能把两者差值归因于图像保存。

联合证据支持的最准确结论是：

> SInDSLAM 的完整区域/稠密光流链在该序列上能显著保护轨迹，但公开 tracking mask 并不是高精度、持续稳定的未知箱子分割。它通过较激进且间歇性的区域剔除仍可能获得良好位姿，这是“SLAM 鲁棒性”和“对象 mask 准确性”两个不同目标。

因此：

- 可以把 SInDSLAM 保留为强外部区域路线对照；
- 不能仅凭 ATE 把整个 `DynaDetect.cc` 直接移植进 DT-SLAM；
- 不能把 runner 15x15 膨胀当作唯一问题，内部区域判决本身已有大块误标；
- 当前更值得做的是少量、边界清楚的模块消融，而不是立刻整合。

## 8. 源码风险与时序先验 A/B

为避免改变官方默认行为，新增三个默认关闭的独立实验开关：

- `SIND_SLAM_FIX_THRESHOLD_MASK_COUNT=1`；
- `SIND_SLAM_REQUIRE_VALID_FLOOD_SEED=1`；
- `SIND_SLAM_DISABLE_TEMPORAL_PRIOR=1`。

三组运行均从第 0 帧处理到第 330 帧，并评价同一 270--330 帧段：

| 模式 | 内部 dynamic ratio 均值 | 中位数 | 最大值 | 全零帧 |
|---|---:|---:|---:|---:|
| 官方默认 | 13.45% | 12.75% | 59.95% | 8/61 |
| 两个源码风险修正 | 11.25% | 13.55% | 33.45% | 9/61 |
| 源码风险修正 + 关闭时序先验 | 11.10% | 12.93% | 30.99% | 9/61 |

单次结果显示两个源码风险修正降低了平均和最坏覆盖，关闭时序先验的额外变化很小；但是不能据此宣称已修好算法：

- 两次完全相同的官方默认路径之间，mask 本身已有平均 3.35% 图像像素发生变化，平均面积相差约 0.53 个百分点；
- 官方默认与源码风险修正版之间平均有 4.73% 像素变化，平均面积下降 2.20 个百分点；变化高于观察到的两次官方差异，但只有单次 variant 运行，尚不足以分离修正效果与非确定性；
- 帧 322 的可视化中，修正版虽减少了覆盖面积，墙面等静态区域仍被大块标红；关闭时序先验也没有消除该问题。

因此，本轮 A/B 的客观结论是：

> 两个明确源码风险值得保留为复现修正候选，但它们不是对象误分割的充分解释；上一帧 mask 先验也不是唯一根因。更基础的 dense-flow/homography residual 与区域判决仍决定主要行为。

## 9. 下一决策

优先级建议：

1. 两个源码风险和时序先验的开关化 A/B 已完成，没有证明它们能单独解决对象误分割；
2. 将 SInDSLAM 保留为轨迹鲁棒性强、计算较重的外部区域 baseline；
3. 不直接移植完整 `DynaDetect.cc`，也不把它的 mask 当作 DT-SLAM 深度过滤真值；
4. 若未来继续区域路线，应把“区域提议”和“运动判决”分成可独立评价的接口，并优先解决一般三维相机运动下的 residual 模型，而不是继续追加形态学参数；
5. 当前轻量 LK 继续作为默认关闭的稀疏实验 baseline，SInDSLAM 独立分支作为重型对照，两者都不应被包装成已完成的未知对象检测器。

当前仍不建议把完整 SInDSLAM 合并进 DT-SLAM。
