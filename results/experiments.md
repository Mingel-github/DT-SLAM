# DT-SLAM 实验记录

> TUM fr3_walking_xyz | 827帧 | YOLOv8n-seg + ONNX Runtime CPU | person类全量删除

---

## 历次运行（按时间排列）

### Baseline（纯几何，无mask，等价于原始ORB-SLAM2）

| #   | 日期    | 配置       | ATE RMSE | Mean  | Std   | 跟踪      | 备注    |
| --- | ----- | -------- |:--------:|:-----:|:-----:|:-------:| ----- |
| 1   | 07-18 | headless | 0.278    | 0.250 | 0.122 | 827/827 |       |
| 2   | 07-18 | 可视化      | 0.275    | 0.243 | 0.130 | 561/827 | 丢266帧 |
| 3   | 07-18 | +RPE评测   | 0.285    | 0.251 | 0.136 | 827/827 |       |

**ATE 波动范围**：0.275-0.285（取 3 次）

### 语义（YOLOv8n-seg, ONNX Runtime CPU）

| #     | 日期        | 配置                      | ATE RMSE  | Mean      | Std       | 跟踪          | mask就绪 | mask年龄      | YOLO耗时      | 备注       |
| ----- | --------- | ----------------------- |:---------:|:---------:|:---------:|:-----------:|:------:|:-----------:|:-----------:| -------- |
| 1     | 07-18     | proto未修复（全图resize→bbox） | 0.287     | 0.254     | 0.140     | 827/827     | 827    | —           | —           | mask严重变形 |
| 2     | 07-18     | proto未修复                | 0.279     | 0.243     | 0.137     | 827/827     | —      | —           | —           |          |
| 3     | 07-18     | proto未修复（Pangolin可视化）   | 0.260     | 0.225     | 0.130     | 827/827     | —      | —           | —           |          |
| **4** | **07-18** | **proto修复+裁剪**          | **0.197** | **0.181** | **0.077** | **827/827** | 825    | med=3 max=4 | mean=66.7ms | **最佳**   |
| 5     | 07-18     | proto修复，+RPE评测          | 0.254     | 0.216     | 0.133     | 827/827     | 825    | med=3 max=4 | mean=68.4ms |          |

**ATE 波动范围**：0.197-0.287。proto 修复前 ~0.26-0.29，修复后最佳 0.197，但第 5 次回升至 0.254。**RANSAC 随机性导致单次跑 ±0.05m 波动**，需要多轮取均值才具有统计意义。

---

## RPE 评测（仅第 5 次跑）

| 指标               | Baseline(#3) | 语义(#5)    | 改善       |
| ---------------- |:------------:|:---------:|:--------:|
| RPE 平移 RMSE（m/帧） | 0.0092       | 0.0102    | -11%     |
| RPE 平移 Mean（m/帧） | 0.0082       | 0.0086    |          |
| RPE 旋转 RMSE（°/帧） | 0.644        | **0.511** | **+21%** |
| RPE 旋转 Mean（°/帧） | 0.516        | 0.383     |          |

---

### 语义 + mask膨胀 + proto margin + 初始化同步（07-19，3次）

> 相较上一节"语义"新增：① mask dilation 7×7椭圆核(~3px) ② proto crop 10% margin ③ 初始化同步等首帧mask ④ sigmoid NaN夹断 ⑤ mProcessedFrames→atomic。①②③可能影响ATE，④⑤纯防御性。

| #   | 日期    | ATE RMSE  | Mean  | Std   | 跟踪      | mask就绪 | mask年龄      | YOLO耗时      | 备注        |
| --- | ----- |:---------:|:-----:|:-----:|:-------:|:------:|:-----------:|:-----------:| --------- |
| 1   | 07-19 | 0.532     | 0.456 | 0.273 | 827/827 | 827    | med=3 max=4 | mean=69.0ms | RANSAC离群点 |
| 2   | 07-19 | **0.181** | 0.119 | 0.137 | 827/827 | 827    | med=3 max=4 | mean=67.9ms | **当前最佳**  |
| 3   | 07-19 | 0.330     | 0.276 | 0.180 | 823/827 | 827    | med=3 max=4 | —           | 丢4帧       |

**ATE 范围**：0.181-0.532，比上节（0.197-0.287）波动更大。跑2达历史最佳 0.181（vs 之前最佳 0.197），但跑1/3偏高，需更多轮取均值。

### RPE（新3次）

| #   | RPE平移RMSE（m/帧） | RPE平移Mean | vs 之前最佳(0.0102) |
| --- |:--------------:|:---------:|:---------------:|
| 1   | 0.0229         | 0.0189    | -124%           |
| 2   | 0.0193         | 0.0148    | -89%            |
| 3   | 0.0431         | 0.0202    | -323%           |

RPE 全部比之前差。mask膨胀后动态区域覆盖面积增大→可用特征点减少→帧间匹配质量下降，RPE上升是预期内的代价。

## 论文参考（同序列 TUM fr3_walking_xyz）

| 方法              | ATE RMSE        | 来源                |
| --------------- |:---------------:| ----------------- |
| ORB-SLAM2       | 0.459           | DynaSLAM Table II |
| ORB-SLAM2       | 0.752           | DS-SLAM Table III |
| DynaSLAM N+G    | **0.015**       | SOTA 参考           |
| DS-SLAM         | 0.0247          |                   |
| DGS-SLAM        | 0.0156          |                   |
| PPS-SLAM        | 0.0156          |                   |
| **DT-SLAM**（我们） | **0.181~0.330** | 仅语义+膨胀，无几何模块      |

> 注：论文报告值是多次跑均值。我们目前单次跑波动大（±0.05m），报告范围而非单点值更诚实。

---

## G0-2A：TUM fr1/xyz 静态负样本 shadow（2026-07-27）

本实验使用专用 raw registered geometry 针孔模型
`525/525/319.5/239.5`，同时保留 ORB-SLAM2 原生 TUM1 非零畸变 Tracking
标定。Region growing 关闭，几何结果不进入 SLAM。

| 指标 | SLAM pose | GT pose |
| --- | ---: | ---: |
| 成对诊断帧 | 786 | 786 |
| valid comparison coverage mean | 71.68% | 71.63% |
| absolute residual mean | 0.00964 m | 0.01154 m |
| positive seed ratio mean | 0.521% | 0.687% |
| positive seed ratio p95 | 1.601% | 2.200% |
| geometry mean | 3.48 ms | 3.91 ms |

轨迹验证：ATE RMSE `0.009922 m`，RPE translation RMSE `0.006052 m/frame`。
SLAM/GT positive ratio 相关系数为 `0.914`，GT 并未消除背景 seed。

27 个抽样帧中，86.20% 的 positive seed 位于深度跳变边缘 2 像素内，
93.53% 位于 3 像素内。因此当前 direct seed 应解释为边界集中的几何不一致
证据，不能直接作为动态 mask 或进入 G1。

完整报告：
`results/g0_2a_2026-07-27/REPORT.md`

---

## G0-2A-dynamic：TUM fr3/walking_xyz 自动 person 代理审计（2026-07-28）

为避免人工逐帧标注，使用离线 YOLOv8n-seg `person` mask 作为动态区域代理标签。
该 mask 不是运动真值，person 外区域也不能直接视为静态背景。Geometry region
growing 关闭，全部结果保持 shadow-only。

| 加权指标（person proxy 非空帧） | SLAM pose | GT pose |
| --- | ---: | ---: |
| person proxy 有效比较覆盖 | 3.314% | 3.477% |
| positive seed 落入 person proxy | 4.260% | 4.391% |
| person 有效像素中的 conditional recall | 14.519% | 15.740% |
| proxy 外 positive rate | 5.330% | 5.879% |

28 个调试抽样帧中，95.43% 的 positive seed 位于深度跳变边缘 2 像素内，
98.36% 位于 3 像素内；person proxy 外对应比例更高。可视化显示证据集中在人物
进入边界，同时也大量出现在屏风、墙面、桌椅、显示器和深度空洞边缘。

GT pose 没有实质改善 overlap。当前 direct seed 应解释为类别无关的局部遮挡/
深度边界证据，不是完整动态区域，不能进入 `mvbDynamic`。G0-3B/C 继续暂缓；
下一步只进入 G0-4F feature-level shadow 诊断。

性能：827 帧，tracking mean `18.307 ms`，active total mean `27.631 ms`，
actual FPS `28.189`；ATE translation RMSE `0.014995 m`。

完整报告：
`results/g0_2a_dynamic_2026-07-28/REPORT.md`

---

## G0-4F：ORB feature-level shadow evidence（2026-07-28）

对 `Frame::mvKeys` 统计距 valid/positive mask 0、1、2、3 像素内的特征，
不写 `mvbDynamic`，不修改 Tracking 或地图。person proxy 非空的 walking
677 帧加权结果：

| 半径 | person eligible coverage | proxy precision | conditional recall | proxy 外候选率 |
| ---: | ---: | ---: | ---: | ---: |
| 0 px | 5.328% | 7.090% | 15.281% | 7.561% |
| 1 px | 7.947% | 6.738% | 21.922% | 15.693% |
| 2 px | 11.420% | 6.880% | 24.654% | 23.392% |
| 3 px | 15.391% | 7.276% | 26.389% | 30.235% |

fr1/xyz 真正静态对照的候选/eligible 为：

```text
r0=1.108%, r1=2.721%, r2=4.533%, r3=6.479%
```

动态场景确实带来更多几何不一致，但扩大窗口主要增加 proxy 外候选，precision
没有改善；GT pose 也不改变结论。G0-4F 没有通过 G1-F 门控，继续保持 shadow-only。

完整报告：
`results/g0_4f_2026-07-28/REPORT.md`
