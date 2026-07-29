# DT-SLAM TUM semantic baseline 正式轨迹与运行效率对照

日期：2026-07-29

## 1. 目的与边界

本实验开始使用正式轨迹指标，而不是 FPS 或 person proxy，冻结：

```text
ORB-SLAM2 RGB-D baseline
vs
修复后的 synchronous YOLOv8-seg semantic-only baseline
```

几何模块在两组中均关闭：

```yaml
Geometry.Enable: 0
```

因此本报告只评价 Phase 0 semantic cleanup 的轨迹效果，不评价 G2 geometry。

## 2. 数据和公平性

序列：

```text
TUM rgbd_dataset_freiburg3_walking_xyz
```

输入采用此前审计生成的一对一 RGB/depth 关联：

```text
827 RGB-D pairs
max RGB/depth timestamp difference <= 20 ms
no RGB/depth reuse
```

两组使用相同：

- 827 帧；
- association；
- `TUM3.yaml`；
- ORB vocabulary；
- Viewer off；
- 本地代码状态。

唯一实验差异：

```text
baseline:      不传模型参数
semantic-only: 传入 weights/yolov8n-seg.onnx
```

每组独立运行三次，不挑最好结果。

## 3. 评测定义

ATE：

```bash
evo_ape tum groundtruth.txt CameraTrajectory.txt -va --align
```

- translation APE RMSE；
- SE(3) Umeyama alignment；
- scale 固定为 1，不做尺度修正；
- 最大时间匹配差 0.01 s。

RPE：

```bash
evo_rpe tum groundtruth.txt CameraTrajectory.txt \
  -va --align --delta 1 --delta_unit f
```

- translation RPE RMSE；
- 相邻一帧。

evo 版本：

```text
evo 1.36.5
```

当前环境会给出 SciPy/NumPy 版本范围 warning。为排除该 warning 对 ATE 结果的
疑问，另用仅依赖 NumPy 的独立时间关联和无尺度 Umeyama SVD 实现交叉核验；
六次 ATE RMSE 与 evo 输出逐项一致到显示精度。

## 4. 单次结果

| 模式 | run | 写出 poses | ATE 匹配对 | ATE RMSE (m) | RPE 对 | RPE RMSE (m/frame) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | 1 | 827 | 826 | 0.940707 | 825 | 0.025407 |
| baseline | 2 | 827 | 826 | 0.730574 | 825 | 0.024644 |
| baseline | 3 | 814 | 813 | 0.690009 | 812 | 0.042619 |
| semantic-only | 1 | 827 | 826 | 0.016477 | 825 | 0.011858 |
| semantic-only | 2 | 827 | 826 | 0.017752 | 825 | 0.012771 |
| semantic-only | 3 | 827 | 826 | 0.014428 | 825 | 0.011876 |

baseline run 3 少写出 13 个 pose，说明发生跟踪不完整。它的 ATE 较低不能简单
解释为更好，因为缺失区间可能降低整体误差统计；报告仍保留该次结果。

semantic-only 三次均写出完整 827 poses。

## 5. 三次重复汇总

| 指标 | baseline | semantic-only |
| --- | ---: | ---: |
| ATE RMSE mean | 0.787097 m | 0.016219 m |
| ATE RMSE median | 0.730574 m | 0.016477 m |
| ATE RMSE population std | 0.109874 m | 0.001369 m |
| RPE RMSE mean | 0.030890 m/frame | 0.012168 m/frame |
| RPE RMSE median | 0.025407 m/frame | 0.011876 m/frame |
| RPE RMSE population std | 0.008300 | 0.000426 |
| 完整轨迹次数 | 2/3 | 3/3 |

按三次中位数：

```text
ATE RMSE: 0.730574 m -> 0.016477 m
相对下降: 97.74%
误差倍率: 44.34x

RPE RMSE: 0.025407 m/frame -> 0.011876 m/frame
相对下降: 53.26%
误差倍率: 2.14x
```

## 6. 运行完整性

semantic-only 三次均为：

```text
mask ready = 827/827
mask age median/max = 0/0
```

实际运行速度：

```text
baseline:      28.46 / 28.54 / 28.55 FPS
semantic-only: 28.34 / 28.38 / 28.12 FPS
```

本节只用于确认实验完整执行。轨迹结论来自 ATE/RPE，不使用 FPS 排名。

## 7. 客观结论

可以确认：

1. Phase 0 semantic cleanup 已经形成有效 semantic baseline；
2. 在 `fr3_walking_xyz` 上，semantic-only 显著改善 ATE 和 RPE；
3. semantic-only 的三次离散度显著小于原始 baseline；
4. semantic-only 没有出现轨迹缺帧；
5. 之前 person proxy 的用途仅是几何 Shadow 诊断，不再代替系统轨迹评价。

不能据此确认：

1. geometry 模块提高了 ATE——geometry 尚未过滤任何观测；
2. semantic-only 在所有低动态或静态序列上都无退化；
3. 当前系统已经稳定达到 30 FPS；
4. ATE 改善能够证明完整动态深度区域已经恢复。

## 8. 决策

```text
walking semantic trajectory baseline = 冻结
geometry ATE baseline = 尚不存在
```

下一步扩展到低动态 `fr3_sitting_static`，检查 semantic-only 是否错误删除大量
静止人物特征并造成轨迹退化。通过后，再将 baseline/semantic-only 作为未来
G1-F geometry ATE 的正式对照。

geometry 第一次有意义的 ATE 测量点保持不变：

```text
G2-3R2 dense-vs-grid coverage upper-bound audit
→ 保守 high-confidence feature evidence
→ G1-F feature filtering
→ semantic-only vs semantic+geometry ATE/RPE
```

## 9. 低动态序列扩展：fr3_sitting_static

为检查语义先验在低动态人物场景中是否造成明显退化，使用：

```text
TUM rgbd_dataset_freiburg3_sitting_static
680 RGB-D pairs
严格一对一 RGB/depth association
最大 RGB/depth 时间差 <= 20 ms
```

实验控制与 `fr3_walking_xyz` 相同：

- baseline 与 semantic-only 各独立运行三次；
- 两组均使用 `TUM3.yaml`，关闭 Geometry 和 Viewer；
- semantic-only 使用 CUDAExecutionProvider；
- 不挑最好结果。

注意：TUM 序列名中的 `static` 描述相机运动方式，不表示画面中完全没有人物
运动。因此本序列是低动态检查，不是真正静态负样本。

## 10. sitting_static 单次轨迹结果

| 模式 | run | 写出 poses | ATE 匹配对 | ATE RMSE (m) | translation RPE RMSE (m/frame) | rotation RPE RMSE (deg/frame) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | 1 | 680 | 676 | 0.008097 | 0.005198 | 0.161712 |
| baseline | 2 | 680 | 676 | 0.008022 | 0.005335 | 0.163444 |
| baseline | 3 | 680 | 676 | 0.007812 | 0.005530 | 0.167154 |
| semantic-only | 1 | 680 | 676 | 0.006425 | 0.005382 | 0.164194 |
| semantic-only | 2 | 680 | 676 | 0.006794 | 0.005512 | 0.165418 |
| semantic-only | 3 | 680 | 676 | 0.006482 | 0.005492 | 0.166151 |

每次轨迹都有 680 poses。ATE 只匹配 676 对，是因为 evo 默认最大时间匹配差为
0.01 s；baseline 与 semantic-only 使用完全相同的输入和匹配口径。

## 11. sitting_static 三次重复汇总

| 指标 | baseline | semantic-only | semantic 相对变化 |
| --- | ---: | ---: | ---: |
| ATE RMSE mean | 0.007977 m | 0.006567 m | -17.68% |
| ATE RMSE median | 0.008022 m | 0.006482 m | **-19.20%** |
| ATE RMSE population std | 0.000121 m | 0.000162 m | — |
| translation RPE RMSE mean | 0.005354 m/frame | 0.005462 m/frame | +2.01% |
| translation RPE RMSE median | 0.005335 m/frame | 0.005492 m/frame | **+2.94%** |
| rotation RPE RMSE mean | 0.164103 deg/frame | 0.165254 deg/frame | +0.70% |
| rotation RPE RMSE median | 0.163444 deg/frame | 0.165418 deg/frame | **+1.21%** |
| 完整轨迹次数 | 3/3 | 3/3 | 相同 |

负号表示误差下降，正号表示误差上升。客观结果是：

```text
ATE 明显改善；
逐帧 translation/rotation RPE 略微恶化；
没有跟踪缺帧。
```

只有三次随机重复，不能把约 1%–3% 的 RPE 差异解释成具有统计显著性的性能
退化；但也不能写成“所有轨迹指标都改善”。

## 12. sitting_static FPS、时效和分段耗时

| 指标 | baseline 三次 | semantic-only 三次 |
| --- | --- | --- |
| actual FPS | 28.5147 / 28.5124 / 28.5073 | 28.5057 / 28.5282 / 28.5243 |
| actual FPS median | 28.5124 | 28.5243 |
| 完整输出 | 680/680，三次 | 680/680，三次 |
| deadline missed | 0 / 0 / 0 | 29 / 14 / 32 |
| tracking mean | 10.739 / 10.603 / 10.619 ms | 9.827 / 9.677 / 9.720 ms |
| semantic total median | — | 12.412 / 12.354 / 12.405 ms |
| semantic steady block median | — | 12.690 / 12.629 / 12.673 ms |
| mask ready | — | 680/680，三次 |
| mask age median/max | — | 0/0，三次 |

actual FPS 接近数据集本身的 `28.6158 FPS`，两组中位数差仅约 `0.04%`，不能
解释为 semantic 模式更快。semantic 的 deadline miss 表明同步语义仍消耗了
帧间预算；当前播放节流使端到端 FPS 没有按 active compute time 线性下降。

语义模式 tracking 均值低于 baseline 是过滤部分人物特征后的实测结果，但它与
GPU 推理耗时属于不同阶段，不能抵消约 12.4 ms 的同步语义阻塞。

## 13. sitting_static 交叉校验

使用独立的：

```text
timestamp one-to-one association
+ no-scale Umeyama SVD alignment
+ translation RMSE
```

重新计算六条轨迹，得到：

```text
baseline:
0.0080970627 / 0.0080220214 / 0.0078122895 m

semantic-only:
0.0064245142 / 0.0067938361 / 0.0064817707 m
```

与 evo 输出逐项一致到报告精度。evo 的 SciPy/NumPy warning 未改变本组 ATE
数值。

## 14. 两序列联合结论与下一测量点

当前正式证据支持：

1. `fr3_walking_xyz`：semantic-only 对 ATE、RPE 和轨迹完整性都有大幅收益；
2. `fr3_sitting_static`：semantic-only 的 ATE 改善，逐帧 RPE 小幅恶化，
   轨迹完整性不变；
3. 两个序列中 semantic mask 都逐帧同步可用，未复用旧 mask；
4. FPS、deadline miss、分段耗时和轨迹指标必须同时报告，不能互相替代；
5. person proxy 仍只是无人工 GT 时的局部几何证据诊断，不是 SLAM 最终指标。

冻结状态：

```text
Phase 0 synchronous semantic baseline = 两序列正式对照完成
walking semantic benefit = 明确通过
sitting_static no-major-regression gate = 通过
30 FPS gate = 未严格通过
geometry trajectory benefit = 尚未测量
```

下一次正式轨迹对照应发生在 G1-F 真的改变 ORB 动态观测之后：

```text
baseline
vs semantic-only
vs geometry-only
vs semantic+geometry

指标：
ATE translation RMSE
RPE translation RMSE
RPE rotation RMSE
trajectory completeness
actual FPS / deadline miss
geometry and semantic per-stage latency
dynamic-feature removal/support statistics
```
