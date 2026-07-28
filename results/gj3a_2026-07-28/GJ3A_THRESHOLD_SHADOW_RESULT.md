# GJ-3A Relative-Threshold Shadow 审计结果

日期：2026-07-28

## 1. 结论

本轮相对阈值适配没有通过实际过滤门控。

两种normalization都在calibration选择了网格中的最小值：

```text
lambda = 0.5
```

它能提高person proxy F1，但代价是在真正静态的TUM fr1/xyz中选择约一半可测深度
区域：

| normalization | calibration F1 | validation F1 | validation recall | fr1/xyz selected area |
| --- | ---: | ---: | ---: | ---: |
| support-weighted frame mean | 0.640 | 0.516 | 79.59% | 50.49% |
| cluster-unweighted frame mean | 0.640 | 0.508 | 75.08% | 53.57% |

同一fr3相机的`sitting_static`事后诊断也选择约50%–52%的区域，说明该结果不能只
归因于fr1/fr3相机或场景差异。

因此：

```text
GJ-3A threshold-family shadow：完成但失败
GJ实际动态过滤：不进入
```

该结论只否定当前identity-`rho`、平均误差倍数阈值的工程适配，不等于证明Ji作者
未公开的原始实现无效。

## 2. 固定实验协议

主score：

```text
mean_squared_error_px2
```

它对应当前声明的：

```text
rho(s) = s
```

两种候选normalization：

```text
mu_feature =
  Σ_j(support_j × r_j) / Σ_j support_j

mu_cluster =
  mean_j(r_j)
```

判定候选：

```text
r_j > lambda × mu
```

lambda网格在validation前固定：

```text
2^(k/2), k=-2..6
```

数据拆分：

```text
calibration:       walking_xyz frame 1..99
dynamic validation: walking_xyz frame 100..199
static validation:  fr1/xyz frame 1..199
reserved:           walking_xyz frame >=200
```

calibration规则：

```text
最大person-proxy像素F1
F1相同时选择更大lambda
```

该规则是自动工程审计，不是Ji论文选参流程。

## 3. support-weighted结果

这是最接近论文“average reprojection error of matched features”文字的解释。

| lambda | cal P | cal R | cal F1 | val P | val R | val F1 | val area | static area |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.500 | 56.38% | 73.94% | 0.640 | 38.18% | 79.59% | 0.516 | 59.31% | 50.49% |
| 0.707 | 61.01% | 65.31% | 0.631 | 38.30% | 64.83% | 0.482 | 48.16% | 41.07% |
| 1.000 | 66.19% | 56.63% | 0.610 | 39.99% | 50.53% | 0.447 | 35.95% | 32.40% |
| 1.414 | 69.18% | 41.87% | 0.522 | 40.77% | 34.50% | 0.374 | 24.07% | 24.15% |
| 2.000 | 73.01% | 27.82% | 0.403 | 38.08% | 18.82% | 0.252 | 14.06% | 15.30% |
| 2.828 | 70.95% | 14.72% | 0.244 | 32.94% | 7.79% | 0.126 | 6.73% | 8.11% |
| 4.000 | 73.17% | 7.62% | 0.138 | 21.67% | 2.46% | 0.044 | 3.23% | 4.19% |
| 5.657 | 73.94% | 3.75% | 0.071 | 27.63% | 1.92% | 0.036 | 1.97% | 1.28% |
| 8.000 | 64.00% | 1.43% | 0.028 | 27.56% | 1.28% | 0.025 | 1.33% | 0.40% |

随着lambda提高，静态选择面积下降，但dynamic validation recall同时快速坍塌。
当前网格中没有观察到兼顾静态保留和动态代理召回的明显工作点。

## 4. cluster-unweighted结果

| lambda | cal F1 | val F1 | val recall | val area | static area |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.500 | 0.640 | 0.508 | 75.08% | 55.67% | 53.57% |
| 0.707 | 0.616 | 0.475 | 60.77% | 44.31% | 43.49% |
| 1.000 | 0.598 | 0.419 | 46.00% | 34.02% | 35.27% |
| 1.414 | 0.525 | 0.360 | 32.38% | 22.72% | 27.40% |
| 2.000 | 0.429 | 0.262 | 19.71% | 14.33% | 18.48% |
| 2.828 | 0.265 | 0.139 | 8.69% | 7.07% | 10.72% |
| 4.000 | 0.172 | 0.079 | 4.46% | 3.82% | 5.53% |
| 5.657 | 0.064 | 0.031 | 1.62% | 1.57% | 2.14% |
| 8.000 | 0.028 | 0.018 | 0.92% | 0.54% | 0.51% |

结果方向与support-weighted一致。更换两种平均方式没有解决静态高尾问题。

## 5. “最大person F1”为什么不够

calibration中person proxy占全部可测cluster面积的`38.74%`；validation中占
`28.45%`。如果选择全部cluster，对应F1分别为：

```text
calibration: 0.558
validation:  0.443
```

`lambda=0.5`将其提升到约`0.640/0.516`，说明相对误差确实对person proxy有
enrichment；但该优化目标不会惩罚另一条真正静态序列中的删除。

这次结果证明：

```text
person proxy F1提高
≠
得到可用于SLAM的动态过滤器
```

若为了压低静态面积再人为加入5%、10%或15%上限，只是在论文未公开阈值之外再补
一个新的主观阈值。本轮不这样做。

## 6. 同相机低动态诊断

在看到主实验失败后，追加了TUM `fr3/sitting_static`前200帧诊断。它与walking
使用同一fr3相机，但画面仍有坐着的人和轻微运动，因此不是严格静态GT，也不参与
lambda选择。

在calibration选出的`lambda=0.5`下：

| normalization | sitting selected area | selected clusters |
| --- | ---: | ---: |
| support-weighted | 50.32% | 45.61% |
| cluster-unweighted | 51.72% | 47.34% |

这与fr1/xyz的约50%–54%接近，说明“相对frame mean的低阈值会持续选择误差高尾”
是跨两个对照均存在的结构现象。

## 7. 结构性解释

每一帧都按自身平均误差归一化时，即使场景静态，也必然存在高于均值的cluster。
较低lambda会在所有帧中选择大块高尾区域；提高lambda虽然减少静态选择，也会同步
删除dynamic validation中的召回。

当前结果不能区分这些高误差来源：

- 真实对象运动；
- optimizer outlier；
- 错误地图匹配；
- 位姿误差；
- cluster混合多个表面；
- 少量高残差支持点；
- 遮挡和图像边缘。

这与GJ-2中静态序列仍有长误差尾部的观察一致。

## 8. 性能与工程边界

本轮三段200帧运行：

| 序列 | tracking mean | active total mean | actual FPS |
| --- | ---: | ---: | ---: |
| walking_xyz | 66.44 ms | 75.32 ms | 13.26 |
| fr1/xyz | 74.48 ms | 83.15 ms | 12.03 |
| sitting_static | 67.94 ms | 77.09 ms | 12.97 |

这些运行包含CPU K-means和每帧raw label写入。离线threshold扫描开销很小，但
不会改变GJ-1无法满足30 Hz的结论。

本阶段没有修改：

```text
YOLO
Optimizer
g2o
Frame::mvbDynamic
Frame::mvpMapPoints
MapPoint写入
LocalMapping
LoopClosing
```

## 9. 输出

```text
results/gj3a_2026-07-28/GJ3A_THRESHOLD_SHADOW_SPEC.md
results/gj3a_2026-07-28/threshold_audit/threshold_grid.csv
results/gj3a_2026-07-28/threshold_audit/summary.json
results/gj3a_2026-07-28/threshold_audit_sitting_as_low_dynamic/
results/gj3a_2026-07-28/walking200_cluster_audit/
results/gj3a_2026-07-28/fr1_xyz200_cluster_audit/
results/gj3a_2026-07-28/sitting200_cluster_audit/
```

## 10. 决策

批准：

- 将GJ-3A记录为完整的负实验；
- 保留GJ-1/GJ-2/GJ-2A作为Ji测量与排序baseline；
- 报告公开材料不足以复现Ji作者二值判定；
- 停止继续为Ji baseline叠加经验阈值。

不批准：

- `lambda=0.5`进入SLAM；
- 为了得到可用结果追加静态面积上限；
- 固定Top-K删除；
- 用当前proxy F1声称未知动态检测完成；
- 修改Optimizer或增加第三次优化。

下一研究步骤不应继续拟合Ji阈值，而应回到主方法G2，先重新定义能明确区分
motion evidence、unknown和静态置信度的鲁棒shadow测量，再单独审批。
