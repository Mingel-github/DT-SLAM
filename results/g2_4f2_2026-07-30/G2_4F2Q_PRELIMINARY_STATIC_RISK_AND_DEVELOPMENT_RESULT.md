# G2-4F2Q 初步静态风险与 Development Sensitivity 结果

日期：2026-07-30  
状态：TUM 静态域初审完成；Bonn 静态域待补；未选择 operating point

## 1. 本轮做了什么

按冻结 SPEC 新增离线工具：

```text
DT-SLAM/tools/audit_sparse_flow_feature_gate.py
```

并在既有 Bonn F1 审计工具中增加默认不启用的：

```text
--feature-gate-curves
```

两者只读取已有 G2-4F1 CSV：

```text
FB error                → correspondence quality
ego-flow residual       → motion inconsistency
1.4826 * median(r)      → per-frame zero-centered robust scale
r / scale               → normalized residual q
```

没有输出动态标签、没有修改 SLAM 状态。

## 2. TUM1 共同坐标域

新增：

```text
Examples/RGB-D/TUM1_GeometrySparseEgoFlowShadow.yaml
```

TUM1 有非零畸变。该配置将 RGB 和 registered depth 一起 remap 到
`P=K` 的无畸变针孔域，并把 Tracking distortion 设为零。因此：

```text
ORB / LK / depth / geometry projection / mask
```

位于同一像素域。

`fr1/xyz` 前 149 个 association 的短测结果：

```text
GT interpolation available     = 149/149
F1 feature rows                = 148,946
F1 frame rows                  = 148
dynamic_decision               = none
direct_slam_state_mutation     = none
actual FPS with diagnostic CSV = 28.657
```

这里的 FPS 包含联合 rectification 和 14.9 万行 feature CSV 诊断写出，不能
替代未来关闭重诊断后的 production FPS。

## 3. 原始静态分布

`fr1/xyz` 是场景静态、相机运动的负样本。非语义 measured feature：

| 指标 | median | p95 | p99 |
| --- | ---: | ---: | ---: |
| FB error | 0.0081 px | 0.1411 px | 10.5853 px |
| ego-flow residual | 0.3776 px | 1.4885 px | 10.7262 px |

长尾很明显。它说明：

- 大多数 LK correspondence 很稳定；
- 少量 catastrophic LK/遮挡/边界/位姿误差会产生很大值；
- 只检查 OpenCV `status=1` 不足以建立可靠 evidence；
- FB quality gate 有必要，但 FB 大不等于对象动态。

## 4. 静态 candidate-rate 曲线

以下只展示 `FB <= 0.25 px` 的代表点。该 FB 值仍只是冻结诊断网格之一，
不是最终阈值。

| q 阈值 | quality coverage | 静态候选率 | MapPoint 候选率 | 每帧候选 median / p95 | 每帧保留 quality feature median |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 80.804% | 0.768% | 0.155% | 3 / 22.3 | 819.0 |
| 6 | 80.804% | 0.355% | 0.072% | 1 / 13.0 | 823.5 |
| 8 | 80.804% | 0.253% | 0.067% | 0 / 12.0 | 824.0 |
| 10 | 80.804% | 0.229% | 0.067% | 0 / 12.0 | 824.0 |

这里必须称为：

```text
static-sequence candidate rate
```

而不是严格 pixel-level false-positive rate。静态序列中的少量候选也可能是：

- 错误 LK；
- 遮挡/显露；
- depth boundary 取样；
- RGB/depth timestamp 误差；
- 初始位姿局部误差；
- 对跟踪本来就有害的普通 outlier。

## 5. Balloon development sensitivity

评价对象仍是冻结的：

```text
moving_observable
+ exact zero person-mask pixels inside coarse bbox
```

共 7 帧，其中 6 帧存在 quality-eligible 框内 ORB。粗框和 motion label 均为
Agent RGB-only development proxy，不是 GT。

同样取 `FB <= 0.25 px`：

| q 阈值 | 有框内 evidence 的帧 | 有框内 candidate 的帧 | 框内候选率中位 | 同帧框外候选率中位 | 框内减框外中位 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 6 | 6 | 82.237% | 3.226% | 74.536% |
| 6 | 6 | 6 | 79.605% | 0.845% | 73.853% |
| 8 | 6 | 6 | 74.342% | 0.472% | 69.959% |
| 10 | 6 | 6 | 74.342% | 0.000% | 73.287% |

这说明 G2-4F1D 的方向性信号不是由明显的 forward-backward tracking failure
搬运；在严格 FB quality subset 中仍然存在。

这是本轮最积极的结果，但仍不能称为通用动态分类：

- 只有 6 个可测 moving proxy frame；
- 框不是像素 GT；
- 未含静止气球同对象负样本；
- TUM 与 Bonn 的相机域不同；
- 每帧尺度假设静态 feature 占多数；
- 还没有审计 depth boundary 对候选的贡献；
- strict holdout 仍未打开。

## 6. 当前决策

```text
F2Q implementation/test gate            = PASS
TUM-domain preliminary static risk      = PASS
balloon development directional signal  = PASS
Bonn-domain static risk                 = NOT EVALUABLE YET
F2D operating point                     = NOT SELECTED
strict holdout                          = SEALED
G1-F / G1-D                             = LOCKED
```

当前不选择 `FB=0.25, q=6/8/10` 中任何一个。它们只是 Pareto 候选。

## 7. 下一步

1. 获取官方 `rgbd_bonn_static_close_far`；
2. 使用现有 Bonn 联合 rectification 和同一 G2-4F1 配置跑短静态审计；
3. 检查 Bonn 静态 candidate-rate 是否与 TUM 同量级；
4. 若跨域成立，再增加 reference-depth boundary 分层；
5. 全部通过后才冻结 F2D operating point 和 holdout protocol。

若 Bonn 静态候选率明显升高，先检查同步、位姿、标定和边界风险；不得通过
查看 strict holdout 或调整组合阈值掩盖失败。

## 8. 原始证据

```text
results/g2_4f2_2026-07-30/static_fr1_xyz_150/
results/g2_4f2_2026-07-30/preliminary_quality_static_risk_curves.json
results/g2_4f2_2026-07-30/balloon_development_curves/
```

