# G1 稀疏几何前端四模式首轮结果

日期：2026-07-31
状态：首轮完成；组合模式通过，纯几何模式不具备独立使用资格

## 1. 运行条件

```text
commit baseline                     be5c272
sequence                            TUM fr3_walking_xyz
associations                        827 one-to-one RGB-D pairs, <=20 ms
viewer                              OFF
semantic provider                   CUDAExecutionProvider
geometry parameters                 frozen q10 / 5%, no retuning
Optimizer/g2o/YOLO/LocalMapping     unchanged
```

四种模式均由 `DT-SLAM/tools/run_sparse_frontend_mode.py` 调用同一
`rgbd_tum` 二进制。TUM3 为零畸变，因此 baseline 和 geometry 的像素域一致。

## 2. 首轮四模式

| 模式 | 轨迹 | ATE RMSE | RPE RMSE | FPS |
|---|---:|---:|---:|---:|
| ORB baseline | 816/827 | 0.926133 m | 0.078290 | 28.423 |
| semantic-only | 827/827 | 0.019142 m | 0.011836 | 28.261 |
| geometry-only | 587/827 | 0.533014 m | 0.096267 | 28.506 |
| semantic+geometry | 827/827 | 0.018693 m | 0.012298 | 27.083 |

semantic-only 和 semantic+geometry 的在线 mask 均为：

```text
ready 827/827
age median=0
age max=0
```

## 3. 几何是否真正生效

首轮 `geometry-only` 完全不提供 YOLO 模型，但仍产生：

```text
tracking applied frames        47
tracking removed associations 240
mapping applied rows           54
mapping vetoed valid-depth    203
```

`semantic+geometry` 产生：

```text
tracking applied frames       182
tracking removed associations 537
mapping applied rows           94
mapping vetoed valid-depth    634
```

两种模式的 tracking/mapping CSV invariant 均通过。这证明类别无关几何路径
确实独立运行，也证明在组合模式中它不是空开关。

## 4. geometry-only 重复检查

由于首轮 geometry-only 只有 587 个轨迹 pose，额外重复两次且不修改参数：

| 模式 | trial | 轨迹 | ATE | RPE | FPS |
|---|---:|---:|---:|---:|---:|
| ORB baseline | 1 | 816 | 0.926133 | 0.078290 | 28.423 |
| ORB baseline | 2 | 827 | 0.855539 | 0.024501 | 28.510 |
| ORB baseline | 3 | 685 | 0.698717 | 0.061885 | 28.519 |
| geometry-only | 1 | 587 | 0.533014 | 0.096267 | 28.506 |
| geometry-only | 2 | 827 | 0.819301 | 0.026502 | 28.309 |
| geometry-only | 3 | 827 | 0.855549 | 0.026420 | 28.420 |

裸 ORB 和 geometry-only 都表现出严重误差及覆盖波动。geometry-only 并没有
稳定替代语义分支；它有时改善裸 ORB、有时相近或更差。由于 q10 只删除极少量
高置信关联，而 walking 中有大面积已知动态人物，这一结果不应通过继续调阈值
修补。

## 5. semantic+geometry 判断

相对同轮 semantic-only：

```text
ATE      -2.35%   （单轮，不作稳定改善声明）
RPE      +3.90%
FPS      -4.17%
coverage identical at 827/827
```

结合此前三轮 G1-M1 与 semantic control 结果，目前只能判断：

- 组合模式完整、稳定，额外开销有限；
- 几何关联和 MapPoint 保护确实生效；
- 没有证据证明它稳定改善 ATE/RPE；
- 也没有观察到灾难性退化。

## 6. 决策

```text
semantic+geometry sparse frontend   PASS FOR EXPERIMENTS, default OFF
semantic-only frozen baseline       PASS
geometry-only standalone system     FAIL / DIAGNOSTIC ONLY
geometry path without YOLO          FUNCTIONALLY CONFIRMED
ATE improvement claim               NOT SUPPORTED
unknown-object detection claim      NOT YET VALIDATED WITH GT
G1-D depth-region filtering         LOCKED
```

原 SPEC 中“geometry-only 必须完整输出轨迹”的条件未通过，因此不能把四模式整体
标为全部通过。项目的目标仍是 semantic 处理已知动态、geometry 补充未知动态；
本结果支持继续使用组合模式做实验，但不支持宣传 geometry 可以独立处理 walking
这类强人体动态场景。

## 7. 下一步

1. 冻结 `semantic+geometry` 的可复现命令，不再调整 q10/5%；
2. 做一次 Viewer ON 定性检查，Viewer 退出 139 继续按既有独立问题记录；
3. 不立即扩展所有四模式到 TUM1/Bonn；先明确其 rectified control；
4. 若继续追求未知箱子定量结论，需要新的可观察箱子序列或真值，不能用 walking
   人体结果代替；
5. G1-D 仍作为独立研究问题，不从 sparse feature flag 直接膨胀 mask。

逐次指标：

```text
results/g1_release_2026-07-31/G1_WALKING_FOUR_MODE_METRICS.csv
results/g1_release_2026-07-31/walking/
```

## 8. Viewer ON 检查

semantic+geometry 使用 Viewer ON 完整处理 827/827 帧：

```text
mask ready                 827/827
mask age median/max        0/0
actual FPS                 26.587
tracking CSV rows          826
mapping CSV rows           262
trajectory save reached    yes
process return             -11 (shell 245)
```

程序在打印 `Saving camera trajectory` 后进入既有 Viewer/Pangolin 关闭崩溃，与
此前 G1-M1 Viewer 和关闭几何过滤的 semantic control 表现相同。因此该退出问题
继续作为独立维护债，不计为本次几何算法运行失败，也不在本阶段修改。
