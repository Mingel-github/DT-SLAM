# G1-M1 MapPoint 写入过滤结果

日期：2026-07-31
状态：完成；允许作为默认关闭的实验模式保留
依据：`G1_M1_MAPPOINT_ADMISSION_FILTER_SPEC.md`

## 1. 结论

G1-M1 已把 G1-F1 q10 的高置信 sparse ego-flow 候选接入真实 MapPoint
写入保护：

```text
通过 G1-F1 与 mapping 安全条件
→ candidate 并入 Frame::mvbDynamic
→ 既有 association 清理
→ 既有 RGB-D depth admission 跳过
→ KeyFrame 保存 dynamic flag
→ 既有 LocalMapping dynamic endpoint guard 生效
```

四类序列的 12 次正式运行均保持完整轨迹，ATE/RPE 相对对应 control 没有超过
约 3% 的中位数退化，FPS 基本不变，全部 CSV 不变量通过。

因此：

> G1-F1 + G1-M1 可以冻结为“稀疏几何前端实验版本”，但仍默认关闭。

这只证明接入安全、候选确实不再写入 MapPoint。由于没有逐点地图动态 GT，
不能把被否决候选全部解释成真实动态，也不能声称稠密地图已经去除动态残影。

## 2. 文献与工程身份

- `[L]` ORB-SLAM2 RGB-D 的 MapPoint 初始化和新关键帧 depth admission；
- `[L]` Ji 2021 与 DynaSLAM 的动态观测不进入静态 tracking/mapping 原则；
- `[E]` 当前 `Frame`、`KeyFrame`、`LocalMapping` 已有统一 `mvbDynamic` 链路；
- `[S]` q10 工作点、5% feature/depth 上限、至少 100 个剩余有效深度以及
  fail-open 逻辑。

G1-M1 不是新的动态检测算法。它是把已经审计过的 G1-F1 候选安全接入当前
ORB-SLAM2 fork 的 MapPoint admission。

没有修改：

- `LocalMapping.cc`；
- `Optimizer.cc`；
- g2o；
- YOLO；
- 位姿优化次数；
- 稠密深度或点云写入。

## 3. 实现结果

新增默认关闭配置：

```yaml
Geometry.SparseFlowMappingFilterEnable: 0
Geometry.SparseFlowMappingFilterMaximumFeatureFraction: 0.05
Geometry.SparseFlowMappingFilterMaximumDepthFraction: 0.05
Geometry.SparseFlowMappingFilterMinimumRemainingDepthFeatures: 100
```

运行时开关：

```text
DT_SLAM_GEOMETRY_MAPPING_FILTER
DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV
```

约束：

```text
RGB-D
G1-F1 enabled
q = 10
G1-M0 counterfactual disabled
```

## 4. 150 帧冒烟

```text
轨迹覆盖                         149 / 149
mapping events                   51
applied events                   14
new dynamic flags               129
vetoed valid-depth features      124
candidate MapPoint in applied      0
fail-open events                   9
invariant violations               0
actual FPS                     27.74
```

其中一帧 candidate depth 比例为约 7.78%，超过 5% 上限，正确进入
`maximum_depth_fraction_fail_open`，没有修改地图。

## 5. 正式 ATE/RPE/FPS

全部结果均是三轮中位数。walking、sitting 与 Bonn 使用在线 CUDA YOLO，
mask age 均为 0；fr1/xyz 不启用语义，用于真正静态的几何安全检查。

| 序列 | G1-M1 ATE RMSE | 相对 control | G1-M1 RPE RMSE | 相对 control | FPS | 覆盖 |
|---|---:|---:|---:|---:|---:|---:|
| TUM walking | 0.017864 m | +0.93% | 0.012343 m/frame | +0.97% | 27.479 | 827/827 |
| TUM sitting_static | 0.006596 m | -2.64% | 0.005669 m/frame | +1.14% | 27.890 | 680/680 |
| Bonn balloon | 0.032519 m | -0.26% | 0.041264 m/frame | +0.88% | 29.538 | 438/438 |
| TUM fr1/xyz | 0.009643 m | -0.66% | 0.005777 m/frame | +0.35% | 29.624 | 792/792 |

对应 control：

| 序列 | control ATE | control RPE | control FPS |
|---|---:|---:|---:|
| walking semantic | 0.017699 m | 0.012224 | 27.484 |
| sitting semantic | 0.006775 m | 0.005605 | 27.931 |
| Bonn semantic | 0.032605 m | 0.040905 | 29.536 |
| fr1/xyz geometry off | 0.009707 m | 0.005757 | 29.643 |

相对只启用 G1-F1 q10：

- walking ATE 中位数从 `0.015462` 回到 `0.017864`，但仍与 control 基本相同；
- sitting、Bonn、fr1/xyz 的变化均较小；
- 因此没有证据说明 G1-M1 能改善定位，也没有证据显示它造成系统性明显退化。

G1-M1 的直接价值是 MapPoint 写入保护，不应借 ATE 的随机改善包装成贡献。

## 6. 写图统计

三轮中位数：

| 序列 | applied KeyFrame events | new dynamic flags | vetoed valid-depth |
|---|---:|---:|---:|
| walking | 88 | 601 | 564 |
| sitting_static | 1 | 1 | 1 |
| Bonn balloon | 25 | 199 | 192 |
| fr1/xyz | 21 | 55 | 43 |

12 次正式运行合计：

```text
mapping rows                         1,200
initialization rows                     12
applied rows                           399
fail-open rows                         103
new dynamic flags                    2,619
vetoed valid-depth features          2,441
candidate-created in fail-open rows    834
invariant violations                     0
```

`candidate-created in fail-open` 不是实现错误：它表示保护条件主动拒绝修改后，
ORB-SLAM2 原行为仍创建了候选 MapPoint。所有 `applied=true` 行的
`candidate_created_mappoints` 均为 0。

## 7. Fail-open 分布

12 次正式运行：

```text
applied                              399
no_candidates                       698
tracking_safeguard_fail_open         61
maximum_depth_fraction_fail_open     27
maximum_feature_fraction_fail_open    3
reference_unavailable_fail_open      12
```

这说明写图过滤不是无条件把 q10 候选全部删除。首帧、重定位窗口、关联安全条件
失败以及候选比例过高时都会保留 ORB-SLAM2 原行为。

## 8. Viewer 检查

完成了 300 帧 G1-M1 Viewer 定性运行：

- 299/299 图像完成；
- 轨迹和 G1-M1 CSV 均成功保存；
- 程序在保存完成后的 Viewer 关闭阶段返回 exit 139。

随后用相同环境运行 100 帧语义基线、关闭 G1-F1/G1-M1，仍在轨迹保存完成后
返回 exit 139。因此本次对照表明该退出问题不依赖 G1-M1；本阶段没有修改
Viewer/Pangolin。正式 Viewer OFF 的 12 次运行均正常退出。

## 9. 验证

```text
make geometric_warp_test rgbd_tum -j$(nproc)     PASS
geometric_warp_test                              PASS
git diff --check                                 PASS
Python audit compile                             PASS
G1-F1 tracking CSV audit                         PASS
G1-M1 mapping CSV audit                          PASS
formal trajectory coverage                       PASS
```

审计工具：

```text
DT-SLAM/tools/audit_sparse_flow_mapping_filter.py
```

原始汇总：

```text
results/g1_m1_2026-07-31/G1_M1_FORMAL_METRICS.csv
results/g1_m1_2026-07-31/mapping_filter_audit.json
results/g1_m1_2026-07-31/tracking_filter_audit.json
```

## 10. 决策

```text
G1-M1 implementation                    PASS
default                                 OFF
tracking + MapPoint sparse protection   EXPERIMENTALLY USABLE
ATE improvement claim                   NOT SUPPORTED
mapping-cleanliness claim               NOT YET MEASURED WITH GT
first-frame geometry protection         UNAVAILABLE
G1-D pixel/depth-region filtering       STILL LOCKED
```

下一步不应继续调 G1-M1 的 5% 参数来追求更好 ATE。应先冻结当前稀疏版本和
可复现运行方式，然后单独决定：

1. 先做地图质量的定性/可量化对照；
2. 或回到尚未解决的 `G1-D`，研究可信动态深度区域，而不把 sparse feature
   flag 直接膨胀为完整像素 mask。
