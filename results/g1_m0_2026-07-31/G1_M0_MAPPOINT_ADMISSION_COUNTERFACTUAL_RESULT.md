# G1-M0 MapPoint 写入 Counterfactual 结果

日期：2026-07-31
状态：完成；G1-M1 具有实际作用，允许进入默认关闭的实现设计
依据：`G1_M0_MAPPOINT_ADMISSION_COUNTERFACTUAL_SPEC.md`

## 1. 结论

G1-M0 证明：

> G1-F1 在 tracking 中清除 q10 MapPoint association 后，ORB-SLAM2 的
> `CreateNewKeyFrame()` 确实会在同一 feature 有有效 RGB-D 深度时重新创建
> MapPoint；此外，大量没有既有 association 的 q10 候选也会直接写入地图。

因此：

```text
tracking filter != mapping protection
```

G1-M1 不是无作用的附加模块，而是完成原始“动态观测不得写入静态 MapPoint”
目标所需的独立步骤。

## 2. 文献与工程依据

- `[L]` ORB-SLAM2 RGB-D 允许单帧深度直接创建 MapPoint；
- `[L]` Ji 2021 强调清理关键帧和局部地图中的动态特征/地图点；
- `[L]` DynaSLAM 的目标同样包括动态观测不参与静态 tracking/mapping；
- `[E]` 当前 DT-SLAM 调用图显示，tracking 清空 association 后，
  `CreateNewKeyFrame()` 会把该位置视为无 MapPoint 并重新创建；
- `[S]` 本阶段只读记录与不变量审计。

具体实现位置来自当前代码，不宣称是论文算法贡献。

## 3. 实现边界

G1-M0 只有在以下条件下才能启用：

```text
RGB-D
G1-F1 enabled
q = 10
mapping counterfactual CSV 非空
```

记录位置：

- `Tracking::StereoInitialization()`；
- `Tracking::CreateNewKeyFrame()`。

没有：

- 设置 `mvbDynamic`；
- 阻止 MapPoint；
- 修改 KeyFrame 内容；
- 修改 LocalMapping、Optimizer、g2o 或 YOLO；
- 新增位姿优化；
- 修改深度区域。

## 4. 150 帧冒烟

```text
mapping events                     77
created MapPoints              14,793
q10 candidate MapPoints             5
tracking removed then recreated      1
candidate-created fraction       0.034%
invariant violations                 0
```

frame 130 首次在线确认：

```text
candidate=1
tracking_removed=1
candidate_created=1
recreated_after_tracking=1
```

## 5. 三序列正式结果

全部使用：

```text
online YOLO CUDA
G1-F1 q10
Viewer OFF
G1-M0 counterfactual only
每条序列三轮
```

9 次正式运行的 `CameraTrajectory` 均覆盖完整输入序列。

### 5.1 三轮中位数

| 序列 | Mapping events | 创建 MapPoint | q10 candidate 创建 | candidate/全部创建 | tracking 删除后重建 |
|---|---:|---:|---:|---:|---:|
| TUM walking | 254 | 26,583 | 653 | 2.411% | 141 |
| TUM sitting_static | 16 | 3,278 | 3 | 0.092% | 0 |
| Bonn balloon | 50 | 12,001 | 202 | 1.714% | 25 |

动态 development 序列中的 candidate admission 明显多于 sitting：

```text
walking / sitting candidate-created rate ≈ 26.3×
balloon / sitting candidate-created rate ≈ 18.7×
```

这是动态富集趋势，不是逐点动态 GT，不能据此计算 precision。

### 5.2 每轮详细计数

| 序列/轮次 | candidate feature | tracking removed | candidate created | recreated |
|---|---:|---:|---:|---:|
| walking 1 | 835 | 136 | 641 | 106 |
| walking 2 | 816 | 160 | 661 | 141 |
| walking 3 | 848 | 187 | 653 | 143 |
| sitting 1 | 3 | 0 | 3 | 0 |
| sitting 2 | 7 | 1 | 7 | 1 |
| sitting 3 | 1 | 0 | 1 | 0 |
| balloon 1 | 238 | 31 | 202 | 28 |
| balloon 2 | 255 | 28 | 209 | 25 |
| balloon 3 | 170 | 20 | 121 | 16 |

`candidate_associations_before_mapping` 几乎为零：

```text
walking: 2 / 1 / 1
sitting: 0 / 0 / 0
balloon: 0 / 0 / 0
```

这说明候选写图主要不是“保留了旧动态 MapPoint”，而是：

1. G1-F1 已清除 association 后被重新创建；
2. 候选本来没有关联，但 RGB-D 深度路径直接创建新 MapPoint。

## 6. 轨迹和速度检查

G1-M0 不改变地图，结果只用于确认 instrumentation 没有明显副作用。

| 序列 | ATE RMSE 中位数 | RPE RMSE 中位数 | FPS 中位数 |
|---|---:|---:|---:|
| walking | 0.016051 m | 0.011831 m/frame | 27.5279 |
| sitting_static | 0.007003 m | 0.005486 m/frame | 27.9398 |
| Bonn balloon | 0.031067 m | 0.040890 m/frame | 29.5946 |

这些数值处于既有 G1-F1 q10 的正常单次波动范围；G1-M0 没有 mapping mutation。

## 7. 不变量审计

10 个 CSV 共：

```text
mapping event rows                  1,038
baseline created MapPoints        140,020
candidate created MapPoints         2,503
tracking-removed then recreated       461
invariant violations                    0
counterfactual_only                  true
direct_mapping_state_mutation        none
mapping_veto                         none
```

工具与结果：

```text
DT-SLAM/tools/audit_sparse_flow_mapping_counterfactual.py
results/g1_m0_2026-07-31/mapping_counterfactual_audit.json
```

## 8. 首帧限制

RGB-D 初始化时没有上一成功帧，因此相邻帧 sparse ego-flow 不可用：

```text
stage=stereo_initialization
candidate_state=reference_unavailable
```

G1-M1 不能伪造初始化几何判断。首帧仍只能依赖语义 mask 和 ORB-SLAM2 原有
初始化条件。未知动态物体若在首帧存在，当前 F1 方法无法提供几何保护；这是
明确限制。

## 9. LocalMapping 范围

当前 `KeyFrame` 已保存 `mvbDynamic`，且
`LocalMapping::CreateNewMapPoints()` 已有两端 dynamic flag 检查。

因此 G1-M1 最小实现可以在 `CreateNewKeyFrame()` 构造 KeyFrame 前，将通过
安全条件的 q10 候选融合到本帧统一 dynamic flag，然后复用：

- 既有 `RemoveDynamicAssociations()`；
- RGB-D depth admission 的 `!mvbDynamic`；
- KeyFrame 对 dynamic flag 的复制；
- 既有 LocalMapping triangulation guard。

不需要新增另一套后端动态状态，也不需要修改 Optimizer。

## 10. 决策

```text
G1-M0 counterfactual                    PASS
candidate MapPoint admission            NON-ZERO, REPEATABLE
tracking-removed recreation             CONFIRMED
G1-M1 implementation                    JUSTIFIED
G1-M1 default                           OFF
initialization geometry protection      UNAVAILABLE
G1-D depth-region filtering             STILL LOCKED
```

下一步 G1-M1 仍需独立 SPEC，至少包含：

1. 只在成功跟踪且准备创建 KeyFrame 时融合 q10 candidate；
2. 复用 G1-F1 的 scale/vector/relocalization/5%/minimum association
   fail-open 状态，不能让 mapping 比 tracking 更激进；
3. 记录实际 veto、剩余 MapPoint 创建量和地图总量；
4. 重新运行 semantic-control、G1-F1、G1-F1+G1-M1 的 ATE/RPE/FPS；
5. 静态退化超过约 10% 或轨迹覆盖下降时停止。

完整逐次指标：

```text
results/g1_m0_2026-07-31/G1_M0_FORMAL_METRICS.csv
```
