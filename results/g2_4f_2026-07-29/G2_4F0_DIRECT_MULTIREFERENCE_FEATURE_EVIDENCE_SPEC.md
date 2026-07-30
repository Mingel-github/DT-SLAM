# G2-4F0 Direct Multi-reference ORB Feature Evidence Shadow SPEC

日期：2026-07-29
状态：冻结后实施
范围：只读 instrumentation 和 development audit。

## 1. 目标

移除失败的 depth-region aggregation 后，直接在当前帧 ORB keypoint 中心读取
已有 multi-reference evidence：

```text
comparison_count
positive_count
negative_count
consistent_count
```

输出逐 feature 诊断，用独立 coarse bbox 和实际 C++ person mask 做离线审计。

## 2. 明确非目标

- 不创建 dynamic/static feature 判决；
- 不选择 count、ratio、support 或 residual threshold；
- 不膨胀 positive mask；
- 不修改 `Frame::mvbDynamic`；
- 不清空 `mvpMapPoints`；
- 不阻止 MapPoint 写入；
- 不修改 YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing；
- 不增加 PoseOptimization；
- 不运行 strict hold-out；
- 不把 coarse bbox 或 visibility 当 motion/pixel GT。

## 3. C++ 输出

仅在显式环境变量提供输出路径时启用：

```text
DT_SLAM_GEOMETRY_MULTIREF_FEATURE_CSV
```

诊断长序列可选：

```text
DT_SLAM_GEOMETRY_MULTIREF_FEATURE_FRAME_IDS=16,25,35,...
```

若未设置 frame filter，则输出所有有 multi-reference evidence 的帧；若设置，
只输出列出的 frame id。frame filter 只控制诊断行，不影响 evidence 计算。

每个 ORB feature 一行：

```text
frame_id
timestamp
feature_index
u_raw
v_raw
octave
has_mappoint
current_frame_outlier_flag
semantic_nonzero
sampling_policy
native_scale
native_u
native_v
comparison_count
positive_count
negative_count
consistent_count
```

约束：

- `u_raw/v_raw` 使用 `Frame::mvKeys`，与 registered/rectified depth 域一致；
- `semantic_nonzero` 使用真实当前 `mSemanticMask != 0`；
- `native_u/v = floor(raw/native_scale)`；
- scale-2 2×2 展开像素共享同一 native cell，不算四个独立测量；
- 无 comparison 保持 `comparison_count=0`，不得解释为静态；
- `positive+negative+consistent == comparison`；
- 只读 `mvpMapPoints`/`mvbOutlier`，不改变内容；
- `current_frame_outlier_flag` 只表示 shadow 采样时仍留在 Frame 中的标志，
  不是初始 PoseOptimization 已清除关联的历史 outlier。它不能替代 GJ-2 的
  pre-clear snapshot。

## 4. 离线审计

输入：

```text
G2-4F0 feature CSV
G2-4D target_box_bbox_preannotations.csv
G2-4D exact C++ person summary/mask metadata
G2-4C corrected depth-time candidates
```

至少按以下 strata 报告：

```text
target visible + person absent
target visible + person present
target absent
inside target bbox
outside target bbox
semantic nonzero
semantic zero
```

指标：

- feature comparison coverage；
- positive-presence feature ratio；
- positive vote ratio；
- single-reference 与 multi-reference support；
- bbox 内外 enrichment；
- unique native evidence cell 数；
- 每个 native cell 的 feature multiplicity；
- has-MapPoint 与 no-MapPoint 分层；
- current frame outlier flag 仅作完整性记录，不作为初始 optimizer-outlier
  分层；
- 缺失 online geometry 帧数。

所有比率同时报告样本量和中位数/分位数，不只报告全局聚合。

## 5. 不变量

```text
dynamic_decision = none
direct_slam_state_mutation = none
selection_is_holdout_evaluation = false
coarse_bbox_is_pixel_ground_truth = false
visibility_is_motion_ground_truth = false
```

程序和脚本必须有确定性 self-test：

- feature center 到 native cell 映射；
- vote conservation；
- semantic `nonzero` 极性；
- bbox 边界包含约定；
- absent frame 不产生虚假 bbox feature；
- duplicate native cells 去重复统计。

## 6. 运行与性能

G2-4F0 是诊断 I/O 运行：

- 轨迹和 feature CSV 必须保存；
- online semantic 必须每帧就绪、age=0；
- 记录 actual FPS，但不得把带逐 feature CSV I/O 的 FPS 当最终性能；
- 不必重跑 strict hold-out；
- 优先复用两条 Bonn development 完整序列。

## 7. 放行门

本阶段不预设数值阈值。结果只做方向决策：

```text
box-local enrichment 是否存在
是否跨多个 native cells
是否独立于 person mask
是否不完全由 optimizer outlier / boundary risk 解释
```

即使方向通过，也只允许进入 motion-label protocol 设计，不直接进入 G1-F。
