# G2-4F1 Sparse Ego-flow Shadow 结果

日期：2026-07-29
状态：实现与 development 审计完成；科学放行门未通过。
范围：连续稀疏 residual；无分类、无融合、无 SLAM 写入。

## 1. 实现身份

本阶段实现：

```text
当前 ORB feature
→ backward LK 到上一成功帧
→ reference depth + 相邻帧 SE(3) 生成 camera-induced flow
→ observed flow - ego flow
→ 连续 residual 与 no-evidence 状态
```

方法身份保持为：

```text
[A/S] FlowFusion ego-flow residual principle
      adapted to sparse ORB features
      + adjacent successful RGB-D frames
      + ORB-SLAM2 initial pose
```

它不是 FlowFusion 复现，也没有引入稠密神经光流。

调用顺序：

```text
initial pose available
→ RunGeometryShadow()
→ RunSparseEgoFlowShadow()
→ TrackLocalMap()
```

reference 只在完整 `Track()` 成功返回后更新，因此当前帧使用 initial pose，
上一帧使用 final pose；不增加第三次 `PoseOptimization`。

## 2. 新增接口与配置

主要实现：

- `GeometricDynamicDetector::ComputeSparseEgoFlow()`；
- `Tracking::RunSparseEgoFlowShadow()`；
- `Tracking::UpdateSparseEgoFlowReference()`；
- `BONN_GeometrySparseEgoFlowShadow.yaml`；
- `TUM3_GeometrySparseEgoFlowShadow.yaml`；
- `audit_bonn_sparse_ego_flow.py`。

输出只包含：

- backward/forward LK 状态；
- forward-backward error；
- reference depth validity；
- SLAM/GT ego projection；
- residual x/y/magnitude；
- `measured/lk_invalid/depth_invalid/projection_invalid/...`；
- 分阶段耗时。

CSV 中不存在 `dynamic/static` feature 标签。

## 3. 测试与数据契约

确定性测试覆盖：

- identity pose；
- 已知相机位移；
- 注入独立像素位移；
- GT/SLAM 分支独立性；
- 无效深度；
- 相机后方投影；
- LK failure。

结果：

```text
geometric_warp_test = PASS
feature CSV extra columns = 0
illegal evidence states = 0
non-adjacent references = 0
measured-state invariant violations = 0
dynamic_decision = none
direct_slam_state_mutation = none
```

80 帧初始 smoke 共输出 `79,246` feature rows 和 `79` frame rows；全部
不变量通过。

## 4. Bonn GT frame 修正

初次运行时，Bonn 文本 GT 直接被解释为 RGB optical frame，导致大量静态
feature 的 GT ego-flow residual 达到约 `10–20 px`。

Bonn 官方页面给出的模型/marker 坐标链为：

```text
T_global = T_ROS^-1 * T_text * T_ROS * T_m
```

因此增加了可选配置：

```text
GroundTruth.TwcRightTransform
```

Bonn F1 配置写入官方 `E = T_ROS * T_m`；相对位姿中左侧全局变换抵消。
普通 TUM 配置不启用该变换。

修正后全候选 feature 的 GT residual 中位数：

| 序列 | 修正前 | 修正后 |
| --- | ---: | ---: |
| nonobstructing | 7.878 px | 3.132 px |
| obstructing | 8.082 px | 2.267 px |

改善明显，但仍高于 SLAM-pose 分支。可能仍含 OptiTrack/RGB 时间同步、
marker-to-optical calibration 与动态前景影响；目前不能把其余差值归因于某一
项。GT residual 继续只作为风险诊断，不参与动态门。

独立 TUM3 walking-xyz 149 帧检查中，SLAM/GT feature residual 中位数分别为
`1.065/1.514 px`，说明通用插值与投影分支没有出现 Bonn 量级的系统错误。

## 5. 完整同步语义实验

开发序列：

- Bonn moving nonobstructing box：778 帧；
- Bonn moving obstructing box：589 帧。

两次均使用：

```text
CUDAExecutionProvider
online YOLO mask = 每帧精确同步
mask age median/max = 0/0
viewer = off
```

### 5.1 端到端和 F1 成本

| 序列 | actual FPS | active_total 中位 | deadline miss | F1 active 中位 |
| --- | ---: | ---: | ---: | ---: |
| nonobstructing | 29.270 | 33.069 ms | 302/778 | 2.550 ms |
| obstructing | 29.698 | 30.127 ms | 25/589 | 2.523 ms |

两条序列都接近 30 FPS，但 nonobstructing 的 deadline miss 很多，说明同步
semantic baseline 仍没有稳定的 33.3 ms 余量。F1 成本可测且较小，不能据此
宣称完整系统已经稳定实时。

候选 feature 输出：

```text
nonobstructing = 24,101 rows
obstructing    = 22,829 rows
```

保存的 C++ person mask 与在线 `semantic_nonzero` feature flag 为
`46,930/46,930` 完全一致。

## 6. 连续证据结果

独立 RGB temporal proxy 不是 GT。48 个原候选中：

```text
moving + person absent  = 1
moving + person present = 6
stationary + person absent = 30
uncertain + person absent = 2
not visible = 9
```

person-mask 像素从箱体框内 feature 中排除后：

| 分层 | 帧数 | frame-balanced 箱内 SLAM residual 中位 |
| --- | ---: | ---: |
| stationary + person absent | 30 | 0.313 px |
| moving + person present | 6 | 1.334 px |
| moving + person absent | 1 | 0.501 px |
| uncertain + person absent | 2 | 0.589 px |

`moving+person-present` 对 `stationary+person-absent` 的 frame-level
rank AUC 为 `0.917`。这是一个方向性信号，但不能作为放行结论，因为：

- positive 只有 6 帧；
- moving 分层仍与人物出现相关；
- 候选来自 development proxy 选帧；
- 粗框不是 pixel GT；
- `moving+person-absent` 只有 1 帧。

箱内/箱外 residual ratio 的同一对照 rank AUC 仅为 `0.644`，说明较大的
全局 residual 不一定局部集中于箱体，初始位姿污染仍可能参与。

## 7. 决策

当前结论是：

```text
implementation gate = PASS
runtime measurement gate = PASS
continuous directional evidence = present but confounded
moving+person-absent scientific gate = NOT EVALUABLE
G1-F release = false
G1-D release = false
```

不得：

- 根据 `0.917` AUC 直接选阈值；
- 把 depth 与 flow score 相加制造分离；
- 将 GT residual 当作已校准真值；
- 让 residual 写入 `mvbDynamic`；
- 解封 strict hold-out。

原始证据：

- `results/g2_4f1_2026-07-29/development_online_semantic_official_gt_frame/`
- `results/g2_4f1_2026-07-29/development_audit_v2_online_semantic_official_gt_frame/`
- `results/g2_4f1_2026-07-29/smoke/`
