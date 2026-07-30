# G2-4F2 可靠 Feature Evidence Gate Shadow SPEC

日期：2026-07-30  
状态：实现前冻结  
范围：离线审计优先；shadow-only；G1-F/G1-D 保持锁定

## 1. 目标

回答：

1. 排除明显不可靠 LK correspondence 后，气球方向性 residual 是否仍存在；
2. 以每帧鲁棒静态尺度归一化后，真正静态序列上的候选率能否受控；
3. 是否存在一个只由静态风险预算选择、而非由气球结果反向拟合的工作点；
4. 该工作点是否保留足够 ORB/MapPoint 观测，具备进入未来 G1-F shadow
   mutation audit 的资格。

本阶段不回答“几何过滤是否改善 ATE”；它仍不改变 SLAM 观测。

## 2. 阶段拆分

### G2-4F2Q：质量与连续静态似然

输入现有 G2-4F1 feature CSV，离线计算：

- raw LK status；
- forward-backward error；
- raw SLAM-pose/GT-pose residual；
- quality eligibility；
- frame robust residual scale；
- normalized residual；
- Student-t-style continuous static weight。

不输出 binary dynamic。

### G2-4F2D：静态风险约束的候选工作点

先冻结静态 candidate-rate 曲线，再选择一个保守 operating point，输出：

```text
high_residual_candidate
ambiguous
static_consistent
no_evidence
```

其中 `high_residual_candidate` 仍不是最终 G1 `dynamic=true`。

## 3. 数据角色

```text
static calibration/development:
  TUM fr1/xyz
  Bonn rgbd_bonn_static
  Bonn rgbd_bonn_static_close_far

motion development:
  Bonn balloon
  Bonn balloon2

strict holdout:
  Bonn balloon_tracking
```

当前本地只有 `fr1/xyz` 和两个 balloon development archive。Bonn static
缺失时允许先完成 F2Q 工具和 `fr1/xyz` 初审，但：

```text
Bonn-domain static risk gate = NOT EVALUABLE
F2D operating point          = NOT FROZEN
strict holdout               = SEALED
```

不得用 `sitting_static` 代替真正静态场景。

## 4. 输入合同

每条 feature 至少需要：

```text
frame
feature_index
octave
has_mappoint
semantic_nonzero
backward_lk_status
forward_lk_status
forward_backward_error_px
reference_depth_valid
slam_ego_projection_valid
slam_residual_magnitude_px
gt_pose_available
gt_ego_projection_valid
gt_residual_magnitude_px
evidence_state
```

每条 frame 需要：

```text
reference_available
domain_valid
feature_count
measured_count
active_total_ms
dynamic_decision=none
direct_slam_state_mutation=none
```

输入坐标域必须为共同 undistorted pinhole domain。TUM1 非零畸变不能直接使用
raw-domain F1；需先将 RGB 与 registered depth 联合 remap，并把 Tracking
distortion 设为零。

## 5. F2Q 计算

### 5.1 Measurement validity

只接受：

```text
evidence_state == measured
backward_lk_status == 1
forward_lk_status == 1
reference_depth_valid == 1
slam_ego_projection_valid == 1
finite residual and FB error
```

其他全部为 `no_evidence`。

### 5.2 FB quality

F2Q 不立即冻结单一 `tau_FB`。离线报告：

```text
raw thresholds: 0.25, 0.5, 1.0, 2.0 px
static empirical quantiles: p90, p95, p99
```

固定列表是诊断网格 `[S]`，不是论文阈值。最终 `tau_FB` 若冻结，只能来自
static calibration，并必须报告对 measured coverage 的影响。

### 5.3 鲁棒尺度

对每帧、每个 FB working point 的 quality-eligible feature 集合
\(\mathcal E_t\)，计算：

\[
\hat\sigma_t
=
\max\left(
\sigma_{\min},
1.4826\operatorname{median}_{i\in\mathcal E_t}r_i
\right).
\]

其中：

- \(r_i\) 为非负 SLAM-pose flow residual magnitude；
- scale estimator 默认排除 `semantic_nonzero=1`；
- invalid/no-evidence 不参与；
- `sigma_min` 只防除零，必须作为配置和报告字段；
- 若 \(|\mathcal E_t|\) 小于预设支持数，则 frame scale 为 invalid，所有
  feature 保持 `no calibrated evidence`。

该式为 `[A/H]`，需同时输出 raw residual，防止归一化掩盖系统性位姿误差。

### 5.4 连续输出

\[
q_i=\frac{r_i}{\hat\sigma_t},
\]

\[
w_i^{static}
=
\min\left(
1,
\frac{11}{10+q_i^2}
\right).
\]

输出：

```text
frame_scale_px
normalized_residual
static_weight
quality_eligible
```

F2Q 不输出 dynamic/static。

## 6. F2D 工作曲线

在实现前冻结候选网格：

```text
FB quality:
  raw 0.25 / 0.5 / 1.0 / 2.0 px
  static p90 / p95 / p99

normalized residual:
  q = 2 / 3 / 4 / 5 / 6 / 8 / 10

raw residual audit:
  0.5 / 1 / 2 / 3 / 5 / 8 / 10 px
```

raw residual 只作解释，不与 q 做事后 OR 组合。

每个工作点报告：

- static-sequence candidate rate；
- candidate frames / all frames；
- quality-eligible coverage；
- MapPoint candidate rate；
- 每帧 candidate count 的 median/p95/max；
- 每帧保留 feature/MapPoint count；
- balloon exact-zero-person moving proxy 的框内 candidate coverage；
- 同帧框外/person-excluded candidate rate；
- paired inside-minus-outside 和 inside/outside ratio；
- SLAM-pose 与 GT-pose方向是否一致。

## 7. 工作点选择规则

当前先报告 Pareto 表，不提前指定最终数值。

只有同时满足以下条件，才允许冻结 F2D operating point：

1. TUM 与 Bonn 两个相机域均有真正静态负样本；
2. 工作点由静态风险预算选择，不读取 strict holdout；
3. balloon/balloon2 development 方向性信号仍存在；
4. 结果不是由高 FB error、semantic region 或单个 frame 搬运；
5. MapPoint/feature 保留数未触及 ORB-SLAM2 原有 tracking 下限；
6. 参数和数据角色在打开 holdout 前写入冻结文档。

若不存在同时满足的工作点，则 F2D 失败，不通过增加 depth/flow 加权和阈值
组合补救。

## 8. 未来 G1-F fail-safe（本阶段只审计）

若 F2D 以后通过，G1-F 仍需先运行 mutation shadow：

- 只统计“若过滤将删除哪些 `mvpMapPoints`”，不真正清空；
- 初始 tracking 剩余 map matches 必须明显高于 ORB-SLAM2 原有 `10/15/20`
  级失败边界；
- 局部地图搜索后再次统计；
- candidate 过多或 scale invalid 时 fail open：本帧不执行几何过滤；
- semantic filtering 行为保持不变；
- 不增加第三次 `PoseOptimization`。

具体安全余量不能现在凭空指定；应由当前代码原有阈值、baseline match
分布和 shadow mutation 统计共同冻结。

## 9. 测试

离线工具必须包含：

- malformed CSV/header rejection；
- non-finite value rejection；
- invalid evidence 不进入尺度；
- semantic feature 不进入默认尺度；
- zero residual 的 scale floor；
- deterministic percentile/MAD；
- FB gate 只影响 quality eligibility，不直接产生 motion；
- 关闭 decision 时输出中不存在 dynamic/static mutation；
- 输入顺序改变不影响 frame-level 结果。

若以后增加 C++ F2Q：

- 其数值必须与离线工具逐 feature 一致；
- 配置默认关闭；
- 关闭后当前轨迹、日志和 F1 CSV 合同不变。

## 10. 实现顺序

```text
1. 完成离线 F2Q/F2D audit tool + self-test
2. 生成 TUM1 rectified sparse-flow shadow config
3. 只跑短静态序列并检查坐标域/不变量
4. 补齐 Bonn static 数据后做跨相机 static-risk curve
5. 在已有 balloon/balloon2 CSV 上做 development sensitivity
6. 冻结或否决 operating point
7. 通过后才讨论一次性 strict holdout
```

## 11. 全阶段不变量

```text
dynamic_decision           = none
depth_flow_fusion          = none
direct_slam_state_mutation = none
G1-F / G1-D                = locked
strict holdout             = sealed and unopened
YOLO / Optimizer / g2o     = unchanged
```

