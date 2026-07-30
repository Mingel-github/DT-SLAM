# G2-4F2D 高残差候选工作点冻结与 Strict Holdout 协议

日期：2026-07-30  
状态：工作点冻结；strict holdout 尚未打开  
范围：候选判决只用于审计；不写 `mvbDynamic`；不修改 SLAM 状态

## 1. 来源边界

工作点建立在：

- `[L/A]` FlowFusion 的 observed flow 减 camera ego-flow 原理；
- `[L/A]` Kalal forward-backward error 的 correspondence reliability；
- `[L/A/H]` Li–Lee 零中心鲁棒尺度/连续静态权重；
- `[S]` 当前项目的跨相机静态风险标定与 proxy development sensitivity。

没有论文为当前稀疏 ORB/LK、640×480、Bonn/TUM 相机直接给出
`FB=0.25 px` 或 `q=10`。这两个数值是项目开发期工作点 `[S/H]`，不是论文
参数。

## 2. 冻结公式

### 2.1 Quality eligibility

feature 必须同时满足：

```text
evidence_state == measured
semantic_nonzero == 0
forward_backward_error_px <= 0.25
finite SLAM-pose residual
```

FB 只判断 correspondence 是否可用，不表示对象运动。

### 2.2 Frame scale

对每帧所有 quality-eligible、非语义 feature：

\[
\hat\sigma_t=
\max(0.001,\,
1.4826\operatorname{median}_i r_i).
\]

若支持数少于 20：

```text
frame_scale_valid = false
all features = no_calibrated_evidence
fail open for any future filter
```

### 2.3 High-residual candidate

\[
q_i=\frac{r_i}{\hat\sigma_t},
\qquad
\operatorname{candidate}_i
\iff q_i\ge 10.
\]

输出状态仍为：

```text
high_residual_candidate
static_consistent
no_evidence
```

`high_residual_candidate` 不等于最终 `dynamic=true`。

不增加：

- raw residual 下限；
- depth/flow 加权和；
- boundary veto；
- region score；
- temporal vote；
- 第三次 PoseOptimization。

## 3. 选择依据

`FB=0.25, q=10` 是预先冻结网格中的保守 Pareto 点：

| 指标 | TUM fr1/xyz static | Bonn static_close_far |
| --- | ---: | ---: |
| quality coverage | 80.697% | 70.663% |
| candidate rate | 0.232% | 0.546% |
| MapPoint candidate rate | 0.068% | 0.117% |
| candidates/frame median | 0 | 3 |
| candidates/frame p95 | 11.8 | 12.8 |
| candidates/frame max | 25 | 21 |

非 holdout balloon/balloon2 development 的 exact-zero-person moving proxy：

```text
measurable frames                 = 6
frames with in-box candidates     = 6
in-box candidate-rate median      = 74.342%
same-frame outside median         = 0.000%
```

选择没有读取 strict holdout。其 SHA-256 在冻结后再次核对为：

```text
3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
```

## 4. Strict holdout 一次性评价

封存对象：

```text
BONN/rgbd_bonn_balloon_tracking.zip
```

正式解封后只允许一次完整评价：

1. 记录冻结代码 commit、配置哈希、archive 哈希；
2. 过滤归档中不存在文件并生成一对一 RGB-D association；
3. 使用 Bonn 共同 rectified pinhole domain；
4. 全序列同步运行现有 YOLO person mask，要求 `mask age=0`；
5. 全序列运行 G2-4F1/F2D shadow；
6. 候选 review 只能使用 rectified RGB temporal clip 和同步 person mask；
7. 选帧、bbox 和 motion proxy 不读取 residual、candidate、depth 或 trajectory；
8. 运行完成后不得根据结果更改 `FB/q` 并重新报告同一序列。

proxy 必须继续写成：

```text
label_source = agent_rgb_temporal_only
is_ground_truth = false
geometry_or_flow_seen = false
```

## 5. Holdout 指标与判定

报告完整序列：

- quality coverage；
- candidate rate；
- MapPoint candidate rate；
- candidate count/frame；
- F1/F2 active time；
- actual FPS/deadline miss；
- shadow 模式 ATE/RPE 健康度。

报告 RGB-only moving、exact-zero-person proxy：

- measurable frame 数；
- 有框内 candidate 的 frame 数；
- in-box/outside candidate rate；
- paired inside-minus-outside；
- SLAM/GT 方向一致性。

在打开 holdout 前冻结有限通过门 `[S/H]`：

```text
at least 5 measurable moving exact-zero-person proxy frames;
at least 80% of those frames contain an in-box candidate;
at least 80% have in-box candidate rate > same-frame outside rate;
full-sequence MapPoint candidate rate <= 0.20%;
no invariant violation;
dynamic_decision=none;
direct_slam_state_mutation=none.
```

这些是项目验证门，不是论文结论。若数据本身不足 5 个可测 proxy frame，
结论为 `holdout scientific gate not evaluable`，不得放宽标签补足。

## 6. G1-F 仍未自动放行

即使 holdout 通过，也只能进入：

```text
G1-F0 mutation shadow
```

即统计“若清除候选 MapPoint 会删除什么”，仍不真正修改。只有原代码匹配下限、
baseline match 分布、fail-open 条件和 mutation shadow 全部通过，才讨论真实
G1-F。

G1-D 继续锁定。

