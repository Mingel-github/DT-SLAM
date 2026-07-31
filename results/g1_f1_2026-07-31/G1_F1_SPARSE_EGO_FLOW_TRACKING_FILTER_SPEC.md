# G1-F1 稀疏 Ego-flow Tracking 过滤 SPEC

日期：2026-07-31
状态：冻结后实现
身份：实验性 feature-association filter；默认关闭

## 1. 目标

允许当前 G2-4F1 连续运动不一致证据第一次影响真实 SLAM，但只影响
`TrackLocalMap()` 中第二次、原本就存在的 `PoseOptimization()` 输入。

不要求几何分类完美。接受少量静态误删，最终以 ATE、RPE、FPS、跟踪成功率和
实际删除量评价系统折中。

## 2. 方法依据与身份

```text
observed sparse flow - RGB-D/SE(3) ego flow
```

来自当前已经实现并审计的 G2-4F1：

- `[L/A]` FlowFusion 的 ego-motion-compensated flow 思想；
- `[A]` 从 dense flow 改成 ORB 位置上的双向 PyrLK；
- `[S]` 当前帧尺度、q 工作点和 ORB-SLAM2 接入；
- `[S]` 安全保护是工程 fail-open，不是论文创新。

本阶段不是 FlowFusion 复现，不输出对象 mask，也不声称获得 unknown-object
precision/recall。

## 3. 候选定义

仅当 sample 同时满足：

```text
evidence_state == measured
forward_backward_error <= 0.25 px
semantic_nonzero == false
```

才允许成为几何候选。

使用 G1-F0 已冻结的 semantic-blind frame scale：

\[
s_t=\max(0.001,\ 1.4826\operatorname{median}_i r_i)
\]

其中尺度统计使用本帧所有质量合格的 feature，包括 semantic feature；最终几何
候选仍排除 semantic feature。至少需要 20 个尺度样本，否则本帧 fail-open。

\[
q_i=r_i/s_t.
\]

本轮只运行已冻结的：

```text
q = 6 / 8 / 10
```

不得在看完 ATE 后新增 q12、q15 或连续调参。

## 4. 准确接入位置

```text
初始 TrackWithMotionModel 或 TrackReferenceKeyFrame
→ 第一次原有 PoseOptimization
→ RunSparseEgoFlowShadow / 生成本帧候选
→ TrackLocalMap
→ SearchLocalPoints
→ 既有语义 association removal
→ G1-F1 清除候选 feature 的 mvpMapPoints
→ 第二次原有 PoseOptimization
```

不得新增第三次 PoseOptimization，不修改 `Optimizer.cc` 或 g2o。

## 5. Fail-open 保护

整帧不执行任何几何删除，若：

- 参考帧/坐标域/尺度无效；
- scale support 少于 20；
- 处于 relocalization 后的严格窗口；
- 当前关联 MapPoint 少于 30；
- 删除后关联数少于 30；
- 候选关联超过当前关联的 5%；
- feature/sample/vector 尺寸不一致。

超过 5% 时整帧 fail-open，不按分数截断 top-N，避免引入新的排序规则。

## 6. 修改边界

第一版只做：

```text
mCurrentFrame.mvpMapPoints[i] = NULL
```

明确不做：

- 不写 `mvbDynamic`；
- 不禁止 CreateNewKeyFrame/MapPoint；
- 不过滤 depth 或稠密点云；
- 不修改 LocalMapping、LoopClosing；
- 不修改 YOLO；
- 不修改 Optimizer/g2o；
- 不新增位姿优化。

因此它是 `tracking association filter`，不是完整 mapping filter。G1-M 和 G1-D
继续独立锁定。

## 7. 配置

默认：

```yaml
Geometry.SparseFlowTrackingFilterEnable: 0
Geometry.SparseFlowTrackingFilterQ: 10.0
Geometry.SparseFlowTrackingFilterMaximumAssociationFraction: 0.05
Geometry.SparseFlowTrackingFilterMinimumAssociations: 30
Geometry.SparseFlowTrackingFilterMinimumScaleSupport: 20
```

环境变量只用于可复现实验矩阵：

```text
DT_SLAM_GEOMETRY_TRACKING_FILTER
DT_SLAM_GEOMETRY_TRACKING_FILTER_Q
DT_SLAM_GEOMETRY_TRACKING_FILTER_CSV
```

q override 只允许 `6/8/10`。

## 8. 必须记录

每帧 CSV 至少包括：

- scale、scale support；
- quality-eligible feature；
- candidate feature；
- filter 前 MapPoint association；
- candidate association；
- 实际删除和剩余 association；
- candidate fraction；
- applied / fail-open；
- fail-open reason；
- relocalization window；
- `pose_reoptimization=none`；
- `mapping_veto=none`。

## 9. 第一轮实验

先 Viewer ON 跑短序列检查可视稳定性，再 Viewer OFF 独立测量。

固定比较：

```text
baseline
semantic only
geometry q6 / q8 / q10
semantic + geometry q6 / q8 / q10
```

指标：

- ATE、RPE；
- actual FPS；
- tracking lost/trajectory coverage；
- applied/fail-open frames；
- removed associations；
- remaining associations。

初步工程停止条件：

- 崩溃或新增轨迹缺失；
- 静态 ATE/RPE 相对同设置无几何版本恶化超过约 10%；
- 跟踪失败明显增加；
- fail-open 保护不满足实现不变量。

10% 是当前项目工程判据，不是论文参数。所有 q 结果并列报告，不以单一序列最好
结果选择工作点。
