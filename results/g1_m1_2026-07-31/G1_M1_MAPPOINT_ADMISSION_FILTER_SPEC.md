# G1-M1 MapPoint 写入过滤 SPEC

日期：2026-07-31
状态：冻结后实现
默认：关闭
范围：稀疏 MapPoint 写入保护；不包含深度区域过滤

## 1. 目标

G1-F1 已在 `TrackLocalMap()` 的第二次既有位姿优化前清除少量高置信 q10
MapPoint association。G1-M0 又确认，同一候选仍可能在
`CreateNewKeyFrame()` 中借助 RGB-D 深度被重新创建为 MapPoint。

G1-M1 的唯一目标是：

> 当一帧满足 G1-F1 的既有安全条件并将被创建为 KeyFrame 时，禁止该帧中
> 高置信 q10 sparse ego-flow 候选写入新的静态 MapPoint。

它不是新的动态检测算法，也不是新的后端状态。

## 2. 依据与方法身份

- `[L]` ORB-SLAM2 RGB-D：初始化和新关键帧可直接从单帧深度创建 MapPoint；
- `[L]` Ji 2021：动态特征不应参与相机位姿估计或静态地图构建；
- `[L]` DynaSLAM：动态观测应从静态 tracking/mapping 中排除；
- `[E]` 当前 DT-SLAM：`KeyFrame` 已复制 `Frame::mvbDynamic`，
  `LocalMapping::CreateNewMapPoints()` 已跳过任一端被标记 dynamic 的特征；
- `[S]` 将已经通过 G1-F1 q10 筛选的候选并入统一 `mvbDynamic`，并采用
  fail-open 上限，是本工程的保守系统集成。

因此，论文归属应写为“已有动态观测排除原则在当前 ORB-SLAM2 fork 中的最小
实现”，不能把写图否决本身写成算法创新。

## 3. 精确接入点

```text
Tracking::CreateNewKeyFrame()
  → mpLocalMapper->SetNotStop(true)
  → G1-M0 读取 mutation 前状态（若启用）
  → G1-M1 检查安全条件并将候选并入 mCurrentFrame.mvbDynamic
  → 既有 RemoveDynamicAssociations(mCurrentFrame)
  → 既有 KeyFrame(mCurrentFrame, ...)
  → 既有 RGB-D depth admission（已经检查 !mvbDynamic）
  → LocalMapping（已经检查 KeyFrame::mvbDynamic）
```

不修改：

- `LocalMapping.cc`；
- `Optimizer.cc`；
- g2o；
- YOLO；
- 位姿优化次数。

## 4. 启用条件

G1-M1 只允许：

```text
sensor                                      RGB-D
Geometry.SparseEgoFlowShadowEnable          1
Geometry.SparseFlowTrackingFilterEnable     1
Geometry.SparseFlowTrackingFilterQ          10
Geometry.SparseFlowMappingFilterEnable      1
mapping-filter CSV                          非空
```

G1-M0 counterfactual 与 G1-M1 不允许同时启用，因为前者声明
`direct_mapping_state_mutation=none`。

## 5. Fail-open 条件

只有全部满足时才写入 dynamic flag：

1. candidate、MapPoint、outlier、depth 和 dynamic 向量尺寸一致；
2. G1-F1 scale 有效；
3. 不在 relocalization 保护窗口；
4. G1-F1 基线 association 不少于 30；
5. q10 candidate association 比例不超过 5%；
6. 删除后 association 仍不少于 30；
7. q10 candidate feature 不超过当前非动态 feature 的 5%；
8. q10 candidate valid-depth feature 不超过当前非动态 valid-depth
   feature 的 5%；
9. 否决后仍至少保留 100 个非动态 valid-depth feature。

任何一项失败时，本帧不做几何写图否决。这里的 5% 和 100 是保护系统完整性的
工程上限 `[S]`，不是论文参数。

G1-M0 三序列数据表明：

- 通过 tracking 条件的 candidate feature 比例最大约 4.78%；
- candidate valid-depth 比例通常较低，但 walking 个别关键帧约 8.1%；
- 因而 5% depth 上限会主动放弃少数激进帧，而不是无条件删点。

## 6. 实际动作

安全条件通过时：

```cpp
mCurrentFrame.mvbDynamic[i] = 1;
```

只对 q10 candidate 且尚未被语义标记的 feature 执行。随后完全复用已有：

- `RemoveDynamicAssociations()`；
- `CreateNewKeyFrame()` 的 `!mvbDynamic` depth admission；
- `KeyFrame` 对 `mvbDynamic` 的复制；
- `LocalMapping` 的 dynamic endpoint guard。

## 7. 初始化限制

RGB-D 首帧没有上一成功帧，无法计算 adjacent-frame sparse ego-flow。

因此 `StereoInitialization()`：

- 不生成或猜测几何候选；
- 不执行 G1-M1；
- 仅记录 `reference_unavailable_fail_open`；
- 仍由语义 mask 和 ORB-SLAM2 原有规则保护。

这是当前方法的明确限制。

## 8. 诊断输出

每个初始化/关键帧事件至少记录：

- frame、timestamp、stage、q；
- scale/vector/tracking-safeguard 状态；
- candidate / available feature 数与比例；
- candidate / available valid-depth 数与比例；
- 配置上限和剩余深度数；
- mutation 前 candidate association；
- G1-F1 已清除的 association；
- 新增 dynamic flag、实际清除 association、实际否决深度数；
- 实际创建 MapPoint 数；
- 其中 candidate MapPoint 数；
- `mapping_filter_applied` 与 fail-open 原因；
- `pose_reoptimization=none`。

关键不变量：

```text
applied=true  → candidate_created_mappoints=0
new_dynamic_flags <= candidate_features
vetoed_depth_features <= candidate_valid_depth_features
remaining_valid_depth = valid_depth - vetoed_depth
```

## 9. 验证顺序

1. 构建、现有 `geometric_warp_test` 和配置 fail-fast；
2. 150 帧 walking 冒烟，核对 CSV 不变量；
3. Viewer ON 短序列只做定性检查；
4. Viewer OFF 正式比较：
   - semantic control；
   - semantic + G1-F1 q10；
   - semantic + G1-F1 q10 + G1-M1；
5. 使用 walking、sitting_static、Bonn balloon，补充 fr1/xyz 静态安全检查；
6. 同时报 ATE、RPE、FPS、轨迹覆盖和 MapPoint veto 统计。

## 10. 停止条件

出现任一情况就保持默认关闭，不进入冻结版本：

- 静态 fr1/xyz ATE 或 RPE 相对对照恶化超过约 10%；
- 任一序列轨迹覆盖下降；
- walking、sitting、Bonn 多次运行中出现系统性明显退化；
- CSV 不变量违反；
- filter 在 scale/向量/relocalization 失败时仍修改地图。

G1-M1 通过后，只能说明“稀疏 tracking + MapPoint 写入保护”可形成实验版本。
它不等于 `G1-D`，也不能声称已经得到可靠的像素级动态深度 mask。
