# G2-2R 共视参考选择 Shadow 规格

日期：2026-07-28

## 1. 本阶段只回答的问题

G2-1 使用最近 5 个成功关键帧做全分辨率稠密 depth warp。实测表明：

- 多参考重复证据比 G0 单参考 seed 更集中；
- 最近关键帧不一定与当前帧保持高重叠；
- 5 次稠密 warp 使在线语义＋几何端到端速度降至约 19–21 FPS；
- 语义清理参考深度后，缺少比较次数不能被解释为静态。

G2-2R 只改变参考选择，回答：

> 使用 ORB-SLAM2 已有的参考关键帧和共视图，能否比“最近关键帧”提供更有解释力
> 的参考覆盖，并为后续减少参考数量提供依据？

本阶段不改变空间采样，不选择动态阈值，不影响 SLAM 状态。

## 2. 文献依据和改造边界

### `[L]` DynaSLAM

DynaSLAM 论文对每帧选择 5 个高重叠历史关键帧，并说明 5 是检测精度与计算量
之间的折中。它使用参考关键帧中带有效深度的 ORB keypoint，不是 5 次全分辨率
稠密 warp。

DynaSLAM 官方 `Geometry.cc` 的字面实现将归一化平移差和欧拉角差按
`0.7/0.3` 合成后降序排序，且使用 `Tcw` 的平移列。该实现与论文“最高重叠”
表述存在可疑不一致，因此 G2-2R 不复制该评分。

### `[L]` ORB-SLAM2

ORB-SLAM2 用共享 MapPoint 数建立共视图。Tracking 中当前 MapPoint 为观察它的
关键帧投票，共享点最多的关键帧成为当前参考关键帧；KeyFrame 提供按共视权重
排序的相邻关键帧。

### `[A]` 当前适配

G2-2R 的候选顺序为：

```text
当前 mCurrentFrame.mpReferenceKF
→ 该关键帧按共视权重排序的相邻关键帧
→ 与现有20帧几何深度缓存按 KeyFrame::mnFrameId 求交
```

这不是 DynaSLAM reference selection reproduction，而是将 DynaSLAM 的“高重叠
参考”意图适配到 ORB-SLAM2 原生共视结构。

## 3. 受控变量

保持不变：

- G0 全分辨率稠密 forward warp；
- z-buffer；
- `0.10 m` G2-1 诊断残差阈值；
- positive、negative、consistent、unknown 分类；
- 最多 20 个成功关键帧深度缓存；
- 语义动态像素在写入参考深度前置为无效；
- C++ 只输出 raw evidence count。

唯一变化：

```text
recent reference policy
vs
covisibility reference policy
```

参考数量 `K=1/2/5` 只作为消融点：

- `K=1`：最小计算路径；
- `K=2`：最小重复证据；
- `K=5`：DynaSLAM 文献设置和 G2-1 对照。

这些值不是最终动态判定阈值。

## 4. 有效性和失败处理

每帧分别记录：

```text
candidate_reference_count
cached_reference_match_count
selected_reference_count
selected_frame_ids
selected_covisibility_weights
selected_frame_ages
per-reference projected coverage
per-reference valid comparison coverage
per-reference runtime
```

只有实际选满配置要求的 `K` 个参考时，才计算该帧固定 K 的多参考直方图。选不满
时记录 `computed=0`，不回退到最近帧，也不把缺失参考作为静态票。

## 5. 验收

在 walking、fr1/xyz 和 sitting 的相同短序列上比较：

- 能选满 K 个参考的帧比例；
- 几何和 person proxy 覆盖；
- proxy precision、conditional recall、unconditional capture；
- fr1 静态选择率和 sitting 低动态选择率；
- G2 mean、median、p95；
- 端到端 actual FPS。

不预设共视选择一定更好，也不预设 K=1/2 一定满足 30 FPS。

## 6. 非目标

- 不实现稀疏 ORB-depth 采样；
- 不实现均匀网格、降分辨率或金字塔 warp；
- 不实现 MAD、Student-t 或时序概率；
- 不生成二值动态 mask；
- 不写 `mvbDynamic` 或清除 `mvpMapPoints`；
- 不新增 `PoseOptimization`；
- 不修改 YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing。
