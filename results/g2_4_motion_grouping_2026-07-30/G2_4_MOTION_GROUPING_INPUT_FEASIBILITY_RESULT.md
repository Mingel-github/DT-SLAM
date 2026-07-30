# G2-4 运动一致性分组：输入可行性审计结果

日期：2026-07-30
状态：development-only 输入审计完成；未实现 motion grouping；G1 继续锁定

## 1. 结论

当前 sparse F1 数据对运动一致性分组提供了**部分支持**，但不支持把固定
3/5 帧 persistence 当成通用动态判决。

```text
两帧短对应（当前帧 + 1 个历史帧）：
  多数候选帧有至少 3 个可用代理内轨迹

三帧轨迹（当前帧 + 2 个历史帧）：
  balloon  4/8 帧有至少 3 个轨迹
  balloon2 4/9 帧有至少 3 个轨迹

六帧轨迹（当前帧 + 5 个历史帧）：
  两序列都只有 2 帧仍有至少 3 个轨迹

代理内已有 MapPoint：
  balloon   8/76  = 10.53%
  balloon2  4/122 = 3.28%
```

因此：

1. 基于稳定 MapPoint identity 的长轨迹聚类，当前输入支持不足；
2. 固定 3/5 帧多数投票会在大量帧直接失去证据；
3. 两帧 transient LK grouping 在多数帧具有输入，但不能覆盖所有帧；
4. 任一 sparse grouping 必须保留 `unknown / insufficient_support`；
5. sparse 路线不能单独承担低纹理未知物体和完整 `M_depth`。

这不是 motion grouping 效果实验。当前没有建立 group、没有选择动态阈值，
也没有修改 SLAM。

## 2. 为什么需要本次审计

已有 F1/F3 CSV 只保存：

```text
feature_index
has_mappoint
```

其中 `feature_index` 每帧重新编号，`has_mappoint` 只是布尔值。CSV 没有保存
稳定的 `MapPoint::mnId`，无法直接重建 2/3/5 帧轨迹。

当前在线 F1 也只执行：

```text
当前 ORB keypoint
→ LK 回溯到上一成功帧
→ forward-backward consistency
→ ego-flow residual
```

它是相邻帧测量，不是已经建立的 feature trajectory database。

## 3. 审计输入

只使用已经打开的 development 数据：

```text
Bonn rgbd_bonn_balloon
Bonn rgbd_bonn_balloon2
```

没有重新打开 `balloon_tracking` strict holdout。

输入：

- G2-4F3 development proxy audit 中的 exact eligible nodes；
- 原 G2-4F1 exact C++ feature CSV；
- 冻结 RGB-only coarse box 已生成的 `inside_box`；
- Bonn 原始 RGB-D ZIP；
- 20 ms 一对一 associations；
- 与 C++ 相同的 Bonn `P=K` rectification。

代理框不是 pixel/object/motion GT。

## 4. 审计方法

新增离线工具：

```text
DT-SLAM/tools/audit_motion_grouping_track_support.py
```

对每个已测量 feature，从候选当前帧向历史帧做因果回溯：

```text
current
→ t-1
→ t-2
→ t-3
→ t-5
```

每一步使用与 F1 相同的：

```text
PyrLK window       21×21
maximum level      3
termination        30 iterations / epsilon 0.01
minimum eigenvalue 1e-4
```

并继续使用 F3 已冻结的：

```text
per-step forward-backward error <= 0.25 px
```

每个历史位置还要求 rectified depth 有效。

需要准确区分：

```text
survive_1_frames = 向前回溯 1 步 = 2 张图像中的对应
survive_2_frames = 向前回溯 2 步 = 3 张图像中的轨迹
survive_3_frames = 向前回溯 3 步 = 4 张图像中的轨迹
survive_5_frames = 向前回溯 5 步 = 6 张图像中的轨迹
```

审计只测量输入支持，不检查共同运动模型。

## 5. 聚合结果

### 5.1 Balloon

| 指标 | 结果 |
| --- | ---: |
| 候选帧 | 8 |
| 代理内起始 eligible tracks | 76 |
| 代理内 MapPoint | 8 / 76 = 10.53% |
| 回溯 1 步仍存活 | 64 / 76 = 84.21% |
| 回溯 2 步仍存活 | 47 / 76 = 61.84% |
| 回溯 3 步仍存活 | 41 / 76 = 53.95% |
| 回溯 5 步仍存活 | 28 / 76 = 36.84% |
| 1 步后仍有 ≥3 tracks 的帧 | 5 / 8 |
| 2 步后仍有 ≥3 tracks 的帧 | 4 / 8 |
| 3 步后仍有 ≥3 tracks 的帧 | 3 / 8 |
| 5 步后仍有 ≥3 tracks 的帧 | 2 / 8 |

### 5.2 Balloon2

| 指标 | 结果 |
| --- | ---: |
| 候选帧 | 9 |
| 代理内起始 eligible tracks | 122 |
| 代理内 MapPoint | 4 / 122 = 3.28% |
| 回溯 1 步仍存活 | 77 / 122 = 63.11% |
| 回溯 2 步仍存活 | 36 / 122 = 29.51% |
| 回溯 3 步仍存活 | 18 / 122 = 14.75% |
| 回溯 5 步仍存活 | 11 / 122 = 9.02% |
| 1 步后仍有 ≥3 tracks 的帧 | 7 / 9 |
| 2 步后仍有 ≥3 tracks 的帧 | 4 / 9 |
| 3 步后仍有 ≥3 tracks 的帧 | 3 / 9 |
| 5 步后仍有 ≥3 tracks 的帧 | 2 / 9 |

### 5.3 支持量高度不均匀

代表性帧：

```text
balloon frame 252:
  start 32 → 5-step 22

balloon frame 39:
  start 9 → 2-step 2 → 3-step 0

balloon2 frame 318:
  start 7 → 5-step 6

balloon2 frame 54:
  start 34 → 2-step 2 → 5-step 0
```

起始点多并不保证长轨迹多。遮挡、快速运动、边界、深度无效和 LK 误差都会使
轨迹迅速减少。

## 6. 背景对照

同一批帧的背景 feature 支持明显更稳定：

```text
balloon background 5-step survival ratio  = 76.86%
balloon2 background 5-step survival ratio = 69.94%
```

这意味着简单的：

```text
轨迹存活时间短 → 动态
```

不是安全判决。动态代理内轨迹较短可能是运动、遮挡或低纹理造成的，但轨迹
长度本身不能区分这些原因。

## 7. 与论文路线的对应

### Dai point correlations

`[L]` 原方法前端可以使用相邻两帧，但节点是已跟踪地图点，并有：

- 三维相对位置 edge state；
- RGB-D 协方差；
- point-correlation optimization；
- 离群边迭代剔除；
- connected components；
- 最大空间体积 component 为静态组的假设。

当前动态代理内 MapPoint 比例只有 3.3%–10.5%。若改用 transient LK nodes，
这是显著 `[A/H]`，不是直接复现。

### ClusterSLAM

`[L]` 使用 landmark motion affinity、短 frame chunks、consensus clustering
和 backend factor graph。当前稳定 MapPoint/长轨迹支持不足，且 backend 超出
范围，因此不适合直接移植。

### 长轨迹 RGB-D motion segmentation

`[L]` Bertholet 等证明 feature trajectories 可以初始化运动分割，但其方法在
整个序列上优化分割和物体运动。当前 3/5 步轨迹覆盖不足，且该方法不是在线
轻量前端。

### FlowFusion / Jaimez / SInDSLAM / DetectFusion

这些方法依靠 dense flow、几何 cluster、surfel/ICP residual 或完整
re-clustering，在低 sparse support 时仍可能形成区域。这说明区域路线不能因
F4 简化失败而整体否决。

## 8. 当前路线决策

### 不批准

```text
固定 3/5 帧 residual 多数投票
基于已有 MapPoint 的 long-track object clustering
track lifetime 直接作为 dynamic score
立即把两帧 LK group 写入 mvbDynamic
```

### 仍可研究

```text
两帧 transient feature 的 motion-coherent grouping
+ uncertainty / support sufficiency
+ explicit unknown
```

但它只能作为高置信 `D_feat` 候选，不能承诺完整 `M_depth`。

### 对原 A/B 二选一的修正

```text
B（sparse grouping）：
  输入部分可行，可做机会式高置信 D_feat；
  不能成为覆盖所有未知动态的唯一方法。

A（更强 region representation）：
  对低纹理和 M_depth 仍有必要；
  用户已放宽 30 FPS 硬约束，因此计算量不再是立即否决理由。
```

合理总结构更可能是：

```text
sparse motion evidence / grouping
           +
stronger geometric region support
```

但不能一次实现两条完整系统。

## 9. 下一步

下一阶段仍先做设计审计，不直接写过滤：

1. 核对 Dai 前端完整两帧 point-correlation 分组能否在 transient LK node
   上形成有依据的最小适配；
2. 核对 DetectFusion/Tateno 的 normal+distance geometric segmentation，
   是否可作为比 G2-3R0 更完整、但比 SInDSLAM 更小的独立区域表示；
3. 比较两者对当前目标的覆盖：
   - `D_feat`；
   - `M_depth`；
   - 低纹理；
   - unknown 状态；
   - 预计代码量和运行时间；
4. 只选择一条做下一次 shadow representation 实验。

G1-F/G1-D 在该选择完成前继续锁定。

## 10. 验证与产物

工具通过：

```text
Python syntax check       PASS
deterministic replay      PASS
dynamic_decision          none
direct_slam_state_mutation none
```

产物：

```text
DT-SLAM/tools/audit_motion_grouping_track_support.py
results/g2_4_motion_grouping_2026-07-30/balloon/per_track.csv
results/g2_4_motion_grouping_2026-07-30/balloon/per_frame.csv
results/g2_4_motion_grouping_2026-07-30/balloon/summary.json
results/g2_4_motion_grouping_2026-07-30/balloon2/per_track.csv
results/g2_4_motion_grouping_2026-07-30/balloon2/per_frame.csv
results/g2_4_motion_grouping_2026-07-30/balloon2/summary.json
```

本阶段没有运行 SLAM 数据集，因此没有 Viewer 开关问题。后续若进行在线短序列
shadow，将先运行 Viewer ON 版本供人工观察，再用 Viewer OFF 独立测量性能。
