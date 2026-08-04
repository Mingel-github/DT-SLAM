# S1 native CPU evidence＋区域判决 shadow 结果

日期：2026-08-04  
结论：受控行为链通过；S1 尚未完成；S2/S3 继续锁定

## 1. 已完成链路

```text
本机 CPU DeepFlow
→ t-2 / large-motion t-1
→ VariationalRefinement
→ previous detector state/label weighted homography
→ observed - induced flow residual
→ above-low/high threshold evidence
→ 作者 reference labels/imgTotalArea
→ cluster-confined region decision
→ 0/125/255 detector state
```

其中 DeepFlow、region classifier 和时序 prior 均在 DT-SLAM 内 clean-room
实现；labels/imgTotalArea 暂时仍由独立作者运行提供，以隔离验证运动证据和判决
行为。

## 2. 作者 CPU 参考

参考目录：

```text
/data/dynaslam/large_results/
sindslam_s1_region_cpu_qualified_tum3_walking_30_v2
```

条件：

- system OpenCV 4.5.4 CPU DeepFlow；
- temporal prior 开启；
- `SIND_SLAM_FIX_THRESHOLD_MASK_COUNT=1`；
- `SIND_SLAM_REQUIRE_VALID_FLOOD_SEED=1`；
- 30 输入帧，29 组有效证据；
- region-valid、labels、threshold mask、9×9 detector state 和 runner final
  mask 分层保存。

作者隔离 CPU build 还修复了两个环境问题：必须链接作者自己的 g2o，避免与
DT-SLAM g2o ABI 混用；headless 审计时只关闭 `imshow/waitKey`，不改变算法。

## 3. 确定性对照结果

### 3.1 区域判决

29 帧逐像素对照：

```text
raw_state_mismatch_total = 0
raw_state_mismatch_max = 0
mismatching_frames = 0
dynamic_decision = shadow_only
direct_slam_state_mutation = none
```

对照目标是作者 `_mask_pre_runner_dilate.png`，不是 runner 再膨胀后的 final
tracking mask。

### 3.2 native CPU DeepFlow＋时序先验

```text
raw_flow_max = 0
observed_flow_max = 0
homography_max = 2.38419e-06
residual_max = 9.53674e-06 px
normalized_max = 1 gray level
low_mask_mismatch = 0
high_mask_mismatch = 0
reference_selection_mismatch = 0
threshold_mismatch = 0
```

时序先验开启时 H 的容差单独设为 `3e-6`；这是小于 residual/mask 输出精度的
数值差异，不改变阈值或判决。关闭 prior 的早期对照仍保留更严格的 `1e-6`。

### 3.3 Tracking shadow

两条 30 帧系统路径均运行成功：

- 全 reference replay；
- native CPU DeepFlow＋native region classifier，replay labels/imgTotalArea。

两者均为 29/29 有效判决，native classifier 的 author-style dynamic pixel 与
独立作者 raw dynamic pixel 每帧一致：

```text
max_dynamic_pixel_delta = 0
actual_slam_removed = 0 for all frames
direct_slam_state_mutation = none
```

native CPU 完整 shadow 的实测：

```text
median tracking = 158.273 ms
actual_fps = 5.5214
deadline_missed = 29/30
```

该速度只代表当前 CPU DeepFlow shadow 链，不外推到 GPU Brox，也不预设最终
系统速度。

### 3.4 默认关闭回归

TUM3 walking 10 帧、原始 `TUM3.yaml`：

```text
sequence completed
new map = 732 points
no SIn S1 runtime log
```

## 4. 本轮修正的关键接口错误

第一次 classifier 对照出现大面积 mismatch，原因不是阈值，而是把三个域错误
合并：

1. 作者 `imgTotalArea`；
2. 正 region-label 支持；
3. DT-SLAM finite depth validity。

拆开后剩余 42 像素差异；进一步核对源码发现 current high seed 不先与
`imgTotalArea` 相交，只有 above-low 支持相交。按作者行为修正后 29 帧 mismatch
降为零。这说明三态 validity 与标签域必须继续独立保存。

## 5. 客观结论

可以确认：

- 本机 CPU DeepFlow evidence backend 已能复现相同作者 CPU evidence；
- 区域判决、上一帧 high 支持和 detector-state homography prior 的行为链已接通；
- shadow 接入没有修改 SLAM 状态。

不能确认：

- DT-SLAM 已拥有独立完整的 SIn-style detector；
- 当前 native RAG labels 等价于作者重聚类；
- mask 在 TUM/Bonn 上具有足够动态对象特异性；
- 可以进入 S2 真实过滤；
- CPU 路径满足实时要求。

## 6. 下一步

S1 下一项不是调 residual 或 flood-fill 阈值，而是把 native region representation
作为明确候选接入同一 classifier，并与作者 labels 做成对行为/质量审计：

```text
author labels + native evidence  （已通过的受控参照）
vs
native 3D/gradient/RAG labels + native evidence
```

必须明确 native validity 的定义，并继续分开报告 filled core、9×9 detector mask
与 project depth-supported mask。只有 native 完整 shadow 在代表性 TUM/Bonn
序列稳定运行并能解释主要差异后，才讨论 S2。

## 7. native gradient/RAG 区域候选成对审计

后续已将 clean-room 的 native 3D/coarse-to-fine、depth-gradient split 和
gradient-only RAG labels 接入同一 CPU DeepFlow 与 region classifier。该路径明确
不包含作者 PEAC/AHC plane re-clustering，因此身份是区域近似候选，不是 SInDSLAM
区域复现。

### 7.1 TUM3 walking，30 帧

| 指标 | 结果 |
| --- | ---: |
| 有效 region decision | 29/29 |
| author-reference raw dynamic ratio，中位 | 16.83% |
| native author-style dynamic ratio，中位 | 16.97% |
| native/author mask IoU，中位 / 均值 | 0.902 / 0.819 |
| IoU 最小值 | 0.187（frame 12） |
| native dynamic ORB 数，中位 | 234 |
| native ORB 中落入 author mask 的比例，中位 | 0.989 |
| native RAG region 数，中位 | 61 |
| classifier 时延，中位 | 4.55 ms |

不能仅凭相近的中位面积宣称等价。frame 12 中作者参考主要标出右侧人物头部，
native 区域把右侧人物大部分并入动态；frame 16、17、29 也存在不同程度的整人
扩展。TUM walking 中人物确实处于动态场景，但作者 mask 不是人工真值，因此这些
差异只能记为区域粒度差异，不能直接判定 native 为误检或更完整检测。

### 7.2 Bonn moving_nonobstructing_box，30 帧

为避免跨输入域比较，本轮使用作者公开实现的 Bonn raw pinhole 配置，独立导出
CPU DeepFlow residual、labels、region-valid 与 detector mask，再用完全相同的
30 帧 association 运行 DT-SLAM native shadow。

| 指标 | 结果 |
| --- | ---: |
| 有效 region decision | 29/29 |
| author-reference raw dynamic ratio，中位 | 19.25% |
| native author-style dynamic ratio，中位 | 10.47% |
| native/author mask IoU，中位 / 均值 | 0.862 / 0.758 |
| IoU 最小值 | 0.057（frame 21） |
| native dynamic ORB 数，中位 | 51 |
| native ORB 中落入 author mask 的比例，中位 | 1.000 |
| native RAG region 数，中位 | 42 |
| classifier 时延，中位 | 3.43 ms |

这里的中位 ORB overlap 只说明 native 选中的点通常是作者 mask 的子集，不说明
对象覆盖充分。frame 10、12、21 中作者参考覆盖移动柜体的大部分，而 native 仅
保留顶部或局部区域；frame 21 的 author/native author-style dynamic pixels 分别
为 79,178/4,547。相反，frame 5 和 13 也出现 native 比作者范围更大的情形。

### 7.3 确定性与状态安全

- 相同 TUM 输入重跑时，dense evidence、native labels、region mask 和所有非
  ORB、非 runtime 字段逐项一致；ORB 提取本身有少量运行波动；
- 两个序列所有 `actual_slam_removed=0`；
- `dynamic_decision=shadow_only`、`direct_slam_state_mutation=none`；
- 新增的 mask 输出只在显式设置目录时启用，默认不写图、不改变判决。

## 8. S1 当前放行结论

可以确认的部分：

- CPU DeepFlow、相机补偿、阈值、时序 prior 和 region classifier 的受控行为链
  已通过；
- native gradient/RAG 区域候选能够稳定运行，在多数测试帧与作者输出具有较高
  重叠；
- shadow 接入没有触碰 Tracking/MapPoint 状态。

仍未通过的部分：

- Bonn 关键移动柜体帧存在明显欠覆盖；
- TUM 部分人物帧存在明显扩大；
- 当前区域候选未保留 SInDSLAM PEAC plane re-clustering 的完整成立条件；
- 因此不能把 native 输出称为作者等价 detector，也不能直接开放为默认过滤。

本地 PEAC/AHC 文件明确为 `AGPL-3.0-or-later`。直接复制会改变当前项目的许可
义务；此前 OpenCV `RgbdPlane` 许可安全替代又已通过实验判定为区域过度碎裂且
成本过高。下一决策必须在以下两项之间明确选择，而不是继续调 RAG 阈值：

1. 保留当前 native candidate，进入**显式实验性、默认关闭、带删除上限的 S2
   对照**，以 ATE/RPE、实际删除和地图点写入量判断其稀疏定位价值；
2. 在接受 AGPL 依赖或依据 PEAC 论文独立实现平面重聚类后，再追求更完整的
   `M_depth`。

无论选择哪项，S3 动态深度过滤仍不得使用当前 native mask。
