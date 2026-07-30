# G2-4C Bonn Moving-Box 自动候选选帧规格

日期：2026-07-29
状态：第二版实现与验收完成；2026-07-29 审阅后修正数据角色和位姿时间戳身份

## 1. 阶段目标

`[S]` 本阶段只为后续少量预标注自动选择信息互补的开发期候选帧。输出是
`review candidate`，不是 box motion label，不是 dynamic-object GT，也不用于
选择 geometry 动态阈值。

因为候选选择本身使用了 geometry inconsistency proxy，所选子集不是独立、
无偏的 hold-out evaluation set，不能用于估计整段序列的 precision/recall、
FPR 或 generalization。

每条 moving-box 序列最多选择 24 帧，分为四个代理层：

```text
proxy_low_inconsistency       6
proxy_high_inconsistency      6
proxy_transition              6
proxy_geometry_difficult      6
```

这些名称禁止简写成 `static/moving/transition/occlusion GT`。

## 2. 事实与方法边界

`[L]` Bonn 官方页面说明 depth 已注册到 RGB，并提供 OptiTrack camera-pose
ground truth。

`[A]` 本工具复用 DT-SLAM 已验证的 depth reprojection 符号和坐标约定，但只在
scale-4 采样网格上使用 GT camera pose 计算选帧代理。它不是 ReFusion、
DetectFusion 或 SInDSLAM 的方法复现。

`[S]` 使用 G2-4B 已通过的统一 undistorted pinhole 域：

```text
raw RGB/depth -> official K,D rectification -> P=K
GT text pose  -> Twc
pair transform-> Tcw(current) * Twc(previous)
```

RGB 和 depth association 的时间戳并不完全相同。工具必须显式记录：

```text
--pose-timestamp-source depth
--pose-timestamp-source rgb
```

- `depth`：用 depth timestamp 插值 GT pose，更接近深度 warp 的物理采集时刻；
- `rgb`：用 RGB timestamp，复现当前 SLAM frame timestamp 约定；
- 不允许在结果中省略所用模式；
- 默认使用 `depth`，并以双模式敏感性对照评估排序稳定性。

`[H]` camera-motion compensated depth inconsistency 的高低可能帮助覆盖 box
运动、启停和困难观察，但也可能来自遮挡、深度空洞、非 box 人体运动、pose
误差或采样误差。因此只用于排序和抽样，不能作为 box motion 事实。

## 3. 每帧代理字段

对相邻 RGB-D pair 记录：

```text
valid_comparison_ratio
positive_residual_ratio
negative_residual_ratio
inconsistent_residual_ratio
mean_abs_residual_m
rgb_temporal_difference
invalid_depth_ratio
depth_boundary_ratio
camera_translation_m
camera_rotation_deg
transition_score
difficulty_score
pose_timestamp_source
pose_timestamp
```

其中：

- positive residual 保持当前定义：`predicted_depth-current_depth > 0`；
- inconsistent 使用既有 provisional `0.10 m`，只用于选帧 proxy；
- depth boundary 使用既有 `max(0.08 m, 0.025*min(z1,z2))`；
- RGB temporal difference 未做光度或几何补偿，只作为辅助描述；
- transition score 是相邻帧 inconsistency ratio 的绝对变化；
- difficulty score 由 invalid、boundary 和 negative-residual proxy 的固定组合
  构成。

不得根据 box 视觉结果回调这些公式。

## 4. 分层选择

`[S]` 选择顺序固定：

1. `proxy_high_inconsistency`：inconsistent ratio 从高到低；
2. `proxy_transition`：transition score 从高到低；
3. `proxy_geometry_difficult`：difficulty score 从高到低；
4. `proxy_low_inconsistency`：inconsistent ratio 从低到高。

各层不重复使用同一帧。第一版只要求同层帧间至少 10 帧，真实 contact sheet
显示 hold-out low-inconsistency 层仍连续选中多张近乎相同的箱体近景，且不同
层出现相邻帧，因此该版冻结为 diversity 失败，不用于后续审查。

`[S]` 第二版不改变任何 geometry proxy，只增加固定的抽样多样性约束。对
`32x24` rectified grayscale thumbnail 使用 normalized mean absolute
difference。分四轮放宽：

```text
round  same-role gap  global gap  same-role appearance distance
1      20 frames      5 frames    0.050
2      10 frames      3 frames    0.025
3       5 frames      1 frame     0
4       0 frames      0 frames    0
```

只有上一轮无法填满 6 帧时才进入下一轮。该规则只减少用户重复审查，不改变
proxy score、排序公式或未来 dynamic decision。

`[S]` 当前 `moving_nonobstructing_box` 和 `moving_obstructing_box` 都已经被
程序化排序、生成 contact sheet 并人工查看，因此二者统一定义为
development/review sequences。`moving_obstructing_box` 不再称 strict
hold-out。

真正的 strict hold-out 必须另选尚未用于方法、选帧规则或可视化检查的序列。
该序列在 score、threshold 和 motion-label protocol 冻结前不得运行 geometry
排序，也不得查看 geometry-conditioned contact sheet。

## 5. 输出

每条序列生成：

```text
all_frame_metrics.csv
selected_frames.csv
selection_summary.json
selected_contact_sheet.png
```

`selection_summary.json` 必须包含：

```text
selection_conditioned_on_geometry_proxy=true
selection_is_holdout_evaluation=false
suitable_for_unbiased_sequence_metrics=false
pose_timestamp_source=rgb|depth
maximum_rgb_depth_timestamp_delta_ms
```

contact sheet 必须显示序列索引、代理层和主要 proxy 数值。它只用于快速审查，
不能代替原始 640x480 图像或标注。

## 6. 验收与停止条件

必须通过：

- identity pose + identical depth 的 synthetic residual 为零；
- 已知相机平移 synthetic test 的变换方向正确；
- 输出全部来自存在的 association 文件；
- 每条序列不超过 24 帧，每层不超过 6 帧；
- 层间无重复帧；
- 优先满足冻结的时间与外观多样性约束；
- 输出按原序列索引可复现；
- 所有 proxy 与 GT 标签用词严格分开。
- 明确记录 pose timestamp source；
- 明确拒绝 hold-out 和无偏序列指标身份。

若高/低/transition 层的数值分布没有区分，仍保留结果并停止，不通过人工挑选
“更好看”的帧修饰结果。

## 7. 明确禁止

本阶段不允许：

```text
box motion GT claims
strict hold-out claims for either current moving-box sequence
sequence-wide precision/recall estimates from the selected 24 frames
dynamic threshold selection
mvbDynamic/mvpMapPoints mutation
YOLO changes
Optimizer/g2o changes
extra PoseOptimization
G1-F/G1-D unlock
ATE improvement claims
```

## 8. 可复现命令模板

```bash
cd /home/zhu/dynaslam_ws

DTSLAM_G24C_TMP=$(mktemp -d /tmp/dtslam_g2_4c_XXXXXX)
unzip -q BONN/rgbd_bonn_moving_nonobstructing_box.zip -d "$DTSLAM_G24C_TMP"
unzip -q BONN/rgbd_bonn_moving_obstructing_box.zip -d "$DTSLAM_G24C_TMP"

python3 DT-SLAM/tools/make_rgbd_association.py \
  "$DTSLAM_G24C_TMP/rgbd_bonn_moving_nonobstructing_box/rgb.txt" \
  "$DTSLAM_G24C_TMP/rgbd_bonn_moving_nonobstructing_box/depth.txt" \
  <fresh-nonobstructing-association-output> \
  --max-difference-ms 20 \
  --dataset-root \
  "$DTSLAM_G24C_TMP/rgbd_bonn_moving_nonobstructing_box"

python3 DT-SLAM/tools/select_bonn_review_frames.py \
  "$DTSLAM_G24C_TMP/rgbd_bonn_moving_nonobstructing_box" \
  <nonobstructing-association> \
  "$DTSLAM_G24C_TMP/rgbd_bonn_moving_nonobstructing_box/groundtruth.txt" \
  <fresh-output-directory> \
  --sequence-name moving_nonobstructing_box_development_review \
  --scale 4 \
  --per-stratum 6 \
  --max-gt-delta-ms 40 \
  --pose-timestamp-source depth
```

另一条序列替换目录和 sequence name；时间戳敏感性对照将最后一个参数改为
`rgb`。工具拒绝覆盖既有输出，正式复跑必须使用新的输出目录。
