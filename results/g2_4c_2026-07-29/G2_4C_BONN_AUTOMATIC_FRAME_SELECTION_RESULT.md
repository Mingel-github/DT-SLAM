# G2-4C Bonn Moving-Box 自动候选选帧结果

日期：2026-07-29
状态：开发期候选选帧通过；2026-07-29 审阅后已修正 hold-out 和时间戳身份；
box 预标注与 semantic coverage 尚未执行

## 1. 输出边界

`[S]` 输出是 geometry-risk-stratified development/review candidates，不是
box motion GT，不是 dynamic classification，也没有参与 SLAM filtering。
所选子集受 geometry proxy 条件化，不能用于估计完整序列的无偏检测指标。

工具：

```text
DT-SLAM/tools/select_bonn_review_frames.py
```

输入使用 G2-4B 已通过的 `P=K` undistorted pinhole RGB-D 域和 Bonn GT camera
pose。scale-4 depth warp 只用于候选排序。

## 2. 自测

```text
python3 DT-SLAM/tools/select_bonn_review_frames.py --self-test
[G2-4C Selection Self-Test] PASS
```

覆盖：

- identity pose + identical plane 的 residual 为零；
- 已知相机前向平移的 transform direction 正确。

## 3. 第一版失败及保留

第一版满足每层 6 帧和全局不重复，但只约束同层 frame gap。contact sheet
显示：

- hold-out low-inconsistency 层连续选中多张近乎相同的 box close-up；
- high/transition 层出现相邻帧；
- 这会增加用户重复审查，不能算自动选帧通过。

失败结果保留在：

```text
results/g2_4c_2026-07-29/first_pass_failed_diversity/
```

没有修改任何 geometry proxy 或排序分数。第二版只加入实现前已记录的 temporal
与 thumbnail appearance diversity 规则。

## 4. 第二版结果

### 4.1 Development/review A：moving_nonobstructing_box

```text
association frames       = 778
eligible frames          = 772
selected frames          = 24
selected unique frames   = 24
global minimum frame gap = 5
```

eligible inconsistency distribution：

```text
min/median/P90/max = 0.00166 / 0.02811 / 0.06543 / 0.21858
```

每层 6 帧：

```text
proxy_high_inconsistency:
  297, 725, 770, 15, 600, 56

proxy_transition:
  730, 761, 314, 8, 385, 364

proxy_geometry_difficult:
  747, 88, 180, 110, 692, 307

proxy_low_inconsistency:
  494, 246, 669, 571, 754, 777
```

同层 minimum pairwise gap 为 `21–41` 帧，minimum thumbnail MAD 为
`0.052–0.132`。

### 4.2 Development/review B：moving_obstructing_box

```text
association frames       = 589
eligible frames          = 587
selected frames          = 24
selected unique frames   = 24
global minimum frame gap = 6
```

eligible inconsistency distribution：

```text
min/median/P90/max = 0.00000 / 0.03261 / 0.09070 / 0.21343
```

每层 6 帧：

```text
proxy_high_inconsistency:
  348, 133, 86, 35, 158, 324

proxy_transition:
  450, 43, 547, 362, 67, 123

proxy_geometry_difficult:
  79, 139, 110, 489, 466, 513

proxy_low_inconsistency:
  199, 219, 566, 239, 6, 587
```

同层 minimum pairwise gap 为 `20–24` 帧，minimum thumbnail MAD 为
`0.069–0.185`。

## 5. 自动输出

本节原始第二版使用 RGB timestamp 插值 GT pose，作为历史 pipeline-matched
结果保留。每条序列均保存：

```text
all_frame_metrics.csv
selected_frames.csv
selection_summary.json
selected_contact_sheet.png
```

位置：

```text
results/g2_4c_2026-07-29/moving_nonobstructing_box/
results/g2_4c_2026-07-29/moving_obstructing_box/
```

程序化验收：

```text
role count             = 6 x 4 for both sequences
duplicate frame        = 0
selection_is_motion_gt = false
dynamic_decision       = false
self-test              = PASS
```

## 6. 视觉复核边界

`[S]` contact sheet 显示第二版减少了连续近重复帧，并覆盖：

- person present/absent；
- box near/far；
- box 部分出入画面；
-不同 camera viewpoints；
- depth/occlusion 风险较高的观察。

这些只是选帧多样性观察，不能据此写成实际 box static/moving/transition 标签。
low-inconsistency 中也存在 box close-up，高 inconsistency 也可能来自 person、
occlusion 或 depth error。

## 7. 原始第二版冻结结论的修正

```text
G2-4C development review selection  = 通过
strict hold-out evaluation          = 不成立
box motion labels                   = 不存在
box pre-annotation                  = 未开始
semantic box coverage               = 未审计
dynamic threshold                   = 未选择
G1-F / G1-D                         = 继续锁定
```

当前两条序列均已按 geometry proxy 排序、生成 contact sheet 并被查看；第一次
diversity 失败还使用了 `moving_obstructing_box` 的联系表调整抽样规则。因此
后者不再称 strict hold-out，只保留 development/review 身份。

## 8. RGB-time 与 depth-time 敏感性修正

原工具用 RGB timestamp 插值 GT pose，但 residual 来自 depth observation。
association 中最大 RGB-depth 时间差分别为：

```text
nonobstructing = 16.629934 ms
obstructing    = 16.669989 ms
```

修正后的工具显式支持：

```text
--pose-timestamp-source rgb
--pose-timestamp-source depth
```

并在 JSON 中写入：

```text
selection_conditioned_on_geometry_proxy=true
selection_is_holdout_evaluation=false
suitable_for_unbiased_sequence_metrics=false
pose_timestamp_source=...
```

四组重跑均成功。RGB-time 完整复现了原第二版候选；depth-time 结果为：

### 8.1 Nonobstructing depth-time

```text
frames/eligible/selected/unique = 778 / 773 / 24 / 24
inconsistency min/median/P90/max =
  0.00216 / 0.02823 / 0.06658 / 0.23116

proxy_high_inconsistency:
  297, 600, 724, 16, 643, 478

proxy_transition:
  313, 730, 761, 364, 385, 537

proxy_geometry_difficult:
  306, 88, 770, 747, 110, 691

proxy_low_inconsistency:
  246, 571, 754, 777, 631, 202
```

### 8.2 Obstructing depth-time

```text
frames/eligible/selected/unique = 589 / 587 / 24 / 24
inconsistency min/median/P90/max =
  0.00000 / 0.03087 / 0.09221 / 0.26728

proxy_high_inconsistency:
  133, 86, 359, 40, 158, 65

proxy_transition:
  79, 350, 46, 124, 448, 25

proxy_geometry_difficult:
  91, 138, 111, 325, 35, 489

proxy_low_inconsistency:
  199, 219, 566, 239, 6, 587
```

### 8.3 敏感性

| Development sequence | Per-frame inconsistency Pearson | Median absolute delta | P95 absolute delta | Selected overlap | Jaccard |
| --- | ---: | ---: | ---: | ---: | ---: |
| nonobstructing | 0.8929 | 0.00205 | 0.02416 | 14/24 | 0.4118 |
| obstructing | 0.9267 | 0.00149 | 0.02446 | 12/24 | 0.3333 |

整体 residual proxy 高相关，但 top-ranked 候选集合对时间戳选择明显敏感，尤其
obstructing 的 transition 层同角色重叠为 `0/6`。因此不能静默继续使用 RGB
timestamp。

冻结决定：

- depth warp 的开发期审查默认使用 `depth` timestamp；
- `rgb` 结果保留为当前 SLAM frame-time 约定的敏感性对照；
- 两套结果都不是 motion GT 或 hold-out；
- 后续 48 帧 person-mask/box pre-annotation review 使用 depth-time 候选；
- 最终 sequence-wide 指标必须使用独立采样或完整序列，不使用该条件化子集。

修正结果：

```text
results/g2_4c_correction_2026-07-29/
  inputs/
  nonobstructing_rgb_time/
  nonobstructing_depth_time/
  obstructing_rgb_time/
  obstructing_depth_time/
```

## 9. 实际重跑命令

从两个本地 archive 解压到独立临时目录后，使用：

```bash
python3 DT-SLAM/tools/make_rgbd_association.py \
  <dataset>/rgb.txt <dataset>/depth.txt <association-output> \
  --max-difference-ms 20 --dataset-root <dataset>

python3 DT-SLAM/tools/select_bonn_review_frames.py \
  <dataset> <association> <dataset>/groundtruth.txt <fresh-output-dir> \
  --sequence-name <development-review-name> \
  --scale 4 --per-stratum 6 --max-gt-delta-ms 40 \
  --pose-timestamp-source depth
```

将最后参数改成 `rgb` 即得到敏感性对照。完整解压与参数模板见同阶段 SPEC。

## 10. 下一小步

不修改 YOLO 地导出修正后 48 个 depth-time 候选帧的：

1. 实际 C++ person detection；
2. 实际进入语义过滤的、实例合并且膨胀后的 person union mask；
3. 与独立 box 预标注的 semantic coverage。

box 预标注不得由待评价的 positive residual、region score 或 G2-4C 排名生成。
当前 C++ semantic 只筛选 COCO class 0 person，因此 box 不会被类别直接识别；
仍需实测 person mask 膨胀、接触和遮挡造成的间接 box coverage。
