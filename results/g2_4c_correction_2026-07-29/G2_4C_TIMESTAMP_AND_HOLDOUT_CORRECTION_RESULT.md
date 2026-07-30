# G2-4C 时间戳与 Hold-out 身份修正结果

日期：2026-07-29

## 结论

```text
原 RGB-time 结果复现             = 通过
depth-time sensitivity rerun     = 通过
两个当前 moving-box 序列身份      = development/review
strict hold-out                  = 尚未选择
selected subset unbiased metric = 禁止
dynamic decision                = none
G1-F / G1-D                     = locked
```

选帧受 geometry inconsistency proxy 条件化，且两条序列及其联系表均已被查看。
`moving_obstructing_box` 还参与了 diversity 规则修正。因此它不再是严格
hold-out。

## 输入重建

本地 archive：

```text
BONN/rgbd_bonn_moving_nonobstructing_box.zip
BONN/rgbd_bonn_moving_obstructing_box.zip
```

重新解压后使用 existing-file-aware 一对一 20 ms association：

| Sequence | Pairs | Unmatched RGB | Missing depth | Max RGB-depth delta |
| --- | ---: | ---: | ---: | ---: |
| nonobstructing | 778 | 0 | 4 | 16.629934 ms |
| obstructing | 589 | 1 | 3 | 16.669989 ms |

association 保存于：

```text
results/g2_4c_correction_2026-07-29/inputs/
```

## 双时间戳重跑

固定：

```text
scale=4
per_stratum=6
max_gt_delta_ms=40
coordinate_domain=undistorted_pinhole_P_equals_K
```

只改变：

```text
pose_timestamp_source=rgb
pose_timestamp_source=depth
```

工具输出明确写入：

```text
selection_conditioned_on_geometry_proxy=true
selection_is_holdout_evaluation=false
suitable_for_unbiased_sequence_metrics=false
```

## 结果

| Sequence | Common eligible | Pearson | Median absolute delta | P95 absolute delta | Selected overlap | Jaccard |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| nonobstructing | 772 | 0.8929 | 0.00205 | 0.02416 | 14/24 | 0.4118 |
| obstructing | 587 | 0.9267 | 0.00149 | 0.02446 | 12/24 | 0.3333 |

per-frame inconsistency 整体相关，但 top-ranked 候选并不稳定。obstructing 的
`proxy_transition` 同角色候选重叠为 `0/6`。

因此：

- 不能省略 pose timestamp identity；
- depth-warp review 默认使用 depth timestamp；
- RGB timestamp 只保留为 pipeline-matched sensitivity 对照；
- 该改动不产生 box motion GT，也不选择动态阈值。

四组原始输出：

```text
results/g2_4c_correction_2026-07-29/nonobstructing_rgb_time/
results/g2_4c_correction_2026-07-29/nonobstructing_depth_time/
results/g2_4c_correction_2026-07-29/obstructing_rgb_time/
results/g2_4c_correction_2026-07-29/obstructing_depth_time/
```

## 后续评价限制

修正后的 48 个 depth-time 候选只用于：

- person semantic-mask coverage review；
- 独立 box 预标注的低人工负担检查；
- 风险与失败模式分析。

禁止用于：

- 完整序列 precision/recall/FPR 的无偏估计；
- 最终 unknown-object generalization 主表；
- 动态阈值选择后的 strict hold-out 声明。

真正的 hold-out 必须是一个尚未运行 geometry proxy、未生成联系表、未参与规则
调整的独立序列。
