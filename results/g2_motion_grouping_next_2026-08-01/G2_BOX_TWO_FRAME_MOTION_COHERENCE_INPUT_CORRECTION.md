# 两帧运动一致性输入审计：运动状态分层修正

日期：2026-08-01
状态：第一版解释作废；冻结描述性修正

## 1. 第一版问题

初版 SPEC 将所有粗箱框 review 帧一起用于 `coherent_proxy_frame` 计数，但本地
已有、在查看 geometry/flow 前生成的 RGB 时序 motion proxy 表明：

```text
moving_nonobstructing_box 24 帧：
  moving     5
  stationary 19
```

因此“24 帧中多少帧具有一致 residual”混合了运动箱子与静止箱子，不能回答
motion grouping 输入是否存在。该问题是在第一次输出后发现，原始结果完整保留：

```text
nonobstructing_v1_failed_mixed_motion/
obstructing_v1_unlabeled/
```

不得引用初版自动 `interpretation` 作为科学结论。

## 2. 修正范围

修正只将既有 motion proxy 按 `source_frame` 连接到同一批 24 帧：

```text
results/g2_4f1_motion_proxy_2026-07-29/
  v2_bbox_temporal_correction/nonobstructing/
  moving_nonobstructing_box_agent_rgb_temporal_motion_proxy_v1.csv
```

该 proxy 明确记录：

```text
label_source=agent_rgb_temporal_only_v1
is_ground_truth=false
geometry_or_flow_seen=false
```

不重新选帧、不修改 bbox、不查看 residual 后改 motion label、不调 F1 参数。

## 3. 修正后的身份

只报告 moving 与 stationary 两组的描述统计和 tied-rank proxy AUC：

- support；
- residual magnitude；
- direction concentration；
- target/background centroid separation；
- normalized separation；
- dispersion；
- coherent-proxy count。

由于修正是在看过初版混合输出后提出，本轮不再设置或宣称新的通过条件。
结果只能决定是否值得另立最小 motion-hypothesis shadow SPEC。

`moving_obstructing_box` 当前 17 帧没有逐帧冻结的同索引 motion proxy，因此只作
无标签描述，不参与 moving/stationary 可分性判断。
