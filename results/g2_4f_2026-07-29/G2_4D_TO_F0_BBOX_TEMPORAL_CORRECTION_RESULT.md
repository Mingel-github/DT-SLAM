# G2-4D 至 G2-4F0 Box 粗框时序复核修正结果

日期：2026-07-29
状态：完成；旧产物保留，新产物为当前有效 development 审计。
范围：shadow-only；不产生动态判决，不修改 SLAM 状态。

## 1. 为什么修正

在准备独立 RGB 时序运动代理时，逐帧查看原始、已校正 RGB，发现
`moving_nonobstructing_box` 的三个 v1 粗框没有覆盖目标箱体：

| frame | v1 问题 | 修正后的粗框 `(x,y,w,h)` | visibility |
| ---: | --- | --- | --- |
| 246 | 覆盖桌面后的背景板 | `(88,153,180,120)` | visible |
| 478 | 覆盖箱体右侧空地面 | `(145,272,125,192)` | visible |
| 537 | 覆盖箱体右侧空地面 | `(140,288,145,192)` | partial |

仅修改上述三行。其 `annotation_source` 为
`agent_rgb_only_coarse_bbox_v2_temporal_correction`，并保留其余 45 行原有
来源和 `unverified` 状态。修正后的框仍是 Agent RGB-only 粗框，不是 pixel
ground truth，也不是 motion ground truth。

## 2. 重算范围

没有重新运行 YOLO 或 SLAM。复用已有在线输出，只重算依赖 bbox 的连接：

1. G2-4D person-mask/box 覆盖；
2. G2-4E exact depth partition 与 box-region intersection；
3. G2-4F0 ORB feature inside/outside evidence；
4. RGB-only 五帧时序审阅图。

所有历史目录均保留。当前修正输出：

```text
results/g2_4d_2026-07-29/
  semantic_box_coverage_review_v3_bbox_temporal_correction/

results/g2_4e_2026-07-29/
  development_partition_v2_bbox_temporal_correction/
  development_audit_inputs_v2_bbox_temporal_correction.json
  development_evidence_audit_v2_bbox_temporal_correction/

results/g2_4f_2026-07-29/
  development_feature_audit_v2_bbox_temporal_correction/

results/g2_4f1_motion_proxy_2026-07-29/
  v2_bbox_temporal_correction/
```

## 3. 一致性检查

```text
semantic coverage candidates       = 48/48
temporally corrected bbox rows     = 3
G2-4E online/offline exact match   = 42/42
G2-4E partition mismatch           = 0
G2-4F0 feature frames              = 42
RGB temporal center exact match    = 48/48
dynamic_decision                   = none
direct_slam_state_mutation         = none
```

早期缺失的 6 帧仍是 `no evidence`，不是静态。

## 4. 对 G2-4E 的影响

nonobstructing、无人物可见箱体 18 帧中：

| 指标 | v1 | 修正后 |
| --- | ---: | ---: |
| 主导区域 positive/comparison 均值 | 3.126% | 3.126% |
| 主导区域 positive/comparison 中位数 | 2.227% | 2.227% |
| 主导区域 bbox coverage 均值 | 93.803% | 92.533% |
| bbox 相交区域数均值 | 1.333 | 1.500 |

正证据统计不变，是因为这些粗框仍主要命中同一个延伸到背景的大区域；拓扑覆盖
统计发生变化。这反而再次说明当前主导 depth region 不是可靠的目标实例。
G2-4E 的负门控不变。

## 5. 对 G2-4F0 的影响

nonobstructing、无人物可见箱体 18 帧：

| 指标 | v1 均值 / 中位数 | 修正后均值 / 中位数 |
| --- | ---: | ---: |
| 框内 comparison coverage | 96.266% / 96.992% | 95.521% / 95.801% |
| 框内 positive presence | 5.402% / 1.632% | 7.866% / 1.746% |
| 框外 positive presence | 9.638% / 6.703% | 9.072% / 6.703% |
| presence enrichment | 0.479× / 0.267× | 0.646× / 0.289× |
| 框内 positive vote | 1.707% / 0.482% | 3.096% / 0.545% |
| 框外 positive vote | 4.185% / 2.697% | 3.870% / 2.697% |
| vote enrichment | 0.379× / 0.156× | 0.638× / 0.205× |

修正提高了均值，主要因为 frame 478/537 的真实箱体位置具有较高正证据；但
RGB 时序审阅将这两个五帧窗口标为 `stationary/high` 代理。这些响应因此不能
直接当作动态检出成功。

修正后：

```text
presence enrichment > 1 : 3/18
vote enrichment > 1     : 4/18
框内无 positive feature : 1/18
```

总体中位数仍低于背景，且静止代理窗口可出现强响应，所以 G2-4F0 仍未通过
动态判决门。

obstructing 的粗框没有修改，其全部数值不变。

## 6. 决策

```text
G2-4D bbox audit reliability       = 已纠正三个已知错误，仍非 GT
G2-4E region decision readiness    = 未通过
G2-4F0 feature decision readiness  = 未通过
G1-F / G1-D                        = locked
strict hold-out                    = sealed and unopened
```

本次修正说明：错误粗框会显著改变均值，必须保留审计来源和版本；但修正没有
推翻“当前 depth-region 和 direct depth feature evidence 尚不足以形成可靠动态
判决”的结论。
