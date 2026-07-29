# G2-4 初步区分能力与风险代理审计结果

日期：2026-07-29

## 1. 结论

```text
walking person-proxy conditional signal = 存在但不稳定
sitting person-proxy separation         = 不成立
static small-region risk                = 明显
general dynamic/static separability     = 未通过、不可判定
dynamic threshold                       = 未选择
classification output                   = none
G1-F / G1-D                             = 继续锁定
```

现有 G2-3R3 evidence 有观察信号，但不足以形成通用动态判决。主要问题不是
coverage，而是 proxy 身份、support/region-size 混淆和缺少边界/遮挡风险字段。

## 2. 输入

只读使用：

```text
results/g2_3r3_2026-07-29/walking_200_region_evidence.csv
results/g2_3r3_2026-07-29/sitting_200_region_evidence.csv
results/g2_3r3_2026-07-29/fr1_xyz_200_region_evidence.csv
```

审计工具：

```text
DT-SLAM/tools/audit_dynamic_static_separability.py
```

完整 JSON：

```text
results/g2_4_2026-07-29/g2_4_preliminary_separability_audit.json
```

## 3. Proxy 分布

| 序列 | semantic-vs-background proxy AUC | eligible within-frame contrasts | delta P10/P50/P90 | semantic score > background 的 frame ratio |
| --- | ---: | ---: | ---: | ---: |
| walking | 0.557 | 161 | -0.114 / 0.166 / 0.687 | 69.6% |
| sitting | 0.444 | 186 | -0.164 / -0.117 / -0.064 | 3.2% |
| fr1/xyz | 0.545 | 34 | -0.066 / 0.143 / 0.517 | 61.8% |

这些是 semantic proxy 对照，不是 motion AUC。

walking 中人物 proxy 的 comparison-vote-weighted positive ratio 为 `0.462`，
nonsemantic background proxy 为 `0.145`，说明存在 conditional signal；但
region-level AUC 接近随机且约 30% eligible frames 没有正向 contrast。

sitting 中人物 proxy weighted ratio 为 `0.071`，background proxy 为 `0.188`。
这与 sitting 人体不等于动态 GT 的已知限制一致，也说明“person region score 高”
不能作为通用规则。

## 4. 静态风险代理

fr1/xyz nonsemantic background proxy：

| descriptive score | region exceedance | comparison-vote mass |
| ---: | ---: | ---: |
| 0.05 | 39.69% | 29.52% |
| 0.10 | 32.96% | 16.10% |
| 0.20 | 24.82% | 1.17% |
| 0.30 | 19.76% | 0.57% |
| 0.50 | 15.27% | 0.29% |
| 0.75 | 10.03% | 0.13% |
| 1.00 | 5.08% | 0.01% |

高 score region 数量不少，但其 comparison-vote mass 很小，表明许多高分来自
小区域或低支持量。它是候选静态误判风险，不是 measured FPR。

示例：

```text
fr1/xyz, region 1-64 pixels, comparison 1-4:
weighted positive ratio = 0.266, unweighted P90 = 1.0

fr1/xyz, region >=256 pixels, comparison >=20:
weighted positive ratio = 0.040, unweighted P90 = 0.258
```

因此任何 region score 都必须同时审计支持量；但本结果不据此创建面积或
comparison threshold。

## 5. 与 G2-3R1 的差异

G2-3R3 已显著提高 comparison coverage，因此“没有 evidence”不再是唯一瓶颈。
新的主要瓶颈是：

- proxy 不是真值；
- region rows 在 frame 内相关；
- 小区域/低支持产生极端 score；
- positive evidence 的 boundary/occlusion 来源没有记录；
- 没有 unknown moving-box motion label。

## 6. 冻结判断

```text
coverage high                      != detection correct
walking conditional proxy signal  != general motion separation
static score exceedance            != measured static FPR
proxy AUC                          != motion AUC
```

当前不进入 G2-4B 阈值设计。下一小步只允许补充 boundary、invalid/occlusion、
reference support 和 pose-risk instrumentation，并同时制定低人工负担的
static/moving-box evaluation protocol。
