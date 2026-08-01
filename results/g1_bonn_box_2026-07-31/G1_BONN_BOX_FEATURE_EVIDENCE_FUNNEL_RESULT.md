# G1 Bonn 箱子特征证据漏斗审计结果

日期：2026-07-31
状态：完成；当前 q10 相邻帧 ego-flow 未在粗箱框内产生候选

## 1. 运行

使用与正式组合模式完全相同的：

```text
778 RGB-D associations
Bonn joint P=K rectification
online CUDA YOLO, mask age 0
G1-F1/G1-M1 q10/5% safeguards
Viewer OFF
```

只在此前 24 个粗箱框 review 帧记录逐特征证据。运行完整 778/778，实际
`29.501 FPS`。新增字段和 candidate-association CSV 仅记录既有 selector 与
filter 状态，不改变候选或 SLAM 行为。

## 2. C++ 连接不变量

逐特征、post-search candidate 和 exact removal 与逐帧 aggregate CSV 全部一致：

```text
all post-search candidate associations    543
all exact removed associations             526
duplicate frame/feature keys                 0
selected-frame aggregate mismatches          0
candidate removed flags vs exact removals     0
```

## 3. 箱框内证据漏斗

| 阶段 | 24 帧全部 | 粗箱框内 | 框内占比 |
|---|---:|---:|---:|
| ORB features | 24,087 | 5,022 | 20.85% |
| measured sparse ego-flow | 22,037 | 4,668 | 21.18% |
| quality eligible | 20,940 | 4,564 | 21.80% |
| frozen q10 candidate | 47 | **0** | 0% |
| q10 candidate with initial association | 1 | **0** | 0% |
| q10 post-SearchLocalPoints association | 10 | **0** | 0% |
| actually removed association | 9 | **0** | 0% |

因此证据不是在 ORB、LK/depth 质量或 MapPoint association 层消失，而是在
`quality eligible → q10 candidate` 层消失。

## 4. q 分布

以下只统计 semantic-static 且 quality-eligible 的特征：

| 区域 | n | median | p95 | p99 | max | q>=3 | q>=5 | q>=10 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 粗箱框内 | 4,555 | 0.571 | 2.779 | 3.422 | 4.934 | 145 | 0 | 0 |
| 粗箱框外 | 16,343 | 0.698 | 2.108 | 5.871 | 30.139 | 462 | 239 | 47 |

`q>=3/5` 只是事后描述，没有改变冻结阈值。箱框约占 eligible 特征的 21.8%；
在 `q>=3` 的 607 点中箱框内占约 23.9%，只有很弱的富集，而 `q>=5` 后箱框内
已经为零。直接降低单点阈值不会自然得到箱子特异性，反而会引入大量框外点。

## 5. 解释边界

事实：

- 箱框内有大量可测 ORB 特征；
- 当前 q10 没有选择其中任何一个；
- 当前运行被过滤的 review 点来自箱框外；
- 不是“箱子低纹理导致完全无特征”；
- 不是“候选存在但没有 MapPoint association”。

合理但尚未被证明的解释：相邻帧中箱子运动相对于相机诱导流不够大或不够稳定，
而深度边界、遮挡、LK 误差或背景不一致产生了更极端的框外 residual。粗箱框和
24 帧不是运动 GT，因此不能进一步声称所有箱子运动时刻都不可检测。

## 6. 决策

```text
lower q threshold                         rejected
retune 5% safeguards                      rejected
claim current q10 protects this box       rejected
G1-D                                      remains locked
current q10 filter                        limited experimental baseline
```

`moving_obstructing_box` 仍有一次开发诊断价值：它能测试更强遮挡/运动是否显著
提高箱子 residual。该检查必须继续使用 q10，不做三轮 ATE；先看候选空间位置。
若该序列仍主要选择背景，则停止当前相邻帧单点 q 路线，回到已有文献审计中的
运动一致性分组/对象候选，而不是继续调阈值或追加 Bonn 序列。

该强遮挡诊断已经完成。最终结论见：

- `G1_BONN_BOX_EVIDENCE_FUNNEL_AND_OBSTRUCTING_RESULT.md`

原始输出：

- `moving_nonobstructing_box/evidence_funnel_run/sparse_flow_features.csv`
- `moving_nonobstructing_box/evidence_funnel_run/candidate_associations.csv`
- `moving_nonobstructing_box/evidence_funnel_run/removed_associations.csv`
- `moving_nonobstructing_box/evidence_funnel_audit/funnel_summary.json`
- `moving_nonobstructing_box/evidence_funnel_audit/funnel_contact_sheet.png`
