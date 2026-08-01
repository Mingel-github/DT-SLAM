# G1 Bonn 箱子证据漏斗与强遮挡诊断结果

日期：2026-07-31
状态：完成；冻结现有相邻帧单点 `q10` 路线的能力边界

## 1. 本轮问题

本轮不重新设计检测器，也不调 `q=10` 或 5% 安全限制，只回答：

```text
ORB feature
→ sparse ego-flow measured
→ quality eligible
→ frozen q10 candidate
→ SearchLocalPoints 后有 MapPoint association
→ 实际被 G1-F1 移除
```

所有新增 CSV 都是默认关闭的只读诊断；没有修改 YOLO、Optimizer、g2o、
LocalMapping，也没有增加位姿优化。

## 2. 非遮挡运动箱子

在此前 24 个粗略 RGB-only 箱框中：

| 阶段 | 全部 | 箱框内 |
|---|---:|---:|
| ORB feature | 24,087 | 5,022 |
| measured | 22,037 | 4,668 |
| quality eligible | 20,940 | 4,564 |
| q10 candidate | 47 | **0** |
| post-search candidate association | 10 | **0** |
| actually removed association | 9 | **0** |

箱框内并不缺 ORB、深度或 LK 测量；证据在 `quality eligible → q10` 处消失。
箱框内 semantic-static eligible 特征的最大 `q=4.934`，不能通过保持冻结阈值
得到任何候选。事后降低阈值也没有依据：箱框内在 `q>=3` 只有弱富集，而
`q>=5` 已为零。

## 3. 强遮挡运动箱子

自动 proxy 选帧集中在箱子离开后的帧，因此判定该自动选择结果不适合本问题，
未据此评价。随后只依据 RGB 时序选择箱体明显占据画面的 17 帧
（frame 160--320，每 10 帧一张），使用粗框作空间诊断。粗框未逐像素验证，
因此不是 GT。

完整序列运行条件：

```text
589/589 RGB-D pairs
Bonn joint P=K rectification
online CUDA YOLO, mask age 0
same q10 / 5% safeguards
Viewer OFF
actual FPS 29.685
```

证据漏斗：

| 阶段 | 17 帧全部 | 粗箱框内 | 框内占比 |
|---|---:|---:|---:|
| ORB feature | 14,289 | 7,426 | 51.97% |
| measured | 10,353 | 6,422 | 62.03% |
| quality eligible | 8,960 | 5,530 | 61.72% |
| q10 candidate | 223 | 26 | 11.66% |
| initial MapPoint association | 0 | 0 | — |
| post-search MapPoint association | 2 | 2 | 100% |
| actually removed association | 2 | 2 | 100% |

semantic-static、quality-eligible 特征中：

| 区域 | n | median q | p95 | p99 | q>=10 | q10 比率 |
|---|---:|---:|---:|---:|---:|---:|
| 箱框内 | 5,512 | 0.629 | 2.309 | 5.902 | 26 | 0.472% |
| 箱框外 | 3,346 | 0.724 | 11.431 | 15.914 | 197 | 5.888% |

框外 q10 比率约为框内的 **12.48 倍**。联系表也显示大量 raw q10 点位于
背景、人物或遮挡/深度边界附近。该现象支持“单点高残差缺少对象特异性”，但
粗框不足以证明每个框外点都是假阳性。

另一方面，`SearchLocalPoints()` 后只留下 2 个带 MapPoint 的候选，它们都在
粗箱框内并被移除。这是当前路线确实触及未知箱子的正面证据，但数量只有 2，
不能外推为稳定保护，也不能把 MapPoint association 描述成经过验证的对象
判别器。

## 4. Mapping 边界

frame 160--320 内，G1-M1 在 7 个 KeyFrame 事件中应用了 15 个 q10 候选写图
否决。现有 aggregate CSV 没有这些 feature 的精确像素坐标，因此无法确认它们
是否位于箱子。鉴于 raw q10 在框外更密集，本轮不能声称 G1-M1 已经清理了箱子
MapPoint；它仍只是确认机制生效的实验保护层。

## 5. 决策

```text
current q10 residual as motion evidence        retained
current q10 as validated object detector       rejected
lower q / retune 5%                            rejected
three-run obstructing-box ATE at this point    not justified
G1-F1/G1-M1                                    experimental, default OFF
G1-D dense depth-region filtering              remains locked
```

不做强遮挡序列三轮 ATE 的理由不是回避负结果，而是 17 个 review 帧中只有 2 个
tracking association 真正改变；轨迹差异很容易被 ORB-SLAM2 运行波动淹没，且
不能回答 box specificity。

当前最准确的结论是：

> 相邻帧 sparse ego-flow residual 能在强运动/遮挡下提供少量类别无关箱子证据，
> 但 raw 单点候选主要落在箱框外，尚不能成为可靠的未知动态对象判决。

因此，下一方法阶段应回到已有文献依据的“运动证据分组/对象候选”问题，研究
空间与短时序一致性如何抑制孤立边界残差；不得继续用单点阈值或 flood fill
修补。开始实现前仍需先核对本地 PaperNotes/PDF，并单独冻结小型 shadow SPEC。

## 6. 原始证据

- `moving_nonobstructing_box/evidence_funnel_run/`
- `moving_nonobstructing_box/evidence_funnel_audit/funnel_summary.json`
- `moving_nonobstructing_box/evidence_funnel_audit/funnel_contact_sheet.png`
- `moving_obstructing_box/evidence_funnel_run/`
- `moving_obstructing_box/evidence_funnel_audit/funnel_summary.json`
- `moving_obstructing_box/evidence_funnel_audit/funnel_contact_sheet.png`
- `moving_obstructing_box/target_box_bbox_review_proxy.csv`
- `moving_obstructing_box/automatic_selection/`（不适合本问题的负面选帧结果）
