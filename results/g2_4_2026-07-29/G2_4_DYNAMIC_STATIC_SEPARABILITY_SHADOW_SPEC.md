# G2-4 动态/静态区分能力与风险代理 Shadow 规格

日期：2026-07-29
状态：G2-4A 测量充分性规格；不包含动态判决阈值

## 1. 阶段位置

```text
G0 geometry correctness               = 基本完成
G2-1...G2-3 evidence representation   = 基本完成
G2-3R4 low-resolution optimization    = 收益门失败，停止
G2-4 separability/risk gate            = 当前阶段
G1-F tracking filtering               = 锁定
G1-D dense filtering                  = 锁定
```

G2-4A 只回答：

> 现有 evidence、数据和风险字段是否足以设计并公正评价一个动态判决？

本阶段不回答“最佳阈值是多少”。

## 2. 冻结输入

第一轮复用 G2-3R3 full-resolution reference partition 上的：

```text
comparison/positive/negative/consistent pixels
comparison/positive/negative/consistent votes
region pixels
semantic-proxy pixels
region positive vote ratio
comparison coverage
```

G2-3R4 candidate 默认关闭，不继续优化。

## 3. 当前标签身份

```text
walking person mask  = semantic proxy，不是 motion GT
sitting person mask  = semantic proxy，包含静止与局部运动
fr1/xyz               = static-background risk proxy，不是逐像素 GT
Bonn camera pose      = trajectory GT，不是 motion mask GT
```

禁止把 proxy AUC 写成 motion-detection AUC。

## 4. G2-4A 只读指标

按三个固定 region-size 档：

```text
1-64, 65-255, >=256 pixels
```

和四个 comparison-support 档：

```text
0, 1-4, 5-19, >=20 compared pixels
```

报告：

- region score 的 P10/P25/P50/P75/P90/P95；
- comparison-vote-weighted positive ratio；
- semantic-dominant、mixed、nonsemantic proxy 分布；
- 同一 frame 内 semantic-dominant 与 background proxy 的 weighted-score 差；
- proxy rank AUC，并明确不是 motion AUC；
- fr1/xyz background score exceedance curve；
- exceedance region ratio 与 comparison-vote mass ratio；
- vote conservation 和字段完整性。

预定义 score grid 仅用于画风险曲线：

```text
0, 0.05, 0.10, 0.20, 0.30, 0.50, 0.75, 1.0
```

它不是候选阈值搜索。

## 5. 必须补齐的混淆字段

在设计动态分数前，需要一次独立、默认关闭的 instrumentation 小步骤：

- positive evidence 到 current depth boundary 的距离；
- invalid-depth/occlusion 邻域标记；
- 每个 region 的 boundary-adjacent positive mass；
- 每个 pixel/region 的有效 reference support；
- frame pose/tracking 质量代理；
- ORB feature 所在 region 及其 evidence；
- mask age/readiness。

目的只是区分：

```text
motion evidence
vs depth boundary / disocclusion / invalid depth / pose-error evidence
```

不得在该 instrumentation 步骤中创建动态标签。

## 6. 标注与数据策略

优先级：

1. 自动使用现有 semantic mask、GT pose、depth 和 static sequence；
2. 使用 Bonn static 与 moving-box 序列做未知类别压力测试；
3. 先完成 Bonn 非零畸变坐标域审计；
4. 若必须获得 pixel motion GT，由 Agent 自动选取少量分层帧并生成预标注；
5. 用户只修正困难边界，不承担从零大量标注。

Bonn 官方未提供逐帧 dynamic mask 的事实必须写入数据卡。

## 7. Calibration 与 hold-out

现有 TUM 三序列已经用于探索，只能作为 exploratory/calibration 候选，不能再
作为唯一最终测试集。

未来判决阶段必须：

- 先定义 calibration sequences；
- 单独保留 hold-out static 与 moving-box sequences；
- 只在 calibration 上选择分数和阈值；
- 参数冻结后只运行一次 hold-out 主结果；
- 不因 hold-out 结果反复回调阈值。

## 8. G2-4A 充分性门

在进入 G2-4B 判决设计前必须同时满足：

```text
evidence 字段和 vote conservation 可复核
边界/invalid/occlusion/pose-risk 字段已记录
至少一个真正静态 hold-out 风险序列
至少一个 unknown moving-box hold-out 序列
motion-label 或明确的 label-free 评价协议已经冻结
calibration/hold-out 无重叠
```

任一项缺失：

```text
dynamic threshold selection = 禁止
G1-F                        = 继续锁定
```

## 9. G2-4B 后续但本轮不实施

只有 G2-4A 通过后，另写 G2-4B SPEC，预冻结：

- 判决单位：pixel、ORB feature 或 region；
- 单一最小可解释 score；
- precision/recall、static-risk 和 unknown-object gates；
- calibration 阈值；
- hold-out protocol；
- 失败停止条件。

不允许从当前 exploratory AUC 直接选择阈值。

## 10. 禁止修改

G2-4A 不修改：

```text
mvbDynamic
mvpMapPoints filtering
YOLO
Optimizer/g2o
PoseOptimization
LocalMapping
LoopClosing
```

所有输出保持：

```text
classification_output=none
dynamic_threshold_selected=false
shadow-only
```
