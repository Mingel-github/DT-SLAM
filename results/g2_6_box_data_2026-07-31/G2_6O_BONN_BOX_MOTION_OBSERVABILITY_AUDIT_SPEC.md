# G2-6O Bonn 箱子运动可观测性审计 SPEC

日期：2026-07-31
状态：冻结后执行
身份：`[S]` 评价数据审计，不是检测算法，不是运动真值

## 1. 目的

G2-6E 已证明 Bonn scanner static model 不能直接生成可信的逐像素
unknown-motion proxy。G2-6O 不修补该失败路线，而只回答一个更基础的问题：

> 新增的 Bonn development 序列中，哪些短时间窗口通过 RGB 时序可以明确看见
> 箱子静止、箱子运动或状态转换，并且人物是否会混淆评价？

只有先确认运动在图像中可见，后续 sparse ego-flow 或 motion grouping 的失败
才具有解释力。

## 2. 输入

仅允许读取：

- 官方 Bonn development archive；
- 对应 RGB-D association；
- 相机标定（只用于 RGB 去畸变显示）；
- RGB 图像时序。

禁止读取：

- geometry、depth residual、F1/F2/F3 score；
- SLAM tracking 结果或 ATE；
- 由待评价方法产生的候选；
- sealed `rgbd_bonn_balloon_tracking` hold-out。

序列名 `placing`、`removing`、`kidnapping` 只描述数据集主题，不作为逐帧标签。

## 3. 候选采样

为减少漏掉缓慢运动，同时避免只挑 RGB 变化最大的片段，候选由三组组成：

1. `uniform`：沿完整时序均匀取样；
2. `rgb_change_high`：按低分辨率灰度时序差从高到低取样；
3. `rgb_change_low`：按同一差值从低到高取样。

三组使用固定最小时间间隔去重。RGB 时序差只用于提高审阅覆盖率，是
`proxy_selection_metric`，不是箱子运动分数，也不能用于报告 detector
precision/recall。

每个候选输出固定跨度的 RGB contact clip；中心帧和各偏移帧必须明确标注。

## 4. Agent 粗审标签

由 Agent 仅看 RGB clip，填写：

```text
box_visibility = visible / partial / absent / uncertain
box_motion = moving / stationary / transition / uncertain / not_visible
person_presence = present / absent / uncertain
confidence = high / medium / low
reason = 简短可核查描述
```

这些标签是 development review annotation：

```text
is_ground_truth = false
label_source = agent_rgb_temporal_review_v1
```

模糊、遮挡、相机运动过强或无法确认对象身份时必须标为 `uncertain`，不得猜测。

## 5. 预冻结判据

本阶段不设 detector 通过阈值，只作数据可用性决策：

- 若至少一条序列存在多段 `moving + person absent` 且置信度不低于 medium 的
  窗口，则允许在这些窗口上运行新的 shadow evidence 审计；
- 若主要窗口均有人员混杂、对象不可见或运动不可确认，则这些序列不能承担
  unknown-box 评价，应停止并考虑自采受控序列；
- `stationary + person absent` 窗口只作为同序列静态风险检查；
- 不允许根据后续 geometry/F1 结果回头修改本阶段标签。

## 6. 与总计划的关系

```text
G2-6E scanner-model proxy      已失败并冻结
G2-6O RGB motion observability 当前步骤
G2-6F shadow evidence audit    仅在 G2-6O 有可用窗口后设计
G1-F / G1-D                    继续锁定
```

G2-6O 不修改：

- `mvbDynamic`；
- `mvpMapPoints`；
- `Optimizer.cc` 或 g2o；
- YOLO；
- LocalMapping / LoopClosing；
- tracking 或 mapping 的任何状态。
