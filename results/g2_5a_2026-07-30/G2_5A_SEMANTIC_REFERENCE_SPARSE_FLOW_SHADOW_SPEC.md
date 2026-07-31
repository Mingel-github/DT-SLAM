# G2-5A Semantic-reference / Semantic-blind Sparse F1 Shadow SPEC

日期：2026-07-30
状态：实现与运行前冻结
范围：自动代理评价；不修改 SLAM；不选择新的部署阈值

## 1. 唯一问题

本阶段只回答：

> 在几何 F1 residual 完全不读取 semantic label 的前提下，同步 person mask
> 覆盖的 ORB feature 是否比同帧非 semantic feature 表现出稳定、更高的
> ego-flow inconsistency？

该问题用于自动检查 F1 连续证据的可分性，减少逐像素人工标注工作。

不回答：

```text
person feature 是否一定正在运动
是否已经检测到 unknown object
是否可直接删除 feature
是否改善 ATE/RPE
```

## 2. 代码路径审计

当前 `ComputeSparseEgoFlow()`：

```text
对 mCurrentFrame.mvKeys 的全部 ORB feature
→ LK 回溯与 forward-backward 检查
→ RGB-D/SE(3) ego projection
→ continuous residual
```

`semantic_nonzero` 在 F1 feature CSV 中仅作为事后记录字段。semantic exclusion
发生在后续 F3 rigidity graph，不发生在 F1 residual 计算之前。

因此本阶段：

```text
不需要修改 C++
不需要增加 semantic bypass
```

只需用在线同步语义重新生成 TUM F1 CSV，并用离线工具读取已有字段。

## 3. 方法来源边界

### `[L/A]` FlowFusion

支持：

\[
\mathbf f_i^{res}
=
\mathbf f_i^{obs}
-
\mathbf f_i^{ego}
\]

可作为类别无关运动不一致证据。

不支持：

- 当前 sparse ORB/LK 版本已是 FlowFusion 复现；
- person mask 是 motion ground truth；
- 某个公开阈值可直接迁移。

### `[L/A]` Li and Lee

支持：

- invalid correspondence 不参与估计；
- 用鲁棒尺度和连续静态权重表达不确定性；
- 不应先把所有观测强制二值化。

不支持：

- 当前二维 residual 的数值阈值；
- semantic mask 作为监督标签；
- 直接修改 ORB-SLAM2 Optimizer。

### `[S]` 当前自动评价协议

semantic mask 只在 residual 计算结束后给 feature 添加：

```text
semantic_reference_proxy = 0/1
```

它不进入：

```text
LK
depth
ego projection
frame robust scale
initial pose
feature score
```

## 4. 数据角色

### 4.1 Semantic-region development

```text
TUM fr3/walking_xyz 前 150 帧
online CUDA YOLO person mask
mask age 必须为 0
```

`walking` 中人物通常运动，但 `semantic_nonzero=1` 仍只称为
`person-region proxy`，不是逐点运动 GT。

### 4.2 Low-motion semantic challenge

```text
TUM fr3/sitting_static 前 150 帧
online CUDA YOLO person mask
mask age 必须为 0
```

`sitting_static` 的 static 描述相机运动方式，人物仍可能手势或轻微运动，不能
作为真静态负样本。

### 4.3 True-static negative

复用：

```text
TUM fr1/xyz static 150
Bonn static_close_far 150
```

它们用于报告 candidate risk，不提供 semantic positive。

### 4.4 Unknown cross-class development

复用：

```text
Bonn balloon
Bonn balloon2
```

冻结 RGB-only coarse bbox 的已有 F1 结果只用于检查方向迁移。已经打开过的
`balloon_tracking` 不再作为新阈值的 strict holdout。

## 5. Feature 有效性

只接受：

```text
evidence_state == measured
backward_lk_status == 1
forward_lk_status == 1
reference_depth_valid == 1
slam_ego_projection_valid == 1
finite FB error
finite residual
FB error <= 0.25 px
```

其余状态统一为 `no_evidence`，不能解释为 static。

`0.25 px` 是此前 F3/F2 已冻结的 correspondence-quality 工作点，不是论文
运动阈值。本阶段不再调它。

## 6. 连续分数

为避免混淆，必须同时计算两个尺度。

主要的 semantic-blind 尺度使用全部 quality-eligible feature：

\[
\hat\sigma_t^{blind}=
\max
\left(
0.001,\,
1.4826\,\operatorname{median}(r_i)
\right),
\]

\[
q_i^{blind}=r_i/\hat\sigma_t^{blind}.
\]

辅助的 combined-pipeline 尺度只使用 `semantic_nonzero=0`：

\[
\hat\sigma_t^{combined}=
\max
\left(
0.001,\,
1.4826\,\operatorname{median}_{semantic=0}(r_i)
\right),
\]

\[
q_i^{combined}=r_i/\hat\sigma_t^{combined}.
\]

其中 \(r_i\) 是 raw SLAM-pose residual magnitude。前者完全不读取 semantic；
后者利用语义先验避免已知 person 污染尺度，因此不能称为 semantic-blind。

必须同时报告：

```text
raw residual
q_blind
q_combined
FB error
has_mappoint
```

任一尺度 support 少于 20 时，对应 q 记为无标定证据。预注册主判定使用
`q_blind`；`q_combined` 只作现有语义＋几何 pipeline 对照。

## 7. 指标

### 7.1 每帧

只有同时满足：

```text
semantic quality-eligible >= 3
nonsemantic quality-eligible >= 20
```

才称为 comparable frame。

报告：

- semantic/nonsemantic feature 数；
- raw residual median、P90；
- q_blind / q_combined median、P90；
- paired median difference 与 ratio；
- Mann–Whitney rank AUC；
- MapPoint 子集的相同统计；
- 两种 frame scale 和各自 support。

### 7.2 聚合

分别对 walking 与 sitting 报告：

- comparable frame 数；
- semantic median > nonsemantic 的帧比例；
- semantic q median > nonsemantic 的帧比例；
- per-frame AUC median/P10/P90；
- semantic/nonsemantic pooled feature 数；
- MapPoint comparable frame 数。

不使用 pooled AUC 作为主结果，避免长帧和同帧相关 feature 主导。

### 7.3 固定候选曲线

对两种 scale mode 分别报告此前已使用的固定网格：

```text
q = 2, 3, 4, 5, 6, 8, 10
```

对每个 q 报告：

- semantic proxy candidate rate；
- nonsemantic candidate rate；
- true-static total/MapPoint candidate rate；
- walking 与 sitting 的差异。

本阶段不从结果中选择一个新 q。

## 8. 预注册判定

### 连续证据支持

walking 同时满足：

```text
comparable frames >= 20
semantic raw median > background >= 80% frames
semantic q median > background >= 80% frames
per-frame AUC median >= 0.75
```

则可写：

> F1 continuous evidence 在自动 person-region proxy 上具有稳定可分性。

### 跨类别一致性

已有 unknown balloon development 必须继续保持：

```text
inside residual median > outside in >=80% comparable frames
```

这里只复用冻结结果，不重选 bbox 或帧。

### G1-F0 资格

只有连续证据支持和跨类别一致性都满足，才允许下一阶段做：

```text
G1-F0 counterfactual deletion audit
```

G1-F0 仍不实际删除 feature。

即使上述条件通过，也不能直接开放 G1-F，因为：

- semantic proxy 不是 motion GT；
- `balloon_tracking` 已完成一次旧工作点评价，不能用于新阈值调参；
- 仍需独立数据或受控实验验证最终工作点。

若 walking 条件失败，则暂停轻量 F1 判决路线，不再增加 score 组合修补。

## 9. 输出

```text
per_frame.csv
candidate_curve.csv
summary.json
G2_5A_*_RESULT.md
```

所有输出必须带：

```text
reference_identity=semantic person-region proxy; not motion ground truth
raw_residual_uses_semantic=false
semantic_blind_scale_uses_semantic=false
combined_scale_excludes_semantic=true
dynamic_decision=none
direct_slam_state_mutation=none
```

## 10. 测试和不变量

- malformed header 失败；
- non-finite 数值不进入统计；
- blind scale 使用全部 eligible，combined scale 排除 semantic；
- tied-rank AUC 有确定结果；
- 输入行顺序改变不影响结果；
- 相同输入重复运行字节一致；
- semantic 全零时只产生 `not_evaluable`，不伪造 AUC；
- 不访问 `balloon_tracking` 原始 CSV；
- 不修改 C++、YOLO、Optimizer、g2o 或后端；
- Viewer OFF 只用于正式计时；需要定性观察时可另跑 Viewer ON。

## 11. 参考

- FlowFusion, ICRA 2020: <https://arxiv.org/abs/2003.05102>
- Li and Lee, *RGB-D SLAM in Dynamic Environments Using Static Point
  Weighting*, IEEE RA-L 2017:
  <https://mediatum.ub.tum.de/doc/1375854/document.pdf>
- 既有详细审计：
  `results/g2_4f2_2026-07-30/G2_4F2_RELIABLE_FEATURE_GATE_LITERATURE_AUDIT.md`
