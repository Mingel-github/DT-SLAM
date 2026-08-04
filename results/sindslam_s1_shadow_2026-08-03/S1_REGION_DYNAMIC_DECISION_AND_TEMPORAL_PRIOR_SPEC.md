# S1 区域动态判决与时序先验规格

日期：2026-08-04  
状态：S1 子步骤；不新增主阶段；shadow-only

## 1. 目的

补齐已实现的稠密光流 residual 与区域标签之间缺少的判决层：

```text
above-low residual support + current high-residual seed
→ cluster-confined flood fill
→ partial/whole-region dynamic state
→ previous detector state for next-frame homography sampling
```

本步骤不修改 `Frame::mvbDynamic`、`mvpMapPoints`、`Optimizer` 或 MapPoint
创建逻辑。S2 仍未开放。

## 2. 方法归属

- `[L]`：SInDSLAM 论文 Section III-C2 的区域内双阈值残差、连通传播、
  `filled > 0.5 * region` 时整区域判动态以及 `0/125/255` 三态输出；
- `[C]`：作者源码实际采用的 Otsu/Triangle 及物理阈值 clamp、上一帧 high
  residual 加入当前 above-low 支持、5×5/9×9 膨胀、簇内 high 支持数与轮廓
  形状限制，以及上一帧三态/标签参与 homography 样本排序；
- `[S]`：修正作者阈值分支对标量调用 `countNonZero` 的错误；没有合法 flood
  seed 时拒绝填充；显式区分作者 `imgTotalArea`、正区域标签和 DT-SLAM
  depth-valid 三个坐标域；失败或无历史输出 unknown。

这里复现的是“经上述两项已声明正确性修正后的作者源码行为 profile”，不是把
论文公式与源码细节混称为同一个算法。

## 3. 输入语义

- `regionLabels`：`CV_32SC1`；正值才是可填充区域，零/负值不是静态证据；
- `regionValidMask`：作者 `imgTotalArea` 等价域，独立于 region label；
- `aboveLowThresholdMask`：源码现有字段名为 `lowResidualMask`，实际语义是
  `residual > low threshold`，包含 high 与中间不确定支持；
- `currentHighResidualMask`：仅当前帧 high residual 能形成 seed；
- previous high residual：只能扩展 above-low 支持，不能独立产生当前 seed；
- previous detector state/labels：只用于下一帧 homography 采样排序。

## 4. 分层输出

- `filledDynamicMaskBeforeDilation`：唯一可称为 cluster-confined 的动态 core；
- `authorStyleDynamicMask`：9×9 后的作者 detector mask，可能越过 label/valid
  边界；
- `dynamicMask`：作者 detector mask 与 region-valid 的交集；
- `rawStateMask`：作者三态，`0=unknown, 125=static, 255=dynamic`；
- `unknownMask`：由独立 validity 建模得到，不得把无证据转换为 static。

作者 runner 额外执行的 15×15 膨胀不属于 detector temporal state，本步骤不把
runner final mask 回灌给下一帧。

## 5. 验收

1. 使用同一作者 CPU reference 的 labels、imgTotalArea、low/high evidence，
   native classifier 与 `_mask_pre_runner_dilate.png` 逐像素一致；
2. native CPU DeepFlow 启用上一帧 detector state/labels 后，与作者 CPU 的
   flow、H、residual、阈值和 low/high mask 成对核对；
3. synthetic test 证明 fill core 不跨 label、previous high 不能成为当前 seed、
   Reset 后不复用旧状态；
4. Tracking shadow 的 `actual_slam_removed=0`，没有直接 SLAM mutation；
5. 开关关闭的原始配置正常运行。

## 6. 本步骤之后仍未解决

受控等价路径仍使用作者导出的 region labels 与 `imgTotalArea`。DT-SLAM native
3D cluster/gradient/RAG 输出尚未替代这两个外部输入，因此本步骤通过不等于 S1
全部完成，也不等于动态 mask 已经通过 TUM/Bonn 质量评价。
