# 可靠运动组判决：假设支持集验证 SPEC

日期：2026-08-01
状态：模块 1 内部实现前冻结；不是新增总阶段

## 1. 目的

G2-MH1 已证明 7 点局部刚体模型能够被正确拟合，但在 TUM fr1/xyz 真静态场景中
也会吸收深度、LK 和位姿噪声。训练内 local fit 因此不能作为动态证据。

本步骤只回答：

> 一个由 7 点生成的局部 SE(3) 假设，是否能在没有参与拟合的三维对应上继续比
> 背景相机模型更好地解释观测？

它属于固定总计划的“可靠运动组判决”模块内部，不创建新的 G 阶段。

## 2. 文献依据和适配边界

### `[L]` Lee et al. 2019

原文明确分为：

```text
m=7 邻近点产生初始 rigid-motion hypothesis
→ 对全部 n 个 grid scene-flow vector 计算 Eq. (1) 刚体误差
→ 得到增加后的 N 点支持集并重新估计 refined hypothesis
→ 对 refined hypotheses 聚类
```

原文在自采数据上使用 `t_inlier=3e-5`，并明确说明参数需要针对新环境重新调节。
该阈值不迁移到 DT-SLAM。

### `[A/S]` 当前稀疏适配

当前输入是 ORB/LK RGB-D 对应而不是规则 grid scene flow。为先检查泛化能力：

1. 保持已经冻结的 anchor＋最近 6 点作为 7 点训练集；
2. 使用距离排序中接下来的 7 个、未参与拟合的点作为 local holdout；
3. 使用全部 eligible 且不属于训练集的点作为 global support audit；
4. 在相同验证点上成对比较局部假设和背景相机模型；
5. 不使用 bbox、motion proxy、q10 或 semantic label 选择支持点。

“接下来的 7 点 local holdout”是本项目的确定性交叉验证设计 `[S]`，不是 Lee
原算法。它用于防止把训练内过拟合误认为物体运动，不声称为论文创新。

## 3. 输出

每个 seed hypothesis 新增连续量：

```text
local holdout count
holdout local/background median, RMS, P90
holdout median improvement
holdout background/local RMS ratio
holdout local-better fraction

global validation count
global local-better count/fraction
global median improvement
```

其中：

```text
improvement = background_error - local_hypothesis_error
local_better = local_error < background_error
```

`local_better` 只用于成对模型比较统计，不是动态类别。非常小的数值差异仍可能
来自噪声，所以本轮不据此建立 feature label。

无足够额外验证点时输出 `insufficient_validation_support`，不能解释成静态。

## 4. 本轮不做

```text
不迁移 Lee t_inlier
不选择新的 residual threshold
不扩展或重估支持集
不做 DBSCAN
不做时序标签
不修改 mvbDynamic / mvpMapPoints / MapPoint admission
不修改 Optimizer、g2o、YOLO 或后端
```

重估没有被取消，而是被放在同一模块的下一个条件步骤：只有独立验证显示运动
proxy 的模型能够泛化、而真静态模型不能时，才允许用支持点重估。若独立验证本身
失败，重估只会放大错误支持，不应实现。

## 5. 测试

合成测试：

1. 静态刚体：正确背景模型与局部模型在 holdout 上都接近零；
2. 两个刚体：对象内部 seed 对未参与拟合的对象点继续具有优势；
3. 混合边界：跨运动边界的 seed 不应在 holdout 上保持低误差；
4. 只有 7--13 个有效点：显式 insufficient validation；
5. 输入重复运行：输出完全确定。

真实数据保持：

```text
TUM fr1/xyz 真静态
Bonn moving_nonobstructing_box 5 moving / 19 stationary RGB proxy
```

Bonn 如果当前执行环境仍无法在线运行 CUDA semantic，必须明确报告为
`semantic_mode=none`，不得把人物运动解释为未知箱子。

## 6. 停止规则

继续同一模块的支持集重估，需要同时看到：

- 真静态的训练内 improvement 在 holdout 上明显衰减；
- moving proxy 的 holdout improvement 或 local-better support 高于 stationary；
- 同帧箱内相对箱外仍保留同方向差异；
- 额外成本与信息增益相称。

若 holdout 与已有 F1 一样主要响应全局误差，或静态背景仍普遍获得相同优势，
则停止该稀疏刚体路线。不得再增加邻域半径、q、面积或经验 margin 补丁。
