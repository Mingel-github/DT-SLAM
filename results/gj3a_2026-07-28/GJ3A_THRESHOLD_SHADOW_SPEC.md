# GJ-3A Relative-Threshold Shadow 审计规范

日期：2026-07-28

## 1. 范围

本阶段只检查一种最接近Ji原文文字的工程适配：

```text
cluster score > lambda × current-frame mean matched-feature error
```

该公式不是Ji论文公开公式，必须标为：

```text
[A/H] GJ-A relative-threshold adaptation
```

不修改Tracking结果，不设置`mvbDynamic`，不清除`mvpMapPoints`，不阻止MapPoint
写入。

## 2. 固定score

主score：

```text
r_j = mean_squared_error_px2
```

它对应当前显式声明的：

```text
rho(s) = s
```

identity工程baseline。Ji论文没有公开`rho`，因此该选择不是论文参数。

## 3. 两种threshold normalization

### support-weighted frame mean

```text
mu_feature =
  Σ_j(valid_reprojection_support_j × r_j)
  / Σ_j(valid_reprojection_support_j)
```

它最接近“average reprojection error of matched features”的字面解释。

### unweighted measured-cluster mean

```text
mu_cluster = mean_j(r_j)
```

它对应“cluster error relative to the others”的另一种可能解释。

论文没有说明采用哪一种，本阶段并列比较，不将任一项写成作者实现。

## 4. lambda网格

在查看validation前固定：

```text
lambda_k = 2^(k/2), k = -2, -1, ..., 6
```

即：

```text
0.5
0.70710678
1.0
1.41421356
2.0
2.82842712
4.0
5.65685425
8.0
```

这是无量纲对数扫描网格`[E]`，不是Ji论文参数。

## 5. 数据拆分

在运行200帧结果前冻结：

```text
dynamic calibration:
  TUM fr3/walking_xyz frame 1..99

dynamic validation:
  TUM fr3/walking_xyz frame 100..199

static validation:
  TUM fr1/xyz frame 1..199

reserved:
  walking_xyz frame >= 200
```

frame 0没有初始位姿，不参与阈值评价。

局限：calibration和dynamic validation来自同一序列的相邻时间段，并非严格独立
序列。本轮只能作为工程门控，不能作为论文最终泛化证据。

## 6. calibration选择规则

对每种normalization分别：

1. 仅在calibration计算所有lambda；
2. 以可测cluster范围内的person-proxy像素F1最大选择lambda；
3. F1相同时选择更大的lambda，作为更保守的删除规则；
4. 选择后冻结lambda，再读取dynamic validation和static validation。

person proxy不是运动GT，因此该规则只衡量人物语义区域代理，不证明未知动态能力。

## 7. 输出指标

动态calibration/validation：

```text
selected cluster fraction
selected depth area fraction
person proxy precision
person proxy recall
person proxy F1
```

static validation：

```text
selected cluster fraction
selected depth area fraction
```

unknown约定：

```text
valid_reprojection_support == 0
→ unknown
→ 不参与normalization
→ 不因阈值被标为static或dynamic
```

## 8. 门控

GJ-3A不能仅因dynamic proxy F1提高而通过。必须同时检查：

- validation相对calibration是否明显崩溃；
- static selected area是否过大；
- 两种normalization结果是否方向一致；
- 候选是否只在单一时间段有效；
- 运行仍为shadow-only。

本阶段不预先设定静态误删百分比上限，因为该数值没有Ji文献依据。若不存在明显
兼顾动态代理和静态保留的工作点，则记录为失败，不追加新的经验阈值。
