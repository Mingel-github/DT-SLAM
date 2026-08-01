# Bonn 箱子两帧运动一致性输入审计 SPEC

日期：2026-08-01
状态：运行前冻结

## 1. 身份与目标

本实验只审计：

> 当前 quality-eligible sparse ego-flow residual 在箱子粗框中是否具有足够的
> 多点支持和共同二维运动结构。

不执行聚类，不选择动态阈值，不生成对象 mask，不修改 SLAM。

## 2. 固定输入

复用已经完成的两次完整 C++ 诊断运行：

```text
Bonn moving_nonobstructing_box：24 个既有粗框 review 帧
Bonn moving_obstructing_box：17 个既有粗框 review 帧
same P=K rectification
same online semantic mask, age 0
same adjacent-frame sparse F1 measurements
```

只保留：

```text
evidence_state == measured
quality_eligible == 1
semantic_nonzero == 0
finite residual x/y
```

`q10` 只作为失败 baseline 计数，不参与本审计的连续向量选择。

粗框不是对象或运动 GT；强遮挡序列部分粗框覆盖很大且与人物重叠。语义非零
feature 会排除，但框内剩余背景仍不可避免。因此只报告 proxy coherence，不能
报告 precision/recall。

## 3. 连续统计

对每个区域 residual vectors \(\mathbf e_i\) 计算 component-wise median：

\[
\widetilde{\mathbf e}=\operatorname{median}_i(\mathbf e_i).
\]

稳健离散度：

\[
d=\operatorname{median}_i
\|\mathbf e_i-\widetilde{\mathbf e}\|_2.
\]

方向集中度：

\[
c=\frac{\|\sum_i\mathbf e_i\|_2}
{\sum_i\|\mathbf e_i\|_2+\epsilon}.
\]

其中 `c` 接近 1 表示向量总体同向，接近 0 表示方向相互抵消；它不是动态概率。

目标/背景中心分离：

\[
\Delta=\|
\widetilde{\mathbf e}_{in}-
\widetilde{\mathbf e}_{out}
\|_2.
\]

并输出描述量：

\[
R_{sep}=\frac{\Delta}
{0.5(d_{in}+d_{out})+\epsilon}.
\]

这些是 `[S/H]` 诊断统计，不来自 Lee 的刚体 hypothesis 公式。

## 4. 必须报告

逐帧：

- 框内/框外 eligible 数；
- 框内是否达到 3/7/10 点（仅描述支持量；7 是 Lee 原实现的 hypothesis 点数，
  不是迁移阈值）；
- residual magnitude median；
- vector median x/y；
- dispersion；
- direction concentration；
- target/background centroid separation；
- normalized separation；
- q10 数量。

聚合：

- 可比较帧数；
- 框内支持量分布；
- `inside magnitude median > outside` 帧数；
- `inside concentration > outside` 帧数；
- `separation > inside dispersion` 帧数；
- 所有原始连续统计。

## 5. 解释级别

运行前冻结三种解释，不据结果改阈值：

```text
unsupported:
  绝大多数目标帧没有多点支持，或 residual 中心与背景不可分

opportunistic only:
  部分帧存在多点共同运动，但不能覆盖多数目标帧

candidate for minimal shadow:
  多数可比较帧同时具有多点支持、目标/背景中心分离和较低组内离散
```

这里的“多数”只按超过一半作描述，不是论文参数或正式检测合格线。任何结果都
不能直接解锁 G1-F/G1-M/G1-D。

为避免看结果后改变解释，定义 `coherent_proxy_frame`：框内/框外各至少 3 个
eligible feature，且同时满足 `separation > inside dispersion` 和
`inside concentration > outside concentration`。若该帧数超过全部 review 帧
的一半，记为 `candidate for minimal shadow`；若至少重复 3 帧但未过半，记为
`opportunistic only`；否则记为 `unsupported`。这是 `[S/H]` 开发期解释规则，
不是动态检测阈值。

## 6. 禁止事项

```text
不调 q / FB / 5%
不运行 DBSCAN/K-means
不做 connected component/flood fill
不把 bbox 输入未来部署算法
不写 mvbDynamic/mvpMapPoints
不修改 YOLO/Optimizer/g2o/backend
```
