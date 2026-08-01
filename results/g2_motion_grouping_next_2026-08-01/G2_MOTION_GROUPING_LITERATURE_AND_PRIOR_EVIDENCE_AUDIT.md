# 共同运动分组：文献与既有证据审计

日期：2026-08-01
状态：方法边界冻结；不实现动态判决

## 1. 当前问题

现有 F1 输出的是：

\[
\mathbf e_i=\mathbf u_{t,i}-\widehat{\mathbf u}_{t,i},
\]

即当前 ORB/LK 观测与 RGB-D/SE(3) 静态预测之间的二维残余运动。严格留出已
证明连续 residual 常有目标方向性，但 `q>=10` 硬候选没有通过冻结的代理条件。
因此下一问题是：

> 质量合格的连续残余向量是否具有足够的多点共同运动结构，可以支持一个最小
> motion-grouping shadow？

不得把问题重新写成“如何给 q10 再加邻域扩张”。

## 2. 本地既有证据

### 已完成且不重复

- balloon/balloon2 短轨迹支持审计：两帧 transient LK 多数候选帧有输入；
  3--5 帧轨迹和稳定 MapPoint 支持明显不足；
- F3/F3U：两帧局部 edge strain 有分布方向性，但没有形成可靠分组；
- G2-4R2：scalar strain + 静态分位数 + connected components 失败；
- G2-4R1：normal+distance 单帧区域严重过分割；
- 简单 depth component / flood fill：背景泄漏；
- q10 strict holdout：硬候选条件失败。

所以本轮不再测试：

```text
轨迹寿命直接判动态
固定 3/5 帧多数票
q10 点的普通连通域
scalar edge threshold + CC
另一组 depth flood-fill 参数
```

## 3. Lee et al. 2019 `[L]`

原文以 RGB-D grid scene flow 构造三维点在相邻帧中的对应，并利用同一刚体上
三维点应共享同一刚体变换的假设：

\[
E(H,X^{(j)},X^{(k)})=
\operatorname{diag}\left((HX^{(j)}-X^{(k)})^T
(HX^{(j)}-X^{(k)})\right).
\]

其空间分组包含：

```text
grid scene flow
→ 邻近 7 点生成 rigid-motion hypothesis
→ hypothesis refinement
→ DBSCAN 聚类多个 motion hypotheses
→ segment matching
→ dual-mode temporal motion model
```

原文 `m=7`、search radius、rigid error、DBSCAN epsilon 等均是在作者自采数据上
调节的参数，不能迁移为本项目阈值。

它支持：

- 分组应由共同刚体运动解释，而不只是空间连通；
- 空间分组与跨帧身份是两个独立步骤；
- 无效 scene flow 必须有未测量状态。

它不直接支持：

- 将二维 F1 residual 直接输入 DBSCAN；
- 用 `q10` 预先截断所有输入；
- 将少数二维同向点称为 SE(3) 对象；
- 复制论文参数。

原始材料：

```text
literature/Lee_2019_Rigid_Motion_RGBD_VO_arXiv1907.08388.pdf
SHA-256 2a238cf524fed051f6d981d088f43774148fdbe42a863eec9391463e82dd1420
```

## 4. SInDSLAM `[L]`

本地原文和 PaperNotes 显示其使用 dense optical-flow residual、三维 K-means、
几何边缘重聚类、cluster-confined region growing 和时序先验。它支持“残差需要
区域上下文”，但完整系统不是轻量稀疏分组，且当前已有区域负实验不能通过再加
一个 merge 阈值绕过。

## 5. Dai / DymSLAM / MVO `[L]`

- Dai 使用 tracked MapPoint、三维 point-correlation edge state、协方差、迭代
  离群边删除和 connected components；当前 transient LK + scalar strain 不是
  其复现。
- DymSLAM 用多模型拟合将特征分配给 ego-motion 与多个刚体运动模型。
- MVO 对 tracklet 做多运动分割并估计完整 SE(3) 轨迹。

它们共同支持“对象组需要运动模型”，但完整算法均超出当前不改后端的最小范围。

## 6. LC-CRF `[L]`

LC-CRF 用 GC-RANSAC 先验、CRF 一元/相互作用项和长期 landmark observations
提高动态地标一致性。它适合未来的确认层，不是从零散 transient feature 首次
发现对象的直接方法；不得简化成无依据的三帧多数投票。

## 7. 本轮允许回答的最小问题

只利用现有、质量合格且语义未标动态的 F1 连续二维向量，在 coarse RGB-only
目标框内外比较：

- 多点支持量；
- 残余向量的稳健中心；
- 围绕中心的离散度；
- 方向集中度；
- 目标与背景残余中心的分离程度。

这是 `[S/H]` 输入充分性审计，不是 Lee reproduction，也不是 motion grouping。
若二维残余在目标内都不形成可重复的共同方向，直接实现二维聚类没有依据；若
形成稳定结构，才允许冻结下一张最小 grouping shadow SPEC。
