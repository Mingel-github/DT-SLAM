# G2-4F3 局部刚性一致性：文献决策

日期：2026-07-30  
状态：只完成方法选择；尚未实现  
前提：G2-4F2 strict holdout 的硬候选门失败，G1 继续锁定

## 1. 失败驱动的问题

holdout 表明：

```text
continuous residual local enrichment:
  SLAM pose 13/14 frames positive
hard q>=10 in-box candidate:
  7/14 frames
```

因此下一步不能只是调低 `q`。需要一个与 residual 幅值不同的空间一致性证据，
用于区分：

- 独立运动刚体上的一致观测；
- 单点 LK 错配；
- 位姿误差造成的散乱 residual；
- 深度边界或无效深度附近的异常。

这与原始研究路线中的“若随机残差和静态误检较多，则增加局部点关联刚性检查”
一致，不是临时偏离。

## 2. 检索顺序

先读取本地：

```text
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/detectfusion.md
/home/zhu/Desktop/paper_notes/comparison_23_papers.md
```

本地没有 Dai 等 point-correlation 原文或笔记，因此只对该缺口使用 primary
fallback：

```text
Dai et al.,
RGB-D SLAM in Dynamic Environments Using Point Correlations,
arXiv:1811.03217, revised 2020; later TPAMI.
https://arxiv.org/abs/1811.03217
```

## 3. 文献能支持什么

### 3.1 Dai 等 point correlation

`[L]` 原方法：

1. 使用 Delaunay triangulation 在 map points 上建立稀疏图；
2. 顶点为 map point，边表示相邻点的 correlation；
3. 若两点相对位置随时间保持一致，则认为一起作刚性运动；
4. 优化并删除不相关边，剩余连通组分离静态场景与不同运动物体；
5. 最大组被假定为可靠静态组，再用于 motion estimation。

它支持：

```text
relative geometry consistency
can reject isolated mismatches
can reveal a rigidly moving group
```

它不直接支持当前工程立即复制：

```text
长期 map-point graph optimization
largest connected component = static
直接修改 motion estimation
动态区域占多数时仍安全
```

### 3.2 SInDSLAM 与 DetectFusion

`[L]` SInDSLAM 强调 point residual 缺少区域上下文，并把 optical-flow
residual 限制在 geometric re-cluster 内判定；`[L]` DetectFusion 也先形成
geometric segment，再用 registration residual 与 segment overlap 扩展未知
运动。

二者共同支持“residual 需要空间一致性上下文”，但当前不能直接采用其：

- dense flow / homography；
- surfel/TSDF registration；
- K-means(K=2)；
- re-clustering 参数；
- region IoU 阈值。

当前 G2-3R4 的低分辨率 region representation 尚未做结构保真评价且收益门
失败，因此本阶段不把它重新包装成可靠对象分割。

## 4. 建议的最小 Shadow 适配

先只在当前/上一帧都具有有效 RGB-D 和可靠 LK 对应的 ORB feature 上：

1. `[A]` 在当前图像域建立局部 Delaunay 或固定 k-NN 稀疏邻接；
2. `[A]` 比较邻接点在两帧中的三维距离变化：

\[
\epsilon_{ij}
=
\left|
\|\mathbf X_{i,t}-\mathbf X_{j,t}\|_2
-
\|\mathbf X_{i,r}-\mathbf X_{j,r}\|_2
\right|;
\]

3. `[S/H]` 同时保留每点连续 ego-flow residual，不先降低 `q`；
4. `[S/H]` 输出 edge strain、有效邻边数、局部刚性支持和 component 统计；
5. 不产生 `dynamic=true`，不进入 tracking 或 mapping。

目标不是证明：

```text
low edge strain => static
```

因为静态背景和独立运动刚体内部都可能保持刚性。要验证的是组合物理条件：

```text
violates the static camera-motion model
+
maintains local internal rigidity
```

能否比单点 hard residual 更可靠地描述独立运动刚体。

## 5. 方法身份

如果实施，必须写成：

> `[A/S/H]` A local, two-frame feature-graph rigidity diagnostic adapted
> from Dai et al.'s point-correlation principle and coupled only in shadow
> with the existing FlowFusion-derived sparse ego-flow residual.

不能称为：

```text
Dai et al. reproduction
point-correlation SLAM
object motion estimation
proven dynamic detector
```

## 6. 实施前门

进入代码前先冻结 G2-4F3 SPEC：

- 图构建域和边数上限；
- current/reference depth 的采样与 invalid 状态；
- edge strain 单位和数值稳定性；
- 不允许读取 `balloon_tracking` 选择阈值；
- 静态序列只作 risk proxy；
- `balloon/balloon2` 仍为 development；
- 另选未用于开发的新序列作未来验证；
- CPU active time 必须单独报告；
- `dynamic_decision=none`、`direct_slam_state_mutation=none`。

如果局部图在低纹理箱子上支持不足，或静态/动态 proxy 不可分，应归档为负结果，
而不是继续堆叠阈值。

