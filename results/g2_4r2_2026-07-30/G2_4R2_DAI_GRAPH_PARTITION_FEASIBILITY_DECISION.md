# G2-4R2 Dai 点关联图：最小拓扑可行性决策

日期：2026-07-30
状态：批准离线 shadow 拓扑审计；不批准在线动态判决；G1-F/G1-D 继续锁定

## 1. 决策

G2-4R1 已说明，当前 Tateno-style normal+distance 单帧分段适配会严重过分割，
不适合作为下一版轻量区域表示。下一步不继续调整其阈值，也不立即引入更完整的
3D 区域系统。

本阶段批准一个更小的问题：

> 使用当前 F3 已保存的两帧局部三维边测量，离线检查“异常边剔除 +
> connected components”是否能把目标代理内的观测从主要背景图中分离。

准确身份是：

> `[A/H]` 受 Dai et al. 点关联图分割启发的静态标定拓扑可行性审计。

它不是 Dai 方法复现，也不是可以部署的动态分类器。

## 2. 对上一轮路线理由的更正

上一轮以：

```text
目标代理内 MapPoint 比例只有 3.28%–10.53%
```

作为延期 Dai 路线的重要理由。这个事实成立，但推论不完整。

Dai 的图在整幅图的 tracked MapPoint 上构造。即使运动目标内部只有少量节点，
这些节点与大静态背景之间的不一致 crossing edges 仍可能被删除，从而形成一个
小连通分量。因此真正需要先回答的是：

```text
当前边测量经过一个不查看动态开发集的静态阈值后，
能否产生有意义的图拓扑分离？
```

不能只根据目标框内 MapPoint 数量提前否定。

同时，MapPoint 稀少仍是重要限制：若框内不足三个 MapPoint，无法声称恢复了
一个稳定刚体组，只能输出 `insufficient_support`。

## 3. 原论文支持的部分 `[L]`

本地原始来源：

```text
results/g2_4f3u_2026-07-30/literature/
  Dai_2022_Point_Correlations_arXiv1811.03217.pdf
  dai_source/bare_jrnl_compsoc.tex
```

Dai et al. 的相关步骤为：

1. 以 tracked MapPoint 为节点，在图像域做 Delaunay 邻接；
2. 三维相对向量是 edge state；
3. 使用 RGB-D 测量协方差构造 point-correlation objective；
4. 根据 Mahalanobis/chi-square 残差迭代删除不一致观测和边；
5. 用 DFS 得到 connected components；
6. 保留最大静态 component，用于后续运动估计。

前端位于初始位姿之后，并只使用当前帧和上一帧的测量。后端另有多关键帧滑窗
复核，当前项目不采用该后端。

## 4. 当前审计与原论文的差异

当前只具备：

```text
F3/F3U transient LK 或 tracked MapPoint 节点
Delaunay 邻接
两帧三维边长度变化
深度不确定度诊断
```

当前不具备：

```text
完整三维 edge-state optimization
论文的全 3×3 RGB-D covariance
明确可复现的 P-value、chi-square 阈值和迭代数
论文后端滑窗复核
稳定的对象身份或对象运动模型
```

因此本阶段只检查 topology feasibility：

```text
固定静态分位数阈值
→ 删除高 strain 边
→ connected components
→ 统计非主要分量是否富集于冻结 bbox proxy
```

阈值分位数和最大节点数 component 都属于 `[S]` 审计设计，不是 Dai 原算法。

## 5. 为什么不直接实现“完整 Dai”

目前不能忠实复现的关键原因：

- 原论文没有在当前本地材料中给出前端 P-value、数值阈值和迭代次数；
- `largest volume` 的具体体积实现没有充分说明，前端段落又写作 largest
  connected component，存在实现歧义；
- 当前 F3 大多数节点是 transient LK，而不是稳定 MapPoint；
- 完整 point-correlation optimization 会明显扩大核心代码和优化逻辑；
- 用户当前明确不希望修改 `Optimizer.cc`、g2o 或后端。

在不知道最小图拓扑本身是否有用前，不应先承担这些改动。

## 6. 两个并列输入分支

审计必须同时报告：

### A. all-transient

使用全部 F3 eligible LK 节点重新构图。

- 优点：动态代理内支持较多；
- 身份：显著 `[A/H]`；
- 风险：LK/depth 噪声可能产生大量假断边。

### B. MapPoint-only

只使用 `has_mappoint=1` 的节点重新做 Delaunay。

- 优点：更接近 Dai 前端输入；
- 风险：目标内节点可能不足，许多帧只能输出 unknown。

不能简单从 all-transient 图中筛选“两端都是 MapPoint”的旧边，因为在节点子集
上重新三角化会生成不同的邻接关系。

## 7. 其他候选为何暂不采用

- FlowFusion 支持 observed-flow minus ego-flow 作为动态证据，但原方法依赖
  dense flow、supervoxel 和稠密联合优化，不直接支持当前稀疏分组。
- Jaimez et al. 通过几何 cluster 联合估计 piecewise-rigid scene flow，方法
  完整但明显重于本阶段的 ORB-SLAM2 前端审计。
- ClusterSLAM 使用 landmark motion affinity、短序列 consensus clustering
  和后端 factor graph，超出“不改后端”的范围。
- DetectFusion 的未知物体分支依赖静态 surfel map 的 ICP residual；当前没有
  该稠密静态模型。

这些论文支持“运动证据必须经过结构化聚合”，但不能作为当前最小实现已经有效
的证明。

## 8. 继续边界

G2-4R2 仍禁止：

```text
选择在线 dynamic threshold
写入 mvbDynamic
过滤 mvpMapPoints
创建/删除 MapPoint
修改 YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing
使用 sealed balloon_tracking holdout 调参
把 bbox proxy 称为运动/对象 ground truth
```

若本轮连最小拓扑分离都不能在静态风险可控时形成动态代理富集，则停止这条
简化图路线；不能继续靠动态数据反复调阈值。

## 9. 参考

- Dai et al., *RGB-D SLAM in Dynamic Environments Using Point
  Correlations*, IEEE TPAMI 2022:
  <https://arxiv.org/abs/1811.03217>
- Huang et al., *ClusterSLAM*, ICCV 2019:
  <https://openaccess.thecvf.com/content_ICCV_2019/html/Huang_ClusterSLAM_A_SLAM_Backend_for_Simultaneous_Rigid_Body_Clustering_and_ICCV_2019_paper.html>
- Jaimez et al., *Fast Odometry and Scene Flow from RGB-D Cameras Based
  on Geometric Clustering*, ICRA 2017:
  <https://cvai.cit.tum.de/_media/spezial/bib/jaimez_et_al_vosf_2017.pdf>
- FlowFusion, ICRA 2020: <https://arxiv.org/abs/2003.05102>
