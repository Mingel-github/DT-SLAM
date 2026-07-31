# G2-4R2 之后的下一路线决策

日期：2026-07-30
状态：路线冻结；下一步先做自动代理评价 SPEC，不直接实现新滤波

## 1. 当前事实

截至 R2，类别无关几何分支的事实不是“完全无信号”，而是：

```text
F1 sparse observed-flow - ego-flow:
  development 连续 residual 有明显局部富集
  strict holdout 13/14 帧方向正确

F2 单点 hard q threshold:
  strict holdout 只覆盖 7/14
  安全预算也略超预注册限制

F3/F3U 两帧 scalar rigidity:
  有分布方向，但没有形成可靠二值判决

F4 / R1 区域容器:
  简单 depth component 会泄漏
  normal+distance 适配严重碎裂

R2 scalar graph + CC:
  all-transient 拓扑召回不足
  MapPoint-only 目标支持不足
```

因此，失败点仍是：

> continuous motion evidence 到可靠 feature decision 的转换。

## 2. 三条可选路线

### A. 立即实现更完整区域/多运动系统

可参考：

- DetectFusion：surfel-map ICP residual + geometric segments；
- SInDSLAM/DGS-SLAM：3D clustering、re-clustering、区域残差；
- Jaimez et al.：geometric cluster + piecewise-rigid scene flow。

优点：

- 有机会输出低纹理区域和 `M_depth`；
- 更接近完整未知动态区域检测。

当前不选的原因：

- 需要引入稠密静态模型、3D cluster 或联合运动估计；
- 已明显超出“轻量 ORB-SLAM2 前端过滤器”的原阶段范围；
- 当前 R1/F4 负结果没有证明其中某个完整子系统值得优先承担；
- 在评价标签仍较弱时增加系统复杂度，会更难解释失败来源。

### B. 实现更完整 Dai point-correlation

优点：

- 与 ORB-SLAM2 稀疏 MapPoint 前端结构相符；
- 有明确原论文数学和图分组依据。

当前不选的原因：

- 当前目标代理内 MapPoint 支持仅在 1/17 帧达到至少三个点；
- 原文前端关键数值细节和 largest-volume 实现仍不充分；
- 完整 edge-state optimizer 会引入新的优化模块；
- R2 已证明当前 scalar 子集不足，不能靠补一个阈值修复。

### C. 先把 F1 连续证据做成可自动评价的跨类别 feature 审计

这是当前选择。

核心思想：

```text
几何算法：
  对全部可测 ORB feature 计算 F1 continuous residual

离线评价：
  只在计算完成后，用同步 semantic mask 标记 feature 所在区域
```

semantic mask 只作为离线 reference proxy，不参与 residual、相机位姿、邻域或
feature score。运行时几何仍可在 semantic 输出为空时工作。

## 3. 为什么选择 C

1. F1 是目前唯一同时在 development 和 strict holdout 保持大多数帧方向正确
   的类别无关连续证据；
2. F1 本身约 2.5–2.9 ms，符合当前前端规模；
3. 精确同步 person mask 已由现有 CUDA pipeline 自动产生，不要求用户逐像素
   人工标注；
4. 可以在 TUM walking 的已知运动区域上标定/审计，再检查其对未知识别气球
   的跨类别迁移；
5. 真静态 fr1/xyz 和 Bonn static 可继续约束背景误删；
6. 若连这种更强自动代理下都不可分，应停止 F1 filtering，而不是继续发明
   grouping 参数。

## 4. 文献身份

- `[L/A]` FlowFusion 支持 observed flow 减 camera-induced flow 作为类别无关
  运动残差；当前使用 sparse LK/ORB 适配。
- `[L/A]` Li & Lee 支持对动态环境中的观测使用连续静态权重和鲁棒尺度；它
  不直接给出本项目 feature threshold。
- `[S]` 使用 semantic mask 作为离线评价标签、而不让它进入几何 score，是
  本项目的验证协议。
- `[H]` 在 person 区域上形成的高精度工作点能迁移到 unknown balloon/box；
  必须实验，不能预设。

不得表述为：

```text
semantic-supervised geometry detector
pixel motion ground truth
FlowFusion reproduction
Li-Lee reproduction
```

## 5. 下一阶段名称与边界

下一阶段冻结为：

> **G2-5A：semantic-reference、semantic-blind sparse F1 separability
> shadow SPEC。**

先写 SPEC，只设计：

- 如何在不改变 SLAM 的情况下记录 semantic 区域内的 F1 residual；
- person mask 如何与边界膨胀区、无效深度和遮挡风险分开；
- walking、sitting、真静态和 unknown development 如何分工；
- 哪些指标代表高精度、哪些只代表方向性；
- 开发、验证和已经打开的 holdout 如何防止再次泄漏；
- 若失败，在哪里停止。

在 SPEC 审阅前不修改 C++。即使 G2-5A 通过，也只允许先做：

```text
G1-F0 mutation simulation / shadow counterfactual
```

不能直接进入真实 `mvpMapPoints` 删除。

## 6. 关于正式应用几何方法

当前不能给出“只差几天即可放行”的保证。准确的最短路径是：

```text
G2-5A 自动代理可分性
→ 高精度工作点在独立数据上通过
→ G1-F0 仅模拟将删除哪些 feature、剩余支持是否足够
→ G1-F 短序列真实过滤
→ ATE/RPE/FPS 与失跟风险对照
```

若 G2-5A 不通过，应暂停轻量 feature filtering。此时只能在“接受更完整重型
区域方法”与“补受控数据/改变论文范围”之间重新选择，不能继续无限追加
shadow 子模块。

## 7. 参考

- FlowFusion, ICRA 2020: <https://arxiv.org/abs/2003.05102>
- Li and Lee, *RGB-D SLAM in Dynamic Environments Using Static Point
  Weighting*, IEEE RA-L 2017:
  <https://mediatum.ub.tum.de/doc/1375854/document.pdf>
- Dai et al., IEEE TPAMI 2022:
  <https://arxiv.org/abs/1811.03217>
- Jaimez et al., ICRA 2017:
  <https://cvai.cit.tum.de/_media/spezial/bib/jaimez_et_al_vosf_2017.pdf>
