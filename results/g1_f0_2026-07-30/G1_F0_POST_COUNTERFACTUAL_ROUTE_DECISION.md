# G1-F0 后几何路线决策

日期：2026-07-30
状态：当前 lightweight hard-filter 路线冻结；下一阶段先补独立评价依据

## 1. 当前可以确定的事实

```text
F1 continuous sparse ego-flow evidence:
  walking person proxy        有明显方向性
  balloon RGB-only bbox       既有开发帧有点级富集

F1 q6/q8/q10 hard candidate:
  初始关联风险                较低
  local-search 后 Bonn 静态    三个 q 均超过 0.20% 预算
  final unknown inlier 作用    balloon 0，balloon2 很少
```

所以失败的不是 observed-flow minus ego-flow 这一物理测量，而是：

> 用单帧、单 feature 的归一化 residual 阈值直接形成可部署动态删除决定。

## 2. 当前不得继续的路线

- 在已打开数据上试 q12/q15；
- 放宽 0.20% 直到 q10 通过；
- 给 F1 再叠加一个没有独立依据的边界/面积/比例阈值；
- 因为没有跨 30/50 就直接跑真实删除 ATE；
- 把 Optimizer 已经剔除的 outlier 写成几何模块收益。

这些操作只会把开发集适配包装成方法进展。

## 3. 为什么暂不直接进入重型方法

已有 shadow 结果已经排除了若干轻量改造：

- simple depth flood fill：大面积链式传播；
- simple depth components：不是对象单位；
- Ji-style K-means/reprojection：保留为 baseline，不作主路线；
- frame-wise normal+distance segmentation：严重碎裂；
- scalar-strain graph connected components：目标 recall 低；
- 短轨迹/MapPoint 图：当前 unknown proxy 中支持不足。

完整 StaticFusion、DetectFusion、Dai point-correlation 或 dense scene flow 仍未被
这些简化失败否定，但它们需要区域/模型状态、长期图或稠密运动估计，已经属于
新的较重研究分支。未经更可靠 unknown-object 评价，立即投入会继续扩大代码而
无法判断收益。

## 4. 推荐下一步：G2-6E 独立评价代理

优先补齐的不是新 detector，而是 agent 可自动生成的 unknown-object reference。

推荐候选：

```text
Bonn 官方静态场景 3D model
+ 官方 camera GT pose
→ 渲染/投影 expected static depth
→ 与当前 registered depth 比较
→ 排除 online person mask
→ unknown foreground review proxy
```

来源边界：

- Bonn 数据集提供 static environment model/pose 的依据已在 G2-4 本地审计记录；
- 模型—观测不一致作为动态证据与 DynaSLAM/ReFusion 类工作有关；
- `[S]` 在本项目中只把它作为离线评价 proxy，不把它接入 SLAM detector；
- 它不是逐像素人工 GT，仍需在 true-static 和少量 RGB contact sheet 上审计
  配准、遮挡和深度噪声。

优点：

- 不要求用户逐帧手工画 mask；
- 与当前 temporal F1 的算法输入独立；
- 能覆盖低纹理箱子深度，而不只评价 ORB feature；
- 可明确区分“当前 bbox 代理太弱”和“F1 方法本身不足”。

当前本地没有找到官方 static model 文件，因此实施前需从 Bonn 官方数据源补齐，
并先冻结坐标链和评测 SPEC。

## 5. G2-6E 后的决策

### 若独立 proxy 显示 F1 在 unknown object 上有高精度连续富集

只研究一个有文献依据的最小多点/短时一致性聚合，仍保持 shadow；不得回到单点
阈值。

### 若 F1 对 unknown object 仍弱

停止 lightweight feature-filter 路线。若论文必须包含 geometry，则另立较重
分支，在以下两类中选择一类完整子问题：

- region hypothesis + motion probability（StaticFusion/DetectFusion 类）；
- multi-motion/scene-flow grouping（Jaimez/FlowFusion 类）。

这需要重新预算代码量、实时性和论文范围。

### 若静态模型 proxy 本身不可靠

冻结它为失败评价路线，不用其调 detector；再决定是否自采受控箱子序列。

## 6. 当前冻结状态

```text
semantic baseline                    frozen and usable
geometry measurements                available shadow library
single-feature hard filtering        stopped
G1-F / G1-D                          locked
Optimizer / g2o / backend            unchanged
third PoseOptimization               not added
recommended next                     G2-6E evaluation-only feasibility
```
