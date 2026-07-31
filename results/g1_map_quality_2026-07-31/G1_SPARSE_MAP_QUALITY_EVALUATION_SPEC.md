# G1 稀疏地图质量评价 SPEC

日期：2026-07-31
状态：冻结后实现
性质：只读评价；不改变 tracking、mapping 或优化

## 1. 目标

ATE/RPE 只能评价轨迹，不能直接回答 G1-M1 是否减少了静态稀疏地图中的动态
污染。本评价将地图质量拆为三个维度：

1. **污染代理**：q10 候选创建的 MapPoint 最终有多少仍存活；
2. **静态地图保留**：最终 MapPoint、KeyFrame 和 observation 数量是否明显下降；
3. **系统可用性**：ATE、RPE、轨迹覆盖和 FPS 是否保持。

核心成对对照：

```text
G1-F1 q10 + G1-M0（只读，不阻止写图）
versus
G1-F1 q10 + G1-M1（实际写图保护）
```

这样只隔离 mapping admission 的影响，不把 tracking filter 差异混入比较。

## 2. 指标身份

- `[L]` ORB-SLAM2 使用 MapPoint observation、局部地图点剔除和 KeyFrame 图维护
  稀疏地图；
- `[L]` Ji 2021、DynaSLAM 的系统目标包括动态观测不进入静态地图；
- `[E]` 当前 G1-M0 已记录 q10 candidate MapPoint 创建，G1-M1 已记录实际 veto；
- `[S]` candidate MapPoint 的最终存活/替换追踪和地图摘要是本项目的只读评价。

q10 candidate survival 只能称为：

> suspicious/dynamic-enriched MapPoint contamination proxy

它不是逐点动态 GT，不能直接当作 precision、recall 或真实动态点比例。

## 3. 生命周期追踪

对 G1-M0 或 G1-M1 fail-open 路径中实际创建的每个 q10 candidate MapPoint，记录：

- 创建 frame、timestamp、feature index 和像素；
- 深度、MapPoint ID、创建模式；
- 当时的 tracking/mapping 状态；
- 系统关闭时原始 MapPoint 是否仍在最终 map；
- 是否被替换以及 replacement chain 的最终 MapPoint ID；
- replacement 最终是否仍在 map；
- 最终 bad 状态和 observation 数。

ORB-SLAM2 的 `Map::EraseMapPoint()` 只从集合移除指针，不立即 delete；系统
`Shutdown()` 又先等待 LocalMapping/LoopClosing 停止，再调用 tracking diagnostics。
因此生命周期只读检查必须位于现有 `SaveGeometryPoseDiagnostics()` 中。

## 4. 最终地图摘要

每次运行输出：

- final MapPoint 数；
- final KeyFrame 数；
- MapPoint observation 总数、均值、中位数；
- candidate-created 数；
- original candidate 直接存活数；
- replacement-chain 后仍存活数；
- candidate proxy survival ratio；
- candidate 被自然剔除/未存活数。

## 5. 解释规则

### 污染代理改善

G1-M0 中最终仍存活的 candidate proxy，是 G1-M1 原本可以阻止的可疑地图点。
G1-M1 的 applied veto 与 fail-open survivor 必须分开报告。

### 静态地图保留

不能只追求 MapPoint 越少越好。若 G1-M1 同时造成：

- final MapPoint/observation 大幅下降；
- 静态 fr1/xyz ATE/RPE 明显退化；
- 轨迹覆盖下降；

则不能称为地图质量提升。

### 不允许的表述

- 不把 candidate proxy 全部称为真实动态点；
- 不把 MapPoint 数减少本身称为地图更干净；
- 不用 Viewer 主观观感替代数值；
- Bonn bbox/person proxy 只能作为补充，不能称为未知箱子 GT。

## 6. 第一轮实验

先各跑一轮成对对照：

```text
TUM walking
TUM sitting_static
Bonn balloon
TUM fr1/xyz
```

若生命周期不变量通过且结果具有解释力，再决定是否需要三轮重复。已有 G1-M0
和 G1-M1 的 ATE/RPE/FPS 三轮结果继续作为轨迹稳定性依据，不无条件重跑全部
组合。

## 7. 实现边界

```text
default OFF
read-only diagnostics
no mvbDynamic mutation
no mvpMapPoints mutation
no MapPoint/KeyFrame/Map modification
no Optimizer/g2o/YOLO/LocalMapping change
no extra PoseOptimization
```

只允许在 `G1-F1 q10 + (G1-M0 xor G1-M1)` 下启用。
