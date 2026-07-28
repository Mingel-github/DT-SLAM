# G2-3R0 深度边界区域划分 Shadow 规格

日期：2026-07-28

## 1. 任务边界

本阶段只回答：

> 当前深度图能否在低开销下被划分为有界几何区域，并避免 G0-3
> “任意 seed 命中巨大深度连通分量后吞并整图”的拓扑失败？

本阶段：

- 生成深度边界、区域标签和区域尺寸统计；
- 不读取几何 residual 或 YOLO mask；
- 不判断任何区域是动态还是静态；
- 不写入 `mvbDynamic`；
- 不影响 Tracking、Optimizer、MapPoint 或地图线程。

因此它是区域表示审计，不是动态检测结果。

## 2. 文献依据与适配边界

### 2.1 SInDSLAM 原型

本地论文：

`Qi et al., Semantic-Independent Dynamic SLAM Based on Geometric
Re-Clustering and Optical Flow Residuals, IEEE T-ITS, 2025`

其深度梯度边界定义为：

```text
δ_depth(u) = max |D(u_neighbour) - D(u)|
edge(u) iff δ_depth(u) > max(τ1 D(u), τ2)
```

论文表 II 给出：

```text
τ1 = 0.025
τ2 = 0.08 m
```

但原方法还包含三维 K-means 初始聚类、平面边缘补全和几何重聚类。

### 2.2 本阶段适配 `[A]`

G2-3R0 仅采用上述深度不连续边界。将有效、非边界像素按四邻域做
connected components，形成诊断区域：

```text
invalid depth   -> label -1
depth boundary  -> label -2
region pixel    -> label 0 ... R-1
```

这是受 SInDSLAM 深度边界公式约束的轻量连通域适配，不是
SInDSLAM 复现。`τ1=0.025、τ2=0.08 m` 仅作为原文起点，不在本阶段
宣称已经适合 DT-SLAM。

DetectFusion 使用深度不连续与法线相似性联合分割。法线分支本阶段不
实现，保留为深度边界不足时的有依据后续对照。

## 3. 输出

每帧输出：

- `boundary_mask`：`CV_8UC1`，255 表示深度不连续边界；
- `labels`：`CV_32SC1`，保留 invalid/boundary/region 三类状态；
- `region_sizes`；
- valid depth、boundary、assigned region pixel 数；
- region count；
- largest region pixel 数及其占 valid depth 的比例；
- top-5 region 占 valid depth 的比例；
- singleton 和小区域数量；
- 纯区域划分耗时。

## 4. 验收与停止条件

必须先通过：

1. 人工小矩阵确定性测试：深度跳变、无效深度屏障、统一平面；
2. TUM `fr3_walking_xyz`、`fr3_sitting_static`、`fr1_xyz` 同帧数审计；
3. 报告区域数量、最大区域比例、边界比例和运行时间分布；
4. 不产生 dynamic mask，不改变轨迹。

若最大区域仍长期接近整幅有效深度，本方法只能记录为新的失败区域
baseline；不得通过“区域内有一个 seed 就整块动态”的规则进入 G1。

若区域拓扑合理，下一阶段才研究：

> 在固定区域内聚合多参考几何证据，而不是继续无界 flood fill。
