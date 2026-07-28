# G2-2G 规则深度网格采样 Shadow 规格

日期：2026-07-28

## 1. 决策背景

G2-2S 已证明 ORB feature-associated depth sampling 可将 K=5 多参考几何降到约
1 ms，但五参考任意比较覆盖只有约 0.47%–0.91%。G0-4F 又证明，直接把稀疏
positive seed 投到当前 ORB 特征附近不能得到足以实际过滤的 precision。

因此当前不能直接进入 G1-F 或 G1-D，也不能重新启用无边界 flood fill。

## 2. 本地文献核对

本地 DynaSLAM 论文、笔记和公开源码支持“特征关联深度观测作为几何种子”，但其
完整像素 mask 依赖后续深度区域生长。

本地 DetectFusion 笔记及论文表明，未知运动残差需要先与法线/深度不连续性形成的
几何段结合，再扩展到对象区域。SInDSLAM 也先做深度聚类、几何边缘切分和区域
重聚类，再在有边界的区域内传播残差。两者都不支持从零散 seed 在整幅深度连通图
上无限传播。

这些材料支持“未来应引入有边界的区域表示”，但完整法线分割、平面提取或重聚类
超出当前最小工单。

## 3. G2-2G 的身份

G2-2G 采用标准规则栅格抽样，在参考深度图上每隔 `s` 个像素保留一个有效深度：

```text
s ∈ {2,4,8}
→ 其余 G2-2R/G2-2S 条件保持不变
→ 稀疏 z-buffer
→ positive / negative / consistent 原始计数
```

它是 `[S]` 成熟工程抽样控制，不是 DynaSLAM、DetectFusion 或 SInDSLAM
复现，也不是论文主方法。目的仅是测出采样密度、像素覆盖和计算时间之间是否存在
可接受的中间工作点。

## 4. 固定变量

- K=5；
- covisibility 参考选择；
- 20 项参考缓存；
- 相同相机位姿和相机模型；
- 相同 0.10 m 诊断残差阈值；
- 相同 z-buffer 和 unknown/static/dynamic evidence 语义；
- 语义清理后的参考深度；
- 不生成动态 mask；
- 不影响 Tracking、Optimizer 或 Mapping。

唯一变量：

```text
dense
orb_depth
grid_depth stride=2/4/8
```

## 5. 输出

必须记录：

- effective sampling policy，例如 `grid_depth_s4`；
- 每参考有效样本、投影样本和有效比较；
- 五参考任意比较覆盖；
- G2 mean/median；
- 与同帧、同位姿、同参考 dense 结果的 positive presence precision/recall；
- 在线语义端到端 active time、actual FPS 和 deadline misses。

无比较像素必须继续保持 unknown。

## 6. 门控

本阶段只允许得出：

1. 存在可接受中间工作点：再研究有边界的区域表示；
2. 规则网格仍然覆盖不足：停止规则采样路线；
3. 覆盖增加但实时性失败：规则网格仅保留为密度对照。

本阶段不得：

- 写 `mvbDynamic`；
- 清除 `mvpMapPoints`；
- 新增 PoseOptimization；
- 过滤 MapPoint 或稠密深度；
- 对网格点做无边界膨胀；
- 将规则网格写成学术创新；
- 修改 YOLO、Optimizer、g2o 或后端。
