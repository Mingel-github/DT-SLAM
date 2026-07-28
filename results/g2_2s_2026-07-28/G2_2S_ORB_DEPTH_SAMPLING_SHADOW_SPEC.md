# G2-2S ORB 特征关联深度采样 Shadow 规格

日期：2026-07-28

## 1. 本阶段问题

G2-2R 表明，全分辨率稠密 depth warp 的成本随参考数近似线性增长：

```text
K=1: 3.34–4.34 ms
K=2: 6.08–7.96 ms
K=5: 13.91–18.66 ms
```

G2-2S 只回答：

> 固定 G2-2R 的共视参考选择、K=5、位姿、z-buffer、残差定义和证据分类，
> 将每个参考帧从全深度像素改为 ORB 特征关联的有效深度样本，能节省多少计算，
> 又损失多少几何覆盖和 person proxy 证据？

本阶段不选择动态阈值，不生成动态 mask，不影响 SLAM。

## 2. 文献依据

### `[L]` DynaSLAM

DynaSLAM 的多视图几何并非对 5 个参考帧各执行一次全分辨率稠密 warp。其公开
`Geometry.cc::ExtractDynPoints()` 对每个参考帧遍历 `refFrame.mvKeys`，读取
关键点对应的有效深度，再投影这些稀疏 feature-associated depth observations。

本地证据：

- `/home/zhu/Desktop/papers/2018_DynaSLAM_Tracking_Mapping_Inpainting.pdf`
  的 Multi-view Geometry；
- `/home/zhu/Desktop/paper_notes/dynaslam.md`；
- `/home/zhu/dynaslam_ws/DynaSLAM/src/Geometry.cc:117` 至约 `:144`。

### 不采用的相邻做法

- NGD-SLAM 的 `15×15` 网格用于在已有动态 mask 内选择 FAST 点并做 LK 光流传播，
  不是深度 warp 采样依据；
- SInDSLAM 的“每 5 帧”用于稠密地图重投影精修频率，不是逐帧几何前端的空间
  采样依据；
- Ji 2021 的全深度 K-means 是区域 baseline，已经实测约 58 ms，不属于本阶段。

## 3. 当前适配及非复现边界

G2-2S 的 `orb_depth` 模式：

```text
关键帧写入几何缓存时
→ 取 Frame::mvKeys 的原始 RGB/depth 像素坐标
→ 去重
→ 只保留 referenceDepth 中有效且未被语义 mask 清除的深度
→ 在后续共视参考 warp 中只投影这些深度样本
→ 多个样本命中同一当前像素时仍执行最近深度 z-buffer
→ 对实际有比较的像素累计 positive/negative/consistent 票
```

这不是 DynaSLAM geometry reproduction，因为当前版本不实现：

- DynaSLAM 的参考排序；
- `30°` parallax gate；
- 当前深度局部 patch 搜索；
- depth variance gate；
- DynaSLAM region growing；
- DynaSLAM 的 `0.4/0.6 m` 阈值。

准确身份为：

> `[A] DynaSLAM feature-associated depth sampling adapted to the existing
> G2-2R covisibility-selected signed-depth evidence framework.`

## 4. 受控变量

固定：

- `K=5`；
- `covisibility` 参考策略；
- 20 项关键帧深度缓存；
- `0.10 m` 诊断残差阈值；
- 相同 Tcw 和几何相机模型；
- 相同 z-buffer；
- 相同语义参考深度清理；
- 相同 evidence histogram；
- unknown 与 static 分离。

唯一变量：

```text
dense
vs
orb_depth
```

## 5. 输出和验收指标

每个参考记录：

- cached ORB-depth sample count；
- valid reference sample count；
- projected sample count；
- unique z-buffer pixel count；
- valid comparison count；
- prediction/comparison coverage；
- total runtime。

在 walking、sitting、fr1/xyz 相同 199 帧短序列上比较：

1. G2 mean/median/p95；
2. active total 和 actual FPS；
3. 选满 K 的帧率；
4. full-density 相对运行时间；
5. comparison coverage；
6. person proxy precision、conditional recall、unconditional capture；
7. fr1 静态背景选择率；
8. sitting 低动态 proxy 外选择率。

端到端时间使用 `dense` 和 `orb_depth` 独立运行。由于不同运行速度会影响
ORB-SLAM2 异步 LocalMapping 的关键帧生成时序，证据保持还必须增加一组
same-reference audit：

```text
DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT=1
```

该诊断只在 `orb_depth` 运行内部，对同一当前帧、同一位姿和同一组已选参考额外
计算一份 dense histogram。其端到端时间不用于实时性评价；关闭环境变量后没有
额外 dense 计算。

为避免把先前 G0 单参考帧稠密 shadow 的成本计入 G2-2S 部署候选，还需增加一次
隔离计时：

```yaml
Geometry.SingleReferenceShadowEnable: 0
Geometry.MultiReferenceShadowEnable: 1
Geometry.MultiReferenceSamplingPolicy: "orb_depth"
```

该开关默认值必须为 `1`，因此旧配置及既有实验路径不变；只在 G2-2S 专用配置中
关闭。隔离计时仍保留在线 CUDA 语义、Tracking、LocalMapping 和数据读取，因而
只能用于判断“同步语义＋G2-2S”的端到端速度，不能把总帧耗时全部归因于几何。

## 6. 门控

本阶段只能得出以下结论之一：

- 采样显著提速，但覆盖不足：保留为 sparse evidence baseline；
- 采样提速且证据保持可接受：进入下一轮 shadow 复核；
- 采样未显著提速：停止该路线。

无论结果如何，本阶段都不得：

- 写入 `mvbDynamic`；
- 清除 `mvpMapPoints`；
- 新增 `PoseOptimization`；
- 阻止 MapPoint 创建；
- 过滤稠密深度；
- 修改 YOLO、Optimizer、g2o 或后端；
- 将无比较像素解释为静态。
