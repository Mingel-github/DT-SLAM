# S2 SIn 风格区域特征过滤规格

日期：2026-08-04  
状态：S2 最小安全收尾已通过限定验收；S3 可进入独立规格阶段

## 0. 2026-08-04 强遮挡收尾修订

`moving_obstructing_box` 表明，作者式“过滤后至少保留 250 个原始 ORB 特征”不足以
保证 ORB-SLAM2 仍有足够的静态地图匹配：首次 LOST 帧仍剩 686 个原始特征。因此不得
继续提高 250、增加 mask 面积上限或修改 detector 阈值。

S2 只增加一次最小的 tracking fail-open：

1. `TrackReferenceKeyFrame()` 低于原生 pre-pose 15 matches 时，撤销本帧新增的
   SIn geometry flags，重新执行同一次 BoW 匹配；
2. `TrackWithMotionModel()` 在原生宽窗口搜索后仍低于 20 matches 时，撤销 geometry
   flags，再执行一次宽窗口匹配；
3. `TrackLocalMap()` 中 `SearchLocalPoints()` 后，若当前 MapPoint associations 的理论
   最大内点数已低于原生成功条件（普通帧 30、重定位窗口 50），撤销 geometry flags，
   重新执行 `SearchLocalPoints()`，随后仍只运行原有的一次 `PoseOptimization()`；
4. 已处于 `LOST` 的帧，在 `Relocalization()` 前撤销 SIn geometry flags；
5. fail-open 只影响本帧 Tracking。Tracking 结束后恢复完全相同的 SIn geometry flags，
   继续禁止这些特征创建新 MapPoint；semantic flags 始终不撤销。

其中 `[L]` 是 SInDSLAM 的 250-feature restore 意图与 ORB-SLAM2 原生 15/20/30/50
成功条件；`[A/S]` 是把二者用于当前 `mvbDynamic` 接入方式。它不是新的动态检测算法，
不增加第三次位姿优化，也不承诺一定解决优化后内点不足。

若该策略不能减少强遮挡 LOST，或破坏 nonobstructing 正收益/静态序列，则撤销并记录为
负结果，不继续叠加新的安全阈值。

## 1. 目标

把 S1 native region detector 的有效动态区域转换为 ORB 特征动态状态，使其：

1. 在 `Track()` 前写入 `mCurrentFrame.mvbDynamic`，由已有
   `ORBmatcher` 动态标志检查在匹配阶段跳过；
2. 若既有或后续代码路径仍产生动态特征关联，则在已有
   `PoseOptimization()` 前由 `RemoveDynamicAssociations()` 兜底清除；
3. 通过既有 `mvbDynamic` 检查，禁止相同特征创建新的稀疏 MapPoint；
4. 不新增第三次位姿优化，不修改 `Optimizer.cc`、g2o、YOLO 或回环/局部建图
   算法。

S2 只验证 `D_feat` 和新稀疏 MapPoint 写入保护；不把当前区域 mask 用于深度图
或稠密地图，因此不等于 S3。

## 2. 文献与源码依据

- `[L]` SInDSLAM：dynamic mask 在 ORB 提取阶段剔除 mask 内特征；若剩余特征
  少于 250，则恢复未过滤特征，避免跟踪失效；
- `[A]` DT-SLAM：保留原 ORB 提取，不改 extractor；把 native region mask 映射
  到已有 ORB 特征，并复用当前工程 `ORBmatcher` 中已有的 `mvbDynamic` 跳过逻辑、
  `RemoveDynamicAssociations()` 兜底逻辑和 MapPoint 写入检查；
- `[S]` semantic 与 geometry 通过 `mvbDynamic` 并集进入既有过滤链。运行时可用
  是否提供 YOLO 模型形成 semantic-only、geometry-only、semantic+geometry；
- `[S]` SIn 的 250-feature fallback 在本项目只撤销**新增几何标志**，不撤销
  已有语义动态标志。

## 3. 配置与默认状态

```yaml
SInStyle.RegionFeatureFilterEnable: 0
SInStyle.RegionFeatureFilterMinimumRemainingFeatures: 250
```

- 默认关闭；
- 开启时必须同时启用 native RAG、native dense-flow residual 和 region
  classifier；
- 不允许 `reference_replay` 作者 mask 成为正式过滤输入；
- 第一轮只提供实验配置，不修改现有 baseline 配置。

## 4. 每帧规则

1. 对每个当前 ORB 特征读取 `regionDynamicResult.dynamicMask`；
2. 形成 geometry candidate vector；unknown/无效域不解释为 static，也不判
   dynamic；
3. 计算加入 geometry 后的剩余特征数；
4. 若剩余数小于 250，则 geometry fail-open，只保留原有 semantic 状态；
5. 否则将 geometry candidates 写入 `mCurrentFrame.mvbDynamic`；
6. `Track()` 中首先由既有 `ORBmatcher` 跳过这些特征；
7. `RemoveDynamicAssociations()` 只作为后续关联兜底，MapPoint 创建继续使用既有
   `mvbDynamic` 检查。

因此，CSV 中 `actual_removed_associations=0` 不表示过滤未生效：在当前执行顺序下，
候选通常已经在匹配入口被跳过，不会先形成关联再被兜底函数清除。

不增加面积、IoU、动态比例或 residual 阈值补丁。

## 5. 必须记录

逐帧 CSV 至少记录：

- filter enable/applied/state；
- geometry candidate features；
- semantic overlap；
- newly dynamic features；
- remaining features；
- 因 geometry flag 实际清除过关联的唯一特征数；
- tracking state after；
- `dynamic_decision=region_feature_filter` 或 `shadow_only`；
- 是否发生直接 SLAM 状态修改。

## 6. 验收顺序

1. 合成/短序列验证 fail-open 和 semantic 保留；
2. default-off TUM smoke 必须保持无 SIn 行为；
3. Bonn/TUM 短序列 geometry-only 必须产生非零新动态标志，且不导致早期 LOST；
   关联兜底清除数允许为零，但必须核对 `ORBmatcher` 的 pre-match 跳过路径；
4. 再做完整序列多次 ATE/RPE/FPS：baseline、semantic-only、geometry-only、
   semantic+geometry；
5. 若 geometry-only 或 OR 模式出现明显跟踪退化，保持默认关闭并回到区域表示
   决策；不得通过临时阈值掩盖。

S2 通过只表示区域证据可以安全参与稀疏定位和新 MapPoint 写入，不表示完整
未知动态对象 mask 或动态深度过滤已经完成。
