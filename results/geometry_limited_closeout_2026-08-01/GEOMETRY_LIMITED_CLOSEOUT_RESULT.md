# 当前轻量几何路线收尾评价结果

日期：2026-08-01
状态：轻量稀疏几何路线完成有限收尾；默认关闭；SInDSLAM 延后单独研究

## 1. 最终定位

当前可冻结的几何方法是：

> **基于 RGB-D 相机自运动补偿的稀疏光流残差候选过滤，以及 ORB-SLAM2
> Tracking/MapPoint 写入保护。**

流程为：

```text
ORB/LK 两帧对应
→ RGB-D + 初始位姿预测静态 ego-flow
→ observed flow - ego-flow
→ FB/LK/depth 质量检查
→ frozen q10 高残差候选
→ G1-F1 清除 SearchLocalPoints 后的候选 association
→ G1-M1 禁止通过安全条件的候选创建新 MapPoint
```

文献身份：

- `[L/A]` observed flow - camera-induced flow 来自 FlowFusion 类残余流思想；
- `[L]` forward-backward check 用于 LK 对应可靠性，不是动态标签；
- `[A]` 鲁棒尺度借鉴 Li & Lee 的静态权重思想，但 q10 是本项目工作点；
- `[S]` G1-F1/G1-M1、5% 上限、至少 100 个剩余深度特征和 fail-open 是本项目
  的 ORB-SLAM2 安全集成。

它不是 FlowFusion 复现，也不是对象级未知动态检测器。

## 2. 当前工作树回归

```text
geometric_warp_test                         PASS
rgbd_tum build                              PASS
git diff --check                            PASS
```

TUM fr1/xyz 30 帧回归：

| 模式 | 输入/轨迹 | actual FPS | 几何总开关 | 新刚体 shadow |
|---|---:|---:|---|---|
| 标准 ORB | 29/29 | 25.362 | OFF | 0 次 |
| frozen q10 geometry | 29/29 | 25.382 | ON | 0 次 |

两次短跑受数据时间戳约束约为 25.45 FPS，不能用来重新评价完整系统速度。几何
smoke 的 28 个 tracking row 均处于初始化后的 relocalization fail-open window，
没有实际删除；tracking/mapping 审计均 PASS。这只验证安全回归，不替代正式结果。

## 3. TUM walking 四模式正式结果

| 模式 | 轨迹 | ATE RMSE | RPE RMSE | FPS |
|---|---:|---:|---:|---:|
| ORB baseline（首轮） | 816/827 | 0.926133 m | 0.078290 | 28.423 |
| semantic-only | 827/827 | 0.019142 m | 0.011836 | 28.261 |
| geometry-only（首轮） | 587/827 | 0.533014 m | 0.096267 | 28.506 |
| semantic+geometry | 827/827 | 0.018693 m | 0.012298 | 27.083 |

semantic+geometry 相对同轮 semantic-only：

```text
ATE      -2.35%   单轮观察，不作稳定改善声明
RPE      +3.90%   变差
FPS      -4.17%
coverage 相同，827/827
```

geometry-only 另外两轮虽得到 827/827 轨迹，但 ATE 为 `0.819301/0.855549 m`，
没有稳定替代语义分支。它只适合作为“几何路径确实可脱离 YOLO 运行”的诊断模式。

## 4. Bonn 运动箱子正式结果

### 4.1 moving_nonobstructing_box 三轮中位数

| 模式 | ATE RMSE | RPE RMSE | FPS | 覆盖 |
|---|---:|---:|---:|---:|
| semantic-only | 0.152247 m | 0.051377 | 29.707 | 3/3 均 778/778 |
| semantic+geometry | 0.178114 m | 0.045061 | 29.544 | 3/3 均 778/778 |

组合模式相对 semantic-only：

```text
ATE   +16.99%   变差且轮间波动大
RPE   -12.29%   改善，但不足以证明整体更好
FPS    -0.55%
```

24 个粗箱框内有 4,564 个 quality-eligible 特征，但 q10 candidate 为 0。抽查的
实际移除点没有落入箱框，因此当前方法没有保护该非遮挡箱子的证据。

### 4.2 moving_obstructing_box 开发诊断

强遮挡条件下，17 个粗框内 5,530 个 eligible 特征产生 26 个 q10；最终有 2 个
带 MapPoint association 的候选在箱框内被实际删除。这证明当前路径偶尔能触及
未知箱子。

但是框内 q10 比率为 `0.472%`，框外为 `5.888%`，框外约高 12.48 倍。两点真阳性
线索不能抵消大量背景/人物/边界候选，不能声称形成可靠箱子检测器。

## 5. Tracking 与稀疏地图保护

当前工程事实：

- G1-F1 会在 `SearchLocalPoints()` 后真实清除候选 association；
- G1-M1 会阻止通过安全条件的候选创建新 MapPoint；
- applied 行中 candidate-created MapPoint 始终为 0；
- 候选比例过高、重定位窗口、首帧或支持不足时会 fail-open；
- 没有新增第三次 PoseOptimization；
- 没有修改 Optimizer、g2o、YOLO、LocalMapping 或 LoopClosing 算法。

四类序列共 12 次 G1-M1 正式运行均完整输出轨迹，2,441 个 valid-depth candidate
被否决写入，CSV invariant violation 为 0。真静态 fr1/xyz 的三轮中位 ATE/RPE
变化约为 `-0.66%/+0.35%`，没有观察到灾难性静态退化。

地图生命周期审计显示 G1-M1 确实减少候选临时写入，但 q10 不是动态 GT，且
ORB-SLAM2 会自然剔除大部分可疑 MapPoint。因此只能报告“写入保护执行成立”，
不能报告“地图已被定量净化”。

## 6. 已排除的轻量候选

研究过程中已经可复现地检查并停止：

- 单参考/多参考 depth warp 直接判决；
- all-seed flood fill 和区域支持率修补；
- Ji 2021 K-means＋cluster reprojection 主路线；
- 简单 depth component、normal+distance 单帧分段；
- 单点 q 工作点的跨数据泛化；
- 两帧局部刚性、Delaunay scalar graph；
- 稀疏 7 点刚体 hypothesis 及独立支持验证。

共同失败点是：存在运动不一致信号，但无法在当前轻量表示中可靠地从噪声、遮挡、
深度边界和位姿误差中恢复对象级未知动态区域。

## 7. 收尾判定

| 能力 | 最终状态 |
|---|---|
| 同步 YOLOv8-seg semantic baseline | **可用，推荐默认主线** |
| F1 sparse ego-flow 测量 | **可用** |
| G1-F1 association removal | **工程有效，实验性** |
| G1-M1 MapPoint admission veto | **工程有效，实验性** |
| semantic+geometry 组合运行 | **可用于实验，默认关闭几何** |
| geometry-only 独立使用 | **不建议，仅诊断** |
| 稳定 ATE/RPE 改善 | **未证明** |
| 可靠未知箱子检测 | **未完成** |
| 对象级 motion grouping | **未完成** |
| G1-D 动态深度区域/稠密过滤 | **未实现，不开放** |

推荐运行策略：

```text
普通使用/论文语义基线        semantic-only
消融和机制实验              semantic+geometry，明确 experimental
几何诊断                    geometry-only，不作为最终系统
默认配置                    当前 q10 filtering OFF
```

## 8. 与总计划的关系

本次关闭的是“当前轻量几何路线”，不是宣称原始类别无关几何目标已经成功：

```text
可靠运动组判决             轻量候选均已评价；未得到可靠对象判决，有限收尾
G1-F/G1-M 稀疏接入         已完成，实验可用、默认关闭
G1-D 深度区域过滤           未完成；随当前路线关闭
最终评价与冻结              当前轻量路线已完成
```

后续如果继续未知动态对象目标，应作为一次明确的重型方法升级研究开源
SInDSLAM，而不是继续给当前 F1/q10 添加阈值和邻域补丁。

## 9. SInDSLAM 后续接续点

SInDSLAM 可直接针对当前缺失的对象上下文：

```text
3-D K-means initial clusters
→ depth gradient / plane edge re-clustering
→ RAG + depth-histogram merge
→ dense optical-flow residual inside clusters
→ temporal mask prior
→ dynamic feature and depth mask
```

但它是约 `9 Hz` 的完整语义无关系统路线，包含稠密光流、重聚类、区域增长和时序
先验，不能包装成当前轻量模块的小修补。后续开始前应先单独审计开源代码许可证、
依赖、与现有 rectified RGB-D 域的接口以及可复用范围。

## 10. 证据索引

- `results/g1_release_2026-07-31/G1_SPARSE_FRONTEND_FOUR_MODE_RESULT.md`
- `results/g1_m1_2026-07-31/G1_M1_MAPPOINT_ADMISSION_FILTER_RESULT.md`
- `results/g1_map_quality_2026-07-31/G1_SPARSE_MAP_QUALITY_EVALUATION_RESULT.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_MOVING_NONOBSTRUCTING_BOX_RESULT.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_BOX_EVIDENCE_FUNNEL_AND_OBSTRUCTING_RESULT.md`
- `results/motion_group_support_validation_2026-08-01/MOTION_GROUP_SUPPORT_VALIDATION_RESULT.md`
- `results/geometry_limited_closeout_2026-08-01/default_smoke/`
- `results/geometry_limited_closeout_2026-08-01/geometry_smoke/`
