# S2 SIn 风格区域特征过滤阶段结果

日期：2026-08-04  
状态：实验性 S2 已接入；nonobstructing 三轮通过，但 obstructing 外部验证发生长段 LOST；
默认仍关闭；S3 仍锁定

## 1. 本阶段完成了什么

S2 把 S1 的 native region dynamic mask 映射到当前帧 ORB 特征，并在安全条件满足时
写入 `mCurrentFrame.mvbDynamic`。这会复用当前工程已有的三条路径：

1. `ORBmatcher` 在匹配入口跳过动态特征；
2. `RemoveDynamicAssociations()` 在现有 `PoseOptimization()` 前兜底清除可能残留的关联；
3. RGB-D 初始化、关键帧创建和局部建图已有的 `mvbDynamic` 检查阻止这些特征创建
   新的稀疏 MapPoint。

没有新增第三次位姿优化，没有修改 `Optimizer.cc`、g2o、YOLO、LoopClosing 或
LocalMapping 算法。S3 的动态深度过滤没有开放。

## 2. 方法和安全条件

- `[L]` SInDSLAM：mask 内 ORB 特征不参与跟踪；剩余特征少于 250 时恢复全部特征；
- `[A]` 当前实现不改 ORB extractor，而是在特征提取后把 native region mask 映射到
  `mvbDynamic`，复用现有匹配和 MapPoint 写入接口；
- `[S]` geometry 引入后剩余特征少于 250 时只撤销新增 geometry 标志，不撤销已有
  semantic 标志；
- `[S]` unknown、无有效区域证据和 region unavailable 均 fail-open；
- `[S]` 过滤仅允许 `native_rag` 输出，禁止把独立作者 reference replay mask 直接
  作为正式过滤输入。

配置和环境变量均默认关闭：

```yaml
SInStyle.RegionFeatureFilterEnable: 0
SInStyle.RegionFeatureFilterMinimumRemainingFeatures: 250
```

## 3. 基础验证

### 3.1 Fail-open

在 10 帧 Bonn smoke 中把最少剩余特征人为设为 2000：

- 8 帧进入 `minimum_remaining_features_fail_open`；
- 1 帧 region unavailable；
- 1 帧无候选；
- `applied=0`、新增动态标志为 0、关联清除为 0。

这证明最少特征保护按设计撤销 geometry 影响。

### 3.2 短序列实际接入

- Bonn 30 帧：28/30 帧应用过滤，候选/新增标志总数 2741，剩余特征最少 1224，
  30 帧均保持 tracking OK；
- Bonn 150 帧：111 帧应用、38 帧无候选、1 帧 unavailable，新增标志总数 6310，
  剩余特征最少 1132，150 帧均保持 tracking OK。

`actual_removed_associations=0` 是预期的执行结果：S2 标志在 `Track()` 前写入，当前
`ORBmatcher` 已经跳过这些特征，因此它们通常不会先生成 MapPoint 关联再进入兜底清除。

## 4. Bonn moving_nonobstructing_box 三轮成对实验

### 4.1 协议

- 数据：`rgbd_bonn_moving_nonobstructing_box`，778 帧；
- geometry detector：相同 native gradient-only RAG、CPU DeepFlow、region classifier
  和时序 prior；
- OFF 与 ON 的唯一计划内变量：`SInStyle.RegionFeatureFilterEnable`；
- 无 YOLO，属于 geometry-only 接入验证；
- viewer 关闭；
- ATE/RPE 使用 Bonn groundtruth、SE(3) align 和 `--t_max_diff 0.02`；
- ATE 使用 translation RMSE；RPE 使用逐帧 translation RMSE；
- 每轮均匹配 776/778 poses，RPE 为 775 对。

默认 evo 的 10 ms 时间差只能匹配 485/778，本报告不采用该读数。

### 4.2 完整结果

| 轮次 | 过滤 | ATE RMSE (m) | RPE RMSE (m) | actual FPS | Tracking median (ms) | KeyFrames |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | OFF | 0.451998 | 0.027295 | 3.79762 | 236.378 | 70 |
| 1 | ON  | 0.023104 | 0.014206 | 3.84285 | 233.361 | 38 |
| 2 | OFF | 0.514344 | 0.022127 | 3.80211 | 236.458 | 71 |
| 2 | ON  | 0.022207 | 0.014381 | 3.84199 | 233.065 | 33 |
| 3 | OFF | 0.548142 | 0.018486 | 3.80742 | 236.083 | 75 |
| 3 | ON  | 0.022526 | 0.014420 | 3.83022 | 233.938 | 38 |
| **中位数** | **OFF** | **0.514344** | **0.022127** | **3.80211** | **236.378** | **71** |
| **中位数** | **ON**  | **0.022526** | **0.014381** | **3.84199** | **233.361** | **38** |

三轮 ON 的 ATE 都稳定在 2.2--2.3 cm，而三轮 OFF 为 45.2--54.8 cm。ON 也稳定减少
关键帧数量，并降低 RPE 中位数。由于检测链在 ON/OFF 中都运行，CPU DeepFlow 成本相同，
两种模式的速度接近；这不是一个 30 FPS 结果。

### 4.3 每帧过滤状态的可重复性

前两轮均有：

- 491 帧 `applied`；
- 286 帧 `no_candidates`；
- 1 帧 `region_unavailable_fail_open`；
- 候选总数约 7.62 万，候选非零帧数一致；
- 最少剩余 ORB 特征约 712--714，高于 250 安全线。

ORB 提取和 ORB-SLAM2 运行存在小幅非确定性，因此候选总数和轨迹不要求逐位一致。

## 5. 客观解释

当前证据支持：

1. S2 过滤代码路径真实影响了匹配、关键帧决策和最终轨迹；
2. 在该 Bonn 未知箱子序列及当前协议下，效果跨三轮重复，不能解释为一次偶然运行；
3. 区域级稠密运动证据在这一序列上明显优于此前轻量 LK 点级规则；
4. S2 的默认关闭和 fail-open 设计没有被放松。

当前证据**不支持**：

1. 当前 native mask 已达到像素级 GT 质量；作者 mask 只用于行为审计，不是 GT；
2. 该效果可推广到全部 Bonn、TUM、静态场景或 semantic+geometry 模式；
3. gradient-only RAG 等价复现了 SInDSLAM 的 PEAC 重聚类；Bonn 审计已经观察到部分箱子
   帧 under-coverage；
4. 动态深度和稠密地图已经得到保护；S3 尚未实现；
5. 当前 CPU 版本满足实时要求。

一个重要事实是 OFF 的 ATE 三轮自身在 0.452--0.548 m 间波动，说明 ORB-SLAM2 路线仍有
随机性；但 ON 三轮结果高度集中，因此该序列上的正向系统效应具有较强重复证据。

## 6. Bonn moving_obstructing_box 外部验证

为检查 nonobstructing 结果是否能迁移到强遮挡场景，使用相同 native detector 和同一
OFF/ON 控制变量运行 589 帧 `rgbd_bonn_moving_obstructing_box`。

| 模式 | GT 匹配 poses | ATE RMSE (m) | RPE RMSE (m) | Tracking OK / LOST | KeyFrames | actual FPS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| OFF | 588 | 0.546996 | 0.019121 | 589 / 0 | 69 | 3.33970 |
| ON | 388 | 0.177218 | 0.081901 | 389 / 200 | 54 | 3.35004 |

ON 的 ATE 只评价了能输出轨迹的截断/间歇恢复部分，不能与 OFF 全序列 ATE 作等价比较，
也不能据此写成改善。它从 input 296 首次 LOST，随后多次短暂重定位再丢失；共 200 帧
LOST。丢失前一帧和丢失帧分别有 88/817 个 geometry candidates，丢失帧剩余 686 个
ORB 特征，仍高于论文式 250-feature fallback。因此现有最少剩余特征条件没有防止
强遮挡场景中的过度过滤或地图/重定位支持退化。

首次 LOST 前的若干帧存在 400--700 个候选，region dynamic mask 约覆盖 8--13 万像素；
这与 native gradient-only RAG 在特定帧产生较大区域的已知风险一致。但这里尚无逐像素
GT，不能把所有被删区域直接称为静态误删。

### 6.1 丢失窗口的作者/native 同帧对照

从序列开头运行独立作者 CPU SIn，并只保存 input 280--311；另以 S2 filter OFF 运行
native detector 到 input 311，避免 LOST 截断 detector 输出。比较作者
`mask_pre_runner_dilate` 与 native `region_author_style`：

| 指标 | 结果 |
| --- | ---: |
| 对照帧 | 32 |
| 作者 mask 像素中位数 | 278391.5 |
| native mask 像素中位数 | 119381.5 |
| IoU 中位数 / 均值 | 0.422 / 0.403 |
| native/author 面积比中位数 | 0.424 |

因此“native mask 总面积比作者更大导致丢失”不成立。两者的区域位置和时序状态明显不同。

人工查看 frame 295/296（只作物理场景解释，不当作像素 GT）：

- frame 295 native 只保留很小的人腿/物体附近区域，作者 mask 覆盖右侧大块背景；
- 首次 LOST 的 frame 296，native mask 主要覆盖左侧正在移动的大柜体和中间行人，
  而作者 mask 主要覆盖右侧桌面、地面和门附近区域并漏掉大柜体；
- frame 296 native 有 817 个候选、剩余 686 个特征。

这说明当前 LOST 不能直接归因为 native 误杀静态背景。更符合证据的解释是：强遮挡时，
大柜体和行人占据主要纹理/特征支持；native 将其判为动态后，原始“剩余特征数”虽然仍有
686，但能够与静态地图形成稳定约束的匹配可能不足。作者系统保持轨迹也不等价于其 mask
更准确；它可能保留了柜体上的动态特征，从而以较差的全局精度维持跟踪。

这是动态 SLAM 的观测性边界：当动态前景遮挡大部分静态背景时，“更完整地删除动态点”与
“维持相机跟踪”可能冲突。下一步若研究 fail-safe，应以匹配/空间支持质量为依据，并明确
标为 `[A/S]` 的安全退化设计；不能把一个新的面积阈值包装成检测改进。

独立完整 SInDSLAM 的本地同序列结果为全 588 pose 匹配、ATE 0.274588 m、RPE
0.024518 m；它没有出现当前 native S2 的长段轨迹缺失。这说明“区域路线本身无效”不是
唯一解释，更直接的差异仍是当前 clean-room gradient-only RAG 没有忠实保留作者 PEAC
plane re-clustering 和完整区域行为。

该外部验证否决了“当前 native S2 已可普遍开放”的结论。不得仅把最少剩余特征从 250
改大，或新增最大删除比例来包装通过；这些可以作为安全机制研究，但不能替代区域表示
问题的解决。

## 7. 构建与回归

- `rgbd_tum`、`sin_style_region_dynamic_parity_test`、
  `sin_style_dense_flow_native_parity_test` 构建通过；
- region classifier 对作者参考 29 帧逐像素 parity：
  `raw_state_mismatch_total=0`；
- native CPU DeepFlow＋时序 prior 对作者复现协议 29 帧 parity：raw/observed
  flow 最大差为 0，homography 最大差 `2.38419e-06`，residual 最大差
  `9.53674e-06 px`，low/high mask mismatch 均为 0；
- parity 必须使用作者复现时采用的
  `fr3_walking_author_offset_minus_0p033_associations.txt`。一次误用普通 one-to-one
  association 的运行按预期产生大差异，已作为协议错误排除，不是算法回归；
- 原始 `TUM3.yaml`、9 个有效输入帧 default-off smoke 完成，新建 716 个点，日志中
  无 `SIn S1/S2` 运行输出；
- `git diff --check` 通过。

## 8. 阶段决定

S2 已证明代码链路真实有效，并在一个未知箱子序列上获得可重复正向结果；但强遮挡外部
验证失败，因此不能宣告 S2 完成或默认开放。当前决定是：

1. 保留 S2 实验代码、default-off 和 nonobstructing 正结果；
2. 作者/native 丢失窗口对照已经完成，证明问题不是简单的 mask 面积过大，而是区域位置、
   地图历史和剩余静态约束共同作用；
3. 下一项先规格化“静态匹配支持不足时如何安全退化”，依据 SIn 的 250-feature fallback
   与 ORB-SLAM2 既有最少匹配条件，不修改 detector 阈值；
4. 安全策略通过后，再运行真正静态序列及 TUM semantic/geometry 组合；
5. 上述条件未通过前，S3 动态深度输出继续锁定。

不得为了通过这些实验临时增加残差阈值、区域面积或删除比例补丁。

## 9. 2026-08-04 S2 最小安全收尾结果

### 9.1 实现边界

检测器、DeepFlow、region classifier、RAG 和时序参数全部冻结。新增逻辑只在
ORB-SLAM2 原生匹配成功条件已经不可能满足时，撤销**本帧新增的 SIn geometry
tracking flags**：

- reference-keyframe pre-pose：低于原生 15 matches；
- motion-model 宽窗口 pre-pose：低于原生 20 matches；
- local-map pre-pose：理论最大 MapPoint inliers 低于原生 30，或重定位窗口的 50；
- 已处于 LOST：在 Relocalization 前 fail-open。

semantic flags 从不撤销。Tracking 结束后恢复同一组 SIn geometry flags，使其继续作为
新 KeyFrame/MapPoint 的写入否决。没有新增 PoseOptimization，没有修改 Optimizer、g2o、
YOLO、LocalMapping 或 LoopClosing 算法。

### 9.2 强遮挡定位结果

`moving_obstructing_box` 的 312 帧窗口在“DT-SLAM＋SIn 风格区域过滤、无匹配保护”
首次 LOST 的 frame 296 精确触发一次
`motion_model_pre_pose` fail-open；该窗口全部保持 Tracking OK。完整 589 帧结果：

| 版本 | GT pose pairs | ATE RMSE (m) | RPE RMSE (m) | OK / LOST | fail-open | FPS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DT-SLAM＋SIn 风格区域过滤、无匹配保护 | 388 | 0.177218（截断，不可等价比较） | 0.081901 | 389 / 200 | 无 | 3.35004 |
| DT-SLAM＋SIn 风格区域过滤、带匹配保护 | 588 | **0.245857** | **0.016536** | **589 / 0** | **1** | 3.35235 |
| DT-SLAM 控制组、关闭区域特征过滤 | 588 | 0.546996 | 0.019121 | 589 / 0 | 0 | 3.33970 |

以上三行都是 DT-SLAM 内部实验，不包含作者原版 SInDSLAM。独立作者原版在同序列的
本地结果是完整 588 pose pairs、ATE 0.274588 m、RPE 0.024518 m；由于 detector、
特征接入和地图历史不同，只作为外部参照，不与表内结果合并命名。

唯一一次 fail-open 位于 frame 296，撤销 814 个 SIn geometry tracking flags；Tracking 后
全部恢复为 mapping veto。结果支持“原生 pre-pose 条件能够避免当前已观察到的灾难性
丢失”，但仍只是一轮完整强遮挡实验，不外推为全部序列结论。

### 9.3 正常动态与静态回归

| 序列 | 帧/有效轨迹 | ATE RMSE (m) | RPE RMSE (m) | LOST | fail-open | FPS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| moving_nonobstructing_box | 778 / 776 | 0.024146 | 0.014319 | 0 | 0 | 3.83341 |
| static_close_far（前 300 associations） | 298 / 296 | 0.022153 | 0.018673 | 0 | 0 | 4.43309 |

nonobstructing 的结果保持在此前三轮 ON 的 0.022--0.023 m ATE 附近，且安全逻辑从未
触发；说明它没有把普通动态场景普遍变成 fail-open。静态片段中 125 帧仍产生 region
feature flags，但没有 LOST 或安全回退；该片段只用于灾难性回归检查，不是完整静态序列
随机性对照。

### 9.4 S2 阶段结论

S2 在当前限定协议下完成：SIn 风格 region mask 能够真实过滤 ORB Tracking 和禁止新
MapPoint 写入；强遮挡时只在 ORB-SLAM2 原生匹配必败点发生一次 tracking fail-open，
同时保留 mapping veto。当前配置继续默认关闭，不能宣称已经跨全部场景验证，也不能把
tracking fail-open 解释为 detector 精度提高。

下一步可以进入 S3 的独立规格：复用现有 region mask 输出过滤深度/点云，并分别评价
动态残影与静态地图空洞；不在 S2 上继续增加阈值、LK 对照或新的安全模块。

### 9.5 构建与默认关闭回归

- `rgbd_tum` 与 `sin_style_region_dynamic_parity_test` 构建通过；
- 29 帧 region-classifier parity 仍为 `raw_state_mismatch_total=0`；
- 原始 `TUM3.yaml` 9 个有效输入帧 default-off smoke 新建 716 个 MapPoint，日志无
  `SIn S1/S2` 行为，actual FPS 26.7527；
- `git diff --check` 通过。
