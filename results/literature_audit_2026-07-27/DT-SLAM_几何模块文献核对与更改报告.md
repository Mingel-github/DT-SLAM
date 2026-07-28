# DT-SLAM 几何模块文献核对与更改报告

日期：2026-07-27  
用途：提交给其他 AI 或研究人员做交叉审批  
范围：文献、官方实现、当前 DT-SLAM 实现与实验记录的核对；本报告不修改 SLAM 源码

修订记录：

- v1：完成文献、DynaSLAM 官方代码、当前实现和 G0-3/G0-3R 实验的首轮核对；
- v2：吸收第二轮交叉审阅中经本地代码、数据和原始资料支持的内容，将下一步顺序改为“坐标域检查 → direct seed 审计 → region propagation 对照”，并拆分 feature/depth 两条验证路径。
- v3：完成 G0-2C 输入门控；确认 TUM3 零畸变域可用，发现原 walking_xyz association 的 32 次 depth 复用，并生成 827 对一对一诊断 association；非零畸变 geometry 现在 fail-fast。
- v4：完成 G0-2P 双路径诊断基础设施和 walking_xyz 首轮对照；确认 residual 对位姿来源敏感，但动态序列中 GT residual 并不必然更小，因此仍需真正静态数据和逐像素标注。

## 1. 审查结论

当前研究方向不需要推翻，但必须收紧方法归属和开发门控。

可以继续保留：

1. G0-1：单参考帧稠密深度 warp、z-buffer、预测深度和 signed depth residual；
2. G0-2：validity mask、正残差 seed、负残差诊断以及无证据区域的显式区分；
3. 几何模块位于初始位姿与 `TrackLocalMap()` 之间的 shadow 接入位置；
4. Ji et al. 2021 的 K-means 深度聚类与簇级重投影误差作为独立论文对照；
5. 不修改 `Optimizer.cc`、g2o 和后端 BA 的工程边界。

必须冻结或更正：

1. G0-1 不是 DynaSLAM 几何模块复现，只是受其多视图深度一致性思想启发的单参考帧稠密改造；
2. 当前 `0.10 m` residual threshold 没有直接文献依据，只是 shadow 阶段的实验参数；
3. G0-3 的四邻域局部深度连续传播不能作为动态区域 mask；
4. G0-3R 的区域面积、seed ratio 和 residual median 暂未显示稳定可分性，不能据此补一个经验阈值；
5. 不应无条件增加第三次 `PoseOptimization()`；当前 ORB-SLAM2 路径已经包含初始位姿优化和 `TrackLocalMap()` 内的再次优化；
6. 30 FPS 不能由文献中的“real-time”推导，必须以 DT-SLAM 端到端计时为准。

第二轮审查后，下一步优先级修订为：

```text
坐标域、畸变和时间同步审计
→ GT pose / SLAM pose 的 direct seed 敏感性诊断
→ 少量人工标注下的 direct seed 精度审计
→ 相同 seed 上的 region propagation 局部消融
→ feature-level 与 depth-region 两条路径分别设门控
```

推荐的当前状态定义为：

```text
G0-1 / G0-2：通过工程验证的几何证据提取 shadow
G0-3 / G0-3R：完成的失败实验，默认关闭，不进入过滤
G1：未开始
```

## 2. 证据等级

为避免把推测写成事实，本报告统一使用以下标签：

| 标签    | 含义                     |
| ----- | ---------------------- |
| `[P]` | 原始论文正文明确支持             |
| `[C]` | 作者官方源码明确支持，但论文未必写到实现细节 |
| `[O]` | 当前 DT-SLAM 代码或本地实验直接观察 |
| `[E]` | 通用、成熟的工程实现选择           |
| `[A]` | 在文献原型上的明确改造，不是原方法复现    |
| `[H]` | 尚需实验验证的假设，不应写成方法事实     |

文献笔记只用于定位和交叉检查。发生冲突时，以原始论文正文和作者官方源码为准。

## 3. 本次核对材料

### 3.1 本地原始论文

- `/home/zhu/Desktop/papers/2018_DynaSLAM_Tracking_Mapping_Inpainting.pdf`
- `/home/zhu/Desktop/papers/Ji 等 - 2021 - Towards Real-time Semantic RGB-D SLAM in Dynamic Environments.pdf`

### 3.2 本地笔记

- `/home/zhu/Desktop/paper_notes/dynaslam.md`
- `/home/zhu/Desktop/paper_notes/Ji2021_RealTime_Semantic_RGBD_SLAM.md`
- `/home/zhu/Desktop/paper_notes/comparison_23_papers.md`

### 3.3 本地官方实现

- `/home/zhu/dynaslam_ws/DynaSLAM/include/Geometry.h`
- `/home/zhu/dynaslam_ws/DynaSLAM/src/Geometry.cc`
- `/home/zhu/dynaslam_ws/DynaSLAM/src/Tracking.cc`

### 3.4 本地 DT-SLAM 代码与实验记录

- `DT-SLAM/include/GeometricDynamicDetector.h`
- `DT-SLAM/src/GeometricDynamicDetector.cc`
- `DT-SLAM/src/Tracking.cc`
- `results/g0_2_2026-07-26/DT-SLAM_G0-2_Evidence_实施与验证记录.md`
- `results/g0_2v_2026-07-26/DT-SLAM_G0-2V_可视化诊断实施与检查记录.md`
- `results/g0_3_2026-07-27/DT-SLAM_G0-3_深度连通候选实施与失败分析.md`
- `results/g0_3r_2026-07-27/DT-SLAM_G0-3R_区域证据审计记录.md`

### 3.5 本地缺失后使用的作者机构原文

- ReFusion，IROS 2019：  
  <https://www.ipb.uni-bonn.de/pdfs/palazzolo2019iros.pdf>
- Li and Lee，RA-L 2017：  
  <https://mediatum.ub.tum.de/doc/1375854/document.pdf>
- Bonn RGB-D Dynamic Dataset 官方页面：  
  <https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/index.html>
- TUM RGB-D Dataset 官方序列页面：  
  <https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download>

辅助原始链接：

- DynaSLAM：<https://arxiv.org/abs/1806.05620>
- DynaSLAM 官方仓库：<https://github.com/BertaBescos/DynaSLAM>
- Ji et al. 2021：<https://arxiv.org/abs/2104.01316>

### 3.6 关键源码定位

以下行号对应本次审查时的本地工作树，后续修改后可能漂移：

| 证据                                                         | 文件与位置                                             |
| ---------------------------------------------------------- | ------------------------------------------------- |
| 当前 RGB-D 深度转为 `CV_32F` 米制、Frame 构造、semantic mask 归一化       | `DT-SLAM/src/Tracking.cc:303-346`                 |
| 成功跟踪后清理 semantic depth 并更新 geometry reference              | `DT-SLAM/src/Tracking.cc:348-361`                 |
| 当前 geometry shadow 主调用                                     | `DT-SLAM/src/Tracking.cc:404-502`                 |
| geometry 位于初始跟踪和 `TrackLocalMap()` 之间                      | `DT-SLAM/src/Tracking.cc:685-801`                 |
| 当前相对位姿、dense warp 和 z-buffer                               | `DT-SLAM/src/GeometricDynamicDetector.cc:159-269` |
| signed residual 与 validity                                 | `DT-SLAM/src/GeometricDynamicDetector.cc:271-339` |
| G0-2 evidence classification                               | `DT-SLAM/src/GeometricDynamicDetector.cc:343-440` |
| G0-3 固定米制四邻域扩张                                             | `DT-SLAM/src/GeometricDynamicDetector.cc:442-634` |
| raw `mvKeys`、undistorted `mvKeysUn` 与 RGB-D 深度采样            | `DT-SLAM/src/Frame.cc:414-444,653-686`            |
| semantic mask 明确与原始 `mvKeys` 对齐                                | `DT-SLAM/include/Frame.h:135-166`                 |
| DynaSLAM geometry 总流程和关键帧 DB                               | `DynaSLAM/src/Geometry.cc:29-53`                  |
| DynaSLAM 参考帧选择                                             | `DynaSLAM/src/Geometry.cc:55-97`                  |
| DynaSLAM feature-associated depth seed                     | `DynaSLAM/src/Geometry.cc:100-389`                |
| DynaSLAM region mask、阈值与 dilation                          | `DynaSLAM/src/Geometry.cc:393-428`                |
| DynaSLAM running-mean region growing                       | `DynaSLAM/src/Geometry.cc:1020-1109`              |
| DynaSLAM RGB-D 的 `LightTrack → Geometry → Frame重建 → Track` | `DynaSLAM/src/Tracking.cc:264-301`                |

## 4. 仓库与工作树快照

审查时的仓库根目录：

```text
/home/zhu/dynaslam_ws
```

Git 状态：

```text
branch: main
HEAD:   4d44f623aaf893602e1261e394a68f412e7d7013
commit: 4d44f62 Document G0-1 validation results
```

G0-2、G0-2V、G0-3 和 G0-3R 仍主要存在于本地未提交改动中。审查时检测到的相关修改文件包括：

```text
DT-SLAM/Examples/RGB-D/TUM3.yaml
DT-SLAM/Examples/RGB-D/geometric_warp_test.cc
DT-SLAM/include/GeometricDynamicDetector.h
DT-SLAM/include/Tracking.h
DT-SLAM/src/GeometricDynamicDetector.cc
DT-SLAM/src/Tracking.cc
results/experiments.md
```

因此，本报告针对“本地工作树状态”，不能只根据 GitHub 的已推送 commit 复现。

## 5. 关键文献核对

### 5.1 DynaSLAM 论文实际支持什么

DynaSLAM 的 RGB-D 几何部分包含以下明确步骤：

1. `[P]` 为当前帧选择与其重叠最高的历史关键帧；

2. `[P]` 实验中使用 5 个参考关键帧；

3. `[P]` 投影参考关键帧中由 ORB-SLAM2 特征提取器得到的 keypoint，并利用该点深度和相机位姿得到当前帧预测位置及预测深度；

4. `[P]` 视差角大于 `30°` 的点被忽略；

5. `[P]` 使用
   
   ```text
   Δz = z_proj - z_current
   ```
   
   并在 `Δz > τz` 时将点视为动态；

6. `[P]` 论文用 30 张人工标注 TUM 图像选择阈值，报告 `τz = 0.4 m`；

7. `[P]` 高深度 patch 方差的动态点会重新标为静态，以减轻边界误判；

8. `[P]` 从动态点在深度图中做 region growing，形成像素级 mask；

9. `[P]` 几何 mask 与 CNN mask 可以联合使用。

因此，下列表述成立：

> DynaSLAM 提供了“历史关键帧深度观测投影到当前帧，使用正深度差生成动态证据，再进行深度区域扩展”的直接文献依据。

下列表述不成立：

> 当前 DT-SLAM 的单上一帧、全深度像素、z-buffer 稠密 warp 是 DynaSLAM 几何模块复现。

当前方法缺少或改变了 DynaSLAM 的：

- 5 个重叠关键帧；
- 稀疏 feature-associated depth observation；
- `30°` 视差门控；
- 对当前投影位置考虑重投影误差的邻域深度读取；
- 深度 patch 方差门控；
- DynaSLAM 官方代码中的 region-growing 具体实现。

准确归属应为：

```text
[A] 受 DynaSLAM 多视图深度一致性启发的
    single-reference dense depth-warp shadow measurement
```

### 5.2 DynaSLAM 论文与官方源码并不完全一致

官方 `Geometry.cc` 进一步显示：

| 项目                | 论文           | 官方源码                      |
| ----------------- | ------------ | ------------------------- |
| 参考帧数量             | 5            | `MAX_REF_FRAMES = 5`      |
| 数据库大小             | 未作为核心公式强调    | `MAX_DB_SIZE = 20`        |
| 几何启动条件            | 多参考历史        | 至少 5 个数据库元素               |
| seed 输入           | ORB 特征关联深度   | 参考帧 ORB keypoint 位置读取有效深度 |
| 当前深度搜索            | “考虑重投影误差”    | 投影点附近 `41×41` 窗口          |
| 视差阈值              | `30°`        | `30`                      |
| depth residual 阈值 | `0.4 m`      | `0.6 m`                   |
| patch 方差          | 有门控          | `0.001`                   |
| 区域生长阈值            | 未在论文主公式固定    | `0.20 m`                  |
| 膨胀                | 论文仅描述区域 mask | `31×31` 椭圆核               |
| 尺寸                | 方法描述不限定      | 多处硬编码 `640×480`           |

这意味着：

1. 不能把官方源码中的 `0.6 m` 写成论文参数；
2. 不能把论文的 `0.4 m` 写成官方代码参数；
3. “复现 DynaSLAM”必须先说明复现论文描述还是复现公开代码；
4. 官方实现适合当实现参考，但不是可以无审查照搬的规范。

另外，官方 `GetRefFrames()` 将归一化距离和旋转距离按 `0.7/0.3` 合并后执行降序排序，这在表面上与论文的“最高 overlap”描述不一致。该点应标为“代码—论文不一致，需数值验证”，不应在未运行验证前直接定性为 bug。

### 5.3 当前 z-buffer 的来源

当前 DT-SLAM 将参考帧所有有效深度像素投影到当前帧，当多个点落入同一目标像素时保留最小正深度。

这是：

```text
[E] 标准可见性和前表面处理
```

它不是 DynaSLAM 独有模块，也不构成论文创新。准确表述应为：

> Dense forward warp uses standard z-buffer visibility handling.

当前使用 `cvRound()` 的最近像素 rasterization 会产生投影空洞，这是当前实现性质，不应被解释为静态或动态证据。`validComparisonMask` 正是避免这种误解释的必要接口。

### 5.4 signed depth residual 与正残差 seed

当前定义：

```text
r_z = D_pred - D_current
```

当历史静态表面预测为 3 m，而当前新进入的近物体为 1 m 时：

```text
r_z = 3 - 1 = +2 m
```

因此，正残差作为“当前有更近表面进入”的保守动态 seed，具有：

- `[P]` DynaSLAM 的直接深度差依据；
- `[E]` 遮挡几何的直接物理解释。

但必须限制结论：

```text
positive residual = dynamic evidence
positive residual ≠ 已确认动态对象
```

正残差还可能受以下因素影响：

- 初始位姿误差；
- 深度噪声；
- 遮挡边界；
- forward warp 离散化；
- 参考深度污染；
- 当前深度空洞邻域；
- 参考帧与当前帧基线及视角变化。

这些因素在当前 TUM 可视化中与大面积 seed 同时出现，但各因素的因果贡献尚未被单独消融，因此应标 `[H]`，不能写成已经定量证明的原因排序。

### 5.5 负残差与 disocclusion

当历史帧中近物体占据像素，而当前帧该物体移开并露出更远背景时，可能出现负残差。

当前将其保留为 `negativeDiagnosticMask` 而不直接判动态，是：

```text
[E/A] 基于遮挡/显露关系的保守工程设计
```

它不是 DynaSLAM 论文声称的方法贡献。当前阶段也没有证明所有负残差都是 disocclusion。

准确表述应为：

> Negative residuals are retained as diagnostics because they are ambiguous between disocclusion, pose/depth errors, and other inconsistencies.

### 5.6 “无证据”不等于静态

当前接口显式保存：

- `validComparisonMask`；
- `consistentEvidenceMask`；
- `positiveSeedMask`；
- `negativeDiagnosticMask`。

这是正确的工程边界。

几何状态至少应区分：

```text
unknown / no geometric evidence
geometrically consistent under the current test
positive inconsistency evidence
negative inconsistency evidence
```

其中：

```text
geometrically consistent under one comparison
≠ 已证明长期静态
```

该四分法主要是 `[E]` 的软件接口和不确定性建模，不应包装成某篇论文的复现，也不应在没有进一步建模时声称为新的概率动态分类方法。

### 5.7 ReFusion 的 region-growing 公式核对

需要更正此前“具体公式不应归于 ReFusion”的意见。

ReFusion 原文 Algorithm 1 明确给出：

```text
if ||D(p) - D(n)|| < θ · D(p)
    add n to the queue
```

因此：

```text
|D(p)-D(n)| < θ·D(p)
```

确实可以准确归于 ReFusion 的 depth-aware flood fill。

ReFusion 的完整上下文是：

1. `[P]` 当前 RGB-D 帧先与 TSDF 静态模型做初始 registration；
2. `[P]` 根据相对于模型的 registration residual 得到 raw residual mask；
3. `[P]` raw mask 先形态学处理；
4. `[P]` 用上述深度感知 flood fill 扩张；
5. `[P]` 去除动态 mask 后做第二次 registration；
6. `[P]` 过滤后的 RGB-D 才融合进 TSDF；
7. `[P]` 系统还显式维护历史 free space。

因此，虽然公式归属明确，但直接把它移植到当前 DT-SLAM 仍然属于 `[A]`：

- ReFusion 的 seed 来自“当前帧对静态 TSDF 模型的 registration residual”；
- DT-SLAM 的 seed 来自“上一成功帧对当前帧的 signed depth-warp residual”；
- 两者的参考模型、残差统计、遮挡关系和错误分布不同。

不能由 ReFusion 的成功直接推出该 flood fill 会在当前 seed 上成功。

### 5.8 当前 G0-3 与 DynaSLAM/ReFusion 的准确关系

当前 DT-SLAM G0-3：

```text
positive seed
+ 当前深度 CV_32F
+ 四邻域
+ validComparisonMask 门控
+ |D_neighbor - D_current| <= 0.05 m
+ 所有 seed 可达区域的并集
```

它不是 DynaSLAM region growing 的复现：

- DynaSLAM 官方代码维护区域运行均值；
- 每次从 frontier 中选与区域均值最接近的像素；
- 当最小 frontier 差值达到阈值时停止；
- DynaSLAM seed 更稀疏，并有视差和 patch 方差门控。

它也不是 ReFusion Algorithm 1 的精确复现：

- ReFusion 使用相对阈值 `θ·D(p)`；
- 当前使用固定米制阈值 `0.05 m`；
- seed 来源和前处理不同；
- 当前额外限制在 `validComparisonMask` 内。

最准确的归属是：

```text
[A] 受深度感知 region growing 启发的
    fixed-metric local-continuity candidate expansion
```

#### 5.8.1 当前 G0-3 的图论解释

对当前固定阈值 `δ=0.05 m`，可以把有效比较像素构造成图：

```text
V = validComparisonMask 内且当前深度有效的像素
E = 四邻域中满足 |D(p)-D(n)| <= δ 的边
S = positiveSeedMask 内的 seed
```

当前 G0-3 的输出精确等于：

```text
所有包含至少一个 s∈S 的连通分量的并集
```

因此，“局部深度连续”只定义深度图中的可达性，不定义共同运动归属。斜面、地板或缓慢变化的大平面即使累计深度变化很大，也可能通过一系列局部小变化形成大连通分量。

这给出两个可直接测量、无需先修改算法的结构指标：

1. 最大深度连通分量占 `V` 的比例；
2. 每个连通分量是否被至少一个 positive seed 命中，以及被命中分量的面积并集。

当前约 `95%–97%` 的扩张结果与“大连通分量被分散 seed 命中”的机制相容，但本轮日志尚未直接输出上述两个结构统计。因此，“链式传播是主要原因”目前应写成由代码结构和可视化支持的解释，而不是已经完成单因素消融的因果结论。

### 5.9 G0-3/G0-3R 失败的客观证据

50 帧短实验中，每个序列产生 49 个几何结果：

| 指标                            | walking_xyz | sitting_static |
| ----------------------------- | -----------:| --------------:|
| region / valid comparisons 平均 | 97.01%      | 95.54%         |
| region pixels 平均              | 205981.0    | 205385.1       |
| seed 到 region 增长倍率平均          | 27.15       | 31.37          |
| 仅 G0-3 region grow 平均耗时       | 2.828 ms    | 2.785 ms       |
| G0-3R region grow + 统计        | 4.924 ms    | 4.868 ms       |
| G0-3R geometry total          | 8.400 ms    | 8.290 ms       |

G0-3R 中，walking 与 sitting_static 的 top-region `positive ratio` 范围明显重叠。区域面积、positive ratio 和 residual median 均未显示可直接冻结的分割阈值。

因此：

```text
[O] 当前实现不能输出可用于 SLAM 过滤的动态区域 mask。
```

需要额外注意：

`fr3/sitting_static` 中的 `static` 描述相机运动方式，而不是“画面无动态人物”。Ji et al. 原文也明确将 `fr3/sitting` 描述为两人坐在桌边并有轻微身体运动。因此它只能作为低动态对照，不是真正静态负样本。

当前本地 `TUM/` 只有：

```text
rgbd_dataset_freiburg3_sitting_static
rgbd_dataset_freiburg3_walking_halfsphere
rgbd_dataset_freiburg3_walking_static
rgbd_dataset_freiburg3_walking_xyz
```

当前本地没有真正无动态的 TUM 序列。Bonn 的 box/person/balloon/crowd 数据当前仍是 zip，尚未纳入本次证据审查。

### 5.10 Ji et al. 2021 的准确定位

Ji et al. 的几何模块明确是：

1. `[P]` 将每张新深度图用 K-means 分成 `N` 个三维近邻区域；
2. `[P]` 对每个 cluster 统计其内部匹配特征相对三维 MapPoint 的平均鲁棒重投影误差；
3. `[P]` 相对其他 cluster 误差更大的 cluster 被标为动态；
4. `[P]` 动态 cluster 内特征不参与位姿估计；
5. `[P]` 实验中 `640×480` 使用 `N=24`；
6. `[P]` 论文没有给出一个可直接移植的通用数值动态阈值，只说选取相对较低、依赖匹配特征平均重投影误差的阈值；
7. `[P]` 先用近期高重叠关键帧得到初始位姿，几何检测后再跟踪局部地图。

因此 Ji 2021 应保留为：

```text
[P] 独立、可追溯的文献 baseline
```

不能把“深度 warp + signed residual + flood fill”称为 Ji 2021 的轻量实现，因为其核心测量不同：

```text
Ji: cluster-wise sparse feature reprojection error
G0: dense temporal depth consistency residual
```

另一个必须更正的实时性口径：

- Ji 论文脚注明确把 `100 ms/frame` 称为 real-time，即约 10 Hz；
- 表 IV 报告 geometry `30.14 ms`、tracking `75.82 ms`；
- 这不能证明其方法满足本项目的 TUM 30 FPS 目标。

### 5.11 Li and Lee 的 MAD / Student-t 能支持什么

Li and Lee 2017 的原方法：

1. `[P]` 只在 depth foreground edge points 上估计 static weight；

2. `[P]` 使用配准后源点与目标对应点的三维欧氏距离 `d_i`；

3. `[P]` 使用 Student-t 形式：
   
   ```text
   w_i = (ν+1) / (ν + ((d_i-μ_D)/σ_D)^2)
   ```

4. `[P]` 使用：
   
   ```text
   σ_D = 1.4826 · Median(|d_i - μ_D|)
   ```

5. `[P]` 其静态权重结合前一关键帧和当前帧的比较；

6. `[P]` 权重直接进入 IAICP 注册，不是简单生成二值动态 mask。

因此，未来把 MAD/Student-t 用到当前 `r_z` 上属于：

```text
[A] 将 Li and Lee 的鲁棒尺度/权重思想迁移到另一种残差
```

它不是 Li and Lee 方法复现，也不能保证解决当前区域边界失败。只有在残差分布、有效样本选择和动态占比经实验验证后才可进入 G2。

## 6. 当前 DT-SLAM 实现核对

### 6.1 G0-1 数据与位姿方向

当前 `ComputeWarp()` 使用：

```text
T_current_reference = Tcw_current · inverse(Tcw_reference)
```

再将参考相机坐标点变换到当前相机坐标系。

在 ORB-SLAM2 的 `Tcw` 定义下，该相对变换方向是正确的。合成测试通过说明当前代码符合人工构造的变换预期，但真实序列仍会同时受到位姿精度、时间间隔和深度噪声影响。

### 6.2 参考帧语义清理

`Tracking::GrabImageRGBD()` 在一次完整跟踪成功后：

1. clone 当前米制深度；
2. 将 semantic mask 内深度设为 0；
3. 保存深度、当前 `Tcw` 和 frame id 为下一帧几何参考。

这是合理的 `[E/A]` 防污染措施：已知语义动态区域不会成为下一次几何比较的“静态历史表面”。

但当前只清理语义 mask，因为几何尚未产生可信 final mask。这与 shadow 阶段边界一致。

### 6.3 像素坐标域与畸变风险

本地源码核对得到：

1. `[O]` 当前 geometry 的输入是 RGB 注册深度的原始像素栅格；
2. `[O]` `ComputeWarp()` 直接使用 `K^{-1}` 反投影，并用纯针孔 `K` 投影，没有使用 `mDistCoef`；
3. `[O]` semantic mask 与 `Frame::mvKeys` 对齐，即原始 RGB 像素域；
4. `[O]` `Frame::UndistortKeyPoints()` 另行生成 `mvKeysUn`，匹配和优化主要使用 undistorted keypoint；
5. `[O]` RGB-D 深度在 `Frame::ComputeStereoFromRGBD()` 中从原始 `mvKeys` 位置采样，再与同索引的 `mvKeysUn` 共同用于三维反投影。

当前 TUM3 配置为：

```text
k1 = k2 = p1 = p2 = 0
```

所以在已经完成的 TUM3 G0-3/G0-3R 实验中，非零畸变域不一致不是解释其失败的候选原因。

Bonn 官方页面则同时说明：

- depth 已注册到对应 RGB；
- RGB 相机存在非零畸变：

  ```text
  d0 =  0.039903
  d1 = -0.099343
  d2 = -0.000730
  d3 = -0.000144
  d4 =  0
  ```

“depth 已注册到 RGB”只说明两个模态位于同一 RGB 像素对应关系，不等于图像已经 rectified。因而在 Bonn 上直接沿用当前 raw-pixel pinhole warp 会把畸变误差混入 signed residual。

Bonn 运行前必须冻结共同坐标域，二选一：

1. 将 RGB、depth、semantic mask 和 geometry 输出统一 remap 到 rectified pinhole domain；
2. 保留原始域，但反投影时先恢复无畸变射线，投影时显式应用相机畸变模型。

深度 remap 还必须规定无效值、插值方式和前表面可见性；不能把普通双线性图像 remap 默认视为深度几何上无损。

若 geometry 输出保持原始域，feature shadow 标签应与 `mvKeys` 对齐；若 geometry 改到 rectified 域，则应与 `mvKeysUn` 对齐，或在模块边界做一次明确坐标转换。不能混用：

```text
raw depth / raw semantic mask
+ undistorted ORB coordinates
+ 未声明坐标域的 geometry mask
```

### 6.4 当前调用位置

当前 `Tracking::Track()` 的正常路径是：

```text
TrackWithMotionModel()
或 TrackReferenceKeyFrame()
    └─ PoseOptimization()

RunGeometryShadow()

TrackLocalMap()
    ├─ UpdateLocalMap()
    ├─ SearchLocalPoints()
    └─ PoseOptimization()
```

当前 geometry 位于“初始位姿已经产生、局部地图搜索之前”，符合两阶段前端结构。

这与 Ji 2021 的“初始关键帧跟踪 → 几何检测 → 局部地图跟踪”在系统结构上相容，但当前几何测量不是 Ji 方法。

### 6.5 为什么不能无条件增加第三次 PoseOptimization

当前已有：

1. `TrackWithMotionModel()` 或 `TrackReferenceKeyFrame()` 内的初始 `PoseOptimization()`；
2. `TrackLocalMap()` 的 `SearchLocalPoints()` 之后再次 `PoseOptimization()`。

因此 G1 合理的最小集成应是：

```text
初始 PoseOptimization
→ 几何检测
→ 清除几何动态 mvpMapPoints
→ 进入 TrackLocalMap
→ SearchLocalPoints 可能重新匹配
→ 再过滤一次
→ 使用 TrackLocalMap 已有 PoseOptimization
```

这利用已有第二次优化，不新增第三次。

只有未来实验证明：

- 几何过滤后不能安全进入现有 `TrackLocalMap()`；
- 或需要在 local map 更新前先获得一个单独 refined pose；

才有理由增加额外优化。该决定必须有调用图、耗时和 ATE/RPE 证据，不能因为某论文写了“second registration”就机械移植。ReFusion 的第二次 registration 是 TSDF 直接配准上下文，不等同于 ORB-SLAM2 必须新增一次 `PoseOptimization()`。

### 6.6 `SearchLocalPoints()` 后必须再次过滤

即使几何 mask 生成后清除了当前 `mvpMapPoints`，`TrackLocalMap()` 的 `SearchLocalPoints()` 仍可能将投影落在动态区域的 local MapPoint 重新关联到当前特征。

因此 G1 若启动，必须在：

```text
SearchLocalPoints()
之后、TrackLocalMap() 内 PoseOptimization() 之前
```

再次执行动态关联过滤。

这是由当前 ORB-SLAM2 调用关系直接推出的 `[E]` 集成要求，不是新的动态检测算法。

### 6.7 地图写入必须独立门控

位姿优化中剔除动态关联不能自动保证 MapPoint 不写入。

G1 还必须分别在：

- RGB-D 初始化创建 MapPoint；
- `CreateNewKeyFrame()` 对有效深度特征创建 MapPoint；
- 若存在稠密点云写入路径，则在深度像素写入前；

检查统一动态状态。

该门控属于系统完整性要求，和 DynaSLAM/Ji 均强调“不让动态观测参与 tracking 和 mapping”的目标一致，但具体代码位置由当前 DT-SLAM 架构决定。

## 7. 对已有表述的逐条更改

| 旧表述或建议                                  | 核对结果      | 更改后表述                                                                      |
| --------------------------------------- | --------- | -------------------------------------------------------------------------- |
| G0 是简化 DynaSLAM 几何模块                    | 过强        | G0 是受 DynaSLAM 多视图深度一致性启发的单参考帧稠密改造                                         |
| DynaSLAM 几何主要基于稀疏特征关联深度                 | 基本正确      | 论文明确投影 ORB-SLAM2 keypoint；官方代码在 keypoint 位置读取参考深度，并用当前深度做邻域检查和区域扩张         |
| z-buffer 来自 DynaSLAM                    | 不准确       | z-buffer 是标准可见性工程处理，当前稠密 forward warp 必需                                   |
| 正残差只是一项自定义假设                            | 过弱        | DynaSLAM 明确采用 `z_proj-z_current > τz`；当前“只把正残差作为 seed”有直接文献依据，但阈值和稠密实现仍是改造 |
| `0.10 m` 有 DynaSLAM 参数依据                | 错误        | DynaSLAM 论文为 `0.4 m`，官方代码为 `0.6 m`；`0.10 m` 仅是当前 shadow 实验参数               |
| `                                       | D(p)-D(n) | <θD(p)` 不应归于 ReFusion                                                      |
| 当前 G0-3 是 ReFusion/DynaSLAM region grow | 错误        | 当前是固定米制、局部邻接、all-seed union 改造；不等于两者任一精确实现                                 |
| unknown/static/dynamic 三状态来自某篇论文        | 无直接归属     | 是保守的软件接口与有效性建模；consistent 也不等于长期 static                                    |
| Ji 2021 是当前 G0 的依据                      | 错误        | Ji 是 K-means 深度区域 + cluster reprojection error，应作为独立 baseline              |
| Ji 2021 证明 30 FPS                       | 错误        | 其 real-time 定义为 100 ms/frame；不能支持本项目 30 FPS 声称                             |
| 几何后必须再加一次 PoseOptimization              | 过强        | 先复用 `TrackLocalMap()` 已有第二次优化；不无条件新增第三次                                    |
| G0-3 调一下面积或 seed ratio 阈值即可修好           | 无证据       | 当前两个序列分布重叠，禁止用任意阈值进入过滤                                                     |

## 8. 更改后的研究路线

### 8.1 保留的总目标

```text
ORB-SLAM2 RGB-D
+ YOLOv8-seg 语义分支
+ 类别无关几何证据分支
→ 统一动态观测状态
→ 前端特征过滤和地图写入否决
```

非目标继续保持：

- 不做对象轨迹；
- 不做对象级 BA；
- 不修改 g2o 代价函数；
- 不重构 ORB-SLAM2 后端；
- 不做完整 TSDF/3DGS；
- 不把“几何异常”无条件解释为运动对象。

### 8.2 阶段状态

| 阶段    | 内容                                      | 当前状态      | 是否影响 SLAM |
| ----- | --------------------------------------- | --------- | --------- |
| G0-1  | 单参考帧 dense warp、z-buffer、valid residual | 保留        | 否         |
| G0-2  | 正/负/一致/无证据分类                            | 保留        | 否         |
| G0-2V | 可视化与统计                                  | 保留为诊断     | 否         |
| G0-3  | 固定米制局部深度连通扩张                            | 判定失败，默认关闭 | 否         |
| G0-3R | 区域证据统计                                  | 未发现稳定可分性  | 否         |
| G0-2C | 坐标域、畸变、RGB-depth-GT 时间同步审计               | 已完成首轮门控   | 否         |
| G0-2P | GT pose / SLAM pose 敏感性诊断                 | 基础设施完成；待静态数据 | 否         |
| G0-2A | 人工标注下的 direct seed 审计                     | 待准备数据     | 否         |
| G0-3B | 相同 seed 上的文献传播规则局部消融                      | seed 通过后进行 | 否         |
| G0-4F | ORB feature-level shadow evidence            | seed 通过后可进行 | 否         |
| G1-F  | 高置信几何特征的稀疏 tracking/mapping 过滤             | 未开始       | 将影响       |
| G1-D  | 可信动态深度区域过滤                              | 未开始       | 将影响       |
| GJ    | Ji 2021 文献 baseline                     | 待独立实现     | 影子验证后决定   |
| G2    | 多参考、MAD、时序等增强                           | 未开始       | 待消融       |

### 8.3 下一阶段不应直接做什么

暂不执行：

```text
regionCandidateMask → mvbDynamic
regionCandidateMask OR semanticMask
清除 mvpMapPoints
新增 PoseOptimization
禁止 MapPoint 写入
```

原因不是工程未完成，而是当前 geometry region 没有达到足够的判别性。

### 8.4 建议的下一张工单

原报告把 region propagation 对照放得过早。修订后的第一张工单应先完成 G0-2C、G0-2P 和 G0-2A，全程 shadow-only。

#### G0-2C：坐标域、畸变和时间同步审计

先记录每个输入和输出的明确坐标域：

```text
RGB
depth
semantic mask
mvKeys
mvKeysUn
predicted depth
valid mask
positive/negative evidence
future geometry mask
```

同时检查：

- RGB-depth association 的时间差；
- RGB-GT 的时间差；
- reference/current 是否使用对应时间戳；
- 深度尺度；
- GT pose 的坐标约定；
- 非零畸变时采用 raw-domain distortion model 还是统一 rectification。

TUM/Bonn `groundtruth.txt` 给出的是传感器在世界参考系中的位置和四元数。用于当前 detector 前，必须按数据集定义构造 `Twc`，再求逆得到 `Tcw`，不能把文本行直接当 `Tcw`。对图像时间戳应使用有记录的最近邻门限或平移插值加四元数 SLERP，并报告时间误差。

#### G0-2P：GT pose / SLAM pose 双路径诊断

在完全相同的 reference depth、current depth、相机模型和 rasterization 下比较：

```text
T_t<-r^SLAM = Tcw_t^(initial) · inverse(Tcw_r^(final))
T_t<-r^GT   = Tcw_t^(GT)      · inverse(Tcw_r^(GT))
```

该对照可以诊断 seed 对初始位姿的敏感性，但不是完美单因素实验。GT 路径仍可能受：

- RGB-depth-GT 时间不同步；
- 相机/动捕外参；
- 深度噪声和空洞；
- forward-warp 离散化；
- 非零畸变；
- reference 中真实动态表面；

影响。因此结论应写成“pose sensitivity diagnostic”，不能把 GT residual 当无误差真值。

没有逐像素动态标注时，可以先比较 residual 分布、positive ratio、图像半径分层统计和静态序列 FPR；`seed precision/recall` 必须等 G0-2A 的标注 mask 存在后才能计算。

#### G0-2A：direct seed 人工标注审计

先做小规模 pilot，而不是立即标完整数据集。每种场景选取约 `20–50` 张代表帧是合理起点，但不是最终统计充分性的保证。

标注至少区分：

```text
dynamic
static
ignore / ambiguous boundary
invalid depth
```

建议场景：

1. 真正静态；
2. 静止箱子；
3. 横向运动箱子；
4. 沿视线运动箱子；
5. TUM walking。

必须报告：

```text
dynamic valid coverage
    |V ∩ GT_dynamic| / |GT_dynamic|

conditional seed recall
    |S ∩ GT_dynamic| / |V ∩ GT_dynamic|

seed precision
    |S ∩ GT_dynamic| / |S|

static FPR within valid evidence
    |S ∩ GT_static| / |V ∩ GT_static|
```

边界 ignore band 不计入 precision/FPR，避免把人工边界不确定性当算法错误。

当前 `0.10 m` 不能作为唯一点。应在标注帧上扫描 residual threshold，输出 PR/ROC 或至少 precision-recall-threshold 曲线，再冻结候选工作点。

还应在不放入 seed 的情况下统计当前深度图阈值图的：

- 连通分量数量；
- 最大连通分量占 `V` 的比例；
- 每个分量是否被 seed 命中；
- 被命中分量的面积并集。

这些统计能直接验证 G0-3 的结构性失败是否由“大分量 + 分散 seed”造成。

#### G0-3B：seed 通过后再做 region propagation 对照

所有方法必须使用同一帧、同一 valid mask 和完全相同的 seed：

| 对照 | 内容 | 身份 |
|---|---|---|
| A | positive seed only | 当前最低假设证据 |
| B0 | 固定 `0.05 m` 四邻域 all-seed union | 已失败工程 baseline |
| B1 | DynaSLAM 官方代码式 running-mean growth | `[C/A]` 局部传播消融 |
| B2 | ReFusion 相对深度 flood fill | `[P/A]` 局部传播消融 |

B1 有能力限制“每一步变化小但总深度逐渐漂移”的链式传播，这是由 running mean 规则带来的合理预期；是否能改善当前 seed 必须实验验证，标为 `[H]`。

B2 使用：

```text
|D(p)-D(n)| < θ·D(p)
```

它仍是局部邻接规则，在远距离会允许更大的绝对深度差，不能预设会解决链式传播。

若为了公平只比较传播函数，可以暂不加入 morphology，但此时只能称为“传播规则局部消融”。ReFusion 的完整 pipeline 包含 TSDF registration residual、raw-mask 形态学处理、flood fill 和第二次 registration；缺少这些部分时不能评价为 ReFusion reproduction。

#### G0-4F：feature-level shadow 分支

像素级完整区域失败不必永久阻塞稀疏特征研究，但 feature 分支也不能绕过 seed 审计。

这一拆分具有文献上的任务依据，但它本身是本项目的系统分解 `[S]`，不是某篇论文的原样流程：DynaSLAM 先获得几何动态 keypoint，再用深度区域扩展补全 mask；Ji 2021 的直接目标是删除动态 cluster 内的 ORB 特征；ReFusion 则以像素级动态 mask 保护稠密静态模型。三者分别说明“稀疏特征证据”和“动态深度区域”可以服务于不同下游，但不能据此假定同一组 seed 同时满足两条路径的精度与覆盖率要求。

G0-2A 通过后，可以只在 shadow 模式统计：

- ORB keypoint 是否直接落入 positive seed；
- 动态/静态标注区域内 feature evidence 的 precision、recall；
- 过滤候选后剩余可用匹配数；
- 箱子不同纹理和运动方向下的 feature coverage。

“keypoint 周围小窗口的 positive residual 比例”不是 DynaSLAM 或 Ji 2021 的直接复现，只能标为 `[A/H]` 的局部聚合候选，需与 exact-pixel seed 分开消融。

若 direct feature evidence 具有高 precision、可接受 recall，且保留足够静态匹配，可以先考虑 G1-F；这仍需 tracking/map admission 的独立门控与 ATE/RPE 验证。低纹理箱子和只在进入边缘产生 seed 的横向运动仍是明确局限。

G1-D 只有在区域 mask 通过标注审计后才能启动，用于动态深度、稠密点云或完整表面去残影。

#### 数据门控

本地目前只有 TUM dynamic sequences；`fr3/sitting_static` 不能作为纯静态负样本。

可补充的标准非动态候选包括 TUM 官方 testing/debugging 分类中的 `fr1/xyz`、`fr1/rpy`，以及 handheld SLAM 分类中的 `fr2/desk`。它们适合静态场景诊断，但仍应抽帧人工检查，而不是仅凭序列名假定每个像素绝对静态。

Bonn 官方提供 24 个动态序列和 2 个静态序列：

```text
rgbd_bonn_static
rgbd_bonn_static_close_far
```

当前本地只有若干动态 zip，尚未下载这两个静态序列。Bonn box 数据可用于运动箱子，但运行前必须先完成非零畸变域处理。

G0-2C 实际审计还发现，原 `fr3_walking_xyz/associations.txt` 的 859 行中只有 827 个唯一 depth 路径，有 32 行复用 depth，最大 RGB-depth 差约 38.10 ms。现已另行生成 827 对、无 depth 复用的一对一诊断 association，最大 RGB-depth 差约 6.17 ms；原文件没有覆盖。该问题是 residual 的候选混杂因素，但尚无证据证明它造成了 G0-3 的区域泛滥。

固定相机、完全静止场景的自采 `10–20 s` 序列也有价值：它主要测深度噪声和传感器稳定性；由于相机不运动，它不能替代对移动相机位姿敏感性的测试。

#### 修订后的决策表

| 结果 | 后续动作 |
|---|---|
| GT 与 SLAM 路径在真正静态序列都产生高 FPR | 优先检查坐标域、同步、尺度、rasterization 和阈值 |
| GT seed 明显好于 SLAM seed | 初始位姿敏感性成为主要研究问题之一 |
| seed 精确但 propagation flood | 比较 B1/B2，或转向预分割后区域评分 |
| region 差但 feature evidence 精确且覆盖足够 | 可继续 G0-4F，并在通过门控后单独考虑 G1-F |
| 横向箱子 direct seed recall 系统性偏低 | 记录退化模式，之后再评估自运动补偿光流 |
| B1/B2 在相同高质量 seed 下仍失败 | 停止调 flood fill，转向 Ji/StaticFusion 类区域预分割与区域评分 |

所有数值阈值最终应由标注数据上的曲线和跨序列验证决定，不能在没有标注时先写死。

## 9. 30 FPS 与计时结论

本项目目标约为：

```text
33.3 ms / frame 端到端
```

文献只能提供参考，不能替代本机测量：

- DynaSLAM 论文明确声明未针对实时优化，多视图几何约 `236–334 ms`；
- Ji 2021 将 `100 ms/frame` 定义为 real-time，其 geometry 为 `30.14 ms`；
- 当前 G0-3R shadow geometry 约 `8.3–8.4 ms`，不含语义、ORB 跟踪、Pangolin、调试 PNG 写盘；
- 当前同步 semantic baseline 已接近 30 FPS，故几何是否还能满足 30 FPS 必须以端到端分段统计验证。

至少应分开计时：

```text
dataset I/O
semantic enqueue/wait/inference/postprocess
Frame/ORB extraction
initial tracking
geometry warp
geometry evidence
region generation
TrackLocalMap
viewer/render
debug image write
per-frame wall-clock
sequence wall-clock
```

实时性判定应报告：

- mean；
- median；
- P90/P95/P99；
- max；
- warm-up 与稳定阶段分开；
- 是否保留可视化；
- 是否启用 debug write；
- GPU provider 和模型输入尺寸。

## 10. 风险列表

| 风险                         | 当前证据                             | 处理原则                      |
| -------------------------- | -------------------------------- | ------------------------- |
| 把 G0 称为 DynaSLAM 复现        | 方法组成明显不同                         | 统一改称 inspired/adapted     |
| 把源码参数写成论文参数                | DynaSLAM `0.4 m` vs code `0.6 m` | 论文与代码分栏报告                 |
| 固定 `0.10 m` 过拟合            | 无标注阈值选择实验                        | 仅保留 shadow 参数身份           |
| all-seed region union 吞噬背景 | 两序列覆盖约 96%                       | G0-3 默认关闭                 |
| 用 `sitting_static` 当无动态负样本 | 原文明确存在轻微人体运动                     | 增加真正静态序列                  |
| 用 region ratio 补经验阈值       | walking/sitting 分布重叠             | 在标注数据上做可分性分析              |
| 将无效/未覆盖像素当静态               | forward warp 有空洞                 | 保留 validity mask          |
| 把单帧 consistent 当长期静态       | 只是一项当前比较                         | 使用“consistent evidence”术语 |
| 初始位姿受未知动态污染                | 当前 geometry 依赖初始 pose            | shadow 先测；后续两阶段过滤         |
| GT/SLAM pose 对照被误当成完美隔离实验    | GT 仍受同步、外参和 rasterization 影响       | 只称 pose sensitivity diagnostic |
| Bonn raw depth 直接使用纯针孔 warp     | 官方标定非零畸变；当前 geometry 不读畸变参数       | Bonn 前先冻结共同坐标域             |
| RGB/depth/GT 时间错配污染 residual     | 数据流来自不同时间戳                       | 记录匹配误差并定义插值/门限            |
| 没有标注却报告 seed precision/recall   | TUM/Bonn 不提供现成逐像素动态 mask          | 小规模人工标注并设置 ignore boundary |
| feature 分支用新窗口规则替代验证          | 窗口 seed ratio 不是现有方法直接复现           | 标 `[A/H]` 并与 exact seed 消融    |
| SearchLocalPoints 重新引入动态关联 | 当前调用关系确定                         | G1 在搜索后再次过滤               |
| 无条件第三次优化增加开销               | 已有两次优化路径                         | 优先复用 TrackLocalMap 优化     |
| 过滤跟踪但仍污染地图                 | MapPoint 创建独立发生                  | G1 分别门控 tracking/mapping  |
| 文献“实时”口径误导 30 FPS          | Ji 定义为 100 ms/frame              | 只认本机端到端统计                 |
| DynaSLAM 官方代码被当作无误规范       | 论文/代码参数与参考选择存在差异                 | 复现前先定义目标版本并测试             |

## 11. 建议审批结论

建议交叉审批者逐项确认以下结论：

### 可批准

- [ ] G0-1/G0-2 继续作为 shadow evidence extraction；
- [ ] 单参考帧稠密 warp 的文献归属改为 DynaSLAM-inspired adaptation；
- [ ] z-buffer 标为标准工程处理；
- [ ] 正残差 seed 有 DynaSLAM 原文依据；
- [ ] negative residual 只作诊断；
- [ ] validity 和 consistency 分离；
- [ ] 坐标域/畸变与 direct seed 审计先于 region 对照；
- [ ] GT pose 只用于 shadow 敏感性诊断，不进入最终在线方法；
- [ ] feature-level 与 depth-region 分开设验收门控；
- [ ] Ji 2021 保持独立 baseline；
- [ ] G1 优先复用 `TrackLocalMap()` 已有 PoseOptimization；
- [ ] 地图写入过滤与 tracking 过滤分开。

### 应驳回

- [ ] 将当前 G0-3 mask 用于动态特征过滤；
- [ ] 根据现有 50 帧人为选择 region area/seed-ratio 阈值；
- [ ] 声称当前 G0 是 DynaSLAM/ReFusion/Ji 的复现；
- [ ] 声称文献已经证明本项目可以达到 TUM 30 FPS；
- [ ] 在 Bonn 非零畸变未处理时用 residual 评价几何方法；
- [ ] 没有逐像素标注时声称获得 seed precision/recall；
- [ ] 把 DynaSLAM running-mean 必然改善写成事实；
- [ ] 把 keypoint 小窗口 seed ratio 写成文献复现；
- [ ] 在当前证据不足时修改 `Optimizer.cc`、g2o 或后端。

### 需补充实验后再批准

- [ ] DynaSLAM code-style mean-constrained region grow；
- [ ] ReFusion relative-depth flood fill 的 temporal-warp adaptation；
- [ ] GT pose / SLAM pose 双路径诊断；
- [ ] direct seed 人工标注与阈值曲线；
- [ ] Bonn 的统一畸变坐标域；
- [ ] feature-level shadow evidence；
- [ ] 多参考帧；
- [ ] MAD/Student-t residual weighting；
- [ ] ORB feature dynamic projection；
- [ ] geometry mask 参与 tracking/mapping；
- [ ] 额外 PoseOptimization；
- [ ] 时序投票、光流、刚性图或自由空间。

## 12. 最终冻结表述

建议后续文档统一使用：

> G0 is a shadow-mode geometric inconsistency measurement inspired by DynaSLAM's multi-view depth-consistency principle. It uses a single previous successfully tracked RGB-D frame, dense depth forward warping with standard z-buffer visibility handling, an explicit validity mask, and signed depth residuals. Positive residuals provide conservative foreground-entry evidence. G0 is not a reproduction of DynaSLAM, ReFusion, or Ji et al. 2021.

中文：

> G0 是一个受 DynaSLAM 多视图深度一致性思想启发的影子模式几何不一致证据测量。它使用上一成功跟踪 RGB-D 帧、带标准 z-buffer 可见性处理的稠密深度前向投影、显式有效性 mask 和有符号深度残差。正残差只提供保守的近表面进入证据。G0 不是 DynaSLAM、ReFusion 或 Ji et al. 2021 的复现。

当前下一步应是：

```text
坐标域、畸变与时间同步审计
→ 准备真正静态/静止箱子/运动箱子及少量人工标注
→ GT pose / SLAM pose 下分别审计 direct seed
→ 相同 seed 上做文献传播规则局部消融
→ 分别决定是否进入 G0-4F、G1-F 或 G1-D
```

而不是：

```text
继续增加无来源阈值
→ 直接过滤 ORB 特征
→ 再看 ATE 是否偶然改善
```

## 13. 第二轮交叉审阅的选择性吸收结果

### 完全采纳

1. 当前失败的是 G0-3 区域恢复组合，不是 G0-1/G0-2 测量链或类别无关几何方向；
2. 坐标域/畸变检查和 direct seed 审计必须早于 region-growing 对照；
3. `sitting_static` 不是纯静态负样本；
4. 当前 G0-3 可精确解释为被 seed 命中的深度连通分量并集；
5. Bonn depth 注册到 RGB 不等于已去畸变，非零畸变必须处理；
6. GT pose / SLAM pose 双路径具有高诊断价值；
7. seed 与 propagated mask 必须分别评价；
8. 像素级区域失败不应自动否定 feature-level shadow 研究；
9. G1 应拆为稀疏 feature filtering 和 depth-region filtering 两条门控。

### 部分采纳并降级表述

1. `20–50` 张/场景仅作为标注 pilot，不作为最终统计充分性的保证；
2. GT/SLAM 对照能揭示位姿敏感性，但不能排除同步、外参、畸变、深度噪声和 rasterization；
3. DynaSLAM running-mean 规则在结构上可能抑制链式漂移，仍必须标为待验证假设；
4. ReFusion 无 morphology 的比较只适合传播函数局部消融，不能代表其完整 pipeline；
5. feature evidence 精确时可以继续 G0-4F，但进入 G1-F 还必须检查 recall、剩余匹配数、轨迹和 MapPoint admission；
6. Bonn 静态模型真值有利于最终地图评价，但它不是现成的逐帧动态像素 GT，不能替代人工 mask。

### 不采纳为当前事实

1. 不认定换成 DynaSLAM running-mean 就会修复 G0-3；
2. 不认定所有背景 positive seed 都来自位姿误差；
3. 不认定 GT pose 下 residual 必然接近零；
4. 不把 keypoint 小窗口 positive ratio 当作文献已有方法；
5. 不因标准序列名称就假定每一帧、每个像素绝对静态；
6. 不在 seed 审计前继续实现 region threshold、feature filtering 或地图过滤。
