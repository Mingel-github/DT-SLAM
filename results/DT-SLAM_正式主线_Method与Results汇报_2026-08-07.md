# DT-SLAM 正式主线 Method 与 Results 汇报

日期：2026-08-07

用途：快速理解和核实当前真正并入 DT-SLAM 主线的方法、实验结果与能力边界。

## 1. 范围

本文只介绍已进入正式代码路径的内容：

- YOLOv8-seg 人物语义分支；
- SInDSLAM 风格的类别无关区域几何分支；
- S2 动态 ORB Tracking 过滤和 MapPoint 写入控制；
- Tracking fail-open 安全恢复；
- S3 动态深度输出。

早期 depth warp、flood fill、Ji 2021 聚类、稀疏 LK 单点判决、两帧刚性和其他 shadow 试验不属于当前最终方法。“至少8帧支持”和 OctoMap 尚是固定轨迹的离线 Mapping 实验，未进入在线主链。

# Method

## 2. 系统总体结构

当前系统数据流为：

```text
RGB-D输入
├── YOLOv8-seg
│   └── person像素mask
├── SInDSLAM风格区域几何
│   └── 类别无关geometry mask
└── ORB-SLAM2 RGB-D
    ├── 过滤动态ORB匹配和关联
    ├── 阻止动态观测创建MapPoint
    └── 输出建图用过滤深度
```

系统保留四种正式实验模式：

1. 纯 ORB-SLAM2；
2. 仅语义；
3. 仅几何；
4. 语义＋几何。

当前最初组合基线使用：

\[
M_t^{track}=M_t^{semantic}\lor M_t^{geometry}.
\]

这个并集规则只是首个系统基线，实验没有证明它在所有序列上最优。

## 3. 统一坐标域

语义 mask、注册深度、ORB特征和几何 mask 必须位于同一像素域。

- TUM3 和 Gazebo 使用零畸变/针孔域；
- Bonn RGB 和注册深度先统一重映射到 `P=K` 无畸变针孔域；
- YOLO、ORB、深度聚类、稠密光流和最终 mask 全部使用统一后的图像。

像素 \(\mathbf u=(u,v)\) 和米制深度 \(z\) 的三维反投影为：

\[
\mathbf X=
\begin{bmatrix}
(u-c_x)z/f_x\\
(v-c_y)z/f_y\\
z
\end{bmatrix}.
\]

## 4. YOLOv8-seg 语义分支

当前 ONNX 模型为 YOLOv8n-seg，正式动态类别仅使用 COCO `person`。置信度门槛为0.5，NMS门槛为0.45。

```text
RGB
→ letterbox和张量转换
→ ONNX Runtime CUDA推理
→ person候选和NMS
→ prototype mask解码
→ 恢复到统一像素域
→ 人物二值mask
```

推理虽在工作线程中执行，但正式基线会等待同一帧的 mask，因此 mask age 为0。

若第 \(i\) 个 ORB 特征落在人物 mask 中：

\[
d_{i,t}^{semantic}=1.
\]

语义分支负责已知人物类别的高召回处理，不能发现未配置类别的箱子、气球或推车。

## 5. SInDSLAM 风格区域几何

当前几何的准确名称为：

> SInDSLAM 风格、基于三维区域重组织和稠密光流残差的类别无关动态区域检测。

它是 clean-room 改造，不是作者 SInDSLAM 的等价复现。

### 5.1 三维K-means初始分区

对有效深度像素进行三维反投影，再求解：

\[
\min_{\{C_k,\boldsymbol\mu_k\}}
\sum_k\sum_{\mathbf X_i\in C_k}
\lVert\mathbf X_i-\boldsymbol\mu_k\rVert_2^2.
\]

正式配置只处理 \(0<z<6\text{ m}\) 的有效深度，区域数为：

\[
K=\operatorname{round}\left(\frac{wh}{25600}\right),
\]

640×480时为12个初始簇。系统使用四层深度金字塔，并可用上一帧标签初始化当前帧。

K-means只提供初始空间分区，不直接代表物体或动态。

### 5.2 深度边界切分

系统先做局部有掩码中值深度，然后使用：

\[
\tau_D(u)=\max(0.025D(u),0.08\text{ m})
\]

检测明显深度边界，把箱子前表面与后方墙面等混合簇切开。

### 5.3 RAG区域合并

切分后的区域可能过碎，因此通过区域邻接图 RAG（Region Adjacency Graph）重新合并。主要证据包括：

- 空间邻接；
- 初始K-means人工边界；
- 区域深度直方图；
- 中心深度差；
- 区域面积。

当前正式路线是 gradient-only RAG，没有完整保留 SInDSLAM 的 PEAC/AHC 平面边缘，因此属于 SIn 风格区域近似。

### 5.4 稠密观测光流

正式后端在0.6倍图像尺度运行 CPU DeepFlow，再进行 Variational Refinement 并恢复到原始分辨率。

参考通常取 \(t-2\)，以增加小运动的可观测性；若帧间运动过大，退回 \(t-1\)。

### 5.5 相机自运动补偿与残差

系统从稠密光流中按10像素网格采样，使用 `cv::RHO` 渐进一致性采样估计当前帧到参考帧的 homography：

\[
\widehat{\mathbf u}_r^H=\pi_H(\mathbf u_t).
\]

实际观测光流与相机运动预测之差为：

\[
\delta(\mathbf u_t)=
\left\lVert
\mathbf u_r^{flow}-\widehat{\mathbf u}_r^H
\right\rVert_2.
\]

高残差表示该像素无法被当前全局相机运动解释，但仍可能来自深度视差、光流失败、遮挡或边界，因此不直接删除单个高残差点。

### 5.6 双阈值和区域内动态判决

残差幅值归一化到0–255，通过 Otsu 和 Triangle 阈值得到低/高残差支持。对每个 RAG 区域：

1. 高残差像素至少100个；
2. 高残差轮廓需通过面积和圆度检查；
3. 只在同一几何区域的中高残差支持中填充；
4. 填充超过该区域50%时，整区域判为动态；
5. 否则仅保留局部动态部分；
6. 输出前做9×9膨胀，并与有效区域相交。

这与“K-means簇平均误差超阈值就整簇删除”不同。

几何状态区分为 `dynamic`、`static` 和 `unknown`；深度无效、光流或区域失败时记为 `unknown`，不自动解释为静态。

## 6. S2：动态特征进入ORB-SLAM2

若第 \(i\) 个 ORB 落在几何动态 mask 中：

\[
d_{i,t}^{geometry}=1.
\]

组合模式下：

\[
\texttt{mvbDynamic}[i]
=d_{i,t}^{semantic}\lor d_{i,t}^{geometry}.
\]

`mvbDynamic` 不是新的 g2o 权重，而是在原生优化前阻止动态观测进入匹配或清除其关联：

- ORBmatcher 在 BoW、投影和关键帧匹配路径跳过动态索引；
- `TrackReferenceKeyFrame()` 和 `TrackWithMotionModel()` 在原有 `PoseOptimization()` 前清除动态 MapPoint 关联；
- `TrackLocalMap()` 在 `SearchLocalPoints()` 后、原生 `PoseOptimization()` 前再次清理残留关联；
- RGB-D初始化、新关键帧和 LocalMapping 新建 MapPoint 时拒绝动态特征。

系统没有修改 `Optimizer.cc`，没有增加第三次位姿优化，也没有对象级 BA。

## 7. Tracking fail-open

若几何过滤导致 ORB-SLAM2 原生最低匹配条件无法满足，当前帧暂时撤销新增几何 Tracking 标志并重新匹配。

- 语义人物标志不撤销；
- 不增加额外 PoseOptimization；
- Tracking后恢复几何标志，继续阻止 Mapping 写入。

Fail-open 是安全机制，不表示被恢复的特征已被证明为静态。

## 8. S3：建图用动态深度

S3 支持 `semantic_only`、`geometry_only` 和 `semantic_or_geometry` 三种深度 mask。

\[
D_t^{static}(u)=
\begin{cases}
0,&M_t^{depth}(u)=1,\\
D_t(u),&\text{其他}.
\end{cases}
\]

Tracking 始终使用原始深度；`staticDepthMeters` 只供 Mapping 输出。因此可在同一条位姿轨迹上比较过滤前后地图，不把位姿变化混入 Mapping 评价。

# Results

## 9. 评价指标

- ATE RMSE（Absolute Trajectory Error）：全局绝对轨迹误差；
- RPE RMSE（Relative Pose Error）：相邻帧相对位姿误差；
- 轨迹完整性：成功输出位姿的帧数以及 LOST 帧数；
- 箱子残影保留率：Gazebo箱体投影/体素代理下的 Mapping 指标，不是现实逐像素真值。

ATE和RPE单位为米，越低越好。

## 10. TUM已知人物动态

TUM `fr3/walking_xyz` 三轮中位数：

| 模式 | ATE RMSE | RPE RMSE |
| --- | ---: | ---: |
| 纯 ORB-SLAM2 | 0.730574 m | 0.025407 m |
| 仅 YOLO语义 | **0.016477 m** | **0.011876 m** |

语义分支将 ATE 从约73 cm降到约1.65 cm。该结果跨三轮重复，是当前最稳定的已知类别正证据。

## 11. Bonn未知非遮挡箱子

Bonn `moving_nonobstructing_box` 中，detector在 ON/OFF 两组都完整运行，只控制几何 mask 是否真正影响 ORB-SLAM2。ON/OFF各运行三轮。

| 模式 | ATE RMSE | RPE RMSE | 轨迹完整性 |
| --- | ---: | ---: | ---: |
| 几何检测运行，但不进入SLAM | 0.514344 m | 0.022127 m | 778/778 |
| S2区域几何参与过滤 | **0.022526 m** | **0.014381 m** | 778/778 |

开启S2后的三轮 ATE 均在2.2–2.3 cm，关闭时为45.2–54.8 cm。这证明当前区域几何在该未知箱子序列中具有可重复的真实定位收益。

该结论不能外推为所有未知物体已解决，也不能由 ATE 直接推导出 mask 逐像素准确。

## 12. Bonn强遮挡与fail-open

Bonn `moving_obstructing_box`：

| 版本 | ATE RMSE | RPE RMSE | Tracking OK / LOST |
| --- | ---: | ---: | ---: |
| 几何过滤关闭 | 0.546996 m | 0.019121 m | 589 / 0 |
| 旧S2，只按剩余特征总数保护 | 0.177218 m（仅截断轨迹） | 0.081901 m | 389 / 200 |
| S2＋匹配阶段fail-open | **0.245857 m** | **0.016536 m** | **589 / 0** |

旧S2因过度过滤丢失200帧；新版fail-open仅触发1次即恢复完整589帧轨迹。`0.177218 m` 只对截断轨迹评价，不能与新版完整轨迹直接比较。

## 13. 六序列第一轮范围检查

以下数值均为单轮，用于检查系统适用范围，不应包装成稳定改进：

| 序列 | 纯ORB ATE | 仅语义 | 仅几何 | 语义＋几何 | 几何FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| TUM walking_xyz | 0.870140 | 0.016267 | 0.014655 | 0.014508 | 3.98 |
| TUM sitting_static | 0.007749 | 0.006560 | 0.008079 | 0.006439 | 4.40 |
| TUM fr1_xyz | 0.009830 | 0.009675 | 0.009647 | 0.009552 | 3.24 |
| Bonn nonobstructing box | 0.607423 | 0.352860 | 0.020697 | 0.021611 | 3.84 |
| Bonn obstructing box | 0.345349 | 0.318416 | 0.290795 | 0.298631 | 3.35 |
| Bonn static_close_far | 0.084066 | 0.084186 | 0.082045 | 0.084647 | 3.75 |

该范围检查说明：

- 未知箱子场景中，几何可能远优于只识别人类的语义；
- 真静态和低动态场景未观察到灾难性定位退化；
- 语义与几何简单 OR 没有稳定优于单独分支；
- 几何收益高度依赖场景。

## 14. AWS Small House人物＋箱子仿真

新仿真包含人物和0.5 m/s匀速往返箱子，由用户人工驾驶TurtleBot3，共6328帧、约316.6秒。

| 模式 | ATE RMSE | RPE RMSE | 轨迹覆盖率 | 实际FPS |
| --- | ---: | ---: | ---: | ---: |
| 纯ORB-SLAM2 | 0.059334 m | 0.005624 m | 94.82% | 19.94 |
| 仅语义 | **0.055089 m** | **0.004836 m** | 96.19% | 19.94 |
| 仅几何 | 0.056881 m | 0.006505 m | 97.09% | 4.57 |
| 语义＋几何 | 0.057982 m | 0.005938 m | **97.42%** | 4.38 |

客观结论：

- 仅语义在本轮 ATE/RPE 最好；
- 几何提高轨迹覆盖率并略降 ATE，但 RPE 变差；
- 语义＋几何覆盖率最高，但没有最好定位精度；
- 本表是单轮完整序列，毫米级差异应考虑 ORB-SLAM2 运行波动；
- 不能宣称几何在该仿真中稳定改善定位。

## 15. 固定轨迹离线 Mapping

### 15.1 旧Gazebo 600帧

| 建图方法 | 箱子残影保留率 |
| --- | ---: |
| S3直接累计 | 75.17% |
| S3＋至少8帧支持 | **0%** |
| S3＋OctoMap | **0%** |

旧轨迹中92.17%的箱子候选体素在最后一次箱子命中后被非箱子射线重新穿过，因此 OctoMap 具有充足的 free-space 反证。

### 15.2 AWS Small House

| 建图方法 | 箱子残影保留率 | 结构保留代理 |
| --- | ---: | ---: |
| 普通累计 | 100% | 100% |
| S3直接累计 | 96.35% | 99.63% |
| 至少8帧，不用S3 | 11.60% | 100% |
| S3＋至少8帧 | **8.38%** | **98.17%** |
| OctoMap，不用S3 | 42.43% | 76.10% |
| S3＋OctoMap | 36.31% | 74.96% |

AWS中箱子在约2.01 m范围内重复往返，约78次单程经过。只有5.80%的旧箱子体素在最后命中后得到 free-space 重访，因此 OctoMap 不能复现旧轨迹中的0%。

AWS中最强的箱子清理主要来自至少8帧时间支持：不使用S3已降到11.60%，加入S3后为8.38%。S3直接累计仍保留96.35%，说明当前逐帧 detector 对AWS箱子的独立覆盖较弱。

时间支持和 OctoMap 是离线 Mapping 对照，不属于当前在线主链。

## 16. 运行性能

AWS代表序列的平均成本：

| 模块 | 平均耗时 |
| --- | ---: |
| CPU DeepFlow | 约156 ms/帧 |
| 深度梯度切分 | 约24 ms/帧 |
| RAG合并 | 约9.7 ms/帧 |
| 区域动态判决 | 约2.5 ms/帧 |
| GPU YOLO | 约9.6 ms/帧 |

完整区域几何约3.2–4.6 FPS；仅语义可跟随20 Hz仿真数据。当前性能瓶颈是稠密光流和区域链，不是YOLO、S2标志应用或S3深度置零。

# Conclusion

## 17. 当前可以成立的结论

1. DT-SLAM已实现同帧YOLOv8人物语义和SInDSLAM风格类别无关区域几何。
2. 动态状态已真正进入ORB匹配、原生位姿优化前关联清理、新MapPoint写入控制和动态深度输出。
3. 语义分支在TUM人物场景中具有稳定、显著的定位收益。
4. 区域几何在Bonn未知非遮挡箱子上具有可重复的显著定位收益。
5. fail-open解决了已观察到的强遮挡过度过滤和长段Tracking LOST。
6. Mapping端时间支持和概率占用可以显著减少历史动态残影，但效果受物体停留时间和free-space重访条件影响。

## 18. 当前不能宣称的内容

- 已可靠检测所有未知动态物体；
- 几何在所有数据集上稳定改善ATE；
- 语义和几何的简单并集是最优融合；
- S3 已构成完整在线长期静态地图；
- 当前区域几何已实时；
- 当前 detector 是SInDSLAM等价复现；
- 当前 detector 已形成明显区别于SInDSLAM的核心算法创新。

## 19. 最简摘要

> DT-SLAM已经建立完整的语义—区域几何—Tracking—Mapping实验链。YOLO人物语义在TUM动态人物中稳定有效；SIn风格区域几何在Bonn未知箱子中得到可重复强收益，但在AWS仿真中对箱子的独立覆盖较弱。Mapping端时间支持和OctoMap能减少残影，但仍是离线验证且依赖轨迹条件。剩余主要问题是几何速度、跨域对象特异性、在线长期地图和算法原创性。

## 20. 核对来源

- `results/DT-SLAM_当前语义与区域几何正式算法技术报告_2026-08-06.md`；
- `results/DT-SLAM_成果收束与复现索引_2026-08-07.md`；
- `results/sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md`；
- `results/sindslam_systematic_eval_2026-08-04/SYSTEMATIC_EVALUATION_FIRST_PASS_RESULT.md`；
- `results/aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_PERSON_BOX_FOUR_MODE_RESULT.md`；
- `results/aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_FIXED_TRAJECTORY_MAPPING_RESULT.md`；
- `results/offline_mapping_feasibility_2026-08-06/OFFLINE_FIXED_TRAJECTORY_MAPPING_FEASIBILITY_RESULT.md`。

