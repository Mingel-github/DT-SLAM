# G2-MH1 稀疏三维刚体运动假设 Shadow SPEC

日期：2026-08-01
状态：实现前冻结
总计划归属：G2 可靠未知动态判决；不是新增总阶段

## 1. 本轮只回答什么

当前 F1 已经能测量单个 ORB/LK 观测相对静态相机模型的二维残余运动，但
`q>=10` 单点硬判决没有通过严格留出。最新输入审计又表明：Bonn 非遮挡箱子的
5 个 moving proxy 帧都存在多点共同二维运动结构，但同一简单 coherence 条件
也在 10/19 个 stationary proxy 帧触发。

G2-MH1 只回答：

> 质量合格的相邻帧 RGB-D 对应，能否形成内部拟合良好、且比静态背景相机模型
> 更能解释观测的局部三维刚体运动假设？

本阶段不回答：

```text
这些假设应如何聚类成对象
对象应如何跨帧保持身份
哪一个假设一定是动态
是否应该删除 ORB 特征或深度像素
```

因此输出仍为 `dynamic_decision=none`，不修改任何 SLAM 状态。

## 2. 方法来源与身份

### 2.1 Lee et al. 2019 `[L]`

Lee 等人的原方法使用 grid RGB-D scene flow。其核心物理假设是：同一刚体上的
三维点在相邻时刻共享同一个刚体变换。原方法用邻近的 `m=7` 个点生成局部刚体
假设，再进行 hypothesis refinement、DBSCAN 聚类、segment matching 和
dual-mode temporal model。

原论文中的以下参数是作者在自采室内数据上调节的：

```text
grid size w_grid=16
maximum hypotheses h_max=20
rigid-error threshold t_inlier=3e-5
search radius r_search=2
DBSCAN epsilon=0.005
```

这些数值不得迁移为 DT-SLAM 的动态阈值。

### 2.2 当前适配 `[A]`

当前系统没有 grid dense scene flow，而是：

```text
当前 ORB 特征
→ LK 回溯到上一成功帧
→ forward-backward correspondence check
→ 两帧 metric depth 反投影
```

G2-MH1 将 Lee 的局部三维刚体假设原理适配到上述稀疏 transient
correspondence。它不是 Lee reproduction，也不是 DymSLAM/MVO 的多运动分割。

### 2.3 项目设计 `[S/H]`

- `[S]` 使用确定性的局部邻域和 SVD/Kabsch 最小二乘刚体配准，避免 shadow
  审计受到随机采样影响；
- `[S]` 同时计算同一组点在背景相机模型和局部刚体模型下的连续残差；
- `[H]` 真正属于同一运动箱子的局部点集，局部模型拟合误差应低于背景模型；
- `[H]` 静态背景的局部模型应接近背景相机模型，或不能稳定获得额外解释能力。

上面两项假设必须由实验验证，不能当作既成事实。

## 3. 当前代码已经具备的输入

不新增第二套 LK 或深度预处理。复用：

```text
GeometricDynamicDetector::ComputeSparseEgoFlow()
GeometricDynamicDetector::ComputeLocalRigidity()
Tracking::RunSparseEgoFlowShadow()
Tracking::SparseFlowReference
```

每个有效对应已经能够得到：

- 当前像素 `u_t`；
- 参考像素 `u_r`；
- forward-backward error；
- 参考帧 `CV_32F` 米制深度；
- 当前帧 `CV_32F` 米制深度；
- 参考点 `X_r` 和当前点 `X_t`；
- 语义动态标志；
- 当前初始位姿与上一成功帧最终位姿；
- Bonn 中经过联合 rectification 的共同 `P=K` 针孔域。

当前没有稳定的多帧 feature identity；因此 G2-MH1 严格限定为相邻两帧
hypothesis measurement。

## 4. 输入有效性与 unknown 状态

一个点只有同时满足以下已有条件才进入局部假设：

```text
F1 evidence_state == measured
forward-backward error <= 已冻结的 correspondence-quality 条件
reference/current depth 均有效
reference/current 3-D point 有限
semantic_dynamic == 0（组合模式中排除已知动态；geometry-only 时自然全零）
```

任何缺失都输出对应的 no-evidence 原因，不能解释成静态：

```text
sparse_flow_invalid
forward_backward_rejected
semantic_excluded
reference_depth_invalid
current_depth_invalid
outside_image
insufficient_local_support
degenerate_geometry
numeric_failure
```

## 5. 坐标与模型

参考帧和当前帧中的观测三维点分别为：

\[
X_{r,i}=\pi^{-1}(u_{r,i},D_r(u_{r,i})),
\qquad
X_{t,i}=\pi^{-1}(u_{t,i},D_t(u_{t,i})).
\]

静态背景应由已有相机相对位姿解释：

\[
H_{bg}=T_{cw,t}T_{cw,r}^{-1}.
\]

对局部点集 \(\mathcal N_j\)，用标准最小二乘三维刚体配准估计：

\[
H_j^*=\arg\min_{H\in SE(3)}
\sum_{i\in\mathcal N_j}
\|H X_{r,i}-X_{t,i}\|_2^2.
\]

第一版只使用闭式 SVD/Kabsch 解，不进行 ICP、g2o 或 BA。

对相同点集同时记录：

\[
e_{local,i}=\|H_j^*X_{r,i}-X_{t,i}\|_2,
\]

\[
e_{bg,i}=\|H_{bg}X_{r,i}-X_{t,i}\|_2.
\]

只输出连续描述量：

```text
local fit median / RMS / P90
background fit median / RMS / P90
background-to-local median improvement
background-to-local RMS ratio
```

为便于解释，再记录：

\[
\Delta H_j=H_j^*H_{bg}^{-1},
\]

对应的 rotation angle 和 translation norm。它们是诊断量，不是动态阈值。

## 6. 第一版局部假设生成

Lee 原文使用邻近 7 点生成 hypothesis。当前第一版冻结为：

1. 对每个 quality-eligible anchor，在当前图像坐标中查找最近的其他有效点；
2. 使用 anchor 加 6 个最近点形成 7 点局部集合；
3. 记录该集合的最大图像半径、参考/当前深度跨度；
4. 检查参考点中心化矩阵是否至少具有二维几何跨度；近共线或数值病态时输出
   `degenerate_geometry`；
5. 对非退化集合估计一个 \(H_j^*\)。

这里的 `7` 只用于第一版文献原型审计，不是检测阈值。当前 ORB 不是规则 grid，
所以不复制 Lee 的 `r_search=2`。第一版也不设置经验像素半径；局部半径作为输出
审计，以检查“最近 7 点”是否经常跨越过大区域。

为保持可重复性：

- 邻居按当前图像欧氏距离排序；
- 距离相同时按 feature index 排序；
- 不使用随机 RANSAC；
- 不使用 bbox、person proxy 或 motion proxy 参与 hypothesis 生成。

## 7. 第一版明确不实现的 Lee 组件

```text
entropy-weighted random search
hypothesis inlier threshold
hypothesis refinement
DBSCAN
segment/object label
segment matching
dual-mode temporal model
```

先判断局部三维 hypothesis 本身是否具有可分信息。若这一步失败，继续实现聚类
只会把无区分力的模型组织起来。

## 8. 输出接口草案

只新增纯计算数据结构，命名可在实现时按现有风格调整：

```cpp
enum class GeometricRigidHypothesisState;

struct GeometricRigidHypothesisSample
{
    std::size_t anchorFeatureIndex;
    std::vector<std::size_t> memberFeatureIndices;
    cv::Mat HReferenceToCurrent;
    float localFitMedianMeters;
    float localFitRmsMeters;
    float localFitP90Meters;
    float backgroundFitMedianMeters;
    float backgroundFitRmsMeters;
    float backgroundFitP90Meters;
    float medianImprovementMeters;
    float rmsRatio;
    float relativeTranslationMeters;
    float relativeRotationRadians;
    float maximumImageRadiusPixels;
    float referenceDepthSpanMeters;
    float currentDepthSpanMeters;
    GeometricRigidHypothesisState state;
};

struct GeometricRigidHypothesisResult
{
    std::vector<GeometricRigidHypothesisSample> hypotheses;
    /* validity counts and timing only */
};
```

建议纯函数：

```cpp
static GeometricRigidHypothesisResult ComputeLocalRigidHypotheses(
    const GeometricRigidityResult &rigidity,
    const cv::Mat &TcwReference,
    const cv::Mat &TcwCurrent,
    std::size_t localPointCount = 7);
```

实现时可直接复用 `GeometricRigidityResult::nodes` 中的两帧三维点，避免再次读取
深度或重复质量判断。

## 9. 测试要求

### 9.1 确定性合成测试

1. **纯静态刚体**：所有点只受 \(H_{bg}\) 作用，局部模型应恢复背景变换，
   local/background fit 都接近数值精度；
2. **两个刚体**：背景点受 \(H_{bg}\)，局部物体点受另一个已知 \(H_{obj}\)，
   对象内部 anchor 的局部模型应更接近 \(H_{obj}\)；
3. **混合边界集合**：同时包含两个模型的局部集合应表现出更高 local fit，不能
   静默成为高质量对象假设；
4. **共线点**：输出 `degenerate_geometry`；
5. **无效深度、语义排除、FB 拒绝**：不形成有效 hypothesis；
6. 同一输入重复运行，输出顺序和数值一致。

### 9.2 真实数据 shadow

开发输入：

```text
Bonn moving_nonobstructing_box：既有 5 moving / 19 stationary RGB proxy 帧
Bonn moving_obstructing_box：17 帧无标签描述
TUM fr1/xyz：真静态风险检查
Bonn static_close_far：真静态风险检查
```

coarse bbox 和 RGB motion proxy 只用于运行后的分层评价，不进入算法。

必须分别报告：

- 有效/退化/支持不足 hypothesis 数；
- 最近 7 点的图像半径和深度跨度；
- local fit 与 background fit 分布；
- moving、stationary、框外背景的连续差异；
- hypothesis 计算时间与完整 pipeline FPS；
- 结果对初始 SLAM pose 的依赖；条件允许时保留 GT-pose counterfactual。

任何 AUC 只能称为 development proxy ranking，不是检测精度。

## 10. 停止条件与后续分叉

### 支持继续到 hypothesis clustering

只有当多帧重复观察到：

```text
moving proxy 内存在非退化、低 local fit 的局部模型
且同一点集 background fit 明显更差
且 stationary / 真静态背景不会普遍出现同样结构
```

才另立下一份 Lee-style hypothesis-clustering shadow SPEC。届时仍需单独决定模型
距离、DBSCAN/其他聚类方式和时间确认，不能直接搬论文参数。

### 停止稀疏局部假设路线

若出现任一结构性失败：

- 目标内经常不足 7 个有效三维对应；
- 最近 7 点普遍跨越目标与背景，局部 fit 无意义；
- moving 与 stationary/背景的 hypothesis 分布不可分；
- 初始位姿误差使全图产生同类替代模型；
- 计算成本与获得的信息明显不相称；

则记录负结果，不通过改 `q`、邻域半径、面积或 bbox 规则继续修补。随后再决定
是否转向更完整的 SInDSLAM/FlowFusion 类稠密区域路线。

## 11. 与 G1 的隔离

G2-MH1 完成期间继续禁止：

```text
写 mvbDynamic
清除 mvpMapPoints
阻止 MapPoint 创建
生成或膨胀 M_depth
修改 Optimizer.cc / g2o / BA
新增 PoseOptimization
修改 YOLO
默认打开现有 q10 过滤
```

即使 G2-MH1 结果正面，也只允许先进入 hypothesis clustering 和短时序确认；
不能从一个局部 hypothesis 直接跳到正式 G1 过滤。

## 12. 预计最小实现文件

```text
DT-SLAM/include/GeometricDynamicDetector.h
DT-SLAM/src/GeometricDynamicDetector.cc
DT-SLAM/include/Tracking.h
DT-SLAM/src/Tracking.cc
DT-SLAM/Examples/RGB-D/BONN_*RigidHypothesisShadow.yaml
DT-SLAM/Examples/RGB-D/TUM*_RigidHypothesisShadow.yaml
DT-SLAM/Examples/RGB-D/geometric_warp_test.cc
DT-SLAM/tools/audit_sparse_rigid_hypotheses.py
```

其中 `Tracking` 只负责默认关闭的配置、调用和 CSV；全部数学计算留在
`GeometricDynamicDetector` 的纯函数中。正式实现不得顺带重构现有 G0/G1/G2
代码。
