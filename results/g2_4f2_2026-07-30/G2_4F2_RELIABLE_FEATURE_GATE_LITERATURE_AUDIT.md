# G2-4F2 可靠 Feature Evidence Gate 文献审计

日期：2026-07-30  
状态：文献归属与方法边界冻结；尚未形成动态判决  
范围：G2-4F1 稀疏 ego-flow 连续证据之后、G1-F 真实过滤之前

## 1. 审计结论

G2-4F1 已证明：

```text
observed sparse LK flow - RGB-D/SE(3) ego flow
```

在非 holdout 的 Bonn `balloon/balloon2` 开发帧中，对语义未覆盖的独立运动
气球存在明确方向性证据。但现有材料不支持把某个固定像素值直接写成
`dynamic=true`。

下一阶段必须拆为：

```text
G2-4F2Q:
  correspondence quality
  + continuous residual
  + robust scale/static likelihood
  → quality-eligible continuous feature evidence

G2-4F2D:
  frozen static-risk calibration
  + development sensitivity
  + sealed holdout validation
  → conservative high-residual candidate operating point
```

F2Q 不是动态分类器；F2D 即使通过，也只产生
`high-confidence geometry dynamic candidate`，仍需 G1-F 的保底条件和真实
ATE/RPE 对照。

## 2. 材料与检索顺序

先读取本地 PaperNotes 和原始 PDF：

```text
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/pps_slam.md
/home/zhu/Desktop/paper_notes/Panoptic-SLAM.md
/home/zhu/Desktop/paper_notes/dynaslam.md
/home/zhu/Desktop/papers/Qi 等 - 2025 -
  Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering
  and Optical Flow Residuals.pdf
/home/zhu/Desktop/papers/2025_PPS-SLAM_Precise_Pruning.pdf
/home/zhu/Desktop/papers/Abati 等 - 2024 -
  Panoptic-SLAM Visual SLAM in Dynamic Environments using
  Panoptic Segmentation.pdf
```

本地没有下列原文，因此只对缺口使用 primary/authoritative web fallback：

```text
Zhang et al., FlowFusion, ICRA 2020
  arXiv:2003.05102

Li and Lee, RGB-D SLAM in Dynamic Environments Using Static
  Point Weighting, IEEE RA-L 2017
  DOI: 10.1109/LRA.2017.2724759
  author-hosted PDF:
  https://mediatum.ub.tum.de/doc/1375854/document.pdf

Kalal, Mikolajczyk and Matas, Forward-Backward Error:
  Automatic Detection of Tracking Failures, ICPR 2010
  DOI: 10.1109/ICPR.2010.675
  author-hosted PDF:
  https://cmp.felk.cvut.cz/~matas/papers/kalal-2010-fb_track-icpr.pdf

OpenCV calcOpticalFlowPyrLK documentation
  https://docs.opencv.org/4.5.4/dc/d6b/group__video__track.html
```

## 3. 逐组件来源与可迁移边界

### 3.1 Optical-flow residual

`[L] FlowFusion` 定义 camera ego flow：

\[
\delta\mathbf x^e_{A\rightarrow B}
=
\mathcal W(\mathbf x,\xi)-\mathbf x,
\]

并将 projected scene-flow/flow residual 写为：

\[
\delta\mathbf x^{sf}_{A\rightarrow B}
=
\delta\mathbf x^{of}_{A\rightarrow B}
-
\delta\mathbf x^e_{A\rightarrow B}.
\]

静态像素在正确对应和正确位姿下接近零，独立运动像素通常非零。

它支持 G2-4F1 的物理测量，但原方法同时包含：

- dense PWC-Net optical flow；
- supervoxel；
- intensity/depth/flow 多残差；
- 连续 dynamic level；
- 迭代相机位姿与动态分割。

因此本项目只能写成：

> `[A/S]` FlowFusion ego-flow residual principle adapted to adjacent-frame
> sparse ORB/LK observations and an ORB-SLAM2 RGB-D initial pose.

FlowFusion 的 `alpha_F`、cluster threshold 或 dynamic level 不能直接迁移。
原文还报告轻微运动和极快运动会退化，因此小 residual 不能自动证明静态，
无效 optical flow 也不能解释为静态。

### 3.2 Forward-backward error

`[L] Kalal et al. 2010` 将同一点先正向跟踪、再反向跟踪，并以返回位置与
起始位置的距离定义 forward-backward error。其目的为：

```text
tracking failure detection / trajectory reliability
```

而不是：

```text
independent object motion evidence
```

因此 F2Q 可以使用：

\[
e_{\mathrm{FB},i}
=
\left\|
\mathbf u_{t,i}^{fb}-\mathbf u_{t,i}
\right\|_2
\]

来判断 LK correspondence 是否可靠，但不得使用：

```text
large FB error => dynamic object
```

大 FB error 更可能表示遮挡、出界、弱纹理、光照变化或 LK 失败。Kalal 原文
说明 hard decision 需要 thresholding，但没有为当前相机、分辨率、LK 参数
提供通用阈值。因此 `tau_FB` 必须作为当前系统的 calibration quantity，
不能伪称论文参数。

OpenCV 的 `status`、`err` 和 `minEigThreshold` 是 LK 数值/跟踪接口字段。
其中 `minEigThreshold=1e-4` 是当前 OpenCV 默认设置，不是动态判决阈值。

### 3.3 MAD 与 Student-t 静态软权重

`[L] Li and Lee 2017` 对变换后的 3D depth-edge correspondence distance
\(d_i\) 使用：

\[
\sigma_D
=
1.4826\,
\operatorname{median}|d_i-\mu_D|,
\]

\[
w_i
=
\frac{\nu_0+1}
{\nu_0+\left((d_i-\mu_D)/\sigma_D\right)^2}.
\]

论文中：

- \(\mu_D=0\)，因为理想静态 correspondence distance 为零；
- 无有效 correspondence 的点不参与尺度估计；
- \(\nu_0=10\) 为经验值；
- 权重进入 IAICP，而不是先二值删除所有低权重点；
- keyframe 还同时与上一关键帧和当前帧比较，以改善慢运动可分性。

它支持：

- 对非负 residual 使用零中心鲁棒尺度；
- 用连续 static likelihood/weight 代替一个绝对米制或像素阈值；
- invalid correspondence 不参与尺度估计；
- 先保留软证据，再决定是否进入优化。

它不直接支持：

- 把 3D edge correspondence distance 原样替换成 sparse 2D flow residual 后
  宣称复现；
- 直接采用 \(\nu_0=10\) 得到动态二值标签；
- 采用某个 `w<0.5` 之类未在原文给出的阈值；
- 在 unknown dynamic features 占多数时仍假定 MAD scale 未被污染。

因此本项目若采用：

\[
\hat\sigma_t
=
\max\left(
\sigma_{\min},
1.4826\operatorname{median}_{i\in\mathcal E_t} r_i
\right),
\qquad
q_i=\frac{r_i}{\hat\sigma_t},
\]

其中 \(r_i=\|\mathbf f_{\mathrm{res},i}\|_2\)，只能标为：

> `[A/H]` Li–Lee zero-centered robust static scale adapted from 3D
> correspondence distance to 2D ego-flow residual.

该假设必须通过真正静态序列和不同相机域验证。

### 3.4 SInDSLAM 双阈值

`[L] SInDSLAM` 对 dense optical-flow residual histogram 使用 Triangle
算法得到 `tau_low`，再设：

\[
\tau_{\mathrm{high}}=1.3\tau_{\mathrm{low}}.
\]

高残差、低残差和静态像素随后只在 geometric re-cluster 内传播；并非把
全图每个高 residual sparse feature 直接删除。

该方法支持：

- 固定单阈值对不同 residual 分布不稳健；
- 可以保留高、低、静态/不确定三种状态；
- residual decision 需要区域和 correspondence 可靠性约束。

它不支持当前直接迁移 Triangle 阈值，因为当前分布是：

- 每帧约 1000 个稀疏 ORB，而不是 dense per-pixel flow；
- dynamic object 可能只有极少 feature；
- 当前没有其 re-clustering、PROSAC prior 或 cluster-confined filling；
- 当前 residual 来自 RGB-D/SE(3)，不是 dense flow 与 homography。

所以 Triangle 只能作为未来离线对照 `[A/H]`，不能作为默认 F2 gate。

### 3.5 固定像素阈值为什么不迁移

`[L] PPS-SLAM` 在 semantic detection region 内对 optical-flow epipolar
distance 使用 2 px；`[L] Panoptic-SLAM` 对 data-associated unknown-object
keypoint 的 epipolar distance 使用 0.1 px。

二者的：

- residual 定义；
- candidate support；
- 相机模型；
- 分辨率和特征关联；
- semantic/instance 前置条件；

均与 G2-4F1 不同。两个相差 20 倍的公开阈值本身也说明固定像素值不是可直接
继承的通用常数。

因此明确驳回：

```text
slam_residual > 0.1 px => dynamic
slam_residual > 2.0 px => dynamic
slam_residual > 11.9 px => dynamic
```

其中 `11.9 px` 只是当前 6 个可测气球开发帧的框内中位数。

## 4. F2Q 建议证据表示

每个当前 ORB feature 保留相互独立的三类字段：

```text
measurement_state:
  measured | no_evidence

correspondence_quality:
  raw FB error + LK status

motion_inconsistency:
  raw ego-flow residual magnitude/vector

robust_static_evidence:
  frame robust scale + normalized residual + optional soft weight
```

核心约束：

```text
FB error 只决定“能不能信这个对应”
flow residual 只表达“是否违反静态相机运动”
robust weight 只表达连续静态似然
上述任何单项都不直接等于 dynamic=true
```

建议的软权重仅作审计：

\[
w_i^{\mathrm{static}}
=
\min\left(
1,
\frac{\nu+1}{\nu+q_i^2}
\right),
\quad \nu=10.
\]

`nu=10` 继承 Li–Lee 的实验值为一个对照；是否适合当前 2D residual 属于
`[H]`，不允许直接进入 G1。

## 5. F2D 的阈值来源

没有找到能合法提供本项目通用阈值的论文。因此 F2D 应采用透明的
development calibration，而不是伪造文献参数：

1. 在冻结的真正静态序列上报告 quality-eligible feature 的 residual/F-B
   分布；
2. 报告一组静态 candidate-rate 工作曲线，而不是先选一个结果最好看的点；
3. operating point 只按预先声明的静态风险预算选择；
4. 再评价该点在 `balloon/balloon2` 开发 proxy 上的 sensitivity；
5. 全部设计冻结后，才允许一次性打开 strict holdout；
6. holdout 失败时不得回头改阈值后重复宣称通过。

静态 candidate rate 是“所有 feature 预期静态”条件下的上界代理，不是拥有
pixel GT 的严格 false-positive rate。必须如实命名：

```text
static-sequence candidate rate
```

## 6. 当前允许与禁止

允许：

- F2Q 连续质量/静态似然字段；
- 离线 static-risk curve；
- 继续保持 semantic feature exclusion 的分层统计；
- `fr1/xyz` 和 Bonn 官方 static 序列作负样本；
- `balloon/balloon2` 作 development；
- 在规格冻结后一次性使用 `balloon_tracking` 作 strict holdout。

禁止：

- 现在解封 holdout；
- 把 FB error 当 motion residual；
- 把 LK photometric `err` 当动态标签；
- 复制 `0.1 px`、`2 px`、`1.3×` 或 `nu=10` 后直接过滤；
- 根据 6 个气球帧的中位数拟合阈值；
- 把无深度、无对应或低质量 feature 解释成静态；
- 将 G2 depth vote 与 flow score 相加以制造分离；
- 修改 `mvbDynamic`、`mvpMapPoints`、Optimizer、g2o 或后端；
- 新增第三次 `PoseOptimization`。

## 7. 决策

G2-4F2 的科学身份冻结为：

> `[A/S/H]` A conservative feature-evidence gate combining a
> FlowFusion-derived sparse ego-flow residual, Kalal-style
> forward-backward correspondence validation, and a Li–Lee-inspired
> robust continuous static likelihood, calibrated by an explicit
> static-sequence risk budget.

这不是任何单篇论文的复现，组合能否有效属于实验假设。下一步先实现离线
F2Q/F2D 审计与静态序列采样，不进入真实 SLAM 过滤。

