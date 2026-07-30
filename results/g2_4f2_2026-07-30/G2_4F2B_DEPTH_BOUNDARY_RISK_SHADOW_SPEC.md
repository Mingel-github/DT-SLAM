# G2-4F2B 稀疏 Flow 参考深度边界风险 Shadow SPEC

日期：2026-07-30  
状态：实现前冻结  
范围：只增加诊断字段和离线分层；不产生动态判决；不修改 SLAM 状态

## 1. 触发原因

在相同的冻结诊断网格 `FB <= 0.25 px` 下，Bonn
`rgbd_bonn_static_close_far` 的静态序列候选率高于 TUM `fr1/xyz`：

| q | TUM fr1/xyz | Bonn static close/far |
| ---: | ---: | ---: |
| 4 | 0.768% | 2.340% |
| 6 | 0.355% | 1.193% |
| 8 | 0.253% | 0.771% |
| 10 | 0.229% | 0.558% |

这些数值是 `static-sequence candidate rate`，不是有像素真值的 FPR。
按 G2-4F2 冻结 SPEC，当前不能通过抬高 `q` 或查看 strict holdout 选择参数，
而应先检查同步、坐标域、位姿和深度边界风险。

已确认：

- Bonn RGB 与 registered depth 共同 remap 到 `P=K` 无畸变域；
- 150 帧运行无 deadline miss；
- GT 插值的默认 20 ms bracket 仅覆盖 34/150 帧；
- 将仅用于 GT 诊断的 bracket 上界设为 40 ms 后覆盖 149/150 帧；
- GT 诊断不参与部署证据，也不改变 SLAM 位姿。

## 2. 文献依据

### 2.1 DynaSLAM `[L]`

Bescos et al., *DynaSLAM: Tracking, Mapping, and Inpainting in Dynamic
Scenes*, IEEE RA-L 2018，在几何动态关键点产生后指出：

> 一些被判为动态的关键点位于运动物体边缘；若该点周围 depth patch
> 方差较高，则将其改回静态。

本地原文：

```text
/home/zhu/Desktop/papers/2018_DynaSLAM_Tracking_Mapping_Inpainting.pdf
```

本地官方代码：

```text
/home/zhu/dynaslam_ws/DynaSLAM/src/Geometry.cc
```

代码使用 41×41 patch (`mDmax=20`) 和 `variance < 0.001`。这些实现常数属于
DynaSLAM 自身的 sparse projected-depth seed，不直接适用于当前
sparse LK ego-flow residual。

### 2.2 SInDSLAM `[L]`

Qi et al., *Semantic-Independent Dynamic SLAM Based on Geometric
Re-Clustering and Optical Flow Residuals*, IEEE T-ITS 2025，将深度不连续
定义为：

\[
\delta_D(u)=\max_{n\in\mathcal N(u)}|D(n)-D(u)|,
\]

\[
\operatorname{edge}(u)
\iff
\delta_D(u)>\max(\tau_1D(u),\tau_2),
\]

并报告：

```text
tau_1 = 0.025
tau_2 = 0.08 m
```

原方法还包含 K-means、平面边缘、re-clustering 和 dense optical flow。
本阶段不复现这些组件。

## 3. 本阶段适配 `[A/S]`

对每个具有有效参考深度的 G2-4F1 sample，在参考帧 rectified depth 域记录：

```text
reference_depth_boundary_d1
reference_depth_boundary_d2
reference_invalid_depth_d1
reference_invalid_depth_d2
```

含义：

- `boundary_d1/d2`：以 sample 的参考像素为中心，Chebyshev 半径 1/2
  内是否存在按 SInDSLAM 公式定义的深度不连续像素；
- `invalid_depth_d1/d2`：同一邻域内是否存在无效深度；
- 中心点深度无效时仍保持原有 `DepthInvalid/no_evidence`，不得产生风险字段
  驱动的 motion evidence。

该设计：

- 使用 SInDSLAM 的边界公式作为可追溯风险定义 `[A]`；
- 使用 1/2 pixel 风险带与项目既有 G2-4A 审计保持一致 `[S]`；
- 只检查当前 F1 residual 是否集中在高风险位置 `[H]`；
- 不复制 DynaSLAM 的 patch 大小、方差阈值或 relabel 行为。

## 4. 离线分层

对每个已经冻结的 `(FB, q)` 诊断点，分别报告：

- 各风险带内 quality-eligible 数；
- 各风险带内 candidate 数及 candidate rate；
- 各风险带承载全部 candidate 的比例；
- 同时远离 boundary/invalid depth 两像素的 `clean_d2` 结果；
- MapPoint 的相同分层；
- SLAM pose 与 GT pose 分支分别报告。

本阶段不选择 operating point。

## 5. 判定

### 若 Bonn 额外候选主要集中在 boundary/invalid 风险带

只允许得出：

> reference-depth boundary risk explains part of the cross-domain
> candidate-rate increase.

后续可研究有文献依据的边界降权或 no-evidence 处理，但仍需单独验证动态目标
召回损失。

### 若 clean_d2 仍明显偏高

边界不能解释跨域差异。继续检查：

- RGB/depth 时间偏移；
- SLAM 初始位姿；
- GT frame transform；
- LK 遮挡与重复纹理；
- 相机快速近远运动造成的深度量化/可见性变化。

不得继续添加组合阈值。

## 6. 不变量

```text
dynamic_decision           = none
direct_slam_state_mutation = none
depth_flow_fusion          = none
G1-F / G1-D                = locked
strict holdout             = sealed
YOLO / Optimizer / g2o     = unchanged
```

