# G2-4F3U 不确定性归一化边一致性：文献审计

日期：2026-07-30
状态：已核对原始 PDF 与 arXiv TeX；用于冻结 F3U shadow 实验
前提：F3 已输出 absolute/relative edge strain，但 relative strain 会因长边分母而失真

## 1. 本地优先检索结果

先检查了：

```text
/home/zhu/Desktop/paper_notes/
/home/zhu/Desktop/papers/
/home/zhu/dynaslam_ws/results/
```

本地已有 SInDSLAM、DetectFusion、FlowFusion 的笔记或原文，但没有 Dai
等 point-correlation 原文，也没有可用于核对深度平方噪声系数的原始资料。
因此只对这两个缺口下载了原始论文：

1. Dai et al., *RGB-D SLAM in Dynamic Environments Using Point Correlations*,
   TPAMI 2022 / arXiv:1811.03217v2；
2. Khoshelham and Elberink, *Accuracy and Resolution of Kinect Depth Data
   for Indoor Mapping Applications*, Sensors 2012.

本地原始证据保存在 `literature/`。

## 2. Dai 原方法的准确定义

### 2.1 图边不是单纯的边长比值

`[L]` Dai 将地图点间的 correlation edge 定义为三维相对位置：

\[
\mathbf l_{ij}=\mathbf p_i-\mathbf p_j.
\]

RGB-D 对边的观测为：

\[
\mathbf z_{ijk}=\mathbf y_{ik}-\mathbf y_{jk}.
\]

原方法在多帧地图点图上优化 edge state，并用边残差的平方马氏长度
剔除不一致观测：

\[
\mathbf e_{ijk}^{T}\mathbf C_{ijk}^{-1}\mathbf e_{ijk}.
\]

根据 P-value 选择卡方阈值，且使用 Huber norm 增强鲁棒性。

### 2.2 边协方差来自两个端点

`[L]` 若两个点的测量独立，Dai 使用：

\[
\mathbf C_{ijk}=\mathbf C_{ik}+\mathbf C_{jk}.
\]

每个 RGB-D 点的三维协方差由像素坐标和深度测量向三维点的不确定性传播得到。
论文还在 `3×3` 深度邻域内使用 Gaussian mixture，以提高物体深度边界处的
不确定性。

### 2.3 前端确实使用两帧

`[L]` Dai 的后端维护多关键帧 point-correlation graph；其前端使用上一帧
已跟踪地图点构图，并用当前/上一帧的观测检查边一致性。

这支持 F3 做两帧 shadow 诊断，但不支持称其为 Dai 复现，因为当前没有：

- 长期地图点 edge state；
- 多帧 point-correlation optimization；
- 卡方离群剔除迭代；
- connected-component 静态组判定；
- 对 `Optimizer.cc` 或 BA 的任何改动。

## 3. Kinect/Xtion 深度噪声依据

`[L]` Khoshelham 由 structured-light disparity 误差传播得到：

\[
\sigma_Z=\left|\frac{m}{fb}\right|Z^2\sigma_{d'}.
\]

论文实验使用：

```text
|m/fb| = 2.85e-5   (在论文使用的 cm 单位下)
sigma_d' = 0.5 pixel
```

换成深度以米表示时：

\[
\sigma_Z \approx 0.001425 Z^2\ \text{m}.
\]

这一系数来自 Kinect-style structured-light 测量，不是对 TUM/Bonn 中每台实体
相机的重新标定。因此只能作为 `[L/A]` shadow 噪声模型，不能将其归一化
分数当作精确统计置信度。

## 4. 对当前 F3 的关键修正

当前 F3 的 relative strain 是：

\[
r_{rel}=\frac{|d_t-d_r|}{\max(\epsilon_d,(d_t+d_r)/2)}.
\]

该表达式是 `[S/H]` 工程尺度化，不是 Dai 公式。它会使长边的分数天然变小，
即使边长变化已明显超过深度测量噪声。

有依据的修正方向是：

1. `[L/A]` 用端点深度测量不确定性传播到 edge length；
2. `[A/S]` 将参考帧和当前帧 edge-length variance 相加；
3. `[S]` 输出连续分数：

\[
q_e=\frac{|d_t-d_r|}{\sqrt{\operatorname{Var}(d_t-d_r)}};
\]

4. 保留 absolute strain 作为原始量，relative strain 只作旧对照；
5. 不根据 Dai 的卡方阈值立即产生 hard edge class，因为当前只有两帧、
   协方差也没有针对实体传感器重新标定。

## 5. 结论

```text
literal relative-strain threshold     = reject
Dai-style covariance principle        = supported
Kinect depth-square noise model        = supported, sensor-transfer caveat
hard chi-square edge culling           = not yet authorized
Optimizer / map graph modification     = out of scope
```

F3U 应先实现一个可审计的连续 uncertainty-normalized edge score，再检查它是否
降低距离、深度边界和长边对 strain 的混淆。
