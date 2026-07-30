# G2-4F3U 不确定性归一化边一致性 Shadow SPEC

日期：2026-07-30
状态：实现前冻结
作用域：连续诊断；不产生 dynamic/static 标签；不修改 SLAM 状态

## 1. 研究问题

F3 表明：

- 运动代理框内 ego-flow residual 稳定高于框外；
- 内部边 absolute strain 在大多数可比帧不高于背景；
- 跨框边 absolute strain 在大多数可比帧高于背景；
- 但 relative strain 因长边分母而表现不稳定，深度混合也会产生米级异常。

F3U 只回答：

> 按 RGB-D 深度测量不确定性归一化后，边一致性的连续分布是否比
> `absolute strain / edge length` 更少受深度距离、长边和深度边界影响？

## 2. 方法身份

| 组件 | 来源 | 当前用法 | 性质 |
|---|---|---|---|
| 点/边协方差 | Dai et al. | 只用其测量不确定性思想，不优化图 | `[A]` |
| 深度平方噪声 | Khoshelham and Elberink | `sigma_z = 0.001425 z^2 m` | `[L/A]` |
| `3×3` 深度 mixture | Dai et al. / Dryanovski et al. | 将深度断层和邻域混合体现为更大轴向不确定性 | `[A]` |
| edge-length variance | 标准一阶误差传播 | 将端点轴向深度方差传到两点距离 | `[S]` |
| 连续 normalized score | 上述组合 | 仅输出 shadow score | `[S/H]` |

不得称为 Dai point-correlation reproduction 或卡方动态检测器。

## 3. 输入与有效性

复用 F3 所有条件：

```text
LK forward/backward valid
FB <= 0.25 px
semantic_nonzero == false
reference/current depth valid
Delaunay edge valid
```

新增输入只有完整 reference depth，用于计算参考端点的 `3×3` 邻域不确定性。

不能将下列情况解释为静态：

- 中心深度无效；
- `3×3` 邻域无任何有效深度；
- edge-length Jacobian 不可计算；
- 传播方差非有限或非正。

## 4. 连续测量

### 4.1 单像素轴向基础噪声

\[
\sigma_z(z)=0.001425z^2.
\]

系数固定来自 Khoshelham，不用 development 数据拟合。

### 4.2 `3×3` 深度 mixture

使用 Dai 文中的高斯核：

\[
W=\frac1{16}
\begin{bmatrix}
1&2&1\\2&4&2\\1&2&1
\end{bmatrix}.
\]

对有效邻域权重重新归一化，并计算：

\[
\mu_z=\sum_q w_qz_q,
\qquad
\sigma^2_{z,mix}=\sum_qw_q(\sigma_z(z_q)^2+z_q^2)-\mu_z^2.
\]

重新归一化是 `[S]` 对无效深度的工程处理，必须输出 valid weight，不得隐藏该适配。

### 4.3 边长方差

对端点反投影射线：

\[
\mathbf r(u,v)=
[(u-c_x)/f_x,(v-c_y)/f_y,1]^T,
\quad
\mathbf X=z\mathbf r.
\]

对边 `e = Xi-Xj` 和单位方向 `n=e/||e||`，只传播轴向深度不确定性：

\[
\operatorname{Var}(d)
=
(\mathbf n^T\mathbf r_i)^2\sigma_{z,i}^2
+
(\mathbf n^T\mathbf r_j)^2\sigma_{z,j}^2.
\]

当前和参考测量假定独立：

\[
\sigma^2_{\Delta d}=\operatorname{Var}(d_t)+\operatorname{Var}(d_r).
\]

### 4.4 输出

```text
reference/current depth mixture std [m]
reference/current valid neighborhood weight
edge delta-length standard deviation [m]
uncertainty-normalized absolute strain
node incident normalized median/P90
```

归一化分数：

\[
q_e=|d_t-d_r|/\max(\epsilon_\sigma,\sigma_{\Delta d}).
\]

`epsilon_sigma` 只防除零，不是 dynamic threshold。

## 5. 确定性测试

1. 等深度刚体平移：absolute 和 normalized strain 均接近零；
2. 单节点深度改变：关联边 normalized score 增大；
3. 相同米制 strain 在更远深度下获得更小的 normalized score；
4. `3×3` 深度跳变使 mixture std 大于平滑邻域；
5. invalid depth 不产生有效 uncertainty score；
6. 保留 F3 已有图确定性与 C++/Python 一致性。

## 6. 实验顺序

1. 只构建 `geometric_warp_test` 和 `rgbd_tum`；
2. 短真静态：Bonn `static_close_far` 与 TUM `fr1/xyz`；
3. development：仅 `balloon/balloon2` 的冻结 RGB-only 粗框；
4. 不读取 `balloon_tracking` 选阈值；
5. 不选 hard score threshold；
6. 报告 CPU active time 与 CSV 记录时间。

## 7. 审计问题

- normalized score 与 absolute strain 的深度距离相关性是否降低？
- 长跨界边是否不再因边长分母而被自动压低？
- 深度断层附近是否获得更大不确定性，而非被错当成高置信运动？
- 框内内部边、框外背景边、跨界边的连续分布是否符合物理假设？
- F3U 额外计算是否保持在短测可接受范围？

## 8. 停止条件

任一成立就冻结为负结果，不继续补阈值：

```text
uncertainty score 主要由 numerical floor 决定；
mixture 将大量正常斜面系统性降权；
normalized score 仍被长边或深度距离主导；
static/development 联合分布没有稳定方向性；
传感器系数轻微变动就完全改变顺序；
需要打开 holdout 反复调参才可分。
```

## 9. 不变量

```text
dynamic_decision=none
direct_slam_state_mutation=none
G1-F=locked
G1-D=locked
```

不修改 YOLO、`Optimizer.cc`、g2o、LocalMapping 或 LoopClosing。
