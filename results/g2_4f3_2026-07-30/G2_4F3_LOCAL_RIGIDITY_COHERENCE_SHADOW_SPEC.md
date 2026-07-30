# G2-4F3 局部刚性一致性 Shadow SPEC

日期：2026-07-30  
状态：实现前冻结草案  
作用域：只读诊断；不产生动态标签；不修改 SLAM 状态

## 1. 研究问题

G2-4F2 strict holdout 证明：

```text
连续 sparse ego-flow residual 通常在运动气球局部富集；
全帧 robust scale 上的 q>=10 hard candidate 只有 50% proxy sensitivity。
```

F3 不修改 `q`，而是独立测量：

> 静态相机运动模型不一致的 feature，是否同时属于一个内部三维几何关系稳定的
> 局部刚性组。

## 2. 文献与适配身份

来源账本：

| 组件 | 来源 | 本项目使用 | 性质 |
| --- | --- | --- | --- |
| 稀疏邻接图 | Dai 等 point correlation 的 Delaunay graph | 当前帧有效 ORB/LK/RGB-D 点建局部图 | `[A]` |
| 相对位置长期一致 | Dai 等 | 简化成相邻两帧 edge length change | `[A/H]` |
| flow residual | FlowFusion | 复用现有 sparse RGB-D/SE(3) residual | `[A]` |
| residual 需要空间上下文 | SInDSLAM、DetectFusion | 只作局部图联合审计 | `[A/S]` |
| 组合条件 | 多来源综合 | static-model violation + internal rigidity | `[S/H]` |

不得称为 Dai、FlowFusion、SInDSLAM 或 DetectFusion 复现。

## 3. 输入

复用 G2-4F1 已有数据，不改 LK：

```text
current rectified grayscale
reference rectified grayscale
current CV_32F metric depth
reference CV_32F metric depth
current ORB keypoints
backward/forward LK correspondence
FB error
sparse ego-flow residual vector/magnitude
semantic_nonzero
camera intrinsics K
```

只接受：

```text
LK forward/backward status valid
FB <= 0.25 px
reference depth valid
current depth valid
both 3D points finite and z > 0
semantic_nonzero == false
```

任一条件失败都输出 `no_rigidity_evidence`，不能解释成静态。

## 4. 图与测量

### 4.1 顶点

每个顶点对应当前 ORB feature index，并保存：

```text
u_t, v_t
X_t in current camera coordinates
u_r, v_r
X_r in reference camera coordinates
continuous ego-flow residual
MapPoint presence
```

### 4.2 邻接

第一版优先使用当前图像平面的 Delaunay triangulation，以贴近 Dai 等的稀疏图
原型。必须处理重复/近重复 keypoint，且每条无向边只记录一次。

如果 OpenCV Delaunay 的 feature-index 回映射不稳定，则停止并报告；不能静默
换成全连接图。k-NN 只能作为另行标注的 `[A]` 对照。

### 4.3 Edge strain

对边 \((i,j)\)：

\[
d_{ij,r}
=
\|\mathbf X_{i,r}-\mathbf X_{j,r}\|_2,
\qquad
d_{ij,t}
=
\|\mathbf X_{i,t}-\mathbf X_{j,t}\|_2,
\]

\[
\epsilon_{ij}
=
|d_{ij,t}-d_{ij,r}|.
\]

同时记录相对量：

\[
\epsilon^{rel}_{ij}
=
\frac{|d_{ij,t}-d_{ij,r}|}
{\max(\epsilon_d,\frac12(d_{ij,t}+d_{ij,r}))}.
\]

`epsilon_d` 只作除零保护，不是刚性阈值。第一版不选择
`epsilon` 或 `epsilon_rel` 的 dynamic/static cutoff。

## 5. 输出

### 5.1 Edge-level

```text
frame
node_i / node_j
reference/current 3D distance
absolute/relative edge strain
endpoint residual magnitudes
endpoint FB errors
endpoint semantic flags
endpoint MapPoint flags
endpoint bbox stratum（仅离线开发审计）
evidence_state
```

### 5.2 Node/component-level

第一版只允许：

```text
valid neighbor count
incident edge strain median/p90
residual magnitude
joint residual–strain histogram
```

在没有冻结 edge threshold 前，不建立“rigid component”，避免名称先于证据。

### 5.3 不变量

每帧继续输出：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```

不修改：

```text
mvbDynamic
mvpMapPoints
Optimizer.cc
g2o
LocalMapping
LoopClosing
```

## 6. 实验顺序

1. 确定性合成测试：
   - 同一刚体刚性平移/旋转，edge strain 接近零；
   - 一个节点独立位移，其跨组边 strain 增大；
   - invalid depth 不产生 edge；
   - 重复图像点不产生自环/重复边。
2. 真静态：
   - TUM `fr1/xyz`；
   - Bonn `static_close_far`。
3. development：
   - Bonn `balloon`；
   - Bonn `balloon2`。
4. 只比较预先冻结的 RGB-only bbox proxy 内外连续分布。
5. 不使用 `balloon_tracking` 选择图参数、阈值或规则。
6. 在开发完成前另行冻结新的验证数据；未冻结前不宣布可泛化。

## 7. 预先定义的审计量

不做 hard classification，报告：

```text
valid node/edge coverage
edges per frame
degree distribution
absolute/relative strain distribution
inside vs outside strain
inside vs outside continuous residual
joint residual–strain distribution
MapPoint subset
GT-pose diagnostic（若有）
compute time
memory/logging time
```

评估问题：

- 运动对象框内是否存在“高 residual、低内部 strain”群体；
- 静态背景高 residual 是否更常伴随高/不一致 strain；
- 低纹理对象是否因节点不足而不可评价；
- current depth 噪声是否淹没 edge strain；
- 相邻帧基线是否过小，导致所有 edge strain 都接近噪声。

## 8. 停止条件

以下任一成立则冻结为负结果：

```text
动态 proxy 内有效图节点长期不足；
static 与 moving proxy 的 joint distribution 没有稳定差异；
edge strain 主要由深度噪声支配；
需要读取 balloon_tracking 回调阈值才有分离；
CPU 成本破坏 30 FPS 且无明确区分收益。
```

即使 shadow 分布乐观，下一步也只能冻结一个新的、未见验证协议；不能直接进入
G1-F。
