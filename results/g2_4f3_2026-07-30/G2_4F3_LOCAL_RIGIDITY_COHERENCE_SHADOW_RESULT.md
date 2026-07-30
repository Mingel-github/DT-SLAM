# G2-4F3 局部刚性一致性 Shadow 结果

日期：2026-07-30
状态：第一版连续证据实现与开发审计完成；不形成动态判决；G1 仍锁定

## 1. 结论先行

G2-4F3 得到了值得继续研究的方向性结果，但没有得到可直接用于 SLAM 的动态
分类器。

在冻结的 Bonn `balloon/balloon2` RGB-only 粗框代理中：

```text
框内 ego-flow 中位数 > 同帧框外：15/15 个可比较帧
框内内部边 strain 中位数 <= 背景边：11/14 个可比较帧
跨越框内/框外边 strain 中位数 > 背景边：13/14 个可比较帧
```

聚合到 feature：

| development 序列 | 框内节点 | 框内 flow 中位 | 框外 flow 中位 | 框内同组 strain 中位 | 框外同组 strain 中位 |
| --- | ---: | ---: | ---: | ---: | ---: |
| balloon | 76 | 11.030 px | 0.599 px | 0.00523 m | 0.00946 m |
| balloon2 | 122 | 14.656 px | 0.682 px | 0.00279 m | 0.00785 m |

这与以下 `[S/H]` 假设方向一致：

> 独立运动刚体局部不符合静态相机运动，但其内部点间三维距离可近似保持；
> 对象与背景之间的跨组边则发生更明显变化。

但这些粗框是开发代理，不是像素级对象真值或运动真值；不能据此选择阈值或
宣布泛化。

## 2. 方法身份

| 组件 | 来源与本项目身份 |
| --- | --- |
| 当前图像 Delaunay 邻接 | `[A]` Dai 等 point-correlation 图的局部两帧适配 |
| 相邻两帧三维边长变化 | `[A/H]` 对长期相对位置一致性的简化 |
| 连续 sparse ego-flow residual | `[A]` FlowFusion 原理的稀疏 RGB-D/SE(3) 适配 |
| “高 flow inconsistency + 低内部 strain”联合解释 | `[S/H]`，仍待独立验证 |
| OpenCV `Subdiv2D` | `[S]` 标准工程实现 |

本阶段不是 Dai、FlowFusion、SInDSLAM 或 DetectFusion 复现，也没有实现
长期图优化、对象运动估计或“最大连通组为静态”的假设。

## 3. 实现范围

新增的纯计算路径：

```text
G2-4F1 LK correspondence
→ FB <= 0.25 px correspondence quality condition
→ reference/current metric depth back-projection
→ semantic feature exclusion
→ current-image Delaunay graph
→ absolute and relative 3-D edge strain
→ node incident median/P90
```

输出：

- node CSV：坐标、三维点、flow residual、FB error、邻居数、strain 中位/P90；
- edge CSV：端点、两帧距离、absolute/relative strain、端点 flow/FB/MapPoint/
  semantic 信息；
- frame CSV：有效性计数、重复点、边数和计算时间；
- 每层均保持 `dynamic_decision=none` 与
  `direct_slam_state_mutation=none`。

没有修改：

```text
mvbDynamic
mvpMapPoints
Optimizer.cc / g2o
LocalMapping / LoopClosing
YOLOSegment
PoseOptimization 调用次数
```

## 4. 确定性验证

`geometric_warp_test` 新增：

1. 刚体图像平移：所有有效三维边长变化接近零；
2. 单节点独立位移：至少一条关联边变化显著，背景边保持；
3. 当前深度无效：不产生 rigidity evidence；
4. 重复图像点：显式拒绝，不产生自环或重复边。

当前结果：

```text
[.../G2-4F3 Test] PASS
```

用相同 OpenCV 4.5.4 对一帧静态 C++ CSV 进行 Python 独立回放：

```text
C++ edges      = 2526
Python edges   = 2526
only C++       = 0
only Python    = 0
```

注意：pip OpenCV 4.13 在 Delaunay 退化/并列情况下会比 C++ OpenCV 4.5.4
多 4/2527 条边。因此正式离线回放显式使用：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3
```

## 5. 真静态结果

### Bonn `static_close_far`

150 输入帧，149 个相邻帧测量：

| 指标 | 结果 |
| --- | ---: |
| eligible node / frame 中位 | 814 |
| valid edge / frame 中位 | 2418 |
| graph time 中位 | 1.583 ms |
| total F3 time 中位 / P95 | 1.984 / 2.112 ms |
| static flow 中位 / P90 | 0.299 / 0.829 px |
| static edge absolute strain 中位 / P90 | 0.00249 / 0.02684 m |
| actual FPS / deadline miss | 29.73 / 0/150 |

### TUM `fr1/xyz`

150 输入帧，149 个相邻帧测量：

| 指标 | 结果 |
| --- | ---: |
| eligible node / frame 中位 | 774 |
| valid edge / frame 中位 | 2300 |
| total F3 time 中位 / P95 | 1.902 / 2.103 ms |
| static flow 中位 / P90 | 0.358 / 0.909 px |
| static edge absolute strain 中位 / P90 | 0.00155 / 0.01841 m |
| actual FPS / deadline miss | 28.70 / 2/150 |

FPS 是 F1+F3 shadow 运行健康度，不是 geometry 改善。TUM 低于 30 FPS 也
不能归因于 F3 单项；F3 本身约 1.7 ms。

## 6. Development 代理审计

- 只使用 `balloon/balloon2` development 数据；
- 没有读取 `balloon_tracking` 来选择图、规则或参数；
- 帧由 geometry/flow 之前冻结的 RGB-only temporal proxy 选择；
- box 是 RGB-only 粗框；
- semantic exclusion 复用既有在线 C++ F1 导出的精确
  `semantic_nonzero`；
- current depth 从原始 Bonn ZIP 读取并用与 C++ 相同的 P=K、最近邻规则
  联合去畸变；
- Python 回放使用系统 OpenCV 4.5.4，与 C++ 一致。

限制：

- `balloon` 8 个有框帧中，2 帧没有框内有效节点，另 1 帧只有 1 个；
- strain 可比较帧合计 14；
- `balloon2` frame 230 出现约 2 m 的框内 strain，显示粗框、遮挡和混合深度
  能完全破坏局部刚性测量；
- absolute strain 的跨界分布较符合预期，但当前 relative strain 因长跨界边
  分母较大，并不稳定优于 absolute strain；
- 当前结果不能证明所有未知动态或低纹理箱子可测。

relative strain 不得直接作为下一阶段默认分数，frame 230 也不得作为“调一个
阈值即可消除”的异常点静默删除。

## 7. 性能与工程判断

第一版 F3 图和统计的额外计算约 1.9–2.0 ms，未破坏 Bonn 30 Hz 短测。最终 5 帧
smoke 使用当前完整端点 CSV schema，仍为 29.75 FPS、0 deadline miss。

诊断 CSV 会积累大量 edge 行；完整实验应使用 frame filter 或关闭 edge/node
记录。CSV 内存/写盘不是在线算法预算的一部分，必须与计算时间分开报告。

## 8. 当前决定

```text
F3 pure measurement implementation     = PASS
synthetic correctness                  = PASS
static runtime feasibility             = PASS
development directional hypothesis     = SUPPORTED, not validated
rigidity threshold                     = none
rigid/dynamic component                = none
dynamic decision                       = none
G1-F / G1-D                            = locked
```

下一步不是直接创建 `dynamic=true`，而是先进行有文献依据的 F3 后续设计审计：

1. 核对 Dai 等对 edge consistency、协方差/残差和图分组的原始定义；
2. 核对适用于 RGB-D 深度噪声的标准不确定性处理；
3. 明确如何避免长跨界边使 relative strain 失真；
4. 只提出一个连续 edge reliability / local coherence 表达，不先选 hard
   threshold；
5. 在实现前另行冻结新的验证协议，已经打开的 `balloon_tracking` 不得用于
   调参。

若无法得到有依据且可测的鲁棒边一致性表达，应在本阶段停止，不靠经验阈值
继续修补。

## 9. 主要证据

- `static150.log`
- `static150_nodes.csv{,.edges.csv,.frames.csv}`
- `tum_fr1_xyz_static150.log`
- `tum_fr1_xyz_static150_nodes.csv{,.edges.csv,.frames.csv}`
- `development_proxy_audit/balloon_{nodes,edges,per_frame}.csv`
- `development_proxy_audit/balloon2_{nodes,edges,per_frame}.csv`
- `development_proxy_audit/*_summary.json`
- `static_cpp_python_graph_parity.json`
- `g2_4f3_final_static_summary.json`
- `development_proxy_audit/combined_node_summary.json`
