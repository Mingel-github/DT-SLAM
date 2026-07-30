# G2-4F3U 不确定度归一化边长应变 Shadow 结果

日期：2026-07-30

## 1. 结论

G2-4F3U 的测量链、日志和确定性测试均通过，但“用 Kinect 轴向深度不确定度
归一化两帧边长变化，会使运动刚体内部比静态背景更一致”这一假设没有通过
当前 development proxy 审计。

因此本阶段结论为：

```text
measurement implementation       = PASS
default coherence score          = REJECT
dynamic_decision                 = none
direct_slam_state_mutation       = none
G1-F / G1-D                      = locked
```

不得根据这些已打开的 development 帧选择 hard threshold，也不得将归一化分数
写入 `mvbDynamic`、清除 `mvpMapPoints` 或改变地图写入。

## 2. 方法身份与文献边界

- `[L]` Dai 等使用点关联边、观测协方差和 Mahalanobis 距离进行长期刚体关系
  检验；原方法不是本项目的两帧标量边长差。
- `[L/A]` Khoshelham 与 Elberink 给出结构光 Kinect 轴向深度标准差随距离平方
  增长的模型；本项目以米为单位使用 `sigma_z = 0.001425 z^2`。
- `[S]` 本项目把端点轴向深度不确定度一阶传播到边长变化分母。
- `[S/H]` 连续分数
  `q_edge = |l_cur-l_ref| / max(sigma_delta_l, epsilon)` 是待验证工程假设，
  不是 Dai 方法复现，也不是新的动态类别判决器。

详细来源与公式见：

- `G2_4F3U_UNCERTAINTY_NORMALIZED_EDGE_LITERATURE_AUDIT.md`
- `G2_4F3U_UNCERTAINTY_NORMALIZED_EDGE_SHADOW_SPEC.md`

## 3. 实现内容

在既有 F3 两帧 Delaunay 局部图上新增：

1. 端点 3x3 深度邻域的轴向测量不确定度；
2. 有效邻域权重归一化和深度混合方差；
3. 轴向深度误差到三维边长的一阶传播；
4. edge/node/frame 级连续归一化应变统计；
5. 不确定度无效和分母 floor 使用计数；
6. exact C++ CSV 的离线审计工具。

没有新增动态阈值、component 标签或 SLAM 状态修改。

## 4. 工程验证

```text
make geometric_warp_test rgbd_tum -j$(nproc)       PASS
geometric_warp_test                                 PASS
audit_uncertainty_normalized_rigidity.py py_compile PASS
```

确定性测试覆盖：

- 完全刚性边的归一化应变接近零；
- 单节点独立位移产生非零归一化应变；
- 基础轴向噪声按 `z^2` 缩放；
- 3x3 深度混合会提高估计不确定度；
- shadow invariant 保持不变。

## 5. 真静态短序列

| 序列 | 输入/比较帧 | F3 总耗时 median/P95 | 不确定度度量 median/P95 | q median/P90/P99 | floor / edges | invalid |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Bonn static_close_far | 28 / 27 | 2.307 / 2.381 ms | 0.585 / 0.617 ms | 0.486 / 1.395 / 3.098 | 0 / 67906 | 0 |
| TUM fr1/xyz | 29 / 28 | 2.052 / 2.282 ms | 0.527 / 0.583 ms | 0.583 / 2.118 / 15.612 | 0 / 63350 | 0 |

相对旧 F3，新增不确定度计算的观察增量约为：

```text
Bonn metric median: 0.385 -> 0.585 ms
TUM  metric median: 0.370 -> 0.527 ms
```

这说明该测量本身成本较小，但静态 TUM 的高分位仍有明显长尾；连续分数不能
直接解释为动态概率。

## 6. 冻结 development proxy 审计

审计只使用 geometry 计算前已冻结的 RGB-only coarse box；它们不是像素运动
真值，也未被用于在线分类。

| 序列 | 可比内部帧 | internal q <= background | 可比 crossing 帧 | crossing q > background | q 与 edge length Pearson |
| --- | ---: | ---: | ---: | ---: | ---: |
| balloon | 5 | 2 / 5 | 5 | 5 / 5 | 0.192 |
| balloon2 | 9 | 2 / 9 | 9 | 8 / 9 | 0.143 |
| 合计 | 14 | **4 / 14** | 14 | **13 / 14** | — |

旧的绝对边长应变在相同代理上为：

```text
internal absolute strain <= background: 11 / 14
crossing absolute strain > background:   13 / 14
```

归一化后内部一致性由 11/14 降为 4/14，未改善目标可分性；跨组边仍保持
13/14 的方向性，但单凭这一现象不足以建立动态判决。`balloon2` frame 230
仍出现框内混合深度/遮挡对应造成的极大值，说明轴向噪声模型不能替代可靠的
对应、可见性和区域边界处理。

## 7. 在线动态序列性能记录

两条 Bonn 序列均在 host RTX 4060 Ti、ONNX Runtime CUDA provider、同步
YOLO 和 exact semantic mask 下运行：

| 序列 | 帧数 | semantic median | tracking median | F3 total median | actual FPS | deadline miss |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| balloon | 289 | 12.27 ms | 14.29 ms | 1.85 ms | 28.40 | 284 / 289 |
| balloon2 | 393 | 13.03 ms | 16.68 ms | 1.79 ms | 25.99 | 392 / 393 |

这些是完整同步 pipeline 的结果，不能把低于 30 FPS 全部归因于 F3U；F3U
相对 F3 的额外开销仅约 0.16–0.20 ms median。它同时说明当前 Bonn 完整
pipeline 没有稳定的 30 FPS 余量。

## 8. 决策

1. 保留 F3U 作为已实现的连续诊断量和负结果；默认不用于动态判决。
2. 不选择 hard threshold，不在已打开 development proxy 上继续调参。
3. 不把失败误归因于 Dai 点关联原方法；被否定的是本项目的两帧标量简化。
4. 继续保持 G1-F/G1-D 锁定。
5. 下一步应回到核心证据问题：优先检验与可靠对应、可见性和区域上下文直接
   相关、且有原始文献依据的最小方案；若没有预先定义且可验证的改进假设，
   应停止增加局部 edge score。

## 9. 产物

代码与工具：

- `DT-SLAM/include/GeometricDynamicDetector.h`
- `DT-SLAM/src/GeometricDynamicDetector.cc`
- `DT-SLAM/include/Tracking.h`
- `DT-SLAM/src/Tracking.cc`
- `DT-SLAM/Examples/RGB-D/geometric_warp_test.cc`
- `DT-SLAM/tools/audit_uncertainty_normalized_rigidity.py`

本地原始证据：

- `static30*.csv`、`tum_fr1_xyz_static30*.csv`
- `balloon*.csv`、`balloon2*.csv`
- `development_audit/*.csv`、`development_audit/*.json`
- `literature/` 中的原论文 PDF、文本和 Dai TeX source
