# 可靠运动组判决：独立支持验证结果

日期：2026-08-01
状态：固定总计划“可靠运动组判决”模块内的负结果；shadow-only

## 1. 本轮问题

上一版以 7 个局部 RGB-D 对应拟合一个刚体 SE(3)，并在同 7 点上评价拟合。
TUM 真静态场景也得到明显的局部模型优势，说明训练内误差包含严重过拟合。

本轮按照冻结 SPEC，将数据严格拆为：

```text
训练集：anchor + 最近 6 点
局部独立验证：接下来 7 个未参与拟合的点
全局审计：除训练 7 点外的全部 eligible 点
```

在完全相同的验证点上成对比较局部刚体模型和背景相机模型。本轮不选择动态阈值，
不扩展/重估支持集，不聚类，不修改任何 SLAM 状态。

## 2. 文献边界

- `[L]` Lee et al. 2019 使用 7 个邻近 grid scene-flow 点产生 seed
  hypothesis，然后在全部场景流上寻找支持点、用增加后的 N 点重估，最后才聚类。
- `[S]` “最近 7 点训练＋下一批 7 点 holdout”是本项目为识别训练内过拟合设计的
  确定性交叉验证，不是 Lee 原算法，也不宣称为新动态检测方法。
- Lee 的 `t_inlier=3e-5` 是其数据上的经验参数，本轮没有迁移。

本地依据：

- `results/g2_motion_grouping_next_2026-08-01/literature/Lee_2019_Rigid_Motion_RGBD_VO_arXiv1907.08388.pdf`

## 3. 工程验证

- `geometric_warp_test`：PASS；覆盖静态刚体、两个刚体、跨运动边界、支持不足和
  确定性。
- `geometric_warp_test`、`rgbd_tum`：完整构建成功。
- TUM/Bonn CSV invariant violation：均为 0。
- 每条记录保持：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```

新增输出明确区分 seed hypothesis 状态与 independent validation 状态；无额外点时
为 `insufficient_validation_support`，不解释成静态。

## 4. TUM fr1/xyz 真静态结果

29 个输入、28 个有参考帧的测量帧，共 21,326 个有效 seed hypothesis；其独立
验证均可计算。

| 指标 | 训练 7 点中位数 | 独立 holdout 中位数 |
|---|---:|---:|
| background error - local error | +0.001951 m | **-0.001892 m** |
| background/local RMS ratio | 1.954 | **0.772** |

全局验证结果：

- local-better fraction 中位数：`0.0277`；
- background error - local error 中位数：`-0.06044 m`。

因此，先前静态场景中看似良好的局部模型优势没有泛化到未参与拟合的数据。独立
验证成功识别出训练内过拟合。

## 5. Bonn moving_nonobstructing_box 结果

本轮运行 778 帧；仅保存预先冻结的 24 个 review frame 的逐 hypothesis 数据。
由于本次没有在线运行 YOLO，语义来源记为 `semantic_mode=none`，人物运动是明确
混杂因素。5 个 moving / 19 个 stationary 标签和箱框都只是 RGB review proxy，
不是运动真值或像素 GT。

全部有效 hypothesis 的 holdout 中位数为：

- improvement：`-0.00960 m`；
- background/local RMS ratio：`0.608`；
- local-better fraction：`1/7 = 0.143`；
- global local-better fraction：`0.0153`。

也就是说，局部模型整体仍明显不如背景相机模型。

箱框内的描述性 proxy 比较为：

| 指标 | moving 中位数 | stationary 中位数 | raw proxy AUC | 箱内减箱外 AUC |
|---|---:|---:|---:|---:|
| holdout improvement (m) | -0.01194 | -0.00513 | 0.337 | 0.411 |
| background/local RMS ratio | 0.881 | 0.585 | 0.895 | 0.832 |
| local-better fraction | 0.286 | 0.143 | 0.674 | 0.595 |
| global local-better fraction | 0.0296 | 0.0150 | 0.821 | 0.716 |

RMS ratio和 local-better fraction 对 moving proxy 有相对排序能力，但不能把这个
结果解释为共同运动模型成立：

1. moving 五帧箱内的 RMS ratio 中位数全部小于 1，局部模型仍劣于背景模型；
2. moving 箱内通常只有 1--3/7 个 holdout 点支持局部模型，不构成多数共同支持；
3. improvement 为负，且其 raw/delta AUC 方向不支持局部模型；
4. 评价只有 5 个 moving proxy 帧，并含人物混杂。

因此这里观察到的是“运动时局部模型相对没那么差”的弱排序，不是一个可重估的
独立刚体支持集。

## 6. 计算成本

| 数据 | neighbor search 中位数 | 7 点 fit 中位数 | independent support 中位数 | hypothesis 总中位数 |
|---|---:|---:|---:|---:|
| TUM fr1/xyz | 18.90 ms | 2.88 ms | 19.80 ms | 41.87 ms |
| Bonn nonobstructing | 25.62 ms | 3.38 ms | 26.55 ms | 55.92 ms |

Bonn 完整 shadow pipeline 为 `12.79 FPS`。这不是最终过滤性能，但说明对每个 anchor
扫描全部点的支持审计成本较高；当前信息增益不足以支持继续优化其实现。

## 7. 决策

按照实现前冻结的停止规则：

```text
7 点训练内模型                 会过拟合
独立验证是否去除静态假优势       是
运动箱子是否形成多数独立支持集    否
是否允许 support re-estimation   否
是否允许 DBSCAN / 时序确认       否
是否开放新的 SLAM 过滤           否
```

因此停止当前稀疏 7 点刚体 hypothesis 路线。保留代码和数据为默认关闭、可复现的
shadow 负实验；不再增加邻域半径、q、面积、margin 或聚类补丁。

该结论只否定当前“稀疏最近邻 7 点 seed＋独立稀疏支持”的适配，不否定 Lee 原始
grid scene-flow 完整方法，也不否定一般的多运动分割研究方向。

## 8. 对总计划的影响

总计划没有增加阶段。当前位置仍是四个剩余模块中的第 1 个：可靠运动组判决。
本轮排除了该模块的一条候选实现，但尚未得到可放行的未知动态对象判决。因此：

```text
模块 1 可靠运动组判决       未完成；当前稀疏 7 点路线停止
模块 2 G1-F/G1-M 稀疏接入   仅保留旧实验路径；新判决未放行
模块 3 G1-D 深度区域过滤     未开始/锁定
模块 4 最终评价与冻结        未开始
```

下一方法决策不能继续在本路线打补丁。应在已有文献候选中作一次明确取舍：接受当前
保守 F1 只作为稀疏前端实验结果，或转向具有区域上下文的重聚类/稠密运动方法。该
取舍属于模块 1 的路线选择，不应包装成新的总阶段。

## 9. 产物

源码与工具：

- `DT-SLAM/include/GeometricDynamicDetector.h`
- `DT-SLAM/src/GeometricDynamicDetector.cc`
- `DT-SLAM/src/Tracking.cc`
- `DT-SLAM/Examples/RGB-D/geometric_warp_test.cc`
- `DT-SLAM/tools/audit_sparse_rigid_hypotheses.py`
- `DT-SLAM/tools/audit_sparse_rigid_hypothesis_proxy.py`

原始证据：

- `results/motion_group_support_validation_2026-08-01/fr1_xyz_30_*`
- `results/motion_group_support_validation_2026-08-01/nonobstructing_24_*`
- `results/motion_group_support_validation_2026-08-01/nonobstructing_geometry_only.log`
