# GJ-2A Cluster–Person Proxy 自动审计结果

日期：2026-07-28

## 1. 结论

GJ-2A通过了“cluster重投影误差是否含有可测人物区域排序信号”的方向性门控，
但没有批准GJ-3动态阈值或实际过滤。

在TUM `fr3/walking_xyz`前100帧中：

- 99帧具有初始位姿和可排序cluster；
- identity-`rho`对应的cluster mean squared reprojection error与person proxy
  覆盖比例的全局Spearman相关为`0.308`；
- 99帧中87帧的逐帧Spearman为正，中位数为`0.309`；
- error Top-3覆盖`36.05%`的可测person proxy，占被测cluster面积`19.10%`；
- error Top-3相对1000次随机排列的覆盖增益为`2.14×`，经验`p≤0.001`；
- error Top-3覆盖也高于按ORB优化器离群比例排序的`20.50%`。

这说明Ji式cluster重投影误差不是简单复述已有optimizer outlier标记，并且在该
人物动态序列上存在有限、可重复的区域排序信号。

但它仍不是可靠动态判定器：

- Top-1仅63/99帧具有大于1的面积enrichment；
- fr1/xyz静态序列也出现最高约`50.04 px`的cluster mean error；
- walking与静态误差分布仍明显重叠；
- person proxy是语义区域代理，不是运动真值，也不能验证未知动态箱子；
- 全分辨率CPU K-means约`54–56 ms/frame`，不满足30 Hz主方法目标。

因此当前准确结论是：

```text
GJ-2A排序证据门控：通过
GJ-3绝对阈值与实际过滤：未批准
Ji 2021工程定位：论文比较baseline，而非实时主方法
```

## 2. 审计范围

输入：

```text
GJ-2 initial-pose cluster reprojection CSV
GJ-1无损CV_16UC1 cluster label
既有离线YOLOv8 person proxy mask
```

约定：

```text
label 0    = invalid depth
label 1..K = cluster id + 1
```

代理mask只用于离线审计，没有输入SLAM，没有修改：

```text
Frame::mvbDynamic
Frame::mvpMapPoints
Optimizer
MapPoint写入
LocalMapping
LoopClosing
YOLO推理
```

## 3. 新增的必要对照

除面积和随机排序外，增加了一个已有系统内部基线：

```text
optimizer_outlier_fraction
= optimizer_outlier_support / matched_map_support
```

目的不是构造新方法，而是检查cluster error是否只是复述ORB-SLAM2已经给出的
离群比例。

100帧全局结果：

| 指标 | cluster error | optimizer outlier fraction |
| --- | ---: | ---: |
| Spearman(proxy fraction) | 0.308 | 0.260 |
| Top-1 proxy capture | 12.79% | 4.72% |
| Top-3 proxy capture | 36.05% | 20.50% |
| Top-5 proxy capture | 53.69% | 41.74% |

在该样本上cluster error排序 consistently 高于outlier fraction排序。因此不能把
观察到的信号完全归因于optimizer outlier计数。

## 4. Top-K排序结果

聚合结果：

| 排名 | proxy capture | area fraction | area enrichment | random mean | random enrichment | 经验p |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Top-1 | 12.79% | 6.35% | 2.01× | 5.61% | 2.28× | ≤0.001 |
| Top-3 | 36.05% | 19.10% | 1.89× | 16.84% | 2.14× | ≤0.001 |
| Top-5 | 53.69% | 31.81% | 1.69× | 27.98% | 1.92× | ≤0.001 |

主表使用`mean_squared_error_px2`，因为它与当前声明的`rho(s)=s`严格对应。作为
敏感性检查，使用`mean_error_px`时Top-3 proxy capture为`33.23%`、随机增益
`1.98×`、全局Spearman为`0.320`。两种口径方向一致，但不能混称为同一公式。

`p≤0.001`表示1000次置换中达到或超过实际值的次数为0，使用加一修正后输出
`1/1001`；不能把它解释成真实概率恰好等于`0.000999`。

逐帧稳定性：

| 指标 | 结果 |
| --- | ---: |
| Spearman为正 | 87/99帧 |
| Spearman中位数 | 0.309 |
| Top-1 enrichment > 1 | 63/99帧 |
| Top-3 enrichment > 1 | 85/99帧 |
| Top-5 enrichment > 1 | 85/99帧 |
| 最大单帧占Top-3总捕获比例 | 2.41% |
| 最大单帧占Top-5总捕获比例 | 1.78% |

结果不是由单个帧主导；但Top-1仍不稳定，不能采用“误差最大cluster即动态”的规则。

## 5. walking与静态序列误差

| 指标 | fr3/walking，99个有位姿帧 | fr1/xyz，99个有位姿帧 |
| --- | ---: | ---: |
| 有效重投影支持 | 20154 | 30073 |
| cluster mean error中位数 | 3.45 px | 2.58 px |
| cluster mean error P90 | 7.46 px | 5.65 px |
| cluster mean error最大值 | 53.82 px | 50.04 px |
| 支持点加权mean error | 4.02 px | 3.26 px |
| 每帧平均unknown cluster | 6.06/24 | 4.14/24 |
| GJ-2统计平均耗时 | 0.0232 ms | 0.0276 ms |

walking整体分布更高，但静态序列也有长尾。因此：

```text
高cluster error是动态证据之一
高cluster error不等于动态真值
```

不能从当前数据直接批准一个全局绝对像素阈值。

## 6. 运行时间

100帧raw-label诊断运行：

| 序列 | tracking mean | active total mean | actual FPS |
| --- | ---: | ---: | ---: |
| fr3/walking | 71.25 ms | 80.25 ms | 12.46 |
| fr1/xyz | 74.01 ms | 82.88 ms | 12.06 |

该运行包含每帧无损label PNG写入，不能把整链路耗时全部归因于算法。但日志中的
GJ-1单帧统计仍显示K-means约`54–56 ms`，所以即使去掉调试I/O，全分辨率CPU
K-means也不具备30 Hz余量。

GJ-2重投影统计仅约`0.02–0.03 ms`，不是性能瓶颈。

## 7. 工程修正与验证

本阶段新增：

- 可逆`CV_16UC1` raw cluster label；
- `DT_SLAM_JI_DEBUG_EVERY_N`，用于控制诊断采样；
- `DT_SLAM_JI_DEBUG_RAW_LABELS_ONLY=1`，长样本只保存审计所需label；
- `tools/audit_ji_cluster_proxy.py`；
- cluster像素数与CSV `depth_pixels`逐项一致性检查；
- Top-K面积、随机置换、Spearman和optimizer outlier基线；
- 逐帧稳定性和误差分布汇总。

验证：

```text
[Ji GJ-1/GJ-2 Test] PASS
100帧walking所有label cluster像素数与CSV一致
100帧fr1/xyz所有label cluster像素数与CSV一致
fr1/xyz全零proxy不产生伪person overlap
```

## 8. 决策

批准：

- 将GJ-2A记录为通过的排序信号诊断；
- 将Ji cluster error保留为论文baseline证据；
- 后续若实现GJ-3，必须明确标注阈值、鲁棒函数和最小支持数是适配参数，不得写成
  Ji论文公开配置；
- 继续保持shadow-only，直至阈值来源和静态误检控制经过单独审批。

不批准：

- 直接用当前Top-1或固定像素误差阈值过滤；
- 把person proxy当运动GT；
- 声称已检测未知动态箱子；
- 把CPU K-means纳入30 Hz主方法；
- 修改Optimizer或新增第三次PoseOptimization。

## 9. 主要输出

```text
results/gj2a_2026-07-28/walking100_reprojection.csv
results/gj2a_2026-07-28/walking100_labels/
results/gj2a_2026-07-28/walking100_audit/
results/gj2a_2026-07-28/walking100_mean_error_audit/
results/gj2a_2026-07-28/fr1_xyz100_reprojection.csv
results/gj2a_2026-07-28/fr1_xyz100_labels/
results/gj2a_2026-07-28/fr1_xyz100_audit/
```

## 10. 下一门控

下一步不能直接写GJ-3过滤代码。应先形成一份参数来源与验证方案：

1. 再次核对Ji论文、补充材料和可用官方实现中是否存在动态阈值、`rho`和最小支持数；
2. 若原文未公开，明确区分：
   - `GJ-L`：可复现的文献部分；
   - `GJ-A`：本工程的阈值适配部分；
3. 阈值适配必须使用训练/验证分离，静态序列约束误检，不能在同一100帧上选择并
   报告结果；
4. 在没有未知动态箱子真值前，只能评价人物代理可分性，不能声称完成类别无关
   动态检测；
5. GJ最终只作为比较baseline；实时主方法仍回到G2的深度残差、鲁棒区域化与
   时序确认路线。
