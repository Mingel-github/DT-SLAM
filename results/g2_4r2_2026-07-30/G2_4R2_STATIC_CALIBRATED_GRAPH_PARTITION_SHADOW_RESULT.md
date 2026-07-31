# G2-4R2 静态标定图分割 Shadow 结果

日期：2026-07-30
状态：预注册实验完成；两条分支均未通过；简化 scalar-strain + CC 路线停止

## 1. 结论

本轮对上一阶段的路线理由做了必要纠正，并真正测试了缺失的图拓扑步骤：

```text
两帧 F3 三维 edge strain
→ 真静态数据分位数阈值
→ 异常边删除
→ connected components
→ 主要分量之外节点的开发代理富集
```

结果：

```text
all-transient gate   FAILED
MapPoint-only gate   FAILED
dynamic_decision     none
SLAM mutation        none
sealed holdout       not used
```

因此可以停止：

> 当前两帧 scalar absolute edge strain + 静态分位阈值 + 最大节点数
> component 的简化图路线。

这个结果不否定 Dai et al. 的完整 point-correlation 方法。当前没有实现其
三维 edge-state optimization、全协方差 Mahalanobis 检查、迭代离群剔除、
largest-volume 策略或后端滑窗复核。

## 2. 实现

新增离线只读工具：

```text
DT-SLAM/tools/audit_static_calibrated_graph_partition.py
```

实现两个并列分支：

```text
all_transient:
  所有 F3 measured LK/RGB-D 节点重新做 Delaunay

mappoint_only:
  只取 has_mappoint=1 节点，重新做 Delaunay
```

MapPoint-only 不是从全图旧边中筛边，而是在节点子集上重新三角化。

静态 `rgbd_bonn_static_close_far` 的 142 个可测帧按时间分为：

```text
calibration 71 frames
validation  71 frames
```

只用 calibration edge strain 计算：

```text
q90, q95, q97.5, q99, q99.5
```

动态 `balloon/balloon2` 的冻结 bbox 只参与最后评价，不参与阈值生成。

## 3. 实现一致性

Python 重建的 all-transient Delaunay 图与原 C++ F3 edge CSV：

| 数据 | 可比较边 | missing | extra | 最大 strain 数值差 | edge-set parity |
| --- | ---: | ---: | ---: | ---: | --- |
| static | 293698 | 0 | 0 | 4.31e-7 m | PASS |
| balloon | 23855 | 0 | 0 | 2.92e-7 m | PASS |
| balloon2 | 15289 | 0 | 0 | 4.19e-7 m | PASS |

相同命令运行两次，两个 JSON 和逐帧 CSV 均字节一致：

```text
deterministic replay = PASS
```

因此结果不是 Python/C++ Delaunay 邻接不一致造成的。

## 4. all-transient 结果

| 静态工作点 | threshold | static outside-primary P95 | 动态支持帧 | bbox recall 中位 | recall≥50% 帧 | enrichment 中位 | gate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| q90 | 0.033780 m | 2.82% | 14/17 | 9.69% | 0/14 | 2.59 | FAIL |
| q95 | 0.066785 m | 2.23% | 14/17 | 4.69% | 0/14 | 1.04 | FAIL |
| q97.5 | 0.186533 m | 1.66% | 14/17 | 0% | 0/14 | 0 | FAIL |
| q99 | 0.515779 m | 0.61% | 14/17 | 0% | 0/14 | 0 | FAIL |
| q99.5 | 0.734524 m | 0.26% | 14/17 | 0% | 0/14 | 0 | FAIL |

最敏感的 q90 已在静态 validation 满足项目的 5% P95 风险线，但仍没有一个
支持帧能隔离至少一半 bbox nodes。

代表帧：

```text
balloon frame 39:
  bbox 9 nodes，outside-primary 4，recall proxy 44.44%

balloon frame 252:
  bbox 32 nodes，outside-primary 3，recall proxy 9.38%

balloon2 frame 54:
  bbox 35 nodes，outside-primary 5，recall proxy 14.29%
```

当前 scalar strain 可以删掉少量异常边，但不能稳定切断目标代理与大背景图。

## 5. MapPoint-only 结果

17 个动态开发候选中：

```text
具有 >=3 个 bbox MapPoint nodes：1/17
该唯一支持帧 q90 bbox recall：0%
所有静态工作点 gate：FAIL
```

q90 的 static outside-primary P95 为 1.74%，静态风险较低，但这是以目标支持
几乎消失为代价，不能作为通用未知动态分支。

这次结果补充并验证了之前的输入审计：

> MapPoint 少不必然从理论上否定 Dai 图；但在当前两条 Bonn 开发序列和现有
> 前端状态中，它确实不足以形成可评价的目标组。

## 6. 为什么方向性 edge 证据没有转化为分组

F3 曾得到：

```text
crossing strain median > background：13/14
internal strain median <= background：11/14
```

这是分布层面的方向性，不等于每个 crossing edge 都能被一个统一阈值删除，
也不保证所有目标内部边都保留。Delaunay 图只要留下少量跨组桥接边，目标节点
仍会留在最大连通分量；相反，零散深度/LK 异常也会在背景内形成小分量。

本轮正好验证了：

```text
distribution enrichment
≠
deployable graph cut
```

## 7. 方法边界

本轮支持的结论：

- 此前只按目标 MapPoint 数量延期 Dai 的理由不充分，已纠正；
- 当前 all-transient scalar graph 有输入，但拓扑召回不足；
- 当前 MapPoint-only 图在目标代理内支持严重不足；
- 不应继续在已打开开发代理上调整 strain quantile；
- F1 continuous sparse ego-flow 仍是已观察到的最强局部运动证据。

本轮不支持的结论：

- Dai point correlations 无效；
- 两帧几何分组普遍无效；
- 未知动态物体无法由稀疏特征检测；
- bbox 外节点一定静态；
- 当前结果可以进入 G1-F。

## 8. 决策

按预注册停止条件：

```text
停止：
  scalar absolute strain + static quantile + CC
  all-transient 简化图阈值修补
  当前 MapPoint-only 前端分组

保留：
  F1 continuous sparse ego-flow measurement
  F3/F3U continuous edge diagnostics
  Dai 完整方法的文献身份

继续锁定：
  G1-F
  G1-D
  mvbDynamic / mvpMapPoints mutation
```

下一阶段不能再把另一个阈值接到 F3 上。需要回到总路线决策：

1. 接受更完整、成本更高的文献区域/多运动模型子集；或
2. 将 F1 定位为机会式、高精度 feature evidence，设计独立安全复核；或
3. 暂停几何判决，先补更适合未知刚体的受控数据和评价。

在选择前先形成书面决策，不直接写下一版算法。

## 9. 产物

```text
results/g2_4r2_2026-07-30/
  G2_4R2_DAI_GRAPH_PARTITION_FEASIBILITY_DECISION.md
  G2_4R2_STATIC_CALIBRATED_GRAPH_PARTITION_SHADOW_SPEC.md
  G2_4R2_STATIC_CALIBRATED_GRAPH_PARTITION_SHADOW_RESULT.md
  audit_v1/
    static_thresholds.json
    aggregate_summary.json
    per_frame_partition.csv
```

所有输出明确保持：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```
