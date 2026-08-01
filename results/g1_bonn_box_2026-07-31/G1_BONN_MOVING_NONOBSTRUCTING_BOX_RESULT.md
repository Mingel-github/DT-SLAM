# G1 Bonn moving_nonobstructing_box 对照结果

日期：2026-07-31
状态：完成；结果不支持“当前几何已经保护运动箱子”

## 1. 实验范围

按照运行前冻结的 SPEC，在同一 `P=K` 联合校正域、同一在线 CUDA
YOLOv8-seg 和同一 778 对 RGB-D association 上比较：

```text
semantic-only
semantic+geometry (G1-F1 + G1-M1, q10/5% unchanged)
```

正式指标全部使用 Viewer OFF，每种模式运行三次。另运行一次完整 Viewer ON
组合模式用于定性观察和精确移除点导出。没有根据结果调整阈值，没有修改
YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing。

## 2. 三轮正式指标

| 模式 | ATE RMSE 中位 | RPE RMSE 中位 | FPS 中位 | 轨迹覆盖 |
|---|---:|---:|---:|---:|
| semantic-only | 0.152247 m | 0.051377 m | 29.707 | 3/3 均 778/778 |
| semantic+geometry | 0.178114 m | 0.045061 m | 29.544 | 3/3 均 778/778 |

相对中位数：

```text
ATE   +16.99%（变差）
RPE   -12.29%（改善）
FPS    -0.55%
```

首轮 ATE 从 `0.263285 m` 降到 `0.169284 m`，但第二轮从 `0.152247 m`
升到 `0.350188 m`，第三轮从 `0.128632 m` 升到 `0.178114 m`。因此首轮
改善不能被当成稳定因果效果。RPE 三轮组合结果较集中，但不足以抵消 ATE 的
不稳定性。

完整逐轮数值见 `G1_BONN_MOVING_NONOBSTRUCTING_BOX_METRICS.csv`。

## 3. 过滤路径确实执行且安全限制有效

三次组合运行的 tracking 审计：

```text
rows                         2331
applied frames                582
removed associations         1519
baseline associations     1286114
invariant violations            0
```

三次 mapping 审计：

```text
rows                           143
applied rows                    43
new dynamic flags              294
vetoed valid-depth features    284
fail-open rows                  15
invariant violations             0
```

这证明 G1-F1/G1-M1 的实现会实际改变稀疏前端，并且既定 fail-open 与 5%
限制正常工作；它不证明被移除观测来自运动箱子。

## 4. Viewer 与精确空间审计

普通 ORB-SLAM2 Viewer 不显示“哪一个 MapPoint association 被几何过滤”。为
避免从聚合计数猜测位置，新增默认关闭的只读诊断：

```text
DT_SLAM_GEOMETRY_TRACKING_FILTER_FEATURE_CSV
```

它只在 `ApplySparseFlowTrackingFilter()` 真正将 association 置空前记录
frame、feature index、像素坐标和 MapPoint id，不改变候选或安全条件。

Viewer ON 运行结果：

```text
processed frames              778/778
semantic masks ready          778/778
mask age median/max           0/0
actual FPS                    29.370
exact removed rows               612
per-frame count mismatches         0
duplicate frame/feature keys       0
semantic-dynamic overlap rows      0
```

运行完成并写出轨迹后复现既有 Viewer/Pangolin 关闭段错误 `-11`。正式
Viewer OFF 六次运行均正常，因此没有把这个关闭阶段故障归因于几何过滤。

空间检查复用了此前 24 个**粗略且未验证**的箱框候选和同帧 C++ 人物 mask：

```text
review frames                         24
frames containing exact removals       7
reviewed exact removed points          13
inside coarse box bbox                  0
inside person mask                      0
outside both                           13
```

联系表显示这些抽查点位于椅子、柜门、墙面或其他背景位置，而不是粗箱框内。
由于箱框不是 GT、只抽查 24 帧，这不能计算正式 precision/recall；但它足以否定
“这次运行中已观察到几何主要过滤运动箱子”的积极解释。

审计输出：

- `moving_nonobstructing_box/removed_association_review/contact_sheet.png`
- `moving_nonobstructing_box/removed_association_review/summary.json`
- `moving_nonobstructing_box/removed_association_review/review_counts.csv`

## 5. 客观结论

```text
完整轨迹与同帧语义                  PASS
几何过滤路径实际执行                PASS
安全限制与 CSV 不变量               PASS
运行成本                            小（FPS 中位 -0.55%）
稳定 ATE 改善                       NOT SUPPORTED
RPE 中位改善                        OBSERVED, not sufficient alone
运动箱子空间命中                    NOT OBSERVED in sampled proxy frames
未知对象检测有效性                  NOT PROVEN
```

当前稀疏几何仍可作为默认关闭的实验模式，但不能称为已经验证的未知箱子保护模块。
本结果也说明：仅凭 ATE/RPE 的偶然变化或“过滤数量非零”，不能判断几何过滤的
对象是否正确。

## 6. 下一步决策

不调 q10/5%，不重启 flood fill，不开放 G1-D。合理的下一步是：

1. 在已经下载的 `moving_obstructing_box` 上做同样的**一次开发诊断**；该序列
   箱子造成的遮挡和观测变化更强，可检查当前稀疏 ego-flow 是否只是在
   `nonobstructing` 情形下缺少可观测性。
2. 仍先导出精确移除点并做空间检查；只有看到箱子附近的稳定命中，才值得做
   三轮 ATE/RPE。
3. 如果第二个箱子序列仍主要命中背景，则冻结当前 q10 稀疏过滤为负面/有限
   baseline，回到有文献依据的运动分组或更强对象候选设计，而不是继续运行更多
   Bonn 序列追逐偶然指标。

这是对当前路线的判别性扩展，不是新增长期阶段。
