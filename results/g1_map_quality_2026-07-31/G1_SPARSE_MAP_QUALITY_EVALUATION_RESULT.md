# G1 稀疏地图质量评价结果

日期：2026-07-31
状态：第一轮完成；评价工具默认关闭
性质：MapPoint 生命周期只读审计，不改变 SLAM

## 1. 评价问题

本阶段不把“删掉更多地图点”当作质量更好，而是同时检查三件事：

1. **可疑污染代理**：q10 稀疏 ego-flow 候选所创建的 MapPoint，沿
   `MapPoint::GetReplaced()` 替换链追踪后，最终是否仍留在地图中；
2. **静态信息保留**：最终 MapPoint、KeyFrame、observation 数以及静态
   `fr1/xyz` 是否明显退化；
3. **系统可用性**：ATE、RPE、完整轨迹覆盖和实际 FPS。

核心对照只改变 MapPoint admission：

```text
G1-F1 q10 + G1-M0 counterfactual（候选照常写图）
versus
G1-F1 q10 + G1-M1 filter（通过安全条件的候选禁止写图）
```

候选生命周期只是 `suspicious/dynamic-enriched MapPoint proxy`，不是逐点动态
真值。尤其在 `fr1/xyz` 中出现候选，证明不能把所有 q10 候选称为真实动态点。

## 2. 新增的只读评价

新增默认关闭配置：

```yaml
Geometry.SparseFlowMapQualityAuditEnable: 0
```

新增环境变量：

```text
DT_SLAM_GEOMETRY_MAP_QUALITY_AUDIT
DT_SLAM_GEOMETRY_MAP_QUALITY_PREFIX
```

只允许在 `RGB-D + G1-F1 q10 + (G1-M0 xor G1-M1)` 下启用。系统关闭时，
LocalMapping 和 LoopClosing 已停止，评价器只读取最终 Map、MapPoint replacement
和 observation 状态，输出：

```text
<prefix>_candidate_lifecycle.csv
<prefix>_summary.csv
```

审计工具：

```text
DT-SLAM/tools/audit_sparse_map_quality.py
```

所有输出均声明：

```text
read_only=true
direct_map_mutation=none
```

## 3. walking 三轮结果

由于 ORB-SLAM2 的线程调度和 KeyFrame 生成存在运行波动，walking 使用三轮中位
数，不依据单次结果下结论。

| 指标 | G1-M0 中位数 | G1-M1 中位数 | 观察 |
|---|---:|---:|---|
| ATE RMSE | 0.019059 m | 0.016745 m | M1 低约 12.1%，但不能仅凭三轮归因 |
| RPE RMSE | 0.012335 | 0.012486 | M1 高约 1.2% |
| actual FPS | 27.340 | 27.301 | 基本相同 |
| 轨迹覆盖 | 827/827 | 827/827 | 完整 |
| final MapPoint | 1208 | 1367 | 未出现地图点数量塌缩 |
| final KeyFrame | 98 | 96 | 接近 |
| final observations | 14231 | 16520 | 未出现 observation 塌缩 |
| candidate created | 711 | 174 | M1 只剩 fail-open 候选会创建 |
| candidate final survivors | 25 | 5 | 绝对幸存数下降 |
| candidate survival ratio | 3.59% | 3.42% | 两组候选总体不同，不能作精确因果差值 |

三轮 G1-M1 中位数：

```text
applied KeyFrame rows       92
vetoed valid-depth         662
fail-open candidate-created 174
candidate final survivors    5
```

G1-M0 中约 96%--98% 的可疑候选最终被 ORB-SLAM2 自然剔除，但仍有 14、26、25
个候选通过直接或 replacement 链留在最终地图。G1-M1 能阻止候选临时写入，且
M1 三轮绝对幸存数为 5、19、5；然而两组运行的 KeyFrame 和 MapPoint 集合并不
相同，因此不能声称它准确清除了 `25-5` 个真实动态点。

## 4. 其他序列成对检查

| 序列 | 模式 | ATE | RPE | FPS | final MP/KF | candidate created/survived | applied/veto |
|---|---|---:|---:|---:|---:|---:|---:|
| fr1/xyz | M0 | 0.009563 | 0.005758 | 29.628 | 2474/43 | 69/5 | 0/0 |
| fr1/xyz | M1 | 0.009583 | 0.005791 | 29.621 | 2600/43 | 5/1 | 22/92 |
| Bonn balloon | M0 | 0.032725 | 0.040989 | 29.263 | 2209/46 | 181/5 | 0/0 |
| Bonn balloon | M1 | 0.033792 | 0.041605 | 29.295 | 2116/45 | 0/0 | 25/226 |
| sitting_static | M0 | 0.007184 | 0.005785 | 27.534 | 623/14 | 1/0 | 0/0 |
| sitting_static | M1 | 0.005750 | 0.004902 | 27.735 | 691/15 | 2/0 | 0/0 |

解释：

- `fr1/xyz`：M1 虽否决 92 个静态场景中的候选，但 final MapPoint/observation
  没有减少，ATE 约 +0.2%，RPE 约 +0.6%，未见明显静态退化；这同时证明 q10
  不是动态 GT。
- `Bonn balloon`：M0 的 181 个候选中最终只有 5 个 proxy 存活；M1 的 226 个
  applied 有效深度候选均未创建 MapPoint，最终 proxy 为 0。单次 ATE 约 +3.3%、
  RPE 约 +1.5%、MapPoint 约 -4.2%，是轻微代价而非定位改善证据。
- `sitting_static`：M1 `applied_rows=0`，两次轨迹差异完全不能归因于过滤，
  只能作为运行波动样例。

## 5. 当前质量判断

```text
lifecycle implementation/invariants       PASS
read-only map audit                        PASS
trajectory coverage                       PASS
static catastrophic degradation           NOT OBSERVED
temporary MapPoint admission protection    CONFIRMED
exact dynamic-map precision/recall         NOT MEASURED
ATE improvement claim                      NOT SUPPORTED
cleaner-map causal claim                   SUGGESTIVE, NOT PROVEN
G1-F1 + G1-M1 experimental use             ACCEPTABLE, default OFF
G1-D dense depth filtering                 STILL LOCKED
```

当前最可靠的结论是：G1-M1 确实能阻止通过安全条件的 q10 候选进入新
KeyFrame/MapPoint，且四类序列未出现轨迹覆盖丢失或明显静态地图塌缩。另一方面，
ORB-SLAM2 本身会自然剔除绝大多数可疑 MapPoint，当前 proxy 又不是真值，因此
不能把 M1 包装成已证明“显著提高地图精度”。

## 6. 验证与边界

```text
make geometric_warp_test rgbd_tum -j$(nproc)  PASS
geometric_warp_test                           PASS
audit_sparse_map_quality.py                   PASS, all runs
audit_sparse_flow_mapping_filter.py           PASS
git diff --check                              PASS
Optimizer/g2o/YOLO/LocalMapping modification  NONE in this stage
extra PoseOptimization                        NONE
```

原始逐次指标：

```text
results/g1_map_quality_2026-07-31/G1_MAP_QUALITY_FORMAL_METRICS.csv
results/g1_map_quality_2026-07-31/formal_*/
```

下一步不继续调 q10 或 5% 条件追求某个序列的 ATE。应冻结当前稀疏版本，用 Viewer
做可解释性检查，并在论文/报告中把地图质量结论限定为 proxy；若要获得真正的
动态深度地图质量，仍需另行解决 G1-D 的可信像素区域问题。
