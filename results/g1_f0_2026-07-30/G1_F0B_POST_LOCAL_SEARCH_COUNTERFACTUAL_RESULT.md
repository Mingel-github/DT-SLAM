# G1-F0B `SearchLocalPoints()` 后关联反事实结果

日期：2026-07-30
状态：预注册条件失败；真实 G1-F 继续锁定

## 1. 结论

G1-F0B 已完成默认关闭的在线关联快照、六组同次执行数据和离线双尺度审计。

正面结果：

```text
六序列 q6/q8/q10：
  post-pose baseline>=30 → remaining<30       0 帧
  relocalization strict baseline>=50 → <50    0 帧

同次 F1/snapshot：
  feature index exact match                   PASS
  semantic flag exact match                   PASS
  counted inlier == mnMatchesInliers           PASS
```

失败结果：

```text
Bonn true-static post-pose MapPoint candidate:
  q6   0.3699%
  q8   0.2680%
  q10  0.2237%

预冻结静态预算：
  <=0.20%
```

因此 q6/q8/q10 在 semantic-blind 与 combined 两种尺度下全部失败，没有可放行
的工作点。

同时，unknown development 的 post-pose 候选作用很弱：

```text
balloon   q6/q8/q10 = 0/0/0 MapPoints
balloon2  q6/q8/q10 = 6/2/2 MapPoints
```

所以不能用“没有跨支持线”包装为几何过滤有效；它主要说明候选数量较少，而
当前数据同时显示静态预算仍未通过、unknown 有效作用有限。

## 2. 实现

新增默认关闭开关与输出：

```text
Geometry.SparseFlowCounterfactualShadowEnable
DT_SLAM_GEOMETRY_ASSOCIATION_COUNTERFACTUAL
DT_SLAM_GEOMETRY_ASSOCIATION_SNAPSHOT_CSV
```

调用位置：

```text
SearchLocalPoints()
→ existing RemoveDynamicAssociations()
→ post_search_pre_pose snapshot
→ existing PoseOptimization()
→ existing mnMatchesInliers loop
→ post_existing_pose snapshot
```

C++ 只记录原始关联状态，不计算 q、不清除 MapPoint、不增加优化。离线工具：

```text
DT-SLAM/tools/audit_post_search_mappoint_counterfactual.py
```

使用同次运行的 `(frame, feature_index)` 将 snapshot 与 F1 residual 精确连接。

## 3. 并发状态口径修正

第一版在线快照记录了 `tracking_inliers` 总数，但离线通过导出时重新读取的
`isBad()/Observations()` 推导逐 feature inlier。在 fr1 第 112 帧首次发现：

```text
reconstructed proxy = 445
Tracking count       = 448
```

这只能证明“计数后重新读取状态不稳定”，不能仅凭结果断言是哪一个并发线程
改变了哪几个 MapPoint。

因此没有放宽 invariant，而是在现有 `mnMatchesInliers` 循环内同步记录：

```text
counted_tracking_inlier
```

v2 六序列均满足：

```text
sum(counted_tracking_inlier) == tracking_inliers
```

旧 `f0b_runs/` 保留为输入状态失败记录；`f0b_runs_v2/` 与
`f0b_audit_v2_exact_inlier/` 为权威结果。

## 4. Semantic-blind scale 主结果

比例为 candidate / quality-eligible nonsemantic MapPoint。

### 4.1 `post_search_pre_pose`

| 序列 | q6 | q8 | q10 | baseline 中位/最小 |
| --- | ---: | ---: | ---: | ---: |
| fr1/xyz static | 130 / 0.1907% | 108 / 0.1584% | 102 / 0.1496% | 538.5 / 409 |
| Bonn static | 469 / 0.7955% | 312 / 0.5292% | 239 / 0.4054% | 523.5 / 243 |
| walking | 282 / 0.9234% | 134 / 0.4388% | 60 / 0.1965% | 250.5 / 174 |
| sitting | 15 / 0.0398% | 7 / 0.0186% | 3 / 0.0080% | 292 / 233 |
| balloon | 28 / 0.6197% | 10 / 0.2213% | 1 / 0.0221% | 422.5 / 379 |
| balloon2 | 77 / 2.5710% | 45 / 1.5025% | 27 / 0.9015% | 404 / 346 |

局部搜索确实把更多 high-residual feature 关联到了 MapPoint；这正是原计划要求
在 `SearchLocalPoints()` 后再次检查的原因。

### 4.2 `post_existing_pose`

| 序列 | q6 | q8 | q10 | inlier 中位/最小 |
| --- | ---: | ---: | ---: | ---: |
| fr1/xyz static | 68 / 0.1336% | 61 / 0.1198% | 59 / 0.1159% | 401.5 / 294 |
| Bonn static | 167 / 0.3699% | 121 / 0.2680% | 101 / 0.2237% | 390.5 / 177 |
| walking | 37 / 0.1663% | 13 / 0.0584% | 7 / 0.0315% | 177.5 / 129 |
| sitting | 6 / 0.0199% | 4 / 0.0133% | 0 / 0% | 225 / 177 |
| balloon | 0 / 0% | 0 / 0% | 0 / 0% | 287 / 235 |
| balloon2 | 6 / 0.3352% | 2 / 0.1117% | 2 / 0.1117% | 250 / 148 |

现有 PoseOptimization 会把大量 pre-pose high-residual 关联判为 outlier，但
Bonn 静态尾部仍超过预算。不能因此增加第三次优化，也不能把 Optimizer 的
现有 outlier 机制当作几何方法贡献。

## 5. 双尺度敏感性

两个 true-static 序列没有 semantic feature，因此两种尺度结果完全相同，
Bonn 静态失败不能通过切换 scale identity 消除。

动态序列有少量差异，例如 walking post-pose：

```text
q6 semantic-blind / combined = 37 / 39
q8                           = 13 / 14
q10                          = 7 / 7
```

两种模式均无支持线 crossing，也均因静态预算失败。没有选择一种尺度作为部署
版本。

## 6. 运行完整性

| 序列 | F1/snapshot frames | online semantic | mask age | actual FPS |
| --- | ---: | --- | ---: | ---: |
| fr1/xyz static | 148 | off | — | 28.69 |
| Bonn static | 147 | off | — | 29.72 |
| walking | 148 | CUDA 149/149 | 0/0 | 28.20 |
| sitting | 148 | CUDA 149/149 | 0/0 | 28.23 |
| balloon | 12 selected / 438 tracked | CUDA 438/438 | 0/0 | 29.47 |
| balloon2 | 9 selected / 469 tracked | CUDA 469/469 | 0/0 | 28.16 |

balloon 首次误用了只含候选帧的抽取数据目录，顺序跟踪遇到缺失图像后主动
终止。随后使用本地完整 ZIP 的临时解压副本重跑，权威结果来自完整顺序输入。

## 7. 预注册判定

| 条件 | q6 | q8 | q10 |
| --- | ---: | ---: | ---: |
| 两个 true-static 域可用 | PASS | PASS | PASS |
| 两个 true-static post-pose rate 均 <=0.20% | FAIL | FAIL | FAIL |
| 六序列不跨 post-pose 30 支持线 | PASS | PASS | PASS |
| strict relocalization 不跨 50 支持线 | PASS | PASS | PASS |
| 两种 scale mode 均通过 | FAIL | FAIL | FAIL |

最终：

```text
G1-F0B post-local-search support     FAIL
q selection                          none
G1-F real filtering                  LOCKED
G1-D                                 LOCKED
dynamic_decision                     none
direct_slam_state_mutation           none
pose_reoptimization                  none
```

## 8. 验证

```text
build rgbd_tum/geometric_warp_test    PASS
geometric_warp_test                   PASS
Python self-test                      PASS
default-off no-output                 PASS
feature-index exact join              PASS, 6/6 sequences
semantic flag exact join              PASS, 6/6 sequences
post-pose inlier exact match          PASS, 6/6 sequences
deterministic summary SHA256          932a6545d7bfc402e10a9b13ee3860e35fd770c7ab3f6fe222f2b2a899bfaf0f
deterministic per-frame SHA256        d621103a99574a75ff032fb2444c623d82a1bb5c87d885cf126e0f2fd117f9ee
```

## 9. 决策

当前不得：

- 将 q 提高到 12/15 后在已打开数据上继续选择；
- 把 0.20% 放宽到刚好容纳 q10；
- 只看“不跨 30/50”就开放过滤；
- 运行真实 G1-F ATE 试验；
- 解封 balloon_tracking 调阈值。

G1-F0B 回答了一个关键工程问题：`SearchLocalPoints()` 会重新引入不少
high-residual 关联，所以未来若有可靠判决，确实需要在此处重复应用。但当前
F1 单点 high-residual 工作点尚不足以安全地成为该判决。

下一步应回到书面路线决策，而不是继续微调 F1：

```text
1. 是否补充一个受控 unknown-object 数据集，以区分“代理不足”和“方法不足”；
2. 是否进入有完整对象/运动模型依据的较重路线；
3. 或冻结几何为研究负结果，先发表/评价 semantic-only 系统。
```
