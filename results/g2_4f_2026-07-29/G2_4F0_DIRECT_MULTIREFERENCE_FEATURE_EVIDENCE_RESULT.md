# G2-4F0 Direct Multi-reference ORB Feature Evidence Shadow 结果

日期：2026-07-29
状态：完成，负门控
范围：Bonn development/review 序列；shadow-only。

## 1. 结论

G2-4F0 去掉了 G2-4E 中失败的 depth-region 聚合，直接在当前帧 ORB
feature 中心读取已有 multi-reference depth-warp vote。

在当前 development 候选上，结果仍不支持构造几何动态 feature 判决：

1. 无人物箱子帧中，箱框内 feature 的正证据通常低于框外背景；
2. 两条序列的框内 comparison coverage 都超过 93%，因此负结果不能简单归因于
   “箱框内没有几何比较”；
3. 多参考支持也超过 95%，但 unanimous-positive 中位数为 0；
4. 强正证据主要出现在 person-present 条件，而人物已由语义分支覆盖；
5. obstructing 的 target-absent 帧全局正证据高于无人物 target-visible
   帧，进一步否定了直接选阈值。

```text
dynamic_decision          = none
direct_slam_state_mutation = none
G1-F                      = locked
G1-D                      = locked
strict hold-out           = sealed and unopened
```

这不是对所有多视图深度几何的全局否定。它说明：

> 在当前两条 Bonn development 序列、当前参考选择、scale-2
> multi-reference depth-warp 和当前粗框候选条件下，positive depth evidence
> 没有在无人物未知箱子上的 ORB feature 中形成可批准的局部富集。

## 2. 方法身份与边界

| 组件 | 身份 |
| --- | --- |
| 多参考有符号 depth residual | `[A]` 受 DynaSLAM 多视图一致性启发 |
| comparison/positive/negative/consistent vote | `[S]` 当前项目证据表示 |
| 在 ORB center 读取 vote | `[S]` 诊断映射 |
| scale-2 native cell 读取与去重复统计 | `[S]` 防止 2×2 展开重复计数 |
| RGB-only coarse bbox | `[S]` unverified development proxy |
| person mask | 当前 C++ YOLO person-only 实际输出 |
| 动态阈值、feature 过滤、地图过滤 | 不存在 |

G2-4F0 不是 Ji 2021、DynaSLAM、SInDSLAM、DetectFusion 或 FlowFusion 的复现。
来源与取舍见：

```text
G2_4F_DIRECT_FEATURE_EVIDENCE_LITERATURE_AUDIT.md
G2_4F0_DIRECT_MULTIREFERENCE_FEATURE_EVIDENCE_SPEC.md
```

## 3. 实现

新增 shadow-only 能力：

- `GeometricDynamicDetector::SampleMultiReferenceEvidenceAtFeatures()`；
- 按 `Frame::mvKeys` 的 feature center 读取 native pyramid vote；
- 可选逐 feature CSV；
- 可选 frame-id 诊断过滤；
- 离线 bbox/person 分层审计；
- native-cell 映射、vote conservation 和 absent-bbox 自测。

显式环境变量：

```text
DT_SLAM_GEOMETRY_MULTIREF_FEATURE_CSV
DT_SLAM_GEOMETRY_MULTIREF_FEATURE_FRAME_IDS
```

逐 feature 输出不写入：

- `mvbDynamic`；
- `mvpMapPoints`；
- MapPoint 创建；
- Optimizer 或任何后端状态。

`current_frame_outlier_flag` 只表示 shadow 采样时 Frame 中仍存在的 outlier
标志。它不是初次 PoseOptimization 已清除关联的历史 outlier，不能替代 GJ
阶段的 pre-clear snapshot。

## 4. 在线运行

两条完整 development 序列均使用：

- Bonn 联合 rectification；
- online CUDA YOLO；
- person mask age 0；
- 共视参考选择；
- `pyramid_dense_s2`；
- 只对修正后的 depth-time review frame 输出 feature CSV；
- shadow-only。

| 指标 | nonobstructing | obstructing |
| --- | ---: | ---: |
| 输入帧 | 778 | 589 |
| person mask ready | 778/778 | 589/589 |
| mask age median/max | 0/0 | 0/0 |
| feature CSV 行 | 23,097 | 17,794 |
| actual FPS | 24.575 | 25.931 |
| deadline misses | 735/778 | 493/589 |

该 FPS 包含诊断配置和逐 feature 内存记录/结束时 CSV 输出，不是冻结系统的最终
性能。它也不代表 geometry filtering 的性能，因为 filtering 尚不存在。

输出：

```text
results/g2_4f_2026-07-29/development_feature_runs/
```

## 5. Frame 对齐与缺失证据

48 个 G2-4D review candidates 中，42 帧存在在线 multi-reference feature
evidence。缺失帧与 G2-4E 一致：

```text
nonobstructing: frame 16
obstructing:    frames 6, 25, 35, 40, 46
```

这些帧属于序列早期尚无可用多参考证据，必须解释为 `no evidence`，不能解释为
静态。

## 6. 无人物箱子 Feature Evidence

以下为三处错误粗框完成时序复核修正后的逐帧中位数；粗框和 visibility 是
development proxy，不是 pixel/motion GT。旧数值及修正影响见
`G2_4D_TO_F0_BBOX_TEMPORAL_CORRECTION_RESULT.md`。

### 6.1 moving_nonobstructing_box

`target visible + person absent`：18 帧。

| 指标 | 框内 | 框外 |
| --- | ---: | ---: |
| comparison coverage | 95.801% | 91.924% |
| positive-presence / compared features | 1.746% | 6.703% |
| positive votes / comparison votes | 0.545% | 2.697% |

| 局部性指标 | 中位数 |
| --- | ---: |
| positive-presence enrichment | 0.289× |
| positive-vote enrichment | 0.205× |
| multi-reference fraction | 95.794% |
| unanimous-positive fraction | 0.000% |
| unique comparison native cells | 180 |
| unique positive native cells | 3.5 |
| max features/native cell | 4 |
| has-MapPoint fraction | 30.284% |

只有 `3/18` 帧的 presence enrichment 大于 1，vote enrichment 为
`4/18` 大于 1；`1/18` 帧框内没有任何 positive feature。修正后的均值有所
增加，但 frame 478/537 在独立 RGB 时序代理中属于 stationary/high，不能把
其强响应解释为动态检出成功。

### 6.2 moving_obstructing_box

`target visible + person absent`：9 帧。

| 指标 | 框内 | 框外 |
| --- | ---: | ---: |
| comparison coverage | 93.413% | 仅 6 帧可比较 |
| positive-presence / compared features | 4.060% | 6.704% |
| positive votes / comparison votes | 1.224% | 2.265% |

| 局部性指标 | 中位数 |
| --- | ---: |
| positive-presence enrichment | 0.472×（6 帧） |
| positive-vote enrichment | 0.428×（6 帧） |
| multi-reference fraction | 97.053% |
| unanimous-positive fraction | 0.000% |
| unique comparison native cells | 307 |
| unique positive native cells | 10 |
| max features/native cell | 5 |
| has-MapPoint fraction | 22.439% |

只有 `1/6` 个具有框外比较域的帧 enrichment 大于 1；`3/9` 帧框内没有任何
positive feature。

frames 199、219、239 的粗框填满或几乎填满图像，框外没有 feature。对这些帧
不计算 enrichment，没有伪造框外分母。

## 7. Person 与 Target-absent 对照

### 7.1 Person-present

nonobstructing 的 person-present 样本只有 5 帧，但信号明显增强：

| 指标 | 框内 | 框外 |
| --- | ---: | ---: |
| positive-presence | 50.526% | 29.654% |
| positive vote ratio | 30.488% | 13.067% |

中位 enrichment 分别为 `1.966×` 和 `2.333×`。这说明当前证据能响应较强
人物搬运/遮挡变化，但人物本身已由 semantic branch 覆盖，不能作为未知箱子
成功证据。

obstructing 的 person-present 只有 2 帧，且框内/框外局部性没有相同趋势，
不作稳定统计结论。

### 7.2 Target absent

obstructing 的 target-absent 样本为 8 帧：

| 指标 | 全局/框外中位数 |
| --- | ---: |
| positive-presence | 9.198% |
| positive vote ratio | 5.042% |

这两项都高于无人物 target-visible 的框内中位数 `4.060%/1.224%`。当前
positive evidence 因而没有 target-visible separability，不能直接选阈值。

## 8. 当前 Outlier 字段限制

所有可用 MapPoint 的 `current_frame_outlier_flag` 均为 0。该结果符合当前
采样位置：初次优化后被清除的关联不再保留为可审计的历史 outlier。

因此本阶段不能回答：

```text
初始 PoseOptimization outlier 是否与 direct depth evidence 重合
```

若未来确有必要，应复用 GJ 已有的 pre-clear snapshot 思路单独设计审计；不得
把当前全零字段解释为“优化器没有离群点”。

## 9. 能确认与不能确认

### 已确认

- feature 到 native pyramid cell 的映射与 vote conservation 正确；
- 框内 feature comparison coverage 充足；
- 无人物箱子候选上 direct positive evidence 没有局部富集；
- person-present 是明显混杂因素；
- region aggregation 不是当前失败的唯一原因；
- 当前 evidence 不能进入 G1-F。

### 不能确认

- 每个候选帧中箱子是否正在独立运动；
- 粗框是否精确覆盖箱子；
- 当前 evidence 的 pixel/feature precision、recall 或 AUC；
- 所有运动方向和速度下的 depth-warp 能力；
- 未解封 strict hold-out 的表现；
- 稀疏 ego-flow 是否能补偿横向运动退化。

## 10. 门控决定

```text
feature comparison availability         = 通过
multi-reference availability            = 通过
no-person box-local positive enrichment = 未通过
target-visible separability             = 未通过
motion-label sufficiency                 = 未通过
dynamic-decision readiness               = 未通过
```

因此：

- 不调 positive vote/count/support 阈值；
- 不把 person-present 高响应包装成未知箱子检测；
- 不运行 strict hold-out；
- 不进入 G1-F 或 G1-D；
- 不修改 YOLO、Optimizer 或后端；
- 不新增 PoseOptimization。

## 11. 下一步

按原计划进入 failure-driven evidence 审计，而不是继续修补 depth score：

1. 先冻结 G2-4F1 的本地文献审计和最小 SPEC；
2. 候选方向为稀疏 ego-motion-compensated optical-flow residual；
3. observed flow 与 camera-induced ego flow 必须分开；
4. forward-backward LK 只作为对应有效性检查，不是动态阈值；
5. 仍保持 shadow-only，不融合、不分类、不过滤；
6. 同时设计独立于该 flow residual 的自动 motion-state 预标注，避免循环验证；
7. evidence 和 motion-label protocol 都通过 development 门后，才冻结判决并
   解封 strict hold-out。

FlowFusion 提供“observed flow 减 camera-induced ego flow”的文献原型；
NGD-SLAM/DVI-SLAM 支持 LK 作为轻量时序工具，但它们主要服务语义候选传播或
验证。当前项目若采用稀疏 LK，只能标为对上述原型的 `[A/S]` 轻量前端改造，
不能声称复现任何完整系统。

## 12. 验证

已通过：

```text
geometric_warp_test                         = PASS
audit_bonn_feature_evidence.py --self-test  = PASS
Python syntax compilation                   = PASS
git diff --check                            = PASS
strict hold-out SHA256                      = unchanged
```

原始日志、逐 feature CSV、逐帧审计 CSV 和 JSON 保存在本阶段目录。开发 archive
临时解压副本在验证后删除；strict hold-out 不解压、不查看、不运行。
