# G2-6O Bonn 箱子运动可观测性审计结果

日期：2026-07-31
状态：完成；新增三序列未通过独立 unknown-box 评价资格
身份：`[S]` development RGB 时序粗审，不是运动真值

## 1. 结论

新增的三条 Bonn 箱子序列没有补齐当前最关键的评价缺口：

```text
moving + person absent + confidence >= medium = 0 个候选
```

具体表现：

- `placing_nonobstructing_box`：箱子放置过程清楚，但所有明确运动/转换候选均有
  搬运者；
- `removing_nonobstructing_box`：箱子移除过程清楚，但所有明确运动/转换候选均有
  搬运者；
- `kidnapping_box`：候选中没有人物，但相机运动、多个箱体和视野边界切换使
  RGB-only 审阅无法可靠分离独立箱子运动；12/18 候选保持 `uncertain`。

因此本阶段：

```text
G2-6O independent unknown-box qualification = FAIL
G2-6F new evidence audit                    = NOT RUN
G1-F / G1-D                                 = LOCKED
SLAM mutation                               = NONE
```

该结果不说明三条序列没有动态，也不说明 sparse ego-flow 或 motion grouping
无效。它只说明当前无 GT 的 RGB 时序粗审不足以提供所需的、低混杂的
`moving + person absent` development reference。

## 2. 数据与 association

官方 Bonn 页面说明该数据集包含人员操作箱子等动态任务，并提供传感器 GT pose；
它没有为每帧提供对象运动标签或对象 mask。

三条 archive 已在下载时通过完整性检查。新增的 ZIP 直读 association 支持生成：

| sequence | pairs | missing depth references | max RGB-depth delta |
|---|---:|---:|---:|
| placing nonobstructing box | 720 | 3 | 16.680 ms |
| removing nonobstructing box | 494 | 2 | 16.650 ms |
| kidnapping box | 1091 | 9 | 16.680 ms |

工具只过滤 archive 中实际不存在的文件，不改变时间戳或重新使用 depth 帧。

## 3. 候选生成

新增只读工具：

```text
DT-SLAM/tools/prepare_bonn_box_motion_observability_review.py
```

每条序列固定生成：

```text
uniform          8
RGB change high  6
RGB change low   4
total           18
```

RGB temporal change 只用于提高审阅覆盖率。工具接口不接收 geometry、flow、
depth residual、SLAM score 或 ATE。

总计：

```text
3 sequences
54 unique candidates
sealed balloon_tracking accessed = false
selection is motion GT            = false
```

## 4. 审阅跨度修正

第一版联系表使用：

```text
offsets = -6 -4 -2 0 +2 +4 +6
```

约 0.4 秒跨度足以看见人员搬运，但不足以在 `kidnapping_box` 中区分相机视差与
对象运动。没有重新选择候选或调整 RGB proxy；第二版严格复用相同 54 个
候选和相同 proxy 数值，只扩大显示跨度：

```text
offsets = -30 -20 -10 0 +10 +20 +30
```

边界帧使用合法索引夹取，只影响显示。summary 中记录：

```text
candidate_selection_reused = true
```

这是评价显示修正，不是 detector 调参。

## 5. Agent RGB 时序粗审

标签定义：

```text
box_visibility = visible / partial / absent / uncertain
box_motion = moving / stationary / transition / uncertain / not_visible
person_presence = present / absent / uncertain
confidence = high / medium / low
```

所有标签均声明：

```text
label_source = agent_rgb_temporal_review_v1
is_ground_truth = false
geometry_flow_depth_score_used = false
```

### 5.1 Placing

| label | count |
|---|---:|
| moving + person present + high | 2 |
| transition + person present + high | 2 |
| stationary + person absent + high | 8 |
| uncertain | 1 |
| moving + person absent + medium/high | **0** |

### 5.2 Removing

| label | count |
|---|---:|
| moving + person present + high | 4 |
| transition + person present | 2 |
| stationary + person absent + high | 2 |
| uncertain | 1 |
| moving + person absent + medium/high | **0** |

### 5.3 Kidnapping

| label | count |
|---|---:|
| stationary + person absent + high | 3 |
| stationary + person absent + medium | 3 |
| uncertain + person absent + low | 12 |
| moving + person absent + medium/high | **0** |

`kidnapping_box` 的 `uncertain` 不能根据 archive 名称强制改成 moving。

## 6. 与既有 Bonn 审计的关系

已有 `moving_nonobstructing_box` 与 `moving_obstructing_box` 在 G2-4F1 的
48 个独立 RGB temporal 候选中只有：

```text
moving + person absent  = 1
moving + person present = 6
stationary + person absent = 30
```

所以本次扩展没有改变此前判断：Bonn 当前已审 development 数据的明确箱子运动
大多与人员操作共现，不能独立证明类别无关几何分支对 unknown object 的判断
精度。

## 7. 决策

按冻结 SPEC，不能进入 G2-6F，也不能据此重启 q threshold、region threshold
或真实 MapPoint 删除。

下一步应回到 G1-F0 路线决策中预先写明的分叉：

1. 若可以获得 RGB-D 传感器，采集短的受控箱子序列：
   - 相机静止、箱子静止；
   - 相机运动、箱子静止；
   - 相机静止、箱子独立运动；
   - 相机运动、箱子独立运动；
   - 操作者尽量不进入视野；
2. 若当前不能采集，只允许开展文献支持的 motion grouping
   **development-only shadow feasibility**，不得声称 unknown-object
   precision/recall，也不得解锁 G1。

继续下载更多名称含 `box` 的序列不是默认下一步；除非其提供独立对象运动 GT
或明确的无人运动条件，否则只会重复当前混杂。

## 8. 修改范围与验证

新增：

```text
DT-SLAM/tools/prepare_bonn_box_motion_observability_review.py
results/g2_6_box_data_2026-07-31/
```

修改：

```text
DT-SLAM/tools/make_rgbd_association.py
```

只增加 `--dataset-zip` 的 archive 直读与缺失成员过滤。

验证：

```text
Python syntax checks       PASS
selection tool self-test   PASS
CSV rows                   54
CSV field-count errors     0
candidate reuse            exact
```

本阶段没有修改或运行：

- YOLO；
- `Optimizer.cc`、g2o；
- Tracking/Mapping 状态；
- `mvbDynamic`、`mvpMapPoints`；
- 第三次 PoseOptimization；
- ATE/RPE。
