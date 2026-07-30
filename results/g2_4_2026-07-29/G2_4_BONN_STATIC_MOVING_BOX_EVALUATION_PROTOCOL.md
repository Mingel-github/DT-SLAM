# G2-4 Bonn Static/Moving-Box 低人工负担评价协议

日期：2026-07-29
状态：数据协议冻结候选；坐标域与自动候选选帧已完成

## 1. 数据事实

`[L]` Bonn 官方页面提供：

- 24 个动态序列和 2 个静态序列；
- moving/placing/removing/kidnapping box 等场景；
- OptiTrack camera-pose GT；
- static-environment ground-truth 3D point cloud；
- 已注册到 RGB 的 depth；
- 非零 RGB distortion。

官方页面没有声明逐帧 dynamic-object mask。

本地已有只读 zip：

```text
BONN/rgbd_bonn_moving_nonobstructing_box.zip
BONN/rgbd_bonn_moving_obstructing_box.zip
```

另有 person/crowd/balloon archive，但本轮不扩展范围。本地尚未发现 Bonn static
archive。

## 2. 坐标域先决条件

官方 calibration：

```text
fx = 542.822841
fy = 542.576870
cx = 315.593520
cy = 237.756098
d0 = 0.039903
d1 = -0.099343
d2 = -0.000730
d3 = -0.000144
d4 = 0
```

当前 geometry warp 使用 pinhole K 和 raw registered-depth pixel index；Bonn
具有非零畸变，因此不能直接沿用 TUM3 零畸变假设。

在运行 evidence 前必须二选一并验证：

1. RGB、registered depth、semantic mask 和 ORB 全部进入同一个 undistorted
   pinhole domain；
2. geometry projection 显式实现与 RGB/depth 相同的 distortion domain。

不得只 undistort ORB 而让 depth/mask 保持 raw，也不得把 `mvKeys` 与
`mvKeysUn` 混用。先用棋盘/合成点和真实边缘可视化验证，再运行 Bonn。

坐标域只读审计已经冻结选择：

```text
raw RGB   --linear remap--> rectified RGB --> YOLO + ORB
raw depth --nearest remap-> rectified depth -> tracking + geometry
output P=official K
tracking distortion=0
geometry K=tracking K
```

该组合是 `[A/S/H]`，不是任何论文整体方法的复现。实现和验收细节见：

```text
G2_4B_BONN_COORDINATE_DOMAIN_AUDIT_AND_SPEC.md
```

本地 archive 还存在索引完整性问题：

```text
moving_nonobstructing_box: depth.txt 引用 4 个不存在的 PNG
moving_obstructing_box:    depth.txt 引用 3 个不存在的 PNG
```

association 必须过滤不存在的文件，并保存排除清单。

## 3. 序列角色

2026-07-29 审阅后的冻结角色：

```text
development/review A:
  moving_nonobstructing_box

development/review B:
  moving_obstructing_box

static risk:
  Bonn static（获得后再拆 calibration/hold-out）

strict unknown-dynamic hold-out:
  rgbd_bonn_balloon_tracking.zip
  已封存，尚未解压/查看/运行
```

两条当前 moving-box 序列都已经按 geometry proxy 排序并查看联系表，其中
`moving_obstructing_box` 还参与了 diversity 规则修正，因此不能再称严格
hold-out。它们仍可用于开发期预标注、失败分析和 challenge review。

若 YOLO 实际给 box 产生稳定实例 mask，则该序列不能自动称为“semantic unknown”。
必须逐帧审计 box 是否落入 semantic mask；只有未被 semantic 模块覆盖的 box
证据才能用于 unknown-category 几何结论。

## 4. 自动选帧

Agent 自动从每个 development/review moving-box 序列选择最多 24 帧：

- 6 帧：`proxy_low_inconsistency`；
- 6 帧：`proxy_high_inconsistency`；
- 6 帧：`proxy_transition`；
- 6 帧：`proxy_geometry_difficult`。

选择依据先使用 RGB/depth temporal change、GT camera motion compensation 和
geometry-risk 统计；不依据未来判决阈值。

这些层不是 box static/moving/transition/occlusion GT。因为抽样受 geometry
proxy 条件化，24 帧子集不能用于估计完整序列的无偏 precision/recall/FPR。

static 序列最多选择 24 帧，按 camera angular/translation speed 和
boundary/invalid risk 分层。

G2-4C 已明确将四层命名为 `proxy_*`，不把自动排序输出写成真实 box motion
state。第一版因重复视图过多失败并保留；第二版加入固定的 temporal/appearance
diversity 后，两条 moving-box 序列各输出 24 个唯一候选。详见：

```text
results/g2_4c_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_RESULT.md
```

## 5. 预标注与用户负担

G2-4D 已由 Agent 完成：

- 从现有 C++ 路径导出实际使用的 person union filter mask 和 person
  detection；
- 仅查看 rectified RGB 和 source frame id，生成 target-box 粗 bbox；
- 生成 bbox 与实际 person filter mask 的逐帧覆盖表和联系表；
- 对遮挡、部分出界和 target absent 使用独立状态标记。

当前粗框为混合版本：

```text
45 rows: annotation_source = agent_rgb_only_coarse_bbox_v1
         review_status      = unverified
 3 rows: annotation_source = agent_rgb_only_coarse_bbox_v2_temporal_correction
         review_status      = agent_corrected
is_ground_truth    = false
```

三处修正来自独立五帧 RGB 时序复核；旧审计结果保留，当前有效覆盖审计为
`semantic_box_coverage_review_v3_bbox_temporal_correction`。

本轮没有要求用户逐帧标注，也没有把粗 bbox 自动转成 ORB feature GT。

若后续确实需要 pixel-level box mask，Agent 应先引入与 geometry evidence
独立的单帧提示式分割或 RGB-D 标注工具，再生成可复核预标注。相邻帧传播和
temporal consistency 只能在连续帧上使用，不能在当前 geometry-conditioned
的离散 24 帧之间直接传播。

独立性约束：

- 当前 YOLO 只筛选 COCO class 0 person，不能产生 box mask；
- C++ semantic mask 是实例合并并膨胀后的实际过滤 mask；
- box bbox/mask 不得从 positive residual、region score 或 G2-4C 排名直接
  生成；
- 当前只能报告“独立 RGB-only 粗 bbox”与“实际 C++ person filter mask”的
  development coverage proxy；
- 只有获得独立 pixel mask 后，才能计算对象像素级 semantic coverage；
- geometry-conditioned 24 帧只用于开发期审查，不作为完整序列精度样本。

## 6. 评价单位

G1-F 之前优先评价 ORB feature：

- dynamic-box feature precision/recall；
- static-background feature risk；
- feature coverage；
- boundary/invalid risk 分层；
- semantic-covered 与 semantic-uncovered box 分开报告。

获得独立 pixel mask 后，pixel 指标才可作为补充：

- precision/recall/IoU；
- boundary tolerance；
- 遮挡与 transition 帧单独报告。

camera trajectory GT 只用于 pose error 和 camera-motion compensation，不能当
object-motion GT。

## 7. 数据泄漏控制

- 两条当前 moving-box 序列均为 development/review，可用于设计但不得冒充
  最终泛化结果；
- strict hold-out 只在 score、threshold、motion-label protocol 和抽样规则
  冻结后运行；
- strict hold-out 失败后不得回调参数并重报同一序列；
- static 与 dynamic 预标注工具可以相同，但人工修正记录必须版本化；
- TUM exploratory proxy 指标不进入最终 unknown-box 主表。

## 8. 当前停止点

本协议完成不代表数据门通过。当前仍缺：

```text
Bonn static archive
independent pixel-level box mask（仅在G1-D需要时）
frozen motion-label protocol
```

strict unknown-dynamic hold-out 已封存：

```text
archive = BONN/rgbd_bonn_balloon_tracking.zip
size    = 325919363 bytes
sha256  = 3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
status  = unopened
```

已完成但不等同于 GT：

```text
exact C++ person filter export
RGB-only target-box coarse bbox
development semantic bbox-coverage review
full continuous development geometry evidence
exact online/offline depth-partition join
```

G2-4E 已确认当前固定 depth-region 聚合没有通过 unknown-box 判决门：

- nonobstructing bbox 主导 region 大量延伸至背景；
- bbox-region 正证据与整帧全局证据近乎相同；
- target-absent 对照没有显示更低的全局正证据；
- person-present 是明显混杂因素。

G2-4F0 进一步绕过 region aggregation，直接在 ORB feature 中心审计已有
multi-reference vote。三处粗框修正后，无人物箱子候选的框内 comparison
coverage 中位数为 `95.801%/93.413%`，positive-presence enrichment 为
`0.289x/0.472x`，因此 direct feature evidence 仍没有通过局部富集门。

这两项结果均受 development candidate、unverified coarse bbox 和缺少
motion-state label 限制，不能报告 precision/recall，也不能外推成“所有
depth-warp 几何无效”。

稀疏 ego-flow 的本地文献审计、SPEC、shadow 实现和同步语义全序列审计均已
完成；独立 RGB-only 五帧 motion proxy 在原 48 帧中得到
stationary/moving/uncertain/not_visible=`30/7/2/9`。F1 本身中位成本约
`2.5 ms`，但可靠 `moving+person-absent` 仅有 1 帧。

随后独立于 geometry/flow 的全序列 RGB/semantic 候选扩充又审查 50 个时间窗，
得到 stationary/moving/uncertain=`48/0/2`。这说明当前两条 moving-box
development 序列不能评价关键科学门，不能继续拟合 residual 阈值。

下一步已冻结为 G2-4F1D 非 holdout 开发数据可观测性审计：

```text
development screening:
  rgbd_bonn_balloon
  rgbd_bonn_balloon2

strict holdout:
  rgbd_bonn_balloon_tracking（继续封存）
```

G2-4F1D 只依据 rectified RGB temporal clip 和同步 C++ person mask 选择并
审阅候选，不读取任何 depth/flow/region score。详细协议：

```text
results/g2_4f1_development_data_2026-07-29/
  G2_4F1D_NON_HOLDOUT_DEVELOPMENT_DATA_PROTOCOL.md
```

在这些项目完成前：

```text
G2-4 dynamic threshold selection = 禁止
G1-F                          = 锁定
G1-D                          = 锁定
strict holdout                = sealed and unopened
```
