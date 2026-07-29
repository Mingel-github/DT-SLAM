# G2-3R4 低分辨率区域近似 Shadow 规格

日期：2026-07-29
状态：设计冻结候选；尚未实现或运行实验

## 1. 阶段定位

当前处于几何证据“测量与表示”阶段的后段，尚未进入可靠动态判决和真实
SLAM 过滤。

G2-3R4 只回答：

> `[H]` 在同一个 scale-2 depth-pyramid 域运行当前区域划分，能否在相对
> full-resolution reference partition 的结构损失可接受时，净回收约 3 ms
> 区域划分成本中的足够部分？

它不是：

- 动态区域检测；
- object segmentation；
- full-resolution partition 的 GT 验证；
- 30 FPS 解决方案；
- G1-F 或 G1-D 放行实验。

## 2. 已知输入和成本上界

复用 G2-3R3：

```text
scale = 2
640×480 → 320×240
left-top anchor convention
boundary-aware block average
τ_rel = 0.025
τ_abs = 0.08 m
5 个共视参考
pyramid_dense_s2 evidence
```

G2-3R3 candidate-only：

| 序列 | active mean | full region partition mean | actual FPS |
| --- | ---: | ---: | ---: |
| walking | 38.596 ms | 2.932 ms | 24.995 |
| sitting | 38.997 ms | 3.113 ms | 24.958 |
| fr1/xyz | 39.992 ms | 3.259 ms | 24.299 |

即使完全消除 partition，算术下界仍约为：

```text
walking: 35.664 ms
sitting: 35.884 ms
fr1/xyz: 36.733 ms
```

因此 G2-3R4 不以 30 FPS 为通过条件，也不能宣称将解决实时性。

## 3. 文献与工程归属

详细账本见：

```text
results/g2_3r4_2026-07-29/
G2_3R4_LOW_RESOLUTION_REGION_LITERATURE_AUDIT.md
```

冻结归属：

- `[L/A]` KinectFusion 支持边界保持的低分辨率深度金字塔组件；
- `[A]` G2-3R0 是 SInDSLAM depth-boundary 形式的轻量改造；
- `[S/H]` 在 half-resolution 域运行当前 connected-component partition；
- `[S/H]` 将 half region relation 映射到 full-resolution ORB/mask 域；
- `[H]` 用它替代当前 full-resolution partition 的有效性。

不得把整体组合归因给某篇论文。

## 4. 两种 partition 的准确身份

### 4.1 Reference partition

```text
full-resolution current depth
→ 现有 PartitionDepthByDiscontinuity()
→ 640×480 reference labels F
```

它是：

> 当前高分辨率参考实现。

它不是 object-region GT，也不证明对应 region 是真实物体。

### 4.2 Candidate partition

```text
G2-3R3 已生成的 current half depth
→ 同一个 PartitionDepthByDiscontinuity()
→ 320×240 candidate labels L
```

要求：

- 不重复生成 current half depth；
- scale 固定为 2；
- 阈值固定为 `0.025/0.08m`；
- 不增加面积、seed ratio 或 dynamic threshold；
- 不扫描 scale 或 boundary threshold。

## 5. 坐标域与标签映射

full-resolution pixel `p=(u,v)` 对应：

```text
c(p) = (floor(u/2), floor(v/2))
```

candidate label 映射定义为：

```text
C(p) = L(c(p))
```

其中：

```text
-1 = invalid depth
-2 = depth boundary
>=0 = candidate region label
```

映射后的 2×2 full pixels 共享一个 candidate region cell。它们不是四次独立
区域测量或四次独立 evidence。

## 6. 禁止直接比较 region ID

`F(p)` 和 `C(p)` 的非负 region ID 都是 connected-component 遍历产生的任意
编号。以下比较无效：

```text
F(p) == C(p)
full_region_id == low_region_id
```

所有结构审计必须对 label permutation 不变。

## 7. 公共评价域

定义：

```text
ΩF = {p | F(p) >= 0}
ΩC = {p | C(p) >= 0}
Ω  = ΩF ∩ ΩC
N  = |Ω|
```

必须分别报告：

- full assigned pixels：`|ΩF|`；
- candidate assigned pixels：`|ΩC|`；
- common assigned pixels：`N`；
- candidate assignment retention：`N / |ΩF|`；
- candidate-only assigned ratio：`|ΩC \ ΩF| / |ΩC|`；
- invalid mismatch；
- boundary mismatch。

不得只在 `Ω` 上报告高一致性而隐藏 candidate 丢失的 full region pixels。

## 8. Label-permutation-invariant 结构指标

### 8.1 Contingency table

在每帧 `Ω` 上构造：

```text
n_ij = |{p ∈ Ω | F(p)=i 且 C(p)=j}|
n_i. = Σj n_ij
n_.j = Σi n_ij
```

所有 IoU、merge、split 和 pairwise 指标都从 `n_ij` 计算，不依赖 label 编号。

### 8.2 最优重叠 IoU

```text
IoU_ij = n_ij / (n_i. + n_.j - n_ij)
```

报告：

- 每个 reference region 的 `max_j IoU_ij`；
- 每个 candidate region 的 `max_i IoU_ij`；
- pixel-weighted mean；
- unweighted macro median、P10、P50、P90；
- 按 region pixels 分为 `1–64`、`65–255`、`>=256` 三档的统计。

同一个 region 可以是多个对方 region 的 best match；best IoU 不用于声称
object identity。

### 8.3 相对 reference 的跨边界合并率

candidate region 混入多个 reference region 的 pixel mass：

```text
merge_mass =
Σj (n_.j - max_i n_ij) / N
```

同时报告：

- 每帧 merge mass；
- mean、median、P95；
- candidate region overlap reference-region count 的直方图；
- 多对一 candidate region 数量和面积。

该指标只表示相对 reference partition 的合并，不是对象欠分割 GT。

### 8.4 相对 reference 的区域碎裂程度

reference region 被多个 candidate region 拆分的 pixel mass：

```text
fragmentation_mass =
Σi (n_i. - max_j n_ij) / N
```

同时报告：

- 每帧 fragmentation mass；
- mean、median、P95；
- reference region overlap candidate-region count 的直方图；
- 一对多 reference region 数量和面积。

该指标只表示相对 reference partition 的碎裂，不是对象过分割 GT。

### 8.5 Pixel-pair co-membership

无需生成 `O(N²)` pixel pairs。由 contingency table 精确计算：

```text
same_both = Σij choose(n_ij, 2)
same_ref  = Σi  choose(n_i., 2)
same_cand = Σj  choose(n_.j, 2)

co_membership_precision = same_both / same_cand
co_membership_recall    = same_both / same_ref
co_membership_F1        = harmonic_mean(precision, recall)
```

precision 对相对 reference 的错误合并更敏感；recall 对碎裂更敏感。

必须同时报告大区域支配风险：

- 全局 pixel-pair 指标；
- 逐帧指标；
- 按 reference region 面积分层的指标。

## 9. Boundary 指标

令：

```text
BF = full-resolution reference boundary mask
BC = nearest-expanded half-resolution candidate boundary mask
```

报告：

- candidate boundary 到 reference boundary 的 mean/P95 distance；
- reference boundary 到 candidate boundary 的 mean/P95 distance；
- full-resolution tolerance 1 px 和 2 px 下的 boundary precision；
- tolerance 1 px 和 2 px 下的 boundary recall；
- boundary F1；
- candidate boundary pixel ratio 与 reference boundary pixel ratio。

名称必须写成：

```text
相对 full-resolution reference partition 的 boundary agreement
```

不得写成真实对象边界 precision/recall。

## 10. ORB feature 区域一致性

对 `Frame::mvKeys`：

```text
feature k:
  u_k = static_cast<int>(mvKeys[k].pt.x)
  v_k = static_cast<int>(mvKeys[k].pt.y)
  full label      = F(u_k, v_k)
  candidate label = L(floor(u_k/2), floor(v_k/2))
```

这与当前 semantic/feature labeling 的 raw RGB pixel convention 一致。TUM
keypoint 坐标非负，因此 `static_cast<int>` 与 floor 等价。越界 feature 必须
单独计数，不能夹取到图像边缘。

只对两边 label 均非负的 feature 构造 contingency table，报告：

- full reference feature assignment count；
- candidate feature assignment count；
- common feature assignment retention；
- feature-region best IoU；
- ORB pairwise co-membership precision/recall/F1；
- 一对多和多对一 feature-region mapping。

这只说明区域映射关系，不是 dynamic-feature precision。

## 11. Native half-cell evidence 审计

### 11.1 禁止把 2×2 expansion 当四票

G2-3R4 primary evidence audit 必须在 320×240 native cell 域完成。

每个 half cell 的：

```text
comparison / positive / negative / consistent count
```

只能计一次。full-resolution expanded evidence 只为兼容 G2-3R3 reference
输出保留，不能作为 G2-3R4 primary vote unit。

### 11.2 Full reference 的 cell-domain 投影

对每个 2×2 full block：

- 若所有非边界、有效且已分配 full pixels 属于同一 reference region，
  则得到 strict reference-cell label；
- 若出现多个 full region、full boundary 或没有有效 assigned pixel，
  标为 mixed/boundary/invalid；
- mixed cell 单独报告，不能静默分配给 dominant label。

dominant full label 只允许作为补充诊断，不进入 primary gate。

### 11.3 Evidence assignment 指标

定义 strict reference-cell domain：

```text
Eref = {half cell | strict reference-cell label >= 0}
Ecmp = {cell ∈ Eref | comparison_count(cell) > 0}
Epos = {cell ∈ Eref | positive_count(cell) > 0}
```

retention 的分子是在对应集合中同时具有非负 candidate label 的 cell 数或 vote
和，分母分别是 `|Ecmp|`、`|Epos|` 或这些集合中的 reference vote 和。

在相同 native cells 上报告：

- comparison-cell assignment retention；
- positive-presence-cell assignment retention；
- comparison-vote assignment retention；
- positive-vote assignment retention；
- mixed/boundary cell evidence ratio；
- comparison-weighted merge mass；
- positive-presence-weighted merge mass；
- comparison-weighted fragmentation mass；
- positive-presence-weighted fragmentation mass。

对任意 cell weight `w(c)`，构造：

```text
n^w_ij = Σ w(c), for cells with reference label i and candidate label j
```

然后用 `n^w_ij` 代替 pixel contingency `n_ij`，复用第 8.3、8.4 节的
merge/fragmentation 公式。`w(c)` 分别取 comparison presence、positive
presence、comparison votes 和 positive votes。

总 comparison/positive vote conservation 必须作为确定性测试，但 vote
守恒不代表动态检测正确。

## 12. Semantic proxy 与静态风险代理

G2-3R4 不产生动态分类，禁止报告：

```text
dynamic mask precision
static false-positive rate
unknown-object recall
```

当前只报告风险代理：

1. fr1/xyz：
   - candidate region 的 background comparison coverage；
   - background positive/comparison 分布；
   - positive vote ratio 分布；
2. 相对 reference partition：
   - 静态序列的 comparison/positive evidence 跨区域混合；
   - candidate merge 后 minority reference surface 的 evidence mass；
   - 大 candidate region 是否吸收异常 positive evidence；
3. walking/sitting：
   - semantic proxy comparison coverage；
   - semantic/nonsemantic evidence 分布；
   - 只作为压力和定位诊断，不作为运动 GT。

所有相关字段和报告必须包含：

```text
risk proxy; not a measured dynamic FPR
```

TUM walking 不能证明未知动态物体检测。

## 13. 计时边界

### 13.1 Candidate online cost

候选在线路径只包含：

```text
复用已有 current half depth
+ half-resolution partition
+ native half-cell region aggregation
+ 当前 tracking 必需的 ORB/mask region mapping
```

不得把已有 pyramid depth preprocessing 重复计入 region path。

### 13.2 Audit-only cost

以下只属于审计：

```text
full-resolution reference partition
contingency/IoU/pairwise/boundary metrics
full-vs-half paired CSV
debug visualization
```

Candidate-only FPS 运行必须关闭全部 reference/audit-only 成本。

### 13.3 必报指标

- half partition mean/median/P95；
- half aggregation mean/median/P95；
- online mapping mean/median/P95；
- candidate online region path total；
- paired full reference region path；
- net saved mean/median/P95；
- active total mean/median/P95；
- deadline misses；
- actual FPS；
- mask ready 与 age；
- dense audit 是否关闭。

isolated region runtime 不能替代端到端结果。

## 14. 预冻结门槛

以下是 `[S]` 工程门槛，不是文献阈值或对象分割 GT。实现后不得在三个正式
序列上反复调节这些门槛。

### 14.1 结构安全门

三个序列分别要求：

```text
candidate assignment retention >= 95%
pixel-pair co-membership precision >= 95%
pixel-pair co-membership recall >= 90%
pixel-weighted merge_mass mean <= 2%
pixel-weighted merge_mass P95 <= 5%
pixel-weighted fragmentation_mass mean <= 5%
pixel-weighted fragmentation_mass P95 <= 15%
boundary F1 at 2 px tolerance >= 90%
ORB common assignment retention >= 95%
ORB co-membership F1 >= 95%
```

merge gate 比 fragmentation gate 更严格，因为错误合并会把不同 reference
surface 的 evidence 混在同一区域。

### 14.2 Evidence assignment 门

三个序列分别要求：

```text
comparison-cell assignment retention >= 95%
positive-presence-cell assignment retention >= 95%
comparison-weighted merge mass <= 3%
positive-presence-weighted merge mass <= 5%
```

这些门只评价近似是否破坏当前 evidence allocation，不评价动态准确率。

### 14.3 有界收益门

三个 candidate-only 序列分别要求：

```text
candidate online region path mean <= 1.5 ms
candidate online region path P95 <= 2.0 ms
paired net mean saving >= 2.0 ms
end-to-end active mean improvement >= 1.5 ms
```

actual FPS 和 deadline misses 必须报告，但不以 30 FPS 作为 G2-3R4 的
保留门槛。

## 15. 停止条件

G2-3R4 只允许：

- 一个冻结设计；
- 一个最小实现；
- 修复确定性 bug；
- 一轮正式三序列 paired audit；
- 一轮关闭 audit 的 candidate-only timing。

不得：

- 扫描 scale；
- 扫描 `τ_rel/τ_abs`；
- 增加面积、region count 或 positive ratio 阈值；
- 为通过结构门反复改变 label mapping；
- 开启新的长期逐毫秒 CPU 优化路线。

决策：

```text
结构安全门失败
→ 冻结为结构近似失败，停止该路线。

结构通过但有界收益门失败
→ 冻结为收益不足，停止该路线。

结构和有界收益均通过
→ 保留表示，下一步优先验证动态/静态区分能力。
```

只有非常明确、低风险、一次性的冗余遍历消除可以在评审后单独批准。

## 16. 确定性测试

实现时至少覆盖：

1. scale-2 full→half 坐标映射；
2. invalid、boundary、assigned label 映射；
3. region ID 完全置换后所有结构指标不变；
4. 人工一对一 partition：所有一致性指标为 1；
5. 人工多对一 partition：只触发 merge；
6. 人工一对多 partition：只触发 fragmentation；
7. mixed 2×2 reference cell 不进入 strict cell domain；
8. boundary distance 和 tolerance 计算；
9. ORB feature co-membership；
10. native half-cell vote conservation；
11. geometry disabled 时不创建 candidate region 状态；
12. 所有 shadow 输出不修改 SLAM state。

## 17. 数据与运行

第一轮只使用现有三序列约 199 帧：

```text
TUM fr3/walking_xyz
TUM fr3/sitting_static
TUM fr1/xyz
```

目的分别是：

- walking：已知人物高动态压力、proxy 和覆盖诊断；
- sitting：低动态人物压力诊断；
- fr1/xyz：静态背景风险代理。

本阶段不使用它们证明未知动态能力。

Bonn moving box 留给后续动态判决门控。使用 Bonn 前必须重新确认非零畸变下：

```text
RGB / registered depth / Frame::mvKeys / mask / geometry K
```

位于一致坐标域。

## 18. 输出

计划输出：

```text
results/g2_3r4_2026-07-29/
  G2_3R4_LOW_RESOLUTION_REGION_LITERATURE_AUDIT.md
  G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_SPEC.md
  G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_RESULT.md
  *_region_structure_pairs.csv
  *_region_structure_audit.json
  *_candidate_only.log
```

RESULT 只有在实现、测试和实验完成后创建。

## 19. 非目标和禁止修改

G2-3R4 不允许：

- 选择动态区域阈值；
- 输出 dynamic mask；
- 修改 `mvbDynamic`；
- 过滤 `mvpMapPoints`；
- 修改 MapPoint、KeyFrame 或地图；
- 修改 YOLO、semantic mask 或 async 架构；
- 修改 Optimizer/g2o；
- 增加 PoseOptimization；
- 修改 LocalMapping 或 LoopClosing；
- 测量或宣称 geometry ATE 改善；
- 宣称未知动态检测；
- 删除既有失败实验。

## 20. G2-3R4 之后

若 G2-3R4 保留：

```text
冻结可接受的 evidence representation
→ 可靠动态/静态区分能力和风险代理门控
→ Bonn moving box 或受控箱子序列未知动态验证
→ 在独立 calibration 数据上冻结判决参数
→ G1-F 高置信 feature filtering
→ 四模式多次 ATE/RPE/FPS 对照
→ pixel-region mask 独立通过后再讨论 G1-D
```

不会默认继续 CPU 优化。
