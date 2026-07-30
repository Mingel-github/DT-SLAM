# G2-4F4 区域上下文连续残差 Shadow 规格

日期：2026-07-30
状态：实现前冻结
运行方式：offline shadow audit only

## 1. 输入冻结

development 只使用已经打开的：

```text
rgbd_bonn_balloon
rgbd_bonn_balloon2
```

输入：

- G2-4F3U 在线运行保存的 exact C++ node CSV；
- geometry 计算前冻结的 RGB-only coarse bbox；
- geometry 计算前冻结的 candidate frame/depth path；
- 原始 Bonn depth archive；
- 已冻结的 Bonn rectification `P=K`。

不使用 `rgbd_bonn_balloon_tracking`；它已经完成过 F2 一次性 holdout，不允许
在本阶段用于选择规则。

## 2. 坐标域与区域划分

depth 必须按与在线输入相同的方式处理：

```text
raw registered uint16 depth
→ cv::remap(..., INTER_NEAREST)
→ CV_32F meters, scale 1/5000
→ rectified pinhole P=K, 640×480
```

区域划分精确复用 G2-3R0 定义：

```text
boundary iff any valid 4-neighbor satisfies
|D(n)-D(p)| > max(0.025 D(p), 0.08 m)

invalid depth = -1
boundary      = -2
4-connected non-boundary region >= 0
```

它是轻量 depth-discontinuity partition，不是对象实例 GT。

## 3. Feature 输入

只读取 exact C++ node CSV 中：

```text
evidence_state == measured
semantic_nonzero == 0
```

这些行已满足 F3 的 frozen quality 条件：

- sparse ego-flow measured；
- forward-backward error `<=0.25 px`；
- current/reference depth 有效；
- current image numerical duplicate 已处理。

不得在离线工具中重新计算 LK、ego-flow residual 或 semantic mask。

## 4. Region 统计

feature 以 rounded `(u_current,v_current)` 采样 region label。分别统计：

- assigned / boundary / invalid feature 数；
- region pixel area；
- eligible feature support；
- flow residual median、P90；
- residual MAD；
- feature density；
- bbox 内外 feature 数。

未分配、boundary 或 invalid 保持 no-region evidence，不能解释为 static。

## 5. 冻结 bbox proxy 映射

每帧只为评价定义一个 `proxy-selected region`：

1. 在 bbox 内获得最多 eligible feature 的 region；
2. 并列时选 bbox 内 feature 比例更高者；
3. 再并列时选较小 region id，确保确定性；
4. bbox 内没有 assigned feature 时，该帧不可测。

该选择只使用 bbox、feature 坐标和 region label，不读取 residual。

必须报告：

- selected region 的 bbox 内 feature 数；
- selected region 总 feature 数；
- `bbox_feature_purity = inside / total`；
- selected region pixel 与 bbox 的交叠比例；
- selected region residual median/P90/MAD；
- 同帧非 selected、bbox 外 background region/node residual；
- selected/background ratio 与 difference。

`purity` 只是 coarse-box leakage proxy，不是 segmentation precision。

## 6. 无阈值比较

与 F1/F3 既有 point proxy 成对比较：

- selected region median 是否高于 same-frame background；
- selected region P90 是否高于 background；
- selected region是否降低 frame-to-frame sign consistency；
- region 聚合前后的 paired ratio；
- 可测帧数和低支持帧数；
- region leakage proxy 分布。

不生成：

```text
dynamic=true
static=true
candidate=true
```

## 7. 预冻结停止条件

本阶段是 representation viability，不是分类精度。以下任一成立即停止：

1. 少于 10 个冻结 bbox 帧能映射到至少 3 个 eligible region features；
2. `selected median > background median` 少于 80% 的可比帧；
3. 相比既有 point-level inside/outside，方向一致性明显下降；
4. 多数 selected region 的 feature/pixel overlap 显示其横跨大块背景；
5. 结果主要由 1–2 个 feature 的小区域决定。

80% 是本项目预注册的 representation 保真门，不是论文参数或动态阈值。

若通过，也只允许继续设计 online shadow region support；不得直接进入 G1-F。

## 8. 工程验收

- Python self-test 覆盖 plane、depth step、invalid barrier；
- 同一 synthetic depth 的 boundary/label 结果与 G2-3R0 定义一致；
- CSV 不变量 `dynamic_decision=none`、`direct_slam_state_mutation=none`；
- 工具只写 `results/g2_4f4_2026-07-30/`；
- 不修改核心源码；
- `git diff --check` 通过。
