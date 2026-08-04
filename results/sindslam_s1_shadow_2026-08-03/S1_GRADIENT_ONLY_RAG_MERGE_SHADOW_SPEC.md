# S1 Gradient-only RAG merge shadow 规范

日期：2026-08-03  
状态：实现前冻结

## 1. 身份

本增量是：

> `[A] 论文 RAG/深度直方图合并在当前 gradient-only split 上的
> clean-room shadow adaptation`

它不是完整 SInDSLAM re-clustering，因为还没有 plane edge rejection。
它不产生动态状态，不进入 Tracking。

## 2. 输入和区域属性

输入：

```text
CV_32F meter depth
initial K-means labels
gradient split core labels
raw gradient edge
```

每个 split component 计算：

- 面积；
- 三维中心/中心深度；
- 原 initial-region 身份；
- `[0,6m)` 固定范围的 256-bin、L1-normalized depth histogram；
- 与相邻 component 的共享 fake boundary。

当前 clean-room adaptation 将 `fake boundary` 冻结为：两个属于**不同
initial K-means region** 的 core component 在原始 gradient edge 以外发生的
4-neighbour 直接接触。它是 `[A]` 的零宽离散近似，不是作者源码形态学
`lianjie` mask 的逐行复刻。属于同一个 initial region、但被 gradient edge
切开的 component 永远不是合并候选。

## 3. RAG 证据

主 profile 使用论文 Table II：

```text
tau3=200
lambda1=0.05
lambda2=0.01
omega_low/mid/high=0.7/1.0/2.0
tau_merge=0.9
tau_reject=0.2
```

深度直方图按 Appendix A1--A5 计算 correlation、Bhattacharyya 和
normalized intersection。直方图固定在 `[0,6m)`，使用 256 个 bin 和 L1
归一化；合并组的直方图由原始 bin count 求和后重新归一化，不对 component
直方图做等权平均。空间邻接、fake edge 和深度相似度分别记录，不能只输出
一个不可解释总分。

本实现的 candidate 必须同时满足：

1. 两个当前 group 的 initial-region 身份集合不相交；
2. 存在至少一个 shared fake-boundary contact (`M2>0`)；
3. 7×7 ellipse dilation 后交叠满足论文式 `M1` 门槛；
4. `M3 >= tau_reject`。因为当前没有 plane edge，深度拒绝对所有 pair 生效，
   不采用作者源码对小区域放宽拒绝的启发式。

由于 plane edge 尚不可用：

```text
plane_rejection_available = false
```

所有合并结果必须同时报告这一缺失条件。

## 4. 合并

- 按论文得分降序；同分时 source label 小者优先；
- 中高分阶段采用 `tau_merge`；
- 低分阶段采用 `0.2*tau_merge`；
- 未找到合格目标的低分区域保留，不能按作者源码行为丢入 invalid；
- 两阶段阈值均使用严格 `>`；
- 每次合并后重新计算区域 mask、直方图、面积/深度和所有候选分数，禁止用
  旧 RAG 矩阵分数相加；
- 中高分阶段每轮选择全局最高分 pair，目标为 fixed rank 较小者；同分时按
  target fixed rank、donor fixed rank 升序；
- 低分阶段按 fixed rank 依次处理，为其选择得分最高的活动中高分目标；
- rank weight 按论文 profile 由 pair 中较小的 fixed rank 决定；作者源码按
  较小面积区域标签加权的启发式不混入主 profile；
- 合并顺序和 tie-break 固定，输出必须可重复；
- gradient boundary 不得因简单最近邻回填而消失。

## 5. 输出与统计

```text
merged_labels             -1 invalid, 0 boundary, >0 merged region
candidate/accepted pair counts
M1/M2/M3/weight/reject/total score summaries
pre/post region counts
merge group sizes
unmerged low-score regions
cross-gradient merge violations
histogram/RAG/merge/total runtime
```

作者 final labels 只做 ARI/NMI、边界和区域数的描述性参考，不是 GT。

## 6. 放行条件

- 单元测试证明相似且仅由 fake boundary 分开的区域可合并；
- 明显深度不同或由 gradient true edge 分开的区域不得合并；
- invalid/unknown 不被填成静态区域；
- 输出确定；
- 30 帧 invariant 通过且 `actual_slam_removed=0`；
- 若出现明显跨 gradient 合并或全图塌缩，冻结为失败，不补面积阈值。

即使通过，本增量仍不开放 S2。下一层仍需 dense-flow residual 和时序状态
形成真正动态 mask。
