# S1 梯度深度边缘与区域切分规范

日期：2026-08-03  
阶段：S1 内部第四个增量  
状态：已按本规范实现并完成 30 帧成对 shadow 审计

## 1. 方法身份

本增量只实现：

> `[A] SInDSLAM 论文 Eq.(2)(3) 启发的 gradient-depth edge 与 initial-region 内连通切分`

不加入 PEAC 平面边缘、RAG 合并、光流、时序 dynamic state 或 SLAM 过滤。

## 2. 文献与源码边界

### `[L]` 论文

```text
delta_depth(u) = max |D(un)-D(u)|, un in B(u)
edge if delta_depth > max(tau1*D(u), tau2)
tau1 = 0.025
tau2 = 0.08 m
tau_depth = 6.0 m
```

论文未规定邻域尺寸、median、连通性、形态学、小区域阈值和边界像素归属。

### `[C]` 公开源码

- 5×5 median；
- 实际 5×5 邻域；
- 比例 0.03，不是论文 0.025；
- TUM 原始深度阈值 400，等于 0.08 m；
- raw gradient 后做 4×4 open；
- 后续再与 AGPL PEAC plane edge 合并并 close；
- contour 点数、80 px、dilation 等均为源码启发式。

### `[A]` 本项目

- 在全图深度邻域使用 masked 5×5 median，只收集
  `finite && 0<z<6m`；median/gradient 不受 initial label 边界限制，只有
  后续 connected split 限制在同一个 initial region 内；
- 最少有效支持数可配置并显式记录；
- 使用论文 `0.025/0.08m` 作为主输出；
- 主输出不做 morphology；
- 每个 initial region 内去掉 edge 后做 8 邻接 connected components；
- 小组件不删除、不合并，只统计 `<80 px` 反事实；
- edge 保持 label 0 boundary/unassigned，invalid 保持 -1。

## 3. 输出

```text
filteredDepth             CV_32FC1
medianSupport             CV_16UC1
medianValidMask           CV_8UC1
insufficientSupportMask   CV_8UC1
rawGradientEdgeMask       CV_8UC1
splitBoundaryMask         CV_8UC1
splitCoreLabels           CV_32SC1
    -1 = invalid / insufficient / initial unavailable
     0 = gradient boundary / unassigned
    >0 = split component
splitValidMask            CV_8UC1，只含正 component
```

诊断 PNG 会把内存标签 `-1` 和 `0` 都编码成零；必须结合 initial-label PNG
和 gradient-edge PNG 还原 invalid、insufficient 与 boundary，不能把 PNG
零值统一解释为静态或边界。内存接口仍保留完整三态语义。

不存在动态 mask，且：

```text
dynamic_state_available = 0
dynamic_decision = none
direct_slam_state_mutation = none
```

`median_valid + insufficient = initial_region_pixels` 依赖当前上游保证：每个
positive initial label 都来自有效且小于 6m 的深度。若未来换用其他 initial
partition provider，必须重新验证这一接口前提。

## 4. 区域切分语义

对每个 initial region：

```text
core = initial_region AND median_valid AND NOT raw_edge
components = connectedComponents(core, 8)
```

边缘不立即分配给任一 component，避免把刚切开的表面重新粘回。小组件不
删除，因为论文没有给出 80 px 删除规则，且完整方法后续依赖 RAG 合并。

## 5. 统计

- input/median-valid/insufficient 像素；
- raw edge、boundary、split core 像素；
- initial region 和 split component 数；
- 被切开的 initial region 数；
- fully consumed initial region 数；
- fragmentation 中位数/最大值；
- `<80 px` component 数和像素；
- median、edge、connected-components、total runtime。

## 6. 审计

作者 final labels 不是本增量 GT。只做描述性比较：

- split positive coverage；
- ARI/NMI；
- native raw edge 对 reference partition boundary 的 1/2 px precision/recall；
- initial→split 与 split→reference 的区域数/碎裂变化；
- 边界、core、invalid/unmeasured 的守恒；
- 运行成本。

## 7. 验收

- 0.2 m 明显跳变产生边缘并切分；
- 小于 0.08 m 且小于 2.5% 的变化不产生边缘；
- invalid hole 不制造假边缘；
- `z>=6m` 保持 invalid；
- 小组件不丢失；
- 输出确定；
- 30 帧 TUM shadow 不崩溃且实际删除为 0。

完成后仍不能称为完整 SInDSLAM re-clustering，也不能开放 S2。
