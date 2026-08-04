# S1 RAG 与深度直方图合并：论文/源码核对

日期：2026-08-03  
范围：本地 PaperNotes、论文 PDF、作者公开 `DynaDetect.cc`  
性质：只读审计

## 1. 论文方法 `[L]`

论文的重聚类并不是“把相邻小块直接合并”。每个切分后区域包含：

```text
图像区域、三维中心、深度直方图、区域边缘、fake edge
```

`fake edge` 指初始 K-means 人工产生、但不属于真实几何边缘的边界。论文
用它判断两个碎片是否可能原本属于同一物体。

区域得分：

```text
s_i = lambda1 * area_i - center_depth_i
```

按得分降序排序后，RAG 相关性为：

```text
M_RAG = M1 * (lambda2*M2 + M3) * M_weight * M_rej
```

- `M1`：膨胀后有足够交叠，表示空间邻接；
- `M2`：共同 fake-edge 面积；
- `M3`：深度直方图相关性、Bhattacharyya 和归一化交集；
- `M_weight`：大/近区域降低合并权重，小/远区域提高权重；
- `M_rej`：平面真边或深度相似度过低时拒绝合并。

论文 Table II：

```text
tau3=200, lambda1=0.05, lambda2=0.01
omega_low=0.7, omega_mid=1.0, omega_high=2.0
tau_merge=0.9, tau_reject=0.2
```

先对排名前 70% 的中高分区域用 `tau_merge` 合并，再让低分区域以
`0.2*tau_merge` 尝试并入中高分区域；论文明确说无法合并的低分区域应保留。

## 2. 作者公开源码 `[C]`

公开实现包含论文没有完全冻结的强启发式：

- split 前后使用 4×4 open、9×9/7×7 dilation；
- 论文 Eq.(8) 的 M1 是
  `overlap > 0.4*min(tau3, area_i, area_j)`，公开源码实际写成
  `overlap > min(200, 0.4*smaller_area)`；当前主 profile 采用论文式，不能
  把两者混为同一阈值；
- contour 点数必须 `>50`、面积必须 `>80 px`；
- score 中实际写成 `0.0003*area-z`，不是论文 Table II 的
  `lambda1=0.05`；
- 大区域权重以“前十名”判断，小区域边界使用 `min(0.7*n,15)`，与论文的
  0.5n/0.7n 分段不完全相同；
- 直方图交集项在源码中乘 `0.0005`，与附录 A4 的归一化交集公式不同；
- source 的低分第二阶段默认并入 invalid bucket，而论文说无法合并时保留；
- 中高分合并选择代码没有随候选更新最大值，不能直接视为严格 argmax；
- plane-edge rejection 依赖 PEAC/AGPL 相关实现。

因此，公开源码可用于核对行为和参数，但不适合逐行搬进 DT-SLAM。仓库内
ORB-SLAM2 许可证为 GPL；当前仍采用 clean-room 实现，并单独记录论文与
源码差异。

## 3. 对当前 DT-SLAM 的约束

当前已有 gradient edge，但尚无 plane edge。直接声称复现完整 RAG 不成立：

- `M_rej` 缺少平面真边拒绝；
- 当前 split 保留所有小组件，而作者源码会删除一部分；
- 当前边界像素保持 unassigned，作者源码会膨胀回填；
- 当前直方图应在固定米制范围 `[0,6m)` 内构造，不能依赖每帧最大深度
  归一化而掩盖物理差异。

这些差异不阻止做 clean-room RAG shadow，但必须把输出称为：

> gradient-only RAG merge adaptation

而不是 SInDSLAM re-clustering reproduction。

## 4. 下一增量的安全结论

不应使用“面积小就并回最近区域”的临时规则。下一增量应按论文 RAG 结构
显式输出每项证据，并保留无合并区域：

```text
M1 adjacency
M2 shared fake edge
M3 metric-depth histogram similarity
rank weight
depth rejection
missing plane-rejection flag
```

在 shadow 中先审计哪些 pair 会越过真实梯度边缘、多少区域被合并、是否
把 72 个碎片压回合理数量；只有这些结构检查通过后，才把 merge labels
交给后续 dense-flow dynamic detection。
