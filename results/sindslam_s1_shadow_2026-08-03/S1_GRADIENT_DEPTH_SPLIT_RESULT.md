# S1 梯度深度边缘与区域切分结果

日期：2026-08-03  
阶段：S1 内部第四个增量  
状态：完成；仍为 shadow-only

## 1. 本轮做了什么

在已通过审计的 coarse-to-fine 初始三维分区之后，实现：

```text
masked 5×5 metric-depth median       [A]
paper 0.025 / 0.08 m gradient test   [L/A]
initial region 内 8 邻接连通切分      [A]
```

论文没有冻结 masked median、最小支持数、连通性、小区域处理和边界归属；
这些均明确记为 clean-room 改造。主输出没有复制作者源码的 morphology，
没有加入 PEAC plane edge、RAG merge、dense flow 或 dynamic decision。

## 2. 工程验证

- `sin_style_shadow_test` 通过；
- `rgbd_tum` 构建通过；
- 0.2 m 跳变产生边缘并把单区域切成两块；
- 小于 0.08 m 的测试跳变不产生边缘；
- invalid hole 和 `z>=6m` 不产生虚假有效证据；
- 小组件只统计、不删除；
- 两次相同 30 帧运行的 edge/split PNG 逐像素一致，30/30 通过；
- CSV、标签、面积、碎裂、small-component 和运行时 invariant 全部通过；
- `actual_slam_removed=0`，没有修改 `mvbDynamic`、`mvpMapPoints` 或
  MapPoint 写入。

“没有直接 SLAM 状态变更”不等于“对运行系统零影响”：shadow 计算和 PNG
写盘会增加 Tracking 线程耗时，本轮已单独报告这一开销。

## 3. 30 帧 TUM3 walking 结果

### 3.1 区域与边缘

| 指标 | 结果 |
| --- | ---: |
| initial region 数中位数 | 12 |
| split component 数中位数 | 72 |
| 每帧被切开的 initial region 均值 | 10.2 |
| fragmentation 中位数的帧均值 | 5.37 |
| 单区域最大 fragmentation | 20 |
| raw edge / median-valid | 8.76% |
| split core / 全图 | 60.79% |

结果表明梯度边缘确实切开了初始区域，但仅加入 depth-gradient 后出现明显
过分割。完整 SInDSLAM 随后还使用 plane edge、形态学、轮廓筛选和 RAG
合并；因此当前 72 个 component 不能称为对象区域。

### 3.2 与作者 final partition 的描述性比较

| 指标 | initial partition | gradient split |
| --- | ---: | ---: |
| ARI | 0.5407 | 0.5567 |
| NMI | 0.7505 | 0.7892 |
| boundary precision @2px | 0.3515（initial boundary） | 0.3421（raw edge） |
| reference boundary recall @2px | 0.6474 | 0.8255 |

这些数值只能说明结构关系：gradient edge 覆盖了更多作者最终分区边界，
同时产生了较多额外边缘。作者 final labels 已经过后续 split/merge，不是
本增量的边缘真值，不能据此把 34.2% 解释成真实检测 precision。

### 3.3 运行成本

| 部分 | 30 帧均值 |
| --- | ---: |
| masked median | 25.31 ms |
| gradient edge | 3.22 ms |
| connected split | 3.01 ms |
| gradient split total | 31.93 ms |
| native initial regions total | 7.45 ms |
| 含 reference replay 的端到端 active total | 65.85 ms |
| 本次进程实际 FPS | 15.18 |

当前 masked median 是朴素、可审计的正确性实现，单独占约 25 ms；不能把
它描述为实时实现。上述端到端数字还包含 reference replay、ORB-SLAM2 和
诊断 PNG 写盘，不等于未来完整 SIn detector 的最终速度。

## 4. 客观结论

本轮通过的是：

> 有论文阈值依据的 depth-gradient split 能稳定、确定地把初始区域切开，
> 并提高对作者最终分区边界的描述性覆盖。

本轮没有通过、也没有测试的是：

- 区域是否对应运动对象；
- 区域是否动态；
- unknown box 检测；
- Tracking 或 ATE 改善；
- 完整深度 mask；
- 30 FPS。

单独切分把 12 个初始区域扩展到约 72 个 component，说明下一步不能直接
将 component 当对象或用于过滤。仍需按 SIn 主链审计并实现区域合并；在
此之前 S2 继续锁定。

## 5. 证据文件

- `gradient_split_30.csv/.log`；
- `gradient_split_30_initial_audit.json`；
- `gradient_split_30_audit.json`；
- `gradient_split_30_determinism_audit.json`；
- `gradient_split_30_initial_labels/`；
- `gradient_split_30_outputs/`；
- `gradient_split_30_repeat.csv/.log`；
- `gradient_split_30_repeat_initial_labels/`；
- `gradient_split_30_repeat_outputs/`。

## 6. 下一步

继续留在 S1，优先审计作者 RAG/深度直方图合并的论文定义、源码行为、
依赖和许可证边界，再冻结一个最小 merge shadow 规范。不会直接将本轮
过分割区域接入 Tracking，也不会通过临时面积阈值把它包装成对象。
