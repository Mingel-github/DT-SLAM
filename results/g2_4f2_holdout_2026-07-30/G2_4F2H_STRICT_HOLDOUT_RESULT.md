# G2-4F2H Strict Holdout 结果

日期：2026-07-30  
数据：Bonn `rgbd_bonn_balloon_tracking`，590 帧  
状态：一次性评价完成；冻结科学门失败；G1-F/G1-D 继续锁定

## 1. 方法身份与不变量

本轮评价的工作点在解封前已冻结：

```text
FB correspondence quality <= 0.25 px
frame scale = max(0.001 px, 1.4826 * median(nonsemantic eligible residual))
high-residual candidate = q >= 10
minimum scale support = 20
raw residual threshold = none
boundary veto = none
dynamic decision = none
direct SLAM mutation = none
```

来源边界：

- `[L/A]` FlowFusion 支持 observed flow 减 camera ego-flow 的物理残差；
- `[L/A]` Kalal 等支持 forward-backward error 作为 correspondence reliability；
- `[L/A/H]` Li–Lee 的零中心鲁棒尺度被改造到二维稀疏 flow residual；
- `[S/H]` `FB=0.25 px、q=10` 是开发集冻结工作点，不是论文参数。

本轮没有修改 `mvbDynamic`、`mvpMapPoints`、`Optimizer.cc`、g2o 或后端，也没有
增加第三次 `PoseOptimization`。

## 2. 防泄漏代理

在读取任何 holdout depth、trajectory、flow residual 或 candidate 之前：

1. 仅用共同去畸变针孔域的五帧 RGB clip 选出 14 个
   `moving_observable` proxy；
2. 冻结 14 个 RGB-only coarse balloon bbox；
3. 用同步 C++ person mask 做独立覆盖审计。

结果：

```text
proxy frames                         = 14
visible / partial                    = 13 / 1
exact person-mask zero frames        = 14/14
person pixels inside proxy bbox      = 0/14
motion labels are GT                 = false
geometry_or_flow_seen                = false
```

SHA-256：

```text
motion proxy:
  566f4220331761081b268e2536762970dda85e9743d873ef8feffc9e0facea3a
bbox proxy:
  386d5298160961b8ec83e6f1ac23ef62a2726eadb8ad48a498e6fb8ff14cf68f
review frame list:
  2771ce2720eee7be2f685af109b43eadadfccc43ba279041f56603cc4ccafa3b
```

这些代理不是真值，因此以下结果只能称为 proxy sensitivity / safety audit，不能
写成检测 precision/recall。

## 3. 唯一一次完整运行

输入和运行状态：

```text
frames                         = 590
semantic provider              = CUDAExecutionProvider
semantic masks                 = 590/590
semantic mask age              = median 0, max 0
common image domain            = rectified pinhole P=K
dynamic_decision               = none
direct_slam_state_mutation      = none
feature CSV rows               = 590752
frame diagnostic rows          = 589
```

几何输出 SHA-256：

```text
feature CSV:
  eaed441183868a394b8101434ddbf40943b55ddaa17951f1a688c97ae3369185
frame CSV:
  105b2291c761089fcffaa810ef708f1ef2b2ba36f0d875054da9eeecbed5ce3c
```

没有因结果不理想而重跑或回调 `FB/q`。

## 4. 冻结工作点结果

### 4.1 完整序列

```text
quality-eligible nonsemantic features    = 436910
quality coverage                         = 75.8598%
high-residual candidates                 = 5655
candidate / quality-eligible             = 1.2943%
candidate frames                         = 322/589
MapPoint quality-eligible                 = 107219
MapPoint candidates                       = 241
MapPoint candidate rate                   = 0.2248%
candidates/frame median / p95 / max       = 1 / 58.2 / 99
MapPoint candidates/frame median/p95/max  = 0 / 2 / 7
```

预冻结安全门要求：

```text
full-sequence MapPoint candidate rate <= 0.20%
```

实际为 `0.2248%`，未通过。该指标是预先定义的保守删除预算，不是静态
false-positive rate；序列中确有动态气球。

### 4.2 14 个 RGB-only moving / exact-zero-person proxy

```text
measurable frames                         = 14/14
frames with in-box q>=10 candidate        = 7/14 = 50%
frames with inside rate > outside rate    = 7/14 = 50%
required                                  = at least 80%
```

冻结 hard-candidate 门明显未通过。

对连续 residual 而不是硬候选做成对检查：

```text
SLAM-pose inside median > outside median  = 13/14
GT-pose inside median > outside median    = 12/14
both directions positive                  = 11/14
SLAM/GT sign agreement                    = 11/14

SLAM-pose in-box median residual:
  median = 5.236 px
SLAM-pose outside median residual:
  median = 0.732 px
inside/outside median ratio:
  median = 9.980
```

因此证据支持的结论是：

> 稀疏 ego-flow 连续不一致在大多数 RGB-only moving proxy 中具有正确的局部
> 富集方向，但当前基于全帧鲁棒尺度的 `q>=10` 硬候选化不能稳定保留该证据。

不能写成：

> 已经可靠检测未知动态对象。

## 5. 轨迹健康度

本轮为 shadow-only，轨迹不受 candidate 影响。Bonn GT 与 RGB 时间戳采用与
在线 GT diagnostic 一致的 `40 ms` 最大配对差：

```text
ATE pairs          = 590
ATE RMSE           = 0.031290 m
ATE median         = 0.022419 m

RPE pairs          = 589
RPE RMSE           = 0.024986 m
RPE median         = 0.017872 m
```

这只说明本轮轨迹可评价。没有同序列、同运行条件的 mutation baseline，因此
不能将 ATE/RPE 解释成 geometry 改善。

默认 evo `10 ms` 只配对 370/590 帧；结果保留在 `evo_ape.txt` 和
`evo_rpe.txt`，不作为主报告。

## 6. 性能测量缺陷与修复

首次 holdout 运行记录：

```text
YOLO semantic median / p95 = 6.148 / 9.268 ms
tracking median / p95      = 25.967 / 37.225 ms
end-to-end actual FPS      = 24.002
deadline miss              = 475/590
```

源码核查发现诊断记录容器每帧执行：

```cpp
reserve(current_size + current_frame_samples);
```

这会在每一帧把 capacity 精确增长约 1000，导致历史记录近似反复全量搬移。
对应现象是 F1 `active_ms` 从早期约 `2.8 ms` 逐步升到末段约 `26 ms`。

这属于 `[S]` 诊断基础设施实现缺陷，不是论文方法复杂度。修复只删除该
`reserve`，恢复 `std::vector::push_back` 的摊销增长；不改变 residual、
feature index、候选公式或 SLAM 状态。

按“一次 holdout 运行”协议没有重跑 `balloon_tracking`。修复只在非 holdout
Bonn `static_close_far` 150 帧验证：

```text
F1 active_ms at frames 1/30/60/90/120
  = 2.883 / 2.737 / 2.787 / 2.905 / 2.767 ms
end-to-end actual FPS
  = 29.720
deadline miss
  = 0/150
```

所以：

- 首次 holdout 的 residual/candidate 科学评价仍有效；
- 首次 holdout 的 24 FPS 不代表修复后的算法运行成本；
- static150 只证明 O(N²) 记录增长已消失，不能替代 holdout 性能重测。

## 7. 决策

严格门判定：

| 门 | 要求 | 结果 | 判定 |
| --- | ---: | ---: | --- |
| 可测 moving exact-zero proxy | >=5 | 14 | 通过 |
| 框内有 candidate | >=80% | 50% | 失败 |
| inside rate > outside | >=80% | 50% | 失败 |
| MapPoint candidate rate | <=0.20% | 0.2248% | 失败 |
| invariant violation | 0 | 0 | 通过 |

冻结结论：

```text
G2-4F2 hard candidate gate = FAILED ON STRICT HOLDOUT
G1-F0 mutation shadow      = NOT RELEASED
G1-F real filtering        = LOCKED
G1-D depth/map filtering   = LOCKED
threshold retuning         = FORBIDDEN ON THIS HOLDOUT
```

保留：

- G2-4F1 连续 residual 和 validity/quality 字段；
- `FB<=0.25,q>=10` 作为失败的冻结 baseline；
- 全部失败帧、CSV、日志和代理。

不保留：

- 将 `q>=10` 解释为可靠 `dynamic=true`；
- 以降低阈值修补留出失败；
- 将 static150 性能替代 holdout 科学结果。

