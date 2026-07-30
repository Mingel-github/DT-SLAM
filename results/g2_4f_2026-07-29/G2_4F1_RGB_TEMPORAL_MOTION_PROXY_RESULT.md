# G2-4F1 独立 RGB 时序运动代理结果

日期：2026-07-29
状态：development proxy 已冻结；非 GT。
范围：只用于后续 shadow evidence 分层，不用于训练、调阈值或修改 SLAM。

## 1. 防止循环验证

每个候选仅查看同一已校正针孔域中的五帧 RGB：

```text
t-2, t-1, t, t+1, t+2
```

序列边界使用最近的连续五帧窗口，不复制帧。审阅时只显示中心帧已有 RGB-only
粗框，不读取：

- depth；
- geometry residual/vote/region；
- optical flow；
- candidate proxy 数值；
- SLAM outlier。

因此标签来源为：

```text
agent_rgb_temporal_only_v1
geometry_or_flow_seen = false
is_ground_truth = false
```

它是低成本 development 代理，不替代人工或数据集对象运动真值。

## 2. 输入一致性

```text
候选帧                       = 48
Python/C++ rectified center  = 48/48 byte-exact
边界平移窗口                 = 每序列 1 个
strict hold-out              = 未访问
```

三个错误粗框先完成时序复核修正，再生成当前 v2 审阅图；未使用旧粗框标签。

## 3. 保守标签结果

| 序列 | stationary | moving | uncertain | not_visible |
| --- | ---: | ---: | ---: | ---: |
| moving_nonobstructing_box | 19 | 5 | 0 | 0 |
| moving_obstructing_box | 11 | 2 | 2 | 9 |
| 合计 | 30 | 7 | 2 | 9 |

置信度：

```text
high     = 40
moderate = 6
low      = 2
```

moving 代理主要对应：

- nonobstructing 297/306/313/364/385：人物搬运或放下箱体；
- obstructing 158/199：人物携带箱板。

obstructing 219/239 因目标几乎填满图像、缺少稳定背景，保持 `uncertain/low`；
目标不可独立看到的帧保持 `not_visible`，没有强行推断运动。

## 4. 使用边界

后续 G2-4F1 只能：

1. 在 moving/stationary 代理层中报告连续 ego-flow residual 分布；
2. 将 uncertain/not_visible 独立报告或排除于可分性统计；
3. 同时报告样本数和置信度；
4. 保持原始逐帧原因，不能将代理改称 GT。

禁止：

- 依据 flow 结果反向修改这些标签；
- 在这 7 个 moving 小样本上拟合并宣称通用阈值；
- 解封 strict hold-out；
- 由代理标签直接写入 `mvbDynamic` 或过滤 MapPoint。

## 5. 下一步

按已冻结的 `G2_4F1_SPARSE_EGO_FLOW_SHADOW_SPEC.md` 实现最小稀疏
ego-motion-compensated LK residual：

```text
dynamic_decision            = none
depth_flow_fusion            = none
direct_slam_state_mutation   = none
G1-F / G1-D                  = locked
```
