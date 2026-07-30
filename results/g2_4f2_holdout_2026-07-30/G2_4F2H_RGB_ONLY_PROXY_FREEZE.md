# G2-4F2H RGB-only 留出集代理冻结记录

日期：2026-07-30  
数据：Bonn `rgbd_bonn_balloon_tracking`  
状态：代理已在读取任何 holdout depth、trajectory、geometry、flow 或 candidate 输出前冻结

## 1. 目的与边界

本记录只冻结严格留出集的评价代理，不生成对象运动真值：

```text
label_source = agent_rgb_temporal_only_holdout_v1
is_ground_truth = false
geometry_or_flow_seen = false
```

运动代理来自去畸变共同针孔域中的五帧 RGB clip。候选帧由既有 RGB
变化/语义清单规则产生；选帧和复核均未读取 depth、camera trajectory、
F1 residual 或冻结的 `q=10` candidate。

这一步属于评价协议 `[S]`，不是运动检测方法，也没有论文创新主张。

## 2. 冻结对象

- 14 个 `moving_observable` RGB temporal proxy；
- 13 个完整可见、1 个右边界部分可见；
- 14/14 的当前 C++ person mask 全图均为精确全零；
- 每个对象框均为带少量边缘余量的 agent RGB-only coarse bbox；
- 框不是像素级 GT，不用于训练或调参。

文件：

```text
balloon_tracking_rgb_temporal_motion_proxy.csv
balloon_tracking_rgb_only_coarse_bboxes.csv
balloon_tracking_frozen_review_frames.csv
```

## 3. 文献与方法边界

本阶段真正接受检验的几何量仍是：

- `[L/A]` FlowFusion：observed flow 与 camera-induced ego-flow 的残差；
- `[L/A]` Kalal 等：forward-backward error 只作为对应可靠性；
- `[L/A/H]` Li–Lee：零中心鲁棒尺度思想被改造到二维稀疏 flow residual；
- `[S/H]` `FB<=0.25 px、q>=10` 是开发集冻结工作点，不是论文参数。

RGB temporal proxy 只用于避免依赖 geometry 自己给自己标注；不能把本轮
结果写成有标注数据集上的 precision/recall。

## 4. 解封后的不变量

从本文件和三个 CSV 写入后开始：

```text
不得修改 14 个 proxy 以适配 holdout residual；
不得修改 bbox 以包住 candidate；
不得调整 FB/q；
完整 geometry shadow 只正式运行一次；
不写 mvbDynamic；
不清除 mvpMapPoints；
dynamic_decision=none；
direct_slam_state_mutation=none。
```

若发现实现 bug，保留首次输出并将评价标记为无效；若仅仅结果不理想，不允许
重跑后选择更好结果。

## 5. 预几何一致性修正

第一次 semantic-coverage 输入校验在读取任何 holdout geometry/flow 前拒绝了
frame 381 的 `(254,320,122,162)`：底边超出 480 行图像 2 pixel。只将高度
裁剪为 160；对象选择、运动标签、框的其余边界和冻结工作点均未改变。失败的
partial audit 目录保留，正式 coverage 输出使用 `_v2` 后缀。
