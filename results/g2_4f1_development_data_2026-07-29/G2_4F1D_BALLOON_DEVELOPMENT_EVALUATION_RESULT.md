# G2-4F1D Balloon 非 Holdout 开发评价结果

日期：2026-07-29

## 1. 结论

G2-4F1 在新的非 holdout 气球开发数据上得到明确的方向性证据：

```text
moving observable + exact zero person-mask overlap:
  candidate frames                 = 7
  frames with measured in-box ORB  = 6
  in-box residual median           = 11.919 px
  same-frame static-background     = 0.723 px
  paired in/out ratio median       = 20.045x
  paired difference median         = 11.280 px
  GT-pose in-box residual median   = 11.582 px
```

这里的背景也排除了实际 C++ person mask，因而内外比较使用相同语义口径。
6 个可测帧中，气球框内 residual 均高于同帧框外背景。

这说明 sparse observed-flow minus ego-flow 对语义未覆盖的独立运动气球具有
可用的连续 evidence。它仍不是二值动态检测器，也没有证明一个通用阈值。

冻结状态：

```text
dynamic_threshold          = none
dynamic_decision           = none
depth_flow_fusion          = none
direct_slam_state_mutation = none
G1-F / G1-D                = locked
strict holdout             = sealed and unopened
```

## 2. 数据角色与独立性

协议在查看数据前冻结：

- `rgbd_bonn_balloon`、`rgbd_bonn_balloon2`：development screening；
- `rgbd_bonn_balloon_tracking`：strict holdout，继续封存。

候选选择和 motion proxy 只读取 rectified RGB temporal clip 与同步 C++
person mask，不读取 depth residual、flow residual、region score 或 SLAM
trajectory。标签为：

```text
agent_rgb_temporal_observability_proxy
is_ground_truth       = false
geometry_or_flow_seen = false
```

Bonn 官方只说明数据集中有人玩气球，不提供逐帧对象运动 mask，因此 archive
名称没有被直接当成 motion GT。

协议：

```text
G2_4F1D_NON_HOLDOUT_DEVELOPMENT_DATA_PROTOCOL.md
```

## 3. 数据完整性

| 序列 | 一对一 RGB-D 对 | 缺失 RGB | 缺失 depth | 最大时间差 |
| --- | ---: | ---: | ---: | ---: |
| balloon | 438 | 0 | 2 | 16.670 ms |
| balloon2 | 469 | 0 | 2 | 16.680 ms |

不存在的 depth 文件已由 association 工具过滤，没有伪造或复用配对。

## 4. 同步语义与候选修正

完整 CUDA semantic manifest：

| 序列 | mask 就绪 | semantic total 中位 |
| --- | ---: | ---: |
| balloon | 438/438 | 11.506 ms |
| balloon2 | 469/469 | 11.536 ms |

严格 5 帧 person-absent 初筛只得到 `16/4` 个候选，而且气球大多已离开画面。
随后使用默认关闭的 `--all-frame-screening`，只按 RGB temporal change 与均匀
时间采样，各增加 24 个候选。

低分辨率联系表曾把黄色推车轮误认为气球。Agent 随后使用 exact 640×480
rectified 中心图纠正：

- `balloon` 12 个冻结审阅帧中，4 帧改为 `not_visible`；
- 2 帧只保留为边界 partial；
- 没有把这些错误样本留作正结果。

这次修正说明联系表只能用于筛选，不能代替 exact-center 审阅。

## 5. Semantic coverage

当前 C++ semantic mask 只包含经过 7×7 膨胀的 person union，
`nonzero=filtered`。

| 序列 | 可见/partial | bbox-person coverage 中位 | 最大 |
| --- | ---: | ---: | ---: |
| balloon | 8 | 0.000 | 0.038 |
| balloon2 | 9 | 0.163 | 0.602 |

`balloon` 中有 5 个完整可见运动帧的粗框内 person-mask 像素恰为 0；
`balloon2` 另有 frame 230 为 exact zero。frame 318 约 60.2% 粗框被 person
mask 覆盖，保留为语义混杂反例，不用于 exact-zero 主子集。

粗框是 Agent 的 RGB-only development proxy，不是 pixel GT。

## 6. G2-4F1 连续 evidence

两条完整序列都使用：

```text
online CUDA semantic
mask age = 0
adjacent successful-frame reference
current initial pose / previous final pose
shadow-only sparse LK residual
```

主子集按事先规定的条件选择：

```text
motion_label == moving_observable
visibility in {visible, partial, occluded}
person_mask_pixels_inside_bbox == 0
```

不使用 flow 数值进行选帧。

主结果：

| 指标 | 中位 |
| --- | ---: |
| 框内、去 person 的 SLAM-pose residual | 11.919 px |
| 框外、去 person 的 SLAM-pose residual | 0.723 px |
| 框内减框外 | 11.280 px |
| 框内/框外 | 20.045× |
| 框内 GT-pose residual | 11.582 px |

GT-pose 分支与 SLAM-pose 分支方向一致，支持“信号不只是初始 SLAM 位姿误差”
这一有限判断。Bonn GT 同步与标定仍有剩余不确定性，因此 GT residual 只作
诊断。

## 7. 运行时间

| 序列 | 帧 | actual FPS | active total 中位 | tracking 中位 | F1 中位 | deadline miss |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| balloon | 438 | 29.498 | 32.543 ms | 12.387 ms | 2.544 ms | 119/438 |
| balloon2 | 469 | 28.277 | 35.309 ms | 14.477 ms | 2.691 ms | 403/469 |

F1 自身仍约 2.5–2.7 ms；完整系统是否达到 30 Hz 取决于语义、图像读取和
tracking 总负载。不能从第一条序列外推第二条也满足实时门。

## 8. ATE/RPE 健康度记录

使用 `evo_* --align --t_max_diff 0.02`。20 ms 由本轮 association 的
16.68 ms 最大时间差决定；evo 默认 10 ms 只匹配约 60% 轨迹，旧输出也保留。

| 序列 | 匹配 pose | ATE RMSE | RPE RMSE, delta=1 frame |
| --- | ---: | ---: | ---: |
| balloon | 438/438 | 0.031321 m | 0.020778 m |
| balloon2 | 467/469 | 0.056248 m | 0.032343 m |

这些是“同步语义 + shadow geometry”的运行健康度，不是几何改善量；F1 没有
改变任何 tracking observation。

## 9. 局限与下一步

当前仍缺：

- 独立 pixel-level 气球 motion GT；
- 静止气球的同对象负样本；
- feature decision threshold 的跨序列验证；
- 静态背景 false-positive 门；
- G1-F 真实过滤后的 ATE/RPE 对照。

下一步不是立刻把 `11.9 px` 写成阈值。应先从本地 PaperNotes/PDF 核对
FlowFusion、静态点鲁棒权重和相关 feature gating 的原始方法，冻结：

1. 哪些质量条件允许 feature evidence 进入候选集；
2. 如何同时使用 residual 与 forward-backward tracking quality；
3. threshold 只从 development 取得、如何用静态序列和未打开 holdout 验证；
4. G1-F 的 fail-safe 和最小有效特征数。

在这个判决与静态风险协议通过前，G1-F/G1-D 继续锁定。

## 10. 主要证据

```text
balloon_f1_continuous_audit/summary.json
balloon_f1_continuous_audit/per_frame.csv
balloon_semantic_coverage_audit/summary.json
balloon_semantic_coverage_audit/per_frame_coverage.csv
balloon_f1_online.log
balloon2_f1_online.log
balloon_f1_evo_ape_20ms.txt
balloon_f1_evo_rpe_20ms.txt
balloon2_f1_evo_ape_20ms.txt
balloon2_f1_evo_rpe_20ms.txt
```
