# G2-2G 规则深度网格采样 Shadow 结果

日期：2026-07-28

## 1. 结论

G2-2G 已完成 stride 2/4/8 密度扫描、stride 4 跨序列复核和同帧同位姿同参考
dense 审计。

结论是：

```text
规则网格 = 有效的中间密度工程对照
         ≠ 足以独立输出动态深度区域的方法
         ≠ 已批准的动态特征过滤器
```

stride 4 是当前三种网格中的较好折中：K=5 G2 约 1.60–1.99 ms，五参考任意
比较覆盖约 11.67%–16.39%。但它仍丢失约四分之三的 dense 覆盖，在线同步系统
约 28.08–28.32 FPS，没有稳定达到 30 FPS。

## 2. 方法身份和依据边界

本地 DynaSLAM 支持 feature-associated depth seed，DetectFusion 和 SInDSLAM
支持“几何区域边界＋区域内残差聚合”，但都不提供当前规则网格的直接算法。

因此 G2-2G 明确标记为：

> `[S]` 标准规则栅格抽样工程控制，用于测量采样密度、覆盖和耗时曲线。

它不是 DynaSLAM、DetectFusion 或 SInDSLAM 复现，也不是论文创新。

## 3. 实现

新增：

```yaml
Geometry.MultiReferenceSamplingPolicy: "grid_depth"
Geometry.MultiReferenceGridStride: 4
```

可用环境变量仅为实验覆盖 stride：

```bash
DT_SLAM_GEOMETRY_GRID_STRIDE=2|4|8
```

参考关键帧只缓存规则网格上语义清理后仍有效的深度像素。后续保持：

- G2-2R covisibility、K=5 和 20 项缓存；
- 相同 Tcw、K、0.10 m 诊断阈值；
- 最近表面 z-buffer；
- positive、negative、consistent 和 unknown 分离；
- 不生成动态 mask，不修改 SLAM 状态。

初次实现沿用为约 400 个 ORB 样本设计的哈希 z-buffer。stride 2 达到每参考约
3.6 万样本后，哈希开销不再代表必要算法成本。最终实现对小样本保留哈希，对大
网格使用连续预测深度缓冲和触达索引。该修改不改变证据，只修正数据结构开销。

## 4. Walking 密度扫描

三次均为前 199 帧、在线 CUDA 语义、viewer 关闭、
`mask ready=199/199`、`mask age median/max=0/0`。

| 采样 | 有效样本/参考 | 单参考比较覆盖 | 五参考任意覆盖 | G2 mean | active mean | actual FPS | deadline missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| grid s2 | 36099 | 10.446% | 33.300% | 3.594 ms | 36.197 ms | 26.533 | 160/199 |
| grid s4 | 8967 | 2.596% | 11.674% | 1.598 ms | 32.912 ms | 28.118 | 96/199 |
| grid s8 | 2238 | 0.648% | 3.135% | 1.337 ms | 32.702 ms | 28.165 | 88/199 |

判断：

- stride 2 有明显更高覆盖，但端到端成本超过 30 FPS 预算；
- stride 8 比 stride 4 只节省约 0.26 ms，却将覆盖从约 11.67% 降至 3.13%；
- stride 4 是合理的中间密度对照，但并未通过最终门控。

## 5. Stride 4 跨序列结果

| 序列 | 五参考任意覆盖 | G2 mean | active mean | active p95 | actual FPS | deadline missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 11.674% | 1.598 ms | 32.912 ms | 34.412 ms | 28.118 | 96/199 |
| sitting | 12.245% | 1.698 ms | 33.103 ms | 34.844 ms | 28.318 | 110/199 |
| fr1/xyz | 16.388% | 1.993 ms | 33.438 ms | 36.749 ms | 28.077 | 91/199 |

G2 成本已经很低，但三次实际速度仍低于 30 FPS。不能把总帧时间全部归因于
几何，也不能依据 walking 的 active mean 小于 33.33 ms 宣称实时门控通过。

## 6. Proxy 审计

使用最宽松的 `C>=1,P>=1`：

| 序列 | comparison coverage | proxy precision | conditional recall | proxy/background rate |
| --- | ---: | ---: | ---: | ---: |
| walking | 11.674% | 22.20% | 61.67% | 11.12% |
| sitting | 12.245% | 9.77% | 15.03% | 7.23% |
| fr1/xyz | 16.388% | — | — | 4.58% |

person proxy 不是运动真值。walking 的条件召回只评价获得比较的约 11.67% 像素，
不能解释未覆盖区域。提高到 `C>=2,P>=2` 后覆盖降至约 1.24%，仍不能作为区域
mask。

## 7. 同参考 dense 审计

在 walking 上开启 `DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT=1`，只用于证据
对照，不用于实时性结论：

| 指标 | grid s4 | dense same-reference |
| --- | ---: | ---: |
| 五参考任意覆盖 | 11.646% | 48.884% |
| proxy precision | 23.33% | 25.07% |
| conditional recall | 63.44% | 69.10% |
| unconditional person capture | 2.11% | 16.61% |

在 grid 实际比较的位置：

- 100% 也有 dense comparison；
- 97.24% 的 grid positive 同时为 dense positive；
- grid 找回 dense positive 的 60.66%；
- positive presence 一致率为 91.03%。

这说明规则网格没有明显改变已采样位置的正残差含义，但大部分空间仍没有证据。

## 8. 门控

| 门控 | 结果 |
| --- | --- |
| 文献/工程身份明确 | 通过 |
| unknown 语义保持 | 通过 |
| 原 dense/orb-depth 默认路径保持 | 通过 |
| stride 形成稳定密度—成本曲线 | 通过 |
| 存在稳定 30 FPS 且高覆盖点 | 未通过 |
| 支持动态深度区域 | 未通过 |
| 支持直接 ORB 过滤 | 未证明 |

冻结结论：

```text
G2-2G = 已完成的中间密度负对照
grid s4 = 可保留为未来区域模块的采样/速度 baseline
G1-F = 不进入
G1-D = 不进入
```

## 9. 下一步

不再继续增加 stride 值或对网格做无边界膨胀。下一阶段应先从本地
DetectFusion/SInDSLAM 材料中冻结一个有边界的轻量区域表示 Shadow，并只回答：

> 能否用深度不连续性和表面一致性把图像分成受限区域，再在区域内聚合现有 G2
> 证据，而不重复 G0-3 的链式传播？

完整法线分割、平面提取或重聚类不能一次全部加入；仍需拆成最小区域生成和区域
证据审计两步。

## 10. 产物

- `G2_2G_GRID_DEPTH_SAMPLING_SHADOW_SPEC.md`
- `G2_2G_GRID_DEPTH_SAMPLING_SHADOW_RESULT.md`
- `TUM1_GeometryGridDepthSamplingShadow.yaml`
- `TUM3_GeometryGridDepthSamplingShadow.yaml`
- `walking_grid_s{2,4,8}_online_{histogram,selection}.csv`
- `sitting_grid_s4_online_{histogram,selection}.csv`
- `fr1_xyz_grid_s4_online_{histogram,selection}.csv`
- `walking_grid_s4_same_reference_{histogram,selection}.csv`
- `audit_grid_s4_online/`
