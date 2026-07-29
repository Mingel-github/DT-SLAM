# G2-3R3 二分辨率 dense evidence Shadow 结果

日期：2026-07-29

## 1. 结论

```text
scale-2 depth-pyramid evidence：实现与验证通过
区域 comparison coverage 保真：通过
positive presence 近似：初步通过
exact vote 等价：未通过
同步 30 FPS：未通过
动态判决 / G1-F / G1-D：继续锁定
```

G2-3R3 用 boundary-preserving 320x240 depth pyramid 运行 5 参考 dense warp，
再把 cell evidence 最近邻扩展到 640x480 区域域。它在三序列中保留 full dense
约 99% 的区域 comparison 数量，候选耗时约 4.9–6.1 ms；但逐像素 exact vote
agreement 只有 52%–77%，且包含同步语义、区域划分后的候选-only 系统速度仅
24.3–25.0 FPS。

因此它可以保留为区域级轻量证据候选，不能解释为 full dense 的逐像素等价版本，
也不能进入动态过滤。

## 2. 文献边界

本阶段实现前重新核对了本地 PaperNotes、SInDSLAM/DetectFusion PDF，并补查
KinectFusion 原始论文。

准确归属：

- KinectFusion `[L/A]`：三层 depth/vertex/normal pyramid；block averaging
  后 2x subsampling；只平均与中心深度足够接近的值，以避免跨深度边界平滑；
  多尺度用于 dense ICP。
- SInDSLAM `[L]`：depth Gaussian pyramid 用于加速 K-means clustering，
  不是低分辨率动态 residual。
- DetectFusion `[L]`：三层 coarse-to-fine pyramid 用于相机 tracking，
  不是 motion mask approximation。
- G2-3R3 `[S/H]`：用 scale-2 depth warp 和 nearest cell expansion 近似
  full-resolution region evidence，是本项目的工程假设。

当前没有 TSDF、ICP、vertex/normal map、光流、coarse-to-fine 位姿优化或对象
地图，因此不属于上述系统复现。

## 3. 实现

新增：

```text
boundary-preserving 2x depth downsample
scaled K
cached reference half depth
current half depth
half-resolution dense multi-reference warp
nearest evidence cell expansion
full-resolution region aggregation
same-reference full dense audit
```

深度 block 仅平均：

```text
|d - anchor| <= max(0.025 * anchor, 0.08m)
```

无效 anchor 保持无效。reference half depth 只在关键帧进入缓存时计算一次；当前
half depth 每个证据帧计算一次。

expanded 2x2 cell 中的四个像素共享低分辨率 evidence，不能视为四个独立深度
测量。

## 4. 区域 coverage

| 序列 | pyramid coverage | full dense coverage | pyramid/dense 数量保留 |
| --- | ---: | ---: | ---: |
| walking | 76.94% | 77.40% | 99.40% |
| sitting | 73.09% | 73.52% | 99.42% |
| fr1/xyz | 94.78% | 95.41% | 99.34% |

与 G2-3R2 stride-4 的约 18%–22% coverage 相比，scale-2 pyramid 基本恢复了
full dense 的区域 comparison 数量。

但这是 cell expansion 后的区域覆盖近似；不能把 99% 数量保留写成 99% 独立
像素测量召回率。

## 5. semantic proxy 与背景

semantic proxy comparison coverage：

| 序列 | pyramid | dense |
| --- | ---: | ---: |
| walking | 25.15% | 25.33% |
| sitting | 20.38% | 20.33% |
| fr1/xyz | 10.63% | 10.96% |

semantic proxy 内 positive / comparison：

| 序列 | pyramid | dense |
| --- | ---: | ---: |
| walking | 66.67% | 66.81% |
| sitting | 16.05% | 15.30% |
| fr1/xyz | 30.07% | 28.46% |

非 semantic proxy 背景 positive / comparison：

| 序列 | pyramid | dense |
| --- | ---: | ---: |
| walking | 17.74% | 17.94% |
| sitting | 12.80% | 13.08% |
| fr1/xyz | 7.36% | 7.31% |

区域加权统计非常接近 full dense，但 person semantic mask 仍只是 proxy；fr1 的
非零 semantic mask 也再次说明它不是运动 GT。

## 6. 同像素一致性

现有 same-reference audit 在所有 pyramid comparison 像素上检查 full dense：

| 指标 | walking | sitting | fr1/xyz |
| --- | ---: | ---: | ---: |
| pyramid comparison 也被 dense 比较 | 98.50% | 98.43% | 98.68% |
| pyramid positive 得到 dense positive 支持 | 88.06% | 83.87% | 87.02% |
| dense positive 在 pyramid comparison 域被恢复 | 89.43% | 84.38% | 88.13% |
| positive presence agreement | 94.80% | 95.43% | 98.10% |
| exact comparison/positive vote agreement | 69.62% | 76.98% | 52.22% |

解释：

1. comparison presence 和 positive presence 近似较好；
2. exact vote agreement 明显不足，尤其是 fr1/xyz；
3. block averaging、半分辨率 rasterization 和 nearest expansion 会改变每个
   像素获得的具体参考票数；
4. 因此第一版只允许使用区域统计，不允许逐像素 vote 阈值或特征过滤。

## 7. 候选成本

| 序列 | pyramid total mean / p95 | preprocess mean / p95 | expansion mean / p95 | full dense mean / p95 |
| --- | ---: | ---: | ---: | ---: |
| walking | 4.92 / 5.41 ms | 0.35 / 0.38 ms | 0.78 / 0.82 ms | 14.25 / 15.49 ms |
| sitting | 5.08 / 6.01 ms | 0.36 / 0.38 ms | 0.82 / 1.26 ms | 14.57 / 15.28 ms |
| fr1/xyz | 6.07 / 6.33 ms | 0.38 / 0.39 ms | 0.76 / 0.79 ms | 18.96 / 20.25 ms |

full dense 比 pyramid 慢约 2.9–3.1 倍。full dense 只在上限审计中运行。

全分辨率区域划分仍为约 `2.93–3.26 ms`，区域聚合约 `0.59–0.62 ms`。

## 8. Candidate-only FPS

关闭 full dense audit，只保留：

```text
synchronous semantic
+ pyramid evidence
+ full-resolution region partition
+ one region aggregation
```

结果：

| 序列 | active total mean / median / p95 | deadline miss | actual FPS |
| --- | ---: | ---: | ---: |
| walking | 38.60 / 39.68 / 41.14 ms | 169/199 | 24.99 |
| sitting | 39.00 / 39.50 / 40.59 ms | 182/199 | 24.96 |
| fr1/xyz | 39.99 / 40.63 / 43.89 ms | 179/199 | 24.30 |

因此：

```text
coverage gate = 通过
30 FPS gate   = 未通过
```

不能用 full-dense audit 运行时的 16.8–18.7 FPS 代表候选速度，也不能把
candidate-only 的约 25 FPS 宣称为 30 FPS。

## 9. 验证

```text
[.../G2-3R3 Test] PASS
rgbd_tum build: PASS
audit script py_compile: PASS
三序列 region structure mismatch: 0
mask ready: 199/199
mask age median/max: 0/0
git diff --check: PASS
```

确定性测试覆盖：

- invalid anchor 保持无效；
- boundary-aware block average；
- scale-2 相机内参；
- identity pose comparison；
- expansion 后 vote 守恒。

## 10. 决策

批准：

- `pyramid_dense_s2` 作为区域级 high-coverage Shadow 候选；
- 保留 full dense 作为短序列上限审计；
- 后续只研究区域级支持，不使用 exact pixel vote。

不批准：

- G1-F / G1-D；
- 动态区域阈值；
- 把 expanded cell 当独立像素证据；
- 当前同步路径满足 30 FPS；
- 继续增加第三、第四种采样而没有明确失败目标。

下一步应先处理当前已明确的两个问题：

1. full-resolution region partition 约 3 ms；
2. pyramid evidence + region 在同步 semantic pipeline 中使 active total 达到
   约 39–40 ms。

合理顺序：

```text
G2-3R4:
同一 scale-2 pyramid 上生成低分辨率 region representation，
再投票到 full-resolution region ID 或特征域，
测量边界/区域保真和节省的约束成本。
```

即使省掉全部 3 ms region partition，active total 仍可能高于 33.3 ms。因此
G2-3R4 之后必须基于实测决定：

- 进一步做 CPU 数据遍历优化；
- 或重新讨论 semantic/geometry 调度；
- 而不是预设同步 30 FPS 一定成立。
