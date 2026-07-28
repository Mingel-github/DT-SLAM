# G2-3R0 深度边界区域划分 Shadow 结果

日期：2026-07-28

## 1. 结论

```text
区域拓扑：初步通过
动态判定：没有实施，也没有通过
同步实时门：未通过
G1-F / G1-D：继续锁定
```

SInDSLAM 深度不连续公式的轻量连通域适配，明显避免了 G0-3 中
`95%–97%` valid comparison 被单个 all-seed flood fill 并集直接吞并的结构。
代表帧中，人物、显示器、桌椅等具有明显深度轮廓的区域大多能被边界分开。

但本轮不能表述为“动态检测有效”：

- 区域生成完全没有读取 residual 或 semantic mask；
- 单个最大区域仍占 valid depth 的约 `45%–75%`；
- 前五个区域合计占约 `86%–96%`；
- TUM3 中约 `86%` 的区域不超过 64 像素，存在明显碎片化；
- 全分辨率 CPU 划分耗时约 `3.0–3.4 ms/frame`，当前同步预算容不下。

因此下一步只能在 Shadow 中研究“区域内证据聚合”和降低分区成本，不能采用：

```text
区域内出现任意一个 positive seed
→ 整个区域判动态
```

## 2. 方法身份

本地论文核对结果：

```text
Qi et al., T-ITS 2025:
δ_depth(u) = max |D(u_neighbour)-D(u)|
edge iff δ_depth > max(τ1 D(u), τ2)
Table II: τ1=0.025, τ2=0.08 m
```

当前实现使用同一边界形式和原文参数起点，再对有效、非边界像素做四邻域
connected components。

它不包含原论文的：

- 三维 K-means 初始聚类；
- Gaussian pyramid coarse-to-fine K-means；
- 平面边缘；
- 深度直方图重聚类；
- 光流 residual 动态判定。

所以其身份为：

> `[A] SInDSLAM depth-boundary-constrained lightweight connected-component
> partition`

而不是 SInDSLAM reproduction。

DetectFusion 的法线与距离不连续联合分割只完成了文献核对，没有混入当前实现。

## 3. 实现边界

新增纯函数：

```cpp
GeometricDynamicDetector::PartitionDepthByDiscontinuity(...)
```

输出约定：

```text
labels == -1: invalid depth / unknown
labels == -2: depth boundary
labels >=  0: region id
```

新增独立工具：

```text
Examples/RGB-D/depth_region_partition_audit
```

该工具只读 TUM depth PNG，不启动 System、Tracking、YOLO、Viewer 或
Optimizer。生成区域标签、CSV 统计和可选 RGB overlay；不会改变轨迹。

## 4. 确定性测试

已覆盖：

- 统一深度平面保持为一个区域；
- 1 m / 2 m 深度阶跃将两侧切分；
- 深度不连续像素保持独立 boundary 状态；
- 零深度屏障保持 unknown，并切断连通区域。

结果：

```text
[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0 Test] PASS
```

## 5. 199 帧拓扑审计

统一参数：

```text
τ_rel = 0.025
τ_abs = 0.08 m
```

| 序列 | region mean / p50 | boundary/valid mean | largest/valid mean / p50 / p95 | top-5/valid mean | partition ms mean / p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| fr3_walking_xyz | 254.23 / 139 | 5.09% | 47.58% / 42.09% / 78.38% | 86.46% | 3.00 / 3.37 |
| fr3_sitting_static | 155.55 / 161 | 4.67% | 45.10% / 45.74% / 60.08% | 89.74% | 3.16 / 3.46 |
| fr1_xyz | 14.81 / 15 | 1.40% | 75.16% / 78.43% / 91.92% | 95.53% | 3.38 / 3.61 |

小区域统计：

| 序列 | `<=64 px` region 占 region 数 | singleton 占 region 数 |
| --- | ---: | ---: |
| fr3_walking_xyz | 86.45% | 37.79% |
| fr3_sitting_static | 86.64% | 42.11% |
| fr1_xyz | 26.90% | 14.39% |

解释：

1. `fr1_xyz` 的桌面等连续静态表面形成大区域是合理现象，不能把“大区域”本身
   解释为失败或动态。
2. TUM3 人物场景中边界更多、区域更碎；代表帧可视化显示人物轮廓通常能从背景
   切开，但仅靠目视不能证明动态 precision/recall。
3. top-5 覆盖很高，证明“任意 seed 触发整区”仍会非常危险。后续必须使用
   区域内 valid coverage、positive/consistent 证据比例和最小支持数。
4. 当前 CPU 全分辨率实现约 3 ms；即使不计后续证据聚合，也超过当前同步
   semantic baseline 的约 1 ms 余量。

代表可视化：

- `walking_debug/frame_000120_region_overlay.png`
- `sitting_debug/frame_000120_region_overlay.png`
- `fr1_debug/frame_000120_region_overlay.png`

红色为 depth boundary；区域颜色仅表示 label，不表示动态类别。

## 6. 可复现命令

构建与测试：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM
cmake --build build --target geometric_warp_test \
  depth_region_partition_audit -j4

export LD_LIBRARY_PATH=/home/zhu/dynaslam_ws/pangolin_install/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/thirdparty/onnxruntime/lib:\
${LD_LIBRARY_PATH:-}

./Examples/RGB-D/geometric_warp_test
```

walking 199 帧：

```bash
./Examples/RGB-D/depth_region_partition_audit \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt \
  /home/zhu/dynaslam_ws/results/g2_3r0_2026-07-28/fr3_walking_xyz_199.csv \
  199 0.025 0.08 \
  /home/zhu/dynaslam_ws/results/g2_3r0_2026-07-28/walking_debug 30
```

## 7. 决策

批准：

- 保留 G2-3R0 作为区域拓扑 Shadow；
- 保留 invalid、boundary、region 的分离；
- 后续仅在区域内部聚合几何 evidence；
- 使用当前实现作为全分辨率 CPU 成本基线。

不批准：

- 把任何 region label 当成 dynamic；
- 将区域结果写入 `mvbDynamic`；
- 用单 seed 触发整区；
- 声称复现 SInDSLAM 或 DetectFusion；
- 在 3 ms 成本未处理前接入同步实时 Tracking。

下一项应先冻结一个更小的 G2-3R1 Shadow 规格：

```text
固定 G2-3R0 region labels
+ G2-1/G2-2G comparison/positive/consistent counts
→ 只输出每区域证据直方图
→ 不选动态阈值
```

同时必须把计算路径分开计时；若区域聚合后总成本继续超过预算，应利用原文已有的
coarse-to-fine/pyramid 思路研究低分辨率区域表示，而不是直接进入 G1。
