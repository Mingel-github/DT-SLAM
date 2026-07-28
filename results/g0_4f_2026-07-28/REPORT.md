# G0-4F：ORB feature-level shadow evidence

日期：2026-07-28

## 1. 目的与非目标

G0-4F 检查 G0-2 的 direct positive seed 在 ORB 特征层面是否比像素层面更适合
用于前端过滤。

本阶段只测量：

- ORB 特征中心是否具有有效几何比较；
- ORB 特征中心或其小邻域是否接近 positive seed；
- 候选特征与 person proxy 的重合；
- 真正静态序列中的候选率底线。

本阶段不做：

- 不写 `Frame::mvbDynamic`；
- 不清除 `mvpMapPoints`；
- 不调用额外 PoseOptimization；
- 不修改 Optimizer、YOLO、g2o 或地图写入；
- 不选择最终窗口半径；
- 不把 person proxy 当作运动真值。

## 2. 测量定义

特征坐标使用 `Frame::mvKeys`，与 raw registered RGB/depth/mask 像素域一致。
对半径 `r = 0, 1, 2, 3`：

- `eligible feature`：特征中心位于 `validComparisonMask` 的 Chebyshev
  半径 `r` 内；
- `candidate feature`：特征中心位于 `positiveSeedMask` 的 Chebyshev
  半径 `r` 内；
- `semantic feature`：特征中心落入 person proxy。

实现上使用方形结构元对 valid/positive mask 做对应半径的膨胀，然后只读取
ORB 特征中心。不同半径构成诊断曲线，不代表已选择某个检测阈值。

对 person proxy 非空的动态帧，统计：

```text
semantic eligible coverage = semantic eligible / semantic features
proxy precision            = candidates inside person / candidates
conditional recall         = candidates inside person / semantic eligible
proxy-background rate      = candidates outside person /
                             eligible outside person
```

`proxy-background rate` 不是严格静态 FPR，因为 person mask 外仍可能存在漏检人物
或其他动态/遮挡区域。

## 3. 实现与开关

新增可选环境变量：

```bash
DT_SLAM_GEOMETRY_FEATURE_CSV=/path/to/feature_shadow.csv
```

只有设置该变量时才运行四个半径的特征统计。CSV 使用 long format，每帧分别记录
SLAM pose 和可用的 GT pose。未提供语义 mask 时仍可记录静态候选率，语义相关列为
零。

输出列包括：

```text
frame, reference, timestamp, pose_source, radius_px,
features, semantic_features, eligible, semantic_eligible,
candidates, candidates_inside_semantic, candidates_outside_semantic,
eligible_coverage, semantic_eligible_coverage,
proxy_precision, conditional_recall, proxy_background_rate
```

## 4. TUM `fr3_walking_xyz` 动态代理结果

数据、person proxy 和一对一 association 与 G0-2A-dynamic 完全相同。以下为
SLAM pose、person proxy 非空的 677 帧加权统计：

| 半径 | semantic eligible coverage | proxy precision | conditional recall | proxy-background rate |
| ---: | ---: | ---: | ---: | ---: |
| 0 px | 5.328% | 7.090% | 15.281% | 7.561% |
| 1 px | 7.947% | 6.738% | 21.922% | 15.693% |
| 2 px | 11.420% | 6.880% | 24.654% | 23.392% |
| 3 px | 15.391% | 7.276% | 26.389% | 30.235% |

GT pose 对应结果：

| 半径 | semantic eligible coverage | proxy precision | conditional recall | proxy-background rate |
| ---: | ---: | ---: | ---: | ---: |
| 0 px | 5.672% | 7.500% | 17.115% | 8.482% |
| 1 px | 8.325% | 7.227% | 23.982% | 16.736% |
| 2 px | 11.744% | 7.476% | 27.324% | 24.468% |
| 3 px | 15.653% | 7.891% | 29.171% | 31.281% |

解释：

1. 特征中心的 proxy precision 比像素级的 4.26% 略高，但 7.09% 仍远不足以
   支持直接过滤；
2. 增大邻域可提高 conditional recall，但 proxy-background rate 增长更快；
3. proxy precision 在 0–3 px 间基本不升，说明简单扩大窗口没有改善判别性；
4. GT pose 只带来小幅变化，没有改变结论；
5. person 特征的有效比较覆盖依然很低，原因之一是成功参考帧保存前会将语义区域
   深度置零；无几何覆盖的特征仍必须保持 unknown。

### 动态运行性能

| 指标 | 数值 |
| --- | ---: |
| 帧数 | 827 |
| tracking mean | 17.998 ms |
| active total mean | 27.367 ms |
| actual FPS | 28.558 |
| deadline misses | 4/827 |

本次关闭调试图写入，同时运行 SLAM/GT 两套 geometry 和四个 feature 半径。由于
没有单独对 feature statistic 计时，不能从端到端差值精确归因其开销。

## 5. TUM `fr1/xyz` 静态负样本结果

使用 G0-2A-static 已验证的：

- 792 对一对一 RGB-depth association；
- Tracking 原生 TUM1 非零畸变模型；
- raw registered geometry 的零畸变 `525/525/319.5/239.5` 模型；
- 0.10 m positive residual threshold；
- region growing 关闭。

791 个 SLAM-pose 相邻帧比较的加权候选率：

| 半径 | eligible / all features | candidates / eligible |
| ---: | ---: | ---: |
| 0 px | 81.376% | 1.108% |
| 1 px | 86.652% | 2.721% |
| 2 px | 89.665% | 4.533% |
| 3 px | 91.727% | 6.479% |

GT pose 的 `candidates / eligible` 分别为：

```text
r0=1.397%, r1=3.087%, r2=4.943%, r3=6.890%
```

动态序列的 proxy 外候选率明显高于静态底线，说明动态/遮挡场景确实增加了几何
不一致；但这些不一致没有被可靠定位到 person proxy 内。

### 静态运行性能

| 指标 | 数值 |
| --- | ---: |
| 帧数 | 792 |
| tracking mean | 21.645 ms |
| active total mean | 30.002 ms |
| actual FPS | 29.541 |
| deadline misses | 61/792 |

## 6. 验证

```text
rgbd_tum: build PASS
geometric_warp_test: [Geometry G0-3R Test] PASS
git diff --check: PASS
```

编译仅出现工程已有的 ONNX Runtime C++17 和 Eigen deprecated warnings。

## 7. 冻结结论

G0-4F shadow 诊断完成，但没有通过进入 G1-F 的门控。

```text
G0-4F = 已完成的负结果
G1-F  = 不进入
G1-D  = 不进入
G0-3B/C = 继续暂缓
```

核心原因不是“没有找到一个合适的像素窗口”，而是：

> 当前单参考帧 direct positive seed 对 ORB 特征也缺乏足够的空间判别性；
> 扩大邻域主要增加背景候选，而不是提高 precision。

因此不应继续调整窗口、面积或 seed-ratio 阈值来强行进入过滤。下一项应回到有
明确文献身份的独立对照：Ji 2021 的 depth clustering + cluster reprojection
error（GJ baseline），或者先重新设计比单帧 direct seed 更强的 seed 门控。两者
不能与当前失败的 flood fill 混成一个实现。

## 8. 产物

- `walking_feature_shadow.csv`
- `walking_feature_shadow.log`
- `fr1_xyz_feature_shadow.csv`
- `fr1_xyz_feature_shadow.log`

