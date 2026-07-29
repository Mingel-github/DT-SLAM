# G2-3R2 dense-vs-grid 区域覆盖上限审计结果

日期：2026-07-29

## 1. 结论

```text
同区域 dense-vs-grid 成对审计：通过
stride-4 覆盖瓶颈：确认
dense 实时性：未通过
dense 直接动态判决：不批准
G1-F / G1-D：继续锁定
```

三个序列均显示，stride-4 只保留 dense 约 23%–25% 的 comparison 像素。
dense 证明当前多参考深度几何在大量区域中可以形成比较，但其 CPU 成本过高，
并且会同时增加动态区域信号和静态背景 positive。因此不能把“更多 dense
positive”直接解释为更准确的动态检测。

## 2. 与原计划和文献的关系

当前工作没有偏离最初冻结目标：

```text
RGB-D + 初始位姿
→ 类别无关几何不一致证据
→ 经验证后才允许动态特征/深度过滤
```

原计划中的 G2-Robust 包含多参考和区域评分。当前把它拆为参考选择、采样、
区域表示、区域聚合与覆盖审计，是为了分别验证正确性和实时性，不是增加新的
后端或对象级任务。

来源边界：

- DynaSLAM：历史深度投影、多参考深度一致性 `[L/A]`；
- DetectFusion：运动证据与几何区域重叠 `[L/A]`；
- SInDSLAM：区域内证据比例及深度边界 `[L/A]`；
- 同帧、同参考、同位姿、同区域下的 dense/grid 成对控制：实验设计 `[S]`。

当前不是以上任何系统的复现。没有引入其 TSDF、ICP residual、光流、对象地图、
双遍稠密配准或后端优化。

相对初始顺序的唯一实质调整是：

```text
暂缓 G1 过滤和 G2 的 MAD/时序
先解决 seed、region 和 coverage 是否可靠
```

这是由 G0-3 区域扩张失败和 G2-3R1 覆盖不足触发的风险门控，不是研究目标漂移。

## 3. 实验控制

三个序列各在线运行约 199 帧：

```text
walking_xyz
sitting_static
fr1/xyz
```

共同设置：

```text
synchronous YOLOv8-seg CUDA
最多 5 个 covisibility references
grid_depth stride=4
dense same-reference audit
residual threshold=0.10 m
region τ_rel=0.025, τ_abs=0.08 m
同一 current depth / Tcw / K / region labels
Geometry shadow-only
```

三次都是：

```text
mask ready = 199/199
mask age median/max = 0/0
```

只有能够获得完整 5 个缓存参考帧的帧进入成对审计：

| 序列 | 配对帧 | 配对区域行 |
| --- | ---: | ---: |
| walking | 171 | 37,412 |
| sitting | 186 | 30,371 |
| fr1/xyz | 181 | 2,672 |

所有成对行的 `region_label`、`region_pixels`、semantic proxy pixels、有效深度
和区域总数完全一致，没有结构错配。

## 4. 区域 comparison 覆盖

| 序列 | grid coverage | dense coverage | grid/dense 像素保留 |
| --- | ---: | ---: | ---: |
| walking | 18.27% | 77.70% | 23.51% |
| sitting | 18.17% | 73.25% | 24.81% |
| fr1/xyz | 21.90% | 94.90% | 23.08% |

结论：

1. dense 具有显著更高的区域比较覆盖；
2. stride-4 的主要作用不是近似保存全部 dense evidence，而是用约四分之一
   comparison coverage 换取低成本；
3. fr1/xyz dense 背景覆盖接近完整，说明该序列中参考深度和位姿足以产生大量
   几何比较；
4. walking/sitting 的 dense coverage 仍低于 fr1/xyz，说明动态语义区域从参考
   深度中清除、遮挡、新视野和参考内容也构成独立上限。

## 5. semantic proxy 与背景

semantic proxy comparison coverage：

| 序列 | grid | dense | grid/dense 保留 |
| --- | ---: | ---: | ---: |
| walking | 3.74% | 27.17% | 13.77% |
| sitting | 2.82% | 20.43% | 13.79% |
| fr1/xyz | 1.39% | 11.46% | 12.10% |

在获得比较的 semantic proxy 像素中，positive 比例为：

| 序列 | grid | dense |
| --- | ---: | ---: |
| walking | 63.18% | 68.05% |
| sitting | 13.52% | 15.83% |
| fr1/xyz | 23.42% | 27.36% |

非 semantic proxy 背景的 positive 比例为：

| 序列 | grid | dense |
| --- | ---: | ---: |
| walking | 9.37% | 17.26% |
| sitting | 6.59% | 13.46% |
| fr1/xyz | 4.44% | 7.29% |

解释：

- dense 明显增加 walking person proxy 的有效覆盖，并保留强 positive 信号；
- dense 同时使所有序列的背景 positive 增加，说明位姿误差、深度边界、遮挡和
  传感器噪声也被更充分采样；
- fr1/xyz 的 semantic proxy 不是运动 GT，不能用其 positive 比例评价动态
  precision；
- grid 的 positive pixel count 只保留 dense 的约 12.6%–14.1%；
- `count retention` 不是 motion recall，以上结果仍不能批准动态阈值。

## 6. 区域支持缺失

在 dense 已有 comparison 的区域中，grid 完全无 comparison 的比例：

```text
walking: 45.12%
sitting: 58.16%
fr1/xyz: 17.75%
```

在 dense 已有 positive 的区域中，grid 完全无 positive 的比例：

```text
walking: 53.94%
sitting: 58.83%
fr1/xyz: 17.24%
```

这确认了 stride-4 会遗漏大量小区域或稀疏区域支持，尤其在 walking/sitting
人物场景中更明显。

## 7. 成本与 FPS

| 序列 | grid G2 mean / p95 | dense G2 mean / p95 | partition mean | grid/dense region aggregation mean | actual FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 1.55 / 1.70 ms | 14.00 / 15.35 ms | 2.91 ms | 0.51 / 0.59 ms | 20.27 |
| sitting | 1.67 / 1.74 ms | 14.53 / 15.04 ms | 3.09 ms | 0.56 / 0.59 ms | 19.77 |
| fr1/xyz | 1.89 / 2.01 ms | 18.88 / 20.10 ms | 3.24 ms | 0.62 / 0.58 ms | 17.93 |

dense G2 比 stride-4 约慢 9–10 倍。成对审计同时计算两套 evidence，FPS 用于
说明审计成本，不代表最终只能达到该速度；但直接采用当前 full-resolution dense
实现显然不能满足 30 FPS 目标。

deadline miss：

```text
walking: 166/199
sitting: 181/199
fr1/xyz: 179/199
```

## 8. 验证

```text
geometric_warp_test: PASS
rgbd_tum build: PASS
audit_region_dense_vs_grid.py py_compile: PASS
三序列 region structure mismatch: 0
git diff --check: PASS
```

新增审计工具：

```text
DT-SLAM/tools/audit_region_dense_vs_grid.py
```

## 9. 决策

批准：

- G2-3R2 作为 dense coverage upper-bound 审计；
- “stride-4 是当前区域覆盖的主要瓶颈之一”这一结论；
- 保留 dense 作为离线/短序列上限对照。

不批准：

- 在线直接使用当前 dense 五参考 warp；
- 根据 dense positive 直接过滤特征或区域；
- 把 semantic proxy 当运动 GT；
- 调整动态阈值；
- G1-F 或 G1-D。

下一步应研究有文献与当前结果支持的轻量高覆盖表示，而不是继续调 stride：

```text
优先候选：
低分辨率 dense / Gaussian-pyramid evidence shadow

依据：
SInDSLAM 使用 Gaussian pyramid 进行 coarse-to-fine 深度区域处理；
标准图像金字塔是成熟的计算降采样手段。
```

进入下一实现前必须先冻结：

- 深度降采样方式；
- 相机内参缩放；
- z-buffer 和 mask 的坐标域；
- 低分辨率 evidence 回投当前区域的方式；
- 相对 full-resolution dense 的覆盖保真；
- 独立运行成本，不能同时在线保留 full dense。
