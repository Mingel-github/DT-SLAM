# G2-3R1 区域内多参考证据聚合 Shadow 结果

日期：2026-07-28

## 1. 结论

```text
区域证据统计链路：通过
walking person-proxy 信号：存在
区域动态判决：未通过、未实施
同步 30 FPS：未通过
G1-F / G1-D：继续锁定
```

G2-3R1 成功将 G2-2G 的 comparison/positive/negative/consistent 计数聚合到
G2-3R0 固定区域，并保持：

```text
unknown != static
boundary != static
positive/negative/consistent presence 可重叠
所有 vote 严格守恒
```

walking 的 person semantic proxy 在“获得几何比较”的像素中有较强 positive
信号；但 person proxy 只有 `3.75%` 像素获得比较。区域级区分能力中等且明显
依赖最小支持量。因此不能把少量 positive 扩展到整个区域，也不能选择动态阈值。

## 2. 文献边界

本阶段只借：

- DetectFusion：motion residual mask 与 geometry segments 做区域重叠；
- SInDSLAM：将 residual 类别限制在 cluster 内，并统计 dynamic pixels /
  cluster pixels。

当前 residual 来源、区域生成、参考选择与两篇论文不同。本阶段没有复制：

- DetectFusion 的 TSDF/surfel ICP residual 与 K=2 residual clustering；
- SInDSLAM 的 optical flow residual、Triangle 双阈值、cluster flood fill；
- 两者的动态阈值或二次位姿优化。

所以当前身份为：

> `[A] fixed depth-boundary region evidence aggregation`

## 3. 测试设置

三个序列均在线运行 199 帧：

```text
同步 YOLOv8-seg CUDA
5 个 covisibility references
grid_depth stride=4
residual threshold=0.10 m
region τ_rel=0.025, τ_abs=0.08 m
region evidence shadow enabled
viewer disabled
```

三次均为：

```text
semantic mask ready = 199/199
mask age median/max = 0/0
```

## 4. 像素加权证据结果

| 序列 | 全区域 comparison coverage | positive / compared | semantic-proxy coverage | semantic positive / compared | nonsemantic positive / compared |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 18.60% | 12.85% | 3.75% | 63.42% | 10.12% |
| sitting | 18.23% | 6.69% | 2.79% | 13.03% | 6.38% |
| fr1/xyz | 21.60% | 4.79% | 1.16% | 21.76% | 4.77% |

解释：

1. walking person proxy 的 conditional positive 比例明显较高，说明区域统计没有
   消除原有动态证据。
2. walking person proxy 覆盖仅 `3.75%`。其余 `96.25%` 仍是“没有当前几何
   比较”，不能解释为 static。
3. sitting 中 person 是混合静动对象，其较低 positive 比例符合该场景属性，但
   person proxy 本身不是运动 GT。
4. fr1/xyz 中 semantic proxy 仍非零，说明 YOLO mask 不是严格静态 GT；
   真静态负对照应优先看 nonsemantic `4.77%`。
5. walking 非语义区域也有 `10.12%` positive，可能包含未知动态、人物边界、
   位姿/深度误差或 proxy 漏检，当前数据不能将其拆开。

## 5. 区域级无阈值诊断

使用 `positive_vote_ratio` 对：

```text
walking/sitting 中 semantic overlap >= 50% 的区域
vs
fr1/xyz 中 semantic overlap == 0 的区域
```

做 rank AUC。这里的面积与支持量下限只是诊断分层，不是方法门控：

| 最小 region pixels | 最小 compared pixels | walking vs static AUC | sitting vs static AUC |
| ---: | ---: | ---: | ---: |
| 1 | 1 | 0.647 | 0.621 |
| 65 | 5 | 0.714 | 0.658 |
| 65 | 20 | 0.740 | 0.641 |
| 256 | 20 | 0.775 | 0.666 |

结论：

- 有足够区域支持时 walking 信号增强；
- 但 AUC 仍未达到可以无条件整区过滤的程度；
- AUC 随支持量门槛显著变化，说明“小区域和稀疏比较”是关键混淆因素；
- 不选择 positive ratio、区域面积或 comparison 数阈值。

## 6. 运行时间

| 序列 | G2 evidence mean / p95 | partition mean / p95 | region aggregation mean / p95 | active total mean | actual FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 1.60 / 1.81 ms | 3.00 / 3.40 ms | 0.535 / 0.595 ms | 35.89 ms | 26.55 |
| sitting | 1.72 / 1.82 ms | 3.16 / 3.40 ms | 0.568 / 0.613 ms | 37.53 ms | 25.90 |
| fr1/xyz | 1.91 / 2.10 ms | 3.33 / 3.47 ms | 0.642 / 0.695 ms | 36.70 ms | 26.34 |

deadline misses：

```text
walking 152/199
sitting 182/199
fr1/xyz 163/199
```

区域聚合本身低于 0.7 ms，但全分辨率分区约 3 ms，是当前主要新增成本。完整
G2-3R1 同步路径明确不满足 30 FPS。

## 7. 验证

```text
[Geometry G0/G2-1/G2-2R/G2-2S/G2-2G/G2-3R0/G2-3R1 Test] PASS
git diff --check: PASS
```

测试验证：

- boundary 和 invalid label 不进入区域统计；
- 同一像素不同参考的多类 presence 可以重叠；
- positive + negative + consistent vote 等于 comparison vote；
- semantic proxy 内覆盖和 positive 支持单独计数；
- 没有 dynamic mask 或 SLAM state mutation。

审计脚本：

```text
DT-SLAM/tools/audit_region_evidence.py
```

## 8. 决策

批准：

- G2-3R1 作为逐区域 evidence histogram Shadow；
- 保留逐区域 coverage、presence ratio 和 vote ratio；
- 保留 semantic proxy 与 nonsemantic 的分离审计。

不批准：

- 整区动态阈值；
- 任意 seed / 任意 positive 触发整区；
- 将未覆盖像素视为静态；
- G1 Tracking 或 Mapping 过滤；
- 将 person semantic proxy 当真实运动 GT；
- 声称当前同步实现达到 30 FPS。

下一步不应调 region positive 阈值。应先处理两个独立瓶颈：

1. **区域成本**：依据 SInDSLAM 已有 Gaussian pyramid/coarse-to-fine 思路，
   建立低分辨率 region representation Shadow，并测量边界保真与耗时；
2. **证据覆盖**：对同帧区域比较 grid 与 dense 多参考 evidence 的区域覆盖上限，
   判断问题主要来自 stride-4 采样，还是语义清除后的参考本身没有动态区域深度。

在这两个问题被区分前，不进入动态过滤。
