# G2-1 多参考深度一致性 Shadow 结果

日期：2026-07-28

## 1. 结论

G2-1 的 geometry-only 审计证明了两件事：

1. 最近 5 个关键帧的重复正残差证据，比 G0 的单相邻帧直接 seed 更集中于
   walking 序列中的 person proxy；
2. 当前“5 次全分辨率稠密 forward warp”实现明显不满足 30 FPS。

RTX 4060 Ti 恢复后补做的在线同步语义实验又证明：

3. 语义 mask 清理参考深度会减少参与 warp 的像素，使 G2-1 降到约
   14–19 ms，但同步语义＋G2-1 的端到端速度仍只有约 19–21 FPS；
4. 语义清理会显著改变多参考几何的有效覆盖域。高 `comparison_count`
   会优先排除缺少历史参考覆盖的人物像素，因此不能把条件召回单独解释成整体检测
   能力提升。

因此本阶段结论是：

> 多参考一致性值得保留为后续轻量化几何证据的依据，但当前稠密同步实现不能
> 进入实际 SLAM 过滤，也不适合作为最终实时主路径；下一步必须同时解决计算量、
> 参考有效性和“无证据不等于静态”三个问题。

这不是 DynaSLAM 复现。它是“最近关键帧＋稠密 depth warp＋逐像素计数”的
shadow adaptation。

## 2. 实现和安全边界

新增的 G2-1 只输出：

```text
comparison_count
positive_count
negative_count
consistent_count
```

并保存逐帧二维计数直方图。没有二值动态判断。

确定性测试验证：

```text
positive + negative + consistent == comparison
```

构建与测试结果：

```text
geometric_warp_test: PASS
rgbd_tum: build PASS
```

未执行：

- 未写入 `Frame::mvbDynamic`；
- 未删除 `mvpMapPoints`；
- 未新增 `PoseOptimization`；
- 未影响 MapPoint 创建；
- 未修改 YOLO、Optimizer、g2o 或后端。

## 3. 数据与代理标签

每个序列输入 199 帧（关联文件首行注释占一行）：

| 数据 | 用途 | geometry-only G2-1有效帧 | 在线语义 G2-1有效帧 |
| --- | --- | ---: | ---: |
| TUM fr3/walking_xyz | 动态序列 | 188 | 186 |
| TUM fr1/xyz | 真正静态负样本 | 181 | 181 |
| TUM fr3/sitting_static | 同相机低动态诊断 | 186 | 186 |

G2-1 等到缓存中存在 5 个成功关键帧后才输出，因此有效帧少于输入帧。

### 3.1 首轮 geometry-only 审计

首轮普通受限运行环境无法访问 RTX 4060 Ti，因此该轮没有在线启用 YOLO。代理标签
复用了此前已经保存并核对过的 827 帧离线 person mask：

```text
results/g0_2a_dynamic_2026-07-28/offline_person_proxy
```

逐帧 `comparison_count/positive_count` 原始图与代理 mask 离线相交。工具逐帧验证
了原始图像直方图与 C++ CSV 的 pixel count 完全一致，共验证 188 帧。

person mask 只表示“人物区域代理”，不是运动真值，也不能评价未知动态箱子。

### 3.2 在线同步语义闭环

驱动修复后，在主机侧验证：

```text
GPU: NVIDIA GeForce RTX 4060 Ti
driver: 595.84
CUDA provider: CUDAExecutionProvider, device 0
```

随后对三个序列分别运行 199 帧在线 YOLOv8n-seg＋G2-1。三次运行均满足：

```text
mask ready: 199/199
mask age: median=0, max=0
```

因此这三次结果是当前帧同步语义 mask，不再是离线代理回放。在线 person mask
仍只是语义人物区域代理，不是像素级运动真值。

## 4. 证据质量

候选规则仅在离线审计中枚举：

```text
comparison_count >= C
positive_count >= P
```

C++ 中没有写入这些阈值。

代表结果：

| C | P | walking proxy precision | conditional recall | walking背景率 | fr1静态选择率 | sitting选择率 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 1 | 34.41% | 23.77% | 18.93% | 7.06% | 13.13% |
| 2 | 2 | 38.54% | 16.82% | 11.11% | 2.65% | 7.15% |
| 3 | 2 | 39.02% | 17.43% | 11.19% | 2.89% | 7.22% |
| 3 | 3 | 38.61% | 10.21% | 6.67% | 1.45% | 4.22% |
| 5 | 2 | 40.46% | 17.60% | 10.53% | 2.41% | 6.68% |
| 5 | 3 | 39.84% | 10.74% | 6.59% | 1.42% | 4.14% |
| 5 | 5 | 29.62% | 1.78% | 1.72% | 0.26% | 0.76% |

客观解释：

- 从 1 票提高到 2 票，静态选择率明显下降，proxy precision 上升；
- 继续要求 3–5 票时，静态选择率继续下降，但 proxy recall 很快下降；
- 全票一致不是可用工作点：虽然静态率低，但只保留约 1.78% 的条件代理召回；
- `C=5,P=2` 一类规则存在一定折中，但 17.60% 的代理召回不足以批准实际过滤；
- sitting 的选择率系统性高于 fr1，部分像素可能来自人物轻微运动，也可能来自
  长时间跨度参考、深度噪声或遮挡变化；当前数据不能把这些原因分开。

与 G0 的约 4.26% 像素级 proxy precision 相比，G2-1 的证据集中度明显提高。
但两者参考策略和有效覆盖域不同，因此该比较只能说明趋势，不能当成严格的同条件
算法提升。

## 5. 在线语义与多参考几何的交互

在线审计继续使用：

```text
comparison_count >= C
positive_count >= P
```

但由于参考关键帧中的语义动态深度已被置为无效，人物区域经常没有足够历史参考
覆盖。必须同时报告：

- `semantic coverage`：满足比较次数要求的人物代理像素占全部人物代理像素比例；
- `conditional recall`：只在具备足够比较次数的代理像素中计算的召回；
- `unconditional capture`：最终选中的代理像素占全部代理像素比例。

walking 在线结果：

| C | P | proxy precision | semantic coverage | conditional recall | unconditional capture | fr1静态选择率 | sitting选择率 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 1 | 20.84% | 20.85% | 63.79% | 13.30% | 10.71% | 13.72% |
| 2 | 2 | 22.42% | 13.90% | 61.36% | 8.53% | 3.49% | 7.44% |
| 3 | 3 | 22.13% | 9.05% | 54.92% | 4.97% | 1.75% | 3.91% |
| 5 | 2 | 5.03% | 1.94% | 52.57% | 1.02% | 3.10% | 6.99% |
| 5 | 3 | 7.01% | 1.94% | 48.56% | 0.94% | 1.71% | 3.74% |
| 5 | 5 | 12.75% | 1.94% | 25.50% | 0.49% | 0.27% | 0.69% |

客观解释：

- 在线条件召回较高，是在仅约 2%–21% 人物代理像素具备足够参考覆盖的条件下得到；
- `C=5` 时 semantic coverage 只剩 1.94%，不能把剩余约 98% 无覆盖人物像素解释
  为静态；
- semantic reference cleaning 改变了几何证据的抽样域，geometry-only 与在线结果
  不能只比较 precision/conditional recall；
- 当前简单计数聚合没有形成可靠的语义＋几何融合规则，不批准任何在线 `C/P`
  阈值进入过滤。

这进一步支持 G0-2 已冻结的三状态原则：

```text
有动态证据 / 有静态一致证据 / 没有足够几何证据
```

## 6. 实时性

### 6.1 Geometry-only

G2-1 自身耗时：

| 序列 | mean | median | p95 |
| --- | ---: | ---: | ---: |
| walking | 20.18 ms | 20.38 ms | 23.37 ms |
| fr1/xyz | 23.18 ms | 23.18 ms | 25.12 ms |
| sitting | 21.66 ms | 21.60 ms | 23.09 ms |

端到端：

| 序列 | active mean | actual FPS | deadline missed |
| --- | ---: | ---: | ---: |
| walking | 46.46 ms | 20.96 | 181/199 |
| fr1/xyz | 46.04 ms | 20.97 | 178/199 |
| sitting | 43.72 ms | 22.17 | 181/199 |

这些运行同时保留了 G0 单参考诊断，因此存在约 3 ms 的重复计算。即使后续复用
其中一次 warp，G2-1 当前 20–23 ms 的主体代价仍然没有足够的 30 FPS 余量。

### 6.2 在线同步语义＋G2-1

| 序列 | YOLO steady median | G2-1 mean | G2-1 median | active mean | actual FPS | deadline missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 12.53 ms | 14.02 ms | 14.02 ms | 47.48 ms | 20.70 | 187/199 |
| sitting | 12.52 ms | 14.11 ms | 14.18 ms | 47.38 ms | 20.78 | 186/199 |
| fr1/xyz | 11.89 ms | 18.80 ms | 18.90 ms | 51.85 ms | 19.02 | 189/199 |

在线 G2-1 比 geometry-only 快，主要事实是参与参考 warp 的有效深度像素减少；
当前数据不能把全部差值归因于单一因素。即使如此，端到端仍明确未达到 30 FPS，
所以“当前 5 参考全分辨率稠密实现不进入实时主路径”的结论不变。

## 7. 新发现的参考策略风险

sitting 日志中曾长时间出现：

```text
current frame 42..120
newest selected keyframe = 33
```

这说明“最近 5 个关键帧”并不等于“时间上接近当前帧”；ORB-SLAM2 在一段时间内
可能不创建新关键帧。较老参考既能增加基线，也会增加遮挡、显露和深度视角差异。

因此下一阶段不能直接把关键帧投票次数解释为时序确认。未来参考选择至少需要明确
约束：

- 与当前帧的视角重叠；
- 时间/帧号跨度；
- 有效深度覆盖率；
- 不能把无覆盖解释为静态反对票。

上述约束需要文献依据和独立消融，不能在当前结果后临时补一个年龄阈值。

在线实验又增加一个风险：

- 语义清理后的关键帧对人物区域缺少历史覆盖；
- 比较次数门槛既是“证据充分度”，也是“参考覆盖筛选”；
- 直接提高 `comparison_count` 会造成选择偏差，不能被解释为更强的静态投票。

## 8. 阶段决定

批准保留：

- 多参考 raw evidence count 接口；
- 5 参考结果作为文献启发的 shadow 对照；
- 计数图、直方图和自动代理审计工具。

不批准：

- 当前任一 `C/P` 规则写入 `mvbDynamic`；
- 当前 5 次稠密 warp 进入同步实时主路径；
- 使用 person proxy 证明未知动态物体检测有效；
- 把“最近 5 个关键帧”称为 DynaSLAM 的 overlap selection；
- 现在加入区域生长或第三次位姿优化。

下一步应先设计 G2-2 的轻量证据采样/参考选择 shadow，而不是继续调投票阈值。
G2-2 必须分别记录参考覆盖和残差证据，不能用缺少比较次数充当静态反对票。

## 9. 产物

- `G2_1_MULTI_REFERENCE_SHADOW_SPEC.md`
- `fr1_xyz_200_histogram.csv`
- `walking_200_histogram_raw.csv`
- `walking_200_histogram_proxy.csv`
- `sitting_200_histogram.csv`
- `walking_counts/`
- `audit_with_sitting/g2_1_vote_grid.csv`
- `audit_with_sitting/g2_1_vote_grid.json`
- `walking_200_online_semantic_histogram.csv`
- `sitting_200_online_semantic_histogram.csv`
- `fr1_xyz_200_online_semantic_histogram.csv`
- `walking_200_online_semantic.log`
- `sitting_200_online_semantic.log`
- `fr1_xyz_200_online_semantic.log`
- `audit_online_semantic_all/g2_1_vote_grid.csv`
- `audit_online_semantic_all/g2_1_vote_grid.json`
- `DT-SLAM/tools/audit_multireference_evidence.py`
- `DT-SLAM/tools/attach_proxy_to_multireference_histogram.py`
