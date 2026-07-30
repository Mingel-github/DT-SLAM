# G2-4E 连续 Development Geometry Evidence 成对审计结果

日期：2026-07-29

## 1. 结论

G2-4E 完成了完整连续序列、真实 C++ person mask、exact C++ depth partition
和独立 RGB-only target-box 粗框的成对审计。

结果不支持从当前 region-level evidence 直接构造动态判决：

1. 无人物箱子帧中，bbox 主导 region 的正证据与整帧全局正证据几乎相同；
2. nonobstructing box 经常与大面积背景属于同一个 depth region；
3. obstructing 的 target-absent 帧全局正证据不低于无人物 target-visible 帧；
4. 强正证据主要集中在人物搬运/遮挡箱子的帧，而人物已经由语义分支覆盖；
5. 当前 visibility/bbox 不是 motion/pixel GT，不能据此选阈值。

```text
dynamic_decision = none
direct_slam_state_mutation = none
G1-F = locked
G1-D = locked
```

这不是对 depth-warp 直接像素证据的最终否定。它说明当前“固定 depth region
内聚合 evidence”在未知箱子上缺少可靠对象隔离，并且现有标签不足以区分
moving/static。

## 2. Strict hold-out 已封存

正式封存：

```text
BONN/rgbd_bonn_balloon_tracking.zip
size = 325919363 bytes
sha256 = 3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
```

封存前只依据 archive 名称、当前 person-only 语义边界和本地使用记录选择。
没有：

- 解压；
- 查看图像；
- 生成 association/contact sheet；
- 运行 YOLO、SLAM 或 geometry；
- 使用算法表现。

约束见：

- `STRICT_HOLDOUT_MANIFEST.md`
- `strict_holdout.sha256`

本阶段没有解封 hold-out。

## 3. 完整连续 Development 运行

两条序列都使用：

- 完整一对一 association；
- 原始时间顺序；
- Bonn RGB/depth 联合 rectification；
- online CUDA YOLO；
- person mask age 0；
- G2-3R3 `pyramid_dense_s2`；
- G2-4A risk instrumentation；
- shadow-only。

| 指标 | nonobstructing | obstructing |
| --- | ---: | ---: |
| association frames | 778 | 589 |
| person mask ready | 778/778 | 589/589 |
| mask age median/max | 0/0 | 0/0 |
| region CSV rows | 9,149 | 8,074 |
| frames with region evidence | 737 | 520 |
| active total mean | 41.107 ms | 38.156 ms |
| actual FPS | 24.109 | 25.750 |
| deadline misses | 737/778 | 504/589 |

性能只代表开启离线 risk instrumentation 的 shadow 配置，不代表最终系统。

输出：

```text
results/g2_4e_2026-07-29/development_full_runs/
```

其中 `DT_SLAM_GEOMETRY_MULTIREF_CSV` 和
`DT_SLAM_GEOMETRY_REGION_EVIDENCE_CSV` 分开保存。此前 G2-4B 结果文档中的
复现模板曾把 multiref 变量误写成 region 输出，现已修正；历史原始结果未改。

## 4. Exact partition 连接

新增：

- `DT-SLAM/Examples/RGB-D/box_region_partition_audit.cc`
- `DT-SLAM/tools/audit_bonn_box_region_evidence.py`

离线连接重新读取相同 association 和 raw RGB-D，经过相同
`RGBDInputRectifier` 和 `DepthMapFactor`，调用同一个：

```cpp
GeometricDynamicDetector::PartitionDepthByDiscontinuity()
```

在线/离线检查：

| Sequence | candidates | online geometry available | exact partition matches | mismatches |
| --- | ---: | ---: | ---: | ---: |
| nonobstructing | 24 | 23 | 23 | 0 |
| obstructing | 24 | 19 | 19 | 0 |

缺失：

```text
nonobstructing: frame 16
obstructing:    frames 6, 25, 35, 40, 46
```

这些是序列早期尚无可用多参考 region evidence 的帧，必须解释为
`no online geometry evidence`，不能解释为静态。

## 5. 无人物箱子 evidence

以下均为 development-only 描述统计。

### 5.1 moving_nonobstructing_box

可用的 `target visible + person absent` 帧：18。

| 指标 | 中位数 |
| --- | ---: |
| frame positive/comparison | 2.224% |
| dominant bbox-region positive/comparison | 2.227% |
| dominant positive vote ratio | 0.939% |
| dominant minus frame positive ratio | -0.0003 个百分点附近 |
| bbox intersecting region count | 1 |
| bbox assigned to dominant region | 93.52% |
| dominant region 位于 bbox 内的比例 | 15.02% |
| multi-reference comparison fraction | 85.75% |
| unanimous-positive/comparison | 0.232% |
| positive pixels in boundary d2 band | 30.36% |
| positive pixels in invalid-depth d2 band | 28.91% |

解释：

- 粗框的大部分像素落在一个主导 region；
- 但该 region 只有约 15% 位于粗框内，约 85% 延伸到 bbox 外；
- 主导 region 正证据与整帧全局正证据几乎相同；
- 这符合“箱子与桌面/背景被同一 depth-connected region 合并”，而不是
  box-local motion region。

boundary 与 invalid 两个 d2 band 可以重叠，两个百分比不得相加。

### 5.2 moving_obstructing_box

可用的 `target visible + person absent` 帧：9。

| 指标 | 中位数 |
| --- | ---: |
| frame positive/comparison | 2.623% |
| dominant bbox-region positive/comparison | 2.590% |
| dominant positive vote ratio | 0.780% |
| dominant minus frame positive ratio | 0.000% 附近 |
| bbox intersecting region count | 3 |
| bbox assigned-region ratio | 87.15% |
| dominant region 位于 bbox 内的比例 | 44.44% |
| multi-reference comparison fraction | 86.91% |
| unanimous-positive/comparison | 0.089% |

obstructing 的 depth topology 比 nonobstructing 更碎，粗框内 region 数量更多；
主导 region 对箱子的隔离稍好，但正证据仍基本等于整帧全局值。

## 6. Target absent 对照

obstructing 中可用 target-absent 帧：8。

| Stratum | frame positive/comparison median |
| --- | ---: |
| target visible, person absent | 2.623% |
| target absent | 4.148% |

target-absent 的中位数反而更高。因此当前 frame/region positive ratio 不具备
直接的 target-visible separability。

该比较仍受以下限制：

- 候选由 geometry proxy 条件化选择；
- target visible 不是 target moving；
- 样本只有 9 对 8；
- nonobstructing 没有 absent 候选。

所以它只能否决“当前数值已明显可分”，不能估计泛化 AUC/FPR。

## 7. Person-present 条件

| Sequence | person absent dominant positive ratio median | person present median |
| --- | ---: | ---: |
| nonobstructing | 2.227% | 15.067% |
| obstructing | 2.590% | 66.273% |

人物搬运/遮挡时信号明显增强。obstructing 的 person-present 只有 2 帧，不能
作稳定统计，但方向与可视化一致。

这并不是目标成果，因为这些人物已经被 semantic branch 覆盖。当前方法需要
证明的是：

```text
person absent + unknown object independently moving
```

条件下仍能提供可靠证据；G2-4E 没有证明这一点。

## 8. 风险解释

### 已确定

- exact C++ person 和 geometry frame 对齐；
- 在线/离线 partition 完全一致；
- 当前 depth regions 对 nonobstructing box 隔离不足；
- person-present 是明显混杂因素；
- 当前 score 不应进入 G1。

### 仍未知

- 18/9 个 person-absent 箱子候选中，哪些帧箱子真的在运动；
- direct per-pixel evidence 在 bbox 内是否比 region aggregate 更有效；
- ORB feature 上是否存在高精度小集合；
- 独立 pixel mask 下的真实 box coverage；
- strict hold-out 表现。

## 9. 决策

G2-4E 判决门：

```text
person-absent stable evidence          = 未通过
target-visible vs absent separation    = 未通过
region object isolation                = nonobstructing 未通过
boundary/invalid independence          = 未通过或证据不足
multi-reference availability           = 通过
dynamic-decision readiness             = 未通过
```

因此：

- 不选择 region dynamic threshold；
- 不根据当前 48 帧拟合组合 score；
- 不运行 strict hold-out；
- 不进入 G1-F/G1-D；
- 不将人物驱动的高响应包装成 unknown-box 检测成功。

## 10. 下一步

下一步应先回到本地 PaperNotes/PDF 做有依据的方法审计，再决定最小增强：

1. 独立 motion-state 标签或 agent 自动预标注方案；
2. bbox/ORB feature 上的 direct geometry evidence，而不是先做 region
   threshold；
3. 是否采用 Ji 2021 式区域预分割/重投影对照；
4. 若横向运动 direct depth residual 天然弱，是否需要文献支持的稀疏
   ego-motion-compensated flow；
5. 只有 evidence 定义通过 development 门后，才冻结规则并解封 strict
   hold-out。

必须优先使用本地 PaperNotes 和原始 PDF，不凭空增加组合阈值。

## 11. 验证与清理

完成以下回归：

```text
box_region_partition_audit build = PASS
semantic_review_export build     = PASS
geometric_warp_test              = PASS
select_bonn_review_frames self-test             = PASS
audit_bonn_semantic_box_coverage self-test       = PASS
audit_bonn_box_region_evidence self-test         = PASS
Python syntax compilation                         = PASS
git diff --check                                  = PASS
strict hold-out SHA256 check                      = PASS
```

在线/离线 exact partition 匹配和 CSV/JSON 结果保留在本阶段目录。验证后删除了：

- `/tmp` 中 749 MB 的可再生成 development archive 解压副本；
- 源目录中两个可由 CMake 重新生成的审计二进制。

没有删除原始 archive、源码、运行日志、CSV、JSON、轨迹或阶段报告。

## 12. 2026-07-29 Box 粗框修正后的复算

nonobstructing frame 246/478/537 的上游 RGB-only 粗框完成时序复核修正后，
已在新目录重建 exact C++ depth partition/bbox intersections，并与原在线
region evidence 重连：

```text
development_partition_v2_bbox_temporal_correction/
development_evidence_audit_v2_bbox_temporal_correction/
```

在线/离线 partition 仍为 `42/42` exact match、0 mismatch。无人物箱子
主导区域 positive/comparison 的均值与中位数不变
（`3.126%/2.227%`），因为修正前后仍主要命中同一大型背景连通区域；
主导区域 bbox coverage 均值由 `93.803%` 变为 `92.533%`，bbox 相交区域数
均值由 `1.333` 变为 `1.500`。

因此 G2-4E 的负门控不变。旧目录不删除，当前有效修正说明见
`results/g2_4f_2026-07-29/G2_4D_TO_F0_BBOX_TEMPORAL_CORRECTION_RESULT.md`。
