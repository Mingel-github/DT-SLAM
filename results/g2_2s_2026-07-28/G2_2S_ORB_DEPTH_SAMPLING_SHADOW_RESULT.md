# G2-2S ORB 特征关联深度采样 Shadow 结果

日期：2026-07-28

## 1. 结论

G2-2S 已完成实现、确定性测试、三个 TUM 短序列的在线 CUDA 成对实验和同帧同
参考证据审计。

结论是：

```text
ORB-depth sampling
= 显著有效的稀疏计算基线
≠ 足以生成动态深度区域的像素级几何主方法
≠ 已批准的动态特征过滤器
```

它将 K=5 几何计算从约 `14–19 ms` 降至约 `0.82–1.09 ms`，但将整图任意
比较覆盖从约 `49%–72%` 降至约 `0.47%–0.91%`。进一步隔离旧 G0 稠密
shadow 后，同步语义＋G2-2S 达到约 `28.20–28.72 FPS`，仍没有稳定达到
30 FPS。

## 2. 实现及来源边界

实现：

```text
关键帧 Frame::mvKeys 原始像素
→ 去重
→ 只保留语义清理后仍有有效深度的像素
→ 共视参考选择保持不变
→ 仅投影这些 feature-associated depth samples
→ 稀疏 z-buffer
→ positive / negative / consistent 原始计数
```

来源是 DynaSLAM 公开的 feature-associated depth observation 思路。本地
`DynaSLAM/src/Geometry.cc:117–144` 明确遍历参考帧 ORB keypoint 并读取其
深度。

当前实现是适配而非复现，因为没有实现 DynaSLAM 的：

- 参考排序；
- parallax gate；
- 当前深度 patch 搜索；
- depth variance gate；
- region growing；
- 原论文阈值。

NGD-SLAM 的 15×15 网格属于动态 mask 光流传播，SInDSLAM 的每 5 帧属于地图
精修频率，均未被错误移植为本阶段的深度采样依据。

## 3. 工程验证

构建：

```text
make geometric_warp_test rgbd_tum -j$(nproc)
```

测试：

```text
[Geometry G0/G2-1/G2-2R/G2-2S Test] PASS
```

确定性测试确认：

- 未采样像素保持 unknown；
- sampled positive/negative 符号正确；
- per-reference sample/comparison 统计正确；
- 参考选择复制时保留 ORB-depth sample；
- 原 dense 路径保持默认且原测试继续通过。

在线实验均使用：

- RTX 4060 Ti；
- ONNX Runtime `CUDAExecutionProvider`；
- K=5；
- `covisibility`；
- viewer 关闭；
- 前 199 帧；
- `mask ready=199/199`；
- `mask age median/max=0/0`。

未修改 YOLO、Optimizer、g2o 或后端，没有新增 PoseOptimization，没有修改
Tracking/Mapping 状态。

## 4. 独立在线运行时间

这组实验用于比较真实端到端成本。由于 ORB-SLAM2 LocalMapping 异步运行，不同
速度会改变关键帧生成时序，因此证据保持另用第 6 节的同参考审计。

| 序列 | dense G2 mean | orb-depth G2 mean | G2 加速 | dense active mean | orb-depth active mean | dense FPS | orb-depth FPS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 14.494 ms | 0.930 ms | 15.59× | 49.071 ms | 36.752 ms | 20.065 | 26.212 |
| sitting | 14.758 ms | 0.823 ms | 17.94× | 49.198 ms | 36.040 ms | 20.060 | 26.913 |
| fr1/xyz | 19.365 ms | 1.086 ms | 17.82× | 53.879 ms | 37.220 ms | 18.336 | 26.142 |

事实：

- 稀疏采样本身的计算下降超过一个数量级；
- 上表的两种运行都仍包含旧 G0 单参考帧 dense shadow，因此适合做受控
  dense/orb-depth 对比，不适合判断 G2-2S 独立部署后的总速度；
- 剩余时间不能全部归因于几何，包含同步语义等待、G0 dense shadow、Tracking
  和图像读取。

### 4.1 G2-2S 隔离计时

新增默认开启的 `Geometry.SingleReferenceShadowEnable`，仅在两份 G2-2S
专用配置中设为 `0`。这不会改变任何旧配置；本组只关闭 G0 计算，仍保留在线
CUDA 语义、G2-2S、Tracking、LocalMapping 和数据读取。

| 序列 | G2 mean | G2 median | active mean | active median | active p95 | actual FPS | deadline missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 0.779 ms | 0.760 ms | 32.464 ms | 32.503 ms | 35.265 ms | 28.204 | 79/199 |
| sitting | 0.866 ms | 0.864 ms | 32.005 ms | 32.107 ms | 33.223 ms | 28.721 | 66/199 |
| fr1/xyz | 1.005 ms | 1.007 ms | 32.451 ms | 32.488 ms | 35.328 ms | 28.443 | 63/199 |

三次均为 `mask ready=199/199`、`mask age median/max=0/0`。可确认：

- K=5 sparse G2 自身只占约 `0.78–1.00 ms`；
- active mean/median 已低于单帧 `33.33 ms`，但 p95 在 walking 和 fr1/xyz
  超出预算；
- `actual_fps` 仍只有 `28.20–28.72`，且有 `63–79/199` 帧出现 deadline
  overrun；
- 因此“几何成本门控”通过，但“稳定 30 FPS 端到端门控”仍未通过，不能依据
  均值宣称已达到 30 FPS。

## 5. 空间覆盖代价

下面是每个参考帧的均值，以及五参考聚合后的整图任意比较覆盖。

| 序列 | dense有效样本/参考 | orb-depth有效样本/参考 | dense单参考比较覆盖 | orb-depth单参考比较覆盖 | dense五参考任意覆盖 | orb-depth五参考任意覆盖 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 143290 | 384 | 40.165% | 0.109% | 48.661% | 0.474% |
| sitting | 153294 | 465 | 41.959% | 0.134% | 49.933% | 0.571% |
| fr1/xyz | 227737 | 755 | 55.330% | 0.201% | 71.855% | 0.907% |

约 `99%` 的 dense comparison coverage 在 ORB-depth 模式下变为 unknown。
这些无比较像素没有被解释为静态。

## 6. 同帧、同位姿、同参考审计

跨独立运行比较时，完全相同参考集合的帧只有约 `4%–18%`，原因是不同运行速度
会影响异步 LocalMapping 的关键帧时序。因此增加了仅由以下变量开启的诊断：

```text
DT_SLAM_GEOMETRY_DENSE_SAMPLING_AUDIT=1
```

它在同一帧、同一位姿和同一组参考上同时计算 sparse 和 dense。该模式的端到端
时间不用于实时性结论。

### 6.1 正残差存在性

以 dense 的 `positive_count>0` 作为同位置内部对照：

| 序列 | sparse比较像素也有dense比较 | sparse positive中也是dense positive | dense positive在sparse中的召回 | positive存在性总一致 |
| --- | ---: | ---: | ---: | ---: |
| walking | 100.0% | 91.31% | 54.35% | 86.63% |
| sitting | 100.0% | 84.81% | 53.74% | 89.59% |
| fr1/xyz | 100.0% | 92.68% | 64.08% | 95.38% |

解释：

- sparse positive 大多也得到 dense positive 支持；
- 但 sparse 只找回 dense 在这些位置产生的约 `54%–64%` positive；
- exact comparison/positive vote count 一致率很低，因为 dense 模式会由大量
  额外参考深度样本向同一像素增加比较票；
- 因此 dense 的 C/P 阈值不能直接搬到 orb-depth 模式。

### 6.2 Person proxy

使用同参考在线审计的 `C>=1,P>=1`，避免高票规则在稀疏模式下几乎没有覆盖：

| 模式 | walking覆盖 | proxy precision | conditional recall | unconditional capture | fr1静态背景率 | sitting proxy外率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| dense | 48.711% | 22.59% | 66.47% | 15.17% | 7.59% | 14.53% |
| orb-depth | 0.470% | 20.75% | 60.69% | 0.108% | 8.42% | 12.34% |

条件指标看起来接近，是因为它们只评价极少数获得 sparse 比较的像素。无条件
person proxy 捕获从 `15.17%` 降到 `0.108%`，才反映完整像素域的信息损失。

## 7. 门控判断

| 门控 | 结果 |
| --- | --- |
| 文献来源和适配边界明确 | 通过 |
| dense默认路径不变 | 通过 |
| 稀疏路径确定性和unknown语义 | 通过 |
| K=5几何显著提速 | 通过 |
| G2-2S自身约1 ms | 通过 |
| 同步系统稳定达到30 FPS | 未通过（28.20–28.72 FPS） |
| 保持像素级覆盖 | 未通过 |
| 支持动态深度区域输出 | 未通过 |
| 支持直接ORB过滤 | 未证明 |

冻结结论：

```text
G2-2S = 已完成的轻量 sparse evidence baseline
G1-F  = 不因本结果自动解锁
G1-D  = 明确不能由本结果解锁
```

## 8. 下一步边界

当前不能：

- 用条件 precision/recall 掩盖约 99% 的无覆盖区域；
- 对 sparse count 继续套用 dense C/P 阈值；
- 从 sparse seed 直接恢复完整 mask；
- 重新启用已经失败的无限 flood fill；
- 因为几何本身约 1 ms 就宣称系统达到 30 FPS。

下一步需要先复核以下二选一：

1. 若优先保护稀疏 SLAM：研究 sparse evidence 与当前 ORB feature 的对应关系，
   仍先做 shadow；
2. 若优先输出动态深度区域：必须研究具有空间覆盖的中间密度或区域表示，
   ORB-depth sampling 只能作为种子和速度对照。

在决定前不自动进入 MAD、时序投票或实际过滤，因为这些步骤不能恢复不存在的
空间观测。

## 9. 产物

- `G2_2S_ORB_DEPTH_SAMPLING_SHADOW_SPEC.md`
- `G2_2S_ORB_DEPTH_SAMPLING_SHADOW_RESULT.md`
- `TUM1_GeometryOrbDepthSamplingShadow.yaml`
- `TUM3_GeometryOrbDepthSamplingShadow.yaml`
- `*_dense_online_{histogram,selection}.csv`
- `*_orb_depth_online_{histogram,selection}.csv`
- `*_orb_depth_g2_only_online_{histogram,selection}.csv`
- `*_orb_depth_g2_only_online.log`
- `*_same_reference_audit_histogram.csv`
- `*_agreement_geometry_selection.csv`
- `audit_dense_online/`
- `audit_orb_depth_online/`
- `audit_same_reference_*/`
