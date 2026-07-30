# DT-SLAM 项目交接说明

更新时间：2026-07-29
工作区：`/home/zhu/dynaslam_ws`
源代码：`/home/zhu/dynaslam_ws/DT-SLAM`

> 本文是新 Codex 会话的单一交接入口，不替代代码、实验日志和阶段报告。若本文与本地代码或原始日志冲突，以本地可核查证据为准，并先记录差异，不得凭记忆猜测。

---

## 1. 项目目标与冻结边界

目标系统：

```text
ORB-SLAM2 RGB-D
+ YOLOv8-seg 已知动态语义分支
+ 类别无关几何分支
→ 动态 ORB 特征和动态深度区域过滤
```

研究目标：

- 语义分支处理已知动态类别；
- 几何分支独立于 YOLO，发现未知独立运动物体；
- 减少动态特征造成的位姿偏差；
- 减少动态深度或 MapPoint 写入静态地图造成的残影；
- 在 RTX 4060 Ti 上尽量接近 TUM RGB-D 的 30 FPS。

当前明确不做：

- 不做动态对象轨迹或对象级状态估计；
- 不做对象级 BA；
- 不修改 `Optimizer.cc`、g2o 边或后端优化目标；
- 不增加无条件的第三次 `PoseOptimization()`；
- 不实现 TSDF、OctoMap、3DGS 或完整动态地图；
- 不重构 ORB-SLAM2 后端线程；
- 不把几何 shadow 结果提前写入 `mvbDynamic`、`mvpMapPoints` 或地图。

本地工作区是权威版本，GitHub 仅作为备份。不能因远端分支缺少未提交实验就覆盖本地状态。

---

## 2. 仓库状态

当前分支：

```text
main
```

当前已推送提交：

```text
1a4bd8a Add geometry separability and Bonn coordinate audits
```

检查时 `HEAD` 与 `origin/main` 均为 `1a4bd8a`。G2-4C 选帧工具、时间戳/
hold-out 修正、G2-4D C++ person 导出/box coverage、G2-4E 连续 evidence
审计、G2-4F0 direct feature evidence、阶段文档及其原始结果仍在本地，尚未
提交或推送。

当前主要未提交文件：

```text
DT-SLAM/tools/select_bonn_review_frames.py
DT-SLAM/Examples/RGB-D/semantic_review_export.cc
DT-SLAM/Examples/RGB-D/box_region_partition_audit.cc
DT-SLAM/tools/audit_bonn_semantic_box_coverage.py
DT-SLAM/tools/audit_bonn_box_region_evidence.py
DT-SLAM/tools/audit_bonn_feature_evidence.py
DT-SLAM/tools/audit_bonn_sparse_ego_flow.py
DT-SLAM/tools/prepare_bonn_temporal_motion_review.py
DT-SLAM/tools/prepare_bonn_independent_review_candidates.py
DT-SLAM/Examples/RGB-D/BONN_GeometrySparseEgoFlowShadow.yaml
DT-SLAM/Examples/RGB-D/TUM3_GeometrySparseEgoFlowShadow.yaml
results/g2_4_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_SPEC.md
results/g2_4_2026-07-29/G2_4B_BONN_COORDINATE_DOMAIN_RESULT.md
results/g2_4c_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_RESULT.md
results/g2_4c_correction_2026-07-29/G2_4C_TIMESTAMP_AND_HOLDOUT_CORRECTION_RESULT.md
results/g2_4c_correction_2026-07-29/
results/g2_4_2026-07-29/G2_4_BONN_STATIC_MOVING_BOX_EVALUATION_PROTOCOL.md
results/DT-SLAM_几何模块阶段进度_2026-07-28.md
results/DT-SLAM_HANDOFF_2026-07-29.md
results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_SPEC.md
results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_RESULT.md
results/g2_4e_2026-07-29/
results/g2_4f_2026-07-29/
results/g2_4f1_2026-07-29/
results/g2_4f1_expansion_2026-07-29/
results/g2_4f1_development_data_2026-07-29/
```

注意：

- 不要清理、重置或覆盖这些本地改动；
- 原始日志和结果目录可能受 `.gitignore` 影响，`git status` 不一定列出其中全部文件；
- 提交前先核查每个结果目录，不要把大体积临时图像或无关日志盲目加入 Git。

---

## 3. 已冻结的语义基线

同步 YOLOv8-seg CUDA 语义基线已完成修复并冻结：

- 模型：`weights/yolov8n-seg.onnx`
- 正式运行使用 ONNX Runtime `CUDAExecutionProvider`
- 语义 mask 在正式短序列运行中为每帧就绪，`age=0`
- 当前没有启用异步语义架构
- 几何开发不得顺带修改 YOLO 推理、语义 mask 逻辑或 Optimizer

正式 ATE/FPS 报告：

```text
/home/zhu/dynaslam_ws/results/ate_semantic_baseline_2026-07-29/REPORT.md
```

### TUM fr3/walking_xyz，严格一对一关联，827 帧

三次运行的中位数：

| 模式 | ATE RMSE | 平移 RPE | 实际 FPS |
| --- | ---: | ---: | ---: |
| 无语义 baseline | 0.730574 m | 0.025407 m/frame | 约 28.542 |
| 同步语义 | 0.016477 m | 0.011876 m/frame | 约 28.345 |

语义相对 baseline：

- ATE 中位数改善约 97.74%；
- 平移 RPE 中位数改善约 53.26%；
- 语义三次均输出完整 827 帧轨迹；
- baseline 一次只输出 814 帧，须在论文统计中如实保留。

### TUM fr3/sitting_static，严格一对一关联，680 帧

三次运行的中位数：

| 模式 | ATE RMSE | 平移 RPE | 旋转 RPE |
| --- | ---: | ---: | ---: |
| 无语义 baseline | 0.008022 m | 0.005335 m/frame | 0.163444 deg/frame |
| 同步语义 | 0.006482 m | 0.005492 m/frame | 0.165418 deg/frame |

解释：

- ATE 改善约 19.2%；
- RPE 有约 1%–3% 的轻微退化；
- 两种模式轨迹均完整；
- `sitting_static` 不是“完全无动态物体”的真正静态场景。

当前结论仅证明语义基线有效。几何仍为 shadow 模式，因此现阶段不能声称几何改善了 ATE。

---

## 4. 几何研究阶段总览

| 阶段 | 内容 | 当前状态 |
| --- | --- | --- |
| G0-1 | 单参考帧 depth warp、z-buffer、signed residual | 已完成 |
| G0-2 | valid/consistent/positive/negative/unknown 分离 | 已完成 |
| G0-2C | RGB、depth、mask 坐标域与畸变检查 | 已完成 |
| G0-2P | SLAM pose 与 GT pose 敏感性对照 | 已完成 |
| G0-2A-static | TUM fr1/xyz 静态负样本审计 | 已完成 |
| G0-3 | 固定阈值 all-seed region growing | 已实验并判定失败，默认关闭 |
| G0-3R | 区域支持率修补 | 已实验但未解决，默认关闭 |
| G0-4F | ORB feature-level shadow evidence | 已做 proxy 审计，未达到过滤放行条件 |
| GJ | Ji 2021 K-means/簇级重投影文献 baseline | 已实现并审计，保持独立对照身份 |
| G2-1 | 多参考帧 shadow evidence | 已完成 |
| G2-2R | 轻量共视参考选择 | 已完成 |
| G2-2S | ORB-depth 稀疏采样 | 速度好、覆盖不足，不进入过滤 |
| G2-2G | stride-4 网格采样 | 成本和覆盖折中，但覆盖仍不足 |
| G2-3R0 | 深度边界固定区域划分 | 拓扑可用，运行成本未过门槛 |
| G2-3R1 | 固定区域证据聚合 | 已完成 shadow 审计，不设动态阈值 |
| G2-3R2 | full-dense 对 grid 的覆盖上界审计 | 已完成，确认 grid coverage 是瓶颈 |
| G2-3R3 | scale-2 深度金字塔 dense evidence | 已完成，覆盖通过、实时性未通过 |
| G2-3R4 | scale-2 低分辨率区域近似 | 最小实现/测试完成；端到端收益门失败，路线停止 |
| G2-4 | 动态/静态区分能力与风险代理 | G2-4A/B/C/D/E/F0/F1/F1D 完成；depth region/direct depth evidence 未过门，sparse ego-flow 在非 holdout 气球数据上通过方向性 evidence 门 |
| G2-4F1 | 稀疏 observed-flow 减 ego-flow | 实现、测试和同步语义全序列审计完成；约 2.5–2.7 ms，保持 shadow-only |
| G2-4F1D | 非 holdout 气球开发数据门与连续 evidence | 已完成；6 个可测 exact-zero person-overlap 帧中框内/背景 residual 中位 `11.919/0.723 px`；未选择阈值，未放行 G1 |
| G1-F | 几何特征真正参与 tracking/filtering | 未放行 |
| G1-D | 几何区域真正过滤深度或稠密写图 | 未放行 |

必须保留的负实验结论：

```text
局部深度连续 ≠ 同一运动对象。
```

从所有正残差 seed 沿局部深度连续性无限 flood fill，会命中大型深度连通分量并吞掉大部分画面。不得通过再增加一个随意面积阈值或 seed-ratio 阈值掩盖该失败。

---

## 5. 已冻结的主要证据表示：G2-3R3

规范与结果：

```text
/home/zhu/dynaslam_ws/results/g2_3r3_2026-07-29/G2_3R3_PYRAMID_DENSE_EVIDENCE_SHADOW_SPEC.md
/home/zhu/dynaslam_ws/results/g2_3r3_2026-07-29/G2_3R3_PYRAMID_DENSE_EVIDENCE_SHADOW_RESULT.md
```

实现要点：

- 对深度做 boundary-preserving 2× 下采样；
- 内参同步按 2× 缩放；
- 在 320×240 上运行 dense multi-reference warp；
- 将比较次数和正残差次数最近邻展开到 640×480 的区域域；
- 展开后的 2×2 像素只代表同一个低分辨率 cell，不能解释成四次独立测量；
- 仍保留 full-dense audit 作为上界对照。

下采样规则：

- 以 2×2 左上有效深度为 anchor；
- 只平均满足
  `abs(d - anchor) <= max(0.025 * anchor, 0.08 m)`
  的有效值；
- 无有效深度则保持无效。

文献身份：

- 深度边界保持金字塔原则受 KinectFusion 的多尺度深度处理启发；
- 当前 temporal multi-reference evidence 不是 KinectFusion 复现；
- 最近邻 evidence expansion 是本项目 `[S/H]` 工程假设，不是论文原方法。

### 覆盖结果：pyramid vs full dense

| 序列 | Pyramid comparison coverage | Dense comparison coverage | 计数保留率 |
| --- | ---: | ---: | ---: |
| walking | 76.94% | 77.40% | 99.40% |
| sitting | 73.09% | 73.52% | 99.42% |
| fr1/xyz | 94.78% | 95.41% | 99.34% |

像素存在性一致率约 94.80%–98.10%，但精确 vote count 一致率只有 52.22%–76.98%。因此：

- 覆盖门槛通过；
- 不能称为与 full dense 像素等价；
- 尚不能据此设动态过滤阈值。

### 时间结果

Pyramid geometry 总成本均值：

| 序列 | 均值 | P95 |
| --- | ---: | ---: |
| walking | 4.92 ms | 5.41 ms |
| sitting | 5.08 ms | 6.01 ms |
| fr1/xyz | 6.07 ms | 6.33 ms |

其中 full-resolution region partition 仍约 2.93–3.26 ms。

关闭 dense audit 后，语义 + pyramid + full-resolution region 的端到端 active frame：

| 序列 | active mean | active median | actual FPS |
| --- | ---: | ---: | ---: |
| walking | 38.596 ms | 39.677 ms | 24.995 |
| sitting | 38.997 ms | 39.497 ms | 24.958 |
| fr1/xyz | 39.992 ms | 40.629 ms | 24.299 |

结论：

- 相比 full dense，G2-3R3 大幅降低几何测量成本；
- 覆盖已接近 full dense；
- 30 FPS 门槛仍失败；
- 仍保持 shadow-only，不进入 G1-F/G1-D。

测试状态：

```text
[Geometry ... G2-3R3 Test] PASS
rgbd_tum build PASS
Python audit scripts py_compile PASS
region CSV mismatch = 0
git diff --check PASS
```

---

## 6. 文献来源账本

优先级固定为：

```text
本地 PaperNotes
→ 本地 PDF
→ 必要时查原始论文网页
```

不得把二手总结或当前工程假设写成论文原方法。

| 方法或组件 | 可依赖的来源 | 当前使用边界 |
| --- | --- | --- |
| 历史深度一致性、正深度差、区域扩展 | DynaSLAM, RA-L 2018 | 当前 single/dense/multi-reference 均为受启发改造，不是复现 |
| K=24 深度 K-means + cluster reprojection error | Ji et al., ICRA 2021 | 独立 GJ 文献 baseline，不是主实现 |
| motion evidence 与几何 segment 的重叠 | DetectFusion, ICRA 2018 | 只借区域证据聚合结构；无 TSDF/ICP |
| 深度边界形式与 cluster 内残差比例 | SInDSLAM, 2025 | 可作为区域化依据；其 Gaussian pyramid 只服务 K-means，不能替当前低分辨率 evidence 背书 |
| boundary-preserving depth pyramid | KinectFusion, ISMAR 2011 | 只支持多尺度深度预处理原则；当前时序动态证据为改造 |
| 相对深度 flood fill | ReFusion, IROS 2019 | 只能称 Algorithm 1 传播规则的适配；无 TSDF/residual/free-space，不能称复现 |
| z-buffer | 标准可见性处理 | 工程基础，不是动态 SLAM 创新 |

重要本地材料：

```text
/home/zhu/Desktop/papers/2018_DynaSLAM_Tracking_Mapping_Inpainting.pdf
/home/zhu/Desktop/papers/2019_DetectFusion_Known_Unknown_Dynamic_Objects.pdf
/home/zhu/Desktop/papers/Qi 等 - 2025 - Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering and Optical Flow Residuals.pdf
/home/zhu/Desktop/paper_notes/
```

每次引入新子方法，必须记录：

1. 原论文实际做了什么；
2. 本项目保留了什么；
3. 本项目删除或改变了什么；
4. 哪一部分只是 `[S/H]` 工程假设；
5. 需要什么实验才能放行。

---

## 7. 数据、关联文件与运行环境

数据集：

```text
/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz
/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_sitting_static
/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg1_xyz
```

严格一对一关联：

```text
/home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt
/home/zhu/dynaslam_ws/results/gj3a_2026-07-28/fr3_sitting_static_associations_one_to_one_20ms.txt
/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg1_xyz/associations_one_to_one_20ms.txt
```

注意：TUM3 的关联文件首行含注释，`head -n 200` 实际产生 199 帧。G2 正式短序列运行均应看到 `mask=199/199, age=0`。

GPU 运行环境：

```bash
ORT_CAPI=/home/zhu/.local/lib/python3.10/site-packages/onnxruntime/capi
NVIDIA_BASE=/home/zhu/.local/lib/python3.10/site-packages/nvidia
NVIDIA_LIBS=$(find "$NVIDIA_BASE" -mindepth 2 -maxdepth 2 -type d -name lib -printf '%p:')

export LD_PRELOAD="$ORT_CAPI/libonnxruntime.so.1.23.2"
export LD_LIBRARY_PATH="${ORT_CAPI}:${NVIDIA_LIBS}/home/zhu/dynaslam_ws/pangolin_install/lib:/home/zhu/dynaslam_ws/DT-SLAM/lib:${LD_LIBRARY_PATH:-}"
export DT_SLAM_DISABLE_VIEWER=1
```

当前系统已识别 RTX 4060 Ti，驱动、内核模块与用户态库均为 595.84，CUDA 13.2 可用。

Codex 受限执行环境中可能出现：

```text
no CUDA-capable device
```

这不等于主机 GPU 故障。在线 ONNX CUDA 实验应以允许访问 GPU 的本地/升级权限执行；任何 CUDA provider 加载失败的运行都必须标为无效，不能混入统计。

编译与测试：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
make geometric_warp_test rgbd_tum -j$(nproc)

export LD_LIBRARY_PATH="/home/zhu/dynaslam_ws/pangolin_install/lib:/home/zhu/dynaslam_ws/DT-SLAM/lib:/home/zhu/dynaslam_ws/DT-SLAM/thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
../Examples/RGB-D/geometric_warp_test
```

---

## 8. G2-3R4 至 G2-4F0 的推进与当前下一步

G2-3R4 当时冻结的目标是：

```text
G2-3R4：在同一个 scale-2 pyramid 域完成低分辨率深度区域表示，
        避免每帧约 3 ms 的 full-resolution region partition。
```

目标：

- 保留 G2-3R3 的低分辨率 dense evidence；
- 在 320×240 上生成区域或区域代理；
- 将区域证据映射回全分辨率 ORB/mask 域；
- 与 G2-3R3 full-resolution reference partition 做结构一致性和证据分配对照；
- 实测端到端 active time、deadline misses 和 actual FPS。

本地文献核对和设计报告在实现前已经完成：

```text
/home/zhu/dynaslam_ws/results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_LITERATURE_AUDIT.md
/home/zhu/dynaslam_ws/results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_SPEC.md
```

文献审计确认：

- `[L/A]` KinectFusion 只支持 boundary-preserving depth pyramid 组件；
- `[L]` SInDSLAM 的 Gaussian pyramid 用于 K-means 初始化；
- `[L]` DetectFusion 的 coarse-to-fine pyramid 用于 tracking；
- `[S/H]` scale-2 region partition、label/evidence 映射和替代有效性均为本项目假设。

SPEC 已明确：

- full-resolution partition 只是当前高分辨率参考实现，不是 object-region GT；
- region ID 不可直接比较，必须使用 label-permutation-invariant 指标；
- primary evidence audit 保持 native half-cell 计数，不把 2×2 expansion 当四票；
- static 相关指标只称 risk proxy，不称 dynamic FPR；
- G2-3R4 只回答能否在可接受结构损失下回收约 3 ms，不解决 30 FPS；
- 若结构或有界收益门失败，立即停止该路线；
- 即使通过，也先验证动态/静态区分能力，不默认继续 CPU 优化。

实现前评审按以下项目完成：

- KinectFusion 或其他本地原始论文对低分辨率深度分割/金字塔边界处理能支持到什么程度；
- 当前区域标签上采样属于哪一部分工程假设；
- 如何评价跨深度边界合并和区域碎裂；
- 是否能在不创建动态判决阈值的前提下完成 shadow 对照。

最低验收：

- build/test 通过；
- 关闭几何时行为不变；
- 仍是 shadow-only；
- 区域映射与统计可逐帧复核；
- 报告 boundary leakage、region fragmentation、comparison/positive evidence 保留率；
- 报告完整时延分解和 30 FPS 门槛；
- 若即使完全消除约 3 ms partition 仍明显超过 33.3 ms，应如实得出“需要 CPU 优化或重新研究 semantic/geometry 调度”，不能预设实时成立。

禁止在 G2-3R4 中顺带：

- 设定动态区域分类阈值；
- 修改 `mvbDynamic` 或过滤 `mvpMapPoints`；
- 改 YOLO；
- 改 Optimizer；
- 增加 PoseOptimization；
- 宣称 ATE 改善。

G2-3R4 最小实现和 candidate-only 计时已经完成。candidate region 独立成本门
通过，但端到端改善仅为 `1.600/0.287/1.403 ms`，三个序列中两个未达到预冻结
的 `1.5 ms`，因此 bounded benefit overall 失败。按停止条件不再扩展 paired
structure audit，也不继续 CPU 微调；所有未测结构指标保持未知。

详细结果：

```text
results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_RESULT.md
```

G2-3R4 不解锁 G1-F。G2-4A 已用 G2-3R3 region CSV 做初步只读区分审计：
walking 有条件 person-proxy 信号，但 proxy AUC 仅 `0.557`；sitting proxy
AUC 为 `0.444`，不支持通用 person-score 规则；fr1/xyz 显示小区域和低支持量
产生大量极端 score。当前不选择动态阈值。

G2-4 文档：

```text
results/g2_4_2026-07-29/G2_4_DYNAMIC_STATIC_SEPARABILITY_LITERATURE_AUDIT.md
results/g2_4_2026-07-29/G2_4_DYNAMIC_STATIC_SEPARABILITY_SHADOW_SPEC.md
results/g2_4_2026-07-29/G2_4_PRELIMINARY_SEPARABILITY_RISK_AUDIT_RESULT.md
results/g2_4_2026-07-29/g2_4_preliminary_separability_audit.json
results/g2_4_2026-07-29/G2_4A_RISK_INSTRUMENTATION_RESULT.md
results/g2_4_2026-07-29/G2_4_BONN_STATIC_MOVING_BOX_EVALUATION_PROTOCOL.md
results/g2_4_2026-07-29/G2_4B_BONN_COORDINATE_DOMAIN_AUDIT_AND_SPEC.md
results/g2_4_2026-07-29/G2_4B_BONN_COORDINATE_DOMAIN_RESULT.md
results/g2_4_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_SPEC.md
results/g2_4c_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_RESULT.md
results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_SPEC.md
results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_RESULT.md
```

boundary、invalid-depth 和 reference-support instrumentation 已完成且默认
关闭。最终 pose-quality 没有接入，因为 geometry 当前在 `TrackLocalMap()`
之前运行。

Bonn坐标域只读审计已完成。当前feature depth在raw `mvKeys`位置读取，真正冲突
是geometry warp用无畸变针孔模型解释Bonn的非零畸变raw depth pixel。已冻结
最小候选为：RGB线性rectification、depth最近邻rectification、YOLO/ORB/
geometry全部使用`P=official K`的统一域，并将tracking distortion设为零。
本地两份box archive的`depth.txt`还分别引用4个和3个不存在的PNG，association
必须过滤。

默认关闭的Bonn联合输入rectification已经实现。确定性测试、TUM旁路和Bonn
150帧RGB/depth/ORB/geometry shadow通过；rectification mean为`0.631 ms`，
2107条region CSV invariant无违反。host GPU的30帧online YOLO也通过，
mask ready为`30/30`且age median/max为`0/0`。完整坐标域门通过。

G2-4C自动候选选帧已经完成。第一版因重复视图过多失败并保留；第二版在不改
geometry proxy的前提下加入固定多样性门，两条box序列各得到24个唯一候选。

后续独立审阅发现两项必须修正的问题：

1. 两条序列均已按geometry proxy排序并查看联系表，`moving_obstructing_box`
   还参与了diversity规则修正，因此二者都是development/review sequence，
   不是strict hold-out；
2. 原工具用RGB timestamp插值GT pose，而depth warp物理上对应depth timestamp。

修正后的工具显式支持`--pose-timestamp-source rgb|depth`。四组重跑显示
per-frame inconsistency相关较高（`0.893/0.927`），但24帧候选只重叠
`14/24`和`12/24`，因此后续开发期审查默认使用depth-time候选，RGB-time
只保留pipeline-matched sensitivity对照。两套结果都不是motion GT，也不能
估计完整序列的无偏precision/recall。

修正报告：

```text
results/g2_4c_correction_2026-07-29/
  G2_4C_TIMESTAMP_AND_HOLDOUT_CORRECTION_RESULT.md
```

G2-4D 已完成修正后48个depth-time候选帧的实际C++ person detection和最终
person union filter mask导出。48/48均满足requested seq等于returned mask
seq。当前真实mask是`CV_8U`且按`nonzero=filtered`解释；由于实例mask resize
使用线性插值，边缘存在`1..254`，本阶段只记录，没有修改YOLO。

仅查看rectified RGB和source frame id生成了
`agent_rgb_only_coarse_bbox_v1` target-box预标注；没有读取geometry
residual/score/role。预标注明确为unverified且不是GT。nonobstructing仅
5/24帧有person mask，obstructing仅4/24帧有person mask；无人物箱子帧中
person-only语义mask为零。所有粗框内非零覆盖来自人物遮挡/搬运，不能解释为
箱子被正确分割。

G2-4E 已完成上述 development separability audit，并先封存了未运行
geometry proxy 的 `rgbd_bonn_balloon_tracking.zip`。封存 SHA256 为：

```text
3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
```

两条完整 development 序列分别运行 `778/589` 帧，online person mask 均为
每帧就绪、age 为 0；shadow 配置 actual FPS 为 `24.109/25.750`。新增离线
partition audit 使用与在线相同的 rectification、米制深度转换和
`PartitionDepthByDiscontinuity()`；42 个有在线证据的候选帧全部 exact match。

G2-4E 的关键结果是：

- nonobstructing 无人物箱子 bbox 的主导 region 约 85% 延伸到 bbox 外；
- bbox 主导 region 的正证据与整帧全局正证据近乎相同；
- obstructing 的 target-absent 帧全局正证据不低于 target-visible/person-absent；
- 强信号主要由 person-present 搬运/遮挡条件驱动，而人物已由语义分支覆盖。

因此当前 region-level evidence 没有通过动态判决门。没有选择阈值、没有运行
strict hold-out，也没有修改任何 SLAM 状态。详细结果：

```text
results/g2_4e_2026-07-29/G2_4E_CONTINUOUS_DEVELOPMENT_EVIDENCE_AUDIT_RESULT.md
results/g2_4e_2026-07-29/STRICT_HOLDOUT_MANIFEST.md
```

当前下一小步不再是继续调 region score。必须先阅读本地 PaperNotes/PDF，
明确下一项最小证据审计的文献归属；优先考虑 direct bbox/ORB feature evidence
和独立 motion-state 自动预标注，不得直接拟合组合阈值或解封 hold-out。

G1-F 仍需可靠 feature-level 判决门；G1-D 仍必须等待像素区域 mask 的独立验收。

G2-4F0 已按上述边界完成。它去掉失败的 region aggregation，直接在当前 ORB
feature 中心读取现有 scale-2 multi-reference vote，并用 G2-4D 的独立
RGB-only粗框和实际 person mask 做 development 审计。

两条完整序列 online CUDA mask 均逐帧就绪、age=0；输出
`23097/17794` 条 feature 记录，actual FPS 为 `24.575/25.931`。48 个候选中
42 帧有多参考 evidence，早期缺失 6 帧继续保持 no-evidence。

无人物箱子帧的关键中位数：

```text
nonobstructing:
  inside comparison coverage       95.801%
  inside/outside positive presence 1.746% / 6.703%
  presence enrichment              0.289x
  vote enrichment                  0.205x

obstructing:
  inside comparison coverage       93.413%
  inside/outside positive presence 4.060% / 6.704% (outside n=6)
  presence enrichment              0.472x
  vote enrichment                  0.428x
```

所以 current direct positive evidence 在这些未知箱子候选上没有局部富集；
失败不只来自 depth-region aggregation。人物出现时 nonobstructing 信号明显
增强，但人物已由 semantic branch 覆盖。候选没有 motion GT、粗框不是 pixel
GT，因此不能把该负结果泛化为所有 depth-warp 失败。

```text
dynamic_decision          = none
direct_slam_state_mutation = none
G1-F / G1-D               = locked
strict hold-out           = sealed and unopened
```

G2-4F1 的文献审计、SPEC、实现、测试和两条 Bonn 完整同步语义 development
实验均已完成。设计为：当前 ORB feature backward LK 到上一成功帧，使用
上一帧 depth 和当前 initial/上一帧 final SE(3) 计算 ego flow，并可选
GT-pose 对照；所有输出均为连续 residual 或 no-evidence。

在查看 residual 前必须生成的独立
`agent_rgb_temporal_motion_proxy_v1` 已完成：只查看 rectified RGB temporal
clip，不读取任何 geometry/flow residual；48 帧得到
stationary/moving/uncertain/not_visible=`30/7/2/9`。它不是 GT。

生成代理时还发现并修正了 nonobstructing frame 246/478/537 三个错误粗框。
G2-4D/E/F0 已在新目录完成复算，旧目录保留。F0 修正后箱内信号均值上升，但
中位数仍未富集，且 frame 478/537 属于 stationary/high 代理，因此负门控
不变。

G2-4F1 自身中位成本约 `2.5 ms`，完整系统约 `29.27/29.70 FPS`。但关键
`moving+person-absent` 只有 1 帧；随后完全独立于 flow/geometry 的全序列
RGB/semantic 扩充又审查了 50 个时间窗，得到
stationary/moving/uncertain=`48/0/2`。所以当前科学门不可评价，G1 仍锁定。

随后 G2-4F1D 已用 balloon/balloon2 非 holdout 数据补齐方向性开发证据；
G2-4F2 已完成本地文献审计、TUM/Bonn 真静态跨域风险、depth
boundary/invalid 分层和保守工作点冻结。`FB<=0.25, q>=10` 只定义
`high_residual_candidate`，不是 dynamic。TUM/Bonn 静态 candidate rate 为
`0.232%/0.546%`，MapPoint candidate rate 为 `0.068%/0.117%`。

边界分层没有解释 Bonn 的主要静态尾部：`q=10` 时 75.2% 候选仍在
`clean_d2`，因此没有加入 boundary veto。GT-pose 归一化尾部更低，但 raw
residual 更高，只能说明 pose 分支相关风险，不能作因果结论。

下一小步是先固定当前代码 commit，再按已写入的 F2D 协议一次性打开
`balloon_tracking` strict holdout。打开后不得根据结果回调 `FB/q`，且即使
通过也只能进入 G1-F0 mutation shadow，不能直接真实过滤。

G2-4F0 文档：

```text
results/g2_4f_2026-07-29/G2_4F_DIRECT_FEATURE_EVIDENCE_LITERATURE_AUDIT.md
results/g2_4f_2026-07-29/G2_4F0_DIRECT_MULTIREFERENCE_FEATURE_EVIDENCE_SPEC.md
results/g2_4f_2026-07-29/G2_4F0_DIRECT_MULTIREFERENCE_FEATURE_EVIDENCE_RESULT.md
results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_LITERATURE_AUDIT.md
results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_SHADOW_SPEC.md
results/g2_4f_2026-07-29/G2_4D_TO_F0_BBOX_TEMPORAL_CORRECTION_RESULT.md
results/g2_4f_2026-07-29/G2_4F1_RGB_TEMPORAL_MOTION_PROXY_RESULT.md
results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_SHADOW_RESULT.md
results/g2_4f1_expansion_2026-07-29/G2_4F1_INDEPENDENT_CANDIDATE_EXPANSION_RESULT.md
```

---

## 9. 新会话接手检查清单

新 Codex 会话应按以下顺序执行：

1. 阅读本文全文；
2. 执行只读检查：

   ```bash
   cd /home/zhu/dynaslam_ws
   git status --short --branch
   git log -1 --oneline
   git diff --stat
   ```

3. 阅读：

   ```text
   results/DT-SLAM_几何模块阶段进度_2026-07-28.md
   results/g2_3r3_2026-07-29/G2_3R3_PYRAMID_DENSE_EVIDENCE_SHADOW_SPEC.md
   results/g2_3r3_2026-07-29/G2_3R3_PYRAMID_DENSE_EVIDENCE_SHADOW_RESULT.md
   results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_LITERATURE_AUDIT.md
   results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_SPEC.md
   results/g2_3r4_2026-07-29/G2_3R4_LOW_RESOLUTION_REGION_APPROXIMATION_SHADOW_RESULT.md
   results/g2_4_2026-07-29/G2_4_DYNAMIC_STATIC_SEPARABILITY_LITERATURE_AUDIT.md
   results/g2_4_2026-07-29/G2_4_DYNAMIC_STATIC_SEPARABILITY_SHADOW_SPEC.md
   results/g2_4_2026-07-29/G2_4_PRELIMINARY_SEPARABILITY_RISK_AUDIT_RESULT.md
   results/g2_4_2026-07-29/G2_4A_RISK_INSTRUMENTATION_RESULT.md
   results/g2_4_2026-07-29/G2_4_BONN_STATIC_MOVING_BOX_EVALUATION_PROTOCOL.md
   results/g2_4_2026-07-29/G2_4B_BONN_COORDINATE_DOMAIN_AUDIT_AND_SPEC.md
   results/g2_4_2026-07-29/G2_4B_BONN_COORDINATE_DOMAIN_RESULT.md
   results/g2_4_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_SPEC.md
   results/g2_4c_2026-07-29/G2_4C_BONN_AUTOMATIC_FRAME_SELECTION_RESULT.md
   results/g2_4c_correction_2026-07-29/G2_4C_TIMESTAMP_AND_HOLDOUT_CORRECTION_RESULT.md
   results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_SPEC.md
   results/g2_4d_2026-07-29/G2_4D_PERSON_EXPORT_AND_BOX_PREANNOTATION_RESULT.md
   results/g2_4e_2026-07-29/G2_4E_CONTINUOUS_DEVELOPMENT_EVIDENCE_AUDIT_SPEC.md
   results/g2_4e_2026-07-29/G2_4E_CONTINUOUS_DEVELOPMENT_EVIDENCE_AUDIT_RESULT.md
   results/g2_4f_2026-07-29/G2_4F_DIRECT_FEATURE_EVIDENCE_LITERATURE_AUDIT.md
   results/g2_4f_2026-07-29/G2_4F0_DIRECT_MULTIREFERENCE_FEATURE_EVIDENCE_SPEC.md
   results/g2_4f_2026-07-29/G2_4F0_DIRECT_MULTIREFERENCE_FEATURE_EVIDENCE_RESULT.md
   results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_LITERATURE_AUDIT.md
   results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_SHADOW_SPEC.md
   results/g2_4f_2026-07-29/G2_4F1_SPARSE_EGO_FLOW_SHADOW_RESULT.md
   results/g2_4f1_expansion_2026-07-29/G2_4F1_INDEPENDENT_CANDIDATE_EXPANSION_RESULT.md
   results/g2_4f2_2026-07-30/G2_4F2_RELIABLE_FEATURE_GATE_LITERATURE_AUDIT.md
   results/g2_4f2_2026-07-30/G2_4F2_RELIABLE_FEATURE_GATE_SHADOW_SPEC.md
   results/g2_4f2_2026-07-30/G2_4F2B_DEPTH_BOUNDARY_RISK_SHADOW_SPEC.md
   results/g2_4f2_2026-07-30/G2_4F2B_DEPTH_BOUNDARY_RISK_SHADOW_RESULT.md
   results/g2_4f2_2026-07-30/G2_4F2D_CANDIDATE_WORKING_POINT_FREEZE_AND_HOLDOUT_PROTOCOL.md
   results/ate_semantic_baseline_2026-07-29/REPORT.md
   ```

4. 对照当前 `Tracking.cc`、`GeometricDynamicDetector.*` 和 YAML，确认代码确实处于 shadow-only；
5. 若状态与本文不同，先增加“交接差异记录”，不要擅自恢复或重置；
6. 向用户用简短文字复述：
   - 当前阶段；
   - 当前未提交状态；
   - 下一小步；
   - 明确不会做的越界修改；
7. 阅读 G2-3R4 RESULT，确认收益门失败及结构指标未测；
8. 阅读 G2-4 初步结果、风险字段结果、G2-4B/C/D/E/F0/F1/F1D SPEC/RESULT及G2-4C
   correction result；确认两个当前box序列都不是strict hold-out，G2-4D粗框
   也不是pixel/motion GT，G2-4E region 与 G2-4F0 direct feature 判决门均
   未通过；G2-4F1 在原 moving-box 数据上不可评价，但 F1D 已在非 holdout
   balloon/balloon2 上得到独立于 flow 的可观察 motion proxy 和明确方向性
   residual；
9. 确认 `rgbd_bonn_balloon_tracking.zip` 的 SHA-256 仍为
   `3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421`；
   G2-4F2D 工作点和一次性 holdout 协议已经冻结。只能在记录冻结 commit
   后解封一次，不能根据结果回调参数或直接进入 G1。

---

## 10. 信息冲突时的事实优先级

从高到低：

1. 当前本地代码、配置、Git 状态；
2. 原始运行日志、CSV、轨迹和可复现实验；
3. 对应阶段的 `SPEC.md` 与 `RESULT.md`；
4. 本交接文档；
5. 总进度文档；
6. 聊天上下文或自动压缩摘要。

处理规则：

- 不静默选择其中一个版本；
- 明确写出冲突内容和证据路径；
- 用只读检查或最小复现实验解决；
- 未解决前不得将推测写成事实；
- 修正后同步更新阶段结果、总进度和本文三处。

---

## 11. 交接结论

项目没有偏离原定研究边界，但几何模块比最初预想复杂，原因已经被实验定位：

- 单参考帧 depth warp 能提供类别无关的不一致证据；
- 任意正残差 seed 的深度连续 flood fill 不能可靠恢复对象；
- 稀疏 ORB/grid sampling 的覆盖不足；
- full dense 覆盖足够但成本过高；
- scale-2 pyramid dense evidence 已解决大部分覆盖/成本矛盾，但整体仍约 24–25 FPS；
- G2-3R4 已因端到端收益门失败停止；
- G2-4A 已确认现有 proxy 信号不足以直接形成通用判决；
- G2-4B 已完成 Bonn 联合 rectification，RGB/depth/ORB/online mask/geometry
  坐标域门通过；
- G2-4C 已完成两条moving-box development/review序列的自动候选选帧和
  RGB/depth timestamp敏感性修正，但没有box motion label，strict hold-out
  尚未选择；
- G2-4D 已完成48帧真实C++ person filter导出和独立RGB-only box粗框审计，
  验证无人物箱子帧没有语义过滤，同时保持粗框不是GT；
- G2-4E 已封存 strict hold-out，并完成两条连续 development 序列的 exact
  C++ geometry/box 成对审计；当前固定 depth-region 聚合不能隔离
  nonobstructing box，且 person-present 是明显混杂因素，因此动态判决仍为
  `none`；
- G2-4F0 已在 ORB feature 中心直接审计 multi-reference vote；无人物箱子
  条件下框内正证据没有相对背景富集，说明失败不只来自 region aggregation，
  但由于没有 motion GT，不能泛化为所有 depth-warp 几何失败；
- G2-4F1 已实现相邻成功帧 sparse LK observed-flow 减 RGB-D/SE(3)
  ego-flow。两条 Bonn 同步语义开发序列保持约 29.27/29.70 FPS，F1 自身
  中位约 2.55/2.52 ms；`moving+person-present` 有方向性信号，但
  `moving+person-absent` 只有 1 帧，不能形成判决门；
- 独立 RGB/semantic-only 扩充复核了 50 个额外时间窗，得到
  stationary/moving/uncertain=`48/0/2`。没有把 box 填满视野、无法区分
  camera/object motion 的片段强制当 positive，因此当前需要新的可观察
  development 数据，而不是继续调 flow threshold；
- G2-4F1D 已在协议先行的前提下将 balloon/balloon2 降为 development，并保持
  balloon_tracking 封存。RGB-only exact-center 修正后，7 个
  moving-observable 且气球框内 person-mask 像素恰为 0 的帧中有 6 帧存在
  可测 ORB；框内/去人物背景 residual 中位为 `11.919/0.723 px`，成对比值
  中位 `20.045x`，GT-pose 框内中位 `11.582 px`。这是方向性 evidence，
  不是阈值或动态判决；
- G2-4F2 已完成 FlowFusion/Kalal/Li–Lee/DynaSLAM/SInDSLAM 的来源审计，
  并在 TUM/Bonn 真静态域完成 `FB/q` 风险曲线；边界主解释被否决，没有加入
  boundary veto；
- `FB<=0.25, q>=10` 已冻结为项目级 `[S/H]`
  `high_residual_candidate` 工作点。它在两个静态域的 MapPoint 候选率均低于
  0.12%，并保留 6/6 个 development 可测气球帧的框内候选；它仍不是动态
  标签；
- 几何尚未进入真正的 SLAM 过滤，因此现在不测“几何 ATE 改善”，只保留语义基线 ATE/FPS 作为冻结对照。

当前下一步仍不是进入 G1。应先提交冻结代码和协议，再一次性运行 sealed
`balloon_tracking` holdout；无论通过与否都不得回调工作点。通过后也只允许
进入 G1-F0 mutation shadow。

这份交接的目的不是让新会话“相信摘要”，而是让它知道去哪里核查、哪些结论已经冻结、哪些结果仍是假设，以及下一步只能改动什么。

---

## 12. 2026-07-30 当前状态覆盖更新

本节覆盖前文仍写着 `balloon_tracking sealed` 的旧状态。

### 12.1 当前版本与留出状态

```text
pre-holdout freeze commit:
  f964cf0 Freeze reliable sparse-flow candidate gate
balloon_tracking:
  formally opened and evaluated once
threshold retuning:
  none
G1-F0 / G1-F / G1-D:
  locked
```

### 12.2 Strict holdout 结论

14 个在 geometry 前冻结的 RGB-only moving proxy 均有 exact-zero person
mask。冻结 `FB<=0.25,q>=10` 只在 7/14 帧产生框内 candidate，7/14 帧
inside rate 高于 outside；全序列 MapPoint candidate rate 为 `0.2248%`。
对应门为 `80%/80%/<=0.20%`，所以 G2-4F2 strict gate 失败，禁止在该
holdout 上回调阈值。

连续 residual 仍在 13/14 帧表现为框内中位高于框外。因此保留 F1 连续证据，
冻结 `q>=10` 为失败 baseline，不将它写成 `dynamic=true`。

### 12.3 性能记录修正

首次 holdout 开启完整 feature CSV 时发现每帧
`reserve(current_size+samples)` 导致 O(N²) 诊断记录搬移。本次 24.00 FPS
不代表算法本体。删除该 reserve 后在非 holdout static150 验证 F1 约
2.7–2.9 ms、29.72 FPS、0 deadline miss；按协议没有重跑 holdout。

### 12.4 当前下一步

先读：

```text
results/g2_4f2_holdout_2026-07-30/G2_4F2H_STRICT_HOLDOUT_RESULT.md
results/g2_4f2_holdout_2026-07-30/G2_4F3_LOCAL_RIGIDITY_LITERATURE_DECISION.md
results/g2_4f3_2026-07-30/G2_4F3_LOCAL_RIGIDITY_COHERENCE_SHADOW_SPEC.md
```

然后只允许：

```text
G2-4F3 SPEC:
  two-frame local feature graph
  relative 3D edge-length consistency
  continuous ego-flow residual kept separate
  shadow-only edge/component statistics
```

来源是 Dai 等 point-correlation 原理的局部适配 `[A]`，并受
SInDSLAM/DetectFusion“residual 需要空间上下文”的证据支持。禁止复制完整
graph optimization、禁止最大组静态假设、禁止修改 Optimizer/g2o、禁止根据
已打开 holdout 选阈值。
