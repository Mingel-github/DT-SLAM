# DT-SLAM 项目交接说明

更新时间：2026-08-05
工作区：`/home/zhu/dynaslam_ws`
源代码：`/home/zhu/dynaslam_ws/DT-SLAM`

> 本文是新 Codex 会话的单一交接入口，不替代代码、实验日志和阶段报告。若本文与本地代码或原始日志冲突，以本地可核查证据为准，并先记录差异，不得凭记忆猜测。

## 0. 2026-08-04 当前权威状态

本文第 1--35 节保留历史路线和负实验，不能把其中旧 commit 或“下一步”覆盖当前状态。
当前算法检查点为 `16ea79d`，SIn 风格 S1/S2/S3 已提交并推送。工作区中的大型
实验产物仍只保存在本地；本地原始日志优先于摘要，Git 用于保存代码、配置和轻量报告。

当前主线：

| 阶段 | 状态 |
| --- | --- |
| S0：独立 SInDSLAM CPU/GPU、源码和协议审计 | 已完成 |
| S1：native CPU DeepFlow＋区域判决 shadow | 已完成受控 parity 和 TUM/Bonn 行为审计；gradient-only RAG 不等价 PEAC |
| S2：region mask → ORB feature filter | 已完成限定验收、默认关闭；最小原生条件 fail-open 消除已观察到的 obstructing 长段 LOST |
| S3：映射侧动态深度过滤 | 工程接口、合成测试、Bonn 动态/静态首轮和同位姿点云已完成；默认关闭，地图质量未放行 |
| S4：长间隔精修 | 未开始；只有成对点云确认慢速/间歇运动残影后才考虑 |

2026-08-05 已冻结新的 R0--R6 因果审计计划。当前执行状态：R0 三轮 Gazebo
纯 ORB 与全零语义路径配对重复实验已完成；R1 的
`flow -> ego compensation -> residual -> region -> classifier -> temporal`
只读失败层审计是唯一下一步。在 R1 报告完成前，不实施 S4、OctoMap、SE(3)
替换、classifier 修改、plane edge 或新融合。

R0 关键结论：纯 ORB 三轮 ATE 范围为 `0.028619--0.036430 m`，全零语义路径为
`0.029540--0.039638 m`；配对差异方向发生反转。三轮 semantic-only 均使用 CUDA，
600/600 mask 同帧可用，语义动态像素和被拒绝深度均为 0。因此先前单轮差异不能归因于
语义 mask，厘米以内的单轮 ATE 变化不能单独作为方法收益。

详细记录：

- `results/DT-SLAM_R0-R6_因果审计与后续研究计划_2026-08-05.md`
- `results/r0_freeze_2026-08-05/R0_REPRODUCIBLE_BASELINE_SPEC.md`
- `results/r0_freeze_2026-08-05/R0_BASELINE_EQUIVALENCE_RESULT.md`

最重要的新证据：

- Bonn `moving_nonobstructing_box` 778 帧三轮：S2 OFF/ON ATE RMSE 中位
  `0.514344/0.022526 m`，RPE RMSE 中位 `0.022127/0.014381 m`；ON 三轮均完整；
- Bonn `moving_obstructing_box` 589 帧最小安全收尾：只在 frame 296 的 ORB-SLAM2
  motion-model pre-pose `<20 matches` 条件触发一次 fail-open，589/589 Tracking OK；
  ATE/RPE=`0.245857/0.016536 m`，同协议 OFF 为 `0.546996/0.019121 m`；
- fail-open 只撤销本帧 SIn geometry tracking flags，semantic 保留；Tracking 结束后恢复
  同一 flags 继续否决新 MapPoint。没有新 detector 阈值或额外 PoseOptimization；
- nonobstructing 复核 ATE/RPE=`0.024146/0.014319 m`、fail-open 0 次；Bonn
  `static_close_far` 前 300 associations 无 LOST、无 fail-open；
- S3 不改 Tracking：移动非遮挡箱子 778/778 成功，ATE/RPE=`0.019436/0.014075 m`；
  静态近远景前 298 帧 ATE/RPE=`0.021876/0.018684 m`。这些轨迹差异不能归功于 S3；
- S3 过滤器中位耗时约动态/静态 `0.373/0.307 ms`。有效深度总否决比例约
  `9.04%/6.84%`，但静态个别帧有大面积删除，因此只证明接口成立，未证明地图质量；
- 已生成同位姿过滤前/后 PLY，下一步只做箱子残影和静态空洞检查，不增加 S2 阈值或
  安全补丁。
- 5 cm 时间支持代理：动态箱子序列中单帧/至少八帧支持体素的完全删除比例为
  `33.00%/0.20%`，静态序列为 `5.65%/0%`。这是优先清理短暂残影的正面趋势，
  但不是箱子真值，仍需查看成对 PLY 的空间位置。
- 用户已并排查看动态 PCD：未过滤点云有多个重合人影，S3 后约剩两道人影，动态残影
  明显减少。该结果支持 S3 对当前序列的建图价值；残留说明仍有漏检，静态成对点云
  还需最终目视确认。

当前详细报告：

- `results/sindslam_s1_shadow_2026-08-03/S1_NATIVE_CPU_REGION_DECISION_SHADOW_RESULT.md`
- `results/sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_SPEC.md`
- `results/sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md`
- `results/sindslam_s3_2026-08-04/S3_DYNAMIC_DEPTH_FILTER_SPEC.md`
- `results/sindslam_s3_2026-08-04/S3_DYNAMIC_DEPTH_FILTER_RESULT.md`

当前 SIn 改动仍遵守：默认关闭、不新增第三次 `PoseOptimization()`、不修改
`Optimizer.cc`/g2o/YOLO/LoopClosing/LocalMapping 算法。作者 PEAC 文件为
`AGPL-3.0-or-later`，未直接复制进 DT-SLAM。

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
- shadow 模式不写 SLAM 状态；默认关闭的 G1-F1 可实验性清除
  `TrackLocalMap()` 的少量 `mvpMapPoints` 关联；默认关闭的 G1-M1 可在通过
  额外安全条件后阻止同一 q10 候选写入稀疏 MapPoint。二者均不修改优化器或
  后端目标。

本地工作区是权威版本，GitHub 仅作为备份。不能因远端分支缺少未提交实验就覆盖本地状态。

---

## 2. 仓库状态

当前分支：

```text
main
```

当前已推送提交：

```text
e2d6559 Evaluate motion grouping shadow routes
```

当前 G1-F1/G1-M0/G1-M1 实现与正式结果、G2-6E/G2-6O 审计和部分较早工具
仍是本地未提交工作。提交前必须以 `git status` 重新生成精确文件清单；下方
旧阶段的未提交清单仅是历史记录，不再视为当前完整状态。

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

---

## 13. 2026-07-30 G2-4F3 完成状态覆盖更新

本节覆盖前文“F3 尚未实现”的旧描述。

### 13.1 已实现

G2-4F3 已实现 current-image Delaunay 邻接和相邻两帧三维 edge-length
change，输出连续 node/edge/frame 诊断。它只复用 F1 对应关系，不增加
PoseOptimization，不修改动态标志、MapPoint、Optimizer、g2o 或后端。

### 13.2 当前证据

```text
synthetic tests                         PASS
OpenCV 4.5.4 C++/Python edge parity     2526/2526, exact set match
Bonn static F3 median/P95               1.984/2.112 ms
TUM fr1/xyz F3 median/P95               1.902/2.103 ms
development in-box flow > outside       15/15 comparable frames
development internal strain <= outside  11/14 comparable frames
development crossing strain > outside   13/14 comparable frames
```

代理不是 motion/pixel GT。少数帧节点不足，`balloon2` frame 230 有严重混合
深度异常；relative strain 也受长边分母影响。因此只是方向性支持，不是动态
分类器。

### 13.3 当前代码状态

本轮修改尚未提交。新增/修改集中在：

```text
GeometricDynamicDetector.h/.cc
Tracking.h/.cc
geometric_warp_test.cc
BONN/TUM1 GeometryLocalRigidityShadow YAML
audit_local_rigidity_shadow.py
G2-4F3 RESULT、总进度和 HANDOFF
```

### 13.4 下一步限制

先从本地 PaperNotes/原始 PDF 核对鲁棒 edge consistency、RGB-D 深度噪声和
图分组依据，再冻结一个只输出连续 edge reliability 的设计。不得：

```text
读取 balloon_tracking 回调规则
直接选择 rigidity hard threshold
建立 dynamic component
进入 G1-F/G1-D
修改 YOLO/Optimizer/g2o/后端
```

---

## 14. 2026-07-30 G2-4F3U 完成状态覆盖更新

本节覆盖前文“下一步设计鲁棒 edge reliability”的旧描述。

### 14.1 已完成

依据 Dai 等 point-correlation 的协方差归一化思想 `[A]` 和 Khoshelham 等
Kinect 深度误差随距离平方增长的模型 `[L/A]`，实现了一个两帧标量边长变化的
不确定度归一化连续诊断 `[S/H]`。它不是 Dai 原方法复现。

新增 3x3 深度混合不确定度、边长一阶误差传播、node/edge/frame CSV 和离线
审计；没有动态阈值、动态 component 或任何 SLAM mutation。

### 14.2 结果与决策

```text
build + deterministic tests                      PASS
uncertainty denominator floor                    0 / 131256 static edges
development internal q <= background             4 / 14
development crossing q > background              13 / 14
old absolute internal strain <= background       11 / 14
measurement implementation                       PASS
default coherence score                          REJECT
dynamic_decision / direct_slam_state_mutation     none / none
G1-F / G1-D                                      locked
```

轴向不确定度归一化没有改善内部刚体一致性的代理可分性，因此不得选择 hard
threshold。跨组边仍有方向性，但不足以独立产生动态判决。失败的是本项目的两帧
标量适配，不是 Dai 的长期点关联图方法。

### 14.3 性能

新增 uncertainty metric 约 `0.53–0.58 ms` median，相对旧 F3 增量约
`0.16–0.20 ms`。host RTX 4060 Ti 在线同步 YOLO 下：

```text
balloon   289 frames  actual 28.40 FPS  deadline miss 284/289
balloon2  393 frames  actual 25.99 FPS  deadline miss 392/393
```

这是完整 pipeline 性能，不能全归因于 F3U；但 Bonn 上仍没有稳定 30 FPS 余量。

### 14.4 当前下一步限制

先阅读：

```text
results/g2_4f3u_2026-07-30/G2_4F3U_UNCERTAINTY_NORMALIZED_EDGE_LITERATURE_AUDIT.md
results/g2_4f3u_2026-07-30/G2_4F3U_UNCERTAINTY_NORMALIZED_EDGE_SHADOW_SPEC.md
results/g2_4f3u_2026-07-30/G2_4F3U_UNCERTAINTY_NORMALIZED_EDGE_SHADOW_RESULT.md
```

不得继续在已打开 proxy 上堆叠 edge score 或调阈值。下一项实验必须先给出原始
文献依据、预注册假设和失败停止条件，并继续保持 shadow-only；否则应暂停这条
局部刚性分支并重新评估 G2-4 的最小动态判决路线。

---

## 15. 2026-07-30 G2-4F4 区域上下文审计完成状态

### 15.1 路线选择

F3U 之后没有继续堆叠 edge score。先核对本地 PaperNotes 和 DetectFusion /
SInDSLAM 原始 PDF，比较：

```text
继续调整 LK/FB
遮挡/边界 veto
区域内聚合连续 F1 residual
```

只批准第三条做最小离线 shadow audit。方法身份是两篇论文“motion residual
需要独立 geometry segment 上下文”原理的轻量适配 `[A/S/H]`，不是复现。

### 15.2 实验结果

使用冻结的 `balloon/balloon2` RGB-only coarse bbox 和 exact C++ F3U node
CSV，把 measured nonsemantic F1 residual 映射到 G2-3R0 depth component：

```text
support >=3 comparable frames             14
point inside median > background          14/14
selected region median > background       11/14 = 78.57%
pre-registered requirement                >=80%
representation gate                       FAILED
dynamic_decision / mutation               none / none
```

失败来自部分 region 跨入大块静态背景。例如 `balloon frame 39` 的所选区域中
只有 `6/610` 个 eligible feature 在粗框内；点级 inside/background 仍为约
`20×`，整区聚合却降到 `0.884×`。

### 15.3 决策

停止当前轻量 depth-component 容器：

```text
不改 region selector
不调 residual / region threshold
不做 online F4
不进入 G1-F/G1-D
```

保留 F1 continuous residual；被否定的是“G2-3R0 component 可直接作为运动
对象聚合单位”。下一步需要在“实现更完整且较重的文献 re-clustering 子集”与
“回到 feature-level continuous evidence 并重新设计独立验证/fail-safe”之间
做总路线决策，不能自动继续加方法。

详细记录：

```text
results/g2_4f4_2026-07-30/G2_4F4_REGION_CONTEXT_LITERATURE_DECISION.md
results/g2_4f4_2026-07-30/G2_4F4_REGION_CONTEXT_SHADOW_SPEC.md
results/g2_4f4_2026-07-30/G2_4F4_REGION_CONTEXT_SHADOW_RESULT.md
```

## 16. 2026-07-30 运动分组输入审计与下一路线覆盖更新

本节覆盖 15.3 中仍未决的“更强区域还是 feature fail-safe”。

### 16.1 输入审计结论

新增只读工具：

```text
DT-SLAM/tools/audit_motion_grouping_track_support.py
```

在冻结 development `balloon/balloon2` 上，代理内 MapPoint 只占
`10.53%/3.28%`；三图像中仍有至少 3 个轨迹的帧均为 `4`，六图像时均只剩
`2`。因此不批准固定 3/5 帧 persistence 或 MapPoint long-track clustering。

### 16.2 文献决策

原文核对后确认：

- Dai 前端虽使用两帧，但还依赖 tracked MapPoint、point-correlation
  optimization、Mahalanobis edge culling、CC 和最大体积静态组；
- 将其改为 transient LK nodes 不是小改动；
- Tateno/DetectFusion normal+distance segment 可支持低纹理和 `M_depth`，
  但 segment 本身不判断运动；
- DetectFusion 的未知动态仍依赖 static surfel-map ICP residual。

因此下一项冻结为：

```text
G2-4R1 offline normal+distance geometric-segment representation audit
```

它不得修改 `Tracking`/`GeometricDynamicDetector`，不得读取 residual 生成
segment，不得开放 G1。

### 16.3 当前下一步

按已冻结 SPEC 实现：

```text
DT-SLAM/tools/audit_normal_distance_segments.py
```

并与 G2-3R0 在同一 development 帧上做成对 representation audit。详细依据：

```text
results/g2_4_motion_grouping_2026-07-30/
  G2_4_MOTION_GROUPING_INPUT_FEASIBILITY_RESULT.md
  G2_4_MOTION_GROUPING_ROUTE_DECISION.md
  G2_4R1_NORMAL_DISTANCE_SEGMENTATION_SHADOW_SPEC.md
```

继续禁止：

```text
dynamic_decision
mvbDynamic / mvpMapPoints mutation
G1-F / G1-D
Optimizer / g2o / backend modification
third PoseOptimization
```

## 17. 2026-07-30 G2-4R1 完成状态覆盖更新

本节覆盖 16.3 的“下一步实现 G2-4R1”。

### 17.1 实现与输入纠正

已新增：

```text
DT-SLAM/tools/audit_normal_distance_segments.py
```

实现 frame-wise Tateno-style bilateral depth、vertex/normal、
concavity edge、Nguyen-noise point-to-plane distance edge 和 connected
components，并与 G2-3R0 做相同 depth 的配对审计。

正式输入使用 F1 `*_f1_features.csv` 的全部 `evidence_state=measured`
节点；没有误用带 F3U 邻域支持条件的子集。bbox/residual 不参与 segment
生成或参数选择。

### 17.2 结果

```text
Bonn candidate frames                         17
normal regions median balloon/balloon2        2487 / 2669
small-region fraction median                  94.51% / 94.98%
best single-region bbox coverage combined     0.488 median
all segments can cover 80% bbox-valid depth   1 / 17
selected residual > background                14 / 14 comparable
fr1/xyz static region count                   406 median
deterministic replay                          PASS
```

normal+distance 表示把大型背景切开并留下纯净目标片段，但目标有效深度大量落入
boundary/unknown，且整体严重碎裂。按预注册条件冻结为负结果，不允许在已打开
proxy 上调 noise/bilateral/merge/area/residual 参数。

### 17.3 当前状态

```text
G2-4R1 frame-wise adaptation       stopped
full Tateno / DetectFusion         not implemented, not rejected
dynamic_decision                  none
direct SLAM mutation              none
G1-F / G1-D                       locked
```

下一步必须先做有原始文献依据的轻量多 feature 运动一致性分组可行性审计；若
现有短轨迹支持不足，则应明确暂停，而不是继续发明阈值。

详细记录：

```text
results/g2_4r1_2026-07-30/
  G2_4R1_NORMAL_DISTANCE_SEGMENTATION_SHADOW_RESULT.md
```

## 18. 2026-07-30 G2-4R2 图拓扑可行性审计完成状态

本阶段先纠正了上一轮路线理由：目标代理内 MapPoint 少并不能从理论上直接否定
Dai 图，因为少量动态节点仍可能通过删除 crossing edges 从全局静态图分离。
因此没有凭数量停止，而是补做了缺失的 connected-component 拓扑实验。

新增离线只读工具：

```text
DT-SLAM/tools/audit_static_calibrated_graph_partition.py
```

对 all-transient 与 MapPoint-only 两条图分别用 Bonn 真静态前 71 个可测帧
标定 q90–q99.5 strain 工作点，后 71 帧验证静态风险，再评价冻结的
balloon/balloon2 bbox proxy。没有读取 sealed `balloon_tracking`。

核心结果：

```text
all-transient q90:
  static outside-primary P95     2.82%
  dynamic supported             14/17
  bbox recall median             9.69%
  recall >=50%                   0/14
  enrichment median              2.59
  gate                           FAILED

MapPoint-only:
  bbox >=3 nodes                 1/17
  gate                           FAILED
```

C++/Python edge-set parity 在 static/balloon/balloon2 均通过，确定性重放通过。
因此按预注册条件停止“scalar absolute strain + 静态分位阈值 + CC”的简化图
路线，不在已打开开发集上继续调阈值。

这不是 Dai 完整 point-correlation 的失败复现；当前没有实现其 edge-state
optimization、全协方差 Mahalanobis 迭代、largest-volume 或后端滑窗。

当前状态：

```text
dynamic_decision                 none
direct_slam_state_mutation       none
G1-F / G1-D                      locked
下一步                           先做总路线书面决策
```

详细记录：

```text
results/g2_4r2_2026-07-30/
  G2_4R2_DAI_GRAPH_PARTITION_FEASIBILITY_DECISION.md
  G2_4R2_STATIC_CALIBRATED_GRAPH_PARTITION_SHADOW_SPEC.md
  G2_4R2_STATIC_CALIBRATED_GRAPH_PARTITION_SHADOW_RESULT.md
```

### 18.1 下一路线已冻结

R2 后不转入完整 Dai 或重型区域系统，也不继续给 F3 加阈值。下一阶段先写：

```text
G2-5A semantic-reference、semantic-blind sparse F1 separability SPEC
```

几何残差对全部可测 ORB feature 独立计算；同步 person mask 只在离线评价时
标记 feature 所在区域，不进入几何 score。目标是用自动 reference proxy 代替
用户逐像素标注，并检验 person 开发证据能否迁移到 unknown balloon/box。

该阶段通过后仍先做 G1-F0 counterfactual，不直接删除 feature。详细决策：

```text
results/g2_4r2_2026-07-30/G2_4_POST_R2_NEXT_ROUTE_DECISION.md
```

## 19. 2026-07-30 G2-5A 自动语义参考审计完成状态

代码核查确认 F1 已对全部 ORB feature 计算 residual，semantic 只作为 CSV
事后标签，因此本阶段没有修改 C++。

新增离线工具：

```text
DT-SLAM/tools/audit_semantic_reference_sparse_flow.py
```

在线 CUDA 同步语义各运行 149 帧：

```text
walking: mask 149/149, age 0/0, actual 27.99 FPS
sitting: mask 149/149, age 0/0, actual 27.96 FPS
```

核心结果：

```text
walking comparable frames                 113
person-region median > background         95.58%
per-frame AUC median                      0.857

sitting comparable frames                 144
person-region median > background         79.17%
per-frame AUC median                      0.659
```

walking 通过预注册连续证据条件；结合冻结 balloon development 的 `15/15`
方向性结果，允许进入 G1-F0 counterfactual 设计。随后修正了一项尺度身份：
raw residual 始终 semantic-blind；初版 q scale 排除了 semantic feature，
属于 combined pipeline。权威 v2 同时报告用全部合格 feature 得到的
`q_blind` 和排除 semantic feature 的 `q_combined`。该修正不改变 raw
direction/AUC，也不推翻旧 F2 q10 hard gate 的 holdout 失败；当前仍没有部署
阈值。

当前状态：

```text
G2-5A continuous evidence       PASS
G1-F0 counterfactual            allowed next
G1-F / G1-D                     locked
dynamic decision / mutation     none / none
```

详细记录：

```text
results/g2_5a_2026-07-30/
  G2_5A_SEMANTIC_REFERENCE_SPARSE_FLOW_SHADOW_SPEC.md
  G2_5A_SEMANTIC_REFERENCE_SPARSE_FLOW_SHADOW_RESULT.md
```

## 20. 2026-07-30 G1-F0A 初始关联反事实完成状态

G1-F0A 已用 `q=6/8/10` 在六组输入上完成离线假设删除计数。两种尺度均并列
运行：

```text
semantic_blind_all_eligible
combined_semantic_excluded
```

候选始终排除 semantic feature。两种尺度、三个 q 均未出现任何
`baseline>=10 → remaining<10`，也没有跨越 10/15/20/30/50 任一支持线。
两个真静态域的 q6 MapPoint candidate rate 分别为：

```text
fr1/xyz       0.0721%
Bonn static   0.1831%
```

均低于预冻结的 0.20% 工程预算。确定性重放字节一致。

但 unknown development 的作用量很小：

```text
balloon  q6/q8/q10 = 2/0/0 MapPoints
balloon2 q6/q8/q10 = 3/2/1 MapPoints
```

因此 F0A 只通过“初始关联支持可行性”，没有证明 ATE/RPE 会改善，也没有选择
q。下一步允许设计 G1-F0B，在 `SearchLocalPoints()` 后增加默认关闭的
counterfactual instrumentation；真实过滤继续锁定。

```text
dynamic_decision            none
direct_slam_state_mutation  none
pose_reoptimization         none
G1-F / G1-D                 locked
```

详细记录：

```text
results/g1_f0_2026-07-30/
  G1_F0A_INITIAL_ASSOCIATION_COUNTERFACTUAL_SPEC.md
  G1_F0A_INITIAL_ASSOCIATION_COUNTERFACTUAL_RESULT.md
  audit_v2_scale_identity_correction/
```

## 21. 2026-07-30 G1-F0B 局部搜索后反事实完成并停止

新增默认关闭的原始关联快照：

```text
SearchLocalPoints
→ existing semantic association removal
→ post-search snapshot
→ existing PoseOptimization
→ exact mnMatchesInliers membership snapshot
```

在线 C++ 不计算 q、不删除 MapPoint、不增加优化。离线按同次运行的
`(frame,feature_index)` 与 F1 CSV 精确连接。

首版用导出时重新读取的 MapPoint 状态重建 post-pose inlier，在 fr1 frame 112
出现 `445 vs Tracking 448`。没有放宽 invariant；v2 在原计数循环同步记录
`counted_tracking_inlier`，六序列均与 `mnMatchesInliers` 精确一致。

权威 semantic-blind post-pose 结果：

```text
Bonn true static q6/q8/q10     0.3699% / 0.2680% / 0.2237%
预冻结 static budget           <=0.20%
balloon candidate MapPoints    0 / 0 / 0
balloon2 candidate MapPoints   6 / 2 / 2
30/50 support-line crossings   0
```

因此 q6/q8/q10 全部因 Bonn 静态预算失败。没有提高 q、放宽预算或打开 holdout。

```text
G1-F0B                         FAIL
dynamic decision / mutation    none / none
G1-F / G1-D                    locked
下一步                         总路线书面决策
```

详细记录：

```text
results/g1_f0_2026-07-30/
  G1_F0B_POST_LOCAL_SEARCH_COUNTERFACTUAL_SPEC.md
  G1_F0B_POST_LOCAL_SEARCH_COUNTERFACTUAL_RESULT.md
  f0b_runs_v2/
  f0b_audit_v2_exact_inlier/
```

路线决策已冻结：不再调单 feature hard threshold。优先研究 evaluation-only
`G2-6E`：若能取得 Bonn 官方 static environment model，则用官方 pose 投影
静态模型、与当前 depth 比较并排除 person mask，自动生成 unknown-foreground
review proxy。当前本地尚未找到该模型文件；实施前需先补齐官方数据并冻结坐标
链 SPEC。

```text
results/g1_f0_2026-07-30/
  G1_F0_POST_COUNTERFACTUAL_ROUTE_DECISION.md
```

## 22. 2026-07-31 G2-6E Bonn 静态模型评价路线停止于 E1

已从 Bonn 官方站点取得：

```text
1 mm subsampled static point cloud ZIP  676,032,657 bytes
ASCII PLY                               54,676,774 points
official coordinate script
ReFusion IROS 2019 PDF
```

新增 evaluation-only 工具：

```text
DT-SLAM/tools/prepare_bonn_static_model.py
DT-SLAM/tools/audit_bonn_static_model_alignment.py
```

工具流式读取 ZIP，不完整解压 2.32 GB PLY；生成 stride16/stride8 的确定性
`float32 NPY`。静态审计使用：

```text
camera_from_model =
inverse(T_ROS * interpolated_T_i * T_ROS * T_m)
```

保留官方 `T_m` 的约 1.0593 uniform scale，depth timestamp 上执行 translation
linear interpolation 与 quaternion SLERP，并投影到 G2-4B 已验证的 rectified
`P=K` 域。

首次运行发现 `cv::erode(INF)` 产生 `FLT_MAX`、被 `isfinite` 误收的问题；
失败结果保留，修正后自测试和重跑通过。

同一真静态序列的 stride/splat 对照显示：

```text
stride16/splat0 non-risk r>0.1m  56.23%
stride16/splat1 non-risk r>0.1m  42.60%
stride16/splat2 non-risk r>0.1m  37.75%
stride8/splat1  non-risk r>0.1m  40.53%
```

点云加密/splat 可改善覆盖，但不能消除随视角变化的大面积单符号残差。部分大量
有效深度的静态帧 non-risk median 达 `0.10–0.44 m`，`r>0.1 m` 达
`50%–94%`。这违反预冻结的 E1 静态对齐条件。

因此没有运行 moving-box E2，也没有连接 F1 E3。不得使用 moving 数据调坐标链、
per-frame offset 或阈值；不得把该 proxy 写成 object-motion GT。

```text
G2-6E1                         FAIL
G2-6E2 / E3                   NOT RUN
dynamic decision / mutation   none / none
G1-F / G1-D                   locked
```

详细记录：

```text
results/g2_6e_2026-07-31/
  G2_6E_BONN_STATIC_MODEL_UNKNOWN_FOREGROUND_EVALUATION_SPEC.md
  G2_6E_BONN_STATIC_MODEL_ALIGNMENT_RESULT.md
```

## 23. 2026-07-31 G2-6O 新增 Bonn 箱子序列审计完成

新增三条官方 Bonn development archive 的 ZIP 直读 association 和 RGB-only
时序审计。每序列固定选择 `8 uniform + 6 RGB-change-high +
4 RGB-change-low`，共 54 个候选；选帧不读取 geometry、flow、depth residual
或 SLAM 输出。

第一版联系表只有约 0.4 秒，无法可靠判断 `kidnapping_box`。第二版没有重选
候选，而是 exact reuse 相同 frame/role/proxy，扩到约 2 秒显示跨度。

Agent 粗审：

```text
placing 明确 moving/transition     4，全部 person present
removing 明确 moving/transition    6，全部 person present
kidnapping uncertain              12/18，均 person absent
三序列 moving+person-absent >=medium 0
```

所有标签是 `agent_rgb_temporal_review_v1` development proxy，不是 motion
GT。archive 名称没有当作逐帧标签。没有访问 sealed `balloon_tracking`。

冻结状态：

```text
G2-6O independent unknown-box qualification  FAIL
G2-6F                                        NOT RUN
G1-F / G1-D                                  LOCKED
SLAM mutation                                NONE
```

下一步不能继续按 box 名称下载并假定能解决评价缺口。若用户有 RGB-D
传感器，优先采集操作者不入镜的受控 static/moving box 小序列；否则只能另立
有文献依据的 motion-grouping development shadow，明确没有独立精度真值。

新增/修改：

```text
DT-SLAM/tools/prepare_bonn_box_motion_observability_review.py
DT-SLAM/tools/make_rgbd_association.py
results/g2_6_box_data_2026-07-31/
```

## 24. 2026-07-31 G1-F1 实验性 tracking 过滤已完成首轮评价

用户修订了研究决策：不再要求几何候选近乎零假阳性。允许少量静态关联被删除，
前提是最终 ATE/RPE、完整轨迹率和速度没有明显退化。因此 G1-F0 预注册的
`<=0.20%` 静态预算保留为历史诊断，但不再作为绝对阻断条件。

已实现：

```text
G2-4F1 sparse ego-flow residual
→ semantic-blind robust frame scale
→ q6/q8/q10 candidate
→ TrackLocalMap SearchLocalPoints 后
→ existing semantic removal 后
→ second existing PoseOptimization 前
→ 清除少量 mvpMapPoints association
```

保护：

```text
default OFF
relocalization window fail-open
minimum 30 associations
maximum 5% candidate fraction
invalid scale/vector fail-open
no third PoseOptimization
no mapping veto
```

静态 `fr1/xyz` 四组轨迹均完整，q6/q8/q10 相对 control 的 ATE/RPE 变化约
`-0.2%` 到 `+1.7%`。

`fr3/walking_xyz` 在线 CUDA semantic+geometry 三轮均完整：

| 模式 | ATE RMSE 中位数 | RPE RMSE 中位数 | actual FPS 中位数 |
|---|---:|---:|---:|
| semantic control | 0.017699 m | 0.012224 | 27.4836 |
| q6 | 0.014927 m | 0.011882 | 27.3397 |
| q8 | 0.016116 m | 0.011923 | 27.3607 |
| q10 | 0.015462 m | 0.011986 | 27.3472 |

三个 q 在该序列的 ATE 中位数改善约 `8.9%–15.7%`，RPE 改善约
`2.0%–2.8%`，FPS 代价约 `0.5%`。这些结果允许保留 G1-F1，但不足以选择
最终 q 或声明未知箱子已解决。

41 个 CSV、26,928 个 frame row 的 safety audit：

```text
invariant violations = 0
pose_reoptimization  = none
mapping_veto         = none
```

当前状态：

```text
G1-F1 experimental tracking filter   KEEP, default OFF
experimental working point           q10 (conservative)
unknown-box claim                    NOT ESTABLISHED
G1-M / G1-D                          LOCKED
```

`fr3/sitting_static` 的三轮 semantic-control/q6/q8/q10 也全部覆盖 680 帧。
相对 semantic control，ATE 中位变化为：

```text
q6   -2.10%
q8   +3.82%
q10  -3.37%
```

没有达到约 10% 的工程退化线。

Bonn balloon 也完成三轮 12 次完整轨迹：

```text
q6   ATE -5.41%, RPE +2.14%
q8   ATE -6.44%, RPE +0.66%
q10  ATE -1.77%, RPE +0.17%
```

三条 semantic 序列均无明显 ATE 退化。q10 虽不是单序列最好值，但删除最少，
Bonn RPE 变化最小，故冻结为后续保守实验工作点；q6/q8 保留为固定消融，配置
默认仍关闭。

下一步：

1. 进入 G1-M0 MapPoint 写入 counterfactual，只统计 q10 候选可能阻止的
   RGB-D initialization / CreateNewKeyFrame MapPoint；
2. 不修改地图，不顺带实现 G1-D；
3. 只有 counterfactual 证明实际作用且安全条件明确，才设计默认关闭的
   mapping veto；
4. depth-region 过滤继续锁定。

详细记录：

- `results/g1_f1_2026-07-31/G1_F1_SPARSE_EGO_FLOW_TRACKING_FILTER_SPEC.md`
- `results/g1_f1_2026-07-31/G1_F1_SPARSE_EGO_FLOW_TRACKING_FILTER_RESULT.md`
- `results/g1_f1_2026-07-31/G1_F1_FORMAL_METRICS.csv`
- `results/g1_f1_2026-07-31/filter_invariant_audit.json`

## 25. 2026-07-31 G1-M0 MapPoint admission counterfactual 完成

G1-F1 完成后，继续执行原始目标中的独立地图写入检查。当前调用图确认：

```text
G1-F1 清除 association
→ NeedNewKeyFrame
→ CreateNewKeyFrame
→ 无 MapPoint 且有 RGB-D depth
→ 同一 feature 可被重新创建为 MapPoint
```

新增默认关闭、只读的 G1-M0：

```text
Geometry.SparseFlowMappingCounterfactualEnable
DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL
DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL_CSV
```

只允许与 RGB-D、G1-F1 q10 同时启用。记录初始化、新关键帧、候选 MapPoint
创建及 tracking 删除后重建，不设置 dynamic flag、不 veto 地图。

在线 CUDA semantic，三轮中位：

| 序列 | q10 candidate MapPoint | candidate/全部创建 | 删除后重建 |
|---|---:|---:|---:|
| walking | 653 | 2.411% | 141 |
| sitting_static | 3 | 0.092% | 0 |
| Bonn balloon | 202 | 1.714% | 25 |

10 个 CSV、1,038 个写图事件全部通过：

```text
candidate created              2,503
tracking removed then recreated  461
invariant violations               0
mapping mutation                  none
mapping veto                      none
```

首帧没有上一帧参考，F1 几何不可用；不能伪造初始化保护。

当前状态：

```text
G1-M0                              PASS
candidate admission                NON-ZERO, REPEATABLE
G1-M1                              JUSTIFIED, NOT IMPLEMENTED
G1-D                               LOCKED
```

下一步 G1-M1 应复用统一 `mvbDynamic`：

1. 只在成功跟踪且准备创建 KeyFrame 时融合通过安全条件的 q10 candidate；
2. 在 KeyFrame 构造前调用既有 `RemoveDynamicAssociations()`；
3. 复用 RGB-D `!mvbDynamic` admission 与现有 LocalMapping 两端 guard；
4. 默认关闭，重新跑 semantic-control / F1 / F1+M1；
5. 不修改 Optimizer/g2o/YOLO，不引入另一套后端状态。

详细记录：

- `results/g1_m0_2026-07-31/G1_M0_MAPPOINT_ADMISSION_COUNTERFACTUAL_SPEC.md`
- `results/g1_m0_2026-07-31/G1_M0_MAPPOINT_ADMISSION_COUNTERFACTUAL_RESULT.md`
- `results/g1_m0_2026-07-31/G1_M0_FORMAL_METRICS.csv`
- `results/g1_m0_2026-07-31/mapping_counterfactual_audit.json`

## 26. 2026-07-31 G1-M1 稀疏 MapPoint 写入保护完成

默认关闭的 G1-M1 已实现并通过四类序列验证。它只在
`CreateNewKeyFrame()` 构造 KeyFrame 前，将通过安全条件的 q10 candidate
并入已有 `mCurrentFrame.mvbDynamic`，随后复用现有 association 清理、
RGB-D depth admission 和 LocalMapping dynamic endpoint guard。

新增配置：

```text
Geometry.SparseFlowMappingFilterEnable
Geometry.SparseFlowMappingFilterMaximumFeatureFraction
Geometry.SparseFlowMappingFilterMaximumDepthFraction
Geometry.SparseFlowMappingFilterMinimumRemainingDepthFeatures
```

新增环境变量：

```text
DT_SLAM_GEOMETRY_MAPPING_FILTER
DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV
```

限制：

```text
RGB-D + G1-F1 q10
G1-M0 与 G1-M1 不可同时启用
首帧 reference unavailable，几何 fail-open
feature/depth candidate fraction <= 5%
remaining valid depth >= 100
```

三轮中位数：

| 序列 | ATE | RPE | FPS | 轨迹 |
|---|---:|---:|---:|---:|
| walking | 0.017864 | 0.012343 | 27.479 | 827/827 |
| sitting | 0.006596 | 0.005669 | 27.890 | 680/680 |
| Bonn balloon | 0.032519 | 0.041264 | 29.538 | 438/438 |
| fr1/xyz | 0.009643 | 0.005777 | 29.624 | 792/792 |

相对各自 control，ATE/RPE 中位变化均在约 3% 内。12 次正式运行合计
2,619 个新 dynamic flag、2,441 个有效深度 veto，所有 applied 行的
candidate-created MapPoint 为 0，全部审计无违反。

Viewer ON 的 G1-M1 和关闭几何过滤的语义 control 均在轨迹保存完成后
exit 139。该对照说明退出问题与 G1-M1 无关；正式 Viewer OFF 运行均正常。
不要在本阶段顺带修改 Viewer。

当前交接：

```text
G1-F1 + G1-M1 sparse frontend   READY FOR EXPERIMENTS, default OFF
mapping cleanliness GT          not measured
G1-D pixel/depth mask           still unresolved
Optimizer/g2o/YOLO/backend      unchanged
```

下一步先冻结可复现使用说明。若继续研究，优先选择：

1. 地图质量定性/定量评价；
2. 或重新定义有文献依据的 G1-D，但不得把 sparse candidate 直接膨胀成
   dense dynamic mask。

详细记录：

- `results/g1_m1_2026-07-31/G1_M1_MAPPOINT_ADMISSION_FILTER_SPEC.md`
- `results/g1_m1_2026-07-31/G1_M1_MAPPOINT_ADMISSION_FILTER_RESULT.md`
- `results/g1_m1_2026-07-31/G1_M1_FORMAL_METRICS.csv`
- `results/g1_m1_2026-07-31/mapping_filter_audit.json`

## 27. 2026-07-31 G1 稀疏地图质量评价完成

新增默认关闭的 read-only MapPoint lifecycle audit：

```text
Geometry.SparseFlowMapQualityAuditEnable
DT_SLAM_GEOMETRY_MAP_QUALITY_AUDIT
DT_SLAM_GEOMETRY_MAP_QUALITY_PREFIX
```

它仅允许在 `RGB-D + G1-F1 q10 + (G1-M0 xor G1-M1)` 下启用，在 shutdown
阶段沿 replacement chain 统计 candidate MapPoint 是否最终存活，并输出 final
MapPoint/KeyFrame/observation 摘要。没有直接 map mutation。

walking 三轮中位数从 M0 的 `711 created / 25 survived` 变为 M1 fail-open 的
`174 created / 5 survived`；M1 中位应用 92 个 KeyFrame event，否决 662 个
有效深度候选。ATE 中位数由 0.019059 m 变为 0.016745 m，但 RPE 由
0.012335 变为 0.012486，不能声称定位稳定改善。

fr1/xyz、Bonn balloon、sitting 各完成一轮成对检查，轨迹覆盖全部完整；没有
发现静态地图灾难性退化。注意 q10 candidate 不是动态 GT，ORB-SLAM2 又会自然
剔除绝大多数临时 MapPoint，因此当前只能确认 admission protection 生效，不能
声称已经精确测得动态地图污染或显著提高地图精度。

交接状态：

```text
G1-F1 tracking filter                experimentally usable, default OFF
G1-M1 MapPoint admission filter      experimentally usable, default OFF
final-map proxy audit                complete, read-only
exact map dynamic GT                 unavailable
G1-D pixel/depth filtering           locked
```

详细记录：

- `results/g1_map_quality_2026-07-31/G1_SPARSE_MAP_QUALITY_EVALUATION_SPEC.md`
- `results/g1_map_quality_2026-07-31/G1_SPARSE_MAP_QUALITY_EVALUATION_RESULT.md`
- `results/g1_map_quality_2026-07-31/G1_MAP_QUALITY_FORMAL_METRICS.csv`
- `DT-SLAM/tools/audit_sparse_map_quality.py`

## 28. 2026-07-31 G1 四模式首轮结果

新增 `DT-SLAM/tools/run_sparse_frontend_mode.py`，用同一二进制冻结四模式运行。
TUM3 walking 首轮显示：

```text
orb baseline        816/827, ATE 0.926133
semantic-only       827/827, ATE 0.019142
geometry-only       587/827, ATE 0.533014
semantic+geometry   827/827, ATE 0.018693
```

geometry-only 和 ORB baseline 额外重复两次后仍有严重 ATE 和覆盖波动，因此纯
几何只保留为消融诊断，不作为可用系统。semantic+geometry 完整运行，mask age
为 0，移除 537 个 association、否决 634 个有效深度写图候选，说明组合中的
几何路径确实生效；但 ATE 稳定改善仍无证据。

下一步不得继续调 q10/5% 追逐 geometry-only walking。组合模式可默认关闭地
用于实验；未知箱子定量结论仍需要合适的可观察数据/真值，G1-D 继续锁定。

详细记录：

- `results/g1_release_2026-07-31/G1_SPARSE_FRONTEND_FOUR_MODE_FREEZE_SPEC.md`
- `results/g1_release_2026-07-31/G1_SPARSE_FRONTEND_FOUR_MODE_RESULT.md`
- `results/g1_release_2026-07-31/G1_WALKING_FOUR_MODE_METRICS.csv`

Viewer ON 组合模式完整处理 827/827 帧并达到轨迹保存阶段，随后按既有
Viewer/Pangolin 问题返回 `-11`（shell 245）。不要把它归因于本次几何过滤，也
不要在 G1 冻结阶段顺带修改 Viewer。

## 29. 2026-07-31 Bonn moving_nonobstructing_box 判别性检查完成

在 Bonn 联合校正 `P=K` 域中，对 semantic-only 与 semantic+geometry 各运行
三次，均在线 CUDA YOLO、mask age 0、轨迹 778/778。三轮中位：

| 模式 | ATE | RPE | FPS |
|---|---:|---:|---:|
| semantic-only | 0.152247 m | 0.051377 m | 29.707 |
| semantic+geometry | 0.178114 m | 0.045061 m | 29.544 |

组合模式相对中位 ATE `+16.99%`、RPE `-12.29%`、FPS `-0.55%`。首轮 ATE
改善没有在重复运行中成立，不能声称稳定定位收益。

三轮中 G1-F1 移除 1,519 个 association，G1-M1 否决 284 个有效深度写图
候选，所有不变量通过。为回答这些点在哪里，新增默认关闭的精确空间诊断和
review 工具；它不改变算法。Viewer ON 完整运行导出 612 个真实移除点。复用
24 个粗略、未验证箱框抽查时，共审查 13 个移除点，0 个在粗箱框、0 个在人物
mask、13 个在二者之外；联系表主要显示背景位置。

因此当前状态必须表述为：

```text
sparse filter implementation/action       confirmed
safeguards and runtime cost                acceptable
stable ATE benefit                         not supported
unknown moving-box spatial hit             not observed in sampled proxies
validated unknown-object detector          no
```

下一步只做一个有停止条件的 `moving_obstructing_box` 开发诊断：先看精确移除点
是否稳定落在更强遮挡箱子附近，再决定是否值得三轮 ATE/RPE。若仍主要命中背景，
冻结 q10 稀疏过滤为有限/负面 baseline，转回有文献依据的运动分组或对象候选；
不得继续调 q10/5%，不得开放 G1-D。

详细记录：

- `results/g1_bonn_box_2026-07-31/G1_BONN_MOVING_NONOBSTRUCTING_BOX_SPEC.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_MOVING_NONOBSTRUCTING_BOX_RESULT.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_MOVING_NONOBSTRUCTING_BOX_METRICS.csv`
- `DT-SLAM/tools/prepare_sparse_flow_removed_association_review.py`

## 30. 2026-07-31 Bonn 箱子证据漏斗与强遮挡诊断

新增默认关闭的逐特征证据审计，将现有 G1-F1 拆成：ORB、measured、quality
eligible、冻结 q10、post-SearchLocalPoints association 和实际 removal。新增
字段/CSV 只读，不改变候选或 SLAM。

`moving_nonobstructing_box` 的 24 个粗箱框内有 4,564 个 quality-eligible
特征，但 q10 candidate 为 0；证据在 residual 判决处消失，不是低纹理、深度
无效或 MapPoint association 问题。

`moving_obstructing_box` 完整运行 589/589、online CUDA mask age 0、29.685 FPS。
17 个强箱体可见粗框内有 5,530 个 eligible、26 个 q10 candidate，最终有 2 个
post-search association 且 2 个都被移除、均落在粗箱框内；这是少量正面证据。
但 semantic-static eligible 的 q10 比率在框内仅 0.472%，框外为 5.888%，框外
约高 12.48 倍。raw 单点 residual 明显缺少目标特异性。粗框不是 GT，因此不报
precision/recall。

决策：不降低 q、不调 5%、不做该序列三轮 ATE，不开放 G1-D。G1-F1/G1-M1
保留为默认关闭的实验 baseline；下一方法工作转向有文献依据的空间/短时序运动
一致性分组或对象候选，先做本地文献审计和 shadow SPEC。

详细记录：

- `results/g1_bonn_box_2026-07-31/G1_BONN_BOX_FEATURE_EVIDENCE_FUNNEL_SPEC.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_BOX_FEATURE_EVIDENCE_FUNNEL_RESULT.md`
- `results/g1_bonn_box_2026-07-31/G1_BONN_BOX_EVIDENCE_FUNNEL_AND_OBSTRUCTING_RESULT.md`
- `DT-SLAM/tools/audit_sparse_flow_evidence_funnel.py`

## 31. 2026-08-01 共同运动分组输入审计

本地核对发现 7 月 30 日已完成 balloon/balloon2 的短轨迹支持审计，因此没有
重复增加阶段。本轮补充 Lee 2019 原文核对，并在 Bonn 箱子上检查 quality-
eligible 连续二维 ego-flow residual 是否具有共同运动结构。

初版把 moving/stationary 箱框混合统计，解释作废并完整保留。修正仅连接此前
未看 geometry/flow 时冻结的 RGB temporal proxy：非遮挡序列 5 个 moving、
19 个 stationary 帧。moving 框内 residual magnitude、direction concentration、
centroid separation 中位分别为 `1.682 px / 0.975 / 1.934 px`；stationary 为
`0.235 px / 0.676 / 0.190 px`。对应描述性 proxy AUC 为
`0.989 / 0.947 / 0.989`。

5/5 moving 帧具有多点 coherence，但同一简单条件也在 10/19 stationary 帧
触发，所以 coherence 不能直接成为动态判决。当前只允许继续设计 Lee-style
三维 rigid-motion hypothesis shadow；不允许二维 DBSCAN、阈值选择或 G1 状态
写入。

详细记录：

- `results/g2_motion_grouping_next_2026-08-01/G2_MOTION_GROUPING_LITERATURE_AND_PRIOR_EVIDENCE_AUDIT.md`
- `results/g2_motion_grouping_next_2026-08-01/G2_BOX_TWO_FRAME_MOTION_COHERENCE_INPUT_RESULT.md`
- `DT-SLAM/tools/audit_sparse_motion_coherence.py`

## 32. 2026-08-01 G2-MH1 三维刚体假设 SPEC 冻结

在输入审计确认“少量明确运动箱子帧存在多点连续运动结构，但二维 coherence
不能排除静态表面”后，下一项没有直接实现二维 DBSCAN，而是核对 Lee 2019
原文并冻结稀疏三维刚体假设 shadow。

当前代码已经由 F1/F3 提供：两帧 LK 对应、FB 质量、参考/当前米制深度、两帧
三维点、语义排除和背景相机相对位姿。因此下一实现应复用
`GeometricRigidityResult::nodes`，不再写第二套光流或深度接口。

冻结的第一版为：每个有效 anchor 加当前图像中最近 6 个有效点，使用标准
SVD/Kabsch 估计 7 点刚体变换，同时比较局部模型与背景相机模型在同一点集上的
连续三维拟合误差。它只输出 hypothesis、退化原因、局部半径、拟合质量和时间；
不做 refinement、DBSCAN、对象标签、时序模型或 SLAM 状态写入。

下一实施边界：

```text
next implementation       G2-MH1 local 3-D hypothesis shadow only
dynamic decision          none
hypothesis clustering     not yet
G1-F/G1-M/G1-D            unchanged / default OFF or locked
Optimizer/g2o/YOLO        unchanged
```

详细 SPEC：

- `results/g2_motion_grouping_next_2026-08-01/G2_MH1_SPARSE_3D_RIGID_MOTION_HYPOTHESIS_SHADOW_SPEC.md`

## 33. 2026-08-01 G2-MH1 实现与首轮审计完成

已实现 Lee 2019 局部刚体 hypothesis 原型的稀疏适配：每个 F1/F3 有效 anchor
加最近 6 点，用 Kabsch 拟合 7 点 SE(3)，并与背景相机模型在同一点集上的误差
成对记录。实现默认关闭且严格 shadow-only；没有动态阈值、聚类、时序标签或
SLAM 状态写入。合成测试、完整构建、CSV 和 SE(3) 不变量均通过。

TUM fr1/xyz 真静态 smoke 暴露重要风险：局部模型误差中位 `0.00200 m`，背景
模型 `0.00543 m`，RMS ratio `1.959`，同时产生不真实的相对平移/旋转中位
`0.179 m / 0.207 rad`。说明 7 点自由模型会吸收静态深度/LK/位姿噪声。

Bonn 非遮挡箱子 5 moving / 19 stationary proxy 中，background fit raw AUC
`0.979`，但箱内减箱外后为 `0.642`；局部 improvement 的两个 AUC 为
`0.768 / 0.505`，RMS ratio 为 `0.800 / 0.663`。本次 Codex 环境不能初始化
CUDA，Bonn 运行是 geometry-only，并受人物混杂；所有 AUC 仅为开发排序。

模块额外耗时约 `21--30 ms`，完整 pipeline 约 `18.8--20.1 FPS`。当前最强
信号仍是已有背景运动不一致，局部 7 点模型尚未证明补上对象分组层。因此不进入
DBSCAN，不选择阈值，不开放 G1-F/G1-D。下一步先核对 Lee hypothesis
refinement/support verification 能否形成忠实轻量适配，否则冻结该路线为有限/
负面结果。

原文复核确认：Lee 的 7 点只是初始 seed hypothesis，之后对全部 `n` 个 grid
scene-flow vector 计算误差、扩展到 `N` 点并重新估计，最后才聚类。因此当前负面
结果针对“7 点训练内拟合直接用于判断”，不能外推为完整 Lee 路线失败。下一步若
继续，必须先写 support/refinement shadow SPEC，不能直接加 DBSCAN。

详细记录：

- `results/g2_motion_grouping_next_2026-08-01/G2_MH1_SPARSE_3D_RIGID_MOTION_HYPOTHESIS_SHADOW_RESULT.md`
- `DT-SLAM/tools/audit_sparse_rigid_hypotheses.py`
- `DT-SLAM/tools/audit_sparse_rigid_hypothesis_proxy.py`

## 34. 2026-08-01 独立支持验证完成，停止稀疏 7 点刚体路线

按照实现前冻结的 support-validation SPEC，7 点 seed hypothesis 改为在另外 7 个
未参与拟合的邻近点和全部非训练点上，与背景相机模型作成对验证。没有引入阈值、
重估、DBSCAN、动态标签或 SLAM 状态修改。

TUM fr1/xyz 真静态中，训练内 improvement 中位为 `+0.001951 m`，独立 holdout
变为 `-0.001892 m`，RMS ratio 从 `1.954` 变为 `0.772`；证明先前优势确属训练
内过拟合。Bonn moving proxy 箱内的 holdout RMS ratio 高于 stationary
（`0.881/0.585`），但 moving 五帧该比值仍全部小于 1，且通常只有 1--3/7 个
holdout 点支持局部模型。它是弱相对排序，不是可重估的共同运动支持集。

独立支持审计额外中位耗时约 TUM `19.80 ms`、Bonn `26.55 ms`；Bonn 完整
shadow pipeline 为 `12.79 FPS`。按冻结停止规则，不做支持集重估、DBSCAN 或新
经验条件，停止当前稀疏 7 点刚体 hypothesis 路线。

总计划未新增阶段，仍停留在第 1 个剩余模块“可靠运动组判决”。G1-F/G1-M 的旧
实验路径保持默认关闭，G1-D 继续锁定。

详细记录：

- `results/g2_motion_grouping_next_2026-08-01/MOTION_GROUP_SUPPORT_VALIDATION_SPEC.md`
- `results/motion_group_support_validation_2026-08-01/MOTION_GROUP_SUPPORT_VALIDATION_RESULT.md`

## 35. 2026-08-01 当前轻量几何路线完成有限收尾

用户决定先对当前路线作简单收尾，开源 SInDSLAM 留作后续独立研究。本轮没有
新增检测算法、阈值或 SLAM 修改，只复用已经完成的正式 ATE/RPE/FPS、箱子证据
和地图生命周期实验，并补做当前工作树构建、测试及 30 帧默认/几何回归。

收尾结论：同步 semantic baseline 可作为默认主线；F1 sparse ego-flow、G1-F1
association removal 和 G1-M1 MapPoint admission veto 工程有效，可作为默认关闭
的实验模式。semantic+geometry 可完整运行，但没有稳定 ATE/RPE 改善证据；
geometry-only 不具备独立使用资格；未知箱子对象检测和 G1-D 深度区域仍未完成。

TUM walking 单轮组合模式相对 semantic-only 的 ATE 为 `-2.35%`、RPE
`+3.90%`、FPS `-4.17%`，不能宣称稳定改善。Bonn nonobstructing 三轮中位组合
模式 ATE `+16.99%`、RPE `-12.29%`、FPS `-0.55%`，且抽查未观察到箱子命中。
强遮挡箱子只有 2 个实际 association 在粗框内被删除，同时 raw q10 主要位于框外。

当前工作树回归构建和测试通过；标准/几何 30 帧 smoke 都输出 29/29 位姿，新
rigid-hypothesis shadow 均未启动。当前 q10 filtering 继续默认关闭。后续若继续
未知动态对象目标，应把 SInDSLAM 作为明确的重型方法升级，而不是继续补当前
F1/q10。

详细记录：

- `results/geometry_limited_closeout_2026-08-01/GEOMETRY_LIMITED_CLOSEOUT_SPEC.md`
- `results/geometry_limited_closeout_2026-08-01/GEOMETRY_LIMITED_CLOSEOUT_RESULT.md`

## 36. 2026-08-05 R0基线冻结与R1 Gazebo失败层审计完成

R0已把Gazebo 600帧、配置、runner和评价协议冻结。纯ORB-SLAM2与全零语义
路径三轮配对实验全部输出600/600帧；ATE差异方向随轮次改变，因此此前单轮差异
不能归因于全零语义mask。R0 commit为`6c29ff8`，尚未推送。

R1只增加显式开启的只读导出，记录DeepFlow、current-to-reference homography、
残差、初始/梯度/RAG区域、逐区域分类原因、上一帧证据和最终mask。30帧开关对照
中151个非计时检测字段零差异，29张动态深度mask逐像素一致；合成测试通过。

完整600帧审计得到199个主要可见箱子帧。56帧箱体深度超过配置的6 m区域上限，
因此没有箱体区域；剩余143帧中有82帧区域已经覆盖箱体中位95.9%、主区域纯度
中位100%，但箱体high残差覆盖为0，全部被`insufficient_high_pixels`拒绝。
近距离高运动阶段有58帧最终箱体覆盖超过75%，其中位覆盖约99.99%。可见箱体
oracle区域仍有137/199帧high证据不足，故R1停止继续修改RAG、flood fill或最少
high像素数。

联系表同时显示地面、顶面和墙面存在大块残差；箱外含运动行人，不能把全部
non-box统计称为静态误检。下一步按原计划只放行R2：固定DeepFlow、区域、
classifier和temporal prior，对比current/oracle homography与Gazebo参考/在线
SLAM RGB-D SE(3)补偿。R2首轮仍不改变SLAM。

详细记录：

- `results/r0_freeze_2026-08-05/R0_REPRODUCIBLE_BASELINE_SPEC.md`
- `results/r0_freeze_2026-08-05/R0_BASELINE_EQUIVALENCE_RESULT.md`
- `results/r1_gazebo_failure_layer_2026-08-05/R1_READ_ONLY_FAILURE_LAYER_AUDIT_SPEC.md`
- `results/r1_gazebo_failure_layer_2026-08-05/R1_GAZEBO_FAILURE_LAYER_AUDIT_RESULT.md`
- `DT-SLAM/tools/audit_r1_gazebo_failure_layers.py`

## 37. 2026-08-05 R2 运动补偿单变量对照完成

R2固定R1导出的599帧CPU DeepFlow、参考帧和箱体参考，不重新
运行flow、不改变region/classifier/temporal prior，只离线对比五种
相机运动补偿：当前PROSAC homography、oracle-static homography、Gazebo
参考RGB-D/SE(3)、ORB-SLAM2事后轨迹SE(3)和历史常速度预测SE(3)。

方向和数值不变量已通过；当前homography离线重算与R1量化残差的最大
差为`0.007813 px`。Gazebo参考SE(3)在598/598个可比帧中均降低
共同有效像素残差，中位数从`1.137 px`降至`0.406 px`。近3 m箱体
high证据从84.97%提高至93.21%，箱外high从3.56%降至1.45%。

但该结果只是几何上限。SIn风格detector在`Track()`之前运行，当帧最终
ORB位姿当时不存在。事后SLAM SE(3)在393/598帧比当前homography残差更高，
历史常速度SE(3)在506/598帧更高；箱外high比例中位数分别为28.10%
和31.02%。两者均不可上线。即使使用Gazebo参考SE(3)，中距离和远距离
箱体high比例中位数仍为0，说明SE(3)不是远小箱子召回的充分解。

结论：R2证明全局homography的模型能力是背景残差的重要限制，但没有
产出可部署的在线替换。根据预先冻结的停止条件，不把SE(3)接入正式
detector，不新增一次Tracking或第三次PoseOptimization，不进入ATE系统
实验，R3区域/分类器改造暂停。

当前唯一后续是书面路线决策：若优先完成长期静态地图，可进入有SInDSLAM
原文依据的R4/S4长时间隔Mapping精修；它只解决慢速/停留物体残影，
不宣称修复远小箱子。OctoMap、plane edge、新flow和新融合仍未放行。

详细记录：

- `results/r2_ego_compensation_2026-08-05/R2_EGO_COMPENSATION_SHADOW_SPEC.md`
- `results/r2_ego_compensation_2026-08-05/R2_EGO_COMPENSATION_SHADOW_RESULT.md`
- `DT-SLAM/tools/audit_r2_ego_compensation.py`
- `results/r2_ego_compensation_2026-08-05/analysis_final/`

## 38. 2026-08-05 R4/S4 Mapping-only shadow完成并停止

R2后先核对SInDSLAM原文、PaperNotes和公开Mapping源码。原文明确用每5帧
取1帧的深度重投影处理慢速、间歇或短暂停留物体；公开`pubPointCloud.cc`
中可核对到约0.13的相对深度条件和0.4的区域支持逻辑，但ROS关键帧缓存、
降采样计数和前一mask传播与论文文字并不完全一一对应。

因此R4实现为`paper-text-guided clean-room S4 replay`：仅离线复用冻结
Gazebo 600帧的R1 mask、RAG labels和同一SLAM轨迹；每5个输入帧选一个
sparse frame，用`tau_rep=0.13`和`tau_4=0.4`生成独立refined mapping mask。
它不参与Tracking，不改S1–S3，不接OctoMap。

119组sparse frame对中有40个箱体主要可见帧。29个当前S3漏检帧中，
只有11帧出现任意新增箱体像素，没有一帧恢复至25%覆盖，最大最终覆盖
16.42%。中距离3–6 m的新增箱体覆盖中位数仅0.14%；S4新增有效深度
比例均值3.71%、第90百分位10.44%，却没有形成箱体恢复。整区域扩展仅
发生在8/119帧。

根据实施前冻结的停止条件，R4归档为有限负结果：不调`0.13/0.4`，
不搜索时间间隔，不生成正式S3+S4点云，不接在线Mapping。R5 OctoMap也不
放行，因为它不能补齐当前detector的中远距离漏检。

当前进入R6收尾：冻结S1–S3主基线，整理现有TUM/Bonn/Gazebo轨迹、对象、
深度和耗时证据，确定论文定位。不再新增检测模块、阈值或地图后端。

详细记录：

- `results/r2_ego_compensation_2026-08-05/R2_POST_DECISION.md`
- `results/r4_s4_mapping_refinement_2026-08-05/R4_S4_LONG_INTERVAL_MAPPING_REFINEMENT_SHADOW_SPEC.md`
- `results/r4_s4_mapping_refinement_2026-08-05/R4_S4_LONG_INTERVAL_MAPPING_REFINEMENT_SHADOW_RESULT.md`
- `DT-SLAM/tools/audit_r4_long_interval_mapping_refinement.py`
- `results/r4_s4_mapping_refinement_2026-08-05/analysis/`

## 39. 2026-08-05 R6最终证据汇总完成

R6没有新增算法或重跑选择性实验，而是把已有TUM、Bonn、Gazebo轨迹、
Tracking、MapPoint、深度、点云和耗时结果按证据强度分为A/B/C/N四级。

最强A级证据仍是Bonn `moving_nonobstructing_box` S2三轮控制对照：不过滤
ATE中位`0.514344 m`，区域几何过滤后`0.022526 m`；三轮方向和量级
稳定。Bonn强遮挡的最终fail-open版一轮完整轨迹，ATE从`0.546996 m`
降至`0.245857 m`，但只作B级有限证据。TUM六序列四模式数值为单轮C级。

Mapping侧的客观结论是：S3在Bonn同位姿点云中明显减少人物重影，但
Gazebo中箱体逐帧覆盖中位数为0，箱子轨迹残影仍存在。R1/R2/R4分别
排除了“优先修RAG”、“直接上线SE(3)”和“固定5帧S4能恢复箱子”。

当前研究完成度：YOLO已知类别、SIn风格几何、动态ORB过滤、MapPoint写入
否决和动态深度输出链已完成；跨域稳定未知箱子检测、长期干净稠密地图和
30 FPS均未完成。论文定位应为系统集成、可复现实现和跨域失效分析；
不能宣称已经形成跨域成立的新核心检测算法。

本轮R0–R6计划到此完成。S1–S3保留为实验主基线；轻量LK、R2 SE(3)、
R4 S4和OctoMap均保持关闭。后续若继续方法研究，应作为新的立项决策，
不继续给R0–R6追加补丁阶段。

详细记录：

- `results/r6_final_evaluation_2026-08-05/R6_FINAL_EVALUATION_SCOPE.md`
- `results/r6_final_evaluation_2026-08-05/R6_FINAL_EVIDENCE_SUMMARY.md`
- `results/r6_final_evaluation_2026-08-05/r6_key_results.csv`

## 40. 2026-08-06 固定轨迹离线 Mapping 可行性验证

R6之后没有重新开放检测算法。为回答普通累计点云为何永久保留Gazebo箱子轨迹，
本轮固定同一RGB-D、纯ORB-SLAM2轨迹、采样与体素参数，只比较端点累计、时间
支持和标准OctoMap occupied/free更新。10 cm主实验中，不使用动态mask的
OctoMap已将箱子专属残影候选从100%降到2.39%；加入当前S3后降到0%。约
92.17%的旧箱子体素在箱子离开后确实被后续非箱子射线重新穿过，支持“自由空间
反证撤销历史占用”的解释。

补充的S3＋时间支持对照中，S3直接累计仍保留75.17%箱子候选，S3＋至少3帧
降到14.71%，S3＋至少8帧降到0%。固定3/8帧是项目级`[S/H]`工作点，不是论文
参数；当前0%只对0.5 m/s匀速箱子、约10 Hz采样与10 cm体素成立。OctoMap与
时间支持语义不同：前者能用free射线撤销旧占用，后者只延迟确认重复端点。

该实验是离线固定轨迹可行性验证，不是在线Mapping，也没有证明静态地图真实
completeness。≥8帧持续非箱子保留率在纯ORB轨迹下约69.9%（OctoMap），其中
仍混有动态人物和位姿量化影响。详细记录：

- `results/offline_mapping_feasibility_2026-08-06/OFFLINE_FIXED_TRAJECTORY_MAPPING_FEASIBILITY_RESULT.md`
- `DT-SLAM/tools/offline_gazebo_occupancy_audit.cc`
- `/data/dynaslam/large_results/offline_mapping_feasibility_2026-08-06/`

## 41. 2026-08-06 AWS Small House 正式人与箱子序列四模式完成

为降低旧回字走廊重复砖纹理、物体稀少和疑似错误回环的影响，本轮在AWS Small
House录制316.6 s、6328对精确同步RGB-D。人物与箱子均自动持续运动，机器人由
用户人工驾驶约38.76 m；路线不闭环，起终点相距约4.18 m。转换、深度尺度、
内参和相机真值方向均通过检查。

四模式单轮完整结果如下：纯ORB ATE/RPE为0.059334/0.005624 m、覆盖94.82%；
仅语义为0.055089/0.004836 m、覆盖96.19%；仅SIn风格区域几何为
0.056881/0.006505 m、覆盖97.09%；语义OR几何为0.057982/0.005938 m、覆盖
97.42%。仅语义单轮ATE/RPE最好，组合覆盖最高；几何与组合没有稳定降低局部
误差，简单OR未优于仅语义。由于每模式只有一轮且覆盖时间戳集合不同，小幅差异
不能写成稳定增益。

语义两次均为6328/6328同帧mask，age中位数/最大值均为0，约20 FPS。几何在
2491帧产生非零动态区域，总体拒绝约6.77%有效深度；组合拒绝约8.05%，并触发
54帧fail-open。CPU DeepFlow平均约156 ms，使仅几何和组合分别只有4.57和
4.38 FPS；GPU YOLO约9.6 ms，不是当前主瓶颈。

当前新场景的Tracking/检测/性能闭环已完成；正式计时未写出全序列S3 mask，
因此新场景点云残影、时间支持和OctoMap对照尚未完成。详细记录：

- `results/aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_PERSON_BOX_FOUR_MODE_RESULT.md`
- `/data/dynaslam/datasets/aws_small_house_person_box_formal_run1_20260806/`
- `/data/dynaslam/datasets/aws_small_house_person_box_tum_run1_20260806/`

## 42. 2026-08-07 AWS完整S3导出与Mapping复核

AWS Small House 6328帧组合模式S3导出已正常完成，返回码0。6328/6328帧完成检测，
语义mask age中位数和最大值均为0；6165帧具有有效Mapping输出并写出动态深度mask，
163帧因无有效Mapping位姿按设计不输出。包含逐帧PNG和完整诊断写盘的任务实际吞吐
3.27 FPS，不能替代不写mask的正式4.38 FPS计时。

固定同一条组合模式轨迹，以3081个采样帧、10 cm体素和每8像素采样比较后：普通累计
箱子残影代理100%，S3直接累计96.35%，至少8帧不用S3为11.60%，S3＋至少8帧为
8.38%，OctoMap不用mask为42.43%，S3＋OctoMap为36.31%。该结果没有复现旧600帧
片段的0%。

机制差异已有直接数据支持：AWS箱子只在约2.01 m范围内反复运动，总路程157.19 m，
约合78次单程经过；旧Gazebo范围约10.02 m、约17次。更关键的是，旧箱子体素在最后
命中后被非箱子射线重新穿过的比例从旧序列92.17%降到AWS 5.80%，OctoMap缺少free
反证。当前AWS最强结果主要来自正时间支持，S3只把11.60%进一步降到8.38%。

权威记录：

- `results/aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_FIXED_TRAJECTORY_MAPPING_RESULT.md`
- `results/DT-SLAM_当前成果与已解决问题_GPT同步_2026-08-07.md`
- `/data/dynaslam/large_results/aws_small_house_formal_2026-08-06/mapping_comparison_res10cm_step2_stride8/`

本结果再次冻结以下边界：旧Gazebo的0%不能跨轨迹外推；8帧是项目工作点；OctoMap
不是动态检测器；当前离线Mapping不等于在线长期地图。
