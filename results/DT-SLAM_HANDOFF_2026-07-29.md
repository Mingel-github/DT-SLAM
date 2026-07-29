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
670f008 Add lightweight geometry sampling and region shadows
```

检查时 `HEAD` 与 `origin/main` 均为 `670f008`。之后的 G2-3R1、G2-3R2、G2-3R3 和 ATE 报告仍在本地，尚未提交或推送。

已修改的跟踪文件：

```text
DT-SLAM/Examples/RGB-D/geometric_warp_test.cc
DT-SLAM/include/GeometricDynamicDetector.h
DT-SLAM/include/Tracking.h
DT-SLAM/src/GeometricDynamicDetector.cc
DT-SLAM/src/Tracking.cc
results/DT-SLAM_几何模块阶段进度_2026-07-28.md
```

主要未跟踪内容：

```text
DT-SLAM/Examples/RGB-D/TUM1_GeometryPyramidEvidenceShadow.yaml
DT-SLAM/Examples/RGB-D/TUM1_GeometryRegionEvidenceShadow.yaml
DT-SLAM/Examples/RGB-D/TUM3_GeometryPyramidEvidenceShadow.yaml
DT-SLAM/Examples/RGB-D/TUM3_GeometryRegionEvidenceShadow.yaml
DT-SLAM/tools/audit_region_dense_vs_grid.py
DT-SLAM/tools/audit_region_evidence.py
results/ate_semantic_baseline_2026-07-29/
results/g2_3r1_2026-07-28/
results/g2_3r2_2026-07-29/
results/g2_3r3_2026-07-29/
results/g2_3r4_2026-07-29/
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
| G2-3R4 | scale-2 低分辨率区域近似 | 文献审计完成、SPEC 待评审、实现未开始 |
| G1-F | 几何特征真正参与 tracking/filtering | 未放行 |
| G1-D | 几何区域真正过滤深度或稠密写图 | 未放行 |

必须保留的负实验结论：

```text
局部深度连续 ≠ 同一运动对象。
```

从所有正残差 seed 沿局部深度连续性无限 flood fill，会命中大型深度连通分量并吞掉大部分画面。不得通过再增加一个随意面积阈值或 seed-ratio 阈值掩盖该失败。

---

## 5. 最新阶段：G2-3R3

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

## 8. 下一步：G2-3R4，而不是直接进入过滤

当前建议的下一步是：

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

本地文献核对和设计报告已经完成，当前不能跳过 SPEC 评审直接写实现：

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

实现前评审仍需重点检查：

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

G2-3R4 即使通过也不直接解锁 G1-F。必须先完成可靠动态/静态区分门控、未知动态
物体验证和独立数据上的判决参数冻结。G1-D 还必须等待像素区域 mask 的独立验收。

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
   results/ate_semantic_baseline_2026-07-29/REPORT.md
   ```

4. 对照当前 `Tracking.cc`、`GeometricDynamicDetector.*` 和 YAML，确认代码确实处于 shadow-only；
5. 若状态与本文不同，先增加“交接差异记录”，不要擅自恢复或重置；
6. 向用户用简短文字复述：
   - 当前阶段；
   - 当前未提交状态；
   - 下一小步；
   - 明确不会做的越界修改；
7. 得到继续指令后，先评审 G2-3R4 SPEC；评审通过才开始最小 shadow 实现。

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
- 当前最小、合理且有证据支撑的下一步是评审 G2-3R4 SPEC，之后才进行低分辨率区域表示审计实现；
- 几何尚未进入真正的 SLAM 过滤，因此现在不测“几何 ATE 改善”，只保留语义基线 ATE/FPS 作为冻结对照。

这份交接的目的不是让新会话“相信摘要”，而是让它知道去哪里核查、哪些结论已经冻结、哪些结果仍是假设，以及下一步只能改动什么。
