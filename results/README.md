# DT-SLAM `results/` 结果导航

更新日期：2026-08-07

## 1. 从这里开始

`results/` 目前约有 77 个阶段目录、13,500 个文件，总体积约 3.7 GB。其中大部分是历史日志、CSV、轨迹、mask 和点云，不需要逐个阅读。

要了解当前项目，按下列顺序即可：

1. [`DT-SLAM_成果收束与复现索引_2026-08-07.md`](DT-SLAM_成果收束与复现索引_2026-08-07.md)：当前权威入口，包含最终范围、结果矩阵和复现命令。
2. [`DT-SLAM_当前成果与已解决问题_GPT同步_2026-08-07.md`](DT-SLAM_当前成果与已解决问题_GPT同步_2026-08-07.md)：简明总结，适合人工阅读或交给 ChatGPT 审阅。
3. [`DT-SLAM_当前语义与区域几何正式算法技术报告_2026-08-06.md`](DT-SLAM_当前语义与区域几何正式算法技术报告_2026-08-06.md)：学习当前 YOLOv8 语义与 SIn 风格区域几何原理。
4. [`DT-SLAM_全过程总结_尝试试错与未解决问题_2026-08-05.md`](DT-SLAM_全过程总结_尝试试错与未解决问题_2026-08-05.md)：理解早期试错、负结果和路线收敛过程。
5. [`DT-SLAM_成果收束验证记录_2026-08-07.md`](DT-SLAM_成果收束验证记录_2026-08-07.md)：构建、单元检查、smoke 和哈希验证。

## 2. 状态标记

| 标记 | 含义 |
| --- | --- |
| **正式** | 已进入当前系统或最终结论 |
| **正结果** | 在指定序列和协议下得到可重复收益 |
| **负结果** | 实验已完成，但未通过当时的判据；不是当前主方法 |
| **Shadow** | 只计算和记录，不改变 Tracking、MapPoint 或建图输入 |
| **对照** | 用于文献复现、组件消融或性能上限，不代表最终路线 |
| **原始证据** | CSV、JSON、log、轨迹、mask、点云等机器可读输出 |

## 3. 当前正式结果

### 3.1 最终系统范围

| 内容 | 状态 | 文档 |
| --- | --- | --- |
| YOLOv8-seg 已知人物语义 | **正式** | [`ate_semantic_baseline_2026-07-29/REPORT.md`](ate_semantic_baseline_2026-07-29/REPORT.md) |
| SIn 风格区域几何 S1 | **正式** | [`sindslam_s1_shadow_2026-08-03/`](sindslam_s1_shadow_2026-08-03/) |
| S2 几何特征 Tracking 过滤 | **正式** | [`sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md`](sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md) |
| S3 动态深度输出 | **正式** | [`sindslam_s3_2026-08-04/S3_DYNAMIC_DEPTH_FILTER_RESULT.md`](sindslam_s3_2026-08-04/S3_DYNAMIC_DEPTH_FILTER_RESULT.md) |
| TUM/Bonn 代表序列系统对照 | **正式** | [`sindslam_systematic_eval_2026-08-04/SYSTEMATIC_EVALUATION_FIRST_PASS_RESULT.md`](sindslam_systematic_eval_2026-08-04/SYSTEMATIC_EVALUATION_FIRST_PASS_RESULT.md) |
| Bonn 未知箱子结果 | **正结果＋能力边界** | [`sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md`](sindslam_s2_2026-08-04/S2_REGION_FEATURE_FILTER_RESULT.md) |
| Gazebo/AWS 跨域与建图 | **正负并存** | [`aws_small_house_formal_2026-08-06/`](aws_small_house_formal_2026-08-06/) |

### 3.2 最终因果审计

| 阶段 | 目的 | 目录 |
| --- | --- | --- |
| R0 | 冻结可重复基线 | [`r0_freeze_2026-08-05/`](r0_freeze_2026-08-05/) |
| R1 | 定位 Gazebo 失败层 | [`r1_gazebo_failure_layer_2026-08-05/`](r1_gazebo_failure_layer_2026-08-05/) |
| R2 | Homography 与 RGB-D/SE(3) 自运动补偿对照 | [`r2_ego_compensation_2026-08-05/`](r2_ego_compensation_2026-08-05/) |
| R4 | 长时间间隔 Mapping refinement 审计 | [`r4_s4_mapping_refinement_2026-08-05/`](r4_s4_mapping_refinement_2026-08-05/) |
| R6 | 最终证据边界和停止决策 | [`r6_final_evaluation_2026-08-05/R6_FINAL_EVIDENCE_SUMMARY.md`](r6_final_evaluation_2026-08-05/R6_FINAL_EVIDENCE_SUMMARY.md) |

### 3.3 建图结果

| 内容 | 定位 | 文档 |
| --- | --- | --- |
| 旧 Gazebo 固定轨迹时间支持/OctoMap | 离线 Mapping 可行性 | [`offline_mapping_feasibility_2026-08-06/OFFLINE_FIXED_TRAJECTORY_MAPPING_FEASIBILITY_RESULT.md`](offline_mapping_feasibility_2026-08-06/OFFLINE_FIXED_TRAJECTORY_MAPPING_FEASIBILITY_RESULT.md) |
| AWS Small House 四模式 Tracking | 更真实仿真场景 | [`aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_PERSON_BOX_FOUR_MODE_RESULT.md`](aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_PERSON_BOX_FOUR_MODE_RESULT.md) |
| AWS 固定轨迹 Mapping | S3、时间支持、OctoMap 对照 | [`aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_FIXED_TRAJECTORY_MAPPING_RESULT.md`](aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_FIXED_TRAJECTORY_MAPPING_RESULT.md) |
| 作者原版 SInDSLAM 在 Gazebo 的对照 | 外部方法复现 | [`sindslam_original_gazebo_2026-08-06/AUTHOR_SIN_GAZEBO_TRACKING_MASK_AND_POINTCLOUD_RESULT.md`](sindslam_original_gazebo_2026-08-06/AUTHOR_SIN_GAZEBO_TRACKING_MASK_AND_POINTCLOUD_RESULT.md) |

## 4. 历史实验按路线分类

### 4.1 G0：几何测量基础

`g0_*` 目录包含单参考 depth warp、正负深度残差、坐标域、GT/SLAM 位姿敏感性、静态负样本、深度连通区和 feature shadow。

- `g0_1_*`：depth warp 和 z-buffer。
- `g0_2*`：evidence、坐标域、位姿敏感性和可视化。
- `g0_3*`：深度 flood fill；**负结果，不得用于正式过滤**。
- `g0_4f_*`：ORB feature-level shadow evidence。

### 4.2 GJ：Ji 2021 三维聚类对照

`gj_*`、`gj2*`、`gj3*` 是 3D K-means＋簇内重投影误差的文献对照。它们用于说明区域上下文与性能代价，不是当前主检测器。

### 4.3 G1：稀疏特征过滤和 MapPoint 接入

- `g1_f0_*`：Tracking 插入位置的反事实实验。
- `g1_f1_*`：稀疏 LK ego-flow 高残差特征过滤。
- `g1_m0_*` / `g1_m1_*`：MapPoint 写入对照与正式门控。
- `g1_map_quality_*`：稀疏地图质量审计。
- `g1_release_*`：当时的四模式稀疏前端冻结。
- `g1_bonn_box_*`：Bonn 箱子序列的特征证据和轨迹实验。

这组实验证明过滤执行链路真实有效，但早期单点 LK 规则未形成可靠未知对象检测，现为默认关闭的稀疏对照。

### 4.4 G2：类别无关几何探索

`g2_*` 是最大的历史试验组，主要为 **shadow、可行性或停止决策**：

- `g2_1_*`：多参考深度证据。
- `g2_2g/r/s_*`：参考选择、网格和 ORB-depth 采样。
- `g2_3r0–r4_*`：区域表示、聚合、覆盖和低分辨率近似。
- `g2_4*`：Bonn 坐标域、候选帧、动静可分性、LK、刚性和区域上下文。
- `g2_5a_*`：语义参考稀疏 flow。
- `g2_6*`：Bonn 额外箱子数据和静态模型对齐。
- `g2_motion_grouping*`、`g2_mh1_*`：运动分组输入和稀疏 3D 刚体假设。

阅读 G2 时应优先看各目录的 `*_RESULT.md` 和 `*_ROUTE_DECISION.md`，不应从单个 CSV 反推最终方法。

### 4.5 SInDSLAM 引入主线

- [`sindslam_reproduction_2026-08-02/`](sindslam_reproduction_2026-08-02/)：独立 CPU 复现。
- [`sindslam_mask_audit_2026-08-02/`](sindslam_mask_audit_2026-08-02/)：论文、源码和 mask 语义审计。
- [`sindslam_s0_cuda_2026-08-03/`](sindslam_s0_cuda_2026-08-03/)：CUDA BroxFlow 可用性和速度。
- [`sindslam_s1_shadow_2026-08-03/`](sindslam_s1_shadow_2026-08-03/)：clean-room 区域形成、dense flow residual、RAG 和时序判决。
- [`sindslam_s2_2026-08-04/`](sindslam_s2_2026-08-04/)：区域 mask 进入 Tracking 和 MapPoint 写入控制。
- [`sindslam_s3_2026-08-04/`](sindslam_s3_2026-08-04/)：动态深度输出。
- [`sindslam_systematic_eval_2026-08-04/`](sindslam_systematic_eval_2026-08-04/)：TUM/Bonn 代表序列评价。
- [`sindslam_gazebo_moving_box_2026-08-04/`](sindslam_gazebo_moving_box_2026-08-04/)：Gazebo 箱子跨域失败分析。
- [`sindslam_original_gazebo_2026-08-06/`](sindslam_original_gazebo_2026-08-06/)：作者原版对照。

## 5. 按数据集查找

| 数据 | 主要目录 | 用途 |
| --- | --- | --- |
| TUM walking/sitting/fr1_xyz | `ate_semantic_baseline_*`、`g0_*`、`g1_release_*`、`sindslam_systematic_eval_*` | 人物动态、低动态和真静态对照 |
| Bonn 动态 RGB-D | `bonn_expansion_*`、`g1_bonn_box_*`、`g2_6*`、`sindslam_systematic_eval_*` | 未知箱子、遮挡和 fail-open |
| 旧 Gazebo 回字走廊 | `gazebo_person_box_*`、`sindslam_gazebo_moving_box_*`、`offline_mapping_feasibility_*` | 假回环、远小箱子和地图 ghost |
| AWS Small House | `aws_small_house_formal_*` | 特征更丰富的人物＋箱子仿真及 Mapping 对照 |

## 6. 文件名的使用规则

| 后缀/名称 | 含义 |
| --- | --- |
| `*_SPEC.md` | 实验前冻结的设计、参数和停止条件 |
| `*_RESULT.md` / `REPORT.md` | 实验结果与结论，优先阅读 |
| `*_AUDIT.md` | 文献、坐标、数据或实现审计 |
| `*_DECISION.md` | 路线继续/停止的决策 |
| `.csv` / `.json` | 机器可读指标；必须结合对应 RESULT 解释 |
| `.log` / `.txt` | 运行日志和轨迹；通常不是阅读入口 |
| `.png` / `.ply` / `.pcd` | mask、联系表和点云；属于视觉证据 |

## 7. 暂不移动历史目录的原因

很多报告、命令、哈希清单和本地证据直接引用现有路径。大规模移动会造成：

- 复现命令失效；
- Markdown 引用断开；
- 冻结哈希与本地证据路径不一致；
- 历史实验与当前结果的时间关系被破坏。

因此本次采用“保留物理路径＋增加分类导航”的非破坏式整理。顶层遗留的若干 `CameraTrajectory.txt` 和早期日志只是历史输出，不是当前权威结果。

## 8. 今后新增结果的约定

1. 每个新实验使用独立的 `topic_YYYY-MM-DD/` 目录。
2. 正式实验至少包含一份 `SPEC.md` 或协议说明，以及一份 `RESULT.md`。
3. 新结果必须标注为正式、shadow、对照或负结果。
4. 大型轨迹、mask、点云和日志留在本地证据目录；Git 只保存源码、小型摘要和 Markdown 结论。
5. 若新结果改变最终主张，同时更新本文档和成果收束索引。
