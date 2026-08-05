# DT-SLAM 当前实验版本技术说明：YOLOv8-seg 与 SInDSLAM 风格几何

日期：2026-08-05
用途：提交给 ChatGPT/研究合作者，讨论下一步算法方向与论文叙事
代码检查点：`main@16ea79d`（算法代码已推送）
权威工作区：`/home/zhu/dynaslam_ws`

> 这里的“当前版本”指已经完成 S1–S3 接入的实验检查点，不表示论文方法已经最终冻结。几何功能默认关闭，须通过明确配置或四模式 runner 开启。

---

## 1. 方法摘要

当前系统是一个 ORB-SLAM2 RGB-D 前端动态观测过滤系统：

```text
RGB + registered depth
        │
        ├── YOLOv8n-seg / CUDA
        │      └── 已知动态类别 person 的像素 mask
        │
        ├── SInDSLAM 风格 clean-room 区域几何 / CPU
        │      ├── 三维深度初始聚类
        │      ├── 深度边缘切分与区域邻接合并
        │      ├── 稠密光流与全局 homography 运动补偿
        │      ├── 区域内低/高残差判决
        │      └── 类别无关几何动态 mask
        │
        ├── S2：将 mask 映射到 ORB 特征
        │      ├── 匹配阶段跳过动态特征
        │      ├── 现有 PoseOptimization 前清除残留关联
        │      └── 禁止动态特征创建新 MapPoint
        │
        └── S3：建图深度输出
               ├── dynamic depth mask
               └── dynamic pixels = 0 的 static depth clone
```

系统有两个不同输出：

```text
D_feat  ：动态 ORB 特征集合，用于 Tracking 和稀疏 MapPoint
M_depth ：动态深度像素区域，用于深度/点云建图
```

当前几何不是简单 LK 光流法。LK（Lucas–Kanade）只属于早期保留的轻量稀疏实验支线；当前主实验版本使用区域化的稠密 DeepFlow。

---

## 2. 术语与阶段代号

| 代号 | 含义 |
| --- | --- |
| Phase 0 | 语义动态特征处理清理与同步 baseline |
| G0 | 单/多参考 RGB-D 深度投影证据研究 |
| GJ | Ji 2021 三维 K-means＋簇重投影文献基线 |
| G1-LK | 轻量稀疏 ego-flow 高残差特征过滤，现为实验基线 |
| S0 | 独立 SInDSLAM 论文、源码、CPU/GPU 复现审计 |
| S1 | SIn 风格区域和稠密运动检测 |
| S2 | S1 mask 进入 ORB Tracking 与新 MapPoint 写入控制 |
| S3 | S1/语义 mask 进入建图深度输出 |
| S4 | 计划中的长时间隔深度精修，尚未实现 |
| shadow-only | 只计算/记录/显示，不改变 SLAM 状态 |
| fail-open | 几何过滤会使原生 Tracking 条件必然失败时，撤销本帧几何 tracking 标志以保住定位 |
| RAG | Region Adjacency Graph，区域邻接图 |
| ATE | Absolute Trajectory Error，绝对轨迹误差 |
| RPE | Relative Pose Error，相对位姿误差 |

文献归属标签：

- `[L]`：论文/公开方法直接原型；
- `[A]`：基于原型的明确改造；
- `[S]`：本项目系统接口或工程设计；
- `[H]`：尚需实验验证的假设。

---

## 3. RGB-D 输入与坐标域

### 3.1 主调用链

```text
Examples/RGB-D/rgbd_tum.cc
→ System::TrackRGBD(rgb, depth, semantic_mask, timestamp, optional_gt_pose)
→ Tracking::GrabImageRGBD(...)
→ Frame(gray, depth_meters, ...)
→ Tracking::Track()
```

输入要求：

- RGB 与 depth 非空且同尺寸；
- depth 已注册到 RGB；
- `Tracking` 将 depth 转为 `CV_32FC1` 米制深度；
- semantic mask 若存在，必须为 `CV_8UC1` 且与 RGB 同尺寸；
- 项目内部动态 mask 统一为 `0=保留/静态`、`255=动态/拒绝`。

### 3.2 Bonn 去畸变

Bonn depth 虽已注册到 RGB，但原相机有非零畸变。当前使用 `RGBDInputRectifier` 将：

- RGB：双线性插值；
- depth：最近邻插值，避免生成不存在的深度；
- YOLO 输入、ORB、semantic mask、geometry mask：统一到 `P=K` 的无畸变针孔域。

这避免“ORB 在去畸变域、depth warp 在原始域、mask 又在第三个域”的不可解释错误。

---

## 4. YOLOv8-seg 语义分支

### 4.1 源码位置

- `DT-SLAM/include/YOLOSegment.h`
- `DT-SLAM/src/YOLOSegment.cc`
- `DT-SLAM/Examples/RGB-D/rgbd_tum.cc`

### 4.2 推理后端

- ONNX Runtime C++ API；
- 强制 `CUDAExecutionProvider`，device 0；
- CPU fallback 被显式禁用，CUDA 不可用时初始化失败；
- graph optimization：`ORT_ENABLE_ALL`；
- intra-op threads：2；
- 当前模型通常为 `weights/yolov8n-seg.onnx`。

### 4.3 线程模型

`YOLOSegment` 内部有一个工作线程：

```text
PushFrame(rgb, seq)
→ deep copy 到 pending buffer
→ worker preprocess / ONNX / postprocess
→ 写 latest mask + detections + seq
→ WaitForMask(seq) 唤醒
```

但是示例程序对每帧执行：

```text
若当前结果不是 frame i：PushFrame(i)
→ WaitForMask(i)
→ 得到同帧 mask
→ TrackRGBD(frame i, mask i)
```

所以：

- 线程实现层面是独立 worker；
- 系统消费层面是精确同步；
- mask age 为 0；
- 当前不允许使用旧 mask 追踪，也没有真正把 YOLO 与当前帧 Tracking 流水并行。

这是 Phase 0 为保证 baseline 正确作出的设计。

### 4.4 预处理

1. 从 ONNX 输入 shape 读取网络宽高；
2. 保持宽高比 letterbox；
3. padding 值为 `(114,114,114)`；
4. `blobFromImage` 归一化到 `[0,1]`；
5. `swapRB=true`。

### 4.5 检测和 mask 后处理

当前只读取 class 0，即 `person`：

- confidence threshold：0.5；
- Non-Maximum Suppression（非极大值抑制）阈值：0.45；
- 由 32 维 mask coefficient 与 prototype 相乘；
- sigmoid；
- prototype 概率裁到 `[0,1]`；
- bbox 在 prototype 域增加约 10% margin；
- 二值阈值：0.5；
- resize 回原图 bbox；
- 所有实例按位 OR；
- 最终执行 7×7 椭圆核膨胀。

输出：

- mask：`CV_8UC1`，原/校正输入尺寸，`0=static`、`255=dynamic`；
- detections：`std::vector<Detection>`，每项含 `cv::Rect box` 和置信度，用于 Viewer。

### 4.6 semantic mask 如何进入 ORB-SLAM2

`Tracking::GrabImageRGBD()` 对非零输入 mask 做 `cv::compare(mask,0,CMP_NE)`，然后：

```text
UpdateDynamicFeaturesFromMask(frame, semanticMask)
```

对每个 ORB keypoint 检查像素位置：

- `mvbSemanticDynamic[i] = 1`；
- `mvbDynamic[i] = 1`。

ORB 特征是在 mask 应用前提取的；当前不是“mask 内不提取 ORB”，而是“提取后标志并在匹配/地图写入时跳过”。这样保留完整特征集合用于调试和安全回退。

---

## 5. SInDSLAM 风格几何分支：方法来源与边界

### 5.1 为什么采用这条路线

早期 depth residual、简单 flood fill、固定 K-means 和稀疏 LK 证明了低层运动证据存在，但没有建立可靠对象区域。SInDSLAM 提供了一条完整的区域逻辑：

```text
几何重聚类
→ 区域内稠密光流残差
→ 时间先验
→ 动态 mask
```

它能在物体 ORB 特征很少时仍利用稠密像素，并可同时输出 `D_feat` 和 `M_depth`。

### 5.2 不是源码复制，也不是作者等价复现

当前实现是根据论文、PaperNotes、公开源码行为审计和独立 parity 数据进行的 clean-room 重写。没有直接复制作者 `DynaDetect.cc`。

与作者系统的关键差异：

| 项目 | 作者 SInDSLAM | 当前 DT-SLAM |
| --- | --- | --- |
| 初始区域 | 3D K-means | 独立实现 3D K-means |
| 几何边界 | 深度边缘＋PEAC/AHC 平面边缘 | 深度梯度为主；自写 plane substitute 默认关闭 |
| 区域合并 | 作者重聚类/RAG 逻辑 | clean-room gradient-only RAG 近似 |
| 稠密流 | CUDA BroxFlow 为论文主要配置，也有 CPU 路径 | 当前正式 DT 配置为 CPU DeepFlow |
| 相机运动补偿 | 全局 homography | 全局 homography，clean-room 实现 |
| 长时间隔 depth refinement | 论文有 | 未实现（S4） |
| 稠密地图 | ROS/OctoMap 路径 | 无在线稠密 mapper；S3＋离线同位姿点云 |
| ORB 接入 | 作者完整系统 | 复用 DT-SLAM 的 `mvbDynamic` 与安全回退 |

因此正确名称是：

> **SInDSLAM 风格区域几何检测 / SIn-style region geometry**

而不是“SInDSLAM 完整复现”。

---

## 6. S1 详细算法：从深度和 RGB 到区域动态 mask

### 6.1 主要类

- `SInStyleInitialRegionClusterer`
- `SInStyleGradientRegionSplitter`
- `SInStylePlaneEdgeRegionSplitter`（当前默认关闭）
- `SInStyleRAGRegionMerger`
- `SInStyleDenseFlowResidualEstimator`
- `SInStyleRegionDynamicClassifier`
- `SInStyleDynamicDetector`（reference replay/审计）

主调度函数：

```text
Tracking::RunSInStyleRegionShadow(depthMeters, denseFlowGray)
```

函数名保留了 `Shadow` 历史，但当 `RegionFeatureFilterEnable=1` 时输出会真实进入 S2；不能仅按函数名判断当前是否改变 SLAM。

### 6.2 步骤 1：三维初始区域

对有效深度像素反投影：

```math
X(u,v)=D(u,v)K^{-1}[u,v,1]^T.
```

在三维空间中执行 K-means，得到初始 label map。实现支持：

- coarse-to-fine；
- 多层金字塔；
- 固定随机种子；
- 上一帧初始化；
- 最大深度限制；
- 运行时和区域统计。

`[L/A]` 来源于 SInDSLAM/Ji 等深度区域思想，但参数和实现属于本项目 clean-room 版本。

### 6.3 步骤 2：深度梯度边界切分

从当前米制深度计算相对/绝对深度跳变，形成真实边界候选，并在初始 K-means 区域内重新形成连通 core labels。

目的：

- 切开被 K-means 欠分割后混在一起的近/远表面；
- 阻止运动证据直接跨明显深度边缘扩散。

当前 plane-edge substitute 因过分割和成本问题默认关闭；作者 PEAC/AHC 平面重聚类没有被等价保留。这是当前区域表示的一项已知差距。

### 6.4 步骤 3：区域邻接图合并

对切分区域建立 RAG（区域邻接图），使用：

- 相邻边界接触；
- 区域大小；
- 深度分布/直方图；
- 深度拒绝条件；
- 大、中、小区域权重；

合并过度切碎的区域，输出 `mergedLabels`。

目标不是把全图变成很少几个区域，而是在“欠分割”和“过分割”之间取得可用于运动判决的空间单元。Gazebo 日志中部分帧约 12 个初始区域被合并为 6–7 个大区，说明当前 RAG 在跨域场景仍可能过合并。

### 6.5 步骤 4：稠密光流

当前正式实现使用 CPU DeepFlow，在约 0.6 图像尺度计算，再做 Variational Refinement。得到参考帧到当前帧的 observed flow：

```math
f_{obs}(u)=u_t-u_r.
```

这与早期稀疏 LK 不同：

- LK 只在 ORB 特征附近给稀疏对应；
- DeepFlow 给区域内稠密像素运动；
- 因此低纹理箱子仍可能有区域支持，但计算量显著增加。

### 6.6 步骤 5：全局 homography 相机运动补偿

从筛选后的 flow 对应中使用加权样本估计全局 homography。正常使用较长参考间隔，检测到大运动时使用相邻帧。由 homography 得到静态相机诱导二维运动：

```math
f_{ego}^{H}(u)=\pi(H\tilde u)-u.
```

残余流：

```math
r_f(u)=f_{obs}(u)-f_{ego}^{H}(u).
```

`[L/A]` 这一结构来自 SInDSLAM 的 homography 补偿与 FlowFusion 的 observed-minus-ego-flow 思想。

当前限制：单一 homography 只能完整描述一个投影平面或纯旋转等特殊情形；具有明显深度层次和视差的场景中，静态背景也可能出现残余。Gazebo 走廊结果与这一机制相容，但尚未通过控制变量证明它是唯一原因。

### 6.7 步骤 6：残差归一化与双阈值证据

由残余流幅值形成低/高阈值 mask。阈值由 Otsu/Triangle 等图像统计得到，并限制在预设像素范围内；当前常见限制约为：

- low：约 1.7–3 px；
- high：约 3–10 px。

输出不是单一“动态/静态”残差，而是不同强度证据和不可用状态。

### 6.8 步骤 7：区域级判决

`SInStyleRegionDynamicClassifier` 在每个区域内统计：

- low residual support；
- high residual support；
- 轮廓与面积；
- 上一帧 high-dynamic prior；
- whole-region 或 partial-region 填充条件。

当前关键默认值包括：

- minimum high pixels：100；
- minimum contour area：100；
- roundness：0.2；
- large contour：2000；
- whole-region fill ratio：0.5；
- low-mask dilation：5；
- final output dilation：9。

内部可保留三态：

```text
0   = unknown / no dynamic state
125 = static state
255 = dynamic state
```

对外动态 mask 为 `CV_8UC1`，`0=not rejected`、`255=dynamic`。

这些固定像素数量和面积是 Gazebo 远小对象失败的重要候选原因之一：小箱子投影面积不足时，残差可能无法达到区域支持条件。但当前不能只调低阈值就宣称修复，因为这也可能增加背景误检。

### 6.9 步骤 8：短时时序先验

本帧区域动态状态提交给下一帧 dense-flow estimator/classifier，用于帮助维持区域状态。这是相邻帧短时先验，不等同于对象 ID 跟踪，也不是长时间隔慢速物体确认。

---

## 7. S2：区域 mask 如何真实改变 ORB-SLAM2

### 7.1 特征映射

对当前帧每个 ORB keypoint：

```text
若 keypoint 坐标落在 S1 dynamic mask 非零处
→ geometry candidate
→ 在安全条件通过时写 mvbDynamic[i] = 1
```

语义和几何状态区分保存：

- `mvbSemanticDynamic`：语义来源；
- `mvbDynamic`：当前合并后的最终 feature veto；
- SIn 几何新增标志另有逐帧数组，便于 fail-open 只撤销 geometry，不撤销 semantic。

### 7.2 影响 Tracking 的位置

1. **匹配前**：`ORBmatcher` 跳过 `mvbDynamic` 特征；
2. **局部地图搜索后**：`RemoveDynamicAssociations()` 在原有 `PoseOptimization()` 前清除可能残留的动态关联；
3. **不增加优化次数**：仍使用 ORB-SLAM2 原有 TrackWithMotionModel/TrackReferenceKeyFrame 与 TrackLocalMap 优化结构；
4. **不是回溯修正初始位姿**：S1 在当前实现中依赖图像 flow/homography，mask 在 `Track()` 前生成，因此能在当前匹配入口生效；但没有联合优化相机运动与动态分割。

诊断 CSV 中 `actual_removed_associations=0` 通常不表示几何没有工作。更常见原因是候选已在前置 `ORBmatcher` 被跳过，后置清理自然没有残留关联。

### 7.3 影响稀疏地图的位置

- RGB-D 初始化创建 MapPoint 时跳过动态特征；
- `CreateNewKeyFrame()` 的 RGB-D MapPoint 创建跳过动态特征；
- LocalMapping 的新 MapPoint 生成检查动态标志。

S2 只能阻止当前候选创建新的稀疏点，不能自动清除历史已写入的动态 MapPoint。

### 7.4 安全回退

第一层：若合并 dynamic 后剩余特征少于 250，整帧 geometry feature filter 不应用。

第二层：若 ORB-SLAM2 原生的最低匹配条件将失败，则在对应 tracking 阶段只撤销本帧新增几何标志：

- reference-keyframe pre-pose：原生 15 matches；
- motion-model wide-window pre-pose：原生 20 matches；
- local-map pre-pose：理论 MapPoint inliers 低于原生 30，重定位窗口为 50；
- 已经进入 LOST 时，在 Relocalization 前 fail-open。

Tracking 后恢复同一组 geometry mapping veto，继续阻止新 MapPoint 写入。

这是 `[S]` 安全设计，依据 ORB-SLAM2 已有成功条件；它不是 SInDSLAM detector 的一部分，也不代表被恢复的点是静态。

---

## 8. S3：语义/几何如何进入建图深度

### 8.1 接口

主要类：

- `DT-SLAM/include/SInStyleDepthFilter.h`
- `DT-SLAM/src/SInStyleDepthFilter.cc`

输入：

- `depthMeters`：`CV_32FC1`，米；
- `semanticDynamicMask`：`CV_8UC1`，可空；
- `geometryDynamicMask`：`CV_8UC1`，可不可用；
- `geometryEvidenceAvailable`：显式有效性标志。

输出：

- `dynamicDepthMask`：`CV_8UC1`；
- `staticDepthMeters`：`CV_32FC1` clone，动态像素设为 0；
- availability、拒绝像素数、比例和耗时统计。

### 8.2 三种融合方式

| 模式 | 动态深度 mask |
| --- | --- |
| `semantic_only` | `M_semantic` |
| `geometry_only` | `M_geometry`，geometry 不可用时不把 unknown 当 static evidence |
| `semantic_or_geometry` | `M_semantic OR M_geometry` |

硬 OR 只是首个完整系统 baseline，并非已证明最佳融合。

### 8.3 与 Tracking 隔离

执行顺序：

```text
构造原始 Frame 和米制 depth
→ 得到 semantic/S1 masks
→ 先计算 S3 深度副本
→ Track() 始终使用原始 Frame depth
→ 只有 Tracking OK 且 Tcw 有效时才将 S3 输出标记为 mapping-admissible
```

通过 `System` 的 clone 接口读取：

- `GetCurrentDynamicDepthMaskForMapping()`；
- `GetCurrentStaticDepthForMapping()`。

因此 S3 本身不可能改善或恶化当前帧 ATE；两次运行的小 ATE 差异属于 ORB-SLAM2 运行波动。

### 8.4 当前建图形式

DT-SLAM 当前没有在线稠密地图线程。S3 的效果通过离线工具使用同一组位姿、同一采样帧和同一深度范围生成：

- 未过滤点云；
- S3 过滤点云。

这样把“轨迹变化”和“深度 mask 变化”分离。

---

## 9. 四种运行模式

| 对用户显示的模式 | 语义 YOLO | SIn 区域几何 | S2 特征过滤 | S3 深度 mask |
| --- | ---: | ---: | ---: | --- |
| 纯 ORB-SLAM2 | 关 | 关 | 关 | 关 |
| 仅语义 | 开 | 关 | 语义标志生效 | semantic-only |
| 仅几何 | 关 | 开 | 几何标志生效 | geometry-only |
| 语义＋几何 | 开 | 开 | 两类标志合并 | semantic OR geometry |

轻量 LK 几何不属于上述当前 SIn 主模式；它保留为单独的 legacy experimental baseline，默认关闭。

---

## 10. 当前调用时序

```text
rgbd_tum：读取 RGB/depth
→ 可选 Bonn RGB-D 联合 rectification
→ 可选 YOLO worker；逐帧 WaitForMask(i)
→ System::TrackRGBD
→ Tracking::GrabImageRGBD
   → 为 DeepFlow 生成灰度图
   → depth 转 CV_32F 米制
   → 构造 Frame，提取 ORB
   → semantic mask → mvbSemanticDynamic/mvbDynamic
   → RunSInStyleRegionShadow
      → initial cluster
      → gradient split
      → RAG merge
      → dense flow/homography residual
      → region classifier
      → geometry mask
      → 可选 S2：geometry ORB flags
   → RunSInStyleDepthFilter
   → Tracking::Track
      → dynamic-aware matching
      → 原有初始 tracking 路径
      → SearchLocalPoints
      → dynamic association fallback cleanup
      → 原有 PoseOptimization
      → 可选 KeyFrame/MapPoint creation veto
   → FinalizeSInStyleDepthFilterForMapping
→ System clone S3 outputs
```

---

## 11. 当前主要结果

### 11.1 六个代表序列第一轮

| 序列 | 纯 ORB ATE | 仅语义 ATE | 仅几何 ATE | 联合 ATE | 几何 FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| TUM walking | 0.870140 | 0.016267 | 0.014655 | 0.014508 | 3.98 |
| TUM sitting | 0.007749 | 0.006560 | 0.008079 | 0.006439 | 4.40 |
| TUM fr1/xyz | 0.009830 | 0.009675 | 0.009647 | 0.009552 | 3.24 |
| Bonn nonobstructing box | 0.607423 | 0.352860 | 0.020697 | 0.021611 | 3.84 |
| Bonn obstructing box | 0.345349 | 0.318416 | 0.290795 | 0.298631 | 3.35 |
| Bonn static close/far | 0.084066 | 0.084186 | 0.082045 | 0.084647 | 3.75 |

单位为米；这是单轮系统性对照。纯 ORB/语义约 28–30 FPS。

### 11.2 结果含义

- TUM walking：语义已足够强，几何与联合略低，但差异很小；
- Bonn 非遮挡未知箱子：几何提供当前最强正面证据；
- Bonn 强遮挡：几何改善有限，必须依赖安全回退保持完整轨迹；
- 静态序列：没有明显 ATE 退化，但仍存在 2.7%–3.3% 级别的几何深度删除，不能据此称 mask 无误检；
- 联合模式没有稳定胜过 geometry-only，因此融合尚未冻结。

### 11.3 Gazebo 失败边界

600 帧移动箱子片段：

- 纯 ORB ATE 0.028619 m；
- geometry-only ATE 0.037247 m；
- 几何没有改善定位；
- S3 删除约 22.85% 点，但箱子移动轨迹仍明显；
- 箱子较小/远时中位覆盖为 0，接近相机后覆盖才接近完整；
- 全部 mask 像素中只有约 3.60% 属于箱子。

因此不能将当前方法写成跨域可靠未知对象分割器。

---

## 12. 运行时间与硬件

当前实测大致为：

| 模式/模块 | 速度或耗时 |
| --- | --- |
| 纯 ORB-SLAM2 | 约 28–30 FPS（Gazebo 输入发布上限约 20 FPS） |
| YOLOv8n-seg CUDA 同步语义 | 系统约 28–30 FPS |
| 当前 DT-SLAM S1 CPU DeepFlow 全链 | 系统约 3.2–5.3 FPS，视序列而变 |
| 独立作者链 GPU BroxFlow | 约 6.6–7.4 FPS |
| 独立作者链 CPU DeepFlow | 约 3.4–4.3 FPS |
| S3 深度 mask 应用 | 中位约 0.3–0.4 ms/帧 |

SIn 风格几何慢的主要原因不是 ORB-SLAM2，而是：

- 稠密光流迭代；
- K-means 与全图区域遍历；
- 边界切分、RAG 和区域统计；
- 形态学与时序处理。

稠密 detector 可以与稀疏 ORB-SLAM2 共存；“SLAM 是特征法”不限制动态 detection 必须是稀疏法。

---

## 13. 文献来源账本

| 当前部分 | 主要来源 | 当前性质 |
| --- | --- | --- |
| 语义＋几何总体结构 | DynaSLAM 等 | `[L/A]` 系统结构 |
| YOLOv8-seg person mask | YOLOv8-seg | `[A/S]` 轻量语义实现和工程接入 |
| 3D 初始区域 | SInDSLAM、Ji 2021 | `[L/A]` clean-room 实现 |
| 深度梯度切分 | SInDSLAM 几何重聚类思想 | `[A]` 当前只保留部分几何边界 |
| RAG/深度分布合并 | SInDSLAM | `[A]` 非作者等价实现 |
| observed flow − ego-flow | FlowFusion、SInDSLAM | `[L/A]` 稠密区域版本 |
| homography 运动补偿 | SInDSLAM | `[L/A]` clean-room 实现 |
| 双阈值/区域动态判决 | SInDSLAM | `[L/A]` 行为对照后重写 |
| 上一帧动态 prior | SInDSLAM | `[L/A]` 短时时序 |
| semantic/geometry 三模式 | 本项目 | `[S]` 消融和接口设计 |
| `mvbDynamic` 接入 | ORB-SLAM2 结构＋本项目 | `[S]` 系统集成 |
| 原生匹配阈值 fail-open | ORB-SLAM2 条件＋本项目 | `[S]` 安全退化 |
| S3 双输出 | 本项目，受动态 RGB-D mapping 启发 | `[S]` Tracking/Mapping 解耦 |
| 长时间隔 depth refinement | SInDSLAM | `[L]` 尚未实现的 S4 |
| RGB-D/SE(3) 替换 homography | DynaSLAM/FlowFusion/G0 综合 | `[H/A]` 候选下一步，尚未实现 |

---

## 14. 当前方法的优点

1. 语义和几何分别可关闭，支持四模式消融；
2. 几何不依赖 YOLO 类别，已在 Bonn 未知箱子上产生强定位收益；
3. 同一几何区域可服务 ORB 特征过滤和深度过滤；
4. 不修改 Optimizer/g2o，不增加第三次优化；
5. 强遮挡时有基于 ORB-SLAM2 原生条件的 fail-open；
6. 未测量/unknown 与 static 分开；
7. 输入坐标域、mask 极性和深度单位明确；
8. 有合成测试、parity、逐帧 CSV、轨迹和同位姿点云；
9. 正负结果均保留，没有通过临时阈值隐藏 Gazebo 失败。

---

## 15. 当前方法的主要缺陷

1. **速度慢**：CPU 稠密 flow 主导，远低于 30 FPS；
2. **跨域不稳定**：Bonn 强、Gazebo 弱；
3. **远小对象漏检**：固定像素支持和区域尺度可能不适合小目标；
4. **框外误检多**：Gazebo 中大量 mask 与箱子无关；
5. **homography 的三维视差限制**；
6. **gradient-only RAG 与作者完整重聚类有差距**；
7. **慢速/停止物体不可靠**，S4 未实现；
8. **语义 OR 几何未证明最优**；
9. **无在线稠密 mapper**，S3 目前是输出接口和离线地图验证；
10. **历史动态点不会被 S2 自动清理**；
11. **论文级重复统计不足**；
12. **代码研究分支较多，维护成本高**。

---

## 16. 建议让 ChatGPT重点判断的下一步

### 16.1 候选主实验：固定区域，只替换 ego-motion 模型

控制以下内容完全相同：

- observed dense flow；
- initial/merged region labels；
- residual thresholds；
- region classifier；
- temporal prior；
- S2/S3 接口。

唯一比较：

```text
全局 homography ego-flow
vs
RGB-D/SE(3) depth-aware ego-flow
```

评价：

- 静态背景 residual 分布；
- 箱子 residual/coverage；
- valid depth 覆盖率；
- 静态 ORB 删除；
- 动态 ORB 覆盖；
- ATE/RPE；
- S3 残影和静态完整度；
- 运行时间。

这是一个变量清楚、与现有文献和代码均衔接的候选论文改造。但不能预设 SE(3) 更好：它依赖有效 depth 和初始 pose，而 homography 不依赖每像素深度。

### 16.2 是否立即实现 S4

S4 长时间隔重投影有 SInDSLAM 文献依据，适合慢速、暂停后 residual 变弱的物体。它可能减少停留人物残影，但对 Gazebo 远小箱子早期根本没有区域证据的问题帮助有限。

建议先区分：

- 物体曾被检测，后来停止而丢失 → S4 可能适合；
- 物体从远处开始就从未被检测 → 应先处理尺度/区域/flow 证据。

### 16.3 语义与几何融合

当前硬 OR 覆盖面广，但会继承语义对静止人体的整块删除。可讨论：

- 保留三模式作为基础消融；
- 已知动态类别是否需要几何确认其“当前真的在动”；
- 对 Tracking 与 Mapping 使用不同强度的 mask；
- semantic 用作高召回候选，geometry 决定 tracking veto；
- 不能在没有实验前宣称某个概率融合最佳。

### 16.4 论文定位

可选叙事：

1. **系统论文**：YOLOv8 已知动态＋SIn 风格未知动态区域＋ORB-SLAM2 安全过滤＋双输出 mapping；
2. **方法论文**：区域固定情况下的 homography/SE(3) ego-flow 对比与新融合；
3. **工程/负结果论文**：轻量点级方法为什么不足，完整区域链在真实/仿真跨域中的收益与边界；
4. **性能论文**：保留区域链的前提下加速 dense flow，但不能再次删除核心层而退化为已失败的点级方案。

单纯“YOLO OR SIn mask”更像系统集成，算法创新有限。

---

## 17. 最小复现命令

### 17.1 构建

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
cmake ..
make rgbd_tum -j$(nproc)
```

### 17.2 ONNX Runtime GPU 环境

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

ORT_CAPI=/home/zhu/.local/lib/python3.10/site-packages/onnxruntime/capi
NVIDIA_BASE=/home/zhu/.local/lib/python3.10/site-packages/nvidia
NVIDIA_LIBS=$(find "$NVIDIA_BASE" -mindepth 2 -maxdepth 2 -type d -name lib -printf '%p:')

export LD_PRELOAD="$ORT_CAPI/libonnxruntime.so.1.23.2"
export LD_LIBRARY_PATH="${ORT_CAPI}:${NVIDIA_LIBS}/home/zhu/dynaslam_ws/pangolin_install/lib:/home/zhu/dynaslam_ws/DT-SLAM/lib:${LD_LIBRARY_PATH:-}"
```

### 17.3 纯 ORB-SLAM2

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt
```

### 17.4 仅语义

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/zhu/dynaslam_ws/results/g0_2c_2026-07-27/fr3_walking_xyz_associations_one_to_one_20ms.txt \
  weights/yolov8n-seg.onnx
```

### 17.5 四模式 runner 示例

以下以 Bonn 非遮挡箱子为例；`--mode` 可换为 `orb_baseline`、`semantic_only`、`geometry_only` 或 `semantic_geometry`：

```bash
python3 tools/run_sin_style_mode.py \
  --mode geometry_only \
  --working-directory /home/zhu/dynaslam_ws/DT-SLAM \
  --binary /home/zhu/dynaslam_ws/DT-SLAM/Examples/RGB-D/rgbd_tum \
  --vocabulary /home/zhu/dynaslam_ws/DT-SLAM/Vocabulary/ORBvoc.txt \
  --base-settings /home/zhu/dynaslam_ws/DT-SLAM/Examples/RGB-D/BONN_SInStyleRectifiedControl.yaml \
  --sin-settings /home/zhu/dynaslam_ws/DT-SLAM/Examples/RGB-D/BONN_SInStyleRectifiedNativeRegionDecisionCPU.yaml \
  --dataset /data/dynaslam/datasets/rgbd_bonn_moving_nonobstructing_box \
  --associations /home/zhu/dynaslam_ws/results/g1_bonn_box_2026-07-31/inputs/moving_nonobstructing_box_associations_20ms.txt \
  --model /home/zhu/dynaslam_ws/DT-SLAM/weights/yolov8n-seg.onnx \
  --output-directory /home/zhu/dynaslam_ws/results/manual_geometry_only \
  --viewer on
```

注意：runner 和部分 formal 配置目前是本地未跟踪文件；使用前应检查 `git status`，不能仅从 GitHub clone 假定它们存在。

### 17.6 ATE/RPE

```bash
evo_ape tum groundtruth.txt CameraTrajectory.txt -va --align --t_max_diff 0.02
evo_rpe tum groundtruth.txt CameraTrajectory.txt -va --align \
  --delta 1 --delta_unit f --t_max_diff 0.02
```

正式计时关闭 Viewer；Viewer 只用于观察特征、mask 和 Tracking，不应把其 FPS 与 headless 正式结果混合。

---

## 18. 当前主文件清单

### 18.1 语义

```text
DT-SLAM/include/YOLOSegment.h
DT-SLAM/src/YOLOSegment.cc
DT-SLAM/Examples/RGB-D/rgbd_tum.cc
```

### 18.2 RGB-D 输入域

```text
DT-SLAM/include/RGBDInputRectifier.h
DT-SLAM/src/RGBDInputRectifier.cc
```

### 18.3 SIn 风格 S1

```text
DT-SLAM/include/SInStyleDynamicDetector.h
DT-SLAM/src/SInStyleDynamicDetector.cc
DT-SLAM/include/SInStyleInitialRegionClusterer.h
DT-SLAM/src/SInStyleInitialRegionClusterer.cc
DT-SLAM/include/SInStyleGradientRegionSplitter.h
DT-SLAM/src/SInStyleGradientRegionSplitter.cc
DT-SLAM/include/SInStylePlaneEdgeRegionSplitter.h
DT-SLAM/src/SInStylePlaneEdgeRegionSplitter.cc
DT-SLAM/include/SInStyleRAGRegionMerger.h
DT-SLAM/src/SInStyleRAGRegionMerger.cc
DT-SLAM/include/SInStyleDenseFlowResidualEstimator.h
DT-SLAM/src/SInStyleDenseFlowResidualEstimator.cc
DT-SLAM/include/SInStyleRegionDynamicClassifier.h
DT-SLAM/src/SInStyleRegionDynamicClassifier.cc
```

### 18.4 S2/S3 与 ORB-SLAM2 接入

```text
DT-SLAM/include/SInStyleDepthFilter.h
DT-SLAM/src/SInStyleDepthFilter.cc
DT-SLAM/include/Tracking.h
DT-SLAM/src/Tracking.cc
DT-SLAM/include/Frame.h
DT-SLAM/src/Frame.cc
DT-SLAM/src/ORBmatcher.cc
DT-SLAM/src/LocalMapping.cc
DT-SLAM/include/System.h
DT-SLAM/src/System.cc
```

### 18.5 运行与地图工具

```text
DT-SLAM/tools/run_sin_style_mode.py
DT-SLAM/tools/summarize_sin_style_eval.py
DT-SLAM/tools/export_mapping_depth_pointcloud.py
```

---

## 19. 供外部审阅者的最终判断模板

审阅当前方案时，请分别回答以下问题，不要用一个 ATE 数值替代全部结论：

1. **检测层**：几何 mask 是否主要覆盖真实独立运动对象？
2. **可观测性层**：动态对象在远、小、低纹理、慢速、强遮挡情况下是否有充分证据？
3. **Tracking 层**：S2 是否改善 ATE/RPE、保持完整轨迹和足够静态匹配？
4. **Mapping 层**：S3 是否减少动态 ghost，同时保留静态表面？
5. **融合层**：semantic-only、geometry-only、OR 哪个在不同场景更合理？
6. **性能层**：完整 detector 的耗时来自哪里，哪些层不能再次轻量化掉？
7. **学术层**：下一项改造是否有明确文献依据、单一变量和可否证实验？

当前最客观的总结是：

> **YOLOv8-seg 语义基线已成熟；SIn 风格区域几何已成功接入 ORB-SLAM2 的特征过滤和深度输出，并在 TUM/Bonn 获得重要正结果，但在 Gazebo 远小箱子上暴露了对象召回和特异性问题。下一步应围绕一个明确机制做受控改造，而不是继续堆叠零散阈值或新模块。**
