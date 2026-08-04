# S1 native clean-room：初始三维区域增量规范

日期：2026-08-03  
阶段：S1 内部增量，不新增主阶段  
状态：第一增量已实现并完成 30 帧审计；后续 split/merge/flow 尚未实现

## 1. 目的

本增量只实现 SInDSLAM 区域链的第一层：

```text
CV_32F 米制深度
→ 有效三维点
→ 确定性全分辨率 3D K-means
→ native_initial_region_labels
```

它不计算光流、不判动态、不生成可供 Tracking 使用的动态 mask，也不修改
ORB、`mvbDynamic`、`mvpMapPoints`、MapPoint 或 Optimizer。

该拆分不是新增研究模块，而是为了单独验证区域表示、深度单位、相机内参、
时序初始化和运行成本。后续深度/平面边缘切分及 RAG 合并仍属于同一 S1。

## 2. 依据与归属

### `[L]` 论文明确给出的结构

SInDSLAM Section III-B1 给出：

- 深度像素通过相机内参反投影到三维；
- 使用 K-means 形成初始区域；
- 完整方法使用 coarse-to-fine 深度金字塔；
- 完整方法用粗层结果初始化细层，并可由上一帧区域初始化最高层；
- 区域数按 `width * height / 25600` 设置。

640×480 输入对应 12 个初始簇。

### `[C/A]` 公开行为参照与本项目适配

- 公开实现使用 4 层、3×4 初始网格、一次带初始标签的 OpenCV K-means；
- 公开实现把约 6 m 外或无效深度压到特殊的零三维位置，并对 z 维使用
  额外尺度；
- DT-SLAM 接口统一使用 `CV_32FC1` 米制深度，并显式保留 invalid/unknown；
- OpenCV K-means 是现有依赖，可复用项目已有的三维反投影与确定性测试
  经验，但不复制 SInDSLAM 或 PEAC/AHC 源码。

本增量应称为：

> `[A] SIn-style clean-room initial 3D region clustering`

不能称为 SInDSLAM geometric re-clustering 复现，因为尚未实现几何边缘、
平面边缘、区域切分、RAG 和深度直方图合并。

## 3. 输入与状态

输入：

```text
current_depth_meters : CV_32FC1
K                    : 当前输入针孔域的 3×3 内参
input_index
```

有效深度第一版定义：

```text
finite(depth) && depth > 0 && depth < max_depth_m
```

其中 `max_depth_m` 默认 6.0 m，与公开室内行为参照一致，但属于可配置实现
参数，不宣称是跨相机通用阈值。

第一增量尚无时序标签状态，`Reset()` 只维持统一接口语义。金字塔和上一帧
初始化会在同一 S1 的后续行为增量中加入，不另立研究阶段。

## 4. 输出语义

```text
nativeInitialRegionLabels : CV_32SC1
    -1 = 无效、越界或不参与初始聚类
    >0 = native 初始区域 ID

nativeRegionValidMask     : CV_8UC1，0/255
nativeInitialRegionStats  : 区域数、像素数、面积分布、运行时间
```

不存在 `nativeDynamicMask`。没有动态判决必须明确记录为：

```text
native_dynamic_decision = none
direct_slam_state_mutation = none
```

初始区域不等于对象，也不等于静态或动态区域。

## 5. 第一版算法边界

1. 将有效深度反投影为 `(x,y,z)`；
2. 固定 640×480 时 K=12，一般情况使用 `max(1, round(width*height/25600))`；
3. K-means 只使用有效三维点，invalid 始终保持 -1；
4. 固定随机种子和单 attempt，保证审计可复现；
5. 通过薄适配层临时复用当前项目已经测试的
   `JiGeometryBaseline::ComputeDepthClusters()`，只复用 XYZ/K-means 数值
   实现，不复用 Ji 的动态重投影判决；
6. 将 Ji 的 `-1/0..K-1` 标签转换成 S1 内部的 `-1/>0` 语义；
7. 输出 prepare、K-means、label conversion 和 total runtime。

第 3 点是 DT-SLAM 的保守 clean-room 适配：公开实现会把无效/远深度作为
零三维点参与内部 K-means，再在后续阶段排除。两者行为可能不同，必须在
结果中披露，不能包装成逐行等价。

## 6. 与 reference 的比较

S0 保存的 `labels.png` 是作者完成边缘切分和合并后的区域，而本增量只是
初始 K-means。因此不以逐像素标签一致率作为通过条件。

需要报告：

- native 初始区域数和有效覆盖；
- 最小/中位/最大区域面积；
- 区域边界长度与碎裂程度；
- 与作者 final labels 的边界覆盖和区域数量差异，只作描述性诊断；
- 初始化方式（grid 或 temporal）；
- prepare、K-means、label mapping 和 total runtime；
- 重复运行确定性。

## 7. 验收条件

- 静态单平面合成深度不会产生 invalid 泄漏；
- 深度阶跃两侧的三维区域统计可解释；
- 输入 invalid 始终输出 -1；
- label 数、面积和有效像素守恒；
- 相同输入和状态得到相同结果；
- reset 不改变无状态第一增量的确定性输出；
- 30 帧 TUM3 shadow 不崩溃，并记录全部输入；
- `actual_slam_removed=0`；
- 不修改 YOLO、Frame 动态字段、MapPoint 和 Optimizer。

## 8. 本增量不能回答

- 区域是否对应完整箱子或人物；
- 哪个区域正在运动；
- 是否改善 ATE/RPE；
- 是否可开放 S2；
- 是否达到作者完整 SInDSLAM 的 mask 或速度。

## 9. 后续同一 S1 内的忠实性增量

若全分辨率初始区域的单位、标签和统计通过，再在相同接口下加入论文明确
描述的 coarse-to-fine 与上一帧初始化。该后续增量用于降低成本和接近作者
行为，不改变本增量“初始区域不是动态检测”的结论。
