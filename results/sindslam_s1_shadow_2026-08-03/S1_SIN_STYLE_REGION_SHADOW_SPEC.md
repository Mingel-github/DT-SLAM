# S1：SIn 风格区域检测 Shadow 接入规范

日期：2026-08-03  
权威工作区：`/home/zhu/dynaslam_ws`  
阶段：S1（不进入 S2 Tracking 过滤）

## 1. 目的与边界

S1 在 DT-SLAM 中建立独立的区域动态证据接口，并以独立 SInDSLAM
导出的同帧结果作为行为参照。S1 只读取和记录区域结果：

```text
direct_slam_state_mutation = none
dynamic_decision_consumer = shadow_only
```

这里的 `none` 只表示不写入 SLAM 状态。同步 PNG 读取和统计位于
`Track()` 前，可能改变墙钟时间以及 LocalMapping/LoopClosing 的线程调度，
因此不能宣称开启 replay 后轨迹执行时序必然完全不变。

严禁在本阶段：

- 写入 `Frame::mvbDynamic`；
- 清除 `Frame::mvpMapPoints`；
- 阻止 MapPoint 创建；
- 修改 ORB 提取 mask；
- 与 YOLO mask 做 OR/AND；
- 修改 `Optimizer.cc`、g2o、LocalMapping 或 LoopClosing；
- 增加 `PoseOptimization()` 调用。

## 2. 为什么先建立 reference/replay 后端

SInDSLAM 公开实现具有非确定性，CPU DeepFlow 与 GPU BroxFlow 的 mask
也不逐像素一致。如果一开始同时重写聚类、光流、阈值、时序状态和
DT-SLAM 接口，后续无法区分行为差异来自算法重写还是接入错误。

因此 S1 的同一个 `RegionDynamicDetector` 接口按以下顺序完成：

1. `reference_replay`：读取独立 SInDSLAM 对同一 association、同一输入序号
   导出的三态 mask 和标签。这不是检测算法，也不作为最终系统能力；
   它只冻结 DT-SLAM 侧的坐标域、三态语义、ORB 覆盖统计和 shadow
   不变式。
2. `native`：后续在同一接口内做 clean-room SIn-style 重新实现，并与
   `reference_replay` 成对比较。

这不是新增研究阶段，而是计划中“先行为参照、后方法改造”的 S1
内部实现顺序。

## 3. 最小接口

源码审计后冻结文件名为：

```text
include/SInStyleDynamicDetector.h
src/SInStyleDynamicDetector.cc
```

输入：

```text
当前灰度/RGB 图像
当前 CV_32FC1 米制深度
单调输入序号、SLAM Frame ID、reset epoch 和时间戳
后端配置与上一帧检测状态（native 后端使用）
```

输出：

```text
regionLabels              -1=标签不可用，0=invalid/unassigned bucket，>0=正标签
referenceKnownCodeMask    作者最终 mask 编码非零
referenceUnknownMask      作者最终 mask 编码为 0
authorDynamicMask         作者实际 Tracking 候选，raw state == 255
inputDepthValidMask       DT 当前输入深度 finite 且 >0
validMask                 作者编码非零且 DT 输入深度有效
dynamicMask               authorDynamicMask 与 validMask 的交集
unknownMask               项目证据无效，即 validMask 的补集
regionStatistics  标签面积、动态像素和未知像素
runtimeStatistics
```

三态约定固定为：

```text
static  != unknown
dynamic != unknown
unknown 不得解释为 static
```

独立 SIn runner 导出的 final/dilated `0/125/255` mask 在模块边界转换一次：

```text
0   -> 作者输出的 invalid/unassigned code
125 -> 作者输出的 static code
255 -> 作者 ORB 路径实际使用的 dynamic code
```

`125` 不是独立静态真值，`0` 也不等同于“背景”。作者 ORB extractor
直接使用 final mask 的 `255`，不再次检查深度；DT-SLAM 项目证据另将其与
当前有效深度求交。二者必须分别保存，不能用后者替代作者 Tracking 行为。

当前 `inputDepthValidMask` 只表示 DT 输入深度 `finite && >0`；它不是
SIn 内部 `imgTotalArea` 的复现。作者内部还包含深度中值、约 6 m 范围和
图像边界等约束，且 runner 最终膨胀又会把 255 扩展到该区域之外。

作者公开实现实际接收原始 `CV_16U` 深度并依赖 `DepthMapFactor`。本项目
clean-room 接口统一使用米制 `CV_32F`；native 后端实现任何深度阈值时
必须显式换算成米制，不能照搬作者代码中的原始整数阈值。

## 4. 配置与失败语义

第一步配置键：

```yaml
SInStyle.ShadowEnable: 0
SInStyle.Backend: "reference_replay"
SInStyle.ReferenceBackend: "deepflow_cpu" # 或 brox_cuda，禁止混合
SInStyle.ReferenceDirectory: ""
SInStyle.ReferenceMaskSuffix: "_mask_final.png"
SInStyle.RequireLabels: 1
SInStyle.CsvPath: ""
SInStyle.DebugOutputDir: ""
SInStyle.DebugEveryN: 0
```

默认关闭。开启时若目录、mask 类型、尺寸或三态数值非法，立即报错；若
某个 mask 文件不存在，则普通 shadow 运行对该帧 fail-open 为项目全
unknown 并记录 `reference_available=0`；若 mask 存在而配置要求的 label
缺失，则直接报错。冻结实验的审计必须显式给出预期行数和允许缺失的输入
序号，不允许“全部 reference 缺失”仍通过。

Replay 文件按独立、单调且不随 `Tracking::Reset()` 清零的输入序号选择。
CSV 同时记录输入序号、可重置的 SLAM Frame ID 和 reset epoch；禁止用
`Frame::mnId` 直接绑定外部 mask。

## 5. 第一轮同帧对照

优先使用 S0 已导出的 TUM `fr3/walking_xyz` 前 30 帧：

- TUM3 的相机畸变为零；
- DT-SLAM 和独立 SIn 使用相同 association；
- 参照 mask/label 已保存；
- 避免把 Bonn 原始像素域与 DT-SLAM 的 `P=K` 去畸变域混用。

统计至少包括：

- `reference_available`、`labels_available`；
- static/dynamic/unknown 像素和比例；
- 原始 ORB 特征数；
- dynamic/valid/unknown 区域内的 ORB 数；
- `author_dynamic_mask_hit_on_dt_orb_set`、深度支持的动态 ORB 命中，
  以及在 DT-SLAM 已提取 ORB 集合上反事实应用“剩余特征少于 250 则
  恢复全部特征”的 fallback；S1 中
  `actual_removed` 始终为 0；
- label 数；
- load、conversion、statistics 和 total runtime；
- `direct_slam_state_mutation=none`。

## 6. 验收不变式

1. `validMask` 与 `unknownMask` 互补；
2. `dynamicMask` 是 `validMask` 的子集；
3. static、dynamic、unknown 三类覆盖整张图且互斥；
4. 输入和输出尺寸一致；
5. mask 只接受 `0/125/255`；
6. 开关关闭时不读取文件、不增加检测开销、不改变现有输出；
7. 开关打开时 `mvbDynamic`、`mvpMapPoints` 和 MapPoint 创建逻辑不变；
8. replay 输出与输入 PNG 的三态编码计数完全一致（深度交集单独记录）；
9. 预期 reference/label 覆盖必须完整，输入序号连续且不受 reset 影响；
10. label 0 与正 label 上的作者动态像素分别记录，二者守恒全局动态数。

## 7. 当前不能宣称

- reference replay 不是 DT-SLAM 内部动态检测；
- 不能用 replay mask 的 ATE 证明 native 重新实现有效；
- `labels.png` 是公开 runner 导出的标签图，不预设它等于重聚类后的完整
  对象实例图；
- S1 不能宣称定位或地图改善；
- `reference_replay` 的耗时只包含 PNG I/O、转换和审计统计，不是 native
  detector 耗时；
- 作者 `<250` fallback 只在 DT-SLAM 已提取的 ORB 集合上做反事实估计，
  不是作者 ORB extractor 内部候选集合的逐项复现；
- CPU/GPU mask 不一致时不能任选有利结果作为真值。

## 8. 许可证边界

SIn 新增 detector 文件缺少清楚的文件级授权声明；其平面分割依赖的
PEAC/AHC 文件还带有 AGPL-3.0-or-later 标记。因此：

- 不复制 `DynaDetect.cc/.h` 或 PEAC/AHC 源码；
- reference replay 只消费运行结果；
- native 后端依据论文、接口规范和公开行为独立实现；
- 平面边缘若后续实现，只使用许可证清楚的现有依赖或本项目独立算法；
- 产物标记为 `[A] SIn-style clean-room reimplementation`，不称源码移植。
