# G2-1 多参考深度一致性 Shadow 规格

日期：2026-07-28

## 1. 本阶段问题

G0 已经证明单参考稠密 depth warp 能产生可解释的有符号深度残差，但直接
positive seed 在深度边界附近存在较多背景证据，不能进入实际 SLAM 过滤。

G2-1 只回答一个问题：

> 同一当前像素由多个历史关键帧重复观测时，多参考证据是否比单参考证据具有
> 更好的动态代理可分性，并且同步耗时是否仍有可能满足当前实时目标？

本阶段仍为 shadow-only，不修改任何 SLAM 状态。

## 2. 文献依据与改造边界

### 2.1 文献原型 `[L]`

DynaSLAM（Bescos et al., IEEE RA-L 2018）对每个输入帧选择 5 个高重叠历史
关键帧，把参考关键帧中带深度的 ORB keypoint 投影到当前帧，并利用

```text
delta_z = z_projected - z_current
```

提取几何动态点。论文明确说明 5 个参考帧是计算量与检测精度之间的折中。

本地官方源码还显示其参考数据库容量为 20，动态检测实际使用最多 5 个参考帧。

证据位置：

- `/home/zhu/Desktop/paper_notes/dynaslam.md`
- `/home/zhu/Desktop/papers/2018_DynaSLAM_Tracking_Mapping_Inpainting.pdf`
- `/home/zhu/dynaslam_ws/DynaSLAM/include/Geometry.h`
- `/home/zhu/dynaslam_ws/DynaSLAM/src/Geometry.cc`

### 2.2 当前改造 `[A]`

G2-1 不复现 DynaSLAM。差异包括：

- 使用当前项目已有的稠密深度 forward warp 和 z-buffer；
- 第一轮取最近 5 个成功关键帧，不实现 DynaSLAM 的重叠排序；
- 沿用 G0 的 `0.10 m` 实验阈值，仅用于生成可审计的正/负残差计数；
- 不使用 DynaSLAM 的 `0.4 m` 论文阈值或 `0.6 m` 官方代码阈值；
- 不使用深度 patch 方差、region growing、形态学操作；
- 不产生二值动态决定。

因此本阶段准确名称为：

> recent-keyframe multi-reference dense depth-consistency shadow adaptation

## 3. 输入与参考缓存

输入：

```text
current_depth_meters : CV_32FC1
Tcw_current          : 4x4
K                    : 3x3 pinhole intrinsics
semantic_mask        : CV_8UC1，可为空，仅用于代理统计
```

参考缓存：

```text
最多保存 20 个成功写入的 ORB-SLAM2 关键帧
每个参考保存 depth_meters、Tcw、frame_id、timestamp
语义动态像素在写入参考深度前置为无效深度
G2-1 使用最近 5 个参考
少于 5 个参考时不输出 G2-1 结果
```

“20”和“5”来自 DynaSLAM 的公开设置；“最近 5 个而非重叠排序”是本阶段明确
记录的工程简化。

## 4. 输出

每个当前像素输出：

```text
comparison_count : 有效参考深度比较次数
positive_count   : residual > 0.10 m 的次数
negative_count   : residual < -0.10 m 的次数
consistent_count : |residual| <= 0.10 m 的次数
```

约束：

```text
positive_count + negative_count + consistent_count
== comparison_count
```

没有被任何参考覆盖的像素为 `comparison_count=0`，含义是没有几何证据，不是
静态。

C++ 只输出逐帧二维计数直方图：

```text
frame_id
timestamp
reference_count
comparison_count
positive_count
pixel_count
semantic_pixel_count
runtime
```

离线脚本再透明地枚举候选规则；C++ 不硬编码“多少票等于动态”。

## 5. 对照与验收指标

数据：

- TUM `fr3/walking_xyz`：人物 mask 只作为动态代理，不是真值；
- TUM `fr3/sitting_static`：同相机低动态诊断，不是真正静态；
- TUM `fr1/xyz`：静态负样本。

必须报告：

- 每种最小有效比较次数下的像素覆盖率；
- 每种正残差票数/比例下的 proxy precision、conditional recall；
- `fr1/xyz` 静态选择率；
- `fr3/sitting_static` 同相机低动态选择率；
- 单帧 G2-1 额外耗时；
- 端到端实际 FPS。

进入后续阶段的条件不是一个预先指定的数值阈值，而是同时观察到：

1. 相比单参考，存在可重复的 precision/静态误报改善；
2. 改善不是以动态代理召回几乎归零为代价；
3. walking 验证段、fr1 静态段和同相机 sitting 诊断结论一致；
4. 实测额外耗时与 30 FPS 目标的冲突被明确量化。

若不存在这样的工作点，则 G2-1 作为负实验归档，不通过事后补阈值掩盖。

## 6. 明确非目标

- 不写 `Frame::mvbDynamic`；
- 不清除 `mvpMapPoints`；
- 不调用额外 `PoseOptimization`；
- 不阻止 MapPoint 创建；
- 不生成深度区域 mask；
- 不修改 YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing；
- 不实现 MAD、Student-t、时序状态或多参考重叠排序；
- 不宣称复现 DynaSLAM。
