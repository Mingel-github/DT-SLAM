# S1 SIn 风格区域 Shadow：行为参照增量结果

日期：2026-08-03  
状态：S1 第一个增量完成；S1 native clean-room detector 尚未实现  
运行边界：shadow-only

## 1. 本轮完成内容

本轮没有把 SIn mask 用于 ORB 提取或 Tracking。完成的是 S1 的稳定接口
和行为参照后端：

```text
独立 SInDSLAM 导出的同帧三态 mask/label
→ DT-SLAM SInStyleDynamicDetector reference_replay
→ 作者最终编码与 DT 深度支持状态分离
→ DT-SLAM 原始 ORB 只读覆盖统计
→ 作者 250 特征 fallback 反事实统计
→ CSV/invariant 审计
```

该增量的作用是先证明 DT-SLAM 能正确理解并审计 SIn 的输出，避免在
native 重写中同时混入坐标域、mask 极性、ORB 接口和算法差异。

## 2. 新增与修改

新增源码和配置：

- `DT-SLAM/include/SInStyleDynamicDetector.h`；
- `DT-SLAM/src/SInStyleDynamicDetector.cc`；
- `DT-SLAM/Examples/RGB-D/sin_style_shadow_test.cc`；
- `DT-SLAM/Examples/RGB-D/TUM3_SInStyleReferenceShadow.yaml`；
- `DT-SLAM/tools/audit_sin_style_reference_shadow.py`。

最小接入修改：

- `.gitignore`（忽略本地测试可执行文件）；
- `DT-SLAM/CMakeLists.txt`；
- `DT-SLAM/include/Tracking.h`；
- `DT-SLAM/src/Tracking.cc`；
- `DT-SLAM/src/System.cc`。

没有修改：

- `Frame.h/.cc`；
- `ORBextractor.cc`；
- `GeometricDynamicDetector.h/.cc`；
- `YOLOSegment.cc`；
- `Optimizer.cc`、g2o；
- `LocalMapping.cc`、`LoopClosing.cc`；
- MapPoint 创建和写入逻辑。

## 3. 接口和不变式

公开 SIn runner 的 final/dilated tracking mask 被显式保存为：

```text
raw 0   -> invalid/unassigned code
raw 125 -> static-coded state（不是独立静态真值）
raw 255 -> 作者 ORB 路径实际使用的 dynamic code
```

同时保存两套不混淆的结果：

```text
authorDynamicMask = raw == 255
projectDynamicMask = authorDynamicMask AND DT input depth valid
```

项目 depth-supported geometry state 满足：

```text
dynamic subset_of valid
static union dynamic = valid
valid union unknown = full_image
```

S1 没有调用 `UpdateDynamicFeaturesFromMask()`；否则 raw 125 会被 DT-SLAM
现有“非零即动态”的语义接口错误解释成动态。

作者公开 ORB 路径的行为只在 DT-SLAM 已提取的 ORB 集合上作反事实统计：
若 final mask 后剩余特征少于 250，则估计为恢复全部特征。它不是作者
ORB extractor 内部候选集合的逐项复现。本轮 CSV 区分：

```text
author_dynamic_mask_hit_on_dt_orb_set
depth_supported_dynamic_orb_count
would_keep_orb_count
counterfactual_fallback_on_dt_orb_set
counterfactual_removed_on_dt_orb_set
actual_slam_removed = 0
```

Replay 使用独立单调 `input_index` 选择外部文件；CSV 另存可重置的
`Frame::mnId` 与 `reset_epoch`。`Tracking::Reset()` 不会回绕外部文件编号，
也不会清空 reset 前已记录的 S1 行。

## 4. 构建与确定性测试

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
cmake ..
cmake --build . --target sin_style_shadow_test rgbd_tum -j4
```

测试输出：

```text
SIn-style shadow state tests passed
```

单元测试覆盖：

- 0/125/255 三态转换；
- 作者 raw dynamic 在无效深度处仍保留，项目 evidence 归入 unknown；
- dynamic 是 valid 子集；
- valid/unknown 全图守恒；
- 正 label、label 0 动态像素和完整 label 连通分量统计；
- 非法 mask 数值拒绝。

## 5. TUM3 30 帧同帧实验

数据：TUM `fr3/walking_xyz` 前 30 个作者 offset association。  
图像域：640×480，TUM3 零畸变原始域。  
参照：S0 保存的 GPU BroxFlow `mask_final` 和 `labels`。  
第一帧：独立 SIn 不运行 detector，因此 reference unavailable，并在
DT-SLAM 中显式输出全 unknown。

审计结果：

| 项目 | 结果 |
|---|---:|
| CSV 帧 | 30，连续且唯一 |
| 有 SIn reference 帧 | 29 |
| tracking state OK | 30/30 |
| reference PNG 与 CSV raw 三态计数不一致 | 0 |
| invariant 违反 | 0 |
| 作者 final dynamic mask 命中 DT-SLAM ORB 总数 | 9086 |
| 其中有 DT 当前深度支持的命中 | 7712 |
| DT-ORB 集合上的反事实 fallback 帧 | 0/30 |
| 本轮实际删除 ORB/MapPoint | 0 |

审计脚本现在要求 `input_index=0..29` 连续、仅允许 index 0 缺失，并要求
其余 29 帧 mask 与 label 同时存在。故错误目录、后缀错误或 reset 后帧号
回绕不能再以“全 unknown、pass=true”静默通过。

29 个有效参照帧的描述性统计：

| 指标 | 数值 |
|---|---:|
| raw dynamic ratio 均值 / 中位数 | 23.47% / 21.27% |
| 深度有效约束后的 dynamic ratio 均值 | 20.06% |
| unknown ratio 均值 | 32.39% |
| 正 label 数 | 每帧 11 |
| 正 label 的原始连通分量数均值 | 15.76 |
| DT-SLAM 原始 ORB 均值 | 1006.66 |
| 作者 final mask 命中 ORB 均值 | 313.31 |
| 作者 final mask 命中 ORB 比例均值 | 31.12% |
| 深度支持的 dynamic ORB 命中均值 | 265.93 |

作者 final mask 的动态像素中，29 帧合计 `1,714,934` 个落在正 label，
`376,103` 个落在 label 0。后者主要说明 final runner mask 膨胀后的动态
编码并不受正区域标签完全约束；不能要求 per-region 动态合计天然等于全局
动态数，也不能把 label 0 解释为静态背景。

这些是行为覆盖统计，不是动态检测 precision/recall。尤其 `labels.png`
只是公开 runner 导出的区域标签，不能称为对象实例 GT。

## 6. 运行时间

`reference_replay` 的 29 帧平均/中位/P95 为：

```text
4.99 / 5.00 / 5.16 ms
```

这包含 PNG 读取、三态转换、正 label 连通分量和 ORB 只读统计。它不是
native SIn detector 的耗时，也不能与独立 SIn 的 6.6--7.4 FPS 相替代。

30 帧短运行：

| 模式 | tracking mean | active_total mean | deadline miss | actual FPS |
|---|---:|---:|---:|---:|
| 标准 TUM3，SIn 关闭 | 11.16 ms | 19.54 ms | 0/30 | 27.78 |
| reference replay shadow | 16.44 ms | 24.97 ms | 0/30 | 27.79 |

`actual FPS` 受数据集时间戳 pacing 限制；这里能说明短序列没有 miss，不能
代表 native detector 已满足实时。

## 7. 当前结论

可以确认：

- SIn final 三态编码没有被错误二值化，作者 dynamic 与项目深度支持
  dynamic 已分开；
- reference frame、DT-SLAM frame 和 ORB 采样坐标在这组 TUM3 输入中
  对齐；
- 普通 shadow 的 missing reference 会 fail-open 为 unknown，冻结实验则
  要求 29/29 预期 reference 和 label 均存在；
- S1 接口、CSV 和关闭路径已建立；
- shadow 路径没有直接修改 SLAM 状态。

同步 replay 位于 `Track()` 前，会增加约 5 ms 的 I/O/审计成本并可能扰动
并行线程调度。因此这里不能进一步宣称“开启 replay 后轨迹执行行为绝对
不变”。

仍不能确认：

- DT-SLAM 已经实现 SIn 风格 native detector；
- mask 能准确分割未知箱子；
- S1 改善 ATE/RPE 或地图；
- CPU/GPU 两种 reference 行为等价；
- Bonn raw/rectified 域已经完成同帧对照；
- S2 可以开放。

## 8. 下一步（仍属于 S1）

1. 冻结 native detector 的 `DenseFlowProvider`、区域状态和 Reset 接口；
2. 依据论文 clean-room 实现米制深度的 3D 初始区域；
3. 使用许可证清楚的边缘/平面处理替代 AGPL PEAC，记录与作者实现差异；
4. 加入 dense flow、homography residual、双阈值、region-confined 判决和
   时序状态；
5. 在相同 TUM3 帧上与 DeepFlow CPU 或 Brox CUDA 对应 reference 成对
   比较；两种 reference 不平均；
6. native 行为能够解释且完整序列通过后，S1 才完成。S2 仍锁定。
