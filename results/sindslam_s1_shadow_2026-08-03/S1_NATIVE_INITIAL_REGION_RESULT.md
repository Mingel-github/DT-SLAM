# S1 原生初始三维区域增量结果

日期：2026-08-03  
阶段：S1 内部第二个增量  
状态：通过工程与表示审计；不构成动态检测

## 1. 本轮做了什么

在已完成的 SIn 作者结果 replay 接口旁，新增了一个 clean-room 初始区域
提供者：

```text
CV_32F 米制深度
→ finite && 0 < depth < 6 m
→ 针孔反投影 XYZ
→ 确定性全分辨率 OpenCV K-means
→ -1 / 正区域 ID
```

区域数使用论文给出的尺度规则：

```text
K = round(width * height / 25600)
```

640×480 时为 12。实现暂时复用项目现有 `JiGeometryBaseline` 中已经测试
的 XYZ 构造和 K-means 数值路径，但没有调用 Ji 的重投影动态判决。

方法身份必须写成：

> `[A] SIn-style clean-room initial 3D region clustering`

它不是 SInDSLAM 完整 geometric re-clustering。当前尚无深度/平面边缘
切分、RAG 合并、稠密光流、运动残差、时序状态或动态 mask。

## 2. 新增文件

- `DT-SLAM/include/SInStyleInitialRegionClusterer.h`；
- `DT-SLAM/src/SInStyleInitialRegionClusterer.cc`；
- `DT-SLAM/Examples/RGB-D/TUM3_SInStyleNativeInitialRegionShadow.yaml`；
- `DT-SLAM/tools/audit_sin_style_native_initial_regions.py`。

并扩展：

- `DT-SLAM/CMakeLists.txt`；
- `DT-SLAM/include/Tracking.h`；
- `DT-SLAM/src/Tracking.cc`；
- `DT-SLAM/Examples/RGB-D/sin_style_shadow_test.cc`。

## 3. 输出语义和安全边界

原生初始区域：

```text
-1 = invalid / excluded / unmeasured
>0 = initial region ID
```

CSV 明确记录：

```text
native_dynamic_state_available = 0
native_dynamic_decision = none
direct_slam_state_mutation = none
actual_slam_removed = 0
```

本轮没有修改 `Frame::mvbDynamic`、`mvpMapPoints`、MapPoint 创建、YOLO、
Optimizer、g2o、LocalMapping 或 LoopClosing。

## 4. 构建和测试

构建目标：

```text
sin_style_shadow_test
rgbd_tum
```

构建成功。确定性测试输出：

```text
SIn-style shadow state and initial-region tests passed
```

测试覆盖：

- 单平面 K=1 区域守恒；
- invalid、NaN 和 `depth >= 6 m` 不进入聚类；
- K=4 深度阶跃输入；
- 固定种子重复运行和 Reset 后标签一致；
- 非法 divisor 被拒绝；
- 既有 0/125/255 reference 三态不变式继续通过。

## 5. TUM3 30 帧配对审计

数据：TUM `fr3/walking_xyz` 前 30 个作者 offset association。  
作者参照：S0 GPU BroxFlow 保存的 `mask_final` 与 final `labels`。  
运行配置：作者 reference replay 与 native initial partition 同帧开启，Viewer
关闭，所有结果保持 shadow-only。

### 5.1 工程不变式

| 项目 | 结果 |
|---|---:|
| CSV 行 | 30，input index 0--29 连续 |
| 作者 reference 可用 | 29/30；首帧按协议缺失 |
| 作者 PNG 与 CSV 三态计数不一致 | 0 |
| native partition 可用 | 30/30 |
| native 输出区域 | 每帧 12 |
| native label PNG 写入 | 30/30 |
| depth/label/区域数守恒违反 | 0 |
| 实际删除 SLAM 观测 | 0 |
| 两次运行 native label PNG 差异 | 0；逐文件字节一致 |

两轮运行中 native 区域图和区域面积统计完全一致。ORB 落区计数有 20 帧
出现小差异；这是运行间 ORB 集合差异，不能误写成区域聚类不确定性。

### 5.2 表示统计

| 指标 | 结果 |
|---|---:|
| native 有效深度覆盖均值 | 66.64% 图像 |
| native ORB 落区覆盖均值 | 69.87% |
| 每帧有效聚类深度像素，中位数 | 208,483 |
| 最小区域面积，中位数 | 8,827 px |
| 最大区域面积，中位数 | 29,110 px |
| 作者 final 正区域数，中位数 | 11 |
| native 初始区域数，中位数 | 12 |
| label-permutation invariant ARI 均值 | 0.5540 |
| label-permutation invariant NMI 均值 | 0.7527 |
| native 边界 precision，2 px 容差均值 | 0.3483 |
| 作者 final 边界 recall，2 px 容差均值 | 0.6377 |

ARI、NMI 和边界指标仅作描述。作者 `labels.png` 是经过后续边缘切分和合并
的 final labels，而 native 当前只有初始 K-means，因此这些数值既不是动态
检测准确率，也不是本增量的通过阈值。较低的边界 precision 说明初始 K-means
边界与作者最终区域边界仍有明显差异，正是后续 re-clustering 层要解释的
内容；不能把当前 12 个簇称为对象。

### 5.3 运行成本

| 阶段 | mean | median | P95 |
|---|---:|---:|---:|
| XYZ/sample prepare | 1.18 ms | 1.19 ms | 1.25 ms |
| K-means | 30.59 ms | 30.56 ms | 33.00 ms |
| label conversion | 0.42 ms | 0.40 ms | 0.60 ms |
| native initial total | 33.17 ms | 33.06 ms | 35.63 ms |

30 帧端到端短运行：

```text
active_total mean = 59.76 ms
deadline_missed   = 28/30
actual_fps        = 16.60
```

对照此前相同 30 帧：SIn 关闭时 active total 约 19.54 ms，只有 reference
replay 时约 24.97 ms。这里的 paired run 同时包含约 5 ms reference replay、
33 ms native K-means、label PNG 写入和审计；不能把 16.60 FPS 当作最终
detector 的纯计算速度。但数据已经足以否定“全分辨率、每帧从零 K-means
几乎没有成本”的假设。

## 6. 当前结论

已经确认：

- DT-SLAM 米制深度和内参可以稳定产生初始三维区域；
- invalid/unknown 与区域标签没有混淆；
- 区域和作者 reference 可在同一输入序号、同一像素域成对记录；
- 真实数据上的初始分区具有确定性；
- shadow 路径未改变 SLAM 状态。

尚未确认：

- 区域对应人物、箱子或任何对象；
- 哪个区域动态；
- native 输出接近作者完整 SIn mask；
- ATE/RPE 或地图得到改善；
- S2 可以开放。

## 7. 下一步

仍在 S1 内继续，不新增主阶段：

1. 先实现论文有明确依据的 coarse-to-fine/上一帧初始化区域状态，验证能否
   降低每帧从零全分辨率 K-means 的成本；
2. 再增加不复制 AGPL PEAC 源码的深度边缘切分；
3. 平面边缘与 RAG/直方图合并必须单独说明 clean-room 替代方案；
4. 区域表示通过后才加入 dense flow、相机运动补偿、区域内残差和时序
   dynamic state；
5. S1 完整 mask 行为通过前，S2 Tracking 过滤继续锁定。

原始证据：

- `native_initial_30.csv`、`native_initial_30.log`；
- `native_initial_30_reference_audit.json`；
- `native_initial_30_audit.json`；
- `native_initial_30_labels/`；
- `native_initial_30_repeat.csv/.log`；
- `native_initial_30_repeat_labels/`。
