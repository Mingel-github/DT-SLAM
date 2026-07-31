# G2-4R1 法向量＋点平面距离分段 Shadow 结果

日期：2026-07-30

## 1. 结论

G2-4R1 的实现、坐标域、输出不变量和确定性测试均通过，但当前固定参数的
frame-wise normal+distance segment **没有通过预注册的区域表示继续条件**。

准确结论是：

> Tateno-style 单帧法向量/点平面距离边界能把 G2-3R0 的大型背景 component
> 切开，也能在 coarse bbox oracle 下找到较纯净的目标片段；但当前适配在
> Bonn 上严重过分割，并把大量目标有效深度标为 boundary/unknown，不能提供
> 稳定的整对象容器。

因此：

```text
G2-4R1 frame-wise adaptation       = frozen negative result
dynamic_decision                  = none
direct_slam_state_mutation        = none
G1-F / G1-D                       = locked
```

这不否定 Tateno 的完整增量分段系统，也不否定 DetectFusion。当前没有实现：

- Tateno 的全局 label propagation、segment merging 和 map update；
- DetectFusion 的 static surfel map、ICP residual、K=2 residual grouping；
- 自动运动 segment selector。

## 2. 方法身份与文献边界

### `[L]` 文献原型

Tateno 等的 frame-wise geometric segmentation 使用：

```text
bilateral smoothing
→ vertex / central-difference normal
→ 8-neighbor concavity-aware normal edge Φ
→ noise-aware point-to-plane distance edge Γ
→ connected components
```

本阶段使用原文 `tau_phi=0.94`，并使用 Tateno 引用的 Nguyen Kinect
轴向噪声形式：

\[
\sigma_z(z)=0.0012+0.0019(z-0.4)^2\ {\rm m}.
\]

原始来源：

- Tateno et al., *Large Scale and Long Standing Simultaneous
  Reconstruction and Segmentation*, CVIU 2017:
  <https://campar.in.tum.de/pub/tateno2017cviu/tateno2017cviu.pdf>
- Nguyen et al., *Modeling Kinect Sensor Noise for Improved 3D
  Reconstruction and Tracking*, 3DIMPVT 2012；本地副本：
  `results/g2_4_motion_grouping_2026-07-30/Nguyen_2012_Kinect_Sensor_Noise.pdf`

### `[A/S]` 当前适配

- 只做单帧、离线、640×480 depth segmentation；
- 在既有 Bonn/TUM rectified pinhole 域处理 `CV_32F` 米制深度；
- bilateral 半径 2 px、空间 sigma 2 px、深度 sigma 0.05 m 是固定工程参数；
- boundary/normal-unavailable/invalid depth 与可靠 segment 分开；
- G2-3R0 只作为同输入的配对表示基线；
- coarse bbox 和 F1 residual 只在分段完成后用于 development oracle review，
  不参与边界生成、合并或参数选择。

Bonn 上的 Nguyen 参数明确记录为：

```text
noise_model_out_of_domain = true
noise_model_transfer_status = unvalidated_transfer
```

## 3. 输入纠正

实现 smoke 时发现初稿命令误指向 F3U node CSV。F3U 的 `measured` 节点还要求
局部刚性邻域支持，不等于 SPEC 要求的完整 F1 sparse ego-flow evidence。

正式实验已改为：

```text
balloon_f1_features.csv
balloon2_f1_features.csv
evidence_state == measured
slam_residual_magnitude_px
```

因此本结果与 F4 的 F1 证据范围一致，没有用 F3U 的额外支持条件暗中缩小样本。

## 4. 数据与输出

### Development proxy

```text
Bonn balloon   : 8 个 bbox 可连接候选帧
Bonn balloon2  : 9 个 bbox 可连接候选帧
合计           : 17 帧
```

这些 bbox 是冻结的 RGB-only coarse review proxy，不是运动 GT。oracle
single segment 只衡量“表示中是否存在容纳目标的片段上界”，不是部署时可用的
选择器。

### 真静态退化检查

```text
TUM fr1/xyz：前 150 帧
```

### 产物

```text
results/g2_4r1_2026-07-30/
  balloon/
  balloon2/
  fr1_xyz_static150/
```

每个目录包含 `per_frame.csv`、`per_segment.csv`、
`paired_comparison.csv` 和 `summary.json`；动态候选目录还包含边界 mask、
label overlay 和 contact sheet。

## 5. 主要结果

### 5.1 纯表示统计

| 序列             | 帧数  | normal 区域数中位 | ≤64 px 区域比例中位 | boundary/valid 中位 | 最大区域/segmentable 中位 | normal Python 时延中位 |
| -------------- | ---:| ------------:| -------------:| -----------------:| -------------------:| ------------------:|
| balloon        | 8   | 2487         | 94.51%        | 29.63%            | 29.46%              | 214.68 ms          |
| balloon2       | 9   | 2669         | 94.98%        | 29.14%            | 23.71%              | 214.89 ms          |
| fr1/xyz static | 150 | 406          | 93.38%        | 8.33%             | 68.87%              | 194.65 ms          |

对照 G2-3R0：

| 序列             | baseline 区域数中位 | 最大区域/segmentable 中位 | baseline Python 时延中位 |
| -------------- | --------------:| -------------------:| --------------------:|
| balloon        | 25             | 85.36%              | 3.52 ms              |
| balloon2       | 27             | 67.03%              | 3.48 ms              |
| fr1/xyz static | 15             | 82.97%              | 3.28 ms              |

这里的时延来自未优化 Python/NumPy/OpenCV 离线审计，不能解释成等价 C++ 在线
时延；但足以说明当前原型不是可直接接入前端的实现。

### 5.2 目标 proxy 表示上界

17 帧合并后：

| 指标                                   | normal+distance | G2-3R0 oracle 对照 |
| ------------------------------------ | ---------------:| ----------------:|
| best single-segment bbox coverage 中位 | **0.488**       | 0.679            |
| best single-segment bbox purity 中位   | 1.000           | 0.998            |
| best single-segment bbox IoU 中位      | 0.488           | 0.554            |
| 可形成 selected/background residual 比较  | 14/17           | 16/17            |
| selected median > background         | 14/14           | 14/16            |
| inside-box median > background       | 16/16           | 16/16            |

normal+distance 的 oracle target 片段较纯，但覆盖更低。最关键的是：

```text
normal segment 集合可覆盖 >=50% bbox-valid depth：15/17
normal segment 集合可覆盖 >=80% bbox-valid depth： 1/17
G2-3R0 可覆盖 >=80%                         ：17/17
```

也就是说，16/17 帧在分段阶段已有超过 20% 的 bbox 有效深度落入
boundary/unknown；后续无论组合多少 `label>=0` segment 都无法恢复 80% 覆盖。

### 5.3 已知 F4 泄漏帧

在 `balloon 39/200/252` 上，bbox oracle 可以选到纯净小片段，所选片段的 bbox
外 F1 feature leakage 均为 0。但这不能宣称解决了 F4 的自动聚合失败：

- F4 按 bbox 内 eligible feature 数选 region；
- G2-4R1 按 bbox IoU 选择 oracle region；
- bbox 在实际未知动态检测中不可用。

这个结果只证明“表示中存在纯净目标片段”，不证明系统知道该选哪个片段，也
不证明这些片段合起来是完整对象。

## 6. 预注册条件判定

| 条件                                    | 结果                              |
| ------------------------------------- | ------------------------------- |
| 已知大背景泄漏能否被切开                          | oracle 上可以，但无部署 selector        |
| 14 个可比较帧是否保持 motion residual 方向       | 14/14，保持                        |
| 目标是否不依赖大量微小 segment                   | **失败**                          |
| fr1/xyz 是否完全退化成全图单 segment/全 boundary | 未出现极端二值退化，但仍有 406 区域、93.38% 小区域 |
| 确定性                                   | 通过                              |
| 是否需要观察结果后调参数才能继续                      | 是；预注册禁止                         |

最终按停止条件：

```text
representation_continue = false
```

## 7. 确定性与不变量

```text
python3 -m py_compile                              PASS
synthetic plane/step/invalid-depth self-test       PASS
balloon replay non-timing CSV                      identical
balloon replay all PNG/contact sheet               identical
dynamic_decision                                   none
direct_slam_state_mutation                         none
```

没有修改：

- `Tracking`；
- `GeometricDynamicDetector`；
- `mvbDynamic` / `mvpMapPoints`；
- YOLO；
- `Optimizer.cc` / g2o；
- LocalMapping / LoopClosing。

## 8. 决策与下一步

本阶段不允许根据已打开的 balloon/balloon2：

- 放大 Nguyen sigma；
- 调 bilateral sigma；
- 增加 merge/area/residual threshold；
- 用 bbox 或 residual 驱动 segment merge；
- 将 oracle segment 直接写成动态对象。

完整 Tateno 系统中的 label propagation/segment merging 可能缓解单帧碎裂，但
这会进入新的多帧地图分段系统，工程范围和论文故事都会明显扩大；当前结果不能
作为直接实现它的授权。

下一步回到路线决策，而不是 G1：

1. 保留 F1 continuous sparse ego-flow residual；
2. 以原始论文为依据，重新审计轻量“多 feature 运动一致性分组”的最小可行
   形式；
3. 若没有同时满足现有短轨迹支持和类别无关目标的可靠方案，明确暂停这一分支，
   而不是继续堆阈值；
4. G1-F/G1-D 在获得保守、可验证的动态判决前继续锁定。
