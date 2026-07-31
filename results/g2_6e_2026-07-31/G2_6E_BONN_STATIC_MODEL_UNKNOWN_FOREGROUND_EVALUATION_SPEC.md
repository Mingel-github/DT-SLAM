# G2-6E Bonn 静态模型未知前景评价规范

日期：2026-07-31
状态：冻结候选；evaluation-only，不属于运行时 detector

## 1. 研究问题

G2-5A 已表明 sparse observed-flow minus ego-flow 在人物语义区域具有连续证据，
但 G1-F0A/F0B 也表明：单帧、单特征的 `q6/q8/q10` hard threshold 不能安全
形成真实 MapPoint 删除决定。

G2-6E 只回答：

> Bonn 未知动态物体（主要是箱子）所在区域中，现有 sparse ego-flow/depth
> evidence 是否具有足够的点数、空间富集和短时连续性，值得继续研究多特征运动
> 一致性分组？

本阶段不回答“哪个阈值可以部署”，也不产生动态 SLAM 状态。

## 2. 方法身份与来源边界

### `[L]` 数据集提供的事实

Bonn RGB-D Dynamic Dataset 官方页面提供：

- OptiTrack 相机轨迹；
- Leica BLK360 扫描得到的静态环境 ground-truth point cloud；
- full point cloud 和 1 mm subsampled section，均为 ASCII PLY；
- RGB 相机内参和非零畸变；
- 模型对齐公式及官方 `compute_global_transformation.py`。

相关论文为 Palazzolo et al., *ReFusion*, IROS 2019。

### `[A]` 借用的物理测量

把静态模型投影到当前相机视角，用 z-buffer 得到 expected static depth，再与
当前 registered depth 比较。模型—观测不一致作为动态候选的思想与
ReFusion 的静态模型 registration residual/free-space reasoning 有关。

### `[S]` 本项目的评价设计

本项目把模型投影只用于离线评价：

```text
official static point cloud
+ official camera pose
→ expected static depth
− current rectified depth
− exact C++ person mask
→ unknown-foreground review proxy
```

这不是 ReFusion 复现，也不是逐像素动态 GT。

### `[H]` 待验证假设

- 官网的首帧模型对齐公式可一致地推广到逐帧 camera-to-model 变换；
- 1 mm subsampled section 对开发序列视场具有足够覆盖；
- 点云投影孔洞、扫描遮挡和传感器噪声可以通过 validity/risk 分层控制；
- 去除 person mask 后的正向模型—观测残差能作为未知前景 review proxy。

上述假设必须先在 Bonn 真静态序列上验证，不能在 moving-box 序列上边看结果
边修改坐标链。

## 3. 官方坐标信息

官网提供：

```text
T_g = T_ROS^-1 * T_0 * T_ROS * T_m
```

官方脚本实际写为：

```text
T_g = T_ros * T_0 * T_ros * T_m
```

两者一致，因为给定 `T_ROS` 为自逆矩阵。

注意：

- `groundtruth.txt` 是 `timestamp tx ty tz qx qy qz qw`；
- 官方脚本按 `qw,qx,qy,qz` 构造旋转；
- `T_m` 的 3×3 部分含约 `1.0593` 的统一尺度，不能强制正交化；
- `T_g` 的官方用途是把 RGB-D sensor reference-frame model 对齐到 GT model；
- 逐帧渲染时用每帧 `T_i` 替换 `T_0` 是当前待验证的推广，不先写成既定事实。

冻结两个显式候选并仅用静态序列判定方向：

```text
H1: T_model_from_camera(i) =
    T_ROS * T_i * T_ROS * T_m

H2: 若官网语义只适用于首帧 reference model，则：
    T_model_from_camera(i) =
    T_g(0) * T_camera0_from_camerai
```

H1/H2 必须根据官方 pose 定义严格展开，并验证二者是否数学等价；若不等价，
不得通过选择静态残差更小者来隐式调参，必须回查坐标定义。

## 4. 图像坐标域

沿用已经通过的 G2-4B 共同去畸变域：

```text
raw RGB   --linear remap--> rectified RGB
raw depth --nearest remap-> rectified depth
P = official K
tracking distortion = 0
```

expected static depth 必须投影到同一个 rectified pinhole domain。不得混用：

- raw distorted RGB/depth；
- `mvKeys` 与 `mvKeysUn`；
- rectified semantic mask 与 raw model projection。

## 5. 分阶段实现

### G2-6E0：资产与坐标链可行性

- 保存官方页面 URL、脚本、文件大小和 SHA-256；
- 检查 PLY header、属性、范围和单位；
- 解析 GT pose 并建立显式变换单元测试；
- 不生成 unknown proxy。

### G2-6E1：静态序列配准审计

使用 `rgbd_bonn_static_close_far`：

- 渲染 expected static depth；
- 计算 model coverage 和 current-depth comparison coverage；
- 分别报告 signed residual、absolute residual；
- 分层报告 depth boundary、model boundary、invalid depth 和 image border；
- 保存 RGB/current/model/residual overlay；
- 不依据 moving-box 结果调整坐标链或误差方向。

只有静态序列能证明模型投影在大部分有效区域对齐，才进入 E2。

### G2-6E2：开发 moving-box unknown-foreground proxy

使用已经打开的 development/review 序列，不打开 strict hold-out：

- `rgbd_bonn_moving_nonobstructing_box`；
- `rgbd_bonn_moving_obstructing_box`。

生成：

```text
valid_model_comparison
positive_model_residual
negative_model_residual
person_excluded_unknown_foreground_proxy
```

正残差约定：

```text
r_model = z_expected_static - z_current
r_model > 0：当前观测比静态模型表面更靠近相机
```

负残差保留为独立输出，不直接解释为动态。

### G2-6E3：与现有 F1 evidence 独立连接

按 frame timestamp/source frame id 连接：

- proxy 内外 ORB feature 数；
- raw ego-flow residual；
- semantic-blind normalized residual；
- correspondence quality；
- boundary/invalid risk；
- spatial nearest-neighbor distance；
- 2/3 帧短时支持。

G2-6E 不选部署阈值。它只判断是否存在可继续研究的多点/时序结构。

## 6. 必须报告的指标

### 模型与观测质量

- point-cloud projected coverage；
- z-buffer valid coverage；
- current/model joint valid coverage；
- 静态序列 residual median、MAD 和分位数；
- 图像中心/边缘、深度边界/非边界分层；
- frame-to-frame coverage continuity；
- 每帧投影与比较耗时。

### unknown proxy 风险

- true-static 序列 positive proxy ratio；
- person mask 排除前后 ratio；
- model boundary/invalid 邻域中的 proxy ratio；
- 连通区域面积分布，仅作诊断，不作对象分割；
- RGB/contact sheet 人工可视检查，但不把 proxy 写成 GT。

### F1 可行性

- proxy 内 eligible ORB feature 数；
- proxy 内外 residual 分布与 AUC（仅作为 proxy-conditioned 指标）；
- 每帧至少 `N` 个共同运动候选的比例，`N` 不在本阶段调成 detector 参数；
- spatial concentration 和短时 persistence；
- static 序列同类统计。

## 7. 放行条件

G2-6E1 进入 E2 前必须满足：

- 坐标方向和尺度由静态序列与官方定义共同支持；
- 不出现由畸变域错误造成的系统性边缘偏移；
- model/current joint coverage 足以评价箱子主要视场；
- 静态 residual 不表现为大面积单符号系统偏差。

G2-6E3 进入最小 motion grouping 研究前必须同时看到：

- unknown proxy 内有不止极少数 eligible feature；
- 多点 evidence 相对静态背景有稳定富集；
- 富集不是仅由 depth/model boundary 风险解释；
- 多帧上有重复，而不是单帧偶然值；
- 真静态风险可控。

本阶段不预注册一个虚假的统一数值阈值；先报告完整分布，再决定是否值得另立
有文献依据的 grouping SPEC。不得在同一开发数据上选择阈值后把它称为验证。

## 8. 明确非目标

- 不修改 `mvbDynamic`；
- 不删除 `mvpMapPoints`；
- 不修改 `Optimizer.cc`、g2o、LocalMapping 或 LoopClosing；
- 不增加 `PoseOptimization()`；
- 不修改 YOLO；
- 不把 GT pose、static model 或 proxy 接入运行时 detector；
- 不打开 `balloon_tracking` strict hold-out；
- 不实现 object tracker、TSDF、ICP 或 dense scene flow；
- 不把 proxy precision/recall 写成真实动态分割精度。

## 9. 失败处理

- 若静态模型对齐失败：冻结 G2-6E 为失败评价路线，不用 moving 数据调坐标链；
- 若模型覆盖不足：记录不可评价区域，不把它们当静态；
- 若 proxy 被扫描边界/孔洞主导：停止，不用于 F1 评价；
- 若 proxy 可靠但 F1 unknown evidence 弱：停止 lightweight feature-filter 路线；
- 若 proxy 与 F1 都显示稳定多点证据：下一步只研究一个有论文依据的最小
  multi-feature/short-temporal grouping，继续 shadow-only。
