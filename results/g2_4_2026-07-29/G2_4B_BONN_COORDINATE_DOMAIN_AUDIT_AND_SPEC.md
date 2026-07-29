# G2-4B Bonn 坐标域审计与最小一致域规格

日期：2026-07-29
状态：只读审计、最小实现和 online YOLO/mask smoke 完成；
坐标域门通过；G1-F/G1-D 继续锁定

## 1. 本步骤只回答什么

`[S]` 本步骤只回答 Bonn 数据进入 geometry shadow 前，RGB、registered
depth、ORB、semantic mask 和几何投影如何处于同一个坐标域。它不选择动态
阈值，不产生动态分类，不修改 tracking/mapping 状态，也不证明 unknown-object
检测能力。

## 2. 原始数据与本地 archive 事实

`[L]` Bonn 官方页面说明：数据与 TUM RGB-D 格式相同，depth 已注册到 RGB；
RGB 相机标定为：

```text
fx = 542.822841
fy = 542.576870
cx = 315.593520
cy = 237.756098
k1 = 0.039903
k2 = -0.099343
p1 = -0.000730
p2 = -0.000144
k3 = 0.000000
```

来源：

```text
https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/
```

`[S]` 本地只读检查得到：

```text
moving_nonobstructing_box:
  archive RGB PNG       = 778
  archive depth PNG     = 778
  rgb.txt rows          = 778
  depth.txt rows        = 782
  missing depth targets = 4

moving_obstructing_box:
  archive RGB PNG       = 590
  archive depth PNG     = 589
  rgb.txt rows          = 590
  depth.txt rows        = 592
  missing depth targets = 3
```

两类图像均为 `640x480`；RGB 是 8-bit RGB PNG，depth 是 16-bit grayscale
PNG。缺失文件清单：

```text
moving_nonobstructing_box:
  depth/1548266066.11057.png
  depth/1548266073.78529.png
  depth/1548266081.39329.png
  depth/1548266088.86790.png

moving_obstructing_box:
  depth/1548339353.13262.png
  depth/1548339360.70714.png
  depth/1548339368.21497.png
```

`[S]` 因此后续 association 工具必须同时检查时间阈值、一对一约束和实际文件
存在性。仅解析 `rgb.txt/depth.txt` 会生成不可运行的关联。

## 3. 当前代码的坐标域

`[S]` 当前 RGB-D tracking 路径：

1. ORB 在输入 RGB/gray 像素域产生 `mvKeys`；
2. `Frame::UndistortKeyPoints()` 用 `K,D` 生成 `mvKeysUn`；
3. `Frame::ComputeStereoFromRGBD()` 在 `mvKeys` 的原始像素位置读取 registered
   depth；
4. 右像素和后续针孔反投影使用 `mvKeysUn`。

`[S]` 这不是“在 undistorted keypoint 上读取 raw depth”。当前关键点深度读取
路径本身与 registered raw depth 是一致的。

`[S]` 当前 semantic 路径把输入 RGB 直接交给 YOLO，所得 mask 与输入 RGB
同尺寸；feature/mask 标签使用 raw `mvKeys`，所以 TUM 零畸变配置下保持一致。

`[S]` 当前 geometry warp 不接收 distortion coefficients。它用：

```text
x = (u-cx) z / fx
y = (v-cy) z / fy
u' = fx x'/z' + cx
v' = fy y'/z' + cy
```

直接处理输入 depth pixel。因此它只实现无畸变针孔域。`Tracking` 已在
non-zero tracking distortion 且没有 dataset-authorized dedicated raw pinhole
model 时拒绝启动 geometry。

## 4. 冲突结论

`[S]` Bonn 的 raw RGB/depth pixel 属于非零畸变域，而当前 geometry warp
把它当无畸变针孔域。直接把 Bonn 官方 `fx/fy/cx/cy` 填入
`Geometry.Camera.*` 不能消除这个冲突，因为官方同时给出了非零 distortion。

`[S]` 用 OpenCV 4.13、官方 `K,D` 和 `P=K` 对 `65x49` 像素网格做只读测量：

```text
raw -> undistorted displacement:
  median = 0.675 px
  p90    = 1.124 px
  p95    = 1.208 px
  max    = 4.227 px

P=K remap destination pixels with in-bounds raw source:
  99.401%
```

`[S]` 最大偏移已超过 geometry 当前若干 1/2-pixel risk band 的尺度，不能把
它当作可忽略误差。该测量只说明坐标不一致具有工程意义，不证明它对动态检测
精度的具体影响。

## 5. 候选设计比较

### 5.1 Raw distorted geometry

`[A]` 为 geometry unprojection/projection 增加 Brown-Conrady distortion，
可保留 raw RGB/depth/mask 域。

`[S]` 风险是需要同时处理 full-resolution、scale-2 pyramid、采样和投影边界，
并给 geometry 增加另一套相机模型与确定性测试。它会扩大当前科学门之前的代码
表面积。

### 5.2 统一 undistorted pinhole 输入域

`[A]` 用官方 `K,D` 将 RGB 和 registered depth 一起映射到 `P=K` 的
`640x480` undistorted pinhole 域；在该域运行 YOLO、ORB 和 geometry，并在
SLAM 设置中使用同一个 `K`、零 distortion。

`[S]` RGB 使用线性插值；16-bit depth 使用最近邻插值和零边界，避免对深度值
做跨表面线性混合。YOLO 必须接收已经 rectified 的 RGB，使在线 mask 天然位于
同域；未来 precomputed mask 必须来自 rectified RGB，或显式用最近邻 remap。

`[S]` 该设计不改变相机物理 pose，也不改变 GT trajectory 坐标系，只改变图像
采样坐标与相机投影模型。

### 5.3 冻结选择

`[S]` G2-4B 选择“统一 undistorted pinhole 输入域”作为最小候选设计。原因是
它复用 OpenCV 成熟标定组件，并让 RGB/depth/ORB/mask/geometry 共用单一模型；
它不是 ReFusion、DetectFusion 或其他论文整体方法的复现。

`[H]` `P=K` 可以在不裁剪画面的情况下保留足够有效深度和边界结构。这个假设
必须由实现后的真实样本审计验证；99.401% 只表示 remap source 坐标在图像内，
不表示 depth 有效，也不表示对象边界正确。

## 6. 下一小步的实现边界

`[S]` 已在 RGB-D runner 输入端增加默认关闭、Bonn 专用的联合
rectification：

```text
raw RGB ------linear remap------> rectified RGB -> YOLO + TrackRGBD
raw depth ----nearest remap-----> rectified depth -> TrackRGBD + geometry
online mask <--- YOLO(rectified RGB), no second remap
Camera K -----------------------> official K
Camera distortion -------------> zero after rectification
Geometry K ---------------------> same official K
```

`[S]` 零畸变/TUM 配置必须旁路 remap，保持现有输入逐像素不变。不得在
`Frame` 内对 depth 再次 rectification，也不得同时使用非零 `Camera.k*`，否则
会 double-undistort。

本步骤明确不允许：

```text
dynamic threshold/classification
mvbDynamic or mvpMapPoints mutation
YOLO model/code changes
Optimizer/g2o changes
extra PoseOptimization
G2-3R4 performance tuning
ATE improvement claims
```

## 7. 实现后的冻结验收

`[S]` 数学与数据域：

- 合成点的 raw-distort/undistort round-trip max error 不超过 `0.05 px`；
- RGB/depth 输出保持 `640x480`，depth 保持 `CV_16UC1`；
- depth 只使用 nearest-neighbor，invalid depth 保持零；
- runner、YOLO、ORB、semantic mask 和 geometry 记录同一 rectified-domain
  signature；
- tracking distortion 为零，tracking K 与 geometry K 一致。

`[S]` 回归：

- toggle 默认关闭；
- TUM 零畸变旁路的 RGB/depth 输入 hash 与当前路径一致；
- existing `geometric_warp_test` 全部通过；
- 关闭 geometry 时行为不变。

`[S]` Bonn 真实样本风险代理：

- 报告 remap 后 valid-depth retention、new invalid-border mass；
- 报告 RGB gradient 与 depth-discontinuity 对齐变化，只称 alignment proxy；
- 对 boundary/invalid risk 分层，不把边界像素缺失当动态真值；
- association 排除不存在的 depth 文件，并保存排除清单。

`[S]` Shadow 安全：

```text
dynamic_decision=none
direct_slam_state_mutation=none
G1-F/G1-D locked
```

实现结果见：

```text
G2_4B_BONN_COORDINATE_DOMAIN_RESULT.md
```

RGB/depth/ORB/geometry 子门和 host-GPU online YOLO/mask smoke 均已通过。
下一步可以开始 Bonn 自动选帧、box semantic coverage 审计和少量预标注；
仍不能直接开始动态阈值选择。
