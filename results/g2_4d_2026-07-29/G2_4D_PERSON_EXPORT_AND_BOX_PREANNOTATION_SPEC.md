# G2-4D C++ Person 导出与 Box 预标注规范

日期：2026-07-29

## 1. 阶段目标

G2-4D 只为 G2-4 的人工/自动复核准备独立证据，不产生几何动态判决，也不改变 SLAM。

本阶段分为：

- **G2-4D1**：在已冻结的 48 个 Bonn development/review candidates 上，导出真实 C++ 语义管线的 person 检测框和最终 person filter mask。
- **G2-4D2**：使用不读取 geometry score、residual、region label 的方法生成 box 预标注，再审计 person mask 对 box 区域的覆盖。

以下状态保持不变：

```text
dynamic_decision = none
direct_slam_state_mutation = none
G1-F = locked
G1-D = locked
```

候选帧由 geometry proxy 条件化选择，只用于开发和故障分析，不是独立 holdout，也不能用于无偏序列级指标。

## 2. G2-4D1 输入

每条 Bonn 序列使用：

- `BONN_GeometryPyramidEvidenceShadow.yaml`；
- 原始 RGB 图像；
- G2-4C 修正后的 depth-time `selected_frames.csv`；
- 当前冻结的 `weights/yolov8n-seg.onnx`；
- 当前 `RGBDInputRectifier`；
- 当前 `YOLOSegment`，阈值保持 `conf=0.5`、`NMS=0.45`。

候选清单：

- `moving_nonobstructing_box`：24 帧；
- `moving_obstructing_box`：24 帧。

## 3. 坐标域和输出语义

原始 RGB 先经 `RGBDInputRectifier` 映射到 `P=K` 的 640×480 无畸变针孔域，再送入 YOLO。导出的 RGB、框和 mask 均处于同一个 rectified domain。

当前 `YOLOSegment` 的确切输出约定为：

- 仅保留 COCO class 0 `person`；
- `Detection` 仅含边界框和置信度；
- mask 为全部 NMS 后 person 实例 mask 的并集；
- mask 类型为 `CV_8U`，下游实际约定为
  `0=not filtered`、`nonzero=person-filtered`；
- 当前实例 mask 从 proto/bbox 缩放时使用线性插值，因此边缘允许出现
  `1..254`，不能把真实输出误写成严格的 0/255 二值图；
- 并集 mask 最后经过 7×7 椭圆核膨胀。

因此输出必须命名为 **final person filter mask**，不得写成 raw instance mask 或人工真值。

## 4. G2-4D1 最小实现

新增独立可执行程序，逐帧同步执行：

```text
read selected_frames.csv
→ read raw RGB
→ rectify RGB
→ PushFrame(seq)
→ WaitForMask(seq)
→ verify GetMaskSeq()==seq
→ GetDetections()
→ export rectified RGB / final mask / overlay / CSV
```

一次只提交一帧；读取框之前不提交下一帧，以保证框和 mask 属于同一个序号。输出目录若已存在则拒绝覆盖。

每帧 manifest 至少记录：

- sequence、selection role/rank、source frame；
- RGB/depth timestamp 和相对路径；
- rectified domain signature；
- semantic sequence id 和返回 mask sequence id；
- mask size/type、非零像素数和比例；
- `1..254` 中间值像素数和比例；
- person detection count。

每个 detection 记录 bbox 和 confidence。没有 person 时必须保存全零 mask，并在 manifest 中明确 `detection_count=0`。

## 5. G2-4D1 验收

- 两序列各 24 帧，合计 48 帧，无丢帧、无重复；
- `requested_seq == returned_mask_seq` 对全部帧成立；
- RGB 和 mask 均为 640×480；
- mask 类型为 `CV_8UC1`，动态过滤语义为 `mask!=0`；
- 中间灰度像素必须单独计数，不能在导出器内二值化或修复；
- detection count 为 0 时 mask 必须为空；
- 有效 bbox 全部位于图像边界内；
- 每一张输出可追溯到修正后的 depth-time candidate；
- 不修改 `YOLOSegment.cc`、`Optimizer.cc` 或 Tracking 状态。

## 6. G2-4D2 独立性要求

Box 预标注不得读取或优化以下字段：

- positive/negative/inconsistent residual；
- geometry region score、region id；
- G2-4C 的 proxy role、rank 或 proxy value；
- person mask 的像素形状。

候选选择已受 geometry proxy 条件化，因此 selection role 可以用于展示分层，但不能参与 box mask 生成。

允许的 box 预标注来源是：

- RGB/depth 单帧外观与深度；
- 与 geometry evidence 无关的通用交互式/提示式分割模型；
- agent 给出的粗 box prompt，加独立的单帧分割；
- 最终人工复核。

若本地没有独立分割模型或可靠对象提示，先完成 D1 并报告缺口，不得用 geometry mask 冒充 box GT。

## 7. G2-4D2 指标边界

预标注未经过人工确认前，只能报告：

- `proxy_box_*`；
- `semantic_coverage_review_*`；
- 需复核的帧列表。

不得报告 box detection precision/recall、IoU 真值或独立运动识别率。person mask 与 box 预标注的重叠只能回答语义分支可能覆盖多少 box 区域，不能证明 box 是否动态。

## 8. 非目标

- 不选择几何动态阈值；
- 不把 person mask 或 box 预标注写入 `mvbDynamic`；
- 不过滤 `mvpMapPoints`；
- 不进入 G1-F/G1-D；
- 不修改 YOLO 模型或后处理；
- 不修改 Optimizer、g2o、LocalMapping、LoopClosing；
- 不将 48 帧开发候选包装成独立评测集。
