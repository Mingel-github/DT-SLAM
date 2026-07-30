# G2-4D C++ Person 导出与 Box 预标注结果

日期：2026-07-29

## 1. 结论

G2-4D 已完成，但结论边界必须保持克制：

1. 48 个修正后的 Bonn depth-time development candidates 已全部通过真实 C++ YOLO 管线；
2. 导出的 person 框、最终过滤 mask、rectified RGB 和源帧一一对应；
3. person 语义只在人物出现时提供过滤，不能独立覆盖运动箱子；
4. RGB-only target-box 粗框支持“语义存在明确类别缺口”这一开发观察；
5. 粗框不是像素 GT，不能据此报告 box segmentation precision/recall 或动态检测率；
6. 本阶段没有产生 geometry dynamic decision，也没有改变 SLAM。

```text
dynamic_decision = none
direct_slam_state_mutation = none
G1-F = locked
G1-D = locked
```

## 2. 实现

新增独立可执行程序：

- `DT-SLAM/Examples/RGB-D/semantic_review_export.cc`

新增审计工具：

- `DT-SLAM/tools/audit_bonn_semantic_box_coverage.py`

新增输入预标注：

- `results/g2_4d_2026-07-29/target_box_bbox_preannotations.csv`

`semantic_review_export` 复用且不修改：

- `RGBDInputRectifier`；
- `YOLOSegment`；
- `conf=0.5`；
- `NMS=0.45`；
- CUDA Execution Provider。

逐帧只提交一个序号，并在取 detection 前验证：

```text
requested semantic seq == returned mask seq
```

输出全部位于 Bonn `P=K`、640×480 rectified domain。

## 3. 真实 mask 约定的修正

第一次运行按“0/255 严格二值”验收时失败。代码核对确认：

```cpp
cv::resize(maskBin, maskResized, cv::Size(box.width, box.height));
```

当前 resize 使用默认线性插值，因此 mask 边缘存在 `1..254`。随后 mask 做 person 实例并集及 7×7 椭圆核膨胀。

Tracking 的真实边界会执行：

```cpp
cv::compare(mask, 0, semanticMask, cv::CMP_NE);
```

因此正确约定是：

```text
type = CV_8UC1
0 = not filtered
nonzero = person-filtered
```

本阶段没有修改 YOLO 或偷偷二值化输出，而是：

- 原样保存真实 C++ mask；
- 单独记录 `1..254` 中间灰度像素；
- 所有覆盖审计使用 `mask != 0`，与 Tracking 一致。

| 序列 | nonzero mask pixels | 1..254 pixels | 中间值占 nonzero |
| --- | ---: | ---: | ---: |
| moving_nonobstructing_box | 178,962 | 24,222 | 13.53% |
| moving_obstructing_box | 150,874 | 16,420 | 10.88% |

这是一项已记录的语义后处理风险，不代表当前过滤极性错误。是否将 resize 改为 nearest 或在 resize 后重新 threshold，必须另开语义回归工单；G2-4D 不修改冻结的 semantic baseline。

第一次失败的部分输出被保留在：

- `nonobstructing_cpp_person_export_failed_binary_assertion/`；
- `obstructing_cpp_person_export_failed_binary_assertion/`。

它们是审计证据，不用于最终指标。

## 4. D1 导出验收

| 检查 | nonobstructing | obstructing |
| --- | ---: | ---: |
| candidate rows | 24 | 24 |
| unique source frames | 24 | 24 |
| seq/mask-seq mismatch | 0 | 0 |
| 非 640×480 输出 | 0 | 0 |
| 非 `CV_8UC1` mask | 0 | 0 |
| 空 detection 但非空 mask | 0 | 0 |
| 越界 detection bbox | 0 | 0 |
| person detections | 5 | 4 |
| 带 person mask 的帧 | 5 | 4 |
| 全零 mask 帧 | 19 | 20 |

短批次 CUDA 日志：

| 序列 | semantic median | p95 | 备注 |
| --- | ---: | ---: | --- |
| nonobstructing | 8.48 ms | 11.77 ms | 首帧 warm-up 约 263 ms |
| obstructing | 8.58 ms | 12.54 ms | 首帧 warm-up 约 260 ms |

这些候选帧不是按数据集帧率连续运行，且短批次均包含一次 CUDA warm-up。因此该表只验证导出管线，不替代完整系统 FPS。

## 5. D2 RGB-only target-box 预标注

为避免把 geometry evidence 循环当作自己的“真值”，粗框制作时只查看：

- rectified RGB；
- source frame id。

没有读取：

- residual；
- region id/score；
- proxy role/rank/value；
- geometry mask；
- person mask 形状。

预标注状态明确为：

```text
annotation_source = agent_rgb_only_coarse_bbox_v1
review_status = unverified
is_ground_truth = false
```

框表示目标箱子在图像中的粗可见范围；`absent` 表示目标箱子不可见。它不是精确对象 mask。框内 person-mask 比例会同时受到背景、人体遮挡和粗框误差影响。

## 6. Semantic coverage review

| 指标 | nonobstructing | obstructing |
| --- | ---: | ---: |
| candidates | 24 | 24 |
| target visible/partial/occluded | 24 | 16 |
| target absent | 0 | 8 |
| person detected（全部候选） | 5 | 4 |
| person detected（target visible） | 5 | 2 |
| proxy bbox person coverage mean | 7.52% | 1.69% |
| proxy bbox person coverage median | 0.00% | 0.00% |
| proxy bbox person coverage max | 54.66% | 16.56% |
| visible bbox coverage >1% | 5/24 | 2/16 |

逐帧检查显示：

- nonobstructing 中仅帧 297、306、313、364、385 有 person mask；
- 这些帧均是人物直接遮挡或搬运箱子；
- obstructing 中 target 可见时只有帧 158、325 有 person mask；
- 另外两次 person detection 出现在 target 已不可见的帧 138、350；
- 无人物的箱子帧中，当前 person-only 语义 mask 全为零。

因此可以支持的观察是：

> 当前语义分支按设计只提供 person 证据。运动箱子在无人遮挡时没有语义过滤；人物搬运箱子时，mask 与粗框的重叠主要来自人体遮挡，不能解释为箱子被正确分割。

不能支持的结论是：

- YOLO 对箱子的像素 recall 为某个精确百分比；
- 所有粗框中的像素都属于箱子；
- 箱子是否正在运动；
- geometry score 已经能够正确补足语义缺口。

## 7. 输出

真实 C++ 导出：

- `nonobstructing_cpp_person_export/`
- `obstructing_cpp_person_export/`

每个目录包含：

- `manifest.csv`
- `detections.csv`
- `summary.txt`
- `rgb/`
- `mask/`
- `overlay/`

覆盖审计：

- `semantic_box_coverage_review/per_frame_coverage.csv`
- `semantic_box_coverage_review/summary.json`
- `semantic_box_coverage_review/overlays/`
- 两张 bbox + person mask contact sheet

运行日志：

- `nonobstructing_cpp_person_export.log`
- `obstructing_cpp_person_export.log`
- `semantic_box_coverage_review_run.log`

## 8. 可复现命令

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
cmake ..
make semantic_review_export -j$(nproc)
```

数据集解压到 `<bonn_root>` 后，每条序列运行：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

./Examples/RGB-D/semantic_review_export \
  Examples/RGB-D/BONN_GeometryPyramidEvidenceShadow.yaml \
  weights/yolov8n-seg.onnx \
  <bonn_root>/rgbd_bonn_moving_nonobstructing_box \
  /home/zhu/dynaslam_ws/results/g2_4c_correction_2026-07-29/nonobstructing_depth_time/selected_frames.csv \
  /home/zhu/dynaslam_ws/results/g2_4d_2026-07-29/nonobstructing_cpp_person_export
```

CUDA/ONNX Runtime 动态库环境与当前冻结 semantic baseline 相同。

覆盖审计：

```bash
cd /home/zhu/dynaslam_ws

python3 DT-SLAM/tools/audit_bonn_semantic_box_coverage.py \
  --preannotations results/g2_4d_2026-07-29/target_box_bbox_preannotations.csv \
  --export-root results/g2_4d_2026-07-29 \
  --output-dir results/g2_4d_2026-07-29/semantic_box_coverage_review
```

工具为防止误覆盖而拒绝已有输出目录。

## 9. 下一决策

G2-4D 证明了“为何必须有类别无关几何分支”，但没有证明“当前 geometry score 已足够可靠”。

下一步仍属于 G2-4 的判决门控，建议使用本次 target visibility/bbox review 作为独立开发标签，检查已有 risk/evidence 字段是否在：

- 无人物、箱子可见；
- 人物遮挡箱子；
- 箱子不可见；

三类条件下具有稳定且可解释的差异。任何阈值必须先在 development 数据上提出，再用未参与选择和预标注修订的独立连续帧/序列验证。

在此之前：

```text
G1-F 不放行
G1-D 不放行
```

## 10. 2026-07-29 时序复核修正

后续生成五帧 RGB-only 时间窗口时，发现 nonobstructing frame
246/478/537 的 v1 框分别落在背景板或空地面。已只修正这三行并在新目录
重算覆盖审计：

```text
semantic_box_coverage_review_v3_bbox_temporal_correction/
```

修正来源逐行记录为
`agent_rgb_only_coarse_bbox_v2_temporal_correction`；其余 45 行保持原来源。
旧目录保留为历史证据。所有粗框仍是非 GT development proxy。完整影响见：

```text
results/g2_4f_2026-07-29/
G2_4D_TO_F0_BBOX_TEMPORAL_CORRECTION_RESULT.md
```
