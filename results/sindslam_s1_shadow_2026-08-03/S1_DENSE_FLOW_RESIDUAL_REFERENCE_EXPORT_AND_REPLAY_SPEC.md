# S1 原始 Brox residual 导出与 replay 规格

日期：2026-08-04  
状态：SPEC 冻结；实现前置

## 1. 目的

建立作者 CUDA Brox residual 的可检查参考，使后续 DT-SLAM 原生实现能够分别
回答：

1. 接口、方向、尺度和时序是否正确；
2. Brox 与 DeepFlow 后端差异有多大；
3. residual 到动态区域的判决差异来自哪里。

本阶段保持 shadow-only，不输出可用于 Tracking 的动态决定。

## 2. 导出范围

先在 TUM3 walking 的同一 30 帧上导出；首帧/历史不足帧也必须产生 metadata，
并标记 `available=false`，不能隐式当静态。

每帧最少保存：

| 数据 | 类型/单位 | 要求 |
| --- | --- | --- |
| observed flow | `CV_32FC2`，全分辨率 px | current→reference 方向 |
| homography | `CV_64F 3×3` | current→reference |
| induced flow | `CV_32FC2`，px | 可导出或由 `H` 确定性重算 |
| residual flow | `CV_32FC2`，px | `observed-induced` |
| residual magnitude | `CV_32F`，px | 不归一化的物理像素量 |
| normalized residual | `CV_8U` | 仅供复核作者阈值 |
| low/high masks | `CV_8U` | residual threshold evidence，不是 final mask |
| metadata | CSV/JSON | 见下表 |

metadata 至少包含：

```text
input_index, frame_id, available, failure_reason
reference_offset(1/2), large_motion
flow_backend, image_scale, full_resolution_flow_units
homography_sample_count, homography_valid
max_flow_px, max_residual_px
otsu_threshold_u8, triangle_threshold_u8
low_threshold_u8, high_threshold_u8
low_threshold_px, high_threshold_px
low_pixels, high_pixels
flow_ms, refinement_ms, homography_ms, residual_ms, threshold_ms, total_ms
temporal_prior_enabled, threshold_count_fix_enabled
```

文件建议使用每帧 OpenCV `FileStorage` 压缩文件保存浮点矩阵，并用 PNG 保存
8-bit 诊断图；若实际压缩性能不可接受，可改为标准 `.flo` 加 JSON，但必须由
round-trip test 决定，不能只保存可视化颜色图。

## 3. 独立 SIn 导出约束

- 仅在审计环境变量明确开启时写文件；默认作者运行行为不变；
- 输出根目录必须由环境变量提供，不使用作者硬编码路径；
- 不改变 residual、阈值或 final mask 的计算顺序；
- 记录 `USECUDA` 和 OpenCV 版本，日志明确 `Brox_CUDA`；
- 对空 H、点数不足、非有限值和零最大值只标 unavailable；
- 防御性分支不得静默生成全零“静态”图；
- 导出应放在 `/data/dynaslam/large_results`，避免占用系统盘；
- 作者代码改动只用于本地审计，不直接复制进入 DT-SLAM。

## 4. DT-SLAM replay 接口

建议新增独立类，避免继续扩大现有历史实验类：

```text
include/SInStyleDenseFlowResidualEstimator.h
src/SInStyleDenseFlowResidualEstimator.cc
```

最小结果对象需要区分：

```cpp
bool enabled;
bool available;
std::string failureReason;
int referenceOffset;
bool largeMotion;
cv::Mat observedFlow;
cv::Mat homography;
cv::Mat inducedFlow;
cv::Mat residualFlow;
cv::Mat residualMagnitudePx;
cv::Mat normalizedResidual;
cv::Mat lowResidualMask;
cv::Mat highResidualMask;
// thresholds, support counts, runtime
```

replay 第一版只读取并验证，不做 region dynamic decision，不使用 previous mask
重新求 H，也不修改 Frame/Tracking 状态。

## 5. replay 验收

- 30 帧 metadata 行数完整，历史不足帧有明确 unavailable 原因；
- 所有矩阵类型、尺寸、有限性和方向 invariant 通过；
- `residual = observed - induced` 浮点重算误差在数值容差内；
- `magnitude = norm(residual)` 在数值容差内；
- `H` 重算 induced flow 与导出值一致；
- normalized residual 与 `magnitude/max` 一致（考虑 8-bit 量化）；
- low/high mask 与记录阈值一致；
- high mask 是 low-evidence mask 的子集（按实际编码语义审计）；
- 重复 replay 逐像素确定；
- 日志保持 `dynamic_decision=none`、`actual_slam_removed=0`、
  `direct_slam_state_mutation=none`；
- 开关关闭和旧 reference-only 配置均无回归。

作者输出是行为参考，不是动态物体真值；这里不以 mask IoU 或 ATE 作为验收。

## 6. CPU DeepFlow 后续边界

raw replay 通过后，DT-SLAM 可实现系统 OpenCV 的 CPU DeepFlow backend。
它只能与独立 SIn 的 CPU DeepFlow 在同帧、同配置下成对比较；不能要求其与
CUDA Brox 像素等价。CPU 结果通过接口审计后，才决定是否值得将整个 DT-SLAM
统一到 CUDA OpenCV，或将 Brox 保留为外部 worker。

## 7. 停止条件

- 若作者 raw 导出改变 final mask/轨迹，则先修导出，不进入 replay；
- 若方向、缩放或 `t-1/t-2` 无法确定，不实现 native backend；
- 若 FileStorage 无法可靠 round-trip 或空间异常，先更换容器格式；
- 不因本阶段耗时高而删去 residual/区域/时序中的任一层；性能优化在行为成立后
  单独处理。

