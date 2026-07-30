# G2-4E 连续 Development Geometry Evidence 成对审计规范

日期：2026-07-29

## 1. 目标

G2-4E 只回答：

> 当前 C++ geometry evidence 在独立 RGB-only target-box 粗框及 visibility
> 条件下是否呈现稳定、可解释且不完全由人物、深度边界或无效深度驱动的差异？

它不选择动态阈值，不产生 dynamic decision，不改变 SLAM。

```text
dynamic_decision = none
direct_slam_state_mutation = none
G1-F = locked
G1-D = locked
```

## 2. 数据身份

Development：

- `rgbd_bonn_moving_nonobstructing_box`
- `rgbd_bonn_moving_obstructing_box`

两者已经被 geometry proxy 条件化选择并查看，不能报告无偏 sequence-level
precision/recall。

Strict hold-out：

- `rgbd_bonn_balloon_tracking.zip`
- 身份和 SHA-256 见 `STRICT_HOLDOUT_MANIFEST.md`
- G2-4E 不解压、不查看、不运行。

## 3. 为什么必须运行完整连续序列

多参考 geometry 依赖：

- 前序成功 tracking；
- 关键帧历史；
- 共视参考；
- 同步 person mask 对参考深度的清理。

因此不能只把 24 个离散候选帧拼成一个短序列运行。两条 development 序列必须：

```text
原始时间顺序
+ 完整一对一 association
+ online exact C++ person mask
+ Bonn joint rectification
+ 当前 frozen G2-3R3/G2-4A shadow configuration
```

运行后只从完整输出中连接 48 个已冻结候选帧。

## 4. C++ 运行配置

使用：

- `BONN_GeometryPyramidEvidenceShadow.yaml`
- `Geometry.MultiReferenceSamplingPolicy=pyramid_dense`
- `Geometry.MultiReferencePyramidScale=2`
- full-resolution depth-region partition；
- G2-4A risk diagnostics；
- online CUDA YOLO；
- viewer disabled。

必须分别输出：

- full run log；
- multi-reference histogram CSV；
- region evidence CSV；
- CameraTrajectory；
- KeyFrameTrajectory；
- 运行环境与 association 哈希。

`DT_SLAM_GEOMETRY_MULTIREF_CSV` 与
`DT_SLAM_GEOMETRY_REGION_EVIDENCE_CSV` 是两个不同输出，不得混用。

## 5. Box-region 连接

target-box 预标注只提供粗 bbox/visibility，不是 pixel mask。连接步骤：

1. 从相同 association 读取 candidate 的 raw RGB-D；
2. 使用同一个 `RGBDInputRectifier`；
3. 使用同一个 `DepthMapFactor` 转成米；
4. 调用同一个 `PartitionDepthByDiscontinuity()`；
5. 统计 bbox 内每个 region label 的交集；
6. 按 `(frame, sampling_policy, region_label)` 连接在线 region CSV；
7. 用在线 CSV 中重复记录的 partition stats 检查离线重建是否完全一致。

不得按 geometry score 选择 bbox region。所有与粗 bbox 相交的 region 都保留，
并记录：

- bbox intersection pixels；
- bbox coverage；
- region coverage；
- boundary/invalid bbox pixels；
- online evidence/risk 字段。

若 partition stats 或 label 无法匹配，必须报告 mismatch，不得近似连接。

## 6. 审计分层

至少分开：

- target visible；
- target partial；
- target occluded/person-contact；
- target absent；
- person semantic present/absent；
- boundary/invalid risk；
- single-reference 与 multi-reference support。

比较只用于 development 诊断：

- box-intersecting regions 与同帧非 box regions；
- target visible 与 absent；
- person-present 与 person-absent；
- 去除 boundary/invalid 高风险后的变化。

## 7. 禁止把什么写成事实

- 粗 bbox 内所有像素属于 box；
- target visible 等于 target moving；
- region 与 bbox 相交就属于动态对象；
- AUC 或阈值是无偏泛化结果；
- strict hold-out 已通过；
- geometry 已改善 ATE。

## 8. 决策门

只有同时看到以下迹象，才值得另写 dynamic-decision SPEC：

- person-absent box frames 仍存在稳定 evidence；
- evidence 不只集中于 boundary/invalid band；
- target-absent 条件下静态风险明显更低；
- 单参考偶然异常能被 multi-reference support 区分；
- 多个分层/序列方向一致。

若方向不一致或区分很弱：

- 保留负结果；
- 不调一个阈值强行进入 G1；
- 回到 evidence 定义或寻找有文献依据的辅助测量。
