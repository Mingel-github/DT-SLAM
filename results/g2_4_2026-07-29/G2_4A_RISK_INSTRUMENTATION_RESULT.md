# G2-4A Boundary/Invalid/Support 风险字段接入结果

日期：2026-07-29

## 1. 结论

```text
boundary risk fields       = 已接入
invalid-depth risk fields  = 已接入
reference-support fields   = 已接入
final pose-quality field   = 未接入；调用时点不满足
instrumentation default    = disabled
dynamic score/threshold    = none
SLAM state mutation        = none
```

## 2. 字段定义

`[S]` 对每个已分配 region，使用当前 partition domain 的 Chebyshev 邻域：

```text
boundary_d1 = 距 depth-boundary <= 1 domain pixel
boundary_d2 = 距 depth-boundary <= 2 domain pixels
invalid_d1  = 距 invalid-depth <= 1 domain pixel
invalid_d2  = 距 invalid-depth <= 2 domain pixels
```

每个 band 记录：

```text
region pixels
comparison pixels
positive-presence pixels
comparison votes
positive votes
```

reference support 记录：

```text
single-reference comparison pixels
multi-reference comparison pixels
single/multi-reference positive-presence pixels
unanimous-positive pixels
```

这些是描述性风险统计，不是 motion score。

## 3. 默认关闭

配置：

```text
Geometry.RegionRiskDiagnosticsEnable: 1
```

或只用于实验的环境覆盖：

```text
DT_SLAM_GEOMETRY_REGION_RISK_DIAGNOSTICS=1
```

默认值为 `0`。关闭时不构造 boundary/invalid dilation，也不执行 support/risk
计数，旧 G2-3 配置不会自动承担 G2-4A instrumentation 成本。

## 4. Pose-quality 限制

`RunGeometryShadow()` 当前在 `TrackLocalMap()` 前调用。此时 `mnMatchesInliers`
不是当前帧最终 tracking 质量。为避免把陈旧或中间状态写成事实，本轮没有记录
“final pose quality”。

后续若需要该字段，必须单独设计：

- 在 final tracking state 确定后记录；
- 与此前生成的 geometry frame record 通过 frame id 关联；
- 不移动 geometry 计算时点；
- 不增加 PoseOptimization。

## 5. 验证

构建：

```text
geometric_warp_test = PASS
rgbd_tum             = PASS
```

确定性测试覆盖：

- 1/2 pixel boundary band；
- 1/2 pixel invalid band；
- single/multi-reference support；
- unanimous-positive；
- risk band 单调包含；
- vote conservation。

30 帧 smoke：

```text
CSV rows                         = 2508
risk fields present              = true
boundary_d1 <= boundary_d2       = PASS
invalid_d1 <= invalid_d2         = PASS
single + multi == compared pixel = PASS
dynamic_decision                 = none
direct_slam_state_mutation       = none
```

## 6. 性能身份

enabled smoke 单帧示例：

```text
region aggregation ~= 1.83 ms
```

该数值只是 instrumentation 成本样例，不是正式性能结果。G2-4A 属于离线
测量步骤，正式 tracking/FPS 报告必须关闭 instrumentation。
