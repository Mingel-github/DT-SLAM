# S1 稠密光流 residual 的 DT-SLAM shadow replay 结果

日期：2026-08-04  
状态：独立严格 replay 与 DT-SLAM 30 帧系统 replay 均通过；仍无动态判决

## 1. 本轮完成内容

新增了只读 `reference_replay` 接口：

```text
include/SInStyleDenseFlowResidualEstimator.h
src/SInStyleDenseFlowResidualEstimator.cc
```

它读取独立 SInDSLAM CUDA/Brox 审计版本导出的 flow、homography、residual、
阈值和 mask，重新计算并核对方向与数值。随后它被挂入现有 S1 shadow 统计
路径，但不接收或修改 `Frame`、`mvpMapPoints`、`mvbDynamic`、Optimizer 或
MapPoint。

配置文件：

```text
Examples/RGB-D/TUM3_SInStyleDenseFlowResidualReplayShadow.yaml
```

审计工具：

```text
tools/audit_sin_style_dense_flow_tracking_replay.py
```

## 2. 接口语义修正

回放接口明确区分：

- `available=true`：当前帧存在并通过验证的 residual evidence；
- `available=false`：历史不足、参考缺失或非严格模式下参考损坏；
- `dynamicStateAvailable=false`：无论 evidence 是否存在，本阶段都没有动态
  判决。

frame 0 返回 `history_unavailable`，不会生成全零“静态”证据。Tracking 使用
非严格模式，使缺失参考保持 unknown；独立测试使用严格模式 fail-fast。

同时增加 replay 身份核对：

```text
internal_sign = -1
flow_units = full_resolution_pixels
homography_direction = current_to_reference
homography_valid = 1
```

并核对最大 observed flow、最大 residual，以及 normalized 阈值与物理像素
阈值的换算。

## 3. v2 作者参考导出

旧 29 帧浮点证据本身有效，但 metadata 未显式保存上述全部身份字段。隔离的
本地作者审计副本因此新增 3 个 metadata 字段：

```text
local audit commit 4db149a  Record raw flow identity metadata
```

相同 TUM3 walking 30 输入帧重新导出到新目录，未覆盖旧证据：

```text
/data/dynaslam/large_results/sindslam_s1_raw_brox_tum3_walking_30_v2
```

第 0 帧仍由作者 runner 初始化 detector，没有调用 dense detector，因此磁盘
上只有 frame 1--29。DT-SLAM replay 对 frame 0 合成明确的
`history_unavailable`；这是相对冻结 SPEC 的已记录偏差，不把它包装成完全满足
“30 个 metadata 文件”。

## 4. 独立严格 replay

测试目标：

```text
sin_style_dense_flow_replay_test
```

结果：

```text
29/29 evidence frames passed
large_motion = 4
max residual recompute error = 0 px
max normalized quantization error = 1
dynamic_decision = none
direct_slam_state_mutation = none
```

测试还覆盖：默认关闭、frame 0 history unavailable、非严格缺失参考返回
unknown，以及相同输入重复 replay 的逐像素确定性。

## 5. DT-SLAM 30 帧系统 replay

系统同时读取：

- 独立 SIn final state/labels；
- v2 raw Brox residual evidence。

输出：

```text
dense_flow_replay_30.csv
dense_flow_replay_30.log
dense_flow_replay_30_audit.json
```

审计结果：

| 项目 | 结果 |
| --- | ---: |
| DT-SLAM 输入行 | 30 |
| 作者 export 行 | 29 |
| 同帧成功匹配 | 29 |
| frame 0 | `history_unavailable` |
| `dynamicStateAvailable` | 全部 0 |
| `actual_slam_removed` | 全部 0 |
| replay 与 manifest 不变量错误 | 0 |

逐帧 reference lag、large-motion 标志、样本数、阈值、low/high support 和物理
量均与作者 manifest 一致；比较容差只补偿 manifest 默认约六位有效数字的文本
截断。

## 6. 回归与性能解释

旧 `TUM3_SInStyleReferenceShadow.yaml` 不包含 dense-flow 配置，运行时：

```text
dense_flow_enabled = 0
dense_flow_available = 0
dense_flow_failure_reason = disabled
actual_slam_removed = 0
```

SIn 像素级 reference/labels 统计与旧 10 帧结果逐帧一致。ORB 点数在重复运行间
出现约 ±1、mask-hit 约数点的原生波动，因此没有错误宣称 ORB 提取逐点确定。

本次系统 replay 的 `actual_fps≈26.18` 包含 `.flo/.png/YAML` 磁盘读取和全图
逐像素重算验证，只是审计链成本，不能代表原生 Brox 或未来 native backend
速度。

## 7. 当前结论与下一步

本轮证明了 Brox residual 在 DT-SLAM 中的方向、单位、时间参考和阈值接口可以
无歧义复现；它没有证明 residual 已能判断动态对象。

下一步仍按冻结顺序实现/核对 native CPU DeepFlow，并只与独立 SIn CPU
DeepFlow作同后端比较。之后才把连续 residual 放入 region-constrained dynamic
decision 和上一帧状态。S2 继续锁定。

