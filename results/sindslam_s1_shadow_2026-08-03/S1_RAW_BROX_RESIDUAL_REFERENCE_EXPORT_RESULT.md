# S1 原始 Brox residual 参考导出结果

日期：2026-08-04  
状态：30 输入帧/29 组 raw evidence 导出与数值审计通过；DT-SLAM replay 已在后续完成

## 1. 实现范围

本轮没有修改 DT-SLAM 检测或 SLAM 状态。为了不污染已有的独立 SIn 复现
工作树，在隔离的本地审计副本中增加了受环境变量控制的诊断导出：

```text
/home/zhu/dynaslam_ws/.local_sindslam_raw_export
local commit de684e8  Add raw Brox residual audit export
local commit 4db149a  Record raw flow identity metadata
```

该目录被主仓库 `.gitignore` 排除。大体积数据保存在：

```text
/data/dynaslam/large_results/sindslam_s1_raw_brox_tum3_walking_30
/data/dynaslam/large_results/sindslam_s1_raw_brox_tum3_walking_30_v2
```

审计修改只在作者 residual 和阈值计算完成后克隆/写出数据；环境变量未设置时
不写任何证据文件。

## 2. 输出

TUM3 walking 前 30 个输入帧中，第 0 帧只初始化 detector，因此输出 frame
1--29 共 29 组：

- 0.6 尺度、符号转换后的选中 Brox flow；
- full-resolution refined flow；
- full-resolution residual vector；
- normalized residual PNG；
- low/high threshold mask；
- current→reference homography 与逐帧 metadata；
- manifest CSV。

总占用约 164 MiB，与规格预估一致。

## 3. smoke 行为不变验证

在相同 3 帧输入上分别关闭/开启导出，逐像素比较 frame 1、2 的：

- `mask_pre_runner_dilate`；
- `mask_final`；
- `labels`。

六组比较全部 `different_pixels=0`。这证明当前短序列上诊断写出没有改变
作者最终 mask/region 行为。同步磁盘写入仍会污染时延，因此导出运行本身不用于
性能结论。

## 4. 29 帧数值 invariant

`audit_sin_style_dense_flow_export.py` 对全部 29 帧验证：

- `.flo` magic、形状和 `CV_32FC2` 布局正确；
- raw Brox 为 384×288，refined/residual 为 640×480；
- `H` 有限且映射 current→reference；
- `residual = refined_flow - (current - H(current))`；
- residual 重算最大绝对误差约 `1.38e-6 px` 以内；
- magnitude/最大 residual metadata 一致；
- 8-bit normalized residual 重算误差不超过 1；
- low/high masks 与阈值一致；
- high mask 始终为 low mask 子集；
- reference index/lag 一致；
- invariant errors 为 0。

审计结论为 `pass=true`。它只证明序列化、方向、尺度和阈值算术正确，不证明
动态对象检测精度。

## 5. 描述性统计

| 指标 | 结果 |
| --- | ---: |
| `large_motion`，改用 t-1 | 4/29 帧 |
| max observed flow，中位 | 14.84 px |
| max residual，中位 | 12.69 px |
| low threshold，中位 | 2.04 px |
| high threshold，中位 | 3.63 px |
| low-mask pixels，中位 | 51,017 |
| high-mask pixels，中位 | 29,567 |
| flow + refinement，中位 | 43.47 ms |
| residual detector total，中位 | 50.93 ms |

这里的时延来自开启同步导出的审计运行，不能作为无导出性能值。mask pixels
也是 residual evidence 面积，不是最终 dynamic mask 面积。

## 6. 重要观察

- 作者源码在 29 帧中实际使用了两种时间间隔，说明 replay 必须逐帧读取
  reference index，不能默认全是 t-1；
- 部分阈值是非整数 normalized 值，因为源码先按物理像素 clamp，再回到
  0--255 域；replay 不能只读取 PNG 后猜阈值；
- 物理 high threshold 多数在 3--5 px，但最大达到 9.71 px；不能用一个固定
  3 px 阈值概括作者行为；
- raw Brox、refined flow 和 residual 是三种不同数据，不得混称。

## 7. 后续状态

DT-SLAM raw residual replay 已完成，详见：

```text
S1_DENSE_FLOW_RESIDUAL_TRACKING_REPLAY_RESULT.md
```

v2 导出额外明确记录 flow 单位、homography 方向和有效性；replay 仍只输出连续
residual/low/high evidence，不产生 final dynamic mask，不进入 S2。下一步是
native CPU DeepFlow 与独立 CPU SIn 的同后端比较。

## 8. 证据

- `S1_DENSE_FLOW_RESIDUAL_LITERATURE_SOURCE_AUDIT.md`；
- `S1_DENSE_FLOW_RESIDUAL_REFERENCE_EXPORT_AND_REPLAY_SPEC.md`；
- `raw_brox_export_30.log`；
- `raw_brox_export_30_audit.json`；
- `raw_brox_export_smoke_audit.json`；
- `raw_export_behavior_off/` 与 `raw_export_behavior_on/`；
- `/data/dynaslam/large_results/sindslam_s1_raw_brox_tum3_walking_30/manifest.csv`。
