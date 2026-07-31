# G1 稀疏几何前端四模式冻结 SPEC

日期：2026-07-31
状态：实现与运行前冻结
范围：只冻结实验入口与评价，不修改 q10、5% 条件或 SLAM 后端

## 1. 目的

当前已经实现 G1-F1 tracking association 过滤和 G1-M1 MapPoint admission
保护，但“机制生效”不等于“正式四模式结论成立”。本阶段使用同一个二进制、
同一数据关联和同一代码快照，固定以下四种运行模式：

| 模式 | YOLO semantic | sparse geometry | tracking filter | mapping filter |
|---|---:|---:|---:|---:|
| `orb_baseline` | 关 | 关 | 关 | 关 |
| `semantic_only` | 开 | 关 | 关 | 关 |
| `geometry_only` | 关 | 开 | 开 | 开 |
| `semantic_geometry` | 开 | 开 | 开 | 开 |

这里的 `geometry_only` 是类别无关的稀疏 ego-flow residual 过滤，不是完整对象
分割，也不输出可信的动态深度区域。

## 2. 首轮序列

首轮只使用 TUM `fr3_walking_xyz`：

- `TUM3.yaml` 与 `TUM3_GeometrySparseEgoFlowShadow.yaml` 均使用同一零畸变
  640x480 针孔域；
- 避免在首轮把 TUM1/Bonn 的 input rectification 差异混入方法对照；
- walking 已有完整 semantic、G1-F1 和 G1-M1 历史结果，可用于发现明显回归。

若首轮通过，再为 TUM1/Bonn 单独冻结“相同 rectified input、只切换几何”的
公平对照，不直接复用 native-domain baseline。

## 3. 固定参数

```text
G1-F1 residual quantile q                    10%
maximum association removal fraction          5%
minimum remaining associations                30
minimum scale support                          20
G1-M1 maximum candidate feature fraction       5%
G1-M1 maximum candidate depth fraction         5%
G1-M1 minimum remaining valid-depth features 100
```

本阶段不得依据单次 ATE 调整这些参数。

## 4. 输出

每种模式单独保存：

```text
run.log
run_manifest.json
CameraTrajectory.txt
KeyFrameTrajectory.txt
```

几何模式额外保存：

```text
tracking_filter.csv
mapping_filter.csv
```

正式评价至少报告：

- ATE RMSE；
- RPE RMSE，delta=1 frame；
- actual FPS；
- 轨迹覆盖；
- tracking filter applied/removed；
- mapping filter applied/vetoed；
- fail-open 分布。

## 5. 通过条件

首轮不是要求 geometry 必须优于所有模式，而是检查其能否作为稳定实验版本：

1. 四种模式均完整输出轨迹；
2. 几何模式的 CSV invariant 全部通过；
3. `geometry_only` 不依赖 YOLO 模型仍能实际产生几何候选；
4. `semantic_geometry` 的在线 mask age 保持 0；
5. 不出现明显 ATE/RPE、轨迹覆盖或 FPS 灾难性退化；
6. Viewer OFF 正式结果与 Viewer ON 定性检查分开。

单轮 ATE 的改善或恶化只作 smoke 观察。若需要正式比较，继续使用三轮中位数，
不能把 ORB-SLAM2 的运行波动解释为几何贡献。

## 6. 边界

```text
no q/5% retuning
no G1-D depth-mask filtering
no extra PoseOptimization
no Optimizer/g2o/YOLO/LocalMapping changes
no claim of exact dynamic-object precision/recall
```

本阶段只冻结和验证已经实现的稀疏前端，不增加新的几何检测算法。
