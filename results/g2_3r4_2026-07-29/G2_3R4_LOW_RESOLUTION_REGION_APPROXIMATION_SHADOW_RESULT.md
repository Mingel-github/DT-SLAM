# G2-3R4 低分辨率区域近似 Shadow 结果

日期：2026-07-29

## 1. 结论

```text
最小实现与确定性测试         = 通过
native half-cell 计数边界    = 通过
candidate online region 门   = 通过
isolated net-saving 门        = 通过
end-to-end improvement 门     = 失败（2/3 序列失败）
结构一致性 paired audit       = 未执行
G2-3R4 保留决策               = 不保留；默认关闭
dynamic decision              = 未实施
G1-F / G1-D                   = 继续锁定
```

`[S]` G2-3R4 是一次预先有停止条件的有界工程实验。端到端收益是必要门；
该门失败后，不再为这条候选表示扩展结构审计代码或继续逐毫秒优化。

这不证明低分辨率 partition 结构错误；它只说明当前实现没有在三个冻结序列上
稳定达到预设的端到端收益门。

## 2. 代码基线与边界

实验前 GitHub 快照：

```text
commit = 41ca519 Document geometry region evidence shadow stages
origin/main = 41ca519
```

本轮实现：

- `[S]` 复用 G2-3R3 已生成的 native scale-2 depth；
- `[S]` 在 320×240 上运行现有 discontinuity partition；
- `[S]` 保留 native comparison/positive/negative/consistent count；
- `[S]` region aggregation 每个 native cell 只计一次；
- `[S]` semantic proxy 使用 2×2 block-any 投影，仅作诊断；
- `[S]` label/boundary 最近邻映射到 640×480，只计 mapping cost；
- `[S]` 新开关默认关闭，只有专用配置显式开启。

未修改：

```text
mvbDynamic
mvpMapPoints filtering
YOLO
Optimizer/g2o
PoseOptimization 次数
LocalMapping
LoopClosing
```

日志继续报告：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```

## 3. 测试

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
make geometric_warp_test rgbd_tum -j$(nproc)
```

结果：两个目标构建成功。既有 ONNX Runtime C++17 和 Eigen deprecated 警告
仍存在，没有新增编译错误。

确定性测试：

```text
[Geometry ... /G2-3R4 Test] PASS
```

新增断言覆盖：

- pyramid native depth/count domain 为 scale 2；
- native comparison/consistent vote conservation；
- 4×4 full image 对应 2×2 native cells；
- region aggregation 统计 4 个 native cells，而不是 16 个 expanded pixels；
- semantic proxy 的 block-any 投影。

30 帧 smoke test 还确认：

```text
domain_scale=2
dense audit=disabled
dynamic_decision=none
direct_slam_state_mutation=none
```

## 4. Candidate-only 实验

三个序列均使用：

```text
199 frames
ONNX CUDAExecutionProvider device 0
viewer disabled
dense/full-reference audit disabled
scale = 2
tau_rel = 0.025
tau_abs = 0.08 m
```

原始日志：

- `walking_200_candidate_only.log`
- `sitting_200_candidate_only.log`
- `fr1_xyz_200_candidate_only.log`

逐 region CSV：

- `walking_200_candidate_region.csv`
- `sitting_200_candidate_region.csv`
- `fr1_xyz_200_candidate_region.csv`

## 5. Candidate region 路径

逐 region CSV 按 frame 去重后：

| 序列 | computed frames | partition mean/median/P95 | aggregation mean/median/P95 | mapping mean/median/P95 | online total mean/median/P95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 181 | 0.806 / 0.811 / 0.892 ms | 0.173 / 0.175 / 0.197 ms | 0.252 / 0.191 / 0.488 ms | 1.231 / 1.209 / 1.512 ms |
| sitting | 186 | 0.886 / 0.876 / 0.941 ms | 0.184 / 0.180 / 0.243 ms | 0.249 / 0.223 / 0.364 ms | 1.320 / 1.292 / 1.594 ms |
| fr1/xyz | 181 | 0.909 / 0.890 / 0.929 ms | 0.168 / 0.163 / 0.180 ms | 0.237 / 0.185 / 0.561 ms | 1.315 / 1.249 / 1.692 ms |

因此三个序列分别满足：

```text
online region mean <= 1.5 ms
online region P95  <= 2.0 ms
```

相对 G2-3R3 的 full partition + full aggregation 均值，隔离路径估算节省：

| 序列 | G2-3R3 full region path | G2-3R4 candidate path | estimated saving |
| --- | ---: | ---: | ---: |
| walking | 3.518 ms | 1.231 ms | 2.287 ms |
| sitting | 3.720 ms | 1.320 ms | 2.401 ms |
| fr1/xyz | 3.875 ms | 1.315 ms | 2.560 ms |

`[S]` 隔离净节省门通过，但它不能替代端到端结果。

## 6. 端到端结果

| 序列 | G2-3R3 active mean | G2-3R4 active mean/median/P95 | mean improvement | deadline miss | actual FPS |
| --- | ---: | ---: | ---: | ---: | ---: |
| walking | 38.596 ms | 36.996 / 37.955 / 39.411 ms | 1.600 ms | 168/199 | 25.904 |
| sitting | 38.997 ms | 38.711 / 38.878 / 41.474 ms | 0.287 ms | 182/199 | 25.166 |
| fr1/xyz | 39.992 ms | 38.589 / 38.904 / 42.753 ms | 1.403 ms | 179/199 | 25.171 |

预冻结要求三个序列分别满足：

```text
end-to-end active mean improvement >= 1.5 ms
```

结果：

```text
walking = 通过
sitting = 失败
fr1/xyz = 失败
overall = 失败
```

`[H]` 隔离节省没有稳定转化为 active-total 节省，可能与其他同步路径抖动、
tracking 状态差异或调度有关；本实验不能证明具体原因，也不以该假设为由追加优化。

## 7. 未执行的结构审计

没有生成 full-vs-half paired structural audit，因此没有报告：

- assignment retention；
- permutation-invariant IoU；
- merge/fragmentation mass；
- boundary F1；
- ORB co-membership；
- evidence-weighted merge/fragmentation。

这些指标均保持“未知”，不能写成通过或失败。full-resolution partition 仍只是
高分辨率参考实现，不是真值。

停止原因是必要收益门已失败；继续实现完整 paired audit 不会改变“不保留该
优化候选”的工程决策。

## 8. 科学边界与下一步

- `[L/A]` 文献只支持 boundary-preserving pyramid 组件；
- `[S/H]` scale-2 region replacement 仍是本项目工程假设；
- 没有动态分类，因此没有 measured dynamic FPR；
- TUM walking 不是未知动态物体证明；
- 没有 geometry ATE 结论；
- shadow-only 继续保持。

下一步返回核心科学门：

```text
冻结现有可测 evidence representation
→ 验证动态/静态区分能力与风险代理
→ 未知动态物体验证
→ 独立数据冻结参数
```

不继续维护 G2-3R4 性能路线。
