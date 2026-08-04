# S1 native CPU DeepFlow evidence backend 规格

日期：2026-08-04  
状态：SPEC 已执行；属于既定 S1 子步骤，不新增阶段

## 1. 目的

在 DT-SLAM 内 clean-room 实现与独立 SIn CPU 路径可成对核对的 dense-flow
residual evidence backend。它只解决：

```text
current/reference gray
→ CPU DeepFlow
→ t-2 / large-motion t-1 选择
→ VariationalRefinement
→ current-to-reference homography
→ observed - induced residual
→ low/high residual evidence
```

它不解决区域动态判决，不输出 final mask，不形成上一帧动态状态，也不进入 S2。

## 2. 方法归属

- `[L]`：SInDSLAM 论文的 dense optical-flow residual、双阈值及区域内传播框架；
- `[C]`：公开源码的 0.6 scale、优先 t-2、大运动回退 t-1、DeepFlow/Brox
  后端切换、VariationalRefinement、RHO homography 和 1.7/3/10 px clamp；
- `[A]`：DT-SLAM 首个 native backend 使用系统 OpenCV CPU DeepFlow，不冒充
  CUDA Brox；
- `[S]`：无历史、零 flow、空 H、非有限值和不连续帧显式 unavailable/unknown，
  不生成全零静态结果。

实现根据论文、PaperNotes、源码行为审计和 raw evidence 接口重新编写；不复制
作者函数或大段源码。作者源码许可证未明确前，不把作者实现直接合入 DT-SLAM。

## 3. 第一轮关闭 temporal prior 的原因

作者默认 homography 样本排序依赖上一帧 dynamic/static/invalid mask 和 region
label。DT-SLAM 当前尚未完成 native region dynamic decision，若直接使用作者
final mask 作为 prior，会使所谓 native backend 隐含依赖外部答案。

因此第一轮成对比较固定：

```text
SIND_SLAM_DISABLE_TEMPORAL_PRIOR=1
SIND_SLAM_FIX_THRESHOLD_MASK_COUNT=1
```

独立 SIn CPU 和 DT-SLAM native CPU 使用相同的无 prior 固定采样规则。它只用于
验证 DeepFlow、时间参考、方向、homography、residual 和阈值接口。后续 S1
区域动态状态形成后，再按论文/源码加入 previous-state prior；关闭 prior 不是
最终算法主张。

第二个开关启用已审计的阈值分支修正：作者源码该分支把标量阈值传给
`countNonZero`，严格对照改为统计对应的二值阈值图。它是 `[S]` 源码缺陷修正，
不是论文算法改造；作者参照与 DT-SLAM native 实现必须同时使用该修正。

## 4. 输入与状态

```text
input: current gray CV_8UC1, input_index
state: at most previous two gray frames and their input_index
```

- frame 0：缓存图像，输出 `history_unavailable`；
- frame 1：只能使用 t-1，`actual_reference_lag=1`；
- frame >=2：先用 t-2；若作者 large-motion test 成立，改用 t-1；
- reset 或 input index 不连续：清空历史，当前帧重新成为 unavailable 起点。

## 5. 输出

沿用 `SInStyleDenseFlowResidualResult`：

- selected native DeepFlow；
- full-resolution observed flow；
- current→reference H 与 induced flow；
- residual vector/magnitude；
- normalized residual；
- low/high evidence masks；
- reference lag、large-motion、samples、阈值、support、耗时和失败原因。

固定：

```text
dynamicStateAvailable = false
dynamic_decision = none
direct_slam_state_mutation = none
```

## 6. 成对参考

使用隔离作者审计副本的 CPU build，并确保整个进程只链接系统 OpenCV 4.5.4
ABI。不能复用 CUDA build 的 DBoW2/OpenCV 库进入 CPU 进程。

相同的 TUM3 walking 30 帧、相同 association、相同 temporal-prior-off 配置分别
产生作者 raw DeepFlow 参考和 DT-SLAM native 输出。

## 7. 验收

- CPU 独立进程链接中没有 CUDA OpenCV/system OpenCV 混用；
- frame/reference index 和 large-motion 选择一致；
- flow 方向、全分辨率单位与 H 方向一致；
- DeepFlow、refined flow、H、residual、阈值和 low/high support 成对比较；
- 差异必须报告数值分布，不能用“看起来相似”代替；
- 重复 native 运行确定；
- 缺失历史和数值失败保持 unknown；
- 旧 reference-only/replay 配置无回归；
- 不写 `mvbDynamic`，不清除 `mvpMapPoints`，不修改 Optimizer/YOLO。

若同后端仍明显不一致，先检查输入时序、flow 符号、refinement 初始化、样本排序
和 OpenCV ABI；不通过调动态阈值掩盖接口错误。

## 8. 2026-08-04 执行更新

- temporal-prior-off 的 29 帧 native CPU evidence 已与作者 CPU 输出完成严格
  对照，low/high mask mismatch 为零；
- 区域判决形成后，已加入作者源码中的 previous detector-state/region-label
  homography 样本排序；
- temporal-prior-on 对照仍保持 low/high mask mismatch 为零；
- 结果见 `S1_NATIVE_CPU_REGION_DECISION_SHADOW_RESULT.md`；
- 这只完成受控 evidence/decision 行为链，native region representation 尚未完成
  最终接管，S2 继续锁定。
