# S1 稠密光流残差：论文、作者源码与本项目改造边界

日期：2026-08-04  
状态：本地论文、PaperNotes 与作者源码核对完成；尚未实现 DT-SLAM 原生后端

## 1. 本阶段在总路线中的位置

本阶段仍属于 **S1 shadow**。目标是得到连续的像素级运动不一致证据，而不是
直接改变 Tracking：

```text
author Brox behavior reference
→ raw residual replay
→ DT-SLAM native residual backend
→ region-constrained dynamic decision / temporal state
→ S1 通过后才考虑 S2
```

本轮不写 `mvbDynamic`，不清除 `mvpMapPoints`，不修改 Optimizer、MapPoint、
LocalMapping 或 YOLO。

## 2. `[L]` 论文明确给出的算法

本地原论文：

`Qi 等 - 2025 - Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering and Optical Flow Residuals.pdf`

论文 §III-C 定义当前帧像素 `u_t` 的参考帧对应位置：

```text
u'  = u_t - v(u_t)
u'' = H_(t→t-1) u_t / ξ(H_(t→t-1) u_t)
δ(u_t) = ||u'' - u'||
```

其中 `v` 是稠密光流，`H` 是当前帧到参考帧的单应性。静态像素的残差应趋近
零，独立运动会增加残差。论文还明确给出：

- 用上一帧 dynamic/static/invalid 状态对对应点排序，再以 PROSAC 求 `H`；
- dynamic prior 是采样权重，不是当前帧的硬动态标签；
- 使用双阈值：`δ > τ_high` 为高残差，`δ < τ_low` 为静态，中间为待定；
- 论文文字与 Fig. 6 给出 `τ_high = 1.3 τ_low`；
- 动态传播限制在重聚类区域内部；
- 仅当区域包含高残差 seed 才执行 residual-aware fill；
- fill 超过区域面积一半时整区动态，否则仅保留 fill 区域；
- Brox 参数为 `(0.197, 50.0, 0.8, 10, 77, 10)`；
- 光流 residual 在独立线程计算。

论文的 `H` 是全局二维运动近似。它不是 RGB-D/SE(3) ego-flow；第一版不能
在复现 residual 行为的同时替换运动模型。

## 3. `[C]` 公开作者源码的实际行为

核对文件：

- `/data/dynaslam/SInDSLAM_cuda/ORB_SLAM2/src/DynaDetect.cc`；
- `/data/dynaslam/SInDSLAM_cuda/ORB_SLAM2/include/DynaDetect.h`。

`DetectDynaByDenseOpticalFLow()` 的实际路径为：

1. 灰度图缩放到 `0.6`；
2. 优先计算 current 与 `t-2`；
3. 根据 flow magnitude histogram 判断大运动，必要时改用 `t-1`；
4. CUDA 构建使用 Brox，CPU 构建使用 DeepFlow；
5. 源码将 flow 乘 `-1`，再做 `VariationalRefinement`；
6. resize 回 640×480，并将向量乘 `1/0.6`，恢复全分辨率像素单位；
7. 每隔 10 像素采样，随机种子固定为 `12345`；
8. 根据上一帧 mask/region 形成样本排序权重；实际源码在 invalid、static、
   dynamic 三类权重上都叠加 `rng.gaussian(0.5)`，并非论文公式逐字实现；
9. 实际调用 `findHomography(..., cv::RHO)`；
10. 计算 `observed flow - homography-induced flow`；
11. residual magnitude 每帧按最大值归一化到 0–255；
12. 源码同时计算 Otsu 与 Triangle 阈值，再作 1.7/3/10 px clamp；
13. 源码高阈值下限使用 `max(3 px, 1.2 × low)`，与论文的 1.3 不同。
14. 后续源码还会把上一帧 high-residual mask 并入当前 low-residual evidence，
    再在 region 内 flood fill；这不等价于仅传播上一帧 final mask。

方向约定必须冻结为：

```text
f_obs(u) = u_current - u_reference
u_reference = u_current - f_obs(u)
f_H(u) = u_current - H(u_current)
f_residual = f_obs - f_H
```

作者调用 `calc(current,past)` 后取反，使内部 `v=current-past`，因此
`past=current-v`；`H` 同样是 current→past。这里的 `cv::RHO` 是作者代码
事实；PaperNotes 中笼统写成 RANSAC 的位置不能
替代源码事实。`RHO` 与论文所述 PROSAC 关系应记为“作者的 OpenCV 实现
选择”，不声称已逐项证明其内部行为等价于论文数学描述。

## 4. 已知源码风险

- `maxFlow == 0` 或 `maxError == 0` 时存在归一化除零风险；
- `findHomography` 输入不足或失败后，源码直接访问 `H`，缺少显式失败状态；
- 一个阈值分支原代码曾对标量 `thred2` 调 `countNonZero`，审计版本通过
  `SIND_SLAM_FIX_THRESHOLD_MASK_COUNT` 修复；
- `t-1/t-2` 切换会改变 residual 的时间间隔，输出必须逐帧记录；
- 论文按连续 `t/t-1` 描述；源码先 `t-2`、大运动回退 `t-1` 是 `[C]`
  工程行为，不能反写成论文公式；
- previous mask 会形成时序反馈，首帧和 reset 必须显式处理；
- 8-bit normalized residual 丢失真实像素尺度，不能作为唯一保存量。

这些防御性检查属于 `[S]` 成熟工程修正，不应包装成论文方法创新。

## 5. `[A]` DT-SLAM 将采用的最小改造

第一步不是直接写 CPU DeepFlow，而是从独立 CUDA SIn 审计版本导出原始
Brox 参考，然后在 DT-SLAM 中 replay：

```text
独立 SIn：Brox + author homography/residual
             ↓ raw evidence files
DT-SLAM：方向/单位/threshold/replay 一致性审计
             ↓
DT-SLAM native CPU DeepFlow（只和独立 CPU DeepFlow成对比较）
```

原因：当前只保存了 author final mask/labels，无法检查 flow 方向、像素单位、
`H`、归一化或阈值。若直接加入 CPU DeepFlow，会同时引入接口差异和算法后端
差异，失败时无法定位原因。

系统 OpenCV 4.5.4 已提供 CPU `opencv_optflow`，但没有 CUDA Brox。独立 SIn
使用隔离的 CUDA OpenCV。两套 OpenCV 不应链接进同一个 DT-SLAM 进程，否则
存在 ABI、符号和运行库冲突。

## 6. 结论

稠密光流 residual 有直接论文依据，但完整 SIn 效果还依赖区域约束、时序先验
和动态判决。raw residual 只是 S1 的下一层证据，不等于 dynamic mask。

下一步冻结并执行 `S1_DENSE_FLOW_RESIDUAL_REFERENCE_EXPORT_AND_REPLAY_SPEC.md`。
