# G2-2R 共视参考选择 Shadow 结果

日期：2026-07-28

## 1. 结论

G2-2R 已完成实现、单元测试和三个 TUM 短序列的在线 CUDA 消融。

本阶段证明了：

1. ORB-SLAM2 原生参考关键帧和共视图可以驱动几何参考选择；
2. 固定 `K=1/2/5` 的缺失参考被正确保留为“未计算”，没有回退到最近帧，
   也没有把缺失参考当成静态票；
3. 稠密 warp 成本随参考数近似线性增长；
4. 单独改变参考选择策略不能使同步语义＋几何达到 30 FPS；
5. 共视关系代表共享 MapPoint，不保证参考帧时间新鲜，也不保证固定 K 始终可用；
6. 当前 person proxy 结果仍不足以批准任何动态阈值或实际 SLAM 过滤。

因此：

```text
G2-2R = 完成的 shadow reference-selection ablation
      ≠ 已完成的轻量实时几何方法
      ≠ 可进入 G1 的动态过滤器
```

## 2. 实现边界

新增的共视策略为：

```text
mCurrentFrame.mpReferenceKF
→ 该参考关键帧按 ORB-SLAM2 共视权重排序的邻居
→ 与现有 20 个成功关键帧深度缓存按 KeyFrame::mnFrameId 求交
→ 取前 K 个
```

该策略是把 DynaSLAM 的“选择高重叠历史参考”意图适配到 ORB-SLAM2 原生
共视结构，不是 DynaSLAM reference-selection reproduction。

保持不变：

- 全分辨率稠密 depth forward warp；
- z-buffer；
- `0.10 m` 诊断残差阈值；
- positive、negative、consistent、unknown 分离；
- 语义动态像素在进入参考深度缓存前置为无效；
- C++ 仅输出原始证据计数，不生成动态 mask；
- 不修改 YOLO、Optimizer、g2o 或后端；
- 不新增 PoseOptimization；
- 不修改任何 tracking 或 mapping 状态。

## 3. 验证协议

序列均取前 199 帧有效 RGB-D 输入：

| 角色 | 序列 |
| --- | --- |
| 动态 person proxy | TUM `fr3_walking_xyz` |
| 低动态诊断 | TUM `fr3_sitting_static` |
| 静态负样本 | TUM `fr1_xyz` |

在线实验条件：

- RTX 4060 Ti；
- ONNX Runtime `CUDAExecutionProvider`；
- YOLOv8n-seg 同步语义；
- viewer 关闭；
- `K=1/2/5`；
- 三组实验均为 `mask ready=199/199`；
- 三组实验均为 `mask age median/max=0/0`。

`person proxy` 不是运动真值：人物像素可能静止，人物外也可能存在动态观测或
语义漏检。因此所有 precision/recall 只用于 shadow 比较，不能当作论文级
motion segmentation 指标。

## 4. 参考可用率和时间跨度

每个序列有 198 个可尝试选择参考的当前帧。

| 序列 | K | 选满并计算 | 计算率 | 所有已选参考 age 中位数 | age p95 | age 最大值 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| walking | 1 | 198/198 | 100.0% | 2 | 6 | 13 |
| walking | 2 | 189/198 | 95.5% | 3 | 11 | 17 |
| walking | 5 | 185/198 | 93.4% | 7 | 29 | 58 |
| sitting | 1 | 198/198 | 100.0% | 15 | 53 | 96 |
| sitting | 2 | 195/198 | 98.5% | 26 | 112 | 132 |
| sitting | 5 | 186/198 | 93.9% | 56.5 | 143 | 198 |
| fr1/xyz | 1 | 198/198 | 100.0% | 5 | 114 | 168 |
| fr1/xyz | 2 | 191/198 | 96.5% | 10 | 103 | 139 |
| fr1/xyz | 5 | 183/198 | 92.4% | 30 | 143 | 170 |

事实解释：

- `K=1` 总能取得当前参考关键帧，但该参考关键帧可能长期不更新；
- `K=2/5` 在共视图尚未建立足够邻居，或邻居不在 20 项深度缓存时不能选满；
- sitting 和 fr1/xyz 中出现超过 100 帧的参考，说明高共视不等价于时间邻近；
- 这些帧不能直接解释为时序投票，参考 age 必须作为独立变量。

## 5. 运行时间

### 5.1 几何证据本身

单位为 ms/frame，只统计实际选满 K 并计算证据的帧。

| 序列 | K=1 mean | K=2 mean | K=5 mean |
| --- | ---: | ---: | ---: |
| walking | 3.388 | 6.081 | 13.914 |
| sitting | 3.341 | 6.114 | 14.222 |
| fr1/xyz | 4.337 | 7.965 | 18.658 |

### 5.2 在线同步语义＋几何端到端

| 序列 | K | active total mean (ms) | actual FPS | deadline missed |
| --- | ---: | ---: | ---: | ---: |
| walking | 1 | 37.964 | 25.497 | 182/199 |
| walking | 2 | 40.247 | 24.173 | 186/199 |
| walking | 5 | 46.432 | 21.135 | 185/199 |
| sitting | 1 | 37.675 | 25.881 | 189/199 |
| sitting | 2 | 40.331 | 24.275 | 188/199 |
| sitting | 5 | 46.851 | 21.016 | 188/199 |
| fr1/xyz | 1 | 39.502 | 24.781 | 192/199 |
| fr1/xyz | 2 | 42.955 | 22.875 | 190/199 |
| fr1/xyz | 5 | 51.694 | 19.063 | 187/199 |

结论：

- 减少 K 明确降低了稠密 warp 成本；
- 即使 `K=1`，当前同步在线闭环仍只有约 `24.8–25.9 FPS`；
- `K=2` 约 `22.9–24.3 FPS`；
- `K=5` 约 `19.1–21.1 FPS`；
- 因此不能把“减少参考帧数”写成已经满足 30 FPS。

### 5.3 recent K=5 与 covisibility K=5

| 序列 | recent G2 mean | covis G2 mean | recent FPS | covis FPS |
| --- | ---: | ---: | ---: | ---: |
| walking | 14.018 | 13.914 | 20.704 | 21.135 |
| sitting | 14.111 | 14.222 | 20.785 | 21.016 |
| fr1/xyz | 18.803 | 18.658 | 19.016 | 19.063 |

同样执行 5 次稠密 warp 时，共视选择没有产生稳定、实质性的速度改善。表中的小
差异还混有调度波动和可计算帧集合差异，不能归因于选择策略本身。

## 6. Shadow 证据代理结果

为减少 K 的可计算帧差异带来的选择偏差，下面只使用三个 K 都成功计算的公共帧：

```text
walking: 184 frames
sitting: 186 frames
fr1/xyz: 183 frames
```

这里只列出三个有明确比较意义的规则：

- K=1：`C>=1, P>=1`；
- K=2：两参考一致正残差 `C>=2, P>=2`；
- K=5：多数正残差 `C>=3, P>=3`。

| K/规则 | walking proxy precision | walking conditional recall | walking unconditional capture | fr1 static background rate | sitting proxy外 rate |
| --- | ---: | ---: | ---: | ---: | ---: |
| K1 C1/P1 | 6.10% | 22.97% | 1.22% | 2.35% | 5.30% |
| K2 C2/P2 | 7.64% | 21.86% | 0.72% | 1.70% | 2.20% |
| K5 C3/P3 | 23.16% | 55.53% | 5.04% | 2.71% | 4.12% |

观察：

- K=2 的全一致规则降低静态和低动态选择率，但 unconditional capture 也降到
  `0.72%`；
- K=5 多数票在 person proxy 上更集中，但 precision 仍只有 `23.16%`；
- K=5 多数票仍选择 `2.71%` 的 fr1 静态背景和 `4.12%` 的 sitting proxy 外
  区域；
- 以上结果不能批准任何 C/P 阈值，更不能证明被选像素都是真实运动。

## 7. 验收判断

| 项目 | 结果 |
| --- | --- |
| 共视候选与深度缓存求交 | 通过 |
| 固定 K 选不满时保留 unknown/no-computation | 通过 |
| K=1/2/5 可复现实验 | 通过 |
| 参考 age、weight、coverage、per-reference runtime 记录 | 通过 |
| shadow-only、不影响 SLAM | 通过 |
| 共视策略稳定优于 recent | 未证明 |
| 同步语义＋几何达到 30 FPS | 未通过 |
| person proxy 可支持实际动态过滤 | 未通过 |

## 8. 冻结结论与下一步建议

G2-2R 应冻结为参考策略消融，不再继续调整共视权重阈值。当前暴露的两个独立问题
是：

```text
计算问题：全分辨率 warp 随 K 近似线性增长；
建模问题：共视重叠不约束 reference age，固定 K 也不总可用。
```

建议下一步先提交本报告供复核，再决定是否进入 G2-2S。若批准 G2-2S，应只研究
有文献依据的轻量采样，并保持：

- 同一参考选择结果；
- 同一残差定义；
- full-density 结果作为测量对照；
- unknown 与 static 分离；
- 只做 shadow，不进入 tracking/mapping 过滤。

不能因为 K=1 最快就直接把它选为最终方法，也不能因为 K=5 person proxy 指标
较高就忽略其计算代价、旧参考和静态误选。

## 9. 产物

- `G2_2R_COVISIBILITY_REFERENCE_SHADOW_SPEC.md`
- `walking_online_k{1,2,5}_{selection,histogram}.csv`
- `sitting_online_k{1,2,5}_{selection,histogram}.csv`
- `fr1_xyz_online_k{1,2,5}_{selection,histogram}.csv`
- 对应的 `*.log`
- `audit_online_k{1,2,5}/g2_1_vote_grid.{csv,json}`
