# G2-4F2B 参考深度边界风险 Shadow 结果

日期：2026-07-30  
状态：诊断完成；边界主解释被否决；不形成过滤规则

## 1. 实现与合同

按冻结 SPEC，在 G2-4F1 每条 feature CSV 中新增：

```text
reference_depth_boundary_d1
reference_depth_boundary_d2
reference_invalid_depth_d1
reference_invalid_depth_d2
```

边界定义采用 SInDSLAM 原文的：

\[
|D(n)-D(u)|>\max(0.025D(u),0.08\text{ m}).
\]

1/2 pixel 风险带是项目诊断适配 `[A/S]`。它只用于分层，不删除 feature。

确定性测试覆盖：

- 有效中心点附近的 depth step；
- 两像素外 invalid depth；
- invalid depth 不被重解释为 depth boundary；
- 原有 LK/depth/projection no-evidence 合同。

结果：

```text
geometric_warp_test = PASS
dynamic_decision = none
direct_slam_state_mutation = none
```

## 2. 同域短测

两个真正静态、相机运动序列各运行 150 帧：

| 序列 | GT 插值 | F1 compute median/p95 | actual FPS | deadline miss |
| --- | ---: | ---: | ---: | ---: |
| TUM fr1/xyz | 150/150，20 ms bracket | 2.745/3.158 ms | 28.742 | 0/150 |
| Bonn static_close_far | 149/150，40 ms bracket | 2.682/2.978 ms | 29.734 | 0/150 |

FPS 包含逐 feature 重诊断 CSV，不能替代 production FPS。Bonn 的 40 ms
只用于 GT 插值；SLAM、LK 和 evidence 分支不使用 GT。

## 3. SLAM-pose 静态候选分层

以下使用冻结诊断点：

```text
FB <= 0.25 px
q >= 10
```

| 数据域 | 全部候选率 | boundary_d2 承载候选 | invalid_d2 承载候选 | clean_d2 承载候选 | clean_d2 内候选率 |
| --- | ---: | ---: | ---: | ---: | ---: |
| TUM fr1/xyz | 0.232% | 1.4% | 2.1% | 96.8% | 0.283% |
| Bonn static_close_far | 0.546% | 1.7% | 23.7% | 75.2% | 0.577% |

风险带可重叠，因此各列不能相加为 100%。

结论：

- Bonn 无效深度邻域确实承载了高于 TUM 的候选份额；
- 但 Bonn 仍有 75.2% 候选位于 `clean_d2`；
- depth boundary 本身只承载 1.7%；
- 因此“深度边界/空洞是 Bonn 跨域升高的主要原因”不成立。

本阶段不引入 boundary veto。这样做既没有充分解释静态尾部，也可能删除未知
动态物体边缘上真正有价值的 motion evidence。

## 4. GT-pose 诊断

同一 feature correspondence、同一深度和同一风险带，只替换 ego-flow 的
相对相机位姿：

| 数据域 | GT q=10 全部候选率 | GT clean_d2 候选率 |
| --- | ---: | ---: |
| TUM fr1/xyz | 0.069% | 0.087% |
| Bonn static_close_far | 0.032% | 0.028% |

同时，GT raw residual median 反而较大：

```text
TUM:  SLAM 0.378 px, GT 1.108 px
Bonn: SLAM 0.301 px, GT 1.799 px
```

因此可支持的有限判断是：

> GT-pose 分支的归一化 residual tail 更小、跨域更接近；当前 SLAM-pose
> 误差与高 q 静态候选存在明显关联。

不能据此声称：

- GT raw projection 比 SLAM 更准确；
- 所有静态候选都由位姿误差造成；
- Bonn GT frame transform、时间同步和标定没有剩余误差。

GT 仍只作诊断，部署使用禁止。

## 5. 方法来源账本

| 组件 | 来源 | 当前使用 | 性质 |
| --- | --- | --- | --- |
| moving-border depth patch 风险 | DynaSLAM, RA-L 2018 | 说明边缘误判需要单独审计，不搬其 41×41/variance 常数 | `[L/A]` |
| relative+absolute depth edge | SInDSLAM, T-ITS 2025 | 构造参考深度 boundary risk | `[L/A]` |
| 1/2 pixel 风险带 | 本项目 G2-4A | 与既有风险统计保持一致 | `[S]` |
| 风险带只分层、不 veto | 本阶段保守设计 | 避免把诊断相关性写成判决 | `[S]` |
| SLAM/GT 两分支比较 | 本项目诊断 | 定位 pose sensitivity，不部署 GT | `[S/H]` |

## 6. 决策

```text
depth-boundary instrumentation = PASS
depth boundary as main explanation = REJECTED
boundary veto/filter = REJECTED
F2D operating point = evaluated separately
G1-F / G1-D = locked
strict holdout = still sealed
```

## 7. 原始证据

```text
results/g2_4f2_2026-07-30/static_fr1_xyz_150_risk/
results/g2_4f2_2026-07-30/static_bonn_close_far_150_risk/
results/g2_4f2_2026-07-30/cross_domain_static_depth_risk_curves.json
```

