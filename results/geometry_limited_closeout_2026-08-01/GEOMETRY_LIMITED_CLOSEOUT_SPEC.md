# 当前轻量几何路线收尾评价 SPEC

日期：2026-08-01
状态：评价前冻结；不是新增算法阶段

## 1. 目的

本轮不再寻找新的动态判决，不启动 SInDSLAM，只对当前已经实现的轻量稀疏几何
路线作一次有限收尾：

1. 确认当前代码仍能构建、测试并保持几何默认关闭；
2. 汇总已有正式 ATE/RPE/FPS、轨迹覆盖和过滤执行证据；
3. 明确当前版本能用于什么、不能声称什么；
4. 为以后单独研究开源 SInDSLAM 保留清楚的接续边界。

## 2. 冻结对象

本轮收尾对象仅为已经真实接入的轻量稀疏方法：

```text
F1 sparse LK observed flow - RGB-D/SE(3) camera-induced flow
→ FB/LK/depth quality checks
→ frozen robust normalized q10 candidate
→ G1-F1 post-SearchLocalPoints association removal
→ G1-M1 new MapPoint admission veto
→ 5% maximum removal and fail-open safeguards
```

身份为：

> `[L/A/S]` FlowFusion 式 ego-flow residual 的稀疏 ORB/LK 适配，加上本项目的
> ORB-SLAM2 前端安全接入。

它不是 FlowFusion 复现，也不是已验证的未知对象检测器。

## 3. 复用的正式证据

不为了收尾重复耗时完整运行。直接复用参数冻结后得到的：

- TUM fr3/walking_xyz 四模式 ATE/RPE/FPS 与三轮 geometry-only 波动；
- Bonn moving_nonobstructing_box 三轮 semantic-only / semantic+geometry；
- Bonn moving_obstructing_box 的逐特征证据漏斗；
- TUM/Bonn mapping admission 和 tracking removal invariant；
- 后续运动分组、区域传播和刚体假设负实验，用于解释为什么不扩大当前 claim。

## 4. 本轮新增检查

只新增当前工作树回归：

1. `geometric_warp_test`；
2. `rgbd_tum` 完整构建；
3. TUM fr1/xyz 30 帧默认关闭 smoke；
4. 同 30 帧 frozen q10 geometry-only smoke；
5. 检查新完成的 rigid-hypothesis shadow 默认关闭，且两种 smoke 均不启用它；
6. 检查 tracking/mapping CSV 不变量和最大删除限制。

30 帧 smoke 只检查代码回归，不计算或宣称 ATE 改善。

## 5. 收尾判定口径

### 可以保留为实验模式

- 构建和测试通过；
- semantic+geometry 在正式序列中完整运行；
- 几何路径确实删除 association、否决新 MapPoint；
- 安全限制与 fail-open 正常；
- 关闭几何时保持标准语义/ORB 入口；
- 额外耗时没有造成明显系统级崩溃。

### 不允许的结论

- 不声称 geometry-only 是可靠 SLAM；
- 不声称 ATE/RPE 稳定改善；
- 不声称已经检测或分割未知动态对象；
- 不开放 G1-D 动态深度区域过滤；
- 不默认开启当前 q10 过滤；
- 不把 shadow 负实验纳入运行时产品路径。

## 6. 输出

最终 RESULT 必须给出：

```text
工程状态
TUM/Bonn 定量汇总
几何真实贡献与局限
推荐默认运行模式
后续 SInDSLAM 研究接续点
当前仍未实现的功能
```
