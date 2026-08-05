# R2 相机运动补偿单变量 Shadow 对照规范

日期：2026-08-05
前置结果：R1已完成
性质：离线/只读；不改变detector、Tracking、Mapping或阈值

## 1. 为什么进入R2

R1已经排除“优先修改区域”的依据：在6 m以内的82个中远距离漏检帧中，RAG
区域对箱体覆盖中位95.9%、主区域纯度中位100%，但箱体high residual覆盖为0。
联系表还显示地面、顶面和墙面存在大块残差。

因此R2只检查：当前全局homography相机运动补偿是否在具有明显三维视差的走廊中
制造背景残差，及RGB-D/SE(3)补偿能否改善残差可分性。

## 2. 文献与方法身份

- `[L]` SInDSLAM：稠密光流、current-to-reference homography和区域内残差；
- `[L]` FlowFusion：observed flow减camera-induced ego-flow的动态证据；
- `[L]` Plane＋Parallax：单一平面homography与非平面视差不能混为一体；
- `[A/H]` 在同一SIn稠密flow上替换为RGB-D/SE(3)逐像素补偿；是否更好必须由本实验决定。

R2不是FlowFusion复现，也不是把已有稀疏LK模块直接搬入SIn detector。

## 3. 冻结输入

复用R1的：

- 600帧association；
- 599帧CPU DeepFlow；
- 每帧实际reference index；
- 当前homography；
- 当前RGB-D、区域标签和箱体参考；
- S1全部阈值参数。

R2不重新运行flow，不重新聚类，不影响SLAM。

## 4. 对照模型

### 4.1 当前PROSAC homography

严格重算现有induced flow并与R1量化前残差核对方向和数值。

### 4.2 Oracle-static homography

仅用于诊断homography模型能力。先使用Gazebo参考SE(3)与observed flow选择高度符合
静态模型的像素，再在相同current-to-reference方向拟合一个homography。

该选择使用参考信息，不能上线，也不能被描述为实际算法。

### 4.3 Gazebo参考位姿RGB-D/SE(3)

它是逐像素深度感知补偿的几何上限，用来判断SE(3)模型本身是否有价值。

### 4.4 ORB-SLAM2事后轨迹RGB-D/SE(3)

使用冻结R1运行输出的最终相机轨迹。它只回答“若位姿已经得到，SE(3)残差如何”，
不能作为当前前置检测器的在线输入。

### 4.5 历史常速度预测RGB-D/SE(3)

只使用当前帧之前的两个ORB-SLAM2最终位姿预测当前位姿，模拟可在`Track()`前获得
的历史运动模型。它是在线候选，但仍是离线replay，不直接接入detector。

明确禁止为获得当帧位姿而新增一次Tracking或第三次PoseOptimization。

## 5. 方向与公式

SIn当前flow保存的是current-minus-reference displacement：

```math
u_r^{flow}=u_t-v_{obs}(u_t).
```

对当前深度点：

```math
X_t=\pi^{-1}(u_t,D_t(u_t)),
```

```math
\hat u_r^{SE3}=\pi(T_{r\leftarrow t}X_t),
```

```math
v_{ego}^{SE3}=u_t-\hat u_r^{SE3},
```

```math
\delta_{SE3}=\|v_{obs}-v_{ego}^{SE3}\|.
```

所有模型必须保持这一方向。

## 6. 必须先通过的不变量

- 当前homography离线重算残差与R1输出一致；
- identity pose产生零ego-flow；
- 合成纯平移和纯旋转方向正确；
- `Twc/Tcw`相对变换方向明确；
- current/reference时间戳与R1 reference index一致；
- common-valid像素单独报告；
- invalid depth与投影越界保持unknown，不解释为static。

## 7. 第一阶段指标

R2第一阶段只评价residual，不把任何新mask送入classifier：

- 全部像素coverage与common-valid coverage；
- visible-box、local-ring和non-box residual median/p90；
- 每个模型使用相同阈值生成算法后的low/high覆盖；
- 固定3 px下的覆盖，便于解释当前high下限；
- residual随箱体深度、面积和参考间隔的变化；
- 当前homography与oracle homography差距；
- 参考SE(3)、事后SLAM SE(3)与历史预测SE(3)差距。

`non-box`包含运动行人，不直接称为静态背景。对oracle-static采样及视觉确认另行记录
其解释限制。

## 8. 决策规则

| 结果 | 后续结论 |
| --- | --- |
| oracle homography仍差，参考SE(3)明显更好 | homography模型能力不足，SE(3)候选成立 |
| oracle homography好，当前homography差 | 估计/采样问题优先，不替换模型 |
| 参考SE(3)好、事后SLAM SE(3)差 | SLAM位姿精度限制 |
| 事后SLAM SE(3)好、历史预测SE(3)差 | 前置在线预测不足，不能直接上线 |
| 参考SE(3)也不能改善 | 停止SE(3)路线，回查flow/同步/遮挡 |
| 背景改善但远距箱体仍无残差 | 只解决背景误检，不宣称解决远小箱子 |

只有residual层通过后，才另写SPEC决定是否replay原classifier；本阶段不直接进入ATE。

## 9. 交付物

- 离线R2工具；
- 合成方向测试；
- 逐帧、逐模型CSV；
- 汇总JSON和联系表；
- 结果报告；
- 唯一后续建议。
