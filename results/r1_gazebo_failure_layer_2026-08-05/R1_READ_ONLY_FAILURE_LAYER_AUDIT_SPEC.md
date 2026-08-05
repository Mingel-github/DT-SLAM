# R1 Gazebo失败层只读审计规范

日期：2026-08-05
基线：R0 commit `6c29ff8`
性质：只读诊断；不改变 detector、Tracking、Mapping 或阈值

## 1. 目标

在同一次冻结的 SIn 风格运行中，对齐保存：

```text
observed dense flow
→ homography ego compensation
→ residual
→ initial / gradient / RAG regions
→ per-region classifier reason
→ temporal prior contribution
→ final dynamic mask
```

R1只回答失败发生在哪一层，不在本阶段实施修复。

## 2. 冻结行为

以下内容必须与R0完全相同：

- 600帧association；
- `deepflow_cpu`；
- current-to-reference flow方向；
- homography估计、阈值生成和low/high residual mask；
- coarse-to-fine K-means、gradient split与RAG merge；
- region classifier全部参数；
- previous-frame prior；
- S2 feature filter、fail-open和S3 depth filter。

R1只能：

- clone或序列化已经计算出的矩阵；
- 记录已有条件的中间计数；
- 给现有classifier分支附加“为什么通过/拒绝”的字符串；
- 离线读取这些输出。

## 3. C++逐帧导出

通过显式环境变量启用：

```text
DT_SLAM_SIN_R1_AUDIT_DIR=/data/dynaslam/large_results/...
```

未设置时，不执行任何R1写盘操作。

每个有flow证据的帧保存：

- observed flow：Middlebury `.flo`，`CV_32FC2`；
- homography：CSV中的3×3矩阵，方向为current-to-reference；
- residual magnitude：`uint16 PNG`，量化比例64 units/pixel；
- normalized residual：`uint8 PNG`；
- low/high residual masks：二值PNG；
- initial、gradient和RAG labels：`uint16 PNG`；
- current high、previous high、temporal-added support；
- classifier core、final dynamic、raw three-state mask。

标签PNG编码：

```text
original -1 invalid → PNG 0
original  0 boundary → PNG 1
original >0 region   → PNG label+1
```

原始`CV_32S`标签仍只用于当前内存计算；编码只服务离线读取。

## 4. 逐区域判决审计

每个RAG区域记录：

- region label与区域面积；
- current high-residual pixel count；
- 是否通过`minimumClusterHighPixels`；
- high-residual contour总数；
- 通过面积/圆度条件的contour数；
- 找到low-support seed的contour数；
- flood-filled pixel数及占region比例；
- 输出为whole、partial或none；
- 最终拒绝/接受原因。

原因枚举：

```text
insufficient_high_pixels
no_high_contours
contour_geometry_rejected
no_low_support_seed
no_filled_support
accepted_partial_region
accepted_whole_region
```

这些字符串只解释原有分支，不能形成新判决。

## 5. 自动Gazebo参考

离线脚本使用：

- 当前RGB-D association；
- 相机`groundtruth.txt`；
- 箱子`box_groundtruth.txt`；
- 已知0.6 m立方体；
- 相机内参和当前深度。

把当前深度点变换到箱子坐标系，位于箱体体积内的可见深度像素记为：

> simulator-derived visible-box reference

该参考不是人工逐像素标注，也不无条件称为pixel-perfect GT。箱子外区域称为
`non-box`，除非已经排除其他运动体，不能一律称为静态背景。

## 6. 离线指标

按箱子投影面积、深度和图像运动分组，至少计算：

- box/non-box observed flow magnitude分布；
- box/non-box homography residual分布；
- low/high residual覆盖；
- RAG region purity与fragmentation；
- box主区域的classifier拒绝原因；
- temporal prior新增像素及错误持续；
- final mask箱子覆盖和non-box覆盖。

所有分布同时报告有效像素覆盖，不能只在成功测量像素上报告条件结果。

## 7. Counterfactual边界

R1允许的oracle仅用于离线诊断：

- actual region vs visible-box oracle region；
- actual classifier vs oracle accept box region；
- 后续R2才允许计算GT/SLAM SE(3) residual。

oracle输出不进入SLAM，也不能被描述为可部署算法。

## 8. 不变量

同一输入下，启用和关闭R1导出时必须满足：

- detector aggregate CSV中所有非计时、非writer字段一致；
- final geometry mask逐像素一致；
- ORB动态候选与实际删除数一致；
- trajectory完整；
- `dynamic_decision`和SLAM mutation路径没有新增来源。

写盘会降低运行速度，R1运行耗时不用于FPS结论。

## 9. 阶段停止规则

- flow在箱子区域无可分证据：停止向下调classifier；
- visible-box oracle region仍无法用当前residual形成动态证据：停止优化RAG；
- classifier不是箱子区域的直接拒绝原因：不做尺度阈值；
- temporal prior不是错误持续来源：不修改时序传播；
- R1完成前不实施SE(3)、plane edge、S4、OctoMap或新融合。

## 10. 交付物

- C++只读导出接口；
- classifier逐区域审计字段；
- runner显式R1参数；
- 合成/回归测试；
- 600帧中间量索引；
- 自动箱子参考与分层CSV/JSON；
- contact sheet；
- R1结果报告与唯一后续变量建议。
