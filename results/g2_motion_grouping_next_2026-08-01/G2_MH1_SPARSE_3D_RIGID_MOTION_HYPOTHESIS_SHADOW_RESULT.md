# G2-MH1 稀疏三维刚体运动假设 Shadow 结果

日期：2026-08-01
状态：实现和首轮开发审计完成；未通过聚类放行，保持 shadow-only

## 1. 本轮完成内容

按照冻结 SPEC，实现了默认关闭的相邻帧局部三维刚体假设：

```text
已有 F1/F3 质量合格 RGB-D 对应
→ 每个 anchor 加当前图像最近 6 点
→ SVD/Kabsch 拟合 7 点 SE(3)
→ 同一组点分别计算局部模型与背景相机模型误差
→ 输出连续拟合量、退化原因、局部尺度和耗时
```

方法身份保持为：

- `[L]` Lee et al. 2019 的局部 7 点刚体运动假设原型；
- `[A]` grid scene flow 改为现有 ORB/LK RGB-D 稀疏对应；
- `[S]` 确定性最近邻、Kabsch 闭式解、背景模型成对误差与 CSV；
- `[H]` 局部模型相对背景模型的优势是否能够区分未知运动目标，由本轮实验检查。

没有实现 Lee 原文后续的 refinement、DBSCAN、segment matching 或 dual-mode
temporal model。

## 2. 工程边界

配置开关：

```yaml
Geometry.RigidHypothesisShadowEnable: 1
```

它要求已有 `SparseEgoFlowShadowEnable` 和 `LocalRigidityShadowEnable`。默认配置不
开启。运行时输出始终为：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```

本轮没有：

```text
写 mvbDynamic
清除 mvpMapPoints
阻止 MapPoint 创建
新增 PoseOptimization
修改 Optimizer、g2o、YOLO、LocalMapping 或 LoopClosing
```

## 3. 验证

### 3.1 构建与合成测试

```text
make geometric_warp_test rgbd_tum -j$(nproc)     PASS
geometric_warp_test                               PASS
git diff --check                                  PASS
```

合成测试覆盖：纯静态刚体、两个刚体、混合边界邻域、共线退化、无效/排除状态和
重复运行确定性。

### 3.2 TUM fr1/xyz 真静态 30 帧 smoke

```text
输入图像                         29
G2-MH1 计算帧                    28
hypothesis rows                 28187
measured rows                   21333 (75.68%)
CSV / SE(3) invariant violation 0
G2-MH1 total median / p95       21.44 / 25.50 ms
完整 pipeline actual FPS        20.09
```

真静态序列上的 measured hypothesis 中位数：

| 量 | 中位数 |
|---|---:|
| local fit median | 0.00200 m |
| background fit median | 0.00543 m |
| background/local RMS ratio | 1.959 |
| relative translation | 0.179 m |
| relative rotation | 0.207 rad |

这里的相对平移/旋转不是真实物体运动。它说明 7 点自由刚体模型能够吸收局部深度、
LK 和位姿误差，在静态数据上也显著优于固定背景模型。因而“局部拟合更好”不能
直接解释成动态。

### 3.3 Bonn moving_nonobstructing_box 开发审计

运行完整 778 帧，但只保存既有 24 个 review 帧的逐 hypothesis 行：

```text
hypothesis rows                 24099
measured rows                   20634
frame timing rows               777
CSV / SE(3) invariant violation 0
G2-MH1 total median / p95       29.53 / 33.69 ms
完整 pipeline actual FPS        18.81
```

本次 Codex 进程的 CUDA 初始化报 `CUDA failure 100: no CUDA-capable device is
detected`，失败发生在首帧之前，没有产生在线语义实验。正式开发审计因此使用
geometry-only，`semantic_mode=none`。现有 24 张精确 person mask 不是整段逐帧
mask，不能伪装成完整在线语义运行。

24 个粗箱框和 5 moving / 19 stationary RGB 时序标签只用于事后 proxy 排序，
不是目标或运动真值。箱框内逐帧中位数的描述结果：

| 连续量 | moving 中位 | stationary 中位 | raw proxy AUC | 箱内减箱外 proxy AUC |
|---|---:|---:|---:|---:|
| background fit median | 0.01442 m | 0.00446 m | 0.979 | 0.642 |
| local fit median | 0.00632 m | 0.00382 m | 0.768 | 0.505 |
| median improvement | 0.00369 m | 0.00099 m | 0.768 | 0.505 |
| background/local RMS ratio | 1.466 | 1.314 | 0.800 | 0.663 |
| relative translation | 0.547 m | 0.328 m | 0.621 | 0.411 |
| relative rotation | 0.432 rad | 0.201 rad | 0.695 | 0.547 |

`raw proxy AUC` 不能解释为检测精度。尤其 5 个 moving review 帧均有人搬运或遮挡，
而本次没有在线 semantic；箱内减箱外是较保守的同帧对照，但仍不是 GT。

## 4. 结果解释

正面事实：

- 当前稀疏 RGB-D 输入普遍足够形成 7 点非退化假设；
- Kabsch、SE(3)、状态和 CSV 链路正确；
- moving proxy 中背景模型误差明显较大，连续运动不一致仍然存在；
- 所有输出保持 shadow-only，没有误入 SLAM 过滤。

负面或限制：

- 7 点局部模型在真静态数据上同样容易比背景模型拟合得好，存在明显模型自由度
  过拟合；
- 做同帧箱内/箱外对照后，局部模型 improvement 与 local fit 的 proxy AUC 均约
  0.505，未显示稳定新增区分力；
- 最强的 raw 排序仍是 background fit，本质上接近已有 F1/背景运动不一致，而
  不是已经形成共同刚体对象；
- 当前逐 anchor 全邻居排序为二次复杂度，单模块约 21--30 ms，将完整 pipeline
  降至约 19--20 FPS；
- 无在线 semantic 的 Bonn 对照受人物运动混杂，不能给出对象特异性结论。

因此当前不能声称：

```text
已经检测到未知动态箱子
局部刚体模型已优于 F1
可以选择 hypothesis 阈值
可以进入 DBSCAN / temporal confirmation
可以放开 G1-F 或 G1-D
```

## 5. 决策

G2-MH1 作为可复现的 shadow 原型保留并默认关闭。按照冻结 SPEC，本轮证据不足以
直接进入 hypothesis clustering：先停止继续实现 DBSCAN 或调 7 点阈值。

本地 Lee 2019 原文再次核对后确认：其 7 点只用于产生初始 hypothesis；随后会
对全部 `n` 个 grid scene-flow vector 计算刚体变换误差，找出更大支持集，用增加
后的 `N` 点重新估计 refined hypothesis，最后才对 hypothesis 做 DBSCAN。也就是
说，当前 G2-MH1 只实现了论文的 seed-model 层；原论文并不把 7 点训练内拟合误差
直接当成动态判决。

这解释了当前静态过拟合，但不自动证明 refinement 在稀疏 ORB/LK 输入上可行。
下一步应先冻结一份独立的 support/refinement shadow SPEC，明确候选支持域、独立
验证方式和原论文 `t_inlier` 不能直接迁移的事实。若不能形成忠实且轻量的适配，
应把 G2-MH1 记录为有限/负面路线，重新比较 SInDSLAM 区域重聚类或更完整的稠密
运动候选，而不是在当前标量上补经验阈值。

## 6. 产物

源码和配置：

- `DT-SLAM/include/GeometricDynamicDetector.h`
- `DT-SLAM/src/GeometricDynamicDetector.cc`
- `DT-SLAM/include/Tracking.h`
- `DT-SLAM/src/Tracking.cc`
- `DT-SLAM/Examples/RGB-D/geometric_warp_test.cc`
- `DT-SLAM/Examples/RGB-D/BONN_GeometryRigidHypothesisShadow.yaml`
- `DT-SLAM/Examples/RGB-D/TUM1_GeometryRigidHypothesisShadow.yaml`
- `DT-SLAM/tools/audit_sparse_rigid_hypotheses.py`
- `DT-SLAM/tools/audit_sparse_rigid_hypothesis_proxy.py`

原始本地证据：

- `results/g2_mh1_2026-08-01/fr1_xyz_30_*`
- `results/g2_mh1_2026-08-01/nonobstructing_24_hypotheses_v2.csv*`
- `results/g2_mh1_2026-08-01/nonobstructing_24_audit.json`
- `results/g2_mh1_2026-08-01/nonobstructing_24_proxy_audit.json`
- `results/g2_mh1_2026-08-01/nonobstructing_geometry_only_v2.log`
