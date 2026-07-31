# G1-F1 稀疏 Ego-flow Tracking 过滤结果

日期：2026-07-31
状态：实验性 tracking 过滤已实现并完成首轮正式评价；默认关闭
依据：`G1_F1_SPARSE_EGO_FLOW_TRACKING_FILTER_SPEC.md`

## 1. 本阶段回答的问题

本阶段不再要求几何证据达到近乎零假阳性，而是验证一个更符合工程目标的问题：

> 在严格限制删除比例、关联下限和重定位窗口的条件下，少量删除高连续运动残差
> 的 ORB/MapPoint 关联，是否会破坏静态 SLAM，是否可能改善
> semantic+geometry 的动态序列定位？

结论：

- 静态 `fr1/xyz` 没有出现明显退化；
- `fr3/walking_xyz` 的三轮 semantic+geometry 均完整跟踪；
- 三轮中位数上，q6/q8/q10 的 ATE 和 RPE 都优于同成本 semantic control；
- 实际 FPS 中位数代价约 `0.45%–0.52%`；
- 因此 G1-F1 可以保留为**实验性、默认关闭的 tracking-only 路线**；
- 结果尚不能证明未知箱子检测，也不能解锁 mapping/depth 过滤。

## 2. 方法与文献身份

当前证据为：

```text
ORB 位置上的双向 PyrLK observed flow
- RGB-D 深度与初始 SE(3) 位姿预测的 ego flow
→ sparse ego-flow residual
```

- `[L/A]`：受 FlowFusion 自运动补偿 flow residual 思想支持；
- `[A]`：由 dense optical flow 改为 ORB 位置上的稀疏 PyrLK；
- `[S]`：MAD 式帧内尺度、q6/q8/q10、安全保护和 ORB-SLAM2 接入；
- `[S]`：fail-open 条件是工程安全策略，不是论文创新。

这不是 FlowFusion 复现，也不是对象级 motion segmentation。

## 3. 实现边界

准确调用顺序：

```text
初始 tracking 与第一次既有 PoseOptimization
→ 计算 sparse ego-flow candidate
→ TrackLocalMap
→ SearchLocalPoints
→ 既有 semantic association removal
→ G1-F1 清除少量候选 mvpMapPoints
→ 第二次既有 PoseOptimization
```

本阶段：

- 没有新增第三次 `PoseOptimization()`；
- 没有修改 `Optimizer.cc`、g2o、LocalMapping 或 LoopClosing；
- 没有写 `mvbDynamic`；
- 没有禁止新 MapPoint 或 KeyFrame 写入；
- 没有过滤 depth 或稠密点云；
- 没有修改 YOLO；
- 默认配置仍为关闭。

## 4. Fail-open 保护

以下任一条件成立时，本帧完全不删除：

- 几何参考、坐标域或尺度无效；
- 尺度支持少于 20；
- 位于 relocalization 后保护窗口；
- baseline 或删除后 MapPoint 关联少于 30；
- 候选关联超过 baseline 的 5%；
- feature/sample/vector 尺寸不一致。

41 个运行 CSV 共 `26,928` 个 frame row，累计实际删除 `23,235` 个关联，
审计得到：

```text
invariant violations = 0
pose_reoptimization  = none
mapping_veto         = none
```

权威审计：

```text
DT-SLAM/tools/audit_sparse_flow_tracking_filter.py
results/g1_f1_2026-07-31/filter_invariant_audit.json
```

## 5. 确定性与冒烟测试

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
make geometric_warp_test rgbd_tum -j$(nproc)
```

结果：

```text
[Geometry ... /G1-F1 Test] PASS
```

80 帧 q10 无语义冒烟：

```text
78 filter rows
3 applied frames
4 removed associations
0 invariant violations
actual_fps = 28.3453
```

Viewer ON 的 149 帧 q6 定性检查：

```text
trajectory complete
deadline_missed = 0/149
actual_fps = 28.6036   # 仅定性，不作为正式计时
```

## 6. 真静态负样本：TUM fr1/xyz

四组均完整输出 792 帧。所有模式均计算相同 F1 evidence；control 只关闭删除，
因此这是过滤动作的同成本对照。

| 模式 | ATE RMSE | 相对 control | RPE RMSE | 相对 control | FPS | 实际删除 |
|---|---:|---:|---:|---:|---:|---:|
| control | 0.009707 m | — | 0.005757 m/frame | — | 29.6427 | 0 |
| q6 | 0.009851 m | +1.48% | 0.005857 m/frame | +1.74% | 29.6367 | 957 |
| q8 | 0.009687 m | -0.21% | 0.005789 m/frame | +0.56% | 29.6500 | 423 |
| q10 | 0.009829 m | +1.26% | 0.005857 m/frame | +1.74% | 29.6402 | 331 |

客观解释：

- 几何候选并非零假阳性；
- q6 在静态序列 299 帧发生删除，说明它不是“纯动态对象标签”；
- 但 ATE/RPE 变化约在 `-0.2%` 到 `+1.7%`，没有达到约 10% 的工程退化线；
- 这支持“允许少量静态关联被删，但以最终 SLAM 结果约束风险”的新决策。

## 7. Geometry-only walking：波动很大

三轮结果：

| 模式 | 完整轨迹次数 | ATE RMSE 范围 | RPE RMSE 范围 |
|---|---:|---:|---:|
| control | 1/3 | 0.599–0.933 m | 0.0267–0.0413 m/frame |
| q6 | 2/3 | 0.591–0.816 m | 0.0253–0.0281 m/frame |
| q8 | 3/3 | 0.708–0.933 m | 0.0261–0.0270 m/frame |
| q10 | 0/3 | 0.508–0.747 m | 0.0378–0.0761 m/frame |

该组只说明：

- 无语义 ORB-SLAM2 在强人物动态序列上具有显著单次波动；
- q8 在本批三轮中覆盖更稳定；
- ATE 仍然很大，geometry-only 尚不能替代语义；
- 不得用某一次 partial trajectory 的较低 ATE 宣称改善。

## 8. Semantic + geometry walking：三轮正式结果

所有 12 次运行均完整输出 827 帧，YOLO 使用
`CUDAExecutionProvider (device 0)`。

### 8.1 每轮原始结果

| 模式 | trial 1 ATE/RPE | trial 2 ATE/RPE | trial 3 ATE/RPE |
|---|---:|---:|---:|
| semantic control | 0.020793 / 0.012838 | 0.015598 / 0.012224 | 0.017699 / 0.011996 |
| q6 | 0.016629 / 0.011916 | 0.014710 / 0.011501 | 0.014927 / 0.011882 |
| q8 | 0.014464 / 0.011505 | 0.016116 / 0.011923 | 0.017474 / 0.012385 |
| q10 | 0.015398 / 0.012158 | 0.015462 / 0.011986 | 0.017539 / 0.011942 |

单位分别为 `m` 和 `m/frame`。

### 8.2 三轮中位数

| 模式 | ATE RMSE | 相对 control | RPE RMSE | 相对 control | actual FPS | 相对 control |
|---|---:|---:|---:|---:|---:|---:|
| semantic control | 0.017699 m | — | 0.012224 | — | 27.4836 | — |
| q6 | 0.014927 m | -15.66% | 0.011882 | -2.80% | 27.3397 | -0.52% |
| q8 | 0.016116 m | -8.94% | 0.011923 | -2.46% | 27.3607 | -0.45% |
| q10 | 0.015462 m | -12.64% | 0.011986 | -1.95% | 27.3472 | -0.50% |

三轮删除量：

| 模式 | 删除关联范围 | applied frame 范围 | 5% fail-open frame |
|---|---:|---:|---:|
| q6 | 1493–1522 | 392–399 | 固定为 5 |
| q8 | 847–1054 | 279–294 | 固定为 1 |
| q10 | 537–611 | 196–210 | 0 |

## 9. Semantic + geometry sitting_static：三轮泛化结果

`sitting_static` 表示相机运动模式接近静止，并不表示画面中完全没有人物运动。
12 次运行均完整输出 680 帧。

| 模式 | ATE RMSE 中位数 | 相对 control | RPE RMSE 中位数 | 相对 control | actual FPS 中位数 |
|---|---:|---:|---:|---:|---:|
| semantic control | 0.006775 m | — | 0.005605 | — | 27.9311 |
| q6 | 0.006633 m | -2.10% | 0.005527 | -1.39% | 27.9269 |
| q8 | 0.007034 m | +3.82% | 0.005600 | -0.09% | 27.9425 |
| q10 | 0.006547 m | -3.37% | 0.005468 | -2.44% | 27.9092 |

该序列的几何删除量很小：

```text
q6   106–158 associations / 54–63 applied frames
q8    39–44  associations / 29–30 applied frames
q10   19–28  associations / 15–19 applied frames
```

所有 ATE 中位变化都在约 `±4%` 内，没有触发约 10% 工程退化线。q8 在该序列
轻微退化，因此不能只凭 walking 的一次最佳值选 q8。

## 10. Bonn balloon：三轮跨数据集结果

使用 Bonn G2-4B 已验证的共同坐标域：

```text
RGB linear rectification
depth nearest-neighbor rectification
YOLO / ORB / geometry 共用 P=K 无畸变针孔域
```

首次运行使用旧的临时解压 association，在遇到当前目录不存在的 PNG 时按设计
终止。没有跳过错误继续评价。随后将完整 archive 解压到：

```text
/data/dynaslam/datasets/rgbd_bonn_balloon
```

并针对实际文件重新生成 438 对严格一对一 association。三轮 12 次运行均完整。

| 模式 | ATE RMSE 中位数 | 相对 control | RPE RMSE 中位数 | 相对 control | actual FPS 中位数 |
|---|---:|---:|---:|---:|---:|
| semantic control | 0.032605 m | — | 0.040905 | — | 29.5356 |
| q6 | 0.030841 m | -5.41% | 0.041780 | +2.14% | 29.4763 |
| q8 | 0.030505 m | -6.44% | 0.041176 | +0.66% | 29.5699 |
| q10 | 0.032027 m | -1.77% | 0.040974 | +0.17% | 29.4670 |

该结果是混合的：

- 三个 q 的 ATE 中位数都改善；
- RPE 中位数都轻微恶化，但最大约 2.14%；
- 所有变化均远小于约 10% 工程退化线；
- `balloon` 是语义外动态物体的 development 序列，但本阶段没有逐 feature
  气球 GT，不能把 ATE 改善等同于“几何准确检测到气球”。

## 11. 结论边界

### 可以说

- F1 sparse ego-flow residual 已第一次安全进入真实 tracking association 过滤；
- 在一个真静态序列中没有明显退化；
- 在 TUM walking 的三轮 semantic+geometry 对照中，三个预冻结 q 的 ATE/RPE
  中位数均改善；
- 在 TUM sitting_static 的三轮对照中，没有超过约 4% 的 ATE 中位变化；
- 在 Bonn balloon 三轮对照中 ATE 改善约 1.8%–6.4%，RPE 轻微恶化
  约 0.2%–2.1%；
- 过滤本身的端到端 FPS 代价很小；
- q6 更积极，q10 更保守，q8 在 geometry-only 覆盖上表现较稳定。

### 不能说

- 不能说已经可靠检测未知动态对象；
- 不能说 q6/q8/q10 已选出最终阈值；
- 不能说 pixel/depth dynamic mask 已解决；
- 不能说 MapPoint 写图污染已解决；
- 不能把 TUM person 场景改善直接外推到 Bonn 运动箱子；
- 不能依据本阶段打开 G1-D 或 mapping veto。

## 12. 当前决定

```text
G1-F1 experimental tracking filter    KEEP, default OFF
G1-F1 safety/invariant implementation PASS
static non-degradation check          PASS on fr1/xyz first run
semantic+geometry walking             POSITIVE, 3 runs
semantic+geometry sitting_static      NO MATERIAL DEGRADATION, 3 runs
semantic+geometry Bonn balloon        MIXED BUT SAFE, 3 runs
experimental working point            q10 (conservative)
unknown-box claim                     NOT ESTABLISHED
G1 mapping/depth filtering            LOCKED
```

q10 被选为后续**保守实验工作点**，不是论文最终阈值。依据不是它在某条序列
最好，而是：

- 三条 semantic 序列的 ATE 中位数均未退化；
- 删除量在三个 q 中最低；
- walking 与 sitting 没有 5% fail-open；
- Bonn RPE 中位变化只有约 `+0.17%`；
- q6/q8 继续保留为固定消融，不删除其结果。

下一步不再回到 flood fill 或单点 precision 门控。按风险顺序：

1. 冻结 q10 为 G1-F1 后续实验工作点，配置默认开关仍为关闭；
2. 设计 G1-M0 counterfactual：只统计 q10 候选在 RGB-D 初始化与
   `CreateNewKeyFrame()` 中会否创建 MapPoint；
3. 只有 G1-M0 显示实际写图作用且安全条件明确后，才实现默认关闭的 mapping
   veto；
4. G1-D 像素深度过滤仍锁定，不与稀疏 MapPoint 写入保护混为一谈。

完整逐次指标：

```text
results/g1_f1_2026-07-31/G1_F1_FORMAL_METRICS.csv
```
