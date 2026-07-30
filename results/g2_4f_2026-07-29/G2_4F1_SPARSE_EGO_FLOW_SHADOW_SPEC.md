# G2-4F1 Sparse Ego-motion-compensated Flow Shadow SPEC

日期：2026-07-29
状态：实现前冻结
范围：相邻成功 RGB-D 帧、当前 ORB feature、shadow-only。

## 1. 目标

测量稀疏 observed optical flow 相对 RGB-D/SE(3) camera ego flow 的连续
residual，判断它是否为当前 depth residual 的互补证据。

本阶段只回答：

- 当前 ORB feature 是否有可靠相邻帧 LK 对应；
- residual 在独立 RGB temporal `moving/stationary/uncertain` proxy 中是否有
  可审计差异；
- SLAM initial pose 与 GT pose 对 residual 的影响；
- CPU 增量成本是否可接受。

## 2. 非目标

- 不产生 dynamic/static feature；
- 不选择 residual、FB error 或 support 的动态阈值；
- 不与 G2 depth vote 融合；
- 不做 region growing、DBSCAN、K-means 或 optical-flow mask；
- 不替换 ORB matching；
- 不修改 `mvbDynamic`、`mvpMapPoints` 或 MapPoint 写入；
- 不修改 YOLO、Optimizer、g2o、LocalMapping 或 LoopClosing；
- 不增加 PoseOptimization；
- 不运行 strict hold-out；
- 不把 agent motion proxy 当 GT。

## 3. 调用位置

只在 RGB-D 路径启用。

计算位置：

```text
TrackWithMotionModel / TrackReferenceKeyFrame / Relocalization
→ initial pose available
→ RunGeometryShadow()
→ RunSparseEgoFlowShadow()   [新增，只读]
→ TrackLocalMap()
```

这与当前 geometry shadow 一样位于 initial pose 后、`TrackLocalMap()` 前，不
改变已有两次 PoseOptimization 路径。

缓存更新位置：

```text
GrabImageRGBD()
→ Track() 完整返回
→ 若 mState==OK，保存当前 gray/depth/final Tcw/frame/timestamp
→ 若失败，清空缓存
```

因此：

- 当前 residual 使用当前 initial `Tcw`；
- reference 使用上一成功帧 final `Tcw`；
- 不使用失败帧作为下一帧 reference；
- 不跨 tracking-loss gap 做 LK。

## 4. 最小缓存

建议独立结构：

```cpp
struct SparseFlowReference
{
    cv::Mat gray;          // CV_8UC1, common rectified domain
    cv::Mat depthMeters;   // CV_32FC1
    cv::Mat TcwFinal;      // 4x4
    cv::Mat TcwGroundTruth;// optional
    long unsigned int frameId;
    double timestamp;
    bool valid;
};
```

缓存只属于 Tracking shadow instrumentation，不放入 `Frame`，避免扩大所有
Frame copy 的内存和语义。

## 5. 输入域

必须满足：

```text
current gray
reference gray
reference depth
current ORB mvKeys
semantic mask
K used by projection
```

位于同一 pinhole pixel domain。

当前批准配置：

- TUM3 零畸变；
- Bonn G2-4B 联合 rectification，tracking distortion 为零。

若输入仍有非零 tracking distortion，G2-4F1 必须输出 domain-invalid 并跳过；
不能把 raw LK point 直接交给无畸变 pinhole 投影。

## 6. LK 方向与输出

对每个当前 `Frame::mvKeys[i].pt = u_t`：

1. `calcOpticalFlowPyrLK(currentGray, referenceGray, u_t, u_r)`；
2. `calcOpticalFlowPyrLK(referenceGray, currentGray, u_r, u_t_fb)`；
3. 输出 raw forward-backward error `||u_t_fb-u_t||`；
4. 在 `u_r` 最近邻读取 reference depth；
5. 用 initial SLAM pose 计算 `u_t_ego`；
6. 输出 `u_t-u_t_ego` 的 x/y/magnitude；
7. 若 GT pose 可用，对同一对应输出 GT residual。

第一版显式使用 OpenCV LK 默认数值设置，写入日志：

```text
winSize=(21,21)
maxLevel=3
termination=(COUNT|EPS,30,0.01)
flags=0
minEigThreshold=1e-4
```

它们是 `[S]` 固定实现设置，不是动态判决参数。本阶段不做 sweep。

## 7. 逐 Feature CSV

仅在显式环境变量设置时输出：

```text
DT_SLAM_GEOMETRY_SPARSE_FLOW_CSV
DT_SLAM_GEOMETRY_SPARSE_FLOW_FRAME_IDS
```

每行至少包含：

```text
frame
timestamp
reference_frame
reference_timestamp
dt_seconds
feature_index
u_current
v_current
octave
has_mappoint
semantic_nonzero
backward_lk_status
forward_lk_status
u_reference
v_reference
u_forward_back
v_forward_back
lk_error_backward
lk_error_forward
forward_backward_error_px
reference_depth_valid
reference_depth_m
slam_ego_projection_valid
slam_u_ego
slam_v_ego
slam_residual_x_px
slam_residual_y_px
slam_residual_magnitude_px
gt_pose_available
gt_ego_projection_valid
gt_residual_x_px
gt_residual_y_px
gt_residual_magnitude_px
evidence_state
```

`evidence_state` 只能是：

```text
measured
lk_invalid
depth_invalid
projection_invalid
domain_invalid
reference_unavailable
```

不得出现 dynamic/static。

## 8. Frame 级统计与计时

每帧记录：

- current feature count；
- backward/forward LK success；
- valid depth count；
- valid SLAM/GT residual count；
- residual magnitude 的 median/p90/p95（只描述，不分类）；
- backward LK、forward LK、depth+projection、CSV-record 和 total active ms；
- reference frame age 与 `dt_seconds`。

带逐 feature CSV 的 FPS 只作为诊断成本，不能冒充最终系统 FPS。

## 9. 确定性测试

纯几何测试：

- identity pose + zero observed displacement → residual 0；
- known camera translation + matching ego displacement → residual 0；
- injected independent pixel displacement → residual 等于注入量；
- invalid depth/behind-camera/out-of-domain → no evidence；
- SLAM pose 与 GT pose 分支相互独立。

LK 接口测试：

- 合成平移纹理图像；
- backward/forward direction 与符号正确；
- feature index 数量保持；
- LK failure 不产生 residual；
- 关闭环境变量时不记录 CSV。

测试不设 dynamic threshold。

## 10. 独立 Motion-state Proxy

在读取 G2-4F1 residual 之前冻结并生成：

```text
agent_rgb_temporal_motion_proxy_v1.csv
```

候选仍使用当前 48 个 development/review frames；每个候选通常生成
`t-2..t+2` rectified RGB clip/contact sheet。若候选位于序列边界，则使用
距离该候选最近的连续 5 帧窗口，并显式记录实际 start/end 和每帧 offset；
不得复制不存在的未来帧，也不得因此丢弃候选。

标签及要求：

```text
moving | stationary | uncertain | occluded_or_not_visible
confidence
reason
review_frame_start/end
label_source=agent_rgb_temporal_only_v1
is_ground_truth=false
```

标注者不得查看 depth/flow/geometry residual。`uncertain` 保留，不强制转成
stationary。

## 11. Development 审计

按以下 strata 报告：

```text
moving + person absent
stationary + person absent
uncertain
person present
target absent/occluded
inside coarse bbox
outside coarse bbox
```

只报告：

- measured coverage；
- residual distribution；
- inside/outside enrichment；
- SLAM/GT-pose sensitivity；
- LK/FB error distribution；
- MapPoint 与 no-MapPoint 分层；
- active time。

不报告 motion precision/recall，除非以后取得独立可靠 GT。

## 12. 放行与停止

G2-4F1 本身不放行 G1。

继续研究的最低方向性条件：

- `moving + person absent` 有足够 measured coverage；
- residual 相对 `stationary + person absent` 有可重复差异；
- 差异在 GT-pose 分支仍存在；
- 不完全由高 LK/FB error、边界或无效深度解释；
- 额外成本有实测记录。

若未满足：

- 冻结为负结果；
- 不调组合阈值；
- 不把 depth 与 flow 相加制造分离；
- 不解封 strict hold-out；
- 重新评估对象区域/刚性图或数据可观测性。

全阶段不变量：

```text
dynamic_decision          = none
depth_flow_fusion         = none
direct_slam_state_mutation = none
G1-F / G1-D               = locked
strict hold-out           = sealed and unopened
```
