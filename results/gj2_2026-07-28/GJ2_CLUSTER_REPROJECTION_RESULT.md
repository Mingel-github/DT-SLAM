# GJ-2 Cluster Reprojection Shadow 结果

日期：2026-07-28  
依据：Ji et al., ICRA 2021  
范围：初始位姿、ORB-to-cluster映射、cluster重投影证据统计；不做动态判定。

## 1. 结论

GJ-2工程链路通过，但当前证据不足以进入GJ-3动态cluster阈值实现。

- raw `mvKeys`到深度cluster的映射正常；
- `mvKeysUn + Tcw + MapPoint`针孔重投影方向和单位正确；
- 初始PoseOptimization清除关联前的inlier/outlier均被只读快照；
- 无有效地图支持的cluster保持`unknown`；
- GJ-2统计本身约`0.025–0.027 ms/frame`，不是性能瓶颈；
- walking短序列的cluster误差整体高于fr1/xyz，但分布明显重叠；
- 论文未公开`rho`、最小支持数和动态阈值，不能仅凭两段短序列补写阈值。

## 2. 关键实现修正

第一次预检曾在`TrackReferenceKeyFrame/TrackWithMotionModel`返回后快照。这时原生
ORB-SLAM2已经清除了PoseOptimization判出的高残差MapPoint关联，会系统性压低
Ji方法需要测量的cluster残差。

最终版本改为：

```text
初始PoseOptimization完成
→ 清除outlier关联之前只读快照Tcw、keypoint和MapPoint世界坐标
→ 原生Tracking继续清理和TrackLocalMap
→ Track返回并释放地图锁
→ 运行K-means和cluster误差统计
```

这样没有修改Optimizer结果，也没有把全分辨率K-means放入地图互斥锁。

## 3. 统计约定

cluster查询：

```text
Frame::mvKeys -> raw registered depth label
```

重投影：

```text
Frame::mvKeysUn -> observed undistorted pixel
Tcw * MapPoint::GetWorldPos() -> predicted camera point
tracking K -> predicted undistorted pixel
```

当前identity工程baseline：

```text
e_i² = ||u_i - pi(Tcw P_i)||²
rho(e_i²) = e_i²
```

同时输出未平方误差的mean、median、nearest-rank P90和maximum。identity
`rho`及P90定义均是显式工程选择，不是Ji论文公开参数。

## 4. 合成测试

```text
[Ji GJ-1/GJ-2 Test] PASS
```

测试覆盖：

- 已知1像素重投影误差；
- raw feature-to-cluster映射；
- optimizer outlier支持计数；
- 无支持cluster保持unknown；
- GJ-1深度聚类确定性。

## 5. TUM fr3/walking，最终v2，19帧

其中18帧具有初始跟踪位姿：

```text
有效重投影总数: 4418
optimizer outlier支持: 1022
outlier占有效支持: 23.13%
每个有位姿帧平均初始匹配: 257.17
每个有位姿帧平均unknown cluster: 5.17 / 24
支持点加权mean error: 3.71 px
支持点加权mean squared error: 43.32 px²
cluster mean error中位数: 2.89 px
cluster mean error P90: 6.67 px
cluster mean error最大值: 31.60 px
GJ-2统计平均耗时: 0.0248 ms
端到端actual FPS: 12.63
```

## 6. TUM fr1/xyz静态短对照，最终v2，14帧

其中13帧具有初始跟踪位姿：

```text
有效重投影总数: 4145
optimizer outlier支持: 791
outlier占有效支持: 19.08%
每个有位姿帧平均初始匹配: 365.62
每个有位姿帧平均unknown cluster: 3.77 / 24
支持点加权mean error: 3.21 px
支持点加权mean squared error: 39.87 px²
cluster mean error中位数: 2.44 px
cluster mean error P90: 5.08 px
cluster mean error最大值: 10.89 px
GJ-2统计平均耗时: 0.0272 ms
端到端actual FPS: 12.29
```

## 7. 客观解释

walking误差总体高于fr1/xyz，但：

- 两个序列的cluster误差范围重叠；
- 静态场景也存在约19%的optimizer outlier支持；
- 两个序列相机、深度结构和运动方式不同；
- 当前没有逐cluster运动真值；
- 高误差可能来自动态、错误匹配、深度cluster混合或初始位姿误差。

因此当前只能确认：

> Ji式cluster重投影证据已经正确接入，并在walking中表现出更长的高误差尾部。

不能确认：

> 某个固定阈值已经能够可靠区分动态与静态cluster。

## 8. 不变性检查

GJ-1与GJ-2在相同输入帧上的cluster可视图SHA-256完全一致，说明新增重投影统计
没有改变K-means输出。

没有新增PoseOptimization，没有修改：

```text
YOLOSegment
Optimizer.cc
g2o
LocalMapping
LoopClosing
Frame::mvbDynamic
mvpMapPoints内容
MapPoint写入
```

## 9. 输出

最终结果使用`v2`文件：

```text
results/gj2_2026-07-28/walking20_v2.log
results/gj2_2026-07-28/walking20_v2_clusters.csv
results/gj2_2026-07-28/walking20_v2_reprojection.csv
results/gj2_2026-07-28/walking20_v2_debug/

results/gj2_2026-07-28/fr1_xyz15_v2.log
results/gj2_2026-07-28/fr1_xyz15_v2_clusters.csv
results/gj2_2026-07-28/fr1_xyz15_v2_reprojection.csv
results/gj2_2026-07-28/fr1_xyz15_v2_debug/
```

不带`v2`的GJ-2文件是“outlier关联已被清除后的预检”，保留用于说明实现修正，
不作为最终结论。

## 10. 下一门控

暂不批准GJ-3动态阈值或实际过滤。

建议下一步为自动化GJ-2A cluster可分性审计：

```text
离线person proxy mask
+ 原始cluster label
+ cluster重投影CSV
→ 每个cluster的person overlap与误差分布
```

该代理不是真实运动真值，但可以自动检查“高误差cluster是否更集中于人物区域”，
不要求用户手工逐像素标注。只有通过该门控后，才讨论论文未公开阈值的适配与消融。
