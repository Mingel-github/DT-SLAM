# R4/S4 长时间间隔 Mapping 精修 Shadow 规范

日期：2026-08-05
前置结果：R2 后路线决策已放行 Mapping-only shadow
性质：离线、只读、只评价建图 mask

## 1. 研究问题

固定现有 S1–S3 输出和同一相机轨迹，仅增加 SInDSLAM Eq. (17) 风格的长时间间隔
深度重投影，回答：

> 相邻帧 S3 漏掉的慢速、间歇或短暂停留动态深度，能否在不造成明显额外静态删除的
> 条件下，由每5帧取一张 sparse frame 的深度冲突补充出来？

## 2. 冻结输入

第一轮只使用已冻结 Gazebo 600帧片段：

- 相同 associations；
- 相同 RGB-D；
- R1 保存的当前 SIn dynamic mask；
- R1 保存的 RAG region labels（RAG 为区域邻接图）；
- R1 geometry-only 的同一条 ORB-SLAM2 最终轨迹；
- Gazebo visible-box reference 仅用于评价，不进入算法。

不重新运行 DeepFlow，不重新聚类，不改变 S2/S3。

## 3. Paper-text-guided 算法

每5个输入帧选一张 sparse frame。对当前 sparse frame `k` 与上一 sparse frame
`k-1`：

```math
u'=\pi\left(T_{k-1\leftarrow k}\pi^{-1}(u,D_k(u))\right),
```

```math
\delta_{diff}(u)=|D_k(u)-D_{k-1}(u')|.
```

若：

```math
\delta_{diff}(u)>0.13D_k(u)
```

且该像素尚未被当前 S3 mask 判为动态，则成为新增 Mapping seed。

对当前 RAG region `C_i`，若新增 seed 占 region 有效像素比例超过0.4，则整个 region
加入 refined mapping mask；否则只加入 seed。

这两个数值由 SInDSLAM 公开 Mapping 实现核对，但第一版采用全像素比例，不复制源码
中的降采样计数补偿。

## 4. 有效性与遮挡语义

- 当前和参考深度必须有效；
- 投影必须在图像内；
- 最近邻读取参考深度，避免深度插值污染；
- 无效深度、越界和没有参考的像素保持 unknown；
- 第一版不把参考帧 dynamic mask 自动传播为当前动态，避免把源码额外逻辑与 Eq. (17)
  混为一体；
- 只生成 `M_map_refined = M_S3 OR M_long_interval`；
- 不生成自由空间，不把被 mask 的射线解释成 free。

## 5. 必须报告的指标

### 5.1 箱体参考

- sparse frame 中 visible-box 数量；
- S3 原箱体覆盖；
- S4 新增箱体覆盖；
- S3+S4 最终箱体覆盖；
- 按 `<3 m`、`3–6 m`、`>6 m` 分组；
- 漏检帧被 S4 恢复的数量。

### 5.2 箱外风险

- 新增 non-box 像素比例；
- 箱子不可见帧中的新增比例；
- non-box 仍可能包含运动行人，不称为静态误检；
- 联系表人工检查墙面、地面和深度边界。

### 5.3 映射层

只有像素 shadow 结果通过后，才使用同一轨迹和同一 sparse frames 输出：

- S3 点云；
- S3+S4 点云；
- 箱子 swept-volume ghost occupancy；
- 静态高时间支持体素保留率；
- 低时间支持体素删除率；
- 点云空洞与运行时间。

## 6. 通过与停止条件

允许进入点云对照，至少需要：

1. S4 在当前 S3 漏检箱体帧中增加非零覆盖；
2. 新增证据在箱体或人物运动区域有可视化支持；
3. 箱子不可见帧的广泛背景新增不出现结构性失控；
4. 所有输出保持 Mapping-only。

以下任一情况发生即停止，不做阈值搜索：

- S4 几乎不增加动态对象覆盖；
- 主要新增落在大面积墙面、地面或深度边界；
- SLAM位姿误差使深度冲突广泛污染背景；
- 只有调小0.13或0.4才能在当前序列获得结果；
- 需要同时加入 OctoMap、plane edge 或新 flow 才能解释结果。

## 7. 实施边界

第一提交只允许新增一个离线审计工具和结果文件。不得修改：

- `Tracking.cc`；
- SIn detector；
- S3 depth filter；
- `Optimizer.cc`、g2o；
- YOLO；
- LocalMapping、LoopClosing；
- 在线点云或 OctoMap。
