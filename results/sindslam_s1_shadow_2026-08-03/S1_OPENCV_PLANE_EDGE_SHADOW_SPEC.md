# S1 OpenCV plane-edge substitute shadow 规范

日期：2026-08-03  
状态：实现前冻结

## 1. 身份与边界

本增量是：

> `[A] OpenCV RgbdPlane substitute for SIn-style plane-edge evidence`

SInDSLAM 论文使用 PEAC 平面轮廓补充 depth-gradient edge；公开仓库内相关
PEAC/AHC 文件为 AGPL-3.0-or-later，因此不复制其实现。本增量只使用当前工程
已经链接的 BSD OpenCV `rgbd` 模块，不能称为 PEAC 或 SInDSLAM plane-edge
复现。

本阶段仍为 shadow-only：不产生动态判决，不写 `Frame::mvbDynamic`，不清除
`mvpMapPoints`，不改变 Optimizer 或地图写入。

## 2. 输入与输出

输入：

```text
CV_32FC1 meter depth
camera matrix K
initial K-means labels
gradient split result and raw gradient edge
```

输出：

```text
plane labels                 -1 non-plane/invalid, >=0 plane id
raw plane boundary           OpenCV plane-label transition
gradient endpoint mask       paper radius/count rule
retained plane boundary      endpoint-supported plane segments
combined edge                gradient OR retained plane boundary
combined split labels        -1 invalid, 0 combined boundary, >0 core
```

`unknown/non-plane` 不解释成 static，也不自动解释成 boundary。PNG 只用于可视化；
`-1 invalid` 与 `0 boundary` 的区别由内存结果和 CSV 统计保存。

## 3. 平面提议 `[A]`

1. 用 `cv::rgbd::depthTo3d` 将有效米制深度反投影为 organized XYZ；
2. 用 `cv::rgbd::RgbdPlane` 获取平面标签；
3. 只在四邻域标签不同且至少一侧属于平面时生成 raw plane boundary；
4. 参数是 OpenCV substitute 的实验配置，不宣称为 SIn/PEAC 参数：

```text
block_size=16
min_plane_pixels=2000
distance_threshold=0.01 m
sensor_error_a=0.0075
```

## 4. 论文端点约束 `[L/A]`

按论文描述，gradient edge 像素在半径 2 圆形邻域内若 edge 支持少于 5，记为
endpoint。raw plane boundary 去掉 gradient edge 后做 8 邻域连通分量。

由于 OpenCV label transition 与 PEAC contour 的栅格表达不同，本 clean-room
版本将“plane segment covers endpoint”具体化为：segment 经半径 2 ellipse
dilation 后覆盖的不同 endpoint 像素数。仅当计数严格大于 1 时保留 segment。
这是 `[A]` 接口适配，不是论文未写明的事实。

最终：

```text
combined_edge = gradient_edge OR retained_plane_boundary
```

然后在每个 initial K-means region 内，按 combined edge 重新生成 core component。

## 5. RAG 接口

plane shadow 打开时，RAG 输入切换为：

```text
combined split labels + combined real-edge mask
```

关闭时保持原 gradient-only 路径。RAG 的 M1/M2/M3、阈值和 rank 规则不变。
当前没有实现论文独立的 pair-level `M_rej`，因此必须继续报告：

```text
plane_rejection_available = false
```

combined edge 只能阻止同一 initial region 内被切开的 component 回并；它不
等价于“任意 pair 的连接处含 plane edge 就拒绝”。这里的目的只是检验平面
证据作为 split/real-edge 输入是否改善区域表示，不调整动态阈值。

## 6. 必须统计

- 有效深度、平面覆盖率和 plane count；
- raw/retained plane-boundary pixels；
- gradient endpoint 数；
- plane segment 总数、保留数及无端点支持数；
- combined-edge pixels；
- gradient-only 与 combined 的 component 数、碎裂率；
- plane extraction、edge build、endpoint filter、combined split、总耗时；
- 与作者 final labels 的 ARI/NMI和边界 precision/recall，只作描述性参考；
- `dynamic_decision=none`、`actual_slam_removed=0`、
  `direct_slam_state_mutation=none`。

## 7. 放行与停止条件

至少满足：

- 合成平面转折能产生 plane boundary；
- 无平面的 invalid/unknown 不被填成区域；
- 没有超过一个 gradient endpoint 支持的 plane segment 被丢弃；
- combined boundary 在 initial region 内确实阻止跨边连接；
- 重复运行像素级确定；
- 默认关闭时保持旧路径；
- 30 帧 shadow invariant 通过且没有 SLAM 状态修改。

若 OpenCV substitute 造成全图伪边、严重碎裂、几乎没有保留边界，或成本不可
接受，则将其记录为失败替代方案；不通过添加任意面积、膨胀或分数阈值掩盖。

即使通过，也不能进入 S2。S1 后续仍需在冻结的区域内建立稠密 observed-flow
与 camera-induced-flow residual，并加入论文有依据的时序动态状态。
