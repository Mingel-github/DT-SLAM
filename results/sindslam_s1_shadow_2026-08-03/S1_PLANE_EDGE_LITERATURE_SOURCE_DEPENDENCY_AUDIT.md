# S1 Plane edge：论文、源码、依赖与许可证审计

日期：2026-08-03  
性质：本地资料与源码只读审计

## 1. 为什么它属于原计划

SInDSLAM 的几何重聚类不是只有 depth-gradient edge。论文使用：

```text
gradient edge
+ plane edge
→ 几何边界
→ 切分 initial K-means region
→ RAG merge/rejection
```

plane edge 用来补足相近深度或缓慢深度变化处的物体/平面边界，例如脚与
地面。这是 S1 已冻结的“深度/平面边缘辅助区域切分”，不是临时新增模块。

依据：

- `/home/zhu/Desktop/paper_notes/SInDSLAM.md`；
- 本地正式论文正文 pp.2247–2249；
- `/data/dynaslam/SInDSLAM_cuda/ORB_SLAM2/src/DynaDetect.cc`。

## 2. 论文方法 `[L]`

论文先用 PEAC 提取初始 plane edge `E'_plane`，再减去 gradient edge：

```text
{E_seg} = E'_plane - E_grad
```

gradient edge 上的 endpoint 由半径 2 的圆形邻域定义：邻域内 gradient
edge 少于 5 个像素时为 endpoint。只有覆盖超过一个 endpoint 的 plane
segment 才保留，最后：

```text
E_edge = E_grad union E_plane
```

plane edge 还用于 RAG rejection，避免跨真实平面边界合并。

## 3. 作者源码行为 `[C]`

作者实现：

- 把深度反投影为 organized PCL point cloud；
- 调用 bundled PEAC/AHC `PlanarContourExtraction`；
- wrapper 参数约为 `minSupport=2000`、`16×16 window`、refine=true；
- 后处理还含轮廓长度、dilate/erode 和 endpoint 接触启发式；
- 论文要求 segment 覆盖“超过一个 endpoint”，源码实际接受至少一个，
  二者不一致。

这些实现参数是作者工程行为，不应自动写成论文通用参数。

## 4. 许可证边界

bundled PEAC/AHC 头文件明确标记：

```text
SPDX-License-Identifier: AGPL-3.0-or-later
```

因此不把其代码直接复制进当前 DT-SLAM。独立运行和行为审计可以继续；若
在 DT-SLAM 内实现，应依据论文和许可安全 API clean-room 重写。

## 5. 本机可用的许可安全替代

当前构建已经链接 OpenCV `rgbd` 模块。本机
`cv::rgbd::RgbdPlane`：

- 输入 organized `CV_32FC3`；
- 输出 plane labels 和 plane coefficients；
- OpenCV/Willow Garage BSD 许可；
- 不增加 PCL 依赖。

它不是 PEAC，若采用，身份必须写成：

> `[A] OpenCV RgbdPlane substitute for SIn-style plane-edge evidence`

不能称为作者 PEAC 复现，也不能预设输出与作者相同。

PCL `OrganizedMultiPlaneSegmentation` 同样许可安全，但会给当前 DT-SLAM新增
PCL 构建和部署依赖，第一选择不采用。

## 6. 建议的最小 clean-room shadow

输入：

```text
CV_32F metric depth + K
raw gradient edge
initial/split labels
```

步骤：

1. `depthTo3d` 得到 organized XYZ；
2. `RgbdPlane` 得到 plane labels；
3. 从相邻 plane-label 变化提取 raw plane boundary；
4. 非平面和 invalid 保持 unknown，不解释成 edge/static；
5. 减去 gradient support，按论文 endpoint 结构筛选 segment；
6. 输出 `gradient union plane` 边界，先只做 split/RAG shadow 对照。

必须记录：plane coverage、raw/retained boundary、endpoint/segment 数、与作者
final partition 的描述性边界关系、区域数、欠/过分割和耗时。

## 7. 决策

优先实现 OpenCV plane substitute 的独立 shadow，并与当前 gradient-only
版本成对比较。若它造成大面积伪边、区域全碎或运行成本不可接受，则冻结为
失败，不通过添加无文献依据的面积补丁掩盖。

在 plane edge 尚未通过前，可以独立测量 dense-flow residual，但不能产出
正式动态 mask、不能开放 S2，也不能称为完整 SIn-style detector。
