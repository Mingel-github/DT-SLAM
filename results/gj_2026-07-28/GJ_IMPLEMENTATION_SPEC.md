# GJ — Ji et al. ICRA 2021 baseline 实现规范

日期：2026-07-28  
阶段：GJ-1 depth clustering shadow

## 1. 文献身份

目标论文：

> T. Ji, C. Wang and L. Xie, “Towards Real-time Semantic RGB-D SLAM in
> Dynamic Environments,” ICRA 2021.

本阶段的目标是建立可追溯的文献baseline，不是DT-SLAM最终几何方法，也不作为
创新点。

原始论文没有公开实现代码。因此必须区分：

- `[P]`：论文正文明确给出的算法；
- `[E]`：为了在当前工程中得到确定、可复现结果而采用的工程选择；
- `[A]`：为适配当前DT-SLAM接口做出的改造；
- `[U]`：论文未说明，当前阶段不能宣称复现。

## 2. 论文明确支持的部分

论文Section III-B与IV-b明确给出：

1. `[P]` 对每个新深度图运行geometry clustering；
2. `[P]` 将深度点投影到3D，空间接近的点使用K-means分为`N`个cluster；
3. `[P]` 在`640×480`图像上使用`N=24`；
4. `[P]` 一个物体允许被分成多个cluster；
5. `[P]` 对cluster内具有地图对应`P_i`的观测特征`u_i`计算平均重投影误差：

   ```text
   r_j = (1/m) Σ ρ(||u_i - π(T P_i)||²)
   ```

6. `[P]` 相对其他cluster误差较大的cluster被标记为动态；
7. `[P]` 动态cluster内的特征不参与后续位姿估计；
8. `[P]` 论文承认地图匹配点不足时，几何检测区域会不完整；
9. `[P]` 论文报告geometry module约`30.14 ms/frame`，但其“real-time”定义是
   约`100 ms/frame`，不能据此推导DT-SLAM能够达到30 Hz。

## 3. 论文没有明确说明的部分

以下内容在论文中没有足够信息：

- `[U]` K-means初始化方式；
- `[U]` attempts数量；
- `[U]` 最大迭代次数和epsilon；
- `[U]` 是否对稠密深度点降采样；
- `[U]` 无效深度的具体处理代码；
- `[U]` 深度范围裁剪；
- `[U]` 惩罚函数`ρ`的具体形式和参数；
- `[U]` 动态cluster阈值的数值或完整自适应公式；
- `[U]` cluster至少需要多少匹配地图点；
- `[U]` 位姿符号`T_wc`与具体代码坐标变换的约定；
- `[U]` 随机数种子。

因此，GJ-1不得实现动态判定；GJ-2开始前还需要单独定义上述未公开部分的
baseline适配规则。

## 4. GJ-1输入输出

### 输入

```cpp
CV_32FC1 米制当前深度图
raw registered depth像素域的零畸变针孔K
```

### 输出

```cpp
CV_16SC1 cluster label image
  -1 = invalid/no depth
  0..N-1 = cluster id

每个cluster：
  pixel count
  3D centroid

每帧：
  valid depth count
  compactness
  prepare/kmeans/label/total runtime
```

GJ-1不读取语义mask，不读取当前位姿，不读取MapPoint，也不生成dynamic mask。

## 5. GJ-1工程适配

为得到确定、可复现的shadow baseline，本阶段使用：

| 项目 | 当前选择 | 属性 |
| --- | --- | --- |
| 输入点 | 所有有限且`z>0`的深度像素 | `[E]` |
| 3D坐标 | `(u-cx)z/fx, (v-cy)z/fy, z` | `[E]` 标准反投影 |
| cluster数量 | 24 | `[P]` |
| 初始化 | OpenCV `KMEANS_PP_CENTERS` | `[E]` |
| attempts | 1 | `[E]` |
| 最大迭代 | 20 | `[E]` |
| epsilon | `1e-3` | `[E]` |
| RNG seed | 2021，调用后恢复OpenCV RNG状态 | `[E]` |
| 无效label | -1 | `[E]` |
| 接入位置 | `GrabImageRGBD()`内`Track()`返回后 | `[A]` |

除cluster数量外，上述数值均不能称为Ji 2021论文参数。

接入位置在首次短序列测试后从`TrackLocalMap()`之前修正为`Track()`返回后。原因是
`Tracking::Track()`持有地图更新互斥锁；在其内部同步执行约几十毫秒的K-means会阻塞
Local Mapping，并可能通过线程调度间接改变结果。GJ-1只需要当前深度，不依赖初始
位姿或地图匹配，因此应在锁外作为纯shadow测量运行。未来真正进入过滤阶段时，再
根据G1的验收结果选择跟踪内部接入点。

## 6. 代码边界

新增独立文件：

```text
include/JiGeometryBaseline.h
src/JiGeometryBaseline.cc
Examples/RGB-D/ji_geometry_test.cc
Examples/RGB-D/TUM3_JiGeometryShadow.yaml
Examples/RGB-D/TUM1_JiGeometryShadow.yaml
```

最小修改：

```text
CMakeLists.txt
include/Tracking.h
src/Tracking.cc
```

明确不修改：

```text
YOLOSegment
Optimizer.cc
g2o
LocalMapping
LoopClosing
Frame::mvbDynamic
MapPoint写入
```

## 7. GJ-1验收

### 工程验收

- 配置关闭时不执行；
- 只支持RGB-D；
- 非零畸变raw域未提供合法geometry K时fail-fast；
- 无效深度label始终为-1；
- 有效深度label范围为`[0,N-1]`；
- 固定输入重复运行得到相同label；
- 不直接修改SLAM状态；
- 同步K-means耗时仍计入端到端延迟，不能把“无状态写入”解释为“无运行影响”。

### 数据验收

- 静态和动态代表帧cluster可视化可读；
- 统计cluster尺寸极差和空cluster；
- 测量`N=24`全分辨率K-means耗时；
- 不预设30 Hz成立。

## 8. 后续门控

GJ-1通过后才进入GJ-2：

```text
ORB feature → raw depth cluster
MapPoint + Tcw → undistorted reprojection error
cluster support/error statistics
```

GJ-2仍保持shadow-only。由于论文没有公开动态阈值，GJ-3之前必须明确：

1. 哪种`ρ`只是baseline工程选择；
2. 无地图点或支持不足的cluster必须标为unknown，不能标为static；
3. 动态阈值如何作为论文未公开部分进行适配和消融；
4. 不增加第三次PoseOptimization。

## 9. 本地核对材料

- `/home/zhu/Desktop/papers/Ji 等 - 2021 - Towards Real-time Semantic RGB-D SLAM in Dynamic Environments.pdf`
- `/home/zhu/Desktop/paper_notes/Ji2021_RealTime_Semantic_RGBD_SLAM.md`
- `/home/zhu/Desktop/paper_notes/comparison_23_papers.md`
- `results/literature_audit_2026-07-27/DT-SLAM_几何模块文献核对与更改报告.md`
