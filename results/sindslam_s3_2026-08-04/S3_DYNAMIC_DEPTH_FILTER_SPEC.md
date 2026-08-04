# S3 映射侧动态深度过滤规格

日期：2026-08-04  
状态：冻结后实施  
全称：`S3 = SInDSLAM 风格动态区域掩码的映射侧深度过滤`

## 1. 当前工程事实

DT-SLAM 当前仍是 ORB-SLAM2 稀疏地图系统：仓库中没有稠密点云融合器、TSDF、
OctoMap 或既有的 RGB-D 点云写入线程。因此 S3 第一版不能声称“接入现有稠密地图”，
而应先提供明确的映射输入接口和可复现的离线点云评价。

作者原版 SInDSLAM 的 ROS 点云模块会接收 RGB、depth、dynamic mask 和 pose，并在生成
点云时跳过 dynamic mask 内的深度像素。本地源码证据：

```text
/data/dynaslam/SInDSLAM/octomap_pub/src/pubPointCloud.cc
```

该行为只作为 `[L]` 方法依据；不复制作者 PCL/ROS/OctoMap 代码。

## 2. S3 第一版目标

输入：

- 当前注册到 RGB 域的 `CV_32FC1` 米制深度；
- semantic dynamic mask：`CV_8UC1`，`0=保留`、`非零=动态`，可以为空；
- SIn 风格 geometry dynamic mask：同一图像域、同一极性，可以不可用。

输出：

```cpp
struct SInStyleDepthFilterResult
{
    cv::Mat dynamicDepthMask;     // CV_8UC1, 0=keep, 255=reject
    cv::Mat staticDepthMeters;    // CV_32FC1, dynamic pixels set to 0.0f
    SInStyleDepthFilterStats stats;
};
```

第一版支持三个明确模式：

| 配置值 | 中文含义 |
| --- | --- |
| `semantic_only` | 只使用 YOLOv8-seg 语义动态掩码 |
| `geometry_only` | 只使用 SIn 风格几何动态掩码 |
| `semantic_or_geometry` | 两种动态掩码取并集 |

没有证据的像素不会被解释为“已证实静态”；它只是未被当前 mask 否决。统计中分别保留
semantic、geometry、union 和有效深度覆盖量。

## 3. 冻结不变量

1. 不修改传给 `Frame` 的原始深度；
2. 不改变 `mvDepth`、ORB 特征、Tracking、位姿优化或 S2 判决；
3. S2 tracking fail-open 不影响 S3：即使某帧临时借用几何候选完成位姿，几何区域深度
   仍被映射接口否决；
4. 只在 Tracking 成功、当前位姿有效时把过滤深度标记为 mapping-admissible；
5. 动态深度写为 `0.0f`，与 ORB-SLAM2 RGB-D 的无效深度约定一致；原始深度仍保留；
6. 输入为空、类型错误、尺寸错误或 geometry 不可用时必须有明确状态；不得越界或静默
   改变极性；
7. 默认关闭，不引入 OctoMap、PCL、TSDF 或长间隔精修；
8. 不修改 `Optimizer.cc`、g2o、YOLO、LocalMapping 或 LoopClosing 算法。

## 4. 最小工程接口

新增独立小模块：

```text
include/SInStyleDepthFilter.h
src/SInStyleDepthFilter.cc
```

`Tracking` 保存当前结果；`System` 只提供只读 clone 接口：

```cpp
cv::Mat GetCurrentDynamicDepthMask();
cv::Mat GetCurrentStaticDepthForMapping();
```

关闭 S3 或 Tracking 失败时，mapping depth getter 返回空矩阵。该接口不创建后台线程，
也不维护全局点云。

## 5. 诊断与评价

逐帧 CSV 至少记录：

- 输入有效深度像素；
- semantic/geometry/union 动态像素；
- 被否决的有效深度像素；
- 输出有效深度像素；
- geometry evidence 是否可用；
- Tracking 后是否允许写图；
- filter runtime；
- `tracking_state_mutation=none`。

可选输出目录只保存最终 `dynamicDepthMask` PNG。离线点云工具使用：原始 RGB-D、最终
轨迹和这些 mask 生成相同位姿下的未过滤/过滤点云，避免在线点云受后续回环位姿更新影响。

## 6. 第一轮验收

1. 合成矩阵测试：类型、尺寸、极性、并集、无效深度和输入不变性；
2. default-off TUM smoke 结果不变；
3. Bonn `moving_nonobstructing_box`：输出非零动态深度否决，Tracking/ATE/RPE 与 S2
   保持一致；
4. Bonn `static_close_far`：报告静态深度否决比例和空洞风险，不要求像素零误杀；
5. 额外运行时间应只包含 mask 合并与一次深度 copy/setTo，不预设 30 FPS；
6. 第一轮不实现 S4 长间隔精修，也不以大膨胀掩盖残影。

S3 通过只意味着动态区域已经形成明确、可消费的映射深度输出；只有离线点云残影与静态
完整度评价通过后，才能称为动态深度地图保护有效。
