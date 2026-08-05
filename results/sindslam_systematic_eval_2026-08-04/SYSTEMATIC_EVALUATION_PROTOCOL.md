# DT-SLAM SIn 风格几何系统测量协议

日期：2026-08-04

## 1. 固定版本

- 算法提交：`16ea79d`；
- 本轮新增内容仅为成对相机配置和可复现运行器，不修改检测算法；
- 正式计时关闭 Viewer；
- 各模式顺序运行，不并发占用 CPU/GPU。

## 2. 代表序列

| 数据集 | 序列 | 有效 RGB-D 对 | 作用 |
| --- | --- | ---: | --- |
| TUM | fr3_walking_xyz | 827 | 明显人物运动 |
| TUM | fr3_sitting_static | 680 | 局部运动和短暂停留 |
| TUM | fr1_xyz | 792 | 静态负样本 |
| Bonn | moving_nonobstructing_box | 778 | 未知箱子正常运动 |
| Bonn | moving_obstructing_box | 589 | 强遮挡 |
| Bonn | static_close_far | 1750 | 静态近远景和深度空洞风险 |

## 3. 四种系统模式

| 名称 | 语义 YOLO | SIn 区域检测 | S2 Tracking 特征过滤 | S3 建图深度输出 |
| --- | ---: | ---: | ---: | ---: |
| orb_baseline | 关 | 关 | 关 | 关 |
| semantic_only | 开 | 关 | 关 | 仅语义 mask |
| geometry_only | 关 | 开 | 开 | 仅几何 mask |
| semantic_geometry | 开 | 开 | 开 | 语义与几何并集 |

## 4. 坐标域和特征数

- TUM3：官方 TUM3 内参、1000 个 ORB 特征；
- TUM1：官方 TUM1 内参、1000 个 ORB 特征；
- Bonn：RGB、depth、YOLO mask、SIn mask 和 ORB 全部使用同一个去畸变 P=K 域，
  四种模式均使用 1500 个 ORB 特征；
- 不将此前作者行为审计用的 Bonn 原始域结果混入本轮正式表格。

## 5. 轨迹指标

- ATE RMSE：Absolute Trajectory Error，绝对轨迹误差平移均方根；
- RPE RMSE：Relative Pose Error，相邻帧相对位姿平移均方根；
- TUM/Bonn 均使用 SE(3) align；
- 时间戳最大匹配差固定为 0.02 s；
- 同时记录 GT 匹配 pose 数、Tracking OK/LOST、重定位和关键帧数。

## 6. 速度与过滤指标

- actual FPS；
- image I/O、Tracking 和 active total 的 mean/median/p95；
- deadline missed；
- SIn 区域动态像素、动态 ORB、候选剩余特征和 fail-open；
- S3 有效深度拒绝率与过滤耗时；
- 语义推理 provider、mask age 和推理耗时。

## 7. 地图质量

先在每个数据集选代表动态/静态序列，使用同一最终位姿生成过滤前后点云，再比较：

- 低时间支持残影体素的清除比例；
- 高时间支持稳定体素的完整保留比例；
- Viewer/点云人工检查中的人物、箱子残影和大面积静态空洞。

地图指标不会用来替代 ATE/RPE，也不会把无 GT 的代理指标写成对象检测精度。
