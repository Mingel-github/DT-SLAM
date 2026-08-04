# SInDSLAM 独立复现与初步审计结果

日期：2026-08-02

## 1. 结论摘要

SInDSLAM 已在独立目录完成 CPU/non-ROS 构建，并在 TUM、Bonn 动态箱子和 TUM 静态序列上完成端到端运行。整个过程没有修改或合并 DT-SLAM。

当前证据支持以下结论：

- 作者完整的“几何重聚类 + 稠密光流残差 + 时序 mask”路线能够在本机运行，并在 TUM `fr3/walking_xyz` 上复现到接近仓库附带轨迹的 ATE；
- 在 Bonn `moving_nonobstructing_box` 上，ATE 与论文 Table V 中同类 `moving_no_box` 序列处于相近量级，同时显著优于同代码关闭动态检测的单次 baseline；
- 在 TUM `fr1/xyz` 静态序列上，开启和关闭动态检测的 ATE/RPE 基本相同，没有观察到明显静态轨迹退化；
- 后续 mask 审计发现，Bonn non-obstructing 上虽然轨迹很好，但动态 mask 会间歇性覆盖大块墙面、桌面或柜体，也不能持续覆盖箱子；因此“轨迹保护有效”不能写成“未知对象分割准确”；
- 完整 CPU 路线只有约 3.4--4.3 FPS，不能未经取舍直接并入当前 DT-SLAM 实时前端；
- Bonn `moving_obstructing_box` 的 ATE 虽优于 baseline，但仍较高，且逐帧 RPE 反而更差。因此不能宣称 SInDSLAM 已在所有箱子场景稳定复现。

## 2. 源码与复现边界

- 官方仓库：`https://github.com/qimao7213/SInDSLAM.git`
- 固定官方 commit：`be770cdafa1d4ae4e5fde0537fa3c3da73f76cdc`
- 本地官方镜像：`/data/dynaslam/SInDSLAM`
- 本地兼容分支：`reproduction/ubuntu22-opencv45-noros`
- 官方 `master` 未修改；兼容改动只保存在独立分支。
- 根目录未发现针对作者新增代码的明确 LICENSE；嵌套 `ORB_SLAM2/LICENSE.txt` 为 GPLv3。后续移植代码前仍需处理许可证边界，当前只做研究复现与审计。

这不是把 SInDSLAM 合并进 DT-SLAM，也不是将当前轻量 LK 路线替换成 SInDSLAM。

## 3. 本机环境

- Ubuntu 22.04.5
- GCC/G++ 11.4
- CMake 3.22.1
- OpenCV 4.5.4（系统 CPU 构建；本轮使用 DeepFlow）
- Eigen 3.4.0
- PCL 1.12
- Pangolin 0.9.5（`/home/zhu/dynaslam_ws/pangolin_install`）
- RTX 4060 Ti 可用，但系统 OpenCV 未提供本方法所需的 CUDA optical-flow 构建，因此本轮没有伪称 GPU 复现。

## 4. 最小兼容改动

只改了独立 SInDSLAM 分支中的 5 个文件：

| 文件 | 改动 | 性质 |
|---|---|---|
| `ORB_SLAM2/CMakeLists.txt` | ROS include 加存在性检查；OpenCV 从严格 4.2 改为 4.x；选择 non-ROS CPU runner | 构建兼容 |
| `ORB_SLAM2/Thirdparty/DBoW2/CMakeLists.txt` | OpenCV 从严格 4.2 改为 4.x | 构建兼容 |
| `ORB_SLAM2/Thirdparty/g2o/CMakeLists.txt` | 固定 C++14，与主工程保持 Eigen 对齐 ABI 一致 | 运行时兼容 |
| `ORB_SLAM2/include/PEAC/plane_fitter_pcl.hpp` | `pcl_isnan` 改为 `std::isnan` | PCL 1.12 API 兼容 |
| `ORB_SLAM2/Examples/RGB-D/rgbd_tum_noros.cc` | 增加默认不变的 Viewer、动态检测 baseline 和指定帧段 mask 调试输出开关 | 评价基础设施 |

未修改：

- `DynaDetect.cc` 的聚类、阈值、稠密光流、时序和 mask 逻辑；
- ORB-SLAM2 Optimizer/g2o 数学目标；
- 作者的 `TUM1/TUM3/Bonn.yaml` 参数；
- DT-SLAM 的任何源码。

## 5. 已定位的兼容问题

### 5.1 Eigen/g2o 对齐 ABI

原构建中 ORB_SLAM2 明确使用 C++14，而旧 g2o 在 GCC 11 下默认使用 GNU++17。ASan 在第二帧定位到：

```text
AddressSanitizer: attempting free on address which was not malloc()-ed
g2o::HyperGraph::clear()
ORB_SLAM2::Optimizer::PoseOptimization()
```

分配发生在 Eigen 的 aligned allocator，释放发生在用另一 C++ 标准编译的 g2o 中。统一 g2o 为 C++14 后，原 `bad-free` 消失，全部帧可正常处理。

### 5.2 Pangolin 退出崩溃

当前 Pangolin 0.9.5 下，Viewer 开启时序列能够处理并保存轨迹，但进程在退出阶段崩溃。GDB 栈指向：

```text
pangolin::GlFont::~GlFont()
pangolin::PangolinGl::~PangolinGl()
libepoxy
```

这发生在算法完成和轨迹保存之后。批量评价通过：

```bash
export SIND_SLAM_DISABLE_VIEWER=1
```

只关闭 Pangolin Viewer；作者原有的 OpenCV `Gray/imgDyna/imgDynaRGB` 窗口仍保留。默认不设置环境变量时仍保持作者原行为。

## 6. 方法代码审计摘要

作者实现不是“简单 K-means”或“简单光流”：

1. 深度点以固定 640x480 图像构造 12 个初始三维簇；
2. 计算深度边缘和平面边缘，对混合簇继续切分；
3. 建立 Region Adjacency Graph，并利用深度直方图等规则合并过分割区域；
4. CPU 路径使用 OpenCV DeepFlow，计算观测流及相机运动补偿后的残余流；
5. 光流/几何与区域上下文结合生成动态判决；
6. 保存上一帧区域和动态状态作为时序先验；
7. runner 对最终 mask 再执行 15x15 膨胀；
8. mask 中 `255=dynamic`、`125=static`、`0=invalid/unknown`；ORB 描述子计算前跳过值为 255 的关键点。

K-means/re-clustering 与 dense optical flow 在两个并发任务中运行，但整帧 Tracking 仍等待最终 mask，因此不是异步语义/几何流水线。

## 7. 输入与评价协议

### TUM 作者协议

按 README 使用作者 `associate.py` 的：

```text
offset = -0.033 s
max_difference = 0.02 s
```

`fr3/walking_xyz` 得到 823 对，与仓库附带 `EVO/CameraTrajectory.txt` 的 823 帧一致。

### Bonn 协议

- 使用本地一对一、20 ms 内且文件实际存在的 RGB-depth association；
- 使用源码自带 `Bonn.yaml`；
- ATE/RPE 时间关联使用 `t_max_diff=0.02 s`，与作者附带旧 TUM 评价脚本的默认值一致；
- `Bonn.yaml` 将畸变设为 0，而 DT-SLAM 的 Bonn 管线会把 RGB/depth/mask 统一 remap 到去畸变针孔域。两者不能在未标注坐标域差异时直接横比。

## 8. 单次运行结果

所有结果均为单次运行，单位为米。RPE 为 `delta=1 frame` 的平移 RMSE。

| 序列/模式 | 轨迹帧数 | GT 配对 | ATE RMSE | RPE RMSE | 作者报告的检测均值 | wall time | 实际吞吐 |
|---|---:|---:|---:|---:|---:|---:|---:|
| TUM fr3/walking，SIn，作者 offset | 823 | 822 | 0.014852 | 0.012543 | 194.886 ms | 199.04 s | 4.13 FPS |
| TUM fr3/walking，同代码 baseline | 823 | 822 | 1.158072 | 0.028090 | 0 ms | 38.78 s | 21.22 FPS |
| Bonn moving_nonobstructing_box，SIn | 778 | 776 | 0.023097 | 0.014917 | 187.683 ms | 181.33 s | 4.29 FPS |
| Bonn moving_nonobstructing_box，同代码 baseline | 778 | 776 | 0.321157 | 0.032476 | 0 ms | 34.96 s | 22.25 FPS |
| Bonn moving_obstructing_box，SIn | 589 | 588 | 0.274588 | 0.024518 | 221.834 ms | 158.11 s | 3.73 FPS |
| Bonn moving_obstructing_box，同代码 baseline | 589 | 588 | 0.428828 | 0.019075 | 0 ms | 27.14 s | 21.70 FPS |
| TUM fr1/xyz 静态，SIn | 796 | 794 | 0.009687 | 0.005712 | 246.483 ms | 232.68 s | 3.42 FPS |
| TUM fr1/xyz 静态，同代码 baseline | 796 | 794 | 0.009699 | 0.005602 | 0 ms | 35.95 s | 22.14 FPS |

附加核对：

- 仓库附带的作者 TUM `fr3/walking_xyz` 轨迹在当前 evo 协议下 ATE RMSE 为 `0.014398 m`；本机复现为 `0.014852 m`。
- 论文 Table V 的 SInDSLAM `moving_no_box` / `moving_no_box2` ATE 为 `0.0190/0.0307 m`。本地 `moving_nonobstructing_box` 的命名和结果量级相近，但尚未证明它与论文表格中的具体 take 完全相同。
- `moving_obstructing_box` 中 SIn 的全局 ATE 优于 baseline，但局部逐帧 RPE 更差；不能只报告 ATE 的正面部分。

## 9. 当前能说与不能说的结论

### 可以说

- SInDSLAM 官方 CPU/non-ROS 路线已在当前环境完成独立复现；
- TUM walking 结果与作者仓库附带轨迹高度接近；
- Bonn non-obstructing 未知箱子序列出现论文量级的 ATE，并显著优于同代码 baseline；
- 完整实现包含当前 DT-SLAM 轻量 LK 所缺少的区域表示、稠密残余流和时序状态层，但 mask 审计表明该中间层在当前 Bonn 运行中并不等于可靠对象分割；
- 静态 fr1/xyz 本轮未见明显 ATE/RPE 退化。

### 还不能说

- 不能说 SInDSLAM 在所有 Bonn 箱子序列都稳定有效；
- 不能说单次 ATE 差异已经排除了 ORB-SLAM2 随机性；
- 不能说 mask 的像素级 precision/recall 已验证；
- 不能因为 Bonn non-obstructing ATE 很好，就说 mask 主要覆盖未知运动箱子；
- 不能说 CPU 路线满足实时；
- 不能说已经决定将完整 SInDSLAM 合并进 DT-SLAM；
- 不能直接复制源码到 DT-SLAM，许可证边界仍需明确。

## 10. 下一步建议

1. mask/overlay 输出与 Bonn non-obstructing 两个完整时序帧段审计已经完成，详见 `results/sindslam_mask_audit_2026-08-02/SINDSLAM_MASK_AND_MODULE_AUDIT_RESULT.md`；
2. 两个明确源码风险和上一帧 mask 先验的开关化 A/B 已完成；它们能改变最坏覆盖，但没有消除大块静态误标，且公开实现存在运行间 mask 非确定性；
3. 对关键序列做少量重复运行，报告中位数和范围；
4. 单独评估 CUDA/OpenCV 构建成本。当前系统 OpenCV 4.5.4 无 CUDA optical-flow 支持，不能把 RTX 可用等同于 SIn GPU 路线可用；
5. 当前不再继续围绕 morphology 或时序先验追加补丁；主要未解决因素仍是 dense-flow/homography residual 到区域动态判决的可靠性；
6. 决策为：SInDSLAM 独立保留为重型区域外部 baseline；当前轻量 LK 保留为默认关闭的稀疏实验 baseline；暂不直接移植完整 `DynaDetect.cc`。

当前不建议直接把完整 `DynaDetect.cc` 并入 DT-SLAM。
