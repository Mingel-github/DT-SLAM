# S0 CUDA/BroxFlow 配置忠实性核对规范

日期：2026-08-03

## 1. 目的

在不修改系统 OpenCV、不覆盖已验证 CPU SInDSLAM 的前提下，为独立 SInDSLAM 建立 CUDA BroxFlow 路径，并比较 CPU DeepFlow 与 GPU BroxFlow 的行为、轨迹和耗时。

S0 只核对配置忠实性。区域路线是否值得继续，已经由现有 CPU 复现提供初步积极证据。

## 2. 冻结环境

- GPU：NVIDIA GeForce RTX 4060 Ti，compute capability 8.9；
- 驱动：595.84；
- 宿主机只在沙箱外可访问 `/dev/nvidia*`，沙箱内 `nvidia-smi` 失败不代表驱动故障；
- 编译器：GCC/G++ 11.4；
- CPU 对照：系统 C++ OpenCV 4.5.4 + DeepFlow；
- 目标：CUDA Toolkit 11.8 + OpenCV/opencv_contrib 4.5.4 + BroxFlow；
- CUDA/OpenCV source、build、install 全部放在 `/data/dynaslam/toolchains`；
- GPU SIn 使用独立 worktree `/data/dynaslam/SInDSLAM_cuda`。

选择 CUDA 11.8 的理由：官方支持 Ubuntu 22.04/GCC 11，并支持 Ada `sm_89`；选择 OpenCV 4.5.4 是为了与当前 CPU 对照保持 OpenCV 版本一致。若该组合无法构建，必须保存失败证据后再决定固定的新版本，不得静默换版。

## 3. 已发现的隔离风险

1. SIn CMake 原来硬编码 `CV_CUDA=false`，命令行无法开启；
2. `DynaDetect` 使用 `<opencv4/opencv2/...>`，自定义 prefix 下可能回落到系统头文件；
3. DBoW2 现有库依赖系统 OpenCV，GPU worktree 必须重编 DBoW2；
4. SIn CMake 把库和可执行文件写入源码树，仅建立不同 build 目录仍会覆盖 CPU 基线；
5. GPU 路线没有运行时 DeepFlow 回退；
6. Brox 后仍有 GPU 下载和 CPU VariationalRefinement，不能称为全 GPU detector。

## 4. 构建身份检查

运行任何数据前必须全部满足：

- `nvcc --version` 指向隔离 CUDA 11.8；
- OpenCV build information 显示 CUDA `YES` 和 `cudaoptflow`；
- GPU SIn 编译定义包含 `USECUDA`；
- 链接命令包含 `opencv_cudaoptflow`；
- GPU binary 的 `ldd` 只解析到隔离 OpenCV，不混入系统 `/usr/lib/...opencv 4.5.4d`；
- `cv::cuda::getCudaEnabledDeviceCount() > 0`；
- 程序日志明确打印 `Flow backend: Brox_CUDA`；
- 一个独立两帧 Brox smoke 能创建、计算并下载 `CV_32FC2` flow。

## 5. 固定输入

| 序列 | 数据 | Association | 配置 |
|---|---|---|---|
| TUM fr3/walking_xyz | `/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz` | `results/sindslam_reproduction_2026-08-02/fr3_walking_author_offset_minus_0p033_associations.txt` | `TUM3.yaml` |
| Bonn moving_nonobstructing_box | `/data/dynaslam/datasets/rgbd_bonn_moving_nonobstructing_box` | `results/sindslam_mask_audit_2026-08-02/nonobstructing_associations_no_comments.txt` | `Bonn.yaml` |
| Bonn moving_obstructing_box | `/data/dynaslam/datasets/rgbd_bonn_moving_obstructing_box` | `results/g1_bonn_box_2026-07-31/inputs/moving_obstructing_box_associations_20ms.txt` | `Bonn.yaml` |
| TUM fr1/xyz | `/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg1_xyz` | `results/sindslam_reproduction_2026-08-02/fr1_xyz_author_offset_minus_0p033_associations.txt` | `TUM1.yaml` |

S0 不使用 DT-SLAM rectified Bonn 输入，避免同时改变光流后端与坐标域。

## 6. 执行顺序

### S0-A：30 帧功能 smoke

使用 TUM fr3/walking 的前 30 个 association。CPU 与 GPU 各运行一次，只检查：

- 退出码 0；
- 处理 30 帧；
- 第 1--29 帧进入 detector；
- 无 CUDA、NaN、内存或线程错误；
- 轨迹文件产生；
- mask 尺寸、类型和值域正确；
- GPU 显存没有持续逐帧增长。

本步不评价 ATE。

### S0-B：331 帧行为 smoke

GPU 从帧 0 运行到 330，只保存 270--330 的 mask/labels/overlay。CPU 复用现有 `ab_official`。

比较：

- internal/final dynamic ratio；
- 全零帧数和最大覆盖；
- region 数量；
- 动态 ORB 数量；
- CPU/GPU overlay；
- mask IoU，只作为行为差异诊断，不作为等价要求。

### S0-C：完整 CPU/GPU 对照

只有 S0-A/B 通过后才执行。四条固定序列中：

- detector-on：CPU/GPU 各三次；已有 CPU 单次可作为 run-1；
- detector-off：GPU 每序列先跑一次，用于识别更换 OpenCV 本身造成的 ORB-SLAM2 差异；
- 若 GPU detector-off 与 CPU baseline 的差异达到 detector-on 收益量级，再扩展同后端 baseline 重复次数。

报告 ATE、RPE、跟踪完整性、mask、detector time、tracking time、wall time、实际 FPS、峰值 CPU/GPU 内存，并报告中位数与范围。

## 7. 停止条件

立即停止进入下一步的情况：

- 二进制混入两套 OpenCV；
- 没有实际启用 `cudaoptflow` 或 Brox；
- CUDA device count 为 0；
- smoke 崩溃、轨迹缺失、帧数不符或显存持续增长；
- GPU mask 持续全零或接近整图覆盖；
- GPU non-obstructing 多次运行不能优于同后端 detection-off baseline；
- 静态 fr1/xyz 出现持续跟踪失败或超出同后端 baseline 波动的明显退化。

GPU detector 没有明显提速不等于区域方法无效；它只会否定“Brox GPU 已解决性能问题”的结论。

## 8. 源码范围

本阶段只允许在独立 GPU worktree 做以下兼容/诊断改动：

- CMake CUDA 开关可由 cache 控制；
- OpenCV include 使用标准路径；
- 显式输出 OpenCV 版本、flow backend 和 CUDA device count；
- 必要的独立构建路径调整。

不修改 `DynaDetect` 的聚类、阈值、光流参数、时序、区域判决和形态学逻辑；不修改 DT-SLAM。
