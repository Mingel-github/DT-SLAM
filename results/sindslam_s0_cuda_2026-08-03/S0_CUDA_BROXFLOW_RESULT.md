# S0 CUDA/BroxFlow 配置忠实性与完整对照结果

日期：2026-08-03

## 1. 阶段结论

S0 的构建、功能与对照实验已完成，但总门只 **部分通过**。

独立 SInDSLAM 已在隔离环境中真实启用 OpenCV 4.5.4 CUDA BroxFlow，并在 RTX 4060 Ti 上完成：

- 30 帧功能冒烟；
- Bonn 331 帧 CPU/GPU 同帧 mask 行为审计；
- 四条固定序列的 CPU DeepFlow 与 GPU BroxFlow detector-on 三轮完整运行；
- GPU/OpenCV 同后端 detector-off 对照；
- 统一 ATE、RPE、运行时间和内存统计。

S0-A 功能门和 S0-B 行为门通过；S0-C 在 fr3/walking、Bonn non-obstructing 和 fr1/xyz 上通过完整性检查，但在 Bonn obstructing 上未通过轨迹覆盖门。GPU detector-on 三轮都存在较长 LOST 区间，CPU 第三轮也发生一次明显丢失。

这不否定区域路线，也不阻止继续做 **S1 shadow 接入**，但 GPU Brox 不能被当作唯一的“行为真值”，更不能直接启用 DT-SLAM 的真实过滤。S1 必须同时保留 CPU DeepFlow 与 GPU Brox 对照，并继续 shadow-only。

本轮没有修改 DT-SLAM，没有修改 SInDSLAM 的聚类、阈值、Brox 参数、时序先验、区域判决或形态学逻辑。

## 2. 隔离工具链与身份核验

### 2.1 工具链

- GPU：NVIDIA GeForce RTX 4060 Ti；
- 驱动：595.84；
- CUDA Driver / Runtime：13.2 / 11.8；
- 隔离 CUDA Toolkit：`/data/dynaslam/toolchains/cuda-11.8`；
- 隔离 OpenCV：`/data/dynaslam/toolchains/opencv-4.5.4-cuda`；
- OpenCV CUDA 模块包含 `cudaoptflow`、`cudalegacy`、`cudawarping`、`cudaimgproc` 和 `cudaarithm`；
- 独立 GPU worktree：`/data/dynaslam/SInDSLAM_cuda`。

构建过程中确认：OpenCV 4.5.4 的 `BroxOpticalFlow::create()` 还依赖 `cudalegacy`。只有 `cudaoptflow` 时可以链接，但运行会进入 `throw_no_cuda`；补建 `cudalegacy` 后，同一独立 smoke 正常输出 `CV_32FC2` flow。

### 2.2 可执行文件身份

最终 GPU runner 满足：

- 编译定义包含 `USECUDA`；
- 链接到隔离的 `opencv_cudaoptflow` 与 `opencv_cudalegacy`；
- `ldd` 中 OpenCV 全部来自隔离前缀，没有混入系统 OpenCV；
- Pangolin 动态库来自 `/home/zhu/dynaslam_ws/pangolin_install/lib`；
- 运行日志明确输出：

```text
[SIn S0] OpenCV version: 4.5.4
[SIn S0] Flow backend: Brox_CUDA
[SIn S0] CUDA device count: 1
Device 0: "NVIDIA GeForce RTX 4060 Ti" ... sm_89
```

独立两帧 Brox smoke 对人工平移 `(3, 2)` 像素输出平均 flow 约 `(3.17, 2.47)` 像素，证明不是只完成编译而未执行 CUDA flow。

## 3. S0-A：30 帧功能冒烟

输入：TUM `fr3/walking_xyz` 前 30 帧。

结果：

- 退出码 0；
- 处理 30 帧；
- `CameraTrajectory.txt` 为 30 行；
- 第 1--29 帧均进入 detector；
- 29 张最终 mask 均非全图；
- 29 张最终 mask 均为非零输出；
- dynamic ratio 范围约 4.81%--40.49%；
- mean detector time 为 95.3 ms（留档重跑）；
- 未发生 CUDA、NaN、线程、轨迹或显存耗尽错误。

该测试只证明执行链有效，不评价 mask 精度或 ATE。

## 4. S0-B：331 帧 CPU/GPU 行为审计

输入：Bonn `moving_nonobstructing_box`，从帧 0 运行至 330，保存 270--330 共 61 帧。

| 指标 | CPU DeepFlow | GPU BroxFlow |
|---|---:|---:|
| final mask 平均覆盖 | 15.29% | 11.45% |
| final mask 中位覆盖 | 15.95% | 9.32% |
| 最大覆盖 | 58.60% | 34.90% |
| 全零帧 | 8/61 | 19/61 |
| 全图帧 | 0/61 | 0/61 |

CPU/GPU paired mask：

- mean IoU：0.509；
- median IoU：0.586；
- 面积相关系数：0.571。

这说明 Brox 与 DeepFlow 的行为不是逐像素等价，但仍属于活跃且非退化的动态区域输出。代表帧 306、313 中 GPU mask 对人和箱子更集中；帧 322 仍出现静态显示器/桌面区域误标。该观察只作定性审计，不等同于像素 GT。

## 5. S0-C：完整轨迹结果

### 5.1 协议

- ATE：`evo_ape tum ... -va --align --t_max_diff 0.02`；
- RPE：`evo_rpe tum ... -va --align --delta 1 --delta_unit f --t_max_diff 0.02`；
- viewer 关闭；
- detector-on：CPU 与 GPU 各三轮；
- CPU run-1 复用 2026-08-02 已验证轨迹，run-2/3 本轮新跑；
- GPU detector-off：四序列 run-1；因强遮挡后端差异较大，额外补 obstructing 与 fr1 run-2/3；
- 下表为 RMSE 中位数 `[最小值, 最大值]`，单位为米。

### 5.2 Detector-on 三轮

| 序列 | CPU ATE | GPU ATE | CPU RPE | GPU RPE |
|---|---:|---:|---:|---:|
| TUM fr3/walking | 0.014852 `[0.014527, 0.015168]` | 0.014234 `[0.013834, 0.015884]` | 0.011856 `[0.011780, 0.012543]` | 0.011671 `[0.011617, 0.012047]` |
| Bonn non-obstructing | 0.023441 `[0.023097, 0.024608]` | 0.024507 `[0.021867, 0.024987]` | 0.014917 `[0.014681, 0.015007]` | 0.013857 `[0.013777, 0.014401]` |
| Bonn obstructing | 0.274588 `[0.142097, 0.294705]` | 0.250128 `[0.146623, 0.253200]` | 0.024518 `[0.017561, 0.062836]` | 0.051701 `[0.050086, 0.082254]` |
| TUM fr1/xyz | 0.009948 `[0.009687, 0.009956]` | 0.009647 `[0.009633, 0.009922]` | 0.005829 `[0.005712, 0.005851]` | 0.005725 `[0.005715, 0.005746]` |

### 5.3 Tracking 完整性

runner 正常处理完全部输入，不等于每一帧都成功跟踪。ORB-SLAM2 的 TUM trajectory writer 会跳过 LOST 帧，因此轨迹行数必须单独审计。

| 序列 | CPU detector-on | GPU detector-on | GPU detector-off |
|---|---:|---:|---:|
| fr3/walking | 823/823，三轮均完整 | 823/823，三轮均完整 | 823/823 |
| Bonn non-obstructing | 778/778，三轮均完整 | 778/778，三轮均完整 | 778/778 |
| Bonn obstructing | 589/589、589/589、417/589 | 286/589、336/589、341/589 | 589/589，三轮均完整 |
| fr1/xyz | 796/796，三轮均完整 | 796/796，三轮均完整 | 796/796，三轮均完整 |

obstructing 详细 LOST 结构：

- CPU run-3：丢失 172 帧，最长连续 LOST 为帧 360--469，共 110 帧；
- GPU run-1：丢失 303 帧，连续 LOST 为帧 172--474；
- GPU run-2：丢失 253 帧，最长连续 LOST 为帧 243--355，共 113 帧；
- GPU run-3：丢失 248 帧，最长连续 LOST 为帧 229--475，共 247 帧。

因此 obstructing 的 ATE/RPE 只是在成功输出的轨迹子集上计算。尤其 GPU 三轮的 ATE 不能与完整 589 帧 baseline 作直接全序列精度比较；其 RPE 也受轨迹缺口影响，只保留为失败诊断值。

客观解释：

1. GPU Brox 在 walking 和 non-obstructing 上保持了 CPU DeepFlow 的轨迹质量，没有因换后端而失去主要收益。
2. obstructing 的 CPU/GPU 都出现运行间不稳定；GPU 三轮均未保持全序列跟踪，因此不能用较低的子集 ATE 宣称精度更高或轨迹得到完整改善。
3. obstructing 是 S0-C 的明确失败项。其 GPU RPE 偏高与大段 LOST 共同表明当前 Brox mask 在该场景中不能直接作为可靠 Tracking 过滤结果。
4. fr1/xyz 中 CPU/GPU detector-on 都保持完整跟踪，差异在亚毫米 ATE 量级；这不证明 mask 完美，只说明当前序列未出现明显静态轨迹退化。

### 5.4 GPU 同后端 detector-off

| 序列 | detector-on GPU ATE | detector-off GPU ATE | detector-on GPU RPE | detector-off GPU RPE |
|---|---:|---:|---:|---:|
| TUM fr3/walking | 0.014234（三轮中位） | 1.103275（单轮） | 0.011671 | 0.025476 |
| Bonn non-obstructing | 0.024507（三轮中位） | 0.319597（单轮） | 0.013857 | 0.026510 |
| Bonn obstructing | 0.250128（三轮中位） | 0.501677 `[0.479069, 0.607752]` | 0.051701 | 0.023296 `[0.020192, 0.023652]` |
| TUM fr1/xyz | 0.009647（三轮中位） | 0.009577 `[0.009351, 0.009669]` | 0.005725 | 0.005701 `[0.005588, 0.005736]` |

因此可以成立的结论是：

- 区域检测在 walking 与 non-obstructing 上产生稳定、显著的 ATE/RPE 收益；
- obstructing 的成功轨迹子集 ATE 较低，但 detector-on 覆盖只有 48.6%--57.9%；不能称为全序列 ATE 改善。相反，完整性门失败是更优先的结论；
- fr1/xyz 的 detector-on/off 差异很小，on 略差，不能写成静态场景零代价；
- ATE 改善不能替代 mask/地图质量评价，也不能证明所有 SIn 组件都不可缺少。

## 6. 性能

### 6.1 Detector time

mean detector time 中位数：

| 序列 | CPU DeepFlow | GPU BroxFlow | 比值 CPU/GPU |
|---|---:|---:|---:|
| fr3/walking | 194.7 ms | 88.0 ms | 2.21x |
| Bonn non-obstructing | 187.3 ms | 101.5 ms | 1.85x |
| Bonn obstructing | 221.8 ms | 100.3 ms | 2.21x |
| fr1/xyz | 246.2 ms | 105.2 ms | 2.34x |

这是不同实现后端的对照：CPU 使用 DeepFlow，GPU 使用 BroxFlow，不能解释为“同一算法纯 GPU 加速”。GPU 后端仍包含 CPU K-means、边缘、RAG、区域合并、flow 下载和 VariationalRefinement。

### 6.2 完整命令墙钟

墙钟包含 ORB 词典加载、系统初始化、全序列处理与保存。CPU 取新跑 run-2/3，GPU 取三轮中位数。

| 序列 | CPU 墙钟 | GPU 墙钟 | CPU 实际 FPS | GPU 实际 FPS |
|---|---:|---:|---:|---:|
| fr3/walking，823 帧 | 198.9 s | 111.1 s | 4.14 | 7.41 |
| Bonn non-obstructing，778 帧 | 180.3 s | 113.9 s | 4.32 | 6.83 |
| Bonn obstructing，589 帧 | 157.7 s | 86.6 s | 3.73 | 6.80 |
| fr1/xyz，796 帧 | 232.1 s | 120.2 s | 3.43 | 6.63 |

GPU 提速明确，但仍不满足 30 FPS。后续不能以“已有 RTX 4060 Ti”推断完整区域路线实时。

CPU 峰值主存约 0.69--0.75 GB，GPU 进程峰值主存约 0.92--0.98 GB。没有观察到崩溃或逐序列持续增长，但本轮未采集逐帧显存曲线，因此不声称已经严格证明无 GPU 内存增长。

## 7. 兼容性修改范围

只在独立 CUDA worktree 中做：

- 将硬编码 CUDA 开关改为 CMake option；
- 将 OpenCV include 改为标准路径，防止回落系统头文件；
- 删除未使用且会要求未构建 `opencv_ml` 的 namespace 声明；
- runner 输出 OpenCV、flow backend 和 CUDA device；
- runner 支持 viewer 关闭与最大帧数诊断。

未修改：

- `DynaDetect` 的算法参数与主要逻辑；
- ORB-SLAM2 Optimizer、g2o、LocalMapping 和 LoopClosing；
- DT-SLAM 源码；
- YOLO 模型与推理代码。

## 8. 风险与解释边界

1. GPU Brox 与 CPU DeepFlow mask 不等价，S1 不能同时更换 flow、区域规则和融合方式后再宣称复现。
2. obstructing 中 detector-on 的成功子集 ATE 较低、RPE 较高，但由于轨迹不完整，不能解释为“全局误差下降”；轨迹覆盖失败优先于这两个数值。
3. mask 仍存在静态区域误标；项目不要求像素完美，但必须保持足够静态内点和地图完整度。
4. 公开 tracking mask 不等于论文完整建图链；本轮没有验证论文的长间隔 depth refinement 或 OctoMap 输出。
5. `evo` 执行成功，但环境打印 SciPy 1.8.0 与 NumPy 1.26.4 版本范围警告；正式论文表格前应在干净评价环境复算。
6. SInDSLAM 公开仓库许可证尚需最终确认；现阶段只独立运行、审计和重新设计接口，不直接复制大段源码进入 DT-SLAM。
7. obstructing 的 detector-on 轨迹缺失是当前最严重风险；后续任何评价必须同时报告 ATE/RPE 与 tracked-frame coverage。

## 9. 下一步

可以进入 S1，但只能继续 shadow-only；S0-C 的 obstructing 失败禁止直接进入 S2：

```text
DT-SLAM RGB-D 输入
→ SIn 风格 region labels / geometry mask
→ 同帧与独立 SIn GPU runner 对照
→ 动态 ORB 数量、mask 面积、区域标签与耗时审计
→ 不写 mvbDynamic，不清 mvpMapPoints，不阻止 MapPoint
```

S1 的第一版保持行为忠实：不同时替换 homography 为 SE(3)，不加入 YOLO OR 融合，不新增阈值。CPU DeepFlow 作为当前更稳定的行为参照，GPU Brox 作为性能与失败对照。只有同帧输出可解释、且 obstructing 的静态观测保留风险得到说明后，才重新审批 S2 的 semantic-only、geometry-only、semantic OR geometry Tracking 对照。
