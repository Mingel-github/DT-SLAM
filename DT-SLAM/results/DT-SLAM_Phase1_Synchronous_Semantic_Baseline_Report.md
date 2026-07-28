# DT-SLAM Phase 1：同步语义 Baseline 修复与实时化报告

日期：2026-07-24  
工作区：`/home/zhu/dynaslam_ws/DT-SLAM`  
分支：`main`  
冻结提交：`b8f2a422c47601588ac421e5b14194a9fa0f3946`  
提交标题：`Stabilize synchronous CUDA semantic baseline`

## 1. 阶段目标

本阶段只建立一个正确、可测量、接近实时的同步 semantic baseline：

```text
RGB-D
→ YOLOv8n-seg
→ 严格同帧语义 mask
→ 动态特征过滤
→ ORB-SLAM2 Tracking
```

本阶段不引入：

- 几何动态检测；
- async 语义；
- 后端动态地图；
- Optimizer 修改；
- 输入分辨率降低；
- FP16 或 TensorRT；
- 检测框替代分割 mask。

## 2. 语义处理正确性

在前一阶段语义动态特征清理的基础上，本阶段继续确认并保持：

- RGB 与 semantic mask 使用相同帧序号；
- Tracking 不使用过期 mask；
- mask age 始终为 0；
- 语义动态特征不参与关键匹配、优化和地图点准入；
- RGB-D 初始化使用静态特征数量判断；
- `NeedNewKeyFrame()` 的近距离点统计忽略动态特征；
- `Frame` 与 `KeyFrame` 的 BoW 策略保持一致；
- 无语义模式仍可作为纯 ORB-SLAM2 baseline 运行。

最终 150 帧语义测试：

```text
mask ready：150/150
mask age median：0
mask age max：0
```

## 3. CUDA 推理接入

YOLOv8n-seg 已从 CPU ONNX Runtime 切换到 RTX 4060 CUDA：

- 模型输入保持 `640×640`；
- 显式注册 `CUDAExecutionProvider`；
- 输出当前可用 Provider；
- 指定 CUDA device 0；
- CUDA 不可用时直接报错；
- 禁止静默回退 CPU；
- TensorRT 虽然可用，但本阶段未启用。

运行日志确认：

```text
TensorrtExecutionProvider
CUDAExecutionProvider
CPUExecutionProvider

Semantic provider: CUDAExecutionProvider (device 0)
```

当前 GPU ORT 来自 Python `onnxruntime-gpu 1.23.2`。运行时依赖
`LD_PRELOAD` 和 CUDA/cuDNN 的 `LD_LIBRARY_PATH`。CMake 已支持配置
`ONNXRUNTIME_ROOT`，但 GPU C++ 运行时依赖尚未打包进仓库。

## 4. 性能计时体系

### 4.1 YOLO 内部统计

- preprocess；
- ONNX CUDA execution；
- postprocess；
- semantic total；
- mean、median、P95、min、max。

### 4.2 RGB-D 端到端统计

- image IO；
- `GetMaskSeq`；
- frame mutex wait；
- frame copy；
- mask wait；
- semantic block；
- Tracking；
- active total；
- sleep；
- frame wall；
- pacing error；
- deadline overrun；
- actual FPS。

所有统计在序列结束后聚合输出，没有逐帧打印干扰运行。

## 5. 严重并发问题及修复

初始完整测试出现：

```text
实际 FPS：0.177
859 帧序列时间：4841.55 秒
帧提交中位数：3573.5 ms
帧提交 P95：19222 ms
```

代码检查发现，YOLO 工作线程没有新帧时，会持有 `mMutexFrame`
睡眠 2 ms，然后立即重新竞争锁。这会导致主线程在 `PushFrame()`
中长期得不到锁。

该问题来自后加的 YOLO 异步语义线程，不属于原生 ORB-SLAM2，也不是
CUDA 改造引入的。

修复过程：

1. 将 sleep 移出 `mMutexFrame` 锁作用域；
2. 拆分 mutex wait 与图像复制计时；
3. 最终用条件变量替代轮询。

修复后：

```text
frame mutex wait median：0.000080 ms
frame copy median：0.0725 ms
frame submit median：0.0751 ms
```

帧提交区间相比修复前缩短约 5.9 万倍。

## 6. TUM 节拍修正

初始 ORB-SLAM2 示例只使用 `TrackRGBD()` 的时间计算 sleep：

```text
sleep = 数据集帧间隔 - Tracking
```

图像读取没有计入活动时间。本机 image IO 约为 8.5–9 ms，因此实际帧
周期会额外增加约 9 ms。

修改为：

```text
active total =
    image IO
  + semantic
  + Tracking

sleep = 数据集帧间隔 - active total
```

节拍修正效果：

| 指标 | 修正前 | 修正后 |
|---|---:|---:|
| 实际 FPS | 23.50 | 28.98 |
| frame wall median | 41.24 ms | 34.26 ms |
| pacing error median | 9.05 ms | 0.27 ms |

这个节拍问题已存在于初始 ORB-SLAM2 示例，不是语义模块引入；但原有
写法不符合本工程采用的端到端 30 FPS 定义。

## 7. 条件变量优化

将两处 2 ms 轮询替换为条件变量：

```text
PushFrame
→ notify 推理线程

mask 推理完成
→ notify Tracking 主线程
```

同时在 `Stop()` 中唤醒所有等待线程，避免退出死锁。

改进效果：

| 指标 | 轮询版本 | 条件变量 |
|---|---:|---:|
| semantic block median | 14.63 ms | 13.42 ms |
| active total median | 33.48 ms | 32.17 ms |
| 实际 FPS | 28.83 | 29.45 |
| deadline miss | 89/150 | 54/150 |
| mask age | 0 | 0 |

条件变量将 YOLO 内部计算与主线程语义阻塞之间的调度差值，从约
2.1 ms 降至约 0.3 ms。

## 8. 最终 150 帧对照

测试序列：TUM `fr3_walking_xyz` 前 150 帧。

| 指标 | 无语义 baseline | 同步语义 CUDA |
|---|---:|---:|
| 实际 FPS | 29.74 | 29.45 |
| 数据集 FPS | 29.85 | 29.85 |
| image IO median | 8.45 ms | 8.55 ms |
| Tracking median | 12.13 ms | 10.07 ms |
| active total median | 20.66 ms | 32.17 ms |
| deadline miss | 0/150 | 54/150 |
| mask age | — | 0 |

语义模式的净 active 开销约为：

```text
32.17 - 20.66 = 11.51 ms
```

同步语义模式达到无语义 baseline 实际 FPS 的约 99%。

语义模式的 Tracking 在本次短测中比无语义少约 2 ms，这是实测结果；
可能与过滤后参与处理的特征或地图点减少有关，但单次短序列不足以确认
具体因果关系。

初始地图点数分别为：

- 无语义：734；
- 语义：433。

该差异证明两种模式的数据准入不同，但不能单凭点数判断地图质量。

## 9. 当前性能判断

已经确认：

- CUDA 推理正常；
- 同帧语义正确；
- 数秒级锁阻塞已消除；
- 节拍错误已修正；
- 条件变量无死锁；
- 150 帧达到 29.45 FPS；
- Pangolin 可视化保持开启。

仍需注意：

- semantic active total 中位数为 32.17 ms；
- P95 为 33.35 ms；
- 距离 33.3 ms 预算几乎没有余量；
- 新增几何模块后可能低于 30 FPS；
- 尚未在修复后重新跑完整 859 帧；
- 150 帧结果不能替代最终完整数据集实验；
- ONNX Runtime 1.23.2 头文件在 C++14 下仍有 C++17 相关编译警告。

## 10. 修改文件

冻结提交只包含：

```text
DT-SLAM/CMakeLists.txt
DT-SLAM/Examples/RGB-D/rgbd_tum.cc
DT-SLAM/include/YOLOSegment.h
DT-SLAM/src/Frame.cc
DT-SLAM/src/KeyFrame.cc
DT-SLAM/src/Tracking.cc
DT-SLAM/src/YOLOSegment.cc
```

没有提交：

- BONN 数据集；
- 轨迹文件；
- 测试日志；
- `results/experiments.md` 的现有未提交修改；
- 模型权重；
- 其他用户文件。

## 11. 冻结状态

本地冻结提交：

```text
b8f2a422c47601588ac421e5b14194a9fa0f3946
Stabilize synchronous CUDA semantic baseline
```

当前状态：

```text
branch：main
相对 origin/main：ahead 1
GitHub：尚未推送
本地代码：权威版本
```

本阶段已经完成。当前同步 semantic baseline 可作为后续几何模块的稳定
起点，async 暂时搁置。

## 12. 相关日志

- `results/phase1_baseline_no_semantic_150.log`
- `results/phase1_condition_variable_150.log`
- `results/phase1_mutex_fix_30.log`
- `results/phase1_pacing_fix_30.log`
- `results/phase1_pacing_fix_150.log`

