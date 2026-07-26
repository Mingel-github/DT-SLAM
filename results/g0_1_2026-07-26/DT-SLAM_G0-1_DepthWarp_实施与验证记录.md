# DT-SLAM G0-1 Depth Warp 实施与验证记录

> 日期：2026-07-26  
> 工作区：`/home/zhu/dynaslam_ws`  
> 工程：`/home/zhu/dynaslam_ws/DT-SLAM`  
> 分支：`main`  
> 本地提交：`7c65d74 Add G0-1 geometry shadow depth warp`  
> 基线提交：`b8f2a42 Stabilize synchronous CUDA semantic baseline`  
> 状态：G0-1 已完成；G0-2 尚未开始

---

## 1. 本阶段边界

G0-1 只实现单参考帧几何不一致测量：

```text
reference depth
current depth
Tcw_reference
Tcw_current
K
    ↓
single-reference forward depth warp
    ↓
z-buffer
    ↓
predicted_depth
valid_comparison_mask
signed_depth_residual
```

采用的相对变换为：

\[
T_{t\leftarrow r}
=
T_{cw,t}T_{cw,r}^{-1}.
\]

有符号深度残差为：

\[
r_z
=
D_{\mathrm{pred}}-D_t.
\]

本阶段明确没有实现：

```text
不生成 positive_seed_mask
不生成 negative_mask
不进行区域生长
不标记 ORB 特征动态状态
不修改 mvbDynamic
不删除 mvpMapPoints
不增加 PoseOptimization
不阻止 MapPoint 创建
不修改 YOLO、Optimizer、LocalMapping 或 LoopClosing
```

---

## 2. 准确接入方式

当前 `Frame` 不保存整张 RGB-D 深度图，因此：

1. `Tracking::GrabImageRGBD()` 完成 TUM 深度尺度转换后，将当前 `CV_32FC1` 米制深度提供给 G0-1；
2. `TrackWithMotionModel()` 或 `TrackReferenceKeyFrame()` 完成已有的初始位姿估计后；
3. 在进入 `TrackLocalMap()` 前运行只读 G0-1；
4. 当前帧最终跟踪成功后，使用最终 `Tcw` 更新下一帧参考；
5. 跟踪失败、系统 Reset 或无有效位姿时清空参考；
6. 若存在语义 mask，参考深度中的语义动态像素设置为无效，避免将已知动态对象保存成静态参考。

G0-1 运行会增加 Tracking 线程耗时，因此即使它不修改 SLAM 数据，也可能改变多线程调度。当前不能声称开启和关闭后的轨迹会逐字节一致。

---

## 3. 修改文件

| 文件 | 作用 |
| --- | --- |
| `include/GeometricDynamicDetector.h` | G0-1 输入输出、统计结构和最小接口 |
| `src/GeometricDynamicDetector.cc` | 深度 warp、z-buffer、valid mask 和 signed residual |
| `Examples/RGB-D/geometric_warp_test.cc` | 确定性合成测试 |
| `include/Tracking.h` | G0-1 shadow 状态和接入声明 |
| `src/Tracking.cc` | 初始位姿后调用及成功帧参考更新 |
| `Examples/RGB-D/TUM3.yaml` | 默认关闭的 G0-1 配置 |
| `CMakeLists.txt` | 编译几何模块和测试程序 |
| `.gitignore` | 排除测试可执行文件 |

默认配置：

```yaml
Geometry.Enable: 0
Geometry.LogEveryN: 30
```

因此普通 baseline 和 semantic 命令默认不执行几何计算。

---

## 4. 合成测试

测试命令：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

export LD_LIBRARY_PATH="\
/home/zhu/dynaslam_ws/pangolin_install/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/thirdparty/onnxruntime/lib:\
${LD_LIBRARY_PATH:-}"

./Examples/RGB-D/geometric_warp_test
```

结果：

```text
[Geometry G0-1 Test] PASS
```

覆盖五项：

1. 单位位姿静态平面残差为零；
2. 当前近表面产生正残差；
3. 验证 `Tcw_current * inverse(Tcw_reference)` 的变换方向；
4. 无效当前深度保持 `UNKNOWN`，不产生残差；
5. 多个参考点投影到同一像素时，z-buffer 保留最近表面。

---

## 5. TUM 短跑条件

```text
数据集：TUM fr3_walking_xyz
范围：associations.txt 前 50 行
图像：640×480
语义：关闭
Viewer：关闭
运行模式：分别运行 G0-1 disabled 和 enabled
输出：临时目录，未覆盖工程中的既有轨迹
```

这是两次独立 ORB-SLAM2 运行。由于随机性和多线程调度，跨运行差值只能作为工程参考；G0-1 内部计时是更直接的模块开销证据。

---

## 6. G0-1 最终代码内部统计

有效 G0-1 计算帧数为 49，首帧只用于建立参考。

| 指标 | mean | median | P95 | min | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| `total_ms` | 2.939 | 2.931 | 3.216 | 2.607 | 3.273 |
| `warp_ms` | 2.058 | 2.063 | 2.179 | 1.903 | 2.236 |
| `residual_ms` | 0.799 | 0.708 | 0.964 | 0.645 | 0.988 |
| predicted coverage | 71.03% | 71.05% | 73.52% | 67.66% | 74.07% |
| comparison coverage | 68.98% | 69.07% | 71.69% | 65.18% | 71.99% |
| 帧级 mean absolute residual | 0.0611 m | 0.0583 m | 0.0984 m | 0.0167 m | 0.1045 m |
| 帧级 signed residual mean | -0.0037 m | -0.0056 m | 0.0371 m | -0.0483 m | 0.0438 m |

客观解释：

- 单参考帧 forward warp 在该片段上形成约 71% 的预测覆盖；
- 当前有效深度与预测深度共同有效的比较区域约为 69%；
- G0-1 的直接计算开销约为 2.94 ms/帧，P95 为 3.22 ms；
- signed residual 的帧级均值接近零，不代表每个静态像素都接近零；
- mean absolute residual 仍混合位姿误差、深度噪声、遮挡、运动物体和投影取整；
- 这些数据不足以确定 G0-2 的正残差阈值。

---

## 7. G0 开关对照

| 指标 | G0 disabled | G0 enabled | 独立运行差值 |
| --- | ---: | ---: | ---: |
| tracking mean | 11.598 ms | 14.667 ms | +3.069 ms |
| tracking median | 11.328 ms | 14.477 ms | +3.149 ms |
| tracking P95 | 15.232 ms | 15.927 ms | +0.695 ms |
| active total mean | 20.028 ms | 24.043 ms | +4.015 ms |
| actual FPS | 29.685 | 29.541 | -0.144 |
| deadline miss | 0/50 | 1/50 | +1 |

不能根据单次独立对照认定该 deadline miss 一定由 G0-1 导致。可以确认的是：

- G0-1 内部计时和 tracking mean 的增量处于同一量级；
- 50 帧短片段仍接近 TUM 30 FPS；
- 当前同步语义模式的剩余预算尚未在本阶段重新测量；
- 尚不能声称“语义＋G0-1”也满足 30 FPS。

---

## 8. 编译命令

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
cmake ..
make geometric_warp_test rgbd_tum -j$(nproc)
```

结果：编译成功。

构建中仍能看到工程原有的 Eigen deprecated 和 ONNX Runtime C++17 相关警告；本阶段没有修改这些依赖或编译标准。

---

## 9. 50 帧复现命令

先在测试配置中临时设置：

```yaml
Geometry.Enable: 1
Geometry.LogEveryN: 1
```

然后执行：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

export LD_LIBRARY_PATH="\
/home/zhu/dynaslam_ws/pangolin_install/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/lib:\
/home/zhu/dynaslam_ws/DT-SLAM/thirdparty/onnxruntime/lib:\
${LD_LIBRARY_PATH:-}"

G0_RUN_DIR=$(mktemp -d /tmp/dtslam_g0_1_reproduce.XXXXXX)
cd "$G0_RUN_DIR"

DT_SLAM_DISABLE_VIEWER=1 \
/home/zhu/dynaslam_ws/DT-SLAM/Examples/RGB-D/rgbd_tum \
  /home/zhu/dynaslam_ws/DT-SLAM/Vocabulary/ORBvoc.txt \
  /home/zhu/dynaslam_ws/DT-SLAM/Examples/RGB-D/TUM3.yaml \
  /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz \
  <(head -n 50 \
    /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz/associations.txt)
```

测试结束后应恢复：

```yaml
Geometry.Enable: 0
Geometry.LogEveryN: 30
```

当前仓库已经恢复为上述默认状态。

---

## 10. 尚未完成和已知限制

```text
尚未运行 G0-1 + 同步 CUDA semantic 的短时组合计时
尚未在 TUM sitting_static 上验证静态残差分布
尚未生成残差图或 overlay
尚未实现正/负残差 mask
尚未选择深度残差阈值
尚未进行区域生长
尚未统计 ORB 特征位置的几何证据
尚未在 Bonn 动态箱子序列验证类别无关运动证据
```

当前 `sitting_static` 数据目录没有现成 `associations.txt`，因此本阶段没有伪造或临时猜测关联关系。

---

## 11. 阶段结论

G0-1 已满足以下工程验收：

```text
变换方向的确定性测试通过
米制深度输入链路通过
z-buffer 测试通过
invalid depth 保持 unknown
真实 TUM 数据可稳定运行
单参考帧覆盖率已测量
CPU 运行开销已测量
默认关闭时不进入几何路径
未修改 SLAM 动态状态和优化器
```

因此：

> G0-1 可以作为已完成的几何测量基础，但当前证据不足以直接冻结 G0-2 阈值，也不能把 signed residual 直接解释为最终动态检测结果。

下一步应先检查这份记录，再决定是否进入：

```text
G0-2 Evidence
=
positive seed
+
negative diagnostic mask
+
valid / unknown evidence semantics
```

---

## 12. 原始日志校验

```text
G0 enabled:
e13b694f109a2e02ae15f3452348381c95e318b28c640b9d32f2ed46d8b89820

G0 disabled:
f5b4e86a55dbdbaecf19abb2475d18a8560f8edfe9c7b4e5b2d40ecc38e20616
```

日志文件与本记录一起保存在桌面。

