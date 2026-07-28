# DT-SLAM G0-2 Evidence 实施与验证记录

> 日期：2026-07-26  
> 工作区：`/home/zhu/dynaslam_ws`  
> 基线：`4d44f62 Document G0-1 validation results`  
> 阶段：G0-2 Evidence  
> 性质：只读影子证据，不影响 SLAM

---

## 1. 阶段边界

G0-2 在 G0-1 的有效深度比较和 signed residual 上增加证据分类：

```text
UNKNOWN
    validComparisonMask == 0

CONSISTENT_EVIDENCE
    valid != 0
    && |residual| <= threshold

POSITIVE_SEED
    valid != 0
    && residual > threshold

NEGATIVE_DIAGNOSTIC
    valid != 0
    && residual < -threshold
```

当前临时配置：

```yaml
Geometry.ResidualThresholdM: 0.10
```

该数值只是首轮诊断参数，不是冻结阈值，也不是论文结论。

G0-2 没有实现：

```text
不做区域生长
不做形态学膨胀
不采样 ORB 特征状态
不融合语义与几何动态状态
不修改 mvbDynamic
不清除 mvpMapPoints
不调用额外 PoseOptimization
不影响 MapPoint 或 KeyFrame 写入
```

---

## 2. 新增输出

`GeometricWarpResult` 新增：

```cpp
cv::Mat consistentEvidenceMask; // CV_8UC1
cv::Mat positiveSeedMask;       // CV_8UC1
cv::Mat negativeDiagnosticMask; // CV_8UC1
```

所有 mask 均使用：

```text
0   = 不属于该证据类
255 = 属于该证据类
```

三类证据只允许出现在 `validComparisonMask != 0` 的位置。

其中：

- `consistentEvidenceMask` 只表示当前深度与参考预测在阈值内一致；
- 它不能证明像素所属对象在语义或长期意义上静态；
- `positiveSeedMask` 是后续几何动态候选；
- `negativeDiagnosticMask` 只用于分析背景显露、物体移开或几何误差，不作为当前动态前景。

---

## 3. 不变量

每帧必须满足：

\[
N_{\mathrm{consistent}}
+
N_{\mathrm{positive}}
+
N_{\mathrm{negative}}
=
N_{\mathrm{valid}}.
\]

同时：

```text
UNKNOWN 像素不进入任何证据 mask
阈值边界 residual == ±threshold 归入 consistent
valid 像素的 residual 必须是有限值
三张证据 mask 互斥
```

---

## 4. 修改文件

| 文件 | 改动 |
| --- | --- |
| `include/GeometricDynamicDetector.h` | 增加三张证据 mask、统计字段和阈值接口 |
| `src/GeometricDynamicDetector.cc` | 增加 `ClassifyEvidence()` |
| `src/Tracking.cc` | 读取临时阈值并输出 G0-2 统计 |
| `include/Tracking.h` | 将注释更新为通用 G0 shadow 状态 |
| `Examples/RGB-D/TUM3.yaml` | 增加临时 `Geometry.ResidualThresholdM` |
| `Examples/RGB-D/geometric_warp_test.cc` | 增加 G0-2 确定性测试 |

未修改 YOLO、Optimizer、Frame、MapPoint、KeyFrame、LocalMapping 和 LoopClosing。

---

## 5. 合成测试

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
make geometric_warp_test rgbd_tum -j$(nproc)
```

测试：

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
[Geometry G0-2 Test] PASS
```

新增验证：

1. 正残差超过阈值只进入 `positiveSeedMask`；
2. 负残差超过阈值只进入 `negativeDiagnosticMask`；
3. 无效深度保持 unknown；
4. `residual == ±threshold` 归入 consistent；
5. 三类证据像素数之和等于 valid comparison 数。

---

## 6. TUM walking 50 帧

条件：

```text
序列：fr3_walking_xyz
帧数：前 50 个 RGB-D association
语义：关闭
Viewer：关闭
阈值：0.10 m
```

49 个有效几何计算帧的统计：

| 指标 | mean | median | P95 | min | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| total | 3.313 ms | 3.290 ms | 3.653 ms | 3.006 ms | 3.727 ms |
| evidence classification | 0.385 ms | 0.372 ms | 0.519 ms | 0.323 ms | 0.547 ms |
| positive ratio | 3.93% | 3.72% | 6.06% | 1.38% | 7.27% |
| negative ratio | 3.80% | 3.71% | 6.05% | 1.25% | 7.75% |
| consistent ratio | 92.27% | 92.72% | 94.96% | 87.56% | 97.37% |
| comparison coverage | 68.93% | 69.14% | 71.67% | 65.02% | 71.90% |

端到端：

```text
tracking mean:     14.853 ms
tracking P95:      16.533 ms
active total mean: 23.204 ms
deadline miss:     0/50
actual FPS:        29.691
```

---

## 7. TUM sitting-static 50 帧

该数据目录原先没有 `associations.txt`。本次只在 `/tmp` 中按 TUM 标准方式生成 RGB/Depth 最近时间戳一对一关联，没有修改数据集目录。

条件：

```text
序列：fr3_sitting_static
帧数：前 50 个临时 association
语义：关闭
Viewer：关闭
阈值：0.10 m
```

49 个有效几何计算帧的统计：

| 指标 | mean | median | P95 | min | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| total | 3.269 ms | 3.250 ms | 3.672 ms | 2.920 ms | 3.881 ms |
| evidence classification | 0.384 ms | 0.359 ms | 0.526 ms | 0.307 ms | 0.596 ms |
| positive ratio | 3.20% | 2.96% | 4.37% | 1.92% | 6.99% |
| negative ratio | 3.59% | 3.27% | 5.98% | 2.23% | 6.77% |
| consistent ratio | 93.21% | 93.55% | 95.11% | 87.13% | 95.29% |
| comparison coverage | 69.88% | 69.48% | 73.22% | 66.10% | 73.90% |
| mean absolute residual | 0.0449 m | 0.0391 m | 0.0909 m | 0.0255 m | 0.0991 m |
| signed residual mean | -0.0031 m | 0.0006 m | 0.0240 m | -0.0474 m | 0.0266 m |

端到端：

```text
tracking mean:     13.750 ms
tracking P95:      14.995 ms
active total mean: 22.710 ms
deadline miss:     0/50
actual FPS:        29.126
dataset FPS:       29.237
```

`sitting_static` 表示相机运动类型为 static，不等于画面中所有人和所有深度边界具有像素级静态真值。因此其 positive ratio 不能直接命名为 false-positive rate。

---

## 8. 客观比较

在临时 0.10 m 阈值下：

```text
walking positive mean:       3.93%
sitting-static positive mean: 3.20%
差值:                        0.73 percentage points
```

这只能说明：

- G0-2 正负残差分类链路工作正常；
- walking 前 50 帧的正残差比例略高；
- 0.10 m 阈值仍产生数量不可忽略的正负不一致证据；
- 没有像素级动态真值时，不能计算 precision、recall 或误检率；
- 不能根据当前两个短片段证明 0.10 m 是合理最终阈值。

尤其不能推导：

```text
positive residual == 已确认动态对象
negative residual == 已确认背景显露
consistent evidence == 已确认静态对象
```

---

## 9. 性能结论

G0-2 分类新增开销：

```text
walking mean:       0.385 ms
sitting-static mean: 0.384 ms
```

G0-1＋G0-2 总计算：

```text
约 3.27–3.31 ms/帧
P95 约 3.65–3.67 ms
```

两个 50 帧短跑均接近数据集帧率且无 deadline miss。

尚未测试：

```text
同步 CUDA semantic + G0-2 的组合时间
Viewer 开启时的时间
长序列线程调度影响
```

因此不能把“几何分支满足 30 FPS”扩展成“完整 semantic＋geometry 系统已经满足 30 FPS”。

---

## 10. 阶段结论

G0-2 已完成以下工程目标：

```text
valid/unknown 独立表达
正残差 seed mask
负残差诊断 mask
一致性证据 mask
互斥和完备性不变量
可配置米制阈值
证据比例和耗时统计
合成测试
walking 与 sitting-static 短跑
```

但研究结论只应冻结为：

> G0-2 已建立可验证的几何证据分类接口；0.10 m 仍是临时诊断阈值，mask 尚未经过空间位置和像素级真值验证。

---

## 11. 下一步建议

不建议立即进入 G0-3 区域生长。

原因：

1. walking 与 sitting-static 的正残差比例差距较小；
2. 当前不知道正 seed 集中在运动人体、深度边缘还是静态背景；
3. 区域生长会放大 seed，错误 seed 可能吞入更大静态区域。

建议插入一个小诊断步骤：

```text
G0-2V Visualization

输出：
valid coverage
positive seeds
negative diagnostics
RGB overlay

要求：
不修改 SLAM 状态
不把可视化时间计入几何算法耗时
默认关闭文件写入
只保存少量指定帧
```

只有看到 seed 的空间位置基本合理后，再进入 G0-3 region growing。

---

## 12. 原始日志

```text
g0_2_walking50.log
SHA-256:
b970fb1110d00c3c2c0e1d2372f41f40aff8efda0f6cb2851498ee1df2d5d9de

g0_2_sitting_static50.log
SHA-256:
d2522dfdb3d364a8c4c3173d20e59aebf3f99cc4e2e1b930e4d0b7593a5e7674

g0_2_final_code_smoke10.log
SHA-256:
a648afd569632be162989f60d37114d4d1e6ff3db4354dbbf5e95bebd77087be
```

