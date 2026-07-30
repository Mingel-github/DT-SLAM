# G2-4B Bonn 坐标域实现与验证结果

日期：2026-07-29
状态：RGB/depth/ORB/online mask/geometry 一致域门通过
研究状态：shadow-only；dynamic decision=none；G1-F/G1-D 继续锁定

## 1. 实现边界

`[S]` 新增默认关闭的 `RGBDInputRectifier`，只在 `rgbd_tum` 输入端工作：

```text
raw RGB   --INTER_LINEAR--> rectified RGB -> YOLO + TrackRGBD
raw depth --INTER_NEAREST-> rectified depth -> TrackRGBD + geometry
output projection          -> P=official K
tracking distortion        -> zero
```

配置：

```text
RGBD.InputRectification.Enable
RGBD.InputRectification.fx/fy/cx/cy
RGBD.InputRectification.k1/k2/p1/p2/k3
```

安全检查：

- 缺少任一标定值即拒绝；
- tracking `Camera.k*` 非零即拒绝，避免 double-undistort；
- G2-4B 强制 input `K` 与 output `Camera K` 相同，即 `P=K`；
- rectified input 下 dedicated `Geometry.Camera K` 若与 tracking K 不同即拒绝；
- precomputed mask 必须显式声明
  `DT_SLAM_PRECOMPUTED_MASK_DOMAIN=undistorted_pinhole`。

`[S]` 未修改 YOLO 模型或实现，未修改 Optimizer/g2o，未增加
`PoseOptimization`，未产生 motion score 或过滤。

## 2. 确定性测试

命令：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM/build
cmake ..
make geometric_warp_test rgbd_tum -j$(nproc)

cd /home/zhu/dynaslam_ws/DT-SLAM
LD_LIBRARY_PATH=... ./Examples/RGB-D/geometric_warp_test
```

结果：

```text
[.../G2-4A/G2-4B Test] PASS
```

覆盖：

- toggle 关闭时 RGB/depth 不重采样；
- 16-bit depth rectification 结果只包含原 depth 值或 0，没有线性混合值；
- non-zero tracking distortion 被拒绝；
- Bonn 官方 `K,D` 的 raw/undistorted round-trip max error 不超过 `0.05 px`；
- 既有 geometry tests 全部通过。

构建只有既存的 ONNX Runtime C++17 和 Eigen deprecation warnings，没有新增
编译错误。

## 3. TUM 默认关闭回归

`[S]` 用 TUM3 原配置运行 3 帧，日志：

```text
[RGBD Input] domain=input_native rectification=disabled
Images in the sequence: 3
deadline_missed=0/3
```

没有 `input_rectification` timing row，因为该路径完全旁路。该 smoke 证明默认
开关行为，没有重跑正式 TUM ATE，因此不声称轨迹数值逐位等价。

## 4. Bonn archive 与 association

`[S]` `make_rgbd_association.py` 增加可选 `--dataset-root`。它会在一对一
20 ms matching 前过滤不存在的文件，并把 missing count 写入 association
header。

`moving_nonobstructing_box` 结果：

```text
wrote pairs       = 778
unmatched RGB      = 0
unmatched depth    = 0
missing RGB files  = 0
missing depth files= 4
max |RGB-depth dt| = 16.629934 ms
```

“unmatched depth=0”是在过滤 4 个不存在条目之后的值，不能解释为原始
`depth.txt` 完整。

## 5. 真实 RGB/depth rectification 审计

原始 JSON：

```text
results/g2_4_2026-07-29/bonn_nonobstructing_30_rectification_audit.json
```

`[S]` 前 30 个 existing-file pair：

```text
P=K source in bounds                = 99.4010%
out-of-bounds source pixels/frame  = 1840
valid-depth retention mean         = 99.5246%
valid-depth retention min          = 99.5124%
nearest-depth value violations     = 0
```

`[S]` RGB/depth edge alignment risk proxy：

```text
raw mean       = 0.747225
rectified mean = 0.747480
```

proxy 定义为 depth-discontinuity pixel 中，距离 top-10% RGB gradient 的
Chebyshev distance 不超过 2 pixel 的比例。它只能说明联合 remap 未在这 30 帧
上明显破坏该代理，不能证明 RGB-depth 完美注册，更不是 object boundary GT。

## 6. 150 帧 geometry shadow smoke

日志与 CSV：

```text
results/g2_4_2026-07-29/bonn_nonobstructing_150_rectified_shadow.log
results/g2_4_2026-07-29/bonn_nonobstructing_150_rectified_region.csv
```

域签名：

```text
pixel domain:
  jointly rectified RGB/registered-depth input pixels
camera model:
  shared rectified tracking/geometry pinhole K with zero distortion
semantic/feature:
  Frame::mvKeys in rectified input
optimizer:
  Frame::mvKeysUn, same pixels because Camera distortion is zero
```

多参考 geometry 从 frame 39 开始实际执行。结果：

```text
frames                              = 150
region CSV rows                     = 2107
frames with region rows             = 111
input rectification mean            = 0.631 ms
input rectification P95             = 0.715 ms
active total mean                   = 26.894 ms
deadline misses                     = 2/150
actual FPS                          = 29.718
```

这只是短 smoke timing，不是正式 end-to-end performance 结论。

CSV invariant：

```text
single + multi comparison pixels == comparison pixels: 0 violations
boundary d1 <= d2 <= total positive votes:             0 violations
invalid d1 <= d2 <= total positive votes:              0 violations
unanimous <= positive presence pixels:                 0 violations
```

每个 geometry evidence log 继续包含：

```text
dynamic_decision=none
direct_slam_state_mutation=none
```

## 7. Online YOLO/mask 坐标域 smoke

`[S]` 沙箱内的两次失败保留为环境诊断证据：

```text
bonn_nonobstructing_3_rectified_yolo_smoke.log
bonn_nonobstructing_3_rectified_yolo_gpu_smoke.log
```

第一次使用 bundled ONNX Runtime，只发现 `CPUExecutionProvider`；当前 YOLO
production path 禁止 CPU fallback，因此按设计拒绝。

第二次在沙箱内使用 GPU ONNX Runtime，能够列出 TensorRT/CUDA provider，但
`cudaSetDevice` 返回 CUDA error 100。进一步只读核对发现：

```text
nvidia-smi（host）     = 可见 RTX 4060 Ti
/dev/nvidia*（sandbox）= 不可见
```

`[S]` 因此该失败属于沙箱设备隔离，不能写成主机没有 GPU，也不是几何方法
失败。

在允许访问本机 GPU 的同一代码、配置和数据上完成 3 帧和 30 帧 smoke。30 帧
日志：

```text
results/g2_4_2026-07-29/
  bonn_nonobstructing_30_rectified_yolo_gpu_smoke.log
```

结果：

```text
semantic provider              = CUDAExecutionProvider
input domain                   = undistorted_pinhole, 640x480
mask ready                     = 30/30
mask age median/max            = 0/0 frame
input rectification mean/P95   = 0.637/0.762 ms
semantic block steady mean     = 8.904 ms
active total mean              = 26.574 ms
deadline misses                = 0/30
actual FPS                     = 29.724
```

first inference 包含 warm-up，`onnx_execution max=255.970 ms`，但首帧在正式
tracking loop 前预计算，因此 loop 内 exact-frame mask 仍全部就绪。30 帧只是
坐标域和执行 smoke，不是正式性能报告。

`[S]` 该日志同时记录：

```text
YOLO input       = rectified RGB
ORB/feature      = Frame::mvKeys in rectified input
depth/geometry   = rectified registered depth
optimizer pixels = mvKeysUn == mvKeys because Camera distortion is zero
```

所以统一 RGB/depth/ORB/mask/geometry 坐标域已经实测通过。

## 8. 冻结结论

```text
G2-4B math/config validation              = 通过
G2-4B real RGB/depth joint remap          = 通过
G2-4B ORB/geometry rectified-domain smoke = 通过
G2-4B default-off TUM bypass              = 通过
G2-4B online YOLO/mask execution          = 通过（host GPU）
dynamic/static separability               = 尚未验证
dynamic threshold                         = 未选择
G1-F / G1-D                               = 继续锁定
```

`[S]` 下一小步是不修改 YOLO 地实现 Bonn 自动选帧和 box semantic-coverage
审计：先判断 box 在现有 semantic mask 中是否被覆盖，再决定哪些像素可称为
semantic-uncovered geometry evidence。仍不得直接选择动态阈值。

## 9. 可复现运行模板

数据解压与 association：

```bash
cd /home/zhu/dynaslam_ws

DTSLAM_BONN_TMP=$(mktemp -d /tmp/dtslam_bonn_XXXXXX)
unzip -q BONN/rgbd_bonn_moving_nonobstructing_box.zip \
  -d "$DTSLAM_BONN_TMP"

DTSLAM_BONN_DATA="$DTSLAM_BONN_TMP/rgbd_bonn_moving_nonobstructing_box"

python3 DT-SLAM/tools/make_rgbd_association.py \
  "$DTSLAM_BONN_DATA/rgb.txt" \
  "$DTSLAM_BONN_DATA/depth.txt" \
  <fresh-association-output> \
  --max-difference-ms 20 \
  --dataset-root "$DTSLAM_BONN_DATA"
```

150 帧 geometry shadow：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM

export DT_SLAM_DISABLE_VIEWER=1
export DT_SLAM_GEOMETRY_MULTIREF_CSV=<fresh-multiref-csv>
export DT_SLAM_GEOMETRY_REGION_EVIDENCE_CSV=<fresh-region-csv>
export LD_LIBRARY_PATH="/home/zhu/dynaslam_ws/pangolin_install/lib:/home/zhu/dynaslam_ws/DT-SLAM/lib:/home/zhu/dynaslam_ws/DT-SLAM/thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"

./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/BONN_GeometryPyramidEvidenceShadow.yaml \
  "$DTSLAM_BONN_DATA" \
  <(head -n 152 <association-with-two-comment-lines>) \
  > <fresh-shadow-log> 2>&1
```

关联文件头的注释行数必须先检查，不能默认 `head -n 150` 就是 150 个 pair。

30 帧 online YOLO 还需设置 host GPU ONNX Runtime：

```bash
ORT_CAPI=/home/zhu/.local/lib/python3.10/site-packages/onnxruntime/capi
NVIDIA_BASE=/home/zhu/.local/lib/python3.10/site-packages/nvidia
NVIDIA_LIBS=$(find "$NVIDIA_BASE" -mindepth 2 -maxdepth 2 -type d -name lib -printf '%p:')

export LD_PRELOAD="$ORT_CAPI/libonnxruntime.so.1.23.2"
export LD_LIBRARY_PATH="${ORT_CAPI}:${NVIDIA_LIBS}/home/zhu/dynaslam_ws/pangolin_install/lib:/home/zhu/dynaslam_ws/DT-SLAM/lib:${LD_LIBRARY_PATH:-}"

./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/BONN_GeometryPyramidEvidenceShadow.yaml \
  "$DTSLAM_BONN_DATA" \
  <(head -n 32 <association-with-two-comment-lines>) \
  weights/yolov8n-seg.onnx \
  > <fresh-yolo-log> 2>&1
```

这些是复现模板；输出必须使用新路径，不能覆盖本文引用的原始日志。
