# G1 Bonn moving_nonobstructing_box 对照 SPEC

日期：2026-07-31
状态：运行前冻结

## 目标

只比较：

```text
semantic-only
semantic+geometry
```

判断类别无关稀疏几何在 semantic 已启用时是否实际提供额外 tracking/MapPoint
保护，以及是否造成明显 ATE/RPE、覆盖或 FPS 退化。

## 公平坐标域

两种模式必须共享：

```text
raw RGB   -> linear rectification -> P=K RGB
raw depth -> nearest rectification -> P=K depth
ORB / YOLO / semantic mask / geometry all in the same P=K domain
```

semantic-only 使用：

```text
Examples/RGB-D/BONN_RectifiedSemanticBaseline.yaml
```

semantic+geometry 使用：

```text
Examples/RGB-D/BONN_GeometrySparseEgoFlowShadow.yaml
```

两份配置的 camera、rectification、depth 和 ORB 参数必须相同。前者
`Geometry.Enable=0`，后者只打开已冻结的 G1-F1/G1-M1。

## 固定条件

```text
q10 / 5% safeguards unchanged
online CUDA YOLO
mask age must be 0
Viewer OFF for metrics
Viewer ON only for qualitative inspection
no G1-D
no Optimizer/g2o/YOLO/LocalMapping changes
```

Bonn 只有 camera-pose GT，没有逐帧 box motion GT 或动态 mask。几何候选只能
称为 unknown-box development evidence，不能计算或声称真实 precision/recall。

## 输出与判断

报告：

- trajectory coverage；
- ATE/RPE/FPS；
- tracking applied/removed；
- mapping applied/vetoed；
- fail-open 分布；
- Viewer/overlay 中候选与箱子、人物、深度边界的关系。

不根据本序列调整 q10/5%。若几何没有明显附加证据，保留负结果并停止扩张。
