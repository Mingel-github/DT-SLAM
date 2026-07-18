# DT-SLAM 实验记录

> TUM fr3_walking_xyz | 827 帧 | ORB-SLAM2 baseline RMSE ~0.275m

## 语义版（YOLOv8n-seg, ONNX Runtime, CPU）

| # | 日期 | 配置 | RMSE | Mean | Std | 跟踪 | Mask就绪 | Mask年龄 | YOLO耗时 | Tracking |
|---|------|------|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 2026-07-18 | conf=0.5, proto修复+裁剪 | **0.197** | 0.181 | 0.077 | 827/827 | 825/827 | med=3 max=4 | mean=66.7ms | 15.4ms |
| — | — | proto未修复（全图resize） | 0.260~0.287 | — | — | — | — | — | — | — |

## Baseline（纯几何，无mask）

| # | 日期 | 配置 | RMSE | Mean | Std | 跟踪 | 备注 |
|---|------|------|:---:|:---:|:---:|:---:|------|
| 1 | 2026-07-18 | headless | 0.278 | 0.250 | 0.122 | 827/827 | — |
| 2 | 2026-07-18 | 可视化 | 0.275 | 0.243 | 0.130 | 561/827 | 丢266帧 |

---

## 参考基线

| 方法 | RMSE | 来源 |
|------|:---:|------|
| ORB-SLAM2（论文） | 0.459m | DynaSLAM Table II |
| ORB-SLAM3（实测） | 0.340m | results/baseline_ate.md |
| DynaSLAM N+G（论文） | **0.015m** | SOTA参考 |
