# Bonn 额外箱子序列下载记录

日期：2026-07-31
身份：评价数据准备，不是方法变更，不是运动标签

来源：

```text
https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/
```

已下载并通过 `unzip -tq` 完整性检查：

| archive | 本地大小 | SHA-256 |
|---|---:|---|
| `rgbd_bonn_placing_nonobstructing_box.zip` | 383 MiB | `8eda7eabf1f71217d202d12f438201146a1488c1eb3664e21f490701934b9bcc` |
| `rgbd_bonn_removing_nonobstructing_box.zip` | 260 MiB | `a1d4731b6d0a60d50009c4c401336a23957c5182e3be79fd8e7028e9dde1db8d` |
| `rgbd_bonn_kidnapping_box.zip` | 591 MiB | `e38aa756291bf5e4a1a9941be60638d05d40f371f674c82b9ae4396e1a209ca5` |

每个 archive 均包含：

```text
rgb.txt
depth.txt
groundtruth.txt
rgb/*.png
depth/*.png
```

角色约束：

- 三条序列只作为新的 development/review 数据；
- archive 名称表示数据集场景主题，不是逐帧 motion GT；
- 尚未执行候选帧选择、运动状态标注、geometry audit 或参数选择；
- 不打开既有 `balloon_tracking` strict hold-out；
- 不用这些数据修正已经失败的 G2-6E scanner-model 坐标链；
- 后续若使用，必须先通过连续 RGB 时序确认具体运动窗口，并记录人物混杂、
  可见性和遮挡状态。

本次下载没有修改 SLAM、YOLO、Optimizer 或任何过滤逻辑。
