# G2-4E Strict Unknown-Dynamic Hold-out 封存清单

封存日期：2026-07-29

## 1. 封存对象

```text
archive:
  /home/zhu/dynaslam_ws/BONN/rgbd_bonn_balloon_tracking.zip
size_bytes:
  325919363
sha256:
  3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
archive_mtime:
  2019-02-28 18:37:36 +0800
```

## 2. 选择规则

只依据封存前可见的序列元数据选择：

- target family 与 development 的 moving box 不同；
- 当前冻结语义分支只保留 COCO class 0 person，不能把 balloon 作为直接语义类别；
- 序列名称明确为 tracking；
- 本地代码、结果和阶段文档中未发现该序列的 geometry、contact sheet、轨迹或阈值调试记录；
- 选择前没有解压、查看 RGB/depth、生成 association 或运行 SLAM/geometry。

选择没有使用：

- 图像内容；
- geometry residual/score；
- tracking/ATE/FPS；
- semantic mask；
- 人工帧选择；
- 任何方法表现。

## 3. Hold-out 身份

从本文件写入起，`rgbd_bonn_balloon_tracking` 定义为：

```text
strict_unknown_dynamic_holdout = true
development_or_tuning_use      = forbidden
geometry_run_before_freeze     = forbidden
visual_review_before_freeze    = forbidden
```

它只允许在以下内容全部冻结后运行一次正式评测：

1. geometry score/decision 公式；
2. 所有阈值；
3. feature/region 状态约定；
4. motion-label 与排除协议；
5. 抽样和指标脚本；
6. G1-F 是否放行的判据。

## 4. 解封规则

首次 hold-out 运行后必须：

- 立即记录代码 commit、配置哈希、archive 哈希和完整命令；
- 保存完整连续序列结果，不只挑选成功帧；
- 不得根据该序列回调阈值再重报同一序列；
- 若发现实现 bug，只能先记录该次结果无效，修复后将该序列降级为
  development，并另选从未查看的新 hold-out；
- 失败结果必须保留。

## 5. 当前禁止操作

在正式解封前不得：

- 解压该 archive；
- 列出或打开其中的 RGB/depth 文件；
- 生成 contact sheet；
- 运行 YOLO、ORB-SLAM2 或 geometry；
- 将其用于性能优化；
- 根据其结果改变方法。

`balloon`、`balloon2`、`crowd` 和 `person_tracking` 当前不具备本清单定义的
strict hold-out 身份；这不表示它们已被查看或可自动用于调参。
