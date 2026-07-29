# G2-4 Bonn Static/Moving-Box 低人工负担评价协议

日期：2026-07-29
状态：数据协议冻结候选；已只读检查 archive，尚未解压或运行 Bonn SLAM

## 1. 数据事实

`[L]` Bonn 官方页面提供：

- 24 个动态序列和 2 个静态序列；
- moving/placing/removing/kidnapping box 等场景；
- OptiTrack camera-pose GT；
- static-environment ground-truth 3D point cloud；
- 已注册到 RGB 的 depth；
- 非零 RGB distortion。

官方页面没有声明逐帧 dynamic-object mask。

本地已有只读 zip：

```text
BONN/rgbd_bonn_moving_nonobstructing_box.zip
BONN/rgbd_bonn_moving_obstructing_box.zip
```

另有 person/crowd/balloon archive，但本轮不扩展范围。本地尚未发现 Bonn static
archive。

## 2. 坐标域先决条件

官方 calibration：

```text
fx = 542.822841
fy = 542.576870
cx = 315.593520
cy = 237.756098
d0 = 0.039903
d1 = -0.099343
d2 = -0.000730
d3 = -0.000144
d4 = 0
```

当前 geometry warp 使用 pinhole K 和 raw registered-depth pixel index；Bonn
具有非零畸变，因此不能直接沿用 TUM3 零畸变假设。

在运行 evidence 前必须二选一并验证：

1. RGB、registered depth、semantic mask 和 ORB 全部进入同一个 undistorted
   pinhole domain；
2. geometry projection 显式实现与 RGB/depth 相同的 distortion domain。

不得只 undistort ORB 而让 depth/mask 保持 raw，也不得把 `mvKeys` 与
`mvKeysUn` 混用。先用棋盘/合成点和真实边缘可视化验证，再运行 Bonn。

坐标域只读审计已经冻结选择：

```text
raw RGB   --linear remap--> rectified RGB --> YOLO + ORB
raw depth --nearest remap-> rectified depth -> tracking + geometry
output P=official K
tracking distortion=0
geometry K=tracking K
```

该组合是 `[A/S/H]`，不是任何论文整体方法的复现。实现和验收细节见：

```text
G2_4B_BONN_COORDINATE_DOMAIN_AUDIT_AND_SPEC.md
```

本地 archive 还存在索引完整性问题：

```text
moving_nonobstructing_box: depth.txt 引用 4 个不存在的 PNG
moving_obstructing_box:    depth.txt 引用 3 个不存在的 PNG
```

association 必须过滤不存在的文件，并保存排除清单。

## 3. 序列角色

冻结候选：

```text
calibration dynamic:
  moving_nonobstructing_box

hold-out unknown dynamic:
  moving_obstructing_box

static risk:
  Bonn static（获得后再拆 calibration/hold-out）
```

若 YOLO 实际给 box 产生稳定实例 mask，则该序列不能自动称为“semantic unknown”。
必须逐帧审计 box 是否落入 semantic mask；只有未被 semantic 模块覆盖的 box
证据才能用于 unknown-category 几何结论。

## 4. 自动选帧

Agent 自动从每个 moving-box 序列选择最多 24 帧：

- 6 帧：box 明显静止；
- 6 帧：box 开始/停止运动；
- 6 帧：box 稳定运动；
- 6 帧：遮挡、出入视野或深度边界困难帧。

选择依据先使用 RGB/depth temporal change、GT camera motion compensation 和
geometry-risk 统计；不依据未来判决阈值。

static 序列最多选择 24 帧，按 camera angular/translation speed 和
boundary/invalid risk 分层。

## 5. 预标注与用户负担

Agent 负责：

- 生成 box/person 初始 mask；
- 利用相邻帧传播并检查 temporal consistency；
- 将 boundary/invalid 区域单独可视化；
- 生成可点击的逐帧审查页面；
- 自动把 mask 投影成 ORB-feature labels。

用户只需要：

- 修正少量错误 box boundary；
- 标记 box 在该帧是 moving、transition 还是 static；
- 确认严重遮挡帧是否排除。

目标是最多审查 `24 + 24 + 24` 个预标注帧，而不是逐帧从零标注。

## 6. 评价单位

G1-F 之前优先评价 ORB feature：

- dynamic-box feature precision/recall；
- static-background feature risk；
- feature coverage；
- boundary/invalid risk 分层；
- semantic-covered 与 semantic-uncovered box 分开报告。

pixel mask 指标作为补充：

- precision/recall/IoU；
- boundary tolerance；
- 遮挡与 transition 帧单独报告。

camera trajectory GT 只用于 pose error 和 camera-motion compensation，不能当
object-motion GT。

## 7. 数据泄漏控制

- calibration 序列允许设计 score/threshold；
- hold-out 序列只在参数冻结后运行；
- moving_obstructing_box hold-out 失败后不得回调参数并重报同一结果；
- static 与 dynamic 预标注工具可以相同，但人工修正记录必须版本化；
- TUM exploratory proxy 指标不进入最终 unknown-box 主表。

## 8. 当前停止点

本协议完成不代表数据门通过。当前仍缺：

```text
Bonn static archive
automatic frame selector
pre-annotation/review tool
frozen motion-label protocol
```

在这些项目完成前：

```text
G2-4B threshold selection = 禁止
G1-F                      = 锁定
```
