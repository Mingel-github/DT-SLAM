# G2-4F1D 非 Holdout 开发数据可观测性协议

日期：2026-07-29

## 1. 目的

G2-4F1 已通过工程实现和运行时间门，但当前两条 moving-box development
序列缺少足够可靠的 `moving + person-absent` 样本，因而科学门不可评价。

本阶段只回答：

> 本地是否存在一条非 strict-holdout 序列，能够观察到语义分支未覆盖的未知
> 物体独立运动，并可用于继续评价 G2-4F1？

它不是几何方法改进，不选择阈值，不生成动态判决，也不修改 SLAM 状态。

## 2. 数据来源与角色

`[L]` Bonn 官方数据页说明该数据集包含人员操作箱子或玩气球等高动态序列，
并提供相机轨迹真值；官方没有提供逐帧对象运动 mask。

本阶段冻结以下角色：

```text
development screening:
  BONN/rgbd_bonn_balloon.zip
  BONN/rgbd_bonn_balloon2.zip

strict unknown-dynamic holdout:
  BONN/rgbd_bonn_balloon_tracking.zip
```

`balloon` 和 `balloon2` 可以被打开、查看和用于开发期数据可观测性审计。它们
一旦被查看，就不能再作为 strict holdout。

`balloon_tracking` 不得解压、查看图像、运行语义、运行 SLAM 或读取 archive
内部文件。只允许核对 archive 外部文件大小和 SHA-256：

```text
sha256 = 3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
```

## 3. 独立性约束

候选片段筛选只能读取：

- rectified RGB；
- RGB/depth association 的时间戳和文件存在性；
- 同步 C++ semantic person mask/detection；
- 可选相机 GT，用于描述相机运动，不得作为对象运动标签。

筛选和初步标签不得读取：

- G2-4F1 observed-flow、ego-flow 或 residual；
- G0/G2 depth residual、positive vote 或 region score；
- ORB feature residual；
- 未来动态阈值输出。

由此得到的标签仍只是：

```text
agent_rgb_temporal_observability_proxy
is_ground_truth = false
```

不能把 archive 名称 `balloon` 自动解释成每帧气球正在运动。

## 4. 低人工负担审计

Agent 自动完成：

1. 过滤 archive 中不存在的 RGB/depth 文件并生成一对一 association；
2. 使用现有 Bonn rectification，保持 RGB、depth、ORB 与 mask 在同一针孔域；
3. 在线同步运行当前 C++ person semantic 分支，要求候选帧 `mask_age=0`；
4. 仅根据 RGB temporal change、时间多样性和 person-mask 状态生成短时序联系表；
5. Agent 查看联系表，保守标记气球：
   - `moving_observable`；
   - `stationary`；
   - `uncertain_camera_object_motion`；
   - `not_visible`；
6. 对 `moving_observable` 再区分：
   - `person_absent_or_not_overlapping`；
   - `person_overlapping`。

不要求用户逐帧人工标注，也不生成像素级气球 GT。

## 5. 数据门

开发数据门通过至少需要：

- 至少 10 个时间分离的 `moving_observable` 中心帧；
- 至少 5 个 `moving_observable + person_absent_or_not_overlapping` 中心帧；
- 候选片段中有足够静态背景；
- 气球不是整帧填满视野；
- RGB temporal clip 足以区分相机运动与对象相对背景运动；
- 同步 semantic mask 有效且 `age=0`。

若两条序列均不满足，则结论为：

```text
G2-4F1 development scientific gate = not evaluable from local Bonn data
```

不得通过放宽标签、把 uncertain 强制当 moving，或查看 strict holdout 来补足样本。

## 6. 通过后的评价顺序

只有数据门通过后才允许：

1. 冻结所选中心帧和 RGB-only/proxy 标签；
2. 运行已有 G2-4F1 shadow，不改实现；
3. 评价连续 residual 的方向性、背景风险和运行时间；
4. 再决定是否值得设计有文献依据的可靠 feature gate。

即便方向性审计通过，仍不自动解锁 G1-F；阈值、跨序列验证和静态风险门必须
另行冻结。

## 7. 明确禁止

```text
不打开 balloon_tracking
不根据 F1 residual 选帧
不调 depth/flow/region threshold
不把 proxy 写成 GT
不修改 YOLOSegment.cc 或模型
不修改 Optimizer.cc、g2o 或后端
不写 mvbDynamic
不清除 mvpMapPoints
不进入 G1-F / G1-D
```
