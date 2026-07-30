# G2-4F1 独立候选扩充结果

日期：2026-07-29
状态：完成；没有发现新的可靠 `moving+person-absent` 帧。

## 1. 目的与隔离

原 48 个 development 候选只有 1 个 `moving+person-absent` 帧。为判断这是
选帧遗漏还是数据可观察性问题，新增一次独立扩充。

候选生成只读取：

- RGB 图像；
- association；
- 同步 YOLO person mask 非零像素数；
- 已有候选 frame id。

它不读取：

- depth；
- geometry/region score；
- G2-4F0 vote；
- G2-4F1 residual；
- SLAM trajectory。

全部输出标记：

```text
is_ground_truth=false
geometry_or_flow_seen=false
dynamic_decision=none
strict_holdout_opened=false
```

## 2. 全序列语义统计

`semantic_review_export` 增加默认关闭的 manifest-only 模式，只保存
manifest/detections，不保存 1367 组 RGB/mask/overlay。

CUDA 全序列结果：

| 序列 | 帧数 | center person-mask absent |
| --- | ---: | ---: |
| nonobstructing | 778 | 571 |
| obstructing | 589 | 417 |

原 48 候选与先前 C++ 导出逐项复核：

```text
mask pixels mismatch = 0/48
intermediate pixels mismatch = 0/48
detection count mismatch = 0/48
```

## 3. RGB-only 选择

固定选择规则：

- person mask 在五帧窗口内均为空，RGB temporal difference 较大；
- center mask 为空、窗口内出现 semantic transition；
- person-absent 时间轴上的均匀采样；
- 与原候选及新候选保持至少 6 帧间隔。

没有读取待测 flow 或 geometry。

得到：

```text
nonobstructing = 26 clips
obstructing    = 24 clips
```

每个 clip 为 rectified `t-2..t+2` RGB。

## 4. Agent RGB 时序复核

保守标签：

| 序列 | stationary | moving | uncertain |
| --- | ---: | ---: | ---: |
| nonobstructing | 26 | 0 | 0 |
| obstructing | 22 | 0 | 2 |

两个 uncertain 帧为 obstructing frame 206/225。箱体表面几乎占满整个视野，
缺少静态背景，无法只凭 RGB 时间窗区分：

```text
camera relative to box motion
vs.
box independent motion
```

因此没有将其强制标为 moving。

## 5. 结论

扩充没有提供新的可靠 `moving+person-absent` 开发帧。当前主门不可评价主要
是这两条序列中的可观察性/标签支持不足，而不是原 48 帧自动选帧漏掉了大量
明显样本。

所以：

```text
不继续调 G2-4F1 threshold
不使用 uncertain 作为 positive
不进入 G1
不解封 strict hold-out
```

若要继续验证该科学门，需要一条独立的 development 序列，满足：

- 未知物体确实运动；
- 当前 semantic mask 为空或运动物体位于 semantic mask 外；
- 视野保留足够静态背景；
- 最好有静态相机段或对象运动标签。

这可以来自新的非 hold-out 数据、自采受控序列，或在实验协议中明确允许的
额外 Bonn development 序列；不能事后把当前 uncertain 帧改成 positive。

证据：

- `nonobstructing_semantic_manifest/`
- `obstructing_semantic_manifest/`
- `nonobstructing_review_v1/`
- `obstructing_review_v1/`
- `agent_rgb_temporal_expansion_labels.csv`
