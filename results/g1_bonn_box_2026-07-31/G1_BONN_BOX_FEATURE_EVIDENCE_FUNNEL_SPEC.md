# G1 Bonn 箱子特征证据漏斗审计 SPEC

日期：2026-07-31
状态：运行前冻结

## 目标

解释 `moving_nonobstructing_box` 空间抽查中“实际移除点没有落入粗箱框”的
原因。只测量以下漏斗，不修改动态判断：

```text
ORB feature
→ sparse-flow evidence measured
→ quality eligible
→ frozen q10 candidate
→ post-SearchLocalPoints candidate MapPoint association
→ actually removed association
```

## 固定条件

```text
same 778 RGB-D associations
same Bonn joint P=K rectification
online CUDA YOLO, mask age must be 0
q=10 unchanged
tracking/mapping 5% safeguards unchanged
Viewer OFF
semantic+geometry only, one diagnostic run
```

逐特征 CSV 只记录已有 24 个粗箱框 review 帧，避免产生整段近百万行输出。
post-search candidate association 和实际移除点仍记录全序列，以验证逐帧计数不变量。

## 字段定义

- `ORB feature`：校正域 `Frame::mvKeys` 落在粗箱框内；
- `measured`：G2-4F1 具有完整 LK、参考深度和 SLAM ego-flow residual；
- `quality eligible`：与现有 selector 完全相同的 measured、有限 residual 和
  forward-backward error `<=0.25 px`；
- `q10 candidate`：语义非动态且归一化 residual `q>=10`；
- `initial association`：运行 sparse-flow 时已有 MapPoint；
- `post-search association`：`SearchLocalPoints()` 后、G1-F1 过滤前具有有效
  MapPoint 的 q10 candidate；
- `removed`：G1-F1 通过安全限制并真正将 association 置空。

## 评价边界

粗箱框是 RGB-only、未验证 proxy，不是逐像素 GT。因此本审计只能定位证据在哪一
层消失，不能报告正式 precision/recall，也不能据此调整 q10/5%。

## 决策

- ORB 很少：冻结稀疏路线对低纹理箱子的覆盖限制；
- measured 足够但 q10 少：相邻帧 residual/时域证据不足；
- q10 有而 post-search association 少：检测证据不能有效进入 tracking；
- post-search 有且移除有，但主要在框外：目标特异性不足；
- 箱框内形成稳定候选/移除：再进入 `moving_obstructing_box` 判别性检查。
