# R4/S4 长时间间隔 Mapping 精修 Shadow 结果

日期：2026-08-05
性质：离线只读；不参与 Tracking，不修改 S1–S3，不接 OctoMap

## 1. 结论先行

本轮按 SInDSLAM 论文 Eq. (17) 实施的固定5帧间隔深度重投影，没有显著恢复 Gazebo
中远距离移动箱子，且新增证据更多出现在箱体以外。按实施前冻结的停止条件，R4/S4
不进入正式 DT-SLAM，也不继续调整 `0.13` 或 `0.4`。

这是一项范围明确的负结果：它否定的是当前 paper-text-guided、固定5帧、当前 SLAM
轨迹和当前区域标签的 Mapping 精修组合，不能外推为 SInDSLAM 完整 Mapping 或所有
长时间基线方法均无效。

## 2. 实施身份

第一版遵循论文文字：

- 每5个输入帧选择一个 sparse frame；
- 当前 sparse frame 投影到上一 sparse frame；
- 深度差超过 `0.13 * current_depth` 时形成新增 seed；
- 一个区域内新增 seed 比例超过0.4时扩展到整区域；
- refined mapping mask 为当前 S3 mask 与长间隔新增 mask 的并集。

参数值由公开 Mapping 源码核对，但没有复制源码的 ROS关键帧缓存、降采样计数补偿或
上一参考 mask 传播。因此准确身份是：

> `[A] paper-text-guided clean-room S4 replay`

而不是官方源码逐行复现。

## 3. 输入与不变量

- 数据：冻结 Gazebo person＋moving box 600帧片段；
- sparse frame 对：119组；
- 当前 mask：R1保存的 SIn 风格 geometry-only mask；
- 区域：R1保存的 RAG labels；
- 位姿：R1同一 geometry-only ORB-SLAM2 最终轨迹；
- 最大建图深度：6 m；
- Gazebo箱体参考只用于评价，不进入算法；
- identity reprojection测试：通过；
- comparison coverage中位数：99.64%。

RAG 是 Region Adjacency Graph，即区域邻接图。所有无效深度、越界和参考深度缺失
保持 unknown，没有解释为静态或自由空间。

## 4. 总体结果

| 指标 | 结果 |
| --- | ---: |
| sparse frame 对 | 119 |
| 箱体参考不少于100像素的 sparse frames | 40 |
| 当前 S3 箱体覆盖中位数 | 0% |
| S4 新增箱体覆盖中位数 | 0% |
| S3＋S4 箱体覆盖中位数 | 0.21% |
| 当前 S3 漏检箱体帧 | 29 |
| 具有任意新增箱体像素 | 11/29 |
| 恢复到至少25%覆盖 | **0/29** |
| 漏检帧最终箱体覆盖最大值 | 16.42% |
| S4新增有效深度比例中位数 | 0.325% |
| S4新增有效深度比例均值 | 3.71% |
| S4新增有效深度比例第90百分位 | 10.44% |
| 发生整区域扩展的帧 | 8/119 |

新增像素比例呈长尾：多数帧增加很少，但少数帧增加大面积区域。该现象没有转化为箱体
恢复。

## 5. 按箱体距离分组

| 距离 | 帧数 | 当前 S3 箱体覆盖中位数 | S4 新增箱体覆盖中位数 | 最终覆盖中位数 | 新增 non-box 比例中位数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 近距离 `<3 m` | 12 | 99.99% | 0% | 99.99% | 0.60% |
| 中距离 `3–6 m` | 17 | 0% | 0.14% | 0.14% | 0.83% |
| 远距离 `>6 m` | 11 | 0% | 0% | 0% | 0.53% |

近距离箱子本来已经由 S3 删除，S4没有必要增益。中距离是本轮应当补充的主要区间，
但覆盖中位数只从0提高到0.14%。超过6 m的箱体位于当前建图深度范围以外，本轮不尝试
恢复。

`non-box` 可能包含运动行人，不能全部称为静态误删。不过新增 non-box 比例在每个距离
组都高于新增箱体覆盖中位数，说明当前 S4 证据没有形成箱体特异性。

## 6. 可视化审计

联系表颜色：

- 绿色边界：Gazebo可见箱体参考；
- 红色：当前 S3 mask；
- 黄色：长时间间隔新增 seed；
- 紫色：由簇比例扩展得到的新增 region。

联系表显示：中远距离箱体内部新增像素很少，较显著的黄色/紫色区域主要出现在图像
边缘、地面边界和近距离遮挡边缘。该观察只作失败定位；由于箱外还含运动人物，不能把
所有箱外新增都定量解释为静态误检。

## 7. 为什么停止

实施前 SPEC 规定，只有 S4 能在当前 S3 漏检帧中形成有意义的新增动态区域，才进入
同位姿点云对照。实际结果为：

- 29个漏检帧中没有一帧恢复到25%箱体覆盖；
- 中距离新增覆盖中位数仅0.14%；
- 新增证据更多落在箱体外；
- 区域扩展仅发生8帧，也没有形成有效箱体恢复。

因此不生成 S3＋S4 正式点云，不接 OctoMap，不搜索时间间隔和阈值。继续降低0.13或0.4
会成为针对当前序列的经验调参，缺少新的因果依据。

## 8. 能够与不能够得出的结论

可以说：

- 当前固定5帧深度精修没有解决 Gazebo 移动箱子残影；
- S3已检测到的近大箱子不需要该层；
- 当前 R4 不具备进入正式 Mapping 的资格；
- R4/S4和 OctoMap都继续保持关闭。

不能说：

- SInDSLAM完整建图链无效；
- 所有长时间间隔都无效；
- Bonn短暂停留人物一定无法由 S4 改善；
- OctoMap能够或不能够修复当前箱子残影；本轮没有运行 OctoMap；
- 当前结果应通过修改阈值修复。

## 9. 文件

- `R2_POST_DECISION.md`；
- `R4_S4_LONG_INTERVAL_MAPPING_REFINEMENT_SHADOW_SPEC.md`；
- `R4_S4_LONG_INTERVAL_MAPPING_REFINEMENT_SHADOW_RESULT.md`；
- `DT-SLAM/tools/audit_r4_long_interval_mapping_refinement.py`；
- `analysis/r4_frame_metrics.csv`；
- `analysis/r4_region_metrics.csv`；
- `analysis/r4_summary.json`；
- `analysis/r4_contact_sheet.png`；
- `analysis/long_interval_seeds/`；
- `analysis/refined_mapping_masks/`。

工具 SHA-256：

```text
de2866f39387df9e54cef4c962000fba2f828ebf8cdd9940567726097fe4b2db
```

## 10. 最终冻结结论

> R4/S4 已按最小、文献支持的 Mapping-only shadow 进行验证。它在当前 Gazebo
> 片段中没有把中远距离箱子从点状深度冲突恢复成可用动态区域，故作为有限负结果冻结，
> 不进入主系统。下一阶段不能继续增加 Mapping补丁；应回到当前已成立的 S1–S3基线，
> 决定论文定位与最终跨数据集评价范围。
