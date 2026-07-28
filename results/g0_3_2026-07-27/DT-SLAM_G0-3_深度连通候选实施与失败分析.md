# DT-SLAM G0-3 深度连通候选实施与失败分析

日期：2026-07-27

## 1. 阶段目标与边界

G0-3 在 G0-2 positive seed 基础上生成受深度连续性约束的区域候选，继续保持
shadow-only。

输入：

- 当前 `CV_32FC1` 米制深度；
- `validComparisonMask`；
- `positiveSeedMask`；
- 暂定局部深度阈值 `0.05 m`。

规则：

1. 只从 positive seed 出发；
2. 只访问 `validComparisonMask != 0` 且当前深度有效的像素；
3. 使用四邻域；
4. 相邻像素满足
   `abs(depth_neighbor - depth_current) <= 0.05 m` 时允许传播；
5. negative residual 不参与生长；
6. 每个像素最多进入一次搜索，结果取所有种子可达区域的并集。

输出：

- `regionCandidateMask`；
- 深度连通区域数量；
- 候选像素数；
- 最大单区像素数；
- `candidate_pixels / positive_seed_pixels` 增长倍率；
- region grow 单独耗时。

本阶段未加入没有实验依据的最小面积、最大面积、种子比例或对象尺寸阈值。

本阶段仍未执行：

- ORB 特征动态标记；
- MapPoint 关联清除；
- 再次 PoseOptimization；
- TrackLocalMap 过滤；
- MapPoint 或稠密深度写入过滤。

## 2. 配置

新增：

```yaml
Geometry.RegionGrowEnable: 0
Geometry.RegionDepthThresholdM: 0.05
```

`RegionGrowEnable` 默认关闭。实验结束后配置已恢复：

```yaml
Geometry.Enable: 0
Geometry.LogEveryN: 30
Geometry.RegionGrowEnable: 0
Geometry.DebugSave: 0
Geometry.DebugEveryN: 30
```

## 3. 合成测试

`geometric_warp_test` 增加：

1. 两个 positive seeds 位于同一个 1 m 平面时，只生成一个区域；
2. 1 m 与 2 m 平面之间不能跨越；
3. `validComparisonMask == 0` 的整列必须阻断传播；
4. 增长面积、最大区域和增长倍率必须与人工构造值一致。

结果：

```text
[Geometry G0-3 Test] PASS
```

这证明代码符合当前定义，但不证明该定义能正确分割真实动态物体。

## 4. TUM 实验设置

共同设置：

- 前 50 帧；
- 不启用 YOLO；
- Pangolin viewer 关闭；
- residual threshold：`0.10 m`；
- region local depth threshold：`0.05 m`；
- 每个几何帧记录统计；
- 第 1、10、20、30、40 个几何结果保存可视化；
- region 为绿色、positive seed 为红色、negative diagnostic 为蓝色。

序列：

1. `rgbd_dataset_freiburg3_walking_xyz`
2. `rgbd_dataset_freiburg3_sitting_static`

注意：`sitting_static` 中的 `static` 描述相机运动类型，不代表画面中不存在人体运动，
因此它不是严格的无动态负样本。

## 5. 量化结果

每个序列产生 49 个有效几何结果。

| 指标 | walking_xyz | sitting_static |
|---|---:|---:|
| region pixels 平均 | 205981.0 | 205385.1 |
| region pixels 最小 | 186599 | 191813 |
| region pixels 最大 | 217967 | 221145 |
| region / valid comparisons 平均 | 97.01% | 95.54% |
| 最大单区像素数平均 | 111467.1 | 101516.6 |
| seed 到 region 增长倍率平均 | 27.15 | 31.37 |
| region grow 平均耗时 | 2.828 ms | 2.785 ms |
| region grow 最小耗时 | 2.226 ms | 2.269 ms |
| region grow 最大耗时 | 3.540 ms | 4.249 ms |
| geometry total 平均耗时 | 6.325 ms | 6.122 ms |

PNG 诊断帧写入 6 张图，约 `20–23 ms/采样帧`，写盘耗时不计入 geometry total。

日志：

- `g0_3_walking50_visual.log`
  - SHA-256：
    `26bf1b5b9e3605d1daed719c9883c811f6ddcdca4301c0c13d5e2f3f08fdcfa2`
- `g0_3_sitting_static50_visual.log`
  - SHA-256：
    `2e73047fcd12d8b0f0db391f41f87a6d7488751d5d5ff9fd385ba44398726a37`

## 6. 可视化检查

重点检查：

- `walking50_images/frame_000030_region_overlay.png`
- `sitting_static50_images/frame_000030_region_overlay.png`

两张图都显示：

1. 人体表面被扩展；
2. 隔板、桌面、地面、显示器周边及其他静态结构也大面积变绿；
3. 结果接近整个有效 RGB-D 比较区域，而非独立物体区域；
4. 深度断层能阻挡部分传播，但无法阻止每个静态表面上少量 seed 分别触发增长；
5. 局部相邻深度差允许沿平滑表面逐步传播，存在链式扩展。

## 7. 客观结论

### 工程结论

- 实现满足定义；
- 合成边界测试通过；
- 单帧额外计算约 `2.8 ms`；
- 默认关闭；
- 没有改变 SLAM 状态。

### 方法结论

当前定义失败，不能作为动态 region mask：

```text
positive seed
 + 四邻域局部深度连续
 + 所有种子区域取并集
```

在两个 TUM 短片段中覆盖了约 96% 的有效比较像素。计算速度可接受不能抵消区域选择
失效。

因此：

- 不进入 G0-4 ORB 特征投影；
- 不将 `regionCandidateMask` 与 semantic mask 合并；
- 不进行特征、MapPoint 或深度写入过滤；
- 不通过临时增大/减小单个阈值宣称问题已解决。

## 8. 失败原因

当前结果至少揭示两个机制：

1. positive seed 不只来自独立运动，也来自遮挡边界、深度噪声、重投影误差和位姿误差；
2. 即使深度边缘能分开部分物体，每个静态深度表面只要包含少量 seed，也会独立触发
   整个表面增长。

因此，区域生长缺少“该区域中动态证据是否足够”的区域级验证。

## 9. 下一步门控

下一步不应继续特征过滤，而应先增加区域级测量，暂称 `G0-3R`：

1. 对每个深度连通区域统计：
   - region pixels；
   - positive seed pixels；
   - positive seed ratio；
   - residual 中位数；
   - 高残差像素比例；
2. 只输出 top-N 区域统计和离线可视化，不立即设接受阈值；
3. 比较 walking、sitting_static 和真正无动态 RGB-D 片段；
4. 确认动态人体/箱子区域和静态表面的统计是否可分；
5. 只有出现可重复分离后，才定义区域接受规则并进入 G0-4。

若区域统计仍不可分，则需要更强的区域生成边界，例如深度梯度与法向、超像素或短时序
证据，而不是继续依赖单帧局部深度连续性。
