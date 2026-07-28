# DT-SLAM G0-3R 区域证据审计记录

日期：2026-07-27

## 1. 动机

G0-3 的四邻域深度连续增长覆盖了 walking 和 sitting_static 中约 96% 的有效比较域，
不能直接作为动态 mask。

G0-3R 不立即增加接受阈值，而是检查区域级证据是否具有可重复的区分能力。

## 2. 新增诊断

每个由 positive seed 触发的深度连通区域记录：

- 区域像素数；
- positive seed 像素数；
- negative diagnostic 像素数；
- positive seed 比例；
- negative diagnostic 比例；
- signed residual 中位数。

额外输出：

- `regionPositiveSupport`：每个区域填入
  `positiveSeedPixels / regionPixels`；
- `region_support.png`：将上述 `[0,1]` 浮点支持率映射到 `[0,255]`；
- 每帧按 positive seed 数排序的 top 3 区域日志。

中位数使用 `std::nth_element` 计算，不进行完整排序。

本阶段仍未定义“动态区域接受阈值”，也未修改任何 SLAM 状态。

## 3. 构建与合成测试

构建：

```bash
cd /home/zhu/dynaslam_ws/DT-SLAM
make -C build geometric_warp_test rgbd_tum -j"$(nproc)"
```

合成测试覆盖：

- 深度跳变停止；
- unknown barrier 停止；
- 区域 positive seed 计数；
- positive support 比例；
- support image 数值。

结果：

```text
[Geometry G0-3R Test] PASS
```

## 4. 实验设置

使用与 G0-3 相同的两个 TUM 前 50 帧片段：

- `rgbd_dataset_freiburg3_walking_xyz`
- `rgbd_dataset_freiburg3_sitting_static`

共同参数：

```text
residual threshold       = 0.10 m
local depth threshold    = 0.05 m
four-neighbor growth
no semantic model
shadow only
```

注意：`sitting_static` 不是严格无动态序列；`static` 描述相机运动方式，画面中仍有人。

## 5. Top 区域统计

区域按 positive seed 像素数排序。每个序列有 49 个几何帧。

### walking_xyz

| 排名 | 区域面积均值 | positive 数均值 | positive ratio 均值 | positive ratio 范围 | negative ratio 均值 | residual median 均值 |
|---|---:|---:|---:|---:|---:|---:|
| rank 1 | 108083.9 | 2727.4 | 2.467% | 0.354%–8.085% | 2.577% | -0.000526 m |
| rank 2 | 28202.0 | 715.1 | 11.434% | 0.204%–60.661% | 1.943% | 0.009662 m |
| rank 3 | 20228.1 | 439.0 | 18.642% | 0.336%–83.901% | 3.493% | 0.021294 m |

### sitting_static

| 排名 | 区域面积均值 | positive 数均值 | positive ratio 均值 | positive ratio 范围 | negative ratio 均值 | residual median 均值 |
|---|---:|---:|---:|---:|---:|---:|
| rank 1 | 98026.6 | 1664.4 | 1.656% | 0.497%–6.478% | 1.478% | 0.001281 m |
| rank 2 | 59209.6 | 887.1 | 4.962% | 0.473%–43.546% | 3.200% | 0.003049 m |
| rank 3 | 19550.7 | 502.9 | 19.600% | 0.471%–70.098% | 6.757% | 0.015515 m |

## 6. 可分性检查

观察：

1. rank 1 的 walking positive ratio 均值高于 sitting_static，但两者范围明显重叠；
2. rank 2、rank 3 的高 ratio 经常来自较小区域，两个序列都存在；
3. 大区域的 residual median 多数接近 0；
4. 仅使用区域面积、positive ratio 或 residual median，当前数据没有显示稳定分离边界；
5. `region_support.png` 中人物主体通常为低亮度；
6. 高亮区域更多位于图像上方细结构、深度空洞和物体边缘；
7. walking 与 sitting_static 的 support 空间形态相似。

重点图：

- `walking50_images/frame_000030_region_support.png`
- `sitting_static50_images/frame_000030_region_support.png`

因此不能从本次结果中客观推出类似：

```text
positive_ratio > X
```

即可得到可靠动态区域。

## 7. 运行开销

| 指标 | walking_xyz | sitting_static |
|---|---:|---:|
| region grow + 区域统计平均 | 4.924 ms | 4.868 ms |
| geometry total 平均 | 8.400 ms | 8.290 ms |

相比 G0-3 仅增长时约 `2.8 ms`，区域支持图填充和 residual 中位数统计增加了约 `2 ms`。
这是 shadow 诊断开销，尚未做实时优化。

采样帧同步保存 7 张 PNG 的写盘时间不计入 geometry total。

## 8. 日志

- `g0_3r_walking50.log`
  - SHA-256：
    `827a59a704be8c79b3db7900da3a29d0f6234ef615f9d7992acacbdec57ba9c5`
- `g0_3r_sitting_static50.log`
  - SHA-256：
    `2a32441b7e80add9e0c185567b5b87d15a5c2e33516b20d880abaf0266ca9b12`

## 9. 结论

G0-3R 没有证明当前深度区域能够通过一个简单 seed-ratio 阈值可靠区分动态对象和背景。

因此继续冻结以下门控：

- 不进入 ORB 特征投影；
- 不融合 semantic 与 region mask；
- 不修改 PoseOptimization、MapPoint 或地图写入；
- 不把某个仅对当前 50 帧看起来较好的阈值写入方法。

## 10. 下一步建议

当前瓶颈不是计算时间，而是几何证据和区域边界的判别性。

下一步应先准备更有判别力的验证数据：

1. 真正无动态的 RGB-D 序列；
2. 无已知语义类别、独立运动箱子的序列；
3. 静止箱子与运动箱子的成对场景；
4. 尽量包含沿视线和横向两种运动。

随后比较：

- 当前单上一帧参考；
- 更长时间间隔或关键帧参考；
- 多参考投票；
- 深度梯度/法向或超像素区域；
- 短时序 seed 稳定性。

在没有上述对照前，不继续调 region acceptance 阈值。
