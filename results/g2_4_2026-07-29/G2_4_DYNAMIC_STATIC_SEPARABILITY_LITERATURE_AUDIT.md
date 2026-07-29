# G2-4 动态/静态区分能力文献边界审计

日期：2026-07-29

## 1. 阶段命名

`G2-4` 是本项目新增的 `[S]` 阶段标签：

```text
G2-4 = geometry evidence dynamic/static separability shadow gate
```

它不是某篇论文的章节或方法名。

## 2. DetectFusion

原始论文：

```text
Hachiuma et al., DetectFusion: Detecting and Segmenting Both Known and
Unknown Dynamic Objects in Real-time SLAM
```

`[L]` 原方法：

- 从 static-map reference 与当前帧的 ICP 配准得到 geometric residual mask；
- 对 residual 值执行 `K=2` 的 K-means；
- centroid residual 较大的簇作为 dynamic cluster；
- 再与 geometric segments 计算 IoU，将局部 motion residual 扩散到较完整区域；
- 论文明确说明无运动物体时，binary motion mask 可能在 object edges 出现
  spurious dynamic pixels，其 geometric segment intersection 用于去除部分边缘伪点。

`[A]` DT-SLAM 与其不同：

- 当前输入是多参考帧 depth-warp signed residual vote，不是 dense-map ICP residual；
- 当前区域是 depth-discontinuity connected components，不是 DetectFusion 的
  normal/distance geometric segments；
- 当前没有 K=2 residual clustering，也没有 segment IoU motion propagation。

因此禁止写成：

```text
DT-SLAM reproduces DetectFusion motion segmentation
```

DetectFusion 支持的是“残差分布可以作为 motion cue，并且边缘伪证据必须控制”
这一组件级动机，不证明本项目 vote ratio 可直接二分。

## 3. SInDSLAM

原始论文：

```text
Qi et al., Semantic-Independent Dynamic SLAM Based on Geometric
Re-Clustering and Optical Flow Residuals, 2025
```

`[L]` 原方法：

- 计算 camera-induced optical flow 与 dense optical flow 的 residual；
- 使用 Triangle histogram 方法得到 `tau_low`；
- 定义 `tau_high = 1.3 * tau_low`；
- high residual、low residual 和 static 三类在单个 geometric cluster 内处理；
- low-residual 像素只有与 high-residual 区域连通时才传播；
- filled area 超过 cluster 的一半时才扩展到整个 cluster；
- 论文承认 illumination change 会导致静态区域误判。

`[A]` DT-SLAM 与其不同：

- 当前没有 optical flow residual 或 homography；
- positive vote ratio 不是 SInDSLAM 的 residual histogram；
- 当前 connected region 不是其完整 re-clustering；
- 不复制 `1.3`、`0.5` 等阈值。

SInDSLAM 支持“单固定阈值可能不可靠、传播应受区域和连通性限制、必须控制静态
误判”这些设计警告，不支持直接套用其阈值。

## 4. Bonn RGB-D Dynamic Dataset

`[L]` 官方数据页说明：

- 24 个动态序列和 2 个静态序列；
- 包含 moving/placing/removing/kidnapping box、balloon、crowd 等场景；
- 提供 OptiTrack camera-pose ground truth；
- 提供静态环境的 ground-truth 3D point cloud；
- depth 已注册到 RGB；
- RGB 相机具有非零畸变参数。

官方页面没有声明提供逐帧动态物体像素 mask。因此：

```text
Bonn camera-pose GT != dynamic-object mask GT
static 3D model != directly available per-pixel motion label
```

`[S/H]` Bonn moving box 适合未知类别压力测试，但若要报告动态像素 precision/
recall，仍需自动生成并审计对象标注，或设计基于 static model 的独立投影代理。

## 5. 对 G2-4 的约束

允许借用：

- `[L/A]` residual distribution 是 motion cue；
- `[L/A]` edge/occlusion/illumination 是伪动态来源；
- `[L/A]` 局部 residual 不能无条件扩散到整个 region；
- `[S]` 先做分布和混淆审计，再设计判决。

仍待验证：

- `[H]` 多参考 positive vote ratio 能区分真实动态与静态异常；
- `[H]` comparison support、region size、boundary distance 能解释主要混淆；
- `[H]` 现有 evidence 能发现 YOLO 类别之外的 moving box。

## 6. 结论

G2-4 整体是 `[S/H]`，不是 DetectFusion 或 SInDSLAM reproduction。当前只批准
只读 separability/risk audit，不批准动态阈值和 SLAM filtering。
