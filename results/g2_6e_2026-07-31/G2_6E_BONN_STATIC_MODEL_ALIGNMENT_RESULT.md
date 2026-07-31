# G2-6E Bonn 静态模型对齐结果

日期：2026-07-31
结论：G2-6E1 未通过；停止 E2/E3，不生成 moving-box unknown-foreground proxy

## 1. 结论

Bonn 官方静态环境点云、官方 GT pose 和当前 registered depth 不能在现有
坐标信息下直接形成可信的逐像素 unknown-foreground 评价代理。

可以确认：

- 官网变换的逆矩阵是把 GT model 投到当前相机的正确基本方向；
- stride 抽样点云需要最小深度 splat 才能获得足够像素覆盖；
- RGB/depth/model 都位于已验证的 rectified `P=K` 域；
- 首段和部分返回视角的静态背景残差较小。

但全序列存在与相机位置相关的系统性单符号残差。在若干具有大量有效深度的
真静态帧中，即使排除 current/model depth boundary 的 2 像素邻域：

```text
non-risk residual median       约 0.10–0.44 m
non-risk r_model > 0.10 m      约 50%–94%
```

这违反 SPEC 中：

> 静态 residual 不表现为大面积单符号系统偏差

的 E1 放行条件。因此：

```text
G2-6E1 static alignment      FAIL
G2-6E2 moving-box proxy      NOT RUN
G2-6E3 F1 join               NOT RUN
G1-F / G1-D                  LOCKED
SLAM state mutation          NONE
```

不能通过 moving-box 数据调坐标链、残差 offset 或阈值来挽救该代理。

## 2. 官方资产

来源：

```text
https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/
```

下载资产：

| 资产 | 大小/内容 | SHA-256 |
|---|---:|---|
| `rgbd_bonn_groundtruth_1mm_section.zip` | 676,032,657 bytes | `1ce515267759537eb534ba14327f81e98a3459c7d956bd4b23a9964f69467d35` |
| ZIP 内 ASCII PLY | 2,318,666,764 bytes；54,676,774 points | 由 ZIP hash 固定 |
| `compute_global_transformation.py` | 官方坐标脚本 | `913cb25ac0502bf3933d5e4881cac5a864265dfba1c2b753c581f31b13e25868` |
| `Palazzolo_2019_ReFusion_IROS.pdf` | 官方论文 | `e140c00006c1c258f6e3ebadcd0eb5322de943fe5cfce0643d73b49a34ed5199` |

PLY header：

```text
format ascii 1.0
element vertex 54676774
property float x/y/z
property uchar red/green/blue
property float scalar_Scalar_field
```

## 3. 紧凑模型准备

完整 ASCII PLY 不直接解压。工具从 ZIP 流式读取，按零基 vertex index 固定步长
抽取，输出 `Nx3 float32 .npy`：

```text
DT-SLAM/tools/prepare_bonn_static_model.py
```

| 表示 | 点数 | NPY SHA-256 |
|---|---:|---|
| stride 16 | 3,417,299 | `844638984c2e9a35a59e5d9ed200b07c2914ed0e05e747a70282ddecb8ecedd1` |
| stride 8 | 6,834,597 | `96b2337af91d781dd4d4ab45bfd29fff3ff75028ebac222006deb1747f493778` |

这是 `[S]` 评价数据准备，不是 detector，也不声称等价于完整点云。

## 4. 坐标链

官网给出：

```text
T_g = T_ROS^-1 * T_0 * T_ROS * T_m
```

官方脚本写为：

```text
T_g = T_ros * T_0 * T_ros * T_m
```

因为给定 `T_ROS` 自逆，二者一致。

逐帧审计使用：

```text
T_model_from_camera(i) = T_ROS * T_i * T_ROS * T_m
T_camera_from_model(i) = inverse(T_model_from_camera(i))
```

其中：

- `T_i` 在 depth timestamp 上用 translation linear interpolation 和
  shortest-path quaternion SLERP 得到；
- 官方 `T_m` 的约 `1.0593` uniform scale 完整保留，没有强制正交化；
- current depth 使用 nearest-neighbor rectification；
- 投影域为官方 `K` 的无畸变针孔域。

首帧只读方向检查：

| 方向 | joint pixels | residual median | `abs(residual)<0.1m` |
|---|---:|---:|---:|
| `inverse(T_g)` | 180,271 | 0.035 m | 69.0% |
| 错误的直接 `T_g` | 223,768 | 2.102 m | 2.2% |

因此矩阵求逆方向没有歧义；失败发生在更严格的全序列逐像素一致性，而不是简单
正逆矩阵写反。

## 5. 首次失败运行

首次 stride16/splat1 运行发现：

```text
cv::erode(INF depth)
→ isolated all-INF neighborhood becomes FLT_MAX
→ np.isfinite mistakenly accepts it
→ pooled mean = Infinity
```

该运行完整保留：

```text
e1_static_stride16_splat1_failed_infinity_validity/
```

修正后：

```text
valid = finite && depth < 0.5 * FLT_MAX
```

自测试、编译检查和重跑通过。没有通过忽略 warning 继续。

## 6. 静态模型抽样与 splat 对照

完全使用同一真静态序列、同一批每 50 帧抽取的帧；不查看 moving-box 数据。
34 帧具有可插值 GT。

以下 pooled 指标均排除了 current/model depth edge 的 Chebyshev 距离 2 邻域：

| 模型 | splat 半径 | coverage median | residual median | MAD | `abs(r)<0.1` | `r>0.1` | render median |
|---|---:|---:|---:|---:|---:|---:|---:|
| stride16 | 0 | 65.55% | 0.127 m | 0.109 m | 43.00% | 56.23% | 154.0 ms |
| stride16 | 1 | 90.54% | 0.065 m | 0.078 m | 55.66% | 42.60% | 152.3 ms |
| stride16 | 2 | 98.53% | 0.042 m | 0.066 m | 59.76% | 37.75% | 152.1 ms |
| stride8 | 1 | 97.35% | 0.055 m | 0.073 m | 57.52% | 40.53% | 286.5 ms |

解释：

- splat 可以修复由 stride sampling 产生的像素孔洞；
- stride8 可以提高覆盖，但计算约翻倍；
- 更密点云和更大 splat 都没有消除静态单符号残差；
- 因此问题不能归因于 stride16 孔洞，也不能靠继续扩大 splat 修正；
- render time 仅为 Python 离线评价工具成本，不是 SLAM 实时预算。

## 7. 全序列失败证据

stride16/splat1 的代表帧：

| frame | current valid | joint coverage | non-risk residual median | MAD | non-risk `r>0.1` |
|---:|---:|---:|---:|---:|---:|
| 50 | 250,478 | 97.8% | -0.008 m | 0.024 m | 2.9% |
| 300 | 249,541 | 97.7% | 0.018 m | 0.030 m | 11.0% |
| 500 | 252,409 | 78.0% | 0.102 m | 0.083 m | 50.6% |
| 600 | 247,665 | 69.0% | 0.327 m | 0.144 m | 93.4% |
| 650 | 250,584 | 62.1% | 0.343 m | 0.154 m | 92.9% |
| 900 | 255,061 | 98.6% | 0.006 m | 0.021 m | 3.4% |
| 1250 | 230,676 | 98.6% | 0.182 m | 0.108 m | 73.5% |
| 1450 | 249,854 | 55.9% | 0.442 m | 0.199 m | 93.8% |
| 1700 | 252,825 | 97.6% | 0.004 m | 0.018 m | 2.7% |

残差随视角/位置明显变化，返回部分视角后重新变小。`static_close_far` 中还有
静止箱子、机器人和家具等可能未属于 scanner static model 的物体，所以
model inconsistency 本来就不等于 motion。更关键的是，调试图显示较大残差
并不只局限于一个可明确隔离的动态对象边界，无法直接作为逐像素 motion GT。

## 8. 为什么不继续进入 E2

若此时在 moving-box 序列上生成：

```text
z_expected_static - z_current > threshold
```

它将混合：

- 真正的新前景/移动物体；
- 静止但不属于 scanner static model 的物体；
- 模型采样和遮挡边界；
- 模型—序列坐标/尺度误差；
- RGB-D 深度噪声和无效测量。

对该 residual 做 per-frame offset、ICP 或 moving-data threshold fitting 都是新的
方法扩张。尤其 ReFusion 原方法使用 TSDF 初始 registration residual、自由空间
和第二次 registration；当前离线点云投影没有这些条件。不能为了保住评价代理
而把它包装成 ReFusion。

因此按预先冻结的失败处理：

> 若静态模型 proxy 本身不可靠，冻结它为失败评价路线，不用其调 detector。

## 9. 修改与未修改范围

新增：

```text
DT-SLAM/tools/prepare_bonn_static_model.py
DT-SLAM/tools/audit_bonn_static_model_alignment.py
results/g2_6e_2026-07-31/
BONN/rgbd_bonn_groundtruth_1mm_section.zip
```

本阶段没有修改：

- `Tracking.cc/.h` 中的既有 counterfactual 逻辑；
- `GeometricDynamicDetector`；
- YOLO；
- `Optimizer.cc`、g2o；
- LocalMapping、LoopClosing；
- `mvbDynamic`、`mvpMapPoints`；
- 任何真实过滤或地图写入。

## 10. 下一步建议

G2-6E 已排除“直接用 Bonn scanner model + GT pose 生成逐像素 unknown-motion
proxy”这一低成本评价路线。

下一步不能自动转成 ICP/TSDF，也不能继续调 q。需要在下列两项之间重新选择：

1. 用更多具有明确可见运动的 Bonn development 序列，继续 Agent 自动 RGB
   时序审阅和 object-level 粗标注，扩大独立运动样本；
2. 接受评价标注不足这一限制，另立有文献依据的 multi-feature/short-temporal
   motion grouping shadow SPEC，但其结果只能是开发期可行性，不能宣称真实
   unknown-object precision/recall。

在作出该选择前：

```text
G1-F / G1-D remain locked.
```
