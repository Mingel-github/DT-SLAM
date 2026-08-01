# Bonn 数据盘点与单轮 ATE/RPE 扩展结果

日期：2026-08-01

状态：数据准备与单轮补充实验完成；结果仅作扩展性观察，不替代三轮中位数正式评价。

## 1. 数据集来源与坐标域

Bonn RGB-D Dynamic Dataset 官方提供 24 条动态序列和 2 条静态序列，并为每条
序列提供 Optitrack 相机轨迹真值。所有本轮输入均使用项目现有的联合校正路径：

```text
RGB   : linear remap
depth : nearest-neighbor remap
output domain: P=K undistorted pinhole, 640x480
```

官方页面：

```text
https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/index.html
```

## 2. 本地数据盘点

压缩包已统一保存在：

```text
/data/dynaslam/archives
```

已解压、可直接运行的代表序列：

| 类型   | 序列                                                       |
| ---- | -------------------------------------------------------- |
| 箱子运动 | balloon、moving_nonobstructing_box、moving_obstructing_box |
| 箱子放置 | placing_nonobstructing_box、placing_obstructing_box       |
| 箱子移除 | removing_nonobstructing_box、removing_obstructing_box     |
| 极端变化 | kidnapping_box                                           |
| 语义动态 | balloon_tracking                                         |
| 静态安全 | static_close_far                                         |

已下载压缩包、尚未解压：

```text
balloon2
crowd
person_tracking
```

另外保存了 Bonn 静态环境的 1 mm 子区域点云压缩包。

本轮新下载：

```text
placing_obstructing_box
removing_obstructing_box
```

仍未下载但后续可能有价值的独立重复序列：

```text
moving_nonobstructing_box2
moving_obstructing_box2
kidnapping_box2
placing_nonobstructing_box2/3
removing_nonobstructing_box2
```

这些 `_2/_3` 序列适合在方法发生实质改进后作为独立复验集；当前没有必要为了
同一个冻结 q10 规则立即跑完全部 16.4 GB。完整 `static` 为 5.8 GB，当前已有
`static_close_far` 和 TUM `fr1/xyz` 静态检查，因此暂不下载。

## 3. 运行协议

每条序列运行一次：

```text
semantic-only
semantic+geometry
```

共同条件：

```text
Viewer                    OFF
YOLO provider             CUDAExecutionProvider
semantic mask age         0
geometry                  frozen sparse ego-flow q10
tracking safeguard        maximum removal 5%
mapping safeguard         maximum candidate depth 5%
Optimizer/g2o/YOLO        unchanged
```

ATE 使用无尺度修正的 SE(3) Umeyama alignment；RPE 使用相邻一帧的平移误差。

## 4. 单轮结果

| 序列                          | 模式                | 轨迹        | ATE RMSE (m) | RPE RMSE (m/frame) | FPS     | tracking removed | depth veto |
| --------------------------- | ----------------- | ---------:| ------------:| ------------------:| -------:| ----------------:| ----------:|
| moving_obstructing_box      | semantic-only     | 589/589   | 0.357476     | 0.071950           | 29.7067 | 0                | 0          |
| moving_obstructing_box      | semantic+geometry | 589/589   | 0.359484     | 0.067901           | 29.6934 | 286              | 101        |
| placing_nonobstructing_box  | semantic-only     | 720/720   | 0.831354     | 0.027258           | 29.6323 | 0                | 0          |
| placing_nonobstructing_box  | semantic+geometry | 720/720   | 0.862936     | 0.027822           | 29.4389 | 434              | 154        |
| placing_obstructing_box     | semantic-only     | 993/993   | 0.252696     | 0.044600           | 29.5658 | 0                | 0          |
| placing_obstructing_box     | semantic+geometry | 993/993   | 0.236567     | 0.037563           | 29.5469 | 876              | 46         |
| removing_nonobstructing_box | semantic-only     | 494/494   | 0.015280     | 0.026755           | 29.7062 | 0                | 0          |
| removing_nonobstructing_box | semantic+geometry | 494/494   | 0.016458     | 0.027537           | 29.6521 | 487              | 140        |
| removing_obstructing_box    | semantic-only     | 959/959   | 0.347111     | 0.026686           | 29.5888 | 0                | 0          |
| removing_obstructing_box    | semantic+geometry | 959/959   | 0.344927     | 0.027062           | 29.5540 | 825              | 193        |
| kidnapping_box              | semantic-only     | 1091/1091 | 0.028139     | 0.076136           | 29.5338 | 0                | 0          |
| kidnapping_box              | semantic+geometry | 1091/1091 | 0.028334     | 0.076238           | 29.4657 | 274              | 34         |
| static_close_far            | semantic-only     | 1750/1750 | 0.091187     | 0.135179           | 29.4835 | 0                | 0          |
| static_close_far            | semantic+geometry | 1750/1750 | 0.091960     | 0.135367           | 29.3035 | 2026             | 526        |

相对 semantic-only：

| 序列                          | ATE 变化 | RPE 变化  | FPS 变化 |
| --------------------------- | ------:| -------:| ------:|
| moving_obstructing_box      | +0.56% | -5.63%  | -0.04% |
| placing_nonobstructing_box  | +3.80% | +2.07%  | -0.65% |
| placing_obstructing_box     | -6.38% | -15.78% | -0.06% |
| removing_nonobstructing_box | +7.71% | +2.92%  | -0.18% |
| removing_obstructing_box    | -0.63% | +1.41%  | -0.12% |
| kidnapping_box              | +0.69% | +0.13%  | -0.23% |
| static_close_far            | +0.85% | +0.14%  | -0.61% |

负号表示误差或运行代价下降。

## 5. 客观解释

1. 七条序列的两种模式都输出完整轨迹，没有出现覆盖灾难。
2. `placing_obstructing_box` 单轮同时改善 ATE 和 RPE，是正面线索，但不是稳定因果结论。
3. `moving_obstructing_box` 的 ATE 基本相同、RPE 改善；不能仅凭该结果证明箱子检测准确。
4. `removing_nonobstructing_box` 两项均略退化，说明显露区域和强图像运动仍可能触发错误候选。
5. `static_close_far` 删除了较多关联，但轨迹误差只发生约 1% 内变化；这说明安全限制避免了明显退化，也说明候选并非对象特异的动态真值。
6. 所有序列的 FPS 差异小于 0.7%，当前轻量几何成本不是主要瓶颈。
7. 正负结果混合，且每个模式只运行一次，因此不支持“几何稳定改善 ATE”的声明。

## 6. Viewer 对实验的影响

Viewer ON 适合人工观察三维地图、关键帧和相机轨迹，但正式指标保持 Viewer OFF：

- Pangolin/OpenGL 增加渲染工作和 GPU/CPU 调度；
- ORB-SLAM2 是多线程系统，额外调度可能造成小幅轨迹波动；
- FPS 会包含或受到可视化开销影响；
- 当前 Viewer 在序列结束、轨迹保存后会触发已知关闭阶段段错误。

因此固定用途为：

```text
Viewer OFF : ATE/RPE/FPS 正式或补充性数值
Viewer ON  : 定性观察，不替代正式数值
```

Viewer ON 产生的完整轨迹仍可计算描述性 ATE，但不应与 Viewer OFF 的正式时间结果
混为同一组统计。

## 7. 当前结论

```text
dataset expansion                         COMPLETE
single-run paired ATE/RPE/FPS             COMPLETE
stable ATE improvement claim              NOT SUPPORTED
catastrophic trajectory regression        NOT OBSERVED
geometry runtime overhead                 SMALL
unknown-box object specificity            NOT PROVEN BY ATE
```

原始轨迹、日志、ATE/RPE 结果和过滤 CSV 位于本目录的各序列子目录。
