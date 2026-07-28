# GJ-3 动态判定参数来源审计

日期：2026-07-28
范围：Ji et al., ICRA 2021 的cluster动态判定；本阶段不实现过滤。

## 1. 审计结论

公开材料不足以严格复现Ji论文的动态cluster判定。

论文和官方arXiv源码明确给出：

```text
K-means cluster数量N=24
输入分辨率640×480
r_j = (1/m_j) Σ rho(||u_i - pi(T P_i)||²)
相对其他cluster误差较大的cluster判为动态
选择较低阈值，阈值取决于匹配特征的平均重投影误差
```

但没有公开：

```text
rho的具体形式与参数
阈值的数值
阈值与全帧平均误差之间的函数
平均误差按feature加权还是按cluster平均
cluster最小匹配支持数
m_j=0和低支持cluster的状态
K-means初始化、attempts、终止条件和随机种子
可运行的官方实现
```

因此：

```text
GJ-L：文献可复现部分，可继续保留
GJ-A：任何动态阈值实现都必须标为本工程适配
GJ-3实际过滤：当前不批准直接实现
```

## 2. 证据等级

| 标签 | 含义 |
| --- | --- |
| `[P]` | 论文PDF正文明确给出 |
| `[S]` | 官方arXiv源码明确给出 |
| `[O]` | 当前DT-SLAM代码或实验直接观察 |
| `[E]` | 为可执行、可复现而采用的工程选择 |
| `[A]` | 对论文未公开部分的显式适配 |
| `[H]` | 尚未验证的解释或候选，不能写成论文事实 |

## 3. 核对材料

优先本地材料：

```text
/home/zhu/Desktop/paper_notes/Ji2021_RealTime_Semantic_RGBD_SLAM.md
/home/zhu/Desktop/paper_notes/comparison_23_papers.md
/home/zhu/Desktop/papers/Ji 等 - 2021 -
  Towards Real-time Semantic RGB-D SLAM in Dynamic Environments.pdf
```

本地PDF SHA-256：

```text
8d712db79ebb156f06b8ea844f61533c6e011c12a4ba4e70edfdbfb778962505
```

本地没有论文源码、补充材料或作者实现，因此进一步检查了：

```text
官方arXiv页面：https://arxiv.org/abs/2104.01316
官方arXiv e-print源码包
IEEE DOI：10.1109/ICRA48506.2021.9561743
GitHub标题、arXiv编号和作者名检索
```

官方e-print源码包 SHA-256：

```text
eb9be961e39a998450d3db5bbc9800b5fa38dfd5c34436f51c13f13472e8a9df
```

源码包只包含：

```text
root.tex
References.bib
IEEE模板
论文图像
```

没有supplement、参数表、伪代码或实现源码。截至本次审计，没有找到可确认由作者
发布的官方实现。该表述是“当前检索未找到”，不是证明代码绝对不存在。

## 4. PaperNotes与原文核对

PaperNotes中以下描述成立：

- `[P/S]` 每帧深度图在3D空间做K-means；
- `[P/S]` `N=24`用于`640×480`；
- `[P/S]` 对cluster内已匹配地图点的特征计算平均重投影误差；
- `[P/S]` 误差较大的cluster整体删除；
- `[P/S]` 几何模块依赖匹配地图点，支持不足会导致检测区域不完整；
- `[P/S]` 论文报告geometry约`30.14 ms`、tracking约`75.82 ms`；
- `[P/S]` false negative被认为比false positive更不利，因此作者选择较低阈值。

需要收紧的表述：

1. 不能把“较低阈值”写成某个已知数值；
2. 不能说论文使用identity、Huber或Student-t `rho`；
3. 不能把“阈值取决于平均重投影误差”扩写成某个具体倍数公式；
4. 不能说论文要求某个最小cluster支持数；
5. “代码未公开”应写为“当前未找到作者官方实现”；
6. 论文使用`T_wc`命名，但没有给出足够代码级坐标约定，当前DT-SLAM使用
   ORB-SLAM2的`Tcw * Pw`属于经测试的工程落地，不能从下标名称直接推导。

## 5. 原始论文逐项来源账本

| 项目 | 原文状态 | 证据 | 当前处理 |
| --- | --- | --- | --- |
| 深度3D K-means | 明确 | Section III-B | 已实现 |
| `N=24` | 明确 | Section IV-b | 已实现为论文参数 |
| `640×480` | 明确 | Section IV-b | TUM中满足 |
| cluster可切分同一物体 | 明确 | Section III-B | 已记录边界 |
| 只使用matched features | 明确 | 公式前定义`m` | 已使用正式MapPoint快照 |
| `rho(||e||²)`均值 | 明确到符号 | Equation (1) | 已输出平方与未平方统计 |
| `rho`类型 | 未公开 | 只称penalty function | identity仅为`[E]` |
| 动态判定 | “相对更大” | Section III-B | 只做排序审计 |
| 阈值策略 | “较低，取决于平均误差” | Section IV-b | 尚未实现 |
| 阈值数值 | 未公开 | PDF和TeX均无 | 不补写 |
| 阈值完整公式 | 未公开 | PDF和TeX均无 | 不补写 |
| 最小匹配支持 | 未公开 | 无条件或参数 | `m=0`保持unknown |
| 低支持cluster | 未公开 | 只承认检测不完整 | 原样输出support |
| K-means初始化/迭代 | 未公开 | 无实现细节 | OpenCV配置为`[E]` |
| 官方实现 | 未找到 | e-print无代码，检索无官方仓库 | 不声称代码复现 |

## 6. 本次发现的GJ-2A口径问题

Ji公式是：

```text
r_j = mean(rho(squared_reprojection_error))
```

当前GJ-2声明：

```text
rho(s) = s
```

因此严格对应这个identity工程baseline的cluster score应为：

```text
mean_squared_error_px2
```

此前GJ-2A主排序使用`mean_error_px`。它是有意义的诊断量，但不等于
identity-`rho`下的论文公式。

现已修正审计工具：

```text
默认error_field = mean_squared_error_px2
可显式选择mean_error_px、median_error_px或p90_error_px
summary.json必须记录error_field
```

100帧walking复核：

| 指标 | mean squared error | mean error |
| --- | ---: | ---: |
| 全局Spearman | 0.308 | 0.320 |
| Top-1 proxy capture | 12.79% | 9.64% |
| Top-3 proxy capture | 36.05% | 33.23% |
| Top-5 proxy capture | 53.69% | 53.99% |
| Top-3相对随机增益 | 2.14× | 1.98× |

两种统计的方向一致，因此GJ-2A“存在有限排序信号”的结论保留；但主结果现在以
identity-`rho`对应的mean squared error为准。

## 7. 最小支持数的严格解释

论文公式中：

```text
m = cluster内matched features数量
```

数学上只在`m>0`时有定义。论文没有给出：

```text
m >= 2
m >= 3
m >= 5
或按cluster面积归一化的支持率门控
```

因此当前：

```text
m=0 → unknown / no geometry evidence
m>0 → 记录score和support，但不自动判动态
```

是比“无支持即静态”更安全的工程约定，但仍是`[E]`，不是论文明确的三状态模型。

不能因为单点cluster噪声较大就直接新增`m_min=3/5/10`。如果未来需要支持门控，
必须单独做support-stratified validation，并标记为`[A]`。

## 8. 阈值文字能支持到什么程度

原文同时使用两种表述：

```text
cluster error is relatively larger than the others
threshold depends on the average reprojection error of matched features
```

这支持以下有限判断：

- `[P/S]` 阈值不是纯粹与场景无关的固定常数；
- `[P/S]` 判定考虑当前匹配误差的相对尺度；
- `[H]` 可能使用全帧平均误差构造自适应阈值。

但不能确定：

```text
tau = lambda * 全部feature的平均误差
tau = lambda * cluster error均值
tau = 平均值 + lambda * 标准差
tau = 某个percentile
或其他规则
```

即使采用最接近文字的：

```text
tau_t = lambda * frame_mean_error
```

`lambda`、frame mean的加权方式和`rho`仍未公开。这只能命名为：

```text
GJ-A relative-threshold adaptation
```

不能命名为Ji论文复现。

## 9. 对可选修补方案的判断

### 9.1 固定像素阈值

驳回。

依据：

- 不符合论文“取决于平均误差”的文字；
- 当前walking与fr1/xyz静态误差长尾明显重叠；
- 静态cluster mean error也可达到约`50 px`。

### 9.2 每帧固定Top-K cluster

驳回。

依据：

- 论文没有固定动态cluster数量；
- 完全静态帧也会被迫删除K个cluster；
- GJ-2A的Top-K只是排序评价，不是分类规则。

### 9.3 直接加入任意`m_min`

驳回。

依据：

- 论文没有公开最小支持数；
- 低支持确实是已知风险，但数值需要独立验证；
- 任意门控会改变unknown覆盖率和动态召回。

### 9.4 使用MAD/Student-t

不作为GJ严格baseline。

MAD和Student-t可由Li and Lee 2017的static point weighting得到文献依据，但原方法
作用于三维点对应距离和IAICP权重，不是Ji cluster reprojection判定。迁移到当前
score属于`[A]`，应归入G2-Robust，而不是用于填补Ji未公开的`rho`。

### 9.5 相对全帧平均误差的阈值族

可作为未来GJ-A候选，但暂不直接实施过滤。

它最接近Ji的文字描述，但仍必须显式承认：

- threshold函数是解释性适配；
- `lambda`是待标定参数；
- 不等于作者实现；
- 必须在独立calibration split上选择参数。

## 10. 建议的下一开发门控

不直接修改tracking。下一张最小工单应是：

```text
GJ-3A Threshold-Family Shadow Audit
```

只增加离线或shadow统计，比较：

```text
score:
  mean_squared_error_px2（identity-rho主项）

candidate normalization:
  support-weighted frame mean
  unweighted measured-cluster mean

candidate lambda:
  只形成离散参数扫描，不预先宣布最佳值

state:
  m=0保持unknown
  m>0只输出candidate decision
```

验收必须使用预先冻结的数据拆分：

1. calibration：选择`lambda`；
2. validation：报告person proxy排序/分类指标；
3. static validation：约束静态误删；
4. final TUM ATE/RPE：不得参与选参；
5. Bonn或自采运动箱子：单独验证类别无关能力。

在threshold shadow通过前：

```text
不写Frame动态状态
不清除mvpMapPoints
不创建额外PoseOptimization
不阻止MapPoint写入
```

## 11. 最终审批意见

批准：

- GJ-1/GJ-2作为Ji文献可追溯的测量baseline；
- GJ-2A改用显式`error_field`并以mean squared error为identity主结果；
- 将论文未公开项完整记录为unknown；
- 设计GJ-3A离线阈值族shadow审计。

不批准：

- 声称当前已经完整复现Ji动态判定；
- 使用固定像素阈值或固定Top-K进入SLAM；
- 把identity `rho`、任何`lambda`或`m_min`写成Ji论文参数；
- 用同一批100帧同时选参和报告性能；
- 在threshold shadow前进入GJ实际过滤。

当前合理状态：

```text
Ji baseline的测量和排序证据成立；
Ji baseline的二值动态判定因公开参数不足而不可严格复现；
下一步只能做透明、可审计的GJ-A shadow适配。
```
