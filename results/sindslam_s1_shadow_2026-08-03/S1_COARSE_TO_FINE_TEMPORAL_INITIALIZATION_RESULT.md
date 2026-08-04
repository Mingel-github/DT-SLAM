# S1 coarse-to-fine 与时序初始化结果

日期：2026-08-03  
阶段：S1 内部第三个增量  
状态：工程与成本审计通过；仍无动态判决

## 1. 实现内容

在 `SInStyleInitialRegionClusterer` 中新增可切换路径：

```text
4 层最近邻米制深度金字塔
→ 最粗层 grid / previous / mixed 初始化
→ 每层 KMEANS_USE_INITIAL_LABELS
→ 最近邻 label 上采样
→ 全分辨率 initial labels
→ 提交下一帧 temporal prior
```

clean-room 差异：

- invalid 和 `z>=6 m` 不作为零三维点进入 K-means；
- 深度与 label 不做线性插值；
- 各层保持物理米制深度、XYZ 的 z 权重为 1；作者源码会额外按层缩放
  resized depth 并使用 `z*1.5`，因此这里不是其数值行为等价实现；
- 保存上一帧 initial labels，而不是作者 final re-clustered labels；
- 输入序号跳变和 Reset 显式回退网格；
- 新显露/无 prior 像素使用确定性网格补齐；
- 每层保存有效样本、prior/fallback、compactness 与 runtime。

作者对齐配置中：

```text
input 0 = grid，不提交
input 1 = grid，首次提交
input 2+ = previous 或 mixed
```

## 2. 测试

确定性测试新增覆盖：

- 4 层与奇数尺寸金字塔；
- previous prior 与新显露 grid fallback 守恒；
- 输入 index 跳变回退；
- Reset 后等价于新对象；
- 作者对齐的提交时机；
- K-means 前后 OpenCV RNG 状态不变。

测试输出：

```text
SIn-style shadow state and initial-region tests passed
```

TUM3 两次 30 帧运行的 `coarse_to_fine` label PNG 逐文件完全一致。

## 3. 30 帧结果

数据、association、作者 reference 与前一增量相同。

### 3.1 初始化状态

| 项目 | 结果 |
|---|---:|
| grid 初始化 | 2 帧（input 0/1） |
| mixed previous+grid | 28 帧 |
| previous prior coverage 均值 | 91.85% |
| previous prior coverage 中位数 | 98.48% |
| 4 层形状 | 80×60 → 160×120 → 320×240 → 640×480 |
| 每帧区域数 | 12 |
| native partition 可用 | 30/30 |
| 初始化/标签守恒违反 | 0 |
| actual SLAM removed | 0 |

所有连续帧均使用了 previous prior；之所以记录为 `mixed` 而不是 `previous`，
是因为无效深度、新显露像素或 resize 后无 prior 的样本按规范使用网格补齐。

### 3.2 区域表示的描述性指标

| 指标 | full strict | full same-budget | coarse-to-fine temporal |
|---|---:|---:|---:|
| 有效图像覆盖 | 66.64% | 66.64% | 66.64% |
| ARI vs author final | 0.5540 | 0.5325 | 0.5407 |
| NMI vs author final | 0.7527 | 0.7282 | 0.7505 |
| native boundary precision @2 px | 0.3483 | 0.3095 | 0.3515 |
| author boundary recall @2 px | 0.6377 | 0.6051 | 0.6474 |

`full strict` 使用 20 iterations、epsilon 0.001；后两者都使用公开代码行为
参照的 4 iterations、epsilon 0.07。因此三列不是动态准确率对照，而是初始
partition 的成本/结构消融。

粗到细版本没有出现相对于 same-budget full-resolution 的明显结构退化；
上述指标略高只能描述这 29 个作者 final label 帧，不能宣称普遍更准确。

### 3.3 成本分解

| 初始区域路径 | K-means mean | native total mean | 说明 |
|---|---:|---:|---|
| full strict | 30.59 ms | 33.17 ms | 20 iter / ε=0.001 |
| full same-budget | 11.41 ms | 14.01 ms | 4 iter / ε=0.07，从零开始 |
| coarse-to-fine temporal | 3.51 ms | 7.43 ms | 4 层 initial labels |

这组消融说明：

- 从 33.17 到 14.01 ms 的大部分收益来自放宽为作者公开代码的收敛预算；
- 在相同 4 iter / 0.07 下，四层金字塔、空间/上一帧初始标签与
  `KMEANS_USE_INITIAL_LABELS` 的组合把 native total 从 14.01 降到
  7.43 ms；当前实验不能把这 6.58 ms 再归因给其中某一个单项；
- 不能把 33.17→7.43 ms 全部归因于金字塔。

coarse-to-fine 各层 K-means mean：

```text
80×60   0.12 ms
160×120 0.18 ms
320×240 0.65 ms
640×480 2.56 ms
```

30 帧 paired shadow 的端到端结果：

```text
active_total mean = 33.12 ms
deadline_missed   = 16/30
actual_fps        = 27.30
```

该运行还含约 5 ms reference PNG replay、native label PNG 写入和审计，不能
当作完整 native SIn detector FPS。后续边缘、flow 与时序动态判决尚未计入。

## 4. 结论

通过：

- 有论文依据的 coarse-to-fine/上一帧初始化可在 clean-room 米制深度接口
  中稳定实现；
- 真实输入标签确定、prior/fallback 可解释；
- 相同收敛预算下，区域初始成本从 14.01 ms 降至 7.43 ms；
- 本轮没有修改任何 SLAM 观测或地图。

仍未通过/未回答：

- 当前 initial region 仍不是对象；
- 尚无 depth/plane edge split 和 RAG merge；
- 尚无 dense flow 与 dynamic state；
- 不能测 ATE 收益，也不能开放 S2。

## 5. 下一步

仍属于 S1：先实现仅有论文公式依据的 `gradient-depth split` shadow，并与：

```text
initial regions
author final labels
```

做成对边界/碎裂审计。平面边缘和 RAG 合并暂不同时加入，避免再次把多个
不可区分的改动塞在一个结果里。

原始证据：

- `coarse_to_fine_30.csv/.log`；
- `coarse_to_fine_30_audit.json`；
- `coarse_to_fine_30_reference_audit.json`；
- `coarse_to_fine_30_labels/`；
- `coarse_to_fine_30_repeat.csv/.log` 与 labels；
- `full_resolution_author_budget_30.csv/.log`；
- `full_resolution_author_budget_30_audit.json`。
