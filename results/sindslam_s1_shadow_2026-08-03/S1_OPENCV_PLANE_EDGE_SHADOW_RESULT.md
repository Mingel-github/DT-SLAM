# S1 OpenCV plane-edge substitute shadow 结果

日期：2026-08-03  
状态：实现和审计完成；替代方案未通过区域质量/成本条件，默认关闭

## 1. 实现身份

本轮实现的是：

> `[A] OpenCV RgbdPlane substitute for SIn-style plane-edge evidence`

SInDSLAM 使用 PEAC 平面轮廓；作者仓库内相应 PEAC/AHC 文件为
AGPL-3.0-or-later。本轮没有复制该代码，而是使用 DT-SLAM 已链接的 BSD
OpenCV `rgbd` 模块生成平面标签，再实现论文描述的 gradient endpoint
支持规则。

本实现仍为 shadow-only：没有动态判决，没有修改 `mvbDynamic`、
`mvpMapPoints`、Optimizer 或地图写入。

## 2. 实现与验证

- 新增独立 `SInStylePlaneEdgeRegionSplitter`；
- 输入为米制深度、K、initial labels 和 raw gradient edge；
- 输出 plane labels、raw/retained plane boundary、combined edge 和 combined
  split labels；
- RAG 在该开关打开时使用 combined split/edge，关闭时保持 gradient-only；
  论文独立 pair-level `M_rej` 尚未实现并明确报告 unavailable；
- 合成测试覆盖 endpoint-supported 保留、无 endpoint 丢弃、invalid 保持
  unknown、combined edge 切分、确定性和 OpenCV backend smoke；
- `sin_style_shadow_test`、`rgbd_tum` 构建通过；
- 修复并测试了 plane 开关关闭时不得校验不可用输入的回归；旧的
  reference-only 配置完成 10 帧运行，所有 native stage 保持关闭；
- TUM3 walking 30 帧运行两次，全部 plane/RAG PNG 逐像素一致 30/30；
- CSV 与图像 invariant 无错误；
- `dynamic_decision=none`、`actual_slam_removed=0`、
  `direct_slam_state_mutation=none`；
- 默认关闭 30 帧无 SIn 输出，`deadline_missed=0/30`、
  `actual_fps=27.94`。

两次 CSV 不逐字节相同是正常的，因为其中包含 runtime；结构图像是逐像素
确定的。

## 3. 30 帧定量结果

| 指标 | gradient-only RAG | OpenCV plane substitute + RAG |
| --- | ---: | ---: |
| gradient/combined component 中位数 | 72.0 | 437.5 |
| RAG output region 中位数 | 60.5 | 408.5 |
| component 相对 gradient 平均放大 | 1.0× | 6.01× |
| 有效深度 plane coverage 中位数 | — | 82.90% |
| raw plane-boundary pixels 均值 | — | 40,318 |
| retained plane-boundary pixels 均值 | — | 14,046 |
| retained segment fraction 均值 | — | 10.30% |
| RAG vs author final ARI | 0.7438 | 0.7216 |
| RAG vs author final NMI | 0.8338 | 0.7801 |
| boundary precision @2px | 0.3330 | 0.2097 |
| author boundary recall @2px | 0.8405 | 0.8751 |
| plane stage total 均值 | — | 16.86 ms |
| RAG total 均值 | 29.78 ms | 87.96 ms |
| actual FPS | 10.27 | 5.78 / 5.79 |

作者 final labels 只是行为参照，不是真值。即便只作描述性比较，OpenCV
替代仅略微提高作者边界召回，却明显降低边界 precision、ARI 和 NMI，并将
区域数量放大约六倍。

gradient-only 数值已在 `RankWeight=min(rank_i,rank_j)` 修正后的当前二进制
上重跑；此前较早生成、使用旧 rank 公式的 CSV 不再用于本表对照。

## 4. 原因解释的证据边界

已观察到的事实是：

- OpenCV 在每帧给出约 19–30 个平面、约 4 万个 raw boundary pixels；
- 论文式 endpoint 支持只保留约一成 boundary segments，但这些 segment
  很长，仍保留约 1.4 万像素；
- 大量保留边界将 initial region 切成数百个 component；
- 当前 RAG 禁止同一 initial region 内被 real edge 切开的 component 回并，
  但尚无论文式 pair-level plane `M_rej`；
- RAG 数据结构和候选重算成本随 component 数显著增长。

这支持“当前 OpenCV label-transition 与 PEAC contour/论文端点规则组合不
匹配”的判断。不能据此断言 OpenCV 平面检测本身错误，也不能断言 SInDSLAM
的 PEAC plane edge 无效。

## 5. 决策

根据实现前冻结的停止条件，本 OpenCV substitute 记录为失败替代方案：

- 保留代码、配置、测试和结果，默认关闭；
- 不调整 block size、面积、端点数、膨胀或 RAG 阈值来追逐结果；
- 不将其输出用于动态 mask、S2 Tracking 或 S3 Mapping；
- gradient-only RAG 继续作为较轻的区域表示 shadow；
- S1 下一项可独立推进 dense observed-flow 与 camera-induced-flow residual，
  但在形成可靠区域/时序判决前仍不得开放 S2。

## 6. 证据文件

- `S1_PLANE_EDGE_LITERATURE_SOURCE_DEPENDENCY_AUDIT.md`；
- `S1_OPENCV_PLANE_EDGE_SHADOW_SPEC.md`；
- `plane_edge_30_final.csv/.log` 与 repeat；
- `plane_edge_30_audit.json`；
- `rag_merge_30_current.csv/.log` 与
  `rag_merge_30_current_audit.json`（同版本 gradient-only 对照）；
- `reference_only_regression_10.csv/.log`；
- `plane_edge_30_final_outputs/` 与 repeat；
- `plane_edge_30_final_rag_outputs/` 与 repeat；
- `plane_edge_default_off_30.log`。
