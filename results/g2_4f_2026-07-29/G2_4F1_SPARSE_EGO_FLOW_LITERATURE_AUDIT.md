# G2-4F1 稀疏 Ego-motion-compensated Flow 文献审计

日期：2026-07-29
状态：文献与方法边界冻结
范围：G2-4F0 负门控后的 failure-driven shadow evidence。

## 1. 结论

G2-4F0 已证明，当前 multi-reference positive depth evidence 在无人物箱子候选的
ORB feature 上没有局部富集。根据最初冻结路线，下一项合理的互补证据是：

```text
observed sparse image flow
- camera-motion-induced ego flow
= residual flow
```

该方向有明确文献原型，但第一版必须保持很小：

- 只使用相邻成功 RGB-D 帧；
- 只在当前 ORB feature 上运行稀疏 LK；
- ego flow 使用 RGB-D 深度和当前初始 SE(3) 位姿，而不是全局单应矩阵；
- 输出连续 residual 与有效性字段；
- 不设动态阈值、不与 depth score 融合、不写入 SLAM 状态；
- 有 GT pose 时同时输出 GT-pose residual，分离位姿误差影响；
- 在查看 residual 之前冻结独立 RGB 时序 motion-state 预标注。

方法身份应表述为：

> `[A/S]` FlowFusion optical-flow residual principle adapted to sparse
> current-frame ORB observations and an ORB-SLAM2 RGB-D initial pose.

不能称为 FlowFusion、SInDSLAM、DVI-SLAM 或 NGD-SLAM 复现。

## 2. 检索顺序与材料

先核对本地 PaperNotes 和原始 PDF：

```text
/home/zhu/Desktop/paper_notes/SInDSLAM.md
/home/zhu/Desktop/paper_notes/ngd_slam.md
/home/zhu/Desktop/paper_notes/dvi_slam.md
/home/zhu/Desktop/papers/Qi 等 - 2025 -
  Semantic-Independent Dynamic SLAM Based on Geometric Re-Clustering
  and Optical Flow Residuals.pdf
/home/zhu/Desktop/papers/2025_NGD-SLAM_CPU_Real-Time.pdf
/home/zhu/Desktop/papers/2026_DVI-SLAM_Instance_OpticalFlow.pdf
```

本地没有 FlowFusion 原始 PDF，因此只对该项使用 primary web fallback：

```text
Zhang et al., FlowFusion: Dynamic Dense RGB-D SLAM Based on Optical Flow,
ICRA 2020, arXiv:2003.05102
https://arxiv.org/abs/2003.05102
```

## 3. 文献原型

### 3.1 FlowFusion

`[L]` FlowFusion 输入连续 RGB-D 帧，先估计相机 ego-motion 和 dense optical
flow，再计算 camera ego flow。其核心物理关系是：

\[
\mathbf f_{\mathrm{res}}(\mathbf u)
=
\mathbf f_{\mathrm{obs}}(\mathbf u)
-
\mathbf f_{\mathrm{ego}}(\mathbf u).
\]

静态像素的 observed flow 主要来自相机运动，因此 residual 接近零；独立运动
像素具有非零 residual。论文使用 PWC-Net dense flow、supervoxel、联合
dense VO 与迭代动态分割。

它支持本项目：

- observed flow 与 ego flow 必须分开；
- RGB-D 深度和 SE(3) 可以直接生成 ego flow；
- flow residual 对横向图像运动是 depth residual 的合理互补。

它不支持直接照搬：

- dense PWC-Net；
- supervoxel 动态分割；
- 联合稠密位姿迭代；
- residual threshold 或融合权重；
- 实时性保证。

原文还明确报告非常慢或非常快的运动会因 optical-flow 估计失败而退化。因此
flow residual 必须保留 `no evidence/invalid correspondence` 状态。

### 3.2 SInDSLAM

`[L]` SInDSLAM 使用 consecutive grayscale images 的 dense optical flow，
将当前像素按 flow 对应到前一帧，并用 PROSAC 估计 homography 表达相机诱导
对应。其 residual 是 flow correspondence 与 homography correspondence 的
像素距离。

论文随后才执行：

- Triangle 双阈值；
- cluster 内 residual-aware flood fill；
- 时序动态先验；
- 每五帧 depth reprojection 地图精修。

完整系统为 `117.8 ms/frame`，约 9 Hz。

它支持：

- optical-flow residual 是类别无关运动证据；
- residual classification 不能脱离对应可靠性和区域约束；
- 光照变化会造成静态误检。

它不支持本项目立即加入：

- dense flow；
- PROSAC/homography；
- Triangle 阈值；
- re-clustering/flood fill；
- OctoMap。

本项目有 RGB-D 深度和 ORB-SLAM2 SE(3)，因此使用逐点 pinhole ego flow 比
全局平面 homography 更符合当前传感器模型；这是 `[A]`，不是 SInDSLAM 复现。

### 3.3 NGD-SLAM

`[L]` NGD-SLAM 使用 LK：

- 传播上一语义 mask 中采样的动态点；
- 非关键帧跟踪上一帧静态点；
- 以光流替代每帧 ORB matching 降低 CPU 成本。

它支持稀疏 LK 是可接受的轻量时序工具，但其动态点来源是已有语义 mask，且
LK 本身不承担 unknown-object dynamic discovery。因此不能用 NGD-SLAM
证明“LK residual 一定能发现未知箱子”。

### 3.4 DVI-SLAM

`[L]` DVI-SLAM 使用 GFTT/LK、homography 和帧差，再与 Mask R-CNN instance
联合判定。其 object decision 仍依赖语义实例和人为比例规则。

它支持 sparse LK 与相机运动补偿是成熟组件，但：

- homography 假设不适合直接替代 RGB-D SE(3)；
- 语义候选内验证不等于类别无关发现；
- 论文的动态比例阈值不应迁移到本项目。

## 4. 本项目公式

取上一成功帧 \(r\) 和当前帧 \(t\)。对当前 ORB feature
\(\mathbf u_t\)，先用 backward LK 得到上一帧对应 \(\mathbf u_r\)：

\[
\mathbf u_r
=
\operatorname{LK}(I_t,I_r,\mathbf u_t).
\]

observed forward flow 表示为：

\[
\mathbf f_{\mathrm{obs}}
=
\mathbf u_t-\mathbf u_r.
\]

读取上一帧米制深度 \(D_r(\mathbf u_r)\)，利用：

\[
T_{t\leftarrow r}
=
T_{cw,t}^{(0)}
\left(T_{cw,r}^{\mathrm{final}}\right)^{-1}
\]

生成静态点在当前帧的预测：

\[
\widehat{\mathbf u}_t
=
\pi\left(
T_{t\leftarrow r}
\pi^{-1}(\mathbf u_r,D_r(\mathbf u_r))
\right).
\]

camera ego flow：

\[
\mathbf f_{\mathrm{ego}}
=
\widehat{\mathbf u}_t-\mathbf u_r.
\]

residual：

\[
\mathbf f_{\mathrm{res}}
=
\mathbf f_{\mathrm{obs}}-\mathbf f_{\mathrm{ego}}
=
\mathbf u_t-\widehat{\mathbf u}_t.
\]

输出 \(r_x,r_y,\|\mathbf f_{\mathrm{res}}\|_2\)，但不把 magnitude 转成
dynamic/static。

## 5. 为什么从当前 feature 向上一帧做 backward LK

未来 G1-F 的操作单位是当前 `Frame::mvKeys`，所以本阶段需要让每条 residual
天然绑定一个当前 feature index。

```text
current ORB feature
→ backward LK 到上一成功帧
→ 读取上一帧 depth
→ 用初始 SE(3) 投影回当前帧
```

相比“追踪上一帧任意角点再寻找最近当前 ORB”，该路径：

- 不增加 feature association/nearest-neighbor 规则；
- 不改变 ORB 提取；
- 不丢失当前 feature index；
- 仍可通过 forward LK 返回当前帧得到 raw forward-backward error。

这是 `[S]` 接口选择，不是论文贡献。

## 6. 对应有效性与 unknown

第一版只记录：

- backward LK status；
- forward LK status；
- LK photometric error；
- forward-backward error；
- reference depth validity；
- ego projection validity；
- common pinhole-domain validity。

不新增“FB error 小于多少才静态/动态”的判决。OpenCV LK 内部
`minEigThreshold` 是数值求解有效性参数，不是 dynamic threshold；配置与版本
必须记录。

以下全部为 `no residual evidence`：

- LK 失败；
- 对应越界；
- reference depth 无效；
- ego projection 在相机后方或越界；
- 图像不在共同 pinhole domain；
- 上一成功帧缓存不存在；
- 当前初始位姿不存在。

不得将其解释成静态。

## 7. GT-pose 敏感性

当前初始位姿仍可能被动态观测或普通跟踪误差影响。若 Bonn/TUM 提供 GT，则对
同一 observed flow 同时计算：

```text
residual_slam_pose
residual_gt_pose
```

判断：

| GT residual | SLAM residual | 解释 |
| --- | --- | --- |
| 小 | 小 | 对应更符合静态自运动 |
| 小 | 大 | 初始位姿是主要误差源 |
| 大 | 大 | 独立运动、LK错误、深度/同步/遮挡均可能 |
| 大 | 小 | 需检查GT时间戳/坐标域或偶然抵消 |

GT residual 只用于 development 诊断，不能进入部署判决。

## 8. 独立 Motion-state 预标注

G2-4C/D 的 `target visible` 不是 `target moving`。在查看 G2-4F1 residual
之前，应冻结：

```text
agent_rgb_temporal_motion_proxy_v1
```

输入只允许：

- rectified RGB temporal clip；
- source frame id/timestamp；
- 已有 RGB-only center-frame coarse bbox，用于指出目标。

禁止读取：

- depth residual；
- flow residual/vector；
- geometry proxy role/rank/value；
- partition/region score；
- future dynamic decision。

标签：

```text
moving
stationary
uncertain
occluded_or_not_visible
```

每条记录保留 evidence frame range、confidence 和 reason。它仍是 agent
development proxy，不是 GT；uncertain 不得强制二分类。该方案不需要用户逐帧
手工标注，并避免用待测 flow residual 生成自己的标签。

## 9. 风险

- LK 的亮度恒常与小运动假设可能被光照、模糊和快速运动破坏；
- 弱纹理箱子可能没有可靠 LK；
- 遮挡/显露边界会产生真实但不等于对象运动的 residual；
- previous-successful frame 间隔可能因 tracking loss 过大；
- 初始位姿误差会污染整个背景；
- depth 取样落在边界时可能属于错误表面；
- ORB feature 多数不在箱子内部；
- 当前 coarse bbox 和 motion proxy 都不是 pixel GT；
- flow 额外成本尚未实测，不能预设 30 FPS。

## 10. 决策

可以批准 G2-4F1 的最小 shadow instrumentation，但必须先冻结 SPEC，并保持：

```text
dynamic_decision          = none
depth_flow_fusion         = none
direct_slam_state_mutation = none
strict_hold_out           = sealed
G1-F / G1-D               = locked
```

若 sparse residual 在独立 `moving` proxy 上仍无局部富集，或静态/uncertain
条件同样高，则冻结为负结果，不继续堆叠阈值。
