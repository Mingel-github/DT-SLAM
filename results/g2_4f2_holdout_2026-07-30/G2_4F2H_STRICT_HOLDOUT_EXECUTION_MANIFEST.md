# G2-4F2H Strict Holdout 执行清单

冻结时间：2026-07-30  
状态：本文件写入时 archive 尚未打开

## 冻结版本

```text
git_commit:
  f964cf0b170fa785252cccb7499fab40afe9540b
git_branch:
  main
git_status_before_unseal:
  clean; ahead of origin/main by 1
```

## 冻结输入

```text
archive:
  /home/zhu/dynaslam_ws/BONN/rgbd_bonn_balloon_tracking.zip
archive_sha256:
  3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421

config:
  DT-SLAM/Examples/RGB-D/BONN_GeometrySparseEgoFlowShadow.yaml
config_sha256:
  3a2f8ef13a05bccb0ef1750a91d6a7aec75568dacc5f0cf5880e249fb068d97a

development_audit_tool_sha256:
  05951b862028872c0733e3eb1226e62978229f2a9826d02582108f2acb3e2c4c
static_risk_tool_sha256:
  8575f5cecaf30485cb3ae9b53bfe665665c3be59eeca41f4f1d01eb92b0126a1
```

## 冻结工作点

```text
FB threshold                  = 0.25 px
normalized residual threshold = 10
scale                         = max(0.001 px, 1.4826 * median(r))
minimum scale support         = 20
semantic features in scale    = excluded
boundary veto                 = none
raw residual threshold        = none
dynamic decision              = none
direct SLAM mutation          = none
```

## 冻结评价门

以
`G2_4F2D_CANDIDATE_WORKING_POINT_FREEZE_AND_HOLDOUT_PROTOCOL.md`
为唯一判定协议：

```text
measurable moving exact-zero-person proxy frames >= 5
frames with in-box candidate                    >= 80%
frames with inside rate > outside rate          >= 80%
full-sequence MapPoint candidate rate           <= 0.20%
invariant violation                             = 0
```

若 RGB-only proxy 不足，报告 `not evaluable`，不得放宽。

## 解封不变量

```text
一次完整运行；
不根据 holdout 修改 FB/q；
不挑选成功帧替代完整结果；
如发现实现 bug，先保留并宣布本次无效；
即使通过也只允许进入 G1-F0 mutation shadow。
```

## 解封与执行事件

```text
archive SHA-256 rechecked:
  3c63ec5d06ffc7b97f2f3f965f4bdf7e52b72f38cd98e0b532456e0ef7e3c421
association pairs:
  590
missing depth references filtered:
  3
RGB-only proxy frozen before geometry:
  14 moving_observable, 14 exact-zero person mask
formal semantic+F1 geometry runs:
  1
threshold retuning after run:
  none
```

结果见：

```text
G2_4F2H_RGB_ONLY_PROXY_FREEZE.md
G2_4F2H_STRICT_HOLDOUT_RESULT.md
```
