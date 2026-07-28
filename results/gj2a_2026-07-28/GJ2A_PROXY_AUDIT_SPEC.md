# GJ-2A Cluster–Person Proxy 自动审计规范

日期：2026-07-28

## 目标

不进行人工逐像素标注，不把YOLO mask输入SLAM，仅离线检查：

> GJ-2中重投影误差较大的cluster，是否比面积或随机排序更集中地覆盖person
> proxy区域。

该阶段仍是诊断，不产生动态cluster阈值，不修改SLAM。

## 输入

```text
GJ-2 reprojection CSV
GJ-1 raw cluster label PNG
已有离线YOLOv8 person proxy mask
```

raw cluster label使用无损`CV_16UC1` PNG：

```text
0 = invalid depth
1..K = cluster id + 1
```

person proxy来自Ultralytics离线后处理，已经过7×7膨胀。它不是运动真值：

- person内部可能暂时静止；
- person外可能存在未知动态或YOLO漏检；
- proxy与项目C++ YOLO输出不保证逐像素一致。

## 每cluster连续指标

```text
proxy_overlap_pixels
proxy_fraction_of_cluster
proxy_iou
proxy_recall_contribution
mean/median/P90 reprojection error
map support
optimizer outlier support
```

不使用“只要接触person就算动态”等二值规则。

## 排名评价

对每个有初始位姿且具有proxy覆盖的帧：

1. 主结果按`mean_squared_error_px2`降序，它对应当前显式声明的
   `rho(s)=s` identity baseline；`mean_error_px`只作为未平方诊断对照；
2. 统计Top-1、Top-3、Top-5覆盖的person proxy比例；
3. 计算这些cluster占全部被测cluster面积的比例；
4. 计算proxy capture相对面积占比的enrichment；
5. 使用固定seed 2021进行1000次cluster随机排列，得到随机Top-K proxy capture
   均值和经验p值；
6. 计算cluster error与person overlap fraction的Spearman相关。
7. 以`optimizer_outlier_support / matched_map_support`作为已有系统内部对照，
   检查cluster error是否只是在复述优化器离群比例。

这些是可分性诊断，不是动态判定阈值。

Ji论文没有公开`rho`。因此审计工具必须显式记录`error_field`，不能把
`mean_squared_error_px2`和`mean_error_px`的结果混称为同一个论文指标。

## 静态负样本

fr1/xyz没有person proxy时使用全零proxy，只验证：

- 工具不会生成伪person overlap；
- cluster error自身仍可能很高；
- 不能把高error直接等同于动态。

## 门控

只有同时观察到以下趋势，GJ-3才具备“开始讨论阈值”的条件：

- 多帧Top-K error cluster对person proxy有稳定正enrichment；
- 优于随机排序；
- Spearman总体为正；
- 结果不是由单个帧或单个巨大cluster主导。
- cluster error排序不能劣于简单的optimizer outlier fraction排序。

即使通过，也只能说明对person proxy有可分性，不能证明能检测未知动态箱子。
