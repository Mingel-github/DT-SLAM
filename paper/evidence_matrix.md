# DT-SLAM evidence matrix

This is a claim-control document. A row may be promoted into paper prose only
with its stated scope and qualification. `TBD` means it must be checked again
against the source result before writing.

| Candidate claim | Evidence and comparison | Scope / qualification | Status |
| --- | --- | --- | --- |
| Person semantic masking substantially improves localization in a dynamic-person sequence. | TUM `fr3/walking_xyz`, three-run medians: ORB-SLAM2 ATE 0.730574 m; semantic ATE 0.016477 m. | Supports this sequence and the current `person` class, not all dynamic categories. | Verified |
| Region geometry improves localization for a class-unknown moving box. | Bonn `moving_nonobstructing_box`, strict ON/OFF control, three-run medians: geometry OFF ATE 0.514344 m; S2 geometry ON ATE 0.022526 m; both 778/778 frames. | Strong repeated result for this sequence; does not establish universal category-agnostic detection. | Verified |
| Fail-open avoids a long tracking loss caused by over-filtering in a strong-occlusion sequence. | Bonn `moving_obstructing_box`: old S2 389 normal / 200 lost; new S2 + fail-open 589 normal / 0 lost. | It trades some localization gain for tracking continuity; mechanism should not be described as relabeling dynamic points as static. | Verified |
| Geometry is not uniformly beneficial and is not real-time in its current form. | AWS Small House: semantic has best ATE/RPE; geometry raises coverage but lowers speed to about 4.6 FPS. | Include as limitation, not as a headline success claim. | Verified |
| Framewise dynamic filtering alone does not reliably remove historical map artifacts. | AWS fixed-trajectory mapping: S3 direct accumulation retains 96.35% ghost proxy. | Mapping study is offline/fixed-trajectory, not an online DT-SLAM result. | Verified |
| Temporal voxel support can reduce dynamic residual artifacts. | AWS fixed trajectory: at-least-8-frame support + S3 retains 8.38% ghost proxy, structure proxy 98.17%. | Eight means distinct sampled-frame support, not consecutive-frame persistence; a slow/repeated dynamic object can pass. | Verified |
| OctoMap clearing depends on subsequent free-space observations. | AWS fixed trajectory: only 5.80% of old box voxels get later free-space revisit; S3 + OctoMap retains 36.31% ghost proxy. | Do not say OctoMap always removes residual objects. | Verified |

## Source records

- `results/DT-SLAM_正式主线_Method与Results汇报_2026-08-07.md`
- `results/DT-SLAM_成果收束与复现索引_2026-08-07.md`
- `results/aws_small_house_formal_2026-08-06/AWS_SMALL_HOUSE_FIXED_TRAJECTORY_MAPPING_RESULT.md`

## Rules before drafting

1. Use a repeated-median result for headline numerical claims whenever it is
   available; label single-run checks as scope checks.
2. State the baseline, metric, dataset/sequence, and direction of comparison
   with every central number.
3. Never upgrade a mapping audit conducted on a fixed trajectory into an
   online-system claim.
4. Add a new row before writing a new technical or numerical claim.
