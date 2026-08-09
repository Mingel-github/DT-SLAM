# DT-SLAM: working outline

Status: planning document, not submission prose.  
Venue: TBD (use CoRL-style defaults only until a target venue is chosen).  
Working system name: **DT-SLAM**.

## Reader-facing one-line problem

Dynamic objects corrupt feature correspondences and can leave historical
artifacts in RGB-D SLAM maps; semantic labels alone do not cover unknown moving
objects.

## Proposed paper arc

1. **Introduction**
   - Motivate dynamic RGB-D SLAM with people and class-unknown moving objects.
   - State the gap: semantic masking is effective for known categories but does
     not by itself identify unknown moving regions.
   - Introduce DT-SLAM and preview the semantic and region-geometric branches.
   - State only claims supported by the evidence matrix.

2. **Related Work**
   - Semantic dynamic SLAM.
   - Geometry-based / category-agnostic dynamic-region detection.
   - Dynamic-scene mapping and temporal occupancy filtering.

3. **Method**
   - System overview: YOLOv8-seg person masks, SInDSLAM-style region geometry,
     and ORB-SLAM2 integration.
   - Region geometry: 3-D K-means initialization, depth-boundary splitting,
     region-adjacency merging, dense-flow residuals, and region-constrained
     flood fill.
   - SLAM integration: dynamic feature exclusion, association cleanup,
     MapPoint creation control, and fail-open tracking safeguard.
   - Static-depth output for mapping.

4. **Experiments**
   - Datasets, modes, metrics (ATE/RPE), repeat protocol, and implementation
     details.
   - Known-person dynamics: TUM walking.
   - Class-unknown dynamics: Bonn moving box sequences.
   - Scope/range checks: static and AWS simulated scenes.

5. **Results and Analysis**
   - Semantic branch on TUM walking.
   - Geometry branch on the non-obstructing Bonn box sequence.
   - Fail-open behavior under strong occlusion.
   - Limits: geometry is scene-dependent and currently not real-time.

6. **Mapping Study**
   - Explain why framewise front-end filtering alone cannot guarantee removal
     of historical artifacts.
   - Compare direct accumulation, temporal voxel support, and OctoMap under a
     fixed trajectory.
   - State the dependence of OctoMap clearing on later free-space revisits.

7. **Conclusion and Limitations**
   - Recap only the bounded findings above.
   - Pair each limitation with a concrete future direction.

## Figures and tables to prepare later

- System pipeline figure: semantic branch, region-geometry branch, ORB-SLAM2
  integration, and static-depth output.
- Region-geometry explanatory figure: initial 3-D clusters, depth boundaries,
  merged regions, flow residual, constrained dynamic mask.
- Main localization table: clearly separate repeated results from single-run
  scope checks.
- Fail-open timeline/visualization for the obstructing-box sequence.
- Mapping comparison table with fixed-trajectory protocol stated in caption.

## Terminology to lock before drafting English prose

| Concept | Current canonical form | Note |
| --- | --- | --- |
| System | DT-SLAM | Confirm final expansion/title later. |
| Semantic branch | semantic masking | YOLOv8n-seg, currently `person` only. |
| Geometry branch | SInDSLAM-style region geometry | Not a full reproduction of SInDSLAM. |
| Integration module | fail-open tracking safeguard | A tracking safety mechanism, not a dynamic/static reclassification. |
| Mapping output | static-depth output | Dynamic pixels are zeroed for mapping output. |
| Temporal mapping rule | temporal voxel-support filtering | Current threshold uses distinct sampled frames, not eight consecutive frames. |
