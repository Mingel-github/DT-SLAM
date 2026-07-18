# DT-SLAM: Dual-Track Dynamic SLAM

Dynamic SLAM system based on ORB-SLAM2, using a loosely-coupled dual-track architecture for robust tracking in dynamic environments.

## Components

| Directory           | Purpose                                                            |
| ------------------- | ------------------------------------------------------------------ |
| `DT-SLAM/`          | Main development target — ORB-SLAM2 + dynamic filtering            |
| `ORB_SLAM2/`        | Original ORB-SLAM2 (pure geometry baseline)                        |
| `DynaSLAM/`         | Reference implementation (Mask R-CNN + geometry, BertaBescos 2018) |
| `ORB_SLAM3/`        | Reference (visual-inertial, not primary)                           |
| `pangolin_install/` | Pangolin visualization library                                     |
| `TUM/`              | TUM RGB-D dynamic sequences (fr3)                                  |
| `results/`          | Experiment outputs                                                 |

## TUM Sequences

- `rgbd_dataset_freiburg3_walking_xyz` — main dynamic test
- `rgbd_dataset_freiburg3_walking_halfsphere`
- `rgbd_dataset_freiburg3_walking_static`
- `rgbd_dataset_freiburg3_sitting_static` — static control

## Build

```bash
cd DT-SLAM
./build.sh
```

## Baselines

1. **ORB-SLAM2** (pure geometry) — shows degradation on dynamic scenes
2. **DynaSLAM** (Mask R-CNN + geometry) — SOTA 2018 reference

## Evaluation

See `results/` for ATE/RPE comparisons between DT-SLAM and baselines on TUM dynamic sequences.
