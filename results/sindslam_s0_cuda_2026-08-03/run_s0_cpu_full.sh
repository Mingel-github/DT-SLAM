#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 <repeat> [start_index]" >&2
    exit 2
fi

repeat_id="$1"
start_index="${2:-0}"
sin_root=/data/dynaslam/SInDSLAM/ORB_SLAM2
result_root=/home/zhu/dynaslam_ws/results/sindslam_s0_cuda_2026-08-03/full
mkdir -p "$result_root"

export SIND_SLAM_DISABLE_VIEWER=1
export LD_LIBRARY_PATH=/home/zhu/dynaslam_ws/pangolin_install/lib
unset SIND_SLAM_DISABLE_DYNAMIC_DETECTION || true

names=(
    fr3_walking
    bonn_nonobstructing
    bonn_obstructing
    fr1_xyz
)
settings=(
    Examples/RGB-D/TUM3.yaml
    Examples/RGB-D/Bonn.yaml
    Examples/RGB-D/Bonn.yaml
    Examples/RGB-D/TUM1.yaml
)
datasets=(
    /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz
    /data/dynaslam/datasets/rgbd_bonn_moving_nonobstructing_box
    /data/dynaslam/datasets/rgbd_bonn_moving_obstructing_box
    /home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg1_xyz
)
associations=(
    /home/zhu/dynaslam_ws/results/sindslam_reproduction_2026-08-02/fr3_walking_author_offset_minus_0p033_associations.txt
    /home/zhu/dynaslam_ws/results/sindslam_mask_audit_2026-08-02/nonobstructing_associations_no_comments.txt
    /home/zhu/dynaslam_ws/results/sindslam_s0_cuda_2026-08-03/moving_obstructing_associations_no_comments.txt
    /home/zhu/dynaslam_ws/results/sindslam_reproduction_2026-08-02/fr1_xyz_author_offset_minus_0p033_associations.txt
)

cd "$sin_root"
for index in "${!names[@]}"; do
    if (( index < start_index )); then
        continue
    fi
    stem="${names[$index]}_cpu_on_run${repeat_id}"
    echo "[S0 runner] start $stem"
    /usr/bin/time -v ./Examples/RGB-D/rgbd_tum_noros \
        Vocabulary/ORBvoc.txt \
        "${settings[$index]}" \
        "${datasets[$index]}" \
        "${associations[$index]}" \
        2>&1 | tee "$result_root/${stem}.log"
    cp CameraTrajectory.txt "$result_root/${stem}_CameraTrajectory.txt"
    cp KeyFrameTrajectory.txt "$result_root/${stem}_KeyFrameTrajectory.txt"
    echo "[S0 runner] finish $stem"
done
