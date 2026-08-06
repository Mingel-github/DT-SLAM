#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zhu/ros2_ws/install/setup.bash
set -u

output_root="${DT_SLAM_RECORD_ROOT:-/data/dynaslam/datasets}"
run_name="${1:-aws_small_house_person_box_$(date +%Y%m%d_%H%M%S)}"
duration_seconds="${DT_SLAM_RECORD_DURATION:-0}"
run_dir="${output_root}/${run_name}"
bag_dir="${run_dir}/rosbag2"

if [[ -e "$run_dir" ]]; then
  echo "Refusing to overwrite existing run: $run_dir" >&2
  exit 2
fi

mkdir -p "$run_dir"

topics=(
  /clock
  /camera/image_raw
  /camera/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /odom
  /box_odom
  /cmd_vel
  /box_cmd_vel
  /tf
  /tf_static
  /simulation/model_states
)

{
  echo "dataset_name=$run_name"
  echo "created_at=$(date --iso-8601=seconds)"
  echo "world=/home/zhu/dynaslam_ws/simulation/realistic_world_candidates/worlds/aws_small_house_dynamic_candidate.world"
  echo "aws_commit=ff9631ca6d1db9c1ba656498151464b5ab74aafe"
  echo "box_route=world_x_[4.0,6.0]_speed_0.5_mps"
  echo "person_route=world_x_[-8.0,-3.0]_world_y_-3.0"
  echo "duration_seconds=$duration_seconds"
  printf 'topic=%s\n' "${topics[@]}"
} > "$run_dir/manifest.txt"

echo "Recording to: $bag_dir"
echo "Stop with Ctrl+C after manual driving is complete."

record_command=(ros2 bag record --storage sqlite3 --output "$bag_dir" "${topics[@]}")
if [[ "$duration_seconds" == "0" ]]; then
  "${record_command[@]}"
else
  set +e
  timeout --foreground --signal=INT --kill-after=30 \
    "${duration_seconds}" "${record_command[@]}"
  record_status=$?
  set -e
  if [[ $record_status -ne 0 && $record_status -ne 124 ]]; then
    exit "$record_status"
  fi
fi

ros2 bag info "$bag_dir" > "$run_dir/bag_info.txt"
echo "Finished: $run_dir"
