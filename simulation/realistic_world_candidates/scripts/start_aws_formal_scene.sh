#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zhu/ros2_ws/install/setup.bash
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/dt_slam_aws_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

exec ros2 launch \
  /home/zhu/dynaslam_ws/simulation/realistic_world_candidates/launch/aws_small_house_formal.launch.py
