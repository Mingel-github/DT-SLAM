# Realistic indoor world candidates

This directory contains only DT-SLAM-side launch and audit helpers. The original
third-party simulation assets remain unmodified.

## Fixed candidates

- TurtleBot3 House: installed by the ROS 2 Humble TurtleBot3 package.
- AWS RoboMaker Small House, ROS 2 branch, pinned commit:
  `ff9631ca6d1db9c1ba656498151464b5ab74aafe`.

The AWS asset checkout is stored outside the repository at:

`/data/dynaslam/sim_assets/aws-robomaker-small-house-world`

The existing `loop_corridor.world` remains unchanged and serves as a repetitive
appearance / false-loop stress scene.

## Scope

Candidate inspection does not run RTAB-Map and does not modify DT-SLAM. The
same custom RGB-D robot is spawned in both worlds.

## Selected formal scene

AWS RoboMaker Small House is the selected base. The formal candidate copy adds
one west-side person actor and spawns one east-side uniformly moving box. Start
the complete scene with:

```bash
/home/zhu/dynaslam_ws/simulation/realistic_world_candidates/scripts/start_aws_formal_scene.sh
```

The fixed routes are:

- person: world `x=[-8,-3]`, `y=-3`;
- box: world `x=[4,6]`, `y=-3`, speed `0.5 m/s`;
- robot: starts at `(2.75,-1.84)`, facing west, then remains under manual
  `/cmd_vel` control.

The added Gazebo state plugin publishes `/simulation/model_states` at 5 Hz so
the robot, person actor, and box share one simulator-ground-truth stream.

## Recording

After the formal scene is ready, run:

```bash
/home/zhu/dynaslam_ws/simulation/realistic_world_candidates/scripts/record_aws_rgbd_dataset.sh
```

The recorder stores raw RGB-D, both camera-info topics, robot and box odometry,
commands, TF, simulator time, and exact Gazebo model states. It deliberately
does not start RTAB-Map, DT-SLAM, semantic filtering, or record redundant point
cloud topics. DT-SLAM evaluation is performed offline from the recorded raw
data.

Validate a completed bag before conversion or SLAM evaluation:

```bash
source /opt/ros/humble/setup.bash
python3 /home/zhu/dynaslam_ws/simulation/realistic_world_candidates/tools/validate_aws_rosbag.py \
  /data/dynaslam/datasets/<run_name>/rosbag2
```

The validator checks exact RGB/depth timestamp pairing, image metadata, camera
intrinsics, required odometry streams, simulator entities, and actual person and
box displacement. It does not evaluate DT-SLAM accuracy.
