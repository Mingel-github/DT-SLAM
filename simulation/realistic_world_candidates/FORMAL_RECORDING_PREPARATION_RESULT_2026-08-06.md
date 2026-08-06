# AWS Small House formal recording preparation result

Date: 2026-08-06

## Outcome

AWS RoboMaker Small House is ready for a manually driven formal recording.
This preparation changes only the simulation and recording infrastructure. It
does not run RTAB-Map and does not change DT-SLAM, YOLO, SIn-style geometry,
Tracking, loop closing, or Mapping algorithms.

## Frozen scene

- Upstream AWS asset commit:
  `ff9631ca6d1db9c1ba656498151464b5ab74aafe`
- Robot start: `(x=2.75, y=-1.84, yaw=pi)`
- Person actor nominal route: `x=[-8,-3], y=-3`
- Box route: `x=[4,6], y=-3`
- Box speed: `0.5 m/s`, fixed heading, forward/reverse traversal
- Robot motion: manual `/cmd_vel` control only

The person and box operate in separate west/east areas. The central route is
left available for manual driving.

## Recorded raw data

The formal recorder stores:

- RGB and depth images;
- RGB and depth camera calibration;
- simulator clock;
- robot odometry and TF;
- box odometry and command;
- robot manual command;
- `/simulation/model_states`, containing exact Gazebo poses for `geo_bot`,
  `dynamic_box`, and `actor_patrol`.

Point-cloud topics and the old RTAB-Map filter outputs are deliberately omitted.
They are redundant for DT-SLAM RGB-D evaluation and would unnecessarily enlarge
the bag.

## Smoke recording

Valid smoke dataset:

`/data/dynaslam/datasets/aws_small_house_smoke2_20260806`

The first automatic smoke attempt exposed a shell signal-forwarding problem and
created no bag. The recorder was corrected to use foreground signal forwarding;
the second attempt stopped cleanly and wrote complete metadata.

Measured second-run contents:

| Stream | Messages in 9.94 s |
| --- | ---: |
| RGB image | 199 |
| Depth image | 199 |
| RGB camera info | 199 |
| Depth camera info | 199 |
| Robot odometry | 292 |
| Box odometry | 994 |
| Gazebo model states | 50 |
| TF | 293 |
| TF static | 1 |

Validation result:

- 199 exact RGB/depth timestamp pairs;
- zero RGB-only or depth-only timestamps;
- RGB: `640x480 rgb8`;
- depth: `640x480 32FC1`;
- shared RGB/depth intrinsics:
  `fx=fy=554.3827`, `cx=320.5`, `cy=240.5`;
- person sampled world-X range: approximately `2.76 m`;
- box sampled world-X range: approximately `1.95 m`;
- required simulator entities all present;
- validator result: `passed=true`.

The 10-second raw bag is approximately 411 MiB. The `/data` partition had about
342 GiB free during this audit. Recording remains uncompressed to avoid adding
CPU load to Gazebo; archival compression can be performed after conversion and
validation.

## Formal operating order

1. Start the formal AWS scene:

   ```bash
   /home/zhu/dynaslam_ws/simulation/realistic_world_candidates/scripts/start_aws_formal_scene.sh
   ```

2. Start the recorder in a separate terminal:

   ```bash
   /home/zhu/dynaslam_ws/simulation/realistic_world_candidates/scripts/record_aws_rgbd_dataset.sh
   ```

3. After the recorder prints `Recording...`, the user starts keyboard control:

   ```bash
   cd /home/zhu/ros2_ws
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

4. Stop the recorder with `Ctrl+C` after completing the route.

5. Validate the bag before conversion or SLAM execution:

   ```bash
   source /opt/ros/humble/setup.bash
   python3 /home/zhu/dynaslam_ws/simulation/realistic_world_candidates/tools/validate_aws_rosbag.py \
     /data/dynaslam/datasets/<run_name>/rosbag2
   ```

## Next boundary

The recording infrastructure is complete. The next action requires manual user
driving: start one formal bag, notify the user only after rosbag confirms
`Recording...`, then let the user drive the selected route. No new detector or
mapping module should be added before this dataset is recorded and validated.

## Formal run 1 result

The first manually driven formal recording completed successfully:

`/data/dynaslam/datasets/aws_small_house_person_box_formal_run1_20260806`

| Item | Result |
| --- | ---: |
| Wall-time duration | 316.64 s |
| Bag size | 12.8 GiB |
| Total ROS messages | 114,902 |
| Exact RGB/depth pairs | 6,328 |
| Unpaired RGB frames | 0 |
| Unpaired depth frames | 1 boundary frame |
| Manual velocity commands | 3,000 |
| Robot sampled path length | 38.76 m |
| Robot start-to-end distance | 4.18 m |
| Person sampled path length | 180.44 m |
| Box sampled path length | 154.13 m |

The one unpaired depth image lies outside the common RGB/depth timestamp
interval. It is a recording start/stop boundary sample, not an internal dropped
RGB frame. Offline conversion must use the 6,328 exact timestamp pairs and omit
that one depth-only sample.

The robot covered approximately `15.55 m` in world X and `5.06 m` in world Y.
The trajectory is broad enough for the planned person/box dynamic evaluation,
but it did not return to the initial pose: its simulator-ground-truth
start-to-end distance is `4.18 m`. Consequently, this run should not by itself
be described as a closed-loop or loop-closure test.

Final raw-data validation: `passed=true`, with the single documented boundary
frame warning above. The next processing step is deterministic rosbag-to-TUM
conversion followed by offline four-mode DT-SLAM evaluation.
