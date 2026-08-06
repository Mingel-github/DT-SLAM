# DT-SLAM realistic indoor world candidate audit

Date: 2026-08-06

## Scope

This audit selects a more realistic simulation world. It does not change the
DT-SLAM detector, Tracking, loop closing, or Mapping algorithms. The existing
`loop_corridor.world` is retained as a repetitive-appearance / false-loop stress
scene.

The ORB counts below are a lightweight OpenCV preview using 1000 requested
features, scale factor 1.2, and 8 pyramid levels. They are not measurements from
the DT-SLAM ORB extractor and are used only to compare candidate scene views.

## Candidate A: TurtleBot3 House

Source:

`/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world`

Observations:

- Compatible with the current ROS 2 Humble and Gazebo 11 installation.
- Contains multiple rooms and some furniture.
- Several views remain dominated by repeated brick or wood textures.
- The official default spawn is unsuitable for the larger custom RGB-D robot:
  the camera starts very close to a wall and mailbox.
- A center view produced 327 preview ORB points across 9 of 12 image grid cells.
- Two near-wall views produced only 106 and 79 preview ORB points.

Decision: retain as a low-risk fallback, but do not use as the primary realistic
dynamic scene.

## Candidate B: AWS RoboMaker Small House

Source repository:

`https://github.com/aws-robotics/aws-robomaker-small-house-world`

Pinned ROS 2 commit:

`ff9631ca6d1db9c1ba656498151464b5ab74aafe`

Local immutable upstream checkout:

`/data/dynaslam/sim_assets/aws-robomaker-small-house-world`

Observations:

- Contains 68 model directories and multiple furnished rooms.
- Provides tables, chairs, beds, cabinets, carpets, balls, portraits, windows,
  and distinct room layouts.
- Three representative upstream route views produced 658, 1000, and 487
  preview ORB points. A dining-area view reached the configured 1000 point cap.
- Preview features are supplied by distinct objects and portraits instead of a
  single repeated brick texture across the full route.
- Smooth floors still provide few features, which is realistic for an
  ORB-based camera system.
- Some portraits locally dominate a view. This remains a limitation, but the
  portrait appearances differ between rooms and are not a periodic wall motif.
- Gazebo maintained a measured real-time factor of approximately 1.00 with the
  custom RGB-D camera and lidar active.

## Dynamic-space feasibility

The upstream static world remains unchanged. A DT-SLAM candidate copy adds only
one person actor:

`simulation/realistic_world_candidates/worlds/aws_small_house_dynamic_candidate.world`

The joint preview used:

- person route: west open area, `x=[-8,-3]`, `y=-3`;
- moving box route: east open area, approximately `x=[4,6]`, `y=-3`;
- box speed: 0.5 m/s;
- robot observation positions at the entrances to both open areas.

Gazebo pose checks confirmed the person remained on the west route and the box
completed a forward/reverse traversal and returned to its start. The dynamic
routes do not intersect each other and leave a connecting route for manual robot
driving.

## Selection

Freeze AWS RoboMaker Small House as the basis of the new realistic dynamic
simulation world.

Only the following project-specific additions are authorized:

1. the current RGB-D robot and sensor interfaces;
2. one person actor;
3. one uniformly moving box;
4. simulator ground-truth recording;
5. minimal route adjustment if a verified collision occurs.

Do not add artificial fiducials, repeated landmarks, or additional decorative
clutter. Final data collection will be manually driven by the user and will
record raw sensor and ground-truth topics before offline DT-SLAM evaluation.
