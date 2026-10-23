---
title: TF and robot model
parent: senses
code: [src/go2_robot_sdk/urdf/**, src/go2_robot_sdk/meshes/**, src/go2_robot_sdk/dae/**, src/go2_robot_sdk/calibration/**]
---

# TF and robot model

TF is how Oak keeps track of where everything is relative to everything else (the room, its body, each leg, the camera, the lidar), so a dot the lidar sees can be placed on the map and a box the camera sees can be turned into a spot in the room.

Part of: [Senses](senses.md)

## How it works

**Frames.** A frame is a point of view with its own "forward, left, up". Saying "the chair is 2 m ahead" only makes sense if you say ahead of *what*. ROS uses these frames (x is forward, y is left, z is up):

- `map` - the room, as drawn by [SLAM](slam.md). Fixed: a wall's position on `map` should never change.
- `odom` - where Oak *thinks* it started, based on counting its own steps.
- `base_link` - the center of Oak's body. Moves with the robot.
- `front_camera`, `radar` (the lidar), `imu`, `Head_upper`, and one frame per leg part (`FL_hip`, `FL_thigh`, `FL_calf`, `FL_foot`, and the same for FR, RL, RR).

**The TF tree.** The frames hang from each other like a family tree: `map` -> `odom` -> `base_link` -> head -> `front_camera`, and so on. Each link in the tree is a transform, "how to get from the parent to the child". Any program can ask the TF library "where is `front_camera` in `map`, right now?" and it chains the links together. Three different programs keep the tree up to date:

- `robot_state_publisher` reads the robot model (URDF, below) plus `/joint_states` from the driver and publishes the body, legs, camera and lidar.
- The [Go2 driver](go2-driver.md) publishes `odom` -> `base_link` from the robot's own position estimate, also sent on `/odom`.
- [SLAM](slam.md) publishes `map` -> `odom`, the correction that keeps the robot lined up with the map.

**Odometry drift.** Odometry is the robot's step counting: "I walked about 1 m forward and turned about 90 degrees". Each guess is a little off, and the errors add up, like walking through your house with your eyes shut: after a few rooms you think you are in the kitchen doorway but you are a step to the left. That growing error is drift. SLAM fixes it by comparing what the lidar sees with the map and nudging `map` -> `odom`, so `odom` can drift while `map` stays true.

**The robot model (URDF).** `go2.urdf` is a text file that lists every body part (a "link") and every joint, with sizes, positions and the 3D shapes in `dae/` that RViz draws. For example the camera is placed 4.5 cm ahead of and 3 cm above the upper head. Variants: `multi_go2.urdf` (several robots, frames prefixed `robot0/`), `go2_with_realsense.urdf` and `go2_on_steroids.urdf`. The `calibration/` files describe the camera lens (see [Camera](camera.md)).

## Connects to

- [Go2 driver](go2-driver.md) - publishes `/joint_states`, `/odom`, `/imu` and `odom` -> `base_link`.
- [SLAM](slam.md) - publishes `map` -> `odom`.
- [Lidar](lidar.md) and [Camera](camera.md) - their messages are stamped with `odom` and `front_camera`.
- [Nav2](nav2.md), [Auto mapper](auto-mapper.md), [Map viewers](map-viewers.md) - all look up where `base_link` is on `map`.
- [Object detection](object-detection.md) - the in-progress map memory uses TF to put objects on the map.

## How it breaks

- **Drift in height** smears the 3D lidar cloud, so the floor looks like obstacles; Nav2 switched to the 2D scan because of it (`nav2_params.yaml` comments).
- **Stale `map` -> `odom`.** `/scan` arrives about 4 times a second; with a short time-out Nav2 logged "Transform data too old" and stopped. SLAM's `transform_timeout` is now 0.5 s (`mapper_params_online_async.yaml`).
- **Position data stops.** After Ctrl+C in a lab session, `/odom` went silent while the camera kept going ([Test log](test-log.md)).
- **Odd model entries.** `go2.urdf` also declares fixed `map` -> `odom` and `odom` -> `base_link` joints, which the driver and SLAM publish too. TODO(owner): confirm whether these clash with SLAM's live `map` -> `odom` and should be removed.

## Who's on it and what's next

Adib's next focus is odometry reliability and turn accuracy: why position data sometimes stops, and how far Oak really turns compared with what was asked. TODO(owner): owner of the URDF and calibration files.

## The code

- Robot models: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/urdf
- 3D shapes: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/dae
- Camera calibration: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/calibration

Look at the live tree inside Docker while the robot stack runs:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run tf2_ros tf2_echo map base_link'
docker compose exec unitree_ros bash -ic 'ros2 run tf2_tools view_frames'   # writes frames.pdf
```

<!-- sources: src/go2_robot_sdk/urdf/go2.urdf (links, joints, front_camera_joint, radar_joint, map_joint, odom_joint), launch/robot.launch.py (robot_state_publisher), go2_robot_sdk/infrastructure/ros2/ros2_publisher.py (odom transform), config/mapper_params_online_async.yaml (transform_timeout), config/nav2_params.yaml (voxel layer comments), go2_map_viewer_julia/README.md, docs/TEST_LOG.md, contract team updates (Adib) -->
