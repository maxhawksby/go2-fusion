---
title: Senses
parent: oak
---

# Senses

Welcome to Senses: everything Oak uses to perceive the world, from the laser scanner that feels the shape of the room to the camera and the computer vision that recognizes what is in it.

Part of: [Oak](../../README.md)

## What this part of Oak does

A robot can only act on what it can sense. Oak has three kinds of input, all delivered by the [Go2 driver](go2-driver.md) over the robot's Wi-Fi link:

- **Shape** - a lidar (laser scanner) under the chin measures distances all around. Good for walls and furniture, blind to what things *are*.
- **Sight** - a color camera in the head. Good for recognizing things (a chair, a person), poor at telling how far away they are.
- **Body sense** - joint angles, an IMU (a motion sensor, like the one that rotates your phone screen) and odometry (the robot's own estimate of how far it has walked). These tell Oak where its legs are and roughly where it has gone.

Programs on the laptop then turn this raw data into something useful: a flat "slice" of the lidar for mapping, a tree of positions that ties the sensors to the body and the room, and object detection that labels what the camera sees.

## Pages, in reading order

1. [Lidar](lidar.md) - laser points become a 3D point cloud (`/point_cloud2`), a flat ring of distances (`/scan`) for maps, and an optional saved 3D map.
2. [Camera](camera.md) - the front camera's pictures (`/camera/image_raw`) and its lens calibration.
3. [TF and robot model](tf.md) - the robot model and the tree of frames (`map`, `odom`, `base_link`, camera, legs) that says where everything is; also explains odometry drift.
4. [Object detection](object-detection.md) - computer vision: YOLOv8 finds objects in the camera picture and remembers roughly where they are; the older MobileNet `coco_detector` is to be retired. Max's current focus.

Read Lidar and TF first if you are heading for mapping; read Camera and Object detection first if you are heading for computer vision.

## A quick picture of the data flow

```
Go2 robot --Wi-Fi--> Go2 driver
   lidar  -> /point_cloud2 -> pointcloud_to_laserscan -> /scan -> SLAM, Nav2
                           -> lidar_processor -> 3D map (.ply)
   camera -> /camera/image_raw -> object_detection -> /detected_objects_dict
   joints, odometry -> /joint_states, /odom -> TF tree (map, odom, base_link, ...)
```

A topic (the names starting with `/`) is a named channel: one program publishes messages on it and any number of others listen.

## Three words you will meet everywhere

- **Point cloud** - thousands of dots in 3D, one per laser bounce: the room as connect-the-dots with the lines left out.
- **Frame** - a point of view with its own "forward, left, up", such as the room (`map`) or the robot's body (`base_link`). "2 m ahead" only means something once you say of which frame.
- **Odometry drift** - the robot estimates its own movement by counting steps, and the small errors add up, like walking with your eyes shut. SLAM corrects it.

## How Senses connects to the other hubs

- [Maps and navigation](maps-nav.md) is the biggest user: [SLAM](slam.md) draws the map from `/scan` and TF; [Nav2](nav2.md) and the [Auto mapper](auto-mapper.md) avoid obstacles using `/scan`.
- [Brains](brains.md): [Object navigation](object-navigation.md) reads the object memory to walk to "chair_1"; the [LLM agent](llm-agent.md) is meant to use it too, but is not wired to it yet.
- [Driver](driver.md): every sensor topic comes from the [Go2 driver](go2-driver.md); if the robot link drops, every sense goes quiet.
- [Driving](driving.md): you can watch the camera and the lidar in [RViz](rviz-panels.md) while you drive.

## Where things stand

- Lidar, camera and YOLO detection all passed the 2026-10-11 lab test (lidar about 6.8 Hz, YOLO about 4.5 pictures a second on CPU), per the [test log](test-log.md).
- Luis retuned the lidar scan in October so Nav2 stops seeing the floor as a wall.
- Max's branch `max/feat/yolo-map-memory` moves the object memory onto the room map; its pull request was closed without merging on 2026-10-26.
- Adib is next looking at odometry reliability and turn accuracy.

<!-- sources: AGENTS.md (packages, sensor topics), src/go2_llm_agent/README.md (known issues), src/object_navigation/README.md, docs/FUSION.md, docs/TEST_LOG.md (2026-10-11), launch/robot.launch.py, go2_robot_sdk presentation/go2_driver_node.py (topics), the four Senses program pages, origin/feat/yolo-map-memory, contract team updates week of 2026-10-18 -->
