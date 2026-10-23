---
title: Camera
parent: senses
code: [src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors/camera_config.py]
---

# Camera

The camera is Oak's eye: a single color camera in the front of its head, whose pictures feed [object detection](object-detection.md) and let people see what the robot sees.

Part of: [Senses](senses.md)

## How it works

The Go2 sends its video to the computer over the same Wi-Fi link (WebRTC) that carries everything else. The [Go2 driver](go2-driver.md) decodes each video frame into an ordinary picture and publishes it as a ROS message on `/camera/image_raw`.

Alongside every picture, the driver publishes a `CameraInfo` message on `/camera/camera_info`. This is the camera's "calibration": numbers that describe its lens, such as how far the lens is from the sensor (focal length) and how much it bends straight lines (distortion). A program can use them to turn a pixel into a real direction. The numbers live in small YAML files, one per picture height: `front_camera_720.yaml` (1280 x 720) and `front_camera_1080.yaml`. The file `camera_config.py` finds those files, reads them, and picks the one that matches the height of the incoming video.

Both messages are stamped with the frame `front_camera`, a point defined in the robot model on the upper head, so other programs know where the picture was taken from (see [TF and robot model](tf.md)).

## Connects to

- [Go2 driver](go2-driver.md) - decodes the video and publishes the topics.
- [Object detection](object-detection.md) - both detectors read `/camera/image_raw`.
- [TF and robot model](tf.md) - the `front_camera` frame says where the camera sits on the head.
- [RViz panels](rviz-panels.md) - the default RViz setup has an Image display on `/camera/image_raw`.

Topics:

- `/camera/image_raw` (`sensor_msgs/Image`) - the pictures. Sent "best effort", which means a late frame is dropped rather than resent; that is normal for video.
- `/camera/camera_info` (`sensor_msgs/CameraInfo`) - lens calibration for each picture.

With more than one robot, each gets a prefix, for example `/robot0/camera/image_raw`.

## How it breaks

- If the calibration file for the incoming picture height is missing, there is no `CameraInfo` for it (the loader logs an error).
- In the 2026-10-11 lab session, after Ctrl+C the camera kept streaming while lidar and odometry went silent; recreating the container fixed it ([Test log](test-log.md)).
- The driver keeps video going past bad frames since Luis's October driver fixes ([Decisions](decisions.md), row 15).
- Today no detector uses the calibration; `object_detection` assumes a 90 degree view instead.

## Who's on it and what's next

Noah verified the live camera feed over WebRTC and Foxglove and added camera feed selection to the Oak Control dashboard. TODO(owner): who owns the camera code, and whether the detector should start using the calibration.

## The code

- Calibration loader: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors
- Calibration files: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/calibration

Check the feed inside the Docker container while the robot stack runs:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 topic hz /camera/image_raw'
docker compose exec unitree_ros bash -ic 'ros2 topic echo /camera/camera_info --once'
```

<!-- sources: src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors/camera_config.py, src/go2_robot_sdk/calibration/front_camera_720.yaml, src/go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py (publish_camera_data), src/go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py (topic names), src/go2_robot_sdk/urdf/go2.urdf (front_camera_joint), src/go2_robot_sdk/config/single_robot_conf.rviz, docs/TEST_LOG.md, docs/FUSION.md row 15, contract team updates (Noah) -->
