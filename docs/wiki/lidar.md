---
title: Lidar
parent: senses
code: [src/lidar_processor/**, src/lidar_processor_cpp/**, src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors/lidar_decoder.py]
---

# Lidar

The lidar is Oak's sense of shape: a laser scanner under its chin that measures the distance to walls and objects all around, which [SLAM](slam.md) needs to draw the map and [Nav2](nav2.md) needs to avoid bumping into things.

Part of: [Senses](senses.md)

## How it works

**Point clouds.** A lidar fires laser pulses and times how long each takes to bounce back, so each pulse becomes one dot in space: "something is here, 2.3 m ahead and 0.4 m up". Thousands of dots together are a **point cloud**, a 3D connect-the-dots picture of the room with the lines left out.

**From the robot to `/point_cloud2`.** The Go2 sends its lidar data in a compressed form over WebRTC. The driver's `lidar_decoder.py` unpacks it and the driver publishes it as a `PointCloud2` message on `/point_cloud2`, in the `odom` frame (see [TF and robot model](tf.md)). Measured on Oak: about 6.8 messages a second ([Test log](test-log.md)).

**From 3D to a flat scan, `/scan`.** SLAM and Nav2 work on a flat, 2D "slice". The standard ROS node `pointcloud_to_laserscan` (started by `robot.launch.py`) keeps only the points in a height band around the body and turns them into a ring of distances, like a person sweeping a flashlight around the room at waist height. In `robot.launch.py` the band is -0.1 m to +0.5 m relative to `base_link`, and anything closer than 0.1 m is ignored. Luis chose these in October: lower and the floor showed up as a wall; a 0.5 m minimum hid walls right beside the robot.

**The 3D map (`lidar_processor`).** Two optional helper nodes, in Python (`lidar_processor`) and C++ (`lidar_processor_cpp`, same topics and parameters):

- `lidar_to_pointcloud` piles up every cloud it receives into one big cloud on `/pointcloud/aggregated` (up to 1,000,000 points, oldest dropped first) and saves it every 10 s as a `.ply` 3D file (`3d_map.ply` by default). The Python version saves it in the `map` frame, so when SLAM corrects the map the file moves with it, and has a `/lidar_to_pointcloud/reset` service behind the RViz "Reset maps" button.
- `pointcloud_aggregator` cleans that cloud up (distance and height limits, keeping every Nth point) and publishes `/pointcloud/filtered` and `/pointcloud/downsampled`.

Their settings (the same in both versions):

| Node | Parameter | Default | Meaning |
| --- | --- | --- | --- |
| `lidar_to_pointcloud` | `robot_ip_lst` | - | List of robot addresses; with more than one it listens on `/robot{i}/point_cloud2` |
| `lidar_to_pointcloud` | `map_name` | `3d_map` | File name of the saved map |
| `lidar_to_pointcloud` | `map_save` | `true` | Save the map every `save_interval` |
| `lidar_to_pointcloud` | `save_interval` | `10.0` | Seconds between saves |
| `lidar_to_pointcloud` | `max_points` | `1000000` | Most points kept in memory |
| `lidar_to_pointcloud` | `voxel_size` | `0.01` | Grid size (m) for thinning the cloud when saving |
| `pointcloud_aggregator` | `max_range` | `20.0` | Farthest point kept from the robot center (m) |
| `pointcloud_aggregator` | `min_range` | `0.1` | Nearest point kept (m) |
| `pointcloud_aggregator` | `height_filter_min` / `height_filter_max` | `-2.0` / `3.0` | Height band kept (m) |
| `pointcloud_aggregator` | `downsample_rate` | `10` | Keep every Nth point |
| `pointcloud_aggregator` | `publish_rate` | `5.0` | Messages per second |

The C++ version uses PCL (the Point Cloud Library), including PCL's statistical outlier removal, and has its own launch files: `lidar_to_pointcloud.launch.py`, `pointcloud_aggregator.launch.py`, and `lidar_processing_pipeline.launch.py` for both together. It is BSD-3-Clause licensed.

`robot.launch.py` starts the Python versions; `robot_cpp.launch.py`, `mapping.launch.py` and `navigation.launch.py` start the C++ versions.

## Connects to

- [Go2 driver](go2-driver.md) - decodes the lidar and publishes `/point_cloud2`.
- [SLAM](slam.md) - builds the 2D map from `/scan`.
- [Nav2](nav2.md) - marks obstacles in its costmaps from `/scan` only (not the 3D cloud).
- [Auto mapper](auto-mapper.md) - checks `/scan` and `/point_cloud2` before moving; its "Reset maps" button clears the 3D map.
- [RViz panels](rviz-panels.md) - the 3D voxel view of the cloud.

## How it breaks

- **Odometry drift smears the cloud.** The driver's cloud piles up in `odom`, and odometry's height drifts, so the floor gets smeared from about -0.6 to +0.2 m. That is why Nav2 now ignores the cloud and uses `/scan` (comment in `nav2_params.yaml`).
- **The 3D map never forgets.** Points stay until the map is reset, so people who walked by stay in it (auto mapper README).
- **Launch files disagree.** `mapping.launch.py` and `navigation.launch.py` build `/scan` from `/pointcloud/filtered` with a 0.3 m minimum and a -1 to 3 m height band; `robot_cpp.launch.py` uses -0.2 to 2 m. Only `robot.launch.py` has Luis's tuned values, which `nav2_params.yaml` and the SLAM settings assume.
- **The C++ version lags.** `lidar_processor_cpp` has no reset service and does not save in the map frame, so "Reset maps" only clears the 3D map under `robot.launch.py`.
- **Topic name.** `lidar_to_pointcloud` listens on `/robot0/point_cloud2` in single-robot mode; launch files remap it to `/point_cloud2`. Still listed as "check" in the [Test log](test-log.md), open issue 3.
- **Slow decoder.** A faster LZ4 decoder exists on the old `fix_hz_lidar_stream` branch (claimed about 2 Hz to 62 Hz) but was not merged ([Decisions](decisions.md), follow-up 2).

## Who's on it and what's next

Luis tuned the scan (body-relative, 0.1 m minimum) and moved the 3D map to the map frame in October for the auto mapper. TODO(owner): who owns lidar processing, and whether the other launch files should get the same scan settings.

## The code

- Python nodes: https://github.com/maxhawksby/go2-fusion/tree/master/src/lidar_processor
- C++ nodes: https://github.com/maxhawksby/go2-fusion/tree/master/src/lidar_processor_cpp
- Decoder: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors
- Scan settings: `_create_pointcloud_to_laserscan_node` in https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk/launch

Check it and run the unit test inside Docker:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 topic hz /point_cloud2'
docker compose exec unitree_ros bash -ic 'ros2 topic echo /scan --once'
docker compose run --rm unitree_ros bash -c 'python3 -m pytest /ros2_ws/src/lidar_processor/test'
docker compose exec unitree_ros bash -ic 'ros2 launch lidar_processor_cpp lidar_processing_pipeline.launch.py map_name:="my_custom_map" max_range:=15.0 publish_rate:=10.0'
```

<!-- sources: src/lidar_processor/lidar_processor/lidar_to_pointcloud_node.py, frame_transform.py, pointcloud_aggregator_node.py, src/lidar_processor_cpp/README.md and src/*.cpp, src/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors/lidar_decoder.py, ros2_publisher.py (frame odom), launch/robot.launch.py, robot_cpp.launch.py, mapping.launch.py, navigation.launch.py, config/nav2_params.yaml comments, src/go2_auto_mapper/README.md, docs/TEST_LOG.md, docs/FUSION.md, git log Oct 14-19 (Luis) -->
