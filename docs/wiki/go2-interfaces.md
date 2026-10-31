---
title: Go2 interfaces
parent: driver
code: [src/go2_interfaces/**]
verified: 8a9a7118a6d4
---

# Go2 interfaces

`go2_interfaces` is Oak's dictionary of custom message types: it defines the shapes of data that the driver and our other programs agree to send each other when no standard ROS message fits.

Part of: [Driver](driver.md)

## How it works

In ROS 2, every topic carries one message **type**, a fixed list of named fields, like a form with set boxes.
Most of our topics use standard types (for example `geometry_msgs/Twist` for drive commands or `sensor_msgs/PointCloud2` for lidar).
Robot-specific data needs its own forms. Each one is a short `.msg` text file in `msg/`, and the build turns it into Python and C++ code.

This package holds 31 message types. Most came from Unitree's own definitions (they were once a separate package, `unitree_go`, now merged in). The ones our code actually uses today:

| Type | Topic | What it carries | Used by |
| --- | --- | --- | --- |
| `WebRtcReq` | `/webrtc_req` | A robot API command: `api_id` (which command), `topic` (which robot service), `parameter` (JSON text), `id`, `priority` | Sent by [posture](posture.md)'s sit controller, the [LLM agent](llm-agent.md) and the [voice](voice.md) TTS node; read by the [Go2 driver](go2-driver.md) |
| `Go2State` | `/go2_states` | The dog's status: mode, gait, body height, position, velocity, foot forces and more | Published by the driver |
| `IMU` | `/imu` | Orientation (quaternion and roll-pitch-yaw), gyroscope, accelerometer, temperature | Published by the driver |
| `VoxelMapCompressed` | `/utlidar/voxel_map_compressed` | Raw compressed lidar data | Published only when the driver's `publish_raw_voxel` is on |
| `LowState` | `lowstate` | Low-level motor and battery state | Subscribed in CycloneDDS mode (handler is empty for now) |

Note that `/imu` uses this custom `IMU` type, not the standard `sensor_msgs/Imu`, so tools expecting the standard type will not read it directly.

**Example:** make Oak wave hello from a container shell (only with the robot connected and someone watching):

```shell
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

`1016` is "Hello"; other numbers such as 1004 (Stand Up) and 1009 (Sit) are listed in the driver's `domain/constants/robot_commands.py`.

## Connects to

- [Go2 driver](go2-driver.md): publishes `Go2State` and `IMU`, reads `WebRtcReq`.
- [Robot link](robot-link.md): turns each `WebRtcReq` into a message on the WebRTC data channel.
- [Posture](posture.md), [LLM agent](llm-agent.md), [Voice](voice.md): send `WebRtcReq`.

## How it breaks

- **Sit message format:** Riley's agent sent `parameter: ""` for sit while our code sends `"{}"`. The fusion standardized the agent's sit on `"{}"`; its other sport commands still send `""`.
- **Bad JSON in `parameter`:** the driver logs `Invalid JSON in WebRTC request` and sends nothing.
- **Changing a `.msg` file** changes generated code. Rebuild this package and every package that uses it.

## Who's on it and what's next

TODO(owner): no one owns this package; it rarely changes.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_interfaces

This is a CMake package. After editing a `.msg` file, rebuild in a running container (see [Docker](docker.md)):

```shell
docker compose exec unitree_ros bash -ic 'cbs go2_interfaces go2_robot_sdk'
docker compose exec unitree_ros bash -ic 'ros2 interface show go2_interfaces/msg/WebRtcReq'
```

<!-- sources: src/go2_interfaces/CMakeLists.txt (message list, unitree_go merge); src/go2_interfaces/msg/WebRtcReq.msg, Go2State.msg, IMU.msg; imports in src/go2_robot_sdk (go2_driver_node.py, ros2_publisher.py), src/front_leg_controller/sit_controller.py, src/go2_llm_agent/ros2_motion.py, src/speech_processor/tts_node.py; go2_robot_sdk/domain/constants/robot_commands.py (1004, 1009, 1016); application/services/robot_control_service.py (Invalid JSON); docs/WINDOWS_SETUP.md (webrtc_req example); docs/FUSION.md row 11 -->
