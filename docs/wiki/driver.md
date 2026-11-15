---
title: Driver
parent: oak
---

# Driver

Welcome to the Driver hub: the driver is the one program that actually talks to the real robot, turning Oak into something every other ROS program can read from and command.

Part of: [Oak](../../README.md)

## What this part of Oak does

Oak is a Unitree Go2 robot dog. Out of the box it speaks Unitree's own protocol, not ROS.
ROS 2 (the Robot Operating System) is the shared language our programs use: programs called **nodes** pass **messages** on named channels called **topics**.
The driver is the translator in the middle.

It does two jobs:

1. **Robot to ROS (senses).** It receives the dog's data and publishes it as ROS topics: odometry (where Oak thinks it is, `/odom`), the IMU (tilt and acceleration, `/imu`), joint angles (`/joint_states`), the lidar point cloud (`/point_cloud2`), the front camera (`/camera/image_raw`) and the robot's own status (`/go2_states`).
2. **ROS to robot (commands).** It listens for drive commands on `/cmd_vel_out` (the winner chosen by [twist_mux](twist-mux.md)) and for robot API commands such as sit, stand or wave on `/webrtc_req`, and sends them to the dog.

Everything else in the repo, from SLAM to the AI agent, sits on top of these topics. If the driver is not connected, nothing else has data.

## Pages, in reading order

1. [Go2 driver](go2-driver.md) - the `go2_robot_sdk` package: the driver node, the launch files that start the whole stack, and the configs. Start here.
2. [Robot link](robot-link.md) - how the driver physically reaches the dog: WebRTC over Wi-Fi (what the lab uses) or CycloneDDS over an Ethernet cable, plus the encryption key newer firmware needs.
3. [Go2 interfaces](go2-interfaces.md) - the custom ROS message types, like `WebRtcReq` and `Go2State`, that the driver and other packages share.

## Three facts every newcomer should know

- **One connection at a time.** The dog accepts a single WebRTC client. If the Unitree phone app or a teammate's laptop is connected, yours will fail or drop. Coordinate before you connect.
- **The driver reconnects by itself.** If Wi-Fi blips or the data goes silent for 5 seconds, the driver retries, waiting 5 seconds, then 10, up to 60.
- **You can work without the robot.** Most code can be tested in Docker by publishing or echoing topics. Save robot time for the final check.

## Is the driver working? A 1-minute check

With the stack running (see the [Lab checklist](lab-checklist.md)), open a container shell with `docker compose exec unitree_ros bash` and try:

```shell
ros2 node list                 # /go2_driver_node should be in the list
ros2 topic hz /odom            # odometry: about 13 Hz when connected
ros2 topic hz /point_cloud2    # lidar: about 3-7 Hz
```

In the logs, `Robot 0 validated and ready` means the connection succeeded.
A node in the list is not proof on its own: if `ros2 topic hz` prints nothing, no data is arriving.
Rates measured in the lab: IMU about 19 Hz, odometry about 13 Hz, camera about 10 Hz, laser scan about 5 Hz, lidar cloud 3 Hz (Windows test) to 6.8 Hz (2026-10-11).

## How the Driver connects to the rest of Oak

- **[Driving](driving.md):** every drive source ends up at twist_mux, whose output `/cmd_vel_out` the driver forwards to the dog.
- **[Senses](senses.md):** the [lidar](lidar.md), [camera](camera.md) and [TF](tf.md) pages all start from the topics the driver publishes. The driver also broadcasts the `odom` to `base_link` transform (where the body is in the world).
- **[Maps & Nav](maps-nav.md):** SLAM and Nav2 use the driver's `/scan` (made from `/point_cloud2`) and `/odom`.
- **[Brains](brains.md):** the agent, [posture](posture.md) and [voice](voice.md) send robot commands through `/webrtc_req`.
- **[Lab setup](lab-setup.md):** how to start the driver in Docker and connect to Oak in the lab.

## Who's on it

- Luis ported the driver's auto-reconnect, drop detection and the "explicit stop" fix from the team fork (2026-10-09 to 10-11).
- Adib's next task is driver-adjacent: why position data (odometry) sometimes stops, and why a requested turn differs from the measured one.
- TODO(owner): an overall owner for the driver.
