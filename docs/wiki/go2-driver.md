---
title: Go2 driver
parent: driver
code: [src/go2_robot_sdk/**]
---

# Go2 driver

`go2_robot_sdk` is the package that makes Oak a ROS 2 robot: it holds the driver node that talks to the dog, plus the launch files and settings that start the whole robot stack.

Part of: [Driver](driver.md)

## How it works

**The driver node** (`go2_driver_node`) connects to the dog, then runs two loops side by side: one handles ROS messages, the other handles the robot connection.
When data arrives from the dog it decodes it and publishes `/odom`, `/imu`, `/joint_states`, `/point_cloud2`, `/camera/image_raw`, `/camera/camera_info` and `/go2_states`.
When a command arrives on `/cmd_vel_out` or `/webrtc_req`, it sends it to the dog.
A zero velocity is forwarded too, as an explicit stop (an older version dropped zeros, so "stop" never arrived).

**Stale-sensor stop.** Nav2 and the auto mapper steer by the lidar and odometry. If those stop arriving, they keep driving on an old picture of the room. So the driver only forwards a move while its sensors are fresh: odometry, the lidar, and (with one robot) the `/scan` that Nav2's costmap reads. If any of them has sent nothing for `sensor_timeout_sec` (default 1 second), the driver sends the dog a stop instead of the move, and stops it at once if it was already moving. When fresh data comes back, the next move goes through again. Before the first data arrives after start, Oak also will not move. Teleop and the gamepad are held too; sit and stand requests (`/webrtc_req`) are not. Only the WebRTC (Wi-Fi) link is watched; CycloneDDS is not.

Each stop and each resume is reported on `/safety/events` (a `diagnostic_msgs/DiagnosticStatus`, for the scoreboard). `sensor_stale` (level WARN) says which sensors went quiet and how old their data is (`sensor`, `age_s`); `sensor_fresh` (level OK) says they are back, plus how long Oak was held (`stopped_s`). The robot number is in `hardware_id`. This topic is temporary: once Noah's scoreboard event stream (`/auto_mapper/events`, PR #47) is on master, the driver will report these there instead.

Parameters (change live with `ros2 param set /go2_driver_node <name> <value>`):

- `sensor_timeout_sec` (1.0) - how old sensor data may get before Oak is held still. 0 turns the stop off.
- `sensor_watch_scan` (true) - also require `/scan`. Turn it off only when running the driver without the laser-scan converter, or Oak will never move.

The code is split into layers: `domain/` (plain data and constants, like the robot command numbers), `application/` (what to do with data and commands), `infrastructure/` (the WebRTC link, decoders, ROS publishing) and `presentation/` (the ROS node itself).

**Settings** come from environment variables you pass to Docker:

- `ROBOT_IP` - the dog's address (ask a teammate). Several addresses separated by commas means several robots, each under its own `robot0/`, `robot1/` prefix.
- `CONN_TYPE` - `webrtc` (Wi-Fi, the lab default) or `cyclonedds` (Ethernet). See [Robot link](robot-link.md).
- `ROBOT_AES_KEY` - only for firmware 1.1.15 and newer. Never commit it.

**The default launch file** `robot.launch.py` is what Docker runs on start. Besides the driver it starts the robot model publisher, a lidar-to-laser-scan converter (`/scan`), the lidar processor, a text-to-speech node, the gamepad nodes, [twist_mux](twist-mux.md), RViz, Foxglove Bridge (a data viewer you open in a browser or app), SLAM and Nav2.
Turn parts off with arguments such as `rviz2:=false`, `nav2:=false`, `slam:=false`, `foxglove:=false`, `joystick:=false`, `teleop:=false`. `driver:=false` leaves out the driver itself; the [Simulator](simulator.md) uses it to run the same stack with its own stand-in driver.
Other launch files (`robot_cpp`, `navigation`, `mapping`, `webrtc_web`) are variants; note `webrtc_web.launch.py` skips twist_mux.

**Configs** live in `config/`: `twist_mux.yaml`, `joystick.yaml`, `nav2_params.yaml`, `mapper_params_online_async.yaml` (SLAM) and the RViz layouts.

## Connects to

- [Robot link](robot-link.md): the WebRTC connection inside `infrastructure/webrtc/`.
- [Go2 interfaces](go2-interfaces.md): message types such as `WebRtcReq` and `Go2State`.
- [twist_mux](twist-mux.md), [Gamepad](gamepad.md): drive commands in.
- [Lidar](lidar.md), [Camera](camera.md), [TF](tf.md), [SLAM](slam.md), [Nav2](nav2.md): consume what the driver publishes.

## How it breaks

- **Oak walked into a wall** (2026-10-11, old code): the driver dropped every zero velocity, so stop never reached the dog. Fixed; on new code the stop arrived.
- **Data stalls:** the camera kept streaming but lidar and odometry went silent after a Ctrl+C. The driver now treats 5 seconds without data as a lost connection and reconnects. A forced restart also recovered it then: `docker compose up -d --force-recreate`.
- **Oak will not move and the log says `Not moving robot 0: no fresh ...`.** The stale-sensor stop is holding it. The message names the quiet sensor. If it is `scan`, check the laser-scan converter is running (`ros2 topic hz /scan`). If it is `lidar` or `odom`, the robot link stalled; the reconnect above should bring it back, and Oak moves again on its own.
- **Short stops while driving** mean the data arrived more than 1 second late. On a weak Wi-Fi link that may happen often. TODO(max): measure how often on Oak before tuning `sensor_timeout_sec`.
- **One bad video frame used to end the camera feed for good.** Now it gives up only after 10 errors in a row.
- **`ElevenLabs API key not provided`** in the logs is the text-to-speech node, not the driver. Ignore it.
- **H264 decode warnings** are occasional dropped video packets. Harmless.

## Who's on it and what's next

Luis ported reconnect, drop detection and the stop fix. Adib is next looking at odometry reliability and turn accuracy. Max added the stale-sensor stop (issue #29); it still needs a test on Oak. TODO(owner): an owner for this package.

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_robot_sdk

Start the full stack in the lab (from `docker/`; see the [Lab checklist](lab-checklist.md)):

```shell
ROBOT_IP=<ask a teammate> CONN_TYPE=webrtc docker compose up
```

Success: `Robot 0 validated and ready` in the logs. Run the package's tests in a container shell:

```shell
cd /ros2_ws && colcon test --packages-select go2_robot_sdk && colcon test-result --verbose
```

<!-- sources: src/go2_robot_sdk/go2_robot_sdk/main.py; presentation/go2_driver_node.py (env vars, params, topics, video error limit); application/services/robot_control_service.py (explicit stop); infrastructure/webrtc/connection_health.py; launch/robot.launch.py (nodes, launch args); launch/webrtc_web.launch.py; docker/Dockerfile CMD; docs/TEST_LOG.md 2026-10-11; docs/WINDOWS_SETUP.md troubleshooting; docs/FUSION.md row 15 and follow-up 6; git log origin/fix/nav-obstacle-safety (6b42eac); test/test_connection_health.py; domain/sensor_watchdog.py and test/test_sensor_watchdog.py (stale-sensor stop, /safety/events); colcon test command is standard colcon usage, TODO(owner) confirm it passes in Docker -->
