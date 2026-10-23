---
title: WSL and Windows
parent: lab-setup
---

# WSL and Windows

Most of the team uses Windows, and this page shows the two ways to run Oak's container there, so you can pick the simple one or the one that shows RViz windows.

Part of: [Lab setup](lab-setup.md)

## Pick your path

**WSL2** is Linux running inside Windows. **WSLg** is the part of WSL2 that shows Linux app windows on your Windows desktop; it ships with Windows 11 and recent Windows 10, so there is nothing extra to install.

| | Path A: Windows PowerShell | Path B: inside WSL2 |
| --- | --- | --- |
| Compose file | `docker-compose.windows.yml` (add `-f docker-compose.windows.yml` to every command) | the default `docker-compose.yml` |
| RViz and map viewer windows | No (unless you add the extra display flags on the [Command cheat sheet](commands.md)) | Yes, through WSLg |
| Gamepad | No | Yes |
| See the data | Foxglove at `ws://localhost:8765` | RViz, or Foxglove |
| Best for | Keyboard driving, auto mapping, Foxglove | Anything with a window |

Both paths need Docker Desktop running. They use different compose files and are otherwise independent, but only one container at a time can hold the robot's connection, so stop one before starting the other.

## Path A: Windows PowerShell

Tested with Docker Desktop (Linux containers) on Windows 11, Oak on Wi-Fi (WebRTC), ROS 2 Humble.

1. From your checkout: `git submodule update --init --recursive` (pulls `src/go2_robot_sdk/external_lib/aioice`).
2. Build once (slow the first time only: ROS packages, torch, open3d):
   ```powershell
   cd docker
   docker compose -f docker-compose.windows.yml build
   ```
3. In the lab, set the robot address (ask a teammate) and start:
   ```powershell
   $env:ROBOT_IP="<ask a teammate>"; $env:CONN_TYPE="webrtc"
   docker compose -f docker-compose.windows.yml up -d --build
   docker compose -f docker-compose.windows.yml logs -f unitree_ros
   ```
   `CONN_TYPE` already defaults to `webrtc`. Success: `Robot 0 validated and ready` in the logs.
4. Stop it: `docker compose -f docker-compose.windows.yml down`.

The image holds only the dependencies; your checkout is mounted into the container and built when it starts, so editing code never needs an image rebuild (see [Docker](docker.md)).

### Why a separate compose file

`docker-compose.yml` targets Linux hosts. On Windows:

- `devices: /dev/input` (the gamepad) does not exist;
- the X11 socket and `~/.Xauthority` mounts do not apply, so RViz cannot display;
- `network_mode: host` attaches to Docker's Linux VM, not your Windows network adapter, so published ports such as Foxglove's 8765 are unreachable.

`docker-compose.windows.yml` drops those and publishes port 8765 instead.

### See the data in Foxglove

Open Foxglove (desktop app recommended, or app.foxglove.dev in Chrome or Edge), choose Open connection, Foxglove WebSocket, and enter `ws://localhost:8765`.

Topics seen publishing in testing:

| Data | Topic | Rate |
| --- | --- | --- |
| IMU | `/imu` | about 19 Hz |
| Joint states | `/joint_states` | about 1 Hz (expected on firmware 1.1.7+) |
| Odometry | `/odom` | about 13 Hz |
| Camera | `/camera/image_raw` | about 10 Hz |
| Lidar point cloud | `/point_cloud2` | about 3 Hz |
| Laser scan | `/scan` | about 5 Hz |

The upstream README's names `/go2_camera/color/image` and `/pointcloud/aggregated` do not match this build. `/pointcloud/aggregated`, `/pointcloud/filtered` and `/pointcloud/downsampled` published nothing in testing.
Foxglove cannot click-to-navigate like the C++ map viewer, but it shows the same map and sensor data, with no WSL setup.

To send a command from a second PowerShell (this one, api_id 1016, is a sport-mode command):

```powershell
docker compose -f docker-compose.windows.yml exec unitree_ros bash
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

### Ethernet (CycloneDDS) instead of Wi-Fi

`CONN_TYPE=cyclonedds` uses DDS discovery over a direct Ethernet cable and needs no robot address or AES key, but your Ethernet adapter must be on the robot's subnet, and discovery through Docker Desktop on Windows is untested.
Both compose files default to `webrtc`, so set `CONN_TYPE=cyclonedds` explicitly for Ethernet.

## Path B: inside WSL2

Use this path when you need a window (RViz, the C++ map viewer, the Julia viewer).

1. Install Ubuntu 22.04 once, in PowerShell: `wsl --install -d Ubuntu-22.04` (`wsl -l -v` shows it afterwards).
2. Turn on Docker Desktop > Settings > Resources > WSL Integration > the toggle for `Ubuntu-22.04` > Apply & Restart. This is the step people miss: without it, `docker` is not reachable inside the distro even though it works from PowerShell.
3. Open the distro (`wsl -d Ubuntu-22.04`) and check `docker info` gives real output there, not a connection error.
4. `cd` to your checkout's `docker/` folder (Windows drives appear under `/mnt/c`, `/mnt/d` and so on), then:
   ```shell
   docker compose build                  # first time only; slow (ROS deps, torch, open3d)
   docker compose up -d
   docker compose logs -f unitree_ros    # Ctrl+C stops following; the container keeps running
   ```
   Success: `Robot 0 validated and ready`. Shell in: `docker compose exec unitree_ros bash`. Stop: `docker compose down`.
5. Check a window can appear: `docker compose exec unitree_ros bash -ic "ros2 run go2_map_viewer_cpp map_viewer_node"`. A "Go2 Map Viewer" window should open (look in the taskbar; WSLg windows can open behind others).

Run every Path B command inside the WSL shell, not PowerShell: that is what makes the display socket (`/tmp/.X11-unix`) point at WSLg's. All the tools and their commands are on the [Command cheat sheet](commands.md).

### Native alternative: WSL2 without Docker

A WSL2 Ubuntu 22.04 distro with ROS 2 Humble installed natively and the workspace built at `~/ros2_ws` can run the stack without Docker. Stop the Docker container first, then:

```shell
wsl -d Ubuntu-22.04
export ROBOT_IP=<ask a teammate> CONN_TYPE=webrtc
ros2 launch go2_robot_sdk robot.launch.py
```

Only one of the Docker container or the native launch can hold the robot's connection at a time. The team's normal path is Docker; see [Lab setup](lab-setup.md).

## Connects to

- [Docker](docker.md): the compose files and edit loop.
- [Command cheat sheet](commands.md): every tool's run command, including RViz from PowerShell.
- [RViz panels](rviz-panels.md) and [Map viewers](map-viewers.md): the windows Path B can show.
- [Lab checklist](lab-checklist.md): connecting to Oak.

## How it breaks

| Symptom | Fix |
| --- | --- |
| `docker` not found, or `docker info` hangs inside WSL | WSL Integration toggle is off, or Docker Desktop needs a restart after turning it on |
| Window never appears, no error | In the container, `echo $DISPLAY` should not be empty. If it is, restart the WSL shell (WSLg sets it per session) and retry |
| `cannot open display` or X11 errors | `~/.Xauthority` does not exist yet: open any GUI app in WSL once (for example `xeyes`), then retry |
| `rviz2 ... could not connect to display` on Path A | Expected: Path A has no display. Use Foxglove or Path B |
| `RSA key format is not supported` | Firmware 1.1.15+ needs the robot's AES key (32 hex characters) as `ROBOT_AES_KEY`. Fetching it needs the Unitree account the robot is registered to; ask a teammate |
| Handshake or `con_notify` times out | Your PC is not on the robot's network, or the Unitree app is connected. On PowerShell, test with `Test-NetConnection <robot address> -Port 9991` |
| `docker` commands return 500 or hang | Check free space on C:; Docker's WSL disk lives there by default. One teammate moved it to another drive after C: filled up |
| Build crashes WSL | Out of memory: see [Docker](docker.md), or raise `memory=` in `.wslconfig` |
| `ElevenLabs API key not provided` | The text-to-speech node is not configured. Ignore it unless you need speech ([Voice](voice.md)) |
| `H264 ... failed to decode` warnings | Occasional dropped video packets. Harmless |
| `lidar_to_pointcloud` exits cleanly | Lidar is still published on `/point_cloud2` by the driver |
| RViz's built-in Map display crashes | Known with WSL's software graphics; it is off in our layout. Use the MapViewer panel |
| Closing RViz prints `context is invalid` and exit 139 | A crash during shutdown on Humble. Ignore it |

## Who's on it and what's next

TODO(owner): no owner.

<!-- sources: docs/RUNNING_IN_WSL.md and docs/WINDOWS_SETUP.md (folded in here and deleted 2026-10-22; one teammate's machine-specific G: drive paths and the lab network name and robot IP deliberately left out); docs/RUNNING_TOOLS.md; docs/USEFUL_COMMANDS.md (rviz from PowerShell, exit 139); docker/docker-compose.yml and docker-compose.windows.yml; docs/FUSION.md row 7; docs/TEST_LOG.md open issue 5 -->
