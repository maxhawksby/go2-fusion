---
title: Command cheat sheet
parent: lab-setup
---

# Command cheat sheet

This page is the copy-paste list of one-off commands for the running Docker stack: RViz, the map viewers, keyboard driving, auto mapping and rebuilds.

Part of: [Lab setup](lab-setup.md)

## Before you start

All of these assume the stack is already up (`docker compose up -d` from `docker/`) and that you run them from the repo's `docker/` folder.
On Windows PowerShell, add `-f docker-compose.windows.yml` after `docker compose`. Anything that opens a window needs a display: run it inside a WSL2 shell with the default compose file, or use the PowerShell display flags below (see [WSL and Windows](wsl.md)). On a Mac, use `bash docker/mac/go2.sh` instead (see [Mac setup](mac-setup.md)).

## RViz

RViz needs a display. On Windows the container gets one from WSLg, which ships with WSL2 on Windows 11.

**From PowerShell (Windows).** `docker-compose.windows.yml` has no display by default, so pass WSLg's display into the container:

```powershell
docker compose -f docker-compose.windows.yml run --rm -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg unitree_ros rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz
```

This opens RViz on its own, with no robot connection: the panels stay empty until something publishes. The container builds the workspace first, and closing the window removes it. The `/run/desktop/mnt/host/wslg` paths are inside Docker Desktop's VM, so they work as written from PowerShell. A `QStandardPaths: runtime directory ... is not owned by UID 0` warning is harmless.

**From a WSL2 shell.** The default `docker-compose.yml` already mounts the display (one-time setup on [WSL and Windows](wsl.md)). `wsl -d Ubuntu-22.04`, then `cd` to your checkout's `docker/` folder.

```shell
docker compose up -d     # RViz with the robot stack: robot.launch.py starts RViz, the driver, SLAM and Nav2, and connects to ROBOT_IP
docker compose run --rm unitree_ros rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz            # RViz alone, no robot; closing it removes the container
docker compose exec unitree_ros bash -ic 'rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz'    # another RViz window in a running stack
docker compose restart unitree_ros                                                                                # restart the whole stack (driver, SLAM, Nav2, RViz)
docker compose exec unitree_ros bash -c 'ps aux | grep rviz2 | grep -v grep'                                      # is RViz running? restarts nothing
```

Closing RViz on Humble often ends with `rcl node's context is invalid` and exit code 139. That is a crash during shutdown and can be ignored.

## Map viewers

See [Map viewers](map-viewers.md). Both open a window.

**C++ viewer:** click the map to send a Nav2 goal, `C` or `Esc` to cancel.

```shell
docker compose exec unitree_ros bash -ic 'ros2 run go2_map_viewer_cpp map_viewer_node'
```

**Julia viewer:** a read-only map heatmap with an approximate robot marker. Start `rosbridge_server` first (it is not started automatically), then the viewer in another terminal:

```shell
docker compose exec unitree_ros bash -ic 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
docker compose exec unitree_ros bash -ic 'julia --threads=auto --project=/opt/go2_map_viewer_julia /ros2_ws/src/go2_map_viewer_julia/map_viewer.jl'
```

The Julia viewer has an odom-versus-map frame caveat; see [Map viewers](map-viewers.md).

No window available? Foxglove shows the same map and sensor data with no WSL setup, but cannot click-to-navigate (see [WSL and Windows](wsl.md)).

## Keyboard controller

Needs no display, so it works through `docker-compose.windows.yml` as-is. See [Keyboard teleop](keyboard-teleop.md).

```shell
docker compose exec unitree_ros bash -ic 'ros2 run keyboard_teleop keyboard_teleop'
```

`W`/`S` forward and back, `A`/`D` turn, `Q`/`E` strafe, `Z`/`X` speed down and up, `Space` stop, `Esc` quit.

## Autonomous mapping

Drives Oak on its own through Nav2 while building the SLAM map. **Only run it when you are physically there, watching the robot.** It needs `/map` to exist already: drive a little with teleop first if SLAM has not published one yet. See [Auto mapper](auto-mapper.md).

```shell
docker compose exec unitree_ros bash -ic 'ros2 run go2_auto_mapper explorer'
```

## Rebuilding after a source change

Python edits are supposed to need no rebuild under `--symlink-install`. But if the running container's install was built before a package existed, or before `--symlink-install` took effect for it, the installed copy can be a stale, disconnected file. If a fix does not show up at runtime, rebuild just that package:

```shell
docker compose exec unitree_ros bash -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   cd /ros2_ws && colcon build --symlink-install --packages-select <pkg_name>'
```

(`cbs <pkg_name>` in a container shell does the same.) C++ packages such as `go2_map_viewer_cpp` always need this after a source change.

Rebuilding the whole image from scratch, for example to clear out stale baked content:

```shell
docker compose build --no-cache
```

This can take 30-60+ minutes and has failed on flaky `apt` or DNS mirror access more than once. Do not trust a logged "exit 0" after a `tee` or pipe; confirm the image actually changed:

```shell
docker image inspect docker-unitree_ros:latest --format '{{.Created}}'
```

## Connects to

- [Docker](docker.md): the edit loop and everyday compose commands.
- [WSL and Windows](wsl.md) and [Mac setup](mac-setup.md): getting a display.

<!-- sources: docs/USEFUL_COMMANDS.md and docs/RUNNING_TOOLS.md (folded in here and deleted 2026-10-22; one teammate's machine-specific G: drive paths left out); docker/dev_aliases.sh (cbs) -->
