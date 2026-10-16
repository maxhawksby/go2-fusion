# Useful commands

Quick cheat-sheet of one-off commands used while developing against the
running Docker stack. See [RUNNING_TOOLS.md](RUNNING_TOOLS.md) and
[WINDOWS_SETUP.md](WINDOWS_SETUP.md) for full setup context.

For the Oak Control website, ROS bridge, object detector, and camera selector,
follow [DASHBOARD_LAB_GUIDE.md](DASHBOARD_LAB_GUIDE.md).

All of these assume the stack is already up (`docker compose up -d` from
`docker/`, run inside a WSL2 shell if you need a GUI window), and that you run
them from the repo's `docker/` folder. On Windows hosts, add
`-f docker-compose.windows.yml` after `docker compose`.

## rviz

rviz needs a display. The container gets one from WSLg, which ships with
WSL2 on Windows 11.

### From PowerShell (Windows)

`docker-compose.windows.yml` has no display by default, so pass WSLg's display
into the container. From the repo's `docker/` folder:
```powershell
docker compose -f docker-compose.windows.yml run --rm -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg unitree_ros rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz
```
This opens rviz on its own, with no robot connection: the panels stay empty
until something publishes. The container builds the workspace first, and
closing the window removes it. The `/run/desktop/mnt/host/wslg` paths are
inside Docker Desktop's VM, so they work as written from PowerShell. A
`QStandardPaths: runtime directory ... is not owned by UID 0` warning is
harmless.

### From a WSL2 shell

The default `docker-compose.yml` already mounts the display. One-time WSL
setup is in [RUNNING_IN_WSL.md](RUNNING_IN_WSL.md).

```shell
wsl -d Ubuntu-22.04
cd "/mnt/g/broswer download/go2-fusion/docker"   # adjust to your checkout
```

rviz with the robot stack: `robot.launch.py` starts rviz along with the
driver, SLAM and Nav2. This connects to the robot at `ROBOT_IP`:
```shell
docker compose up -d
```

rviz on its own, with no robot connection (the panels stay empty until
something publishes). The container builds the workspace first, and closing
the window removes it:
```shell
docker compose run --rm unitree_ros rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz
```

Open another rviz window in a stack that's already running:
```shell
docker compose exec unitree_ros bash -ic 'rviz2 -d /ros2_ws/src/go2_robot_sdk/config/single_robot_conf.rviz'
```

Closing rviz on Humble often ends with `rcl node's context is invalid` and
exit code 139. That's a crash during shutdown and can be ignored.

Restart the whole stack (driver, SLAM, Nav2, rviz, etc.):
```shell
cd docker    # from the repo root
docker compose restart unitree_ros
```

Check it's actually running without restarting anything:
```shell
docker compose exec unitree_ros bash -c 'ps aux | grep rviz2 | grep -v grep'
```

## Map viewer (C++)

Click the map to send a Nav2 goal, `C`/`Esc` to cancel:
```shell
docker compose exec unitree_ros bash -ic 'ros2 run go2_map_viewer_cpp map_viewer_node'
```

## Map viewer (Julia)

Read-only map heatmap with an approximate robot marker. Needs
`rosbridge_server` running first (not started automatically):
```shell
docker compose exec unitree_ros bash -ic 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
```
Then, in another terminal:
```shell
docker compose exec unitree_ros bash -ic 'julia --threads=auto --project=/opt/go2_map_viewer_julia /ros2_ws/src/go2_map_viewer_julia/map_viewer.jl'
```

## Keyboard controller

```shell
docker compose exec unitree_ros bash -ic 'ros2 run keyboard_teleop keyboard_teleop'
```
`W`/`S` forward-back, `A`/`D` turn, `Q`/`E` strafe, `Z`/`X` speed down/up,
`Space` stop, `Esc` quit.

## Autonomous mapping

Drives the robot on its own via Nav2 while building the SLAM map. Only run
it when you're physically present and watching the robot. Needs `/map` to
already exist (drive a little with teleop first if SLAM hasn't published
one yet):
```shell
docker compose exec unitree_ros bash -ic 'ros2 run go2_auto_mapper explorer'
```

## Rebuilding after a source change

Python edits (`keyboard_teleop`, `go2_auto_mapper`) are supposed to need no
rebuild under `--symlink-install`, but if the running container's install
was built before a package existed or before `--symlink-install` actually
took effect for it, the installed copy can be a stale, disconnected file.
If a fix doesn't show up at runtime, rebuild just that package:
```shell
docker compose exec unitree_ros bash -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   cd /ros2_ws && colcon build --symlink-install --packages-select <pkg_name>'
```
C++ packages (`go2_map_viewer_cpp`) always need this after a source change.

Rebuilding the whole image from scratch (e.g. to clear out stale baked
content):
```shell
cd docker
docker compose build --no-cache
```
This can take 30-60+ minutes and has failed on flaky `apt`/DNS mirror
access more than once. Don't trust a logged "exit 0" after a `tee`/pipe —
confirm the image actually changed instead:
```shell
docker image inspect docker-unitree_ros:latest --format '{{.Created}}'
```
