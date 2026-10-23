---
title: Map viewers
parent: maps-nav
code: [src/go2_map_viewer_cpp/**, go2_map_viewer_julia/**]
---

# Map viewers

The map viewers are two small windows that show Oak's live map and where Oak is on it, written as learning projects; the C++ one also lets you click the map to send Oak somewhere.

Part of: [Maps and navigation](maps-nav.md)

## How it works

Both viewers draw the [SLAM](slam.md) map from `/map` as a grid: white squares are free floor, black are walls, gray are unknown. Both draw Oak as a red dot, found by asking TF where `base_link` is on `map` (see [TF and robot model](tf.md)). Neither shows the camera or the point cloud; RViz and Foxglove do that.

**C++ viewer (`go2_map_viewer_cpp`).** A ROS 2 node that opens a window with SFML, a simple graphics library. Left-click a square and it converts that pixel back into map coordinates and sends a goal to [Nav2](nav2.md) on `/navigate_to_pose`, the same thing RViz's "Nav2 Goal" tool does. Press `C` or `Esc` to cancel the goal. It subscribes to `/map` as "latched" (`transient_local`, `reliable`), so it still gets a map that was published before it started. It is a learning and demo project, not a replacement for RViz or Foxglove. Details:

- The window is titled "Go2 Map Viewer". The map is scaled to fit it, including maps larger than the window; resizing scales the map, and clicks use the same scale, also on high-DPI screens. Clicking the dark border or the hint text does nothing; closing the window quits.
- ROS runs on a background thread while the drawing loop runs on the main thread at up to 30 frames per second.
- SFML comes from the `sfml-dev` rosdep key in `package.xml`, so the Docker image (and `rosdep install --from-paths src --ignore-src -y`) installs it. Never `apt-get install` it by hand.
- Unlike the Python packages, it is C++, so every source change needs a rebuild.

**Julia viewer (`go2_map_viewer_julia`).** A read-only dashboard written in Julia (a language for fast number crunching) with the GLMakie plotting library. It cannot send goals. Julia has no full ROS 2 library here, so it talks to ROS through `rosbridge_server`, which turns ROS messages into JSON over a WebSocket on port 9090. If the TF chain is more than 5 seconds old (say SLAM died but odometry still runs), the status line says `STALE`, so don't trust the dot. It lives at the repo root, not in `src/`, because it is not a colcon package. More details:

- It connects to `ws://localhost:9090` and retries on its own until rosbridge answers, and again if rosbridge drops. The status line shows the connection, the map size, or the last rosbridge error.
- The dot comes from walking TF from `base_link` up to `map` (usually `map` -> `odom` -> `base_link`). It does not use `/odom` alone, because that pose is in the `odom` frame and drifts relative to the map. The `STALE` warning reads `(STALE: tf chain N.Ns old)`; the dot is still drawn.
- Its Julia libraries are baked into the image at `/opt/go2_map_viewer_julia`, while the script itself is the live-mounted `map_viewer.jl`. With more than one thread (`--threads=2`, or `JULIA_NUM_THREADS=2`), a large `/map` message no longer stalls the window.
- Adding a Julia package: `cd go2_map_viewer_julia`, `julia --project=. -e 'import Pkg; Pkg.add("SomePackage")'`, commit the updated `Project.toml` and `Manifest.toml`, then `docker compose build`.

## Connects to

- [SLAM](slam.md) - `/map`, and the `map` frame.
- [TF and robot model](tf.md) - `map` -> `odom` -> `base_link` for the red dot.
- [Nav2](nav2.md) - the C++ viewer sends goals to `/navigate_to_pose`.
- [RViz panels](rviz-panels.md) - a map viewer panel inside RViz does a similar job. RViz's own Map display crashes under WSL's software graphics, so it is off in the team's RViz setup ([Decisions](decisions.md), row 7).
- [WSL and Windows](wsl.md) and [Docker](docker.md) - where the windows can open.

## How it breaks

- **No window on Windows Docker.** `docker-compose.windows.yml` has no display, so neither window can open there; use Foxglove (see [WSL and Windows](wsl.md)). The Linux compose file and WSLg work.
- **"Waiting for map..."** in the C++ viewer means SLAM is not publishing `/map` yet.
- **Julia looks frozen on first start.** GLMakie compiles for several minutes the first time.
- **Julia "retrying".** rosbridge is not started automatically; start it first.
- **A big map can make the Julia window stall** briefly with one thread; start Julia with `--threads=auto`.
- **Click-to-walk not yet tested** on the robot ([Test log](test-log.md), "Not run yet" and open issue 1).

## Who's on it and what's next

TODO(owner): who maintains the viewers. Both came over from the team fork in the fusion; the next step on record is retesting click-to-walk on Oak ([Test log](test-log.md) open issue 1).

## The code

- C++: https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_map_viewer_cpp
- Julia: https://github.com/maxhawksby/go2-fusion/tree/master/go2_map_viewer_julia

With the stack running, on the Linux compose file or WSLg:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run go2_map_viewer_cpp map_viewer_node'

# Julia: rosbridge first, then the viewer in a second terminal
docker compose exec unitree_ros bash -ic 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
docker compose exec unitree_ros bash -ic 'julia --threads=auto --project=/opt/go2_map_viewer_julia /ros2_ws/src/go2_map_viewer_julia/map_viewer.jl'
```

After a C++ change, rebuild with `docker compose exec unitree_ros bash -ic 'cbs go2_map_viewer_cpp'`. Editing `map_viewer.jl` needs no rebuild; adding a Julia package needs `docker compose build`.

<!-- sources: src/go2_map_viewer_cpp/README.md, src/go2_map_viewer_cpp/src/map_viewer_node.cpp, go2_map_viewer_julia/README.md, go2_map_viewer_julia/map_viewer.jl, docker/docker-compose.yml (Julia mount), docs/USEFUL_COMMANDS.md, docs/FUSION.md row 7, docs/TEST_LOG.md -->
