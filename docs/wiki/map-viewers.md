---
title: Map viewers
parent: maps-nav
code: [src/go2_map_viewer_cpp/**]
---

# Map viewers

The map viewer is a small window that shows Oak's live map and where Oak is on it, written as a learning project; click the map to send Oak somewhere.

Part of: [Maps and navigation](maps-nav.md)

## How it works

The viewer draws the [SLAM](slam.md) map from `/map` as a grid: white squares are free floor, black are walls, gray are unknown. It draws Oak as a red dot, found by asking TF where `base_link` is on `map` (see [TF and robot model](tf.md)). It does not show the camera or the point cloud; RViz and Foxglove do that.

**C++ viewer (`go2_map_viewer_cpp`).** A ROS 2 node that opens a window with SFML, a simple graphics library. Left-click a square and it converts that pixel back into map coordinates and sends a goal to [Nav2](nav2.md) on `/navigate_to_pose`, the same thing RViz's "Nav2 Goal" tool does. Press `C` or `Esc` to cancel the goal. It subscribes to `/map` as "latched" (`transient_local`, `reliable`), so it still gets a map that was published before it started. It is a learning and demo project, not a replacement for RViz or Foxglove. Details:

- The window is titled "Go2 Map Viewer". The map is scaled to fit it, including maps larger than the window; resizing scales the map, and clicks use the same scale, also on high-DPI screens. Clicking the dark border or the hint text does nothing; closing the window quits.
- ROS runs on a background thread while the drawing loop runs on the main thread at up to 30 frames per second.
- SFML comes from the `sfml-dev` rosdep key in `package.xml`, so the Docker image (and `rosdep install --from-paths src --ignore-src -y`) installs it. Never `apt-get install` it by hand.
- Unlike the Python packages, it is C++, so every source change needs a rebuild.

**The Julia viewer is retired** (2026-10-29). It was a read-only dashboard that needed Julia and `rosbridge_server` in the image; the team dropped it to keep the image smaller and simpler. Its code is kept in `unused_files/go2_map_viewer_julia/`, outside the Docker image.

## Connects to

- [SLAM](slam.md) - `/map`, and the `map` frame.
- [TF and robot model](tf.md) - `map` -> `odom` -> `base_link` for the red dot.
- [Nav2](nav2.md) - the viewer sends goals to `/navigate_to_pose`.
- [RViz panels](rviz-panels.md) - a map viewer panel inside RViz does a similar job. RViz's own Map display crashes under WSL's software graphics, so it is off in the team's RViz setup ([Decisions](decisions.md), row 7).
- [WSL and Windows](wsl.md) and [Docker](docker.md) - where the window can open.

## How it breaks

- **No window on Windows Docker.** `docker-compose.windows.yml` has no display, so the window cannot open there; use Foxglove (see [WSL and Windows](wsl.md)). The Linux compose file and WSLg work.
- **"Waiting for map..."** in the C++ viewer means SLAM is not publishing `/map` yet.
- **Click-to-walk not yet tested** on the robot ([Test log](test-log.md), "Not run yet" and open issue 1).

## Who's on it and what's next

TODO(owner): who maintains the viewer. It came over from the team fork in the fusion; the next step on record is retesting click-to-walk on Oak ([Test log](test-log.md) open issue 1).

## The code

https://github.com/maxhawksby/go2-fusion/tree/master/src/go2_map_viewer_cpp

With the stack running, on the Linux compose file or WSLg:

```shell
cd docker
docker compose exec unitree_ros bash -ic 'ros2 run go2_map_viewer_cpp map_viewer_node'

```

After a C++ change, rebuild with `docker compose exec unitree_ros bash -ic 'cbs go2_map_viewer_cpp'`.

<!-- sources: src/go2_map_viewer_cpp/README.md, src/go2_map_viewer_cpp/src/map_viewer_node.cpp, docs/USEFUL_COMMANDS.md, docs/FUSION.md row 7, docs/TEST_LOG.md -->
